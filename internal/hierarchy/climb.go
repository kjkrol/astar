package hierarchy

import (
	"math"
	"slices"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/astar"
	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/dijkstra"
	"github.com/kjkrol/hpastar/internal/heap"
)

// Climb asks paths of a hierarchy on one goroutine: its own scratch and counters, and the
// graph's transitions as its caller reads them.
type Climb[T comparable] struct {
	h *Hierarchy[T]
	// the caller's transitions forward, and both ways counting what they expand
	given             graph.Transitions[T]
	forward, backward graph.Transitions[T]
	q                 query[T]
	// a leg's cluster read where the cluster keeps no trees, and the fill finding it
	read       reader[T]
	leg        localFill
	locals     []int32
	expansions uint64
}

// query is what a route or a reaches works with, kept across them
type query[T comparable] struct {
	from, to T
	level    int // the level searched
	// by level: the clusters of from and to, the costs from from to its cluster's nodes and from
	// to's cluster's nodes to to
	startC, goalC       []int32
	startCost, goalCost [][]float64
	// the fills inserting from and to: on level 1 over the graph, above over the level below
	start, goal   fill1[T]
	starts, goals []localFill
	searchers     []searcher // by level
	search        graph.Transitions[nodeRef]
	heuristic     graph.Heuristic[nodeRef]
	path          []nodeRef // the route found, on the level searched
	sources       []dijkstra.Source[T]
	targets       []T
	roots         []int32
	// the route written, and how much of it
	dst   []T
	wrote int
	// the walks along legs: of level 1 by local index, its states; by level, the nodes a level
	// down
	locals                       []int32
	states                       []T
	walks, startWalks, goalWalks [][]nodeRef
	buf                          []T
}

// a search over the nodes of one level, the start and the goal past them, with each node's
// estimate kept once made
type searcher struct {
	s        *astar.Engine[nodeRef]
	at       *placeTop
	dict     *dict.Dense[nodeRef]
	estimate []float64
	made     []uint32 // by node: the search its estimate was made for
	search   uint32
}

// Verdict is why a route was found or not.
type Verdict uint8

const (
	Climbed   Verdict = iota
	NotReady          // a cluster is stale
	NoCluster         // an end lies nowhere in the plane
	TooNear           // the ends share a cluster or lie nearer than the Config's Near asks
	NoPath            // no path joins the ends
)

// NewClimb is a climb of h over forward and backward, which step and price as h's graph does.
func (h *Hierarchy[T]) NewClimb(forward, backward graph.Transitions[T]) *Climb[T] {
	s := &Climb[T]{h: h, given: forward}
	s.forward = func(from, prev T, buf []graph.Transition[T]) []graph.Transition[T] {
		s.expansions++
		return forward(from, prev, buf)
	}
	s.backward = func(to, next T, buf []graph.Transition[T]) []graph.Transition[T] {
		s.expansions++
		return backward(to, next, buf)
	}
	n, q := len(h.levels), &s.q
	q.start, q.goal = h.newFill1(), h.newFill1()
	q.starts, q.goals = make([]localFill, n), make([]localFill, n)
	q.startC, q.goalC = make([]int32, n), make([]int32, n)
	q.startCost, q.goalCost = make([][]float64, n), make([][]float64, n)
	q.walks, q.startWalks, q.goalWalks = make([][]nodeRef, n), make([][]nodeRef, n),
		make([][]nodeRef, n)
	q.searchers = make([]searcher, n)
	q.search, q.heuristic = s.searchFrom, s.estimate
	s.leg.heap = heap.New(h.config.Side * h.config.Side)
	return s
}

// Expanded is how many states and nodes the climb has expanded.
func (s *Climb[T]) Expanded() uint64 { return s.expansions }

// Patience is how many states an exact search expands before it asks the labels whether a path
// joins its ends at all: a few clusters' worth.
func (s *Climb[T]) Patience() int { return 4 * s.h.config.Side * s.h.config.Side }

// levelFor is the level, from 0, the route from from to to is searched on, its ends' clusters
// kept by level: the highest where they lie in different clusters and as far apart, by the
// heuristic, as the Config's Near asks of it; anything but Climbed says why the route is refused
func (s *Climb[T]) levelFor(from, to T) (int, Verdict) {
	h, q := s.h, &s.q
	if !h.Ready() {
		return 0, NotReady
	}
	sc, ok := h.cluster1(from)
	if !ok {
		return 0, NoCluster
	}
	gc, ok := h.cluster1(to)
	if !ok {
		return 0, NoCluster
	}
	if sc == gc {
		return 0, TooNear
	}
	far, near := h.graph.Heuristic(from, to), h.config.Near
	if far < near {
		return 0, TooNear
	}
	q.startC[0], q.goalC[0] = sc, gc
	top := 0
	for j := 0; j+1 < len(h.levels); j++ {
		p := h.levels[j].parent
		q.startC[j+1], q.goalC[j+1] = p[q.startC[j]], p[q.goalC[j]]
		near *= float64(h.config.Factor)
		if top == j && q.startC[j+1] != q.goalC[j+1] && far >= near {
			top = j + 1
		}
	}
	return top, Climbed
}

// Route searches the abstract route from from to to on the highest level its ends' distance
// allows, after inserting its ends on every level up to it; anything but Climbed says why it
// found none.
func (s *Climb[T]) Route(from, to T) Verdict {
	top, v := s.levelFor(from, to)
	if v != Climbed {
		return v
	}
	q := &s.q
	q.from, q.to = from, to
	s.insert1(&q.start, q.startC[0], from, s.forward, &q.startCost[0])
	s.insert1(&q.goal, q.goalC[0], to, s.backward, &q.goalCost[0])
	if !s.joined(q.startC[0], q.goalC[0]) {
		return NoPath
	}
	for j := 1; j <= top; j++ {
		s.insertK(&q.starts[j], j, q.startC, q.startCost, false)
		s.insertK(&q.goals[j], j, q.goalC, q.goalCost, true)
	}
	if !s.search(top) {
		return NoPath
	}
	return Climbed
}

// insert1 fills cluster c of level 1 from st over steps to its nodes, their costs into costs
func (s *Climb[T]) insert1(f *fill1[T], c int32, st T, steps graph.Transitions[T],
	costs *[]float64) {
	q := &s.q
	cl := &s.h.levels[0].clusters[c]
	f.at.set(s.h.block1(c))
	q.sources = append(q.sources[:0], dijkstra.Source[T]{At: st})
	f.d.Fill(q.sources, steps, f.in, cl.states)
	*costs = grown(*costs, len(cl.states))
	for i, n := range cl.states {
		(*costs)[i] = math.Inf(1)
		if cost, ok := f.d.Cost(n); ok {
			(*costs)[i] = cost
		}
	}
}

// insertK fills the cluster cs[j] of level j over the graph of the level below from the nodes of
// cs[j-1], at their costs, to its nodes, their costs into costs[j]; back fills toward them
func (s *Climb[T]) insertK(f *localFill, j int, cs []int32, costs [][]float64, back bool) {
	h, g := s.h, &s.read.graph
	dn, cl := &h.levels[j-1], &h.levels[j].clusters[cs[j]]
	h.readK(g, j, cs[j], back)
	s.locals = grown(s.locals, len(cl.down))
	for i, r := range cl.down {
		s.locals[i] = cl.offset[dn.slot[r.c]] + r.i
	}
	f.start(g, s.locals)
	below := cl.offset[dn.slot[cs[j-1]]]
	for i, cost := range costs[j-1] {
		if !math.IsInf(cost, 1) {
			f.source(below+int32(i), cost)
		}
	}
	f.settle(g)
	s.expansions += uint64(len(f.order))
	costs[j] = grown(costs[j], len(cl.down))
	for i, l := range s.locals {
		costs[j][i] = f.cost(l)
	}
}

// search runs A* over level j's graph from the start to the goal inserted
func (s *Climb[T]) search(j int) bool {
	q, lv := &s.q, &s.h.levels[j]
	sr := &q.searchers[j]
	if sr.s == nil {
		sr.at = &placeTop{}
		sr.dict = dict.NewDense(sr.at.home)
		sr.s = astar.New[nodeRef](q.heuristic, sr.dict, 1024)
	}
	sr.at.base, sr.at.total = lv.base, lv.base[len(lv.clusters)]
	n := int(sr.at.total) + 2
	sr.dict.Fit(n)
	if len(sr.made) < n {
		sr.estimate, sr.made, sr.search = make([]float64, n), make([]uint32, n), 0
	}
	if sr.search++; sr.search == 0 {
		clear(sr.made)
		sr.search = 1
	}
	q.level = j
	q.path = sr.s.SolveInto(q.path[:0], startRef, goalRef, q.search)
	return len(q.path) > 0
}

// searchFrom appends the steps of the level searched from from: the start's to its cluster's
// nodes, the goal's cluster's nodes' to the goal, then the level's own
func (s *Climb[T]) searchFrom(from, _ nodeRef,
	buf []graph.Transition[nodeRef]) []graph.Transition[nodeRef] {
	q := &s.q
	s.expansions++
	j := q.level
	switch {
	case from == startRef:
		for i, cost := range q.startCost[j] {
			if !math.IsInf(cost, 1) {
				buf = append(buf, graph.Transition[nodeRef]{To: nodeRef{q.startC[j], int32(i)},
					Cost: cost})
			}
		}
		return buf
	case from == goalRef:
		return buf
	case from.c == q.goalC[j]:
		if cost := q.goalCost[j][from.i]; !math.IsInf(cost, 1) {
			buf = append(buf, graph.Transition[nodeRef]{To: goalRef, Cost: cost})
		}
	}
	return s.h.edgesOut(j, from, buf)
}

func (s *Climb[T]) estimate(a, _ nodeRef) float64 {
	q := &s.q
	sr := &q.searchers[q.level]
	i := sr.at.home(a)
	if sr.made[i] == sr.search {
		return sr.estimate[i]
	}
	e := 0.0
	switch a {
	case goalRef:
	case startRef:
		e = s.h.graph.Heuristic(q.from, q.to)
	default:
		e = s.h.graph.Heuristic(s.h.levels[q.level].clusters[a.c].states[a.i], q.to)
	}
	sr.estimate[i], sr.made[i] = e, sr.search
	return e
}

// Refine writes the route found into dst, refining its legs only as far as dst holds, and
// reports how many states it wrote.
func (s *Climb[T]) Refine(dst []T) int {
	q := &s.q
	if len(dst) == 0 {
		return 0
	}
	q.dst, q.wrote = dst, 1
	dst[0] = q.from
	j := q.level
	lv := &s.h.levels[j]
	more := q.wrote < len(dst)
	for t := 0; more && t+1 < len(q.path); t++ {
		u, v := q.path[t], q.path[t+1]
		switch {
		case u == startRef:
			more = s.startLeg(j, v.i)
		case v == goalRef:
			more = s.goalLeg(j, u.i)
		case u.c == v.c:
			more = s.intra(j, u.c, u.i, v.i)
		default:
			more = s.emit(lv.clusters[v.c].states[v.i])
		}
	}
	n := q.wrote
	q.dst = nil
	return n
}

// emit writes st next on the route; false once the route fills dst
func (s *Climb[T]) emit(st T) bool {
	q := &s.q
	q.dst[q.wrote] = st
	q.wrote++
	return q.wrote < len(q.dst)
}

// steps writes the route along the nodes w of level j, each step within a cluster refined
func (s *Climb[T]) steps(j int, w []nodeRef) bool {
	lv := &s.h.levels[j]
	for t := 0; t+1 < len(w); t++ {
		a, b := w[t], w[t+1]
		if a.c == b.c {
			if !s.intra(j, a.c, a.i, b.i) {
				return false
			}
		} else if !s.emit(lv.clusters[b.c].states[b.i]) {
			return false
		}
	}
	return true
}

// intra writes the route within cluster c of level j from its node u to its node v, u written
func (s *Climb[T]) intra(j int, c, u, v int32) bool {
	if j == 0 {
		return s.intra1(c, u, v)
	}
	h, q := s.h, &s.q
	cl := &h.levels[j].clusters[c]
	walk := q.walks[j][:0]
	lu, l := h.homeK(j, cl, cl.down[u]), h.homeK(j, cl, cl.down[v])
	if cl.span > 0 {
		for ; l != lu; l = cl.parent(u, l) {
			walk = append(walk, h.nodeAt(j, c, cl, l))
		}
	} else {
		// the leg found as the fill that would have grown its tree finds it
		h.readK(&s.read.graph, j, c, false)
		s.locals = append(s.locals[:0], l)
		s.leg.run(&s.read.graph, lu, s.locals)
		s.expansions += uint64(len(s.leg.order))
		for ; l != lu; l = s.leg.parent[l] {
			walk = append(walk, h.nodeAt(j, c, cl, l))
		}
	}
	walk = append(walk, cl.down[u])
	slices.Reverse(walk)
	q.walks[j] = walk
	return s.steps(j-1, walk)
}

// homeK is the local index in cluster cl of level j of node r a level down
func (h *Hierarchy[T]) homeK(j int, cl *cluster[T], r nodeRef) int32 {
	return cl.offset[h.levels[j-1].slot[r.c]] + r.i
}

// nodeAt is the node a level down of local index l in cluster c of level j
func (h *Hierarchy[T]) nodeAt(j int, c int32, cl *cluster[T], l int32) nodeRef {
	lv, dn, f := &h.levels[j], &h.levels[j-1], int32(h.config.Factor)
	s := int32(0)
	for cl.offset[s+1] <= l {
		s++
	}
	cx, cy := c%lv.cols*f+s%f, c/lv.cols*f+s/f
	return nodeRef{cy*dn.cols + cx, l - cl.offset[s]}
}

// intra1 writes the route within cluster c of level 1 from its node u to its node v, u
// written: along u's tree, or as the fill that would have grown the tree finds it
func (s *Climb[T]) intra1(c, u, v int32) bool {
	h, q := s.h, &s.q
	cl := &h.levels[0].clusters[c]
	lu, l := cl.local[u], cl.local[v]
	walk := q.locals[:0]
	if cl.span > 0 {
		for ; l != lu; l = cl.parent(u, l) {
			walk = append(walk, l)
		}
		q.locals = walk
		for i := len(walk) - 1; i >= 0; i-- {
			if !s.emit(s.stateAt(c, walk[i])) {
				return false
			}
		}
		return true
	}
	h.scan(&s.read, c)
	h.read1(&s.read, c, s.given, nil)
	s.locals = append(s.locals[:0], l)
	s.leg.run(&s.read.graph, lu, s.locals)
	s.expansions += uint64(len(s.leg.order))
	for ; l != lu; l = s.leg.parent[l] {
		walk = append(walk, l)
	}
	q.locals = walk
	for i := len(walk) - 1; i >= 0; i-- {
		if !s.emit(s.read.at[walk[i]]) {
			return false
		}
	}
	return true
}

// stateAt is the state of local index l in cluster c of level 1
func (s *Climb[T]) stateAt(c int32, l int32) T {
	q, side := &s.q, s.h.config.Side
	x0, y0, _, _ := s.h.block1(c)
	layer, p := int(l)/(side*side), int(l)%(side*side)
	q.buf = s.h.layout.At(x0+p%side, y0+p/side, q.buf[:0])
	return q.buf[layer]
}

// startLeg writes the route from from to node n of from's cluster of level j
func (s *Climb[T]) startLeg(j int, n int32) bool {
	h, q := s.h, &s.q
	cl := &h.levels[j].clusters[q.startC[j]]
	if j == 0 {
		walk := q.states[:0]
		for st := cl.states[n]; st != q.from; st, _ = q.start.d.Parent(st) {
			walk = append(walk, st)
		}
		q.states = walk
		for i := len(walk) - 1; i >= 0; i-- {
			if !s.emit(walk[i]) {
				return false
			}
		}
		return true
	}
	f, walk := &q.starts[j], q.startWalks[j][:0]
	l := h.homeK(j, cl, cl.down[n])
	for ; f.parent[l] != l; l = f.parent[l] {
		walk = append(walk, h.nodeAt(j, q.startC[j], cl, l))
	}
	walk = append(walk, h.nodeAt(j, q.startC[j], cl, l))
	slices.Reverse(walk)
	q.startWalks[j] = walk
	return s.startLeg(j-1, walk[0].i) && s.steps(j-1, walk)
}

// goalLeg writes the route from node n of to's cluster of level j to to, n written
func (s *Climb[T]) goalLeg(j int, n int32) bool {
	h, q := s.h, &s.q
	cl := &h.levels[j].clusters[q.goalC[j]]
	if j == 0 {
		for st := cl.states[n]; st != q.to; {
			st, _ = q.goal.d.Parent(st)
			if !s.emit(st) {
				return false
			}
		}
		return true
	}
	f, walk := &q.goals[j], q.goalWalks[j][:0]
	l := h.homeK(j, cl, cl.down[n])
	for ; f.parent[l] != l; l = f.parent[l] {
		walk = append(walk, h.nodeAt(j, q.goalC[j], cl, l))
	}
	walk = append(walk, h.nodeAt(j, q.goalC[j], cl, l))
	q.goalWalks[j] = walk
	return s.steps(j-1, walk) && s.goalLeg(j-1, walk[len(walk)-1].i)
}

// Reaches says whether to can be reached from from, off level 1: true where the nodes from
// reaches within its cluster and those reaching to within its cluster lie in one component of
// the level's graph, as its last build labelled them, and where the labels cannot tell.
func (s *Climb[T]) Reaches(from, to T) bool {
	h, q := s.h, &s.q
	if h.levels[0].stales > 0 || h.labels.pending() {
		return true
	}
	sc, ok := h.cluster1(from)
	if !ok {
		return true
	}
	gc, ok := h.cluster1(to)
	if !ok {
		return true
	}
	if sc == gc {
		q.start.at.set(h.block1(sc))
		q.sources = append(q.sources[:0], dijkstra.Source[T]{At: from})
		q.targets = append(q.targets[:0], to)
		if q.start.d.Fill(q.sources, s.forward, q.start.in, q.targets) == 1 {
			return true
		}
	}
	s.insert1(&q.start, sc, from, s.forward, &q.startCost[0])
	s.insert1(&q.goal, gc, to, s.backward, &q.goalCost[0])
	return s.joined(sc, gc)
}

// joined says whether a node of cluster sc of level 1 the start's insertion reached and one of
// cluster gc the goal's reached lie in one component, as the labels have it
func (s *Climb[T]) joined(sc, gc int32) bool {
	q, lv, alias := &s.q, &s.h.levels[0], s.h.labels.alias
	start, goal := lv.clusters[sc].label, lv.clusters[gc].label
	q.roots = q.roots[:0]
	for i, cost := range q.startCost[0] {
		if r := alias[start[i]]; !math.IsInf(cost, 1) && !slices.Contains(q.roots, r) {
			q.roots = append(q.roots, r)
		}
	}
	for i, cost := range q.goalCost[0] {
		if !math.IsInf(cost, 1) && slices.Contains(q.roots, alias[goal[i]]) {
			return true
		}
	}
	return false
}
