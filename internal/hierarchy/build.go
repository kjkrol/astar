package hierarchy

import (
	"cmp"
	"math"
	"math/bits"
	"slices"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/heap"
)

// a cluster of level 1 read into arrays: its states by local index, their keys, and its graph
type reader[T comparable] struct {
	members members
	graph   localGraph
	at      []T     // by local index: the state there
	count   []int32 // by place: how many states lie there
	layers  int
	buf     []T
	steps   []graph.Transition[T]
}

// what a build works with, kept across builds
type builder[T comparable] struct {
	reader[T]
	fill localFill
	// level 1: the steps leaving the cluster built and entering it, by the state's local index
	outs, ins, group []crossing[T]
	// above: the twins of its children leading out of it
	crossK, groupK []crossingK
	others         []int32
	cands          []candidate[T]
	arena          []int
	root, label    []int32
	runs, cursor   []int32
	order          []int32
	pairs          []pair[T]
	// by class: above, the local indices of its nodes of the cluster built; of several classes,
	// the graph it fills them over
	locals [][]int32
	graphs []localGraph
	// of several classes, listed once a job: the classes refusing each step of the arena, and by
	// a state's key where its steps lie there
	masks  []uint8
	listed members
	lists  [][2]int32
	// the transitions read off the graph since the job's step began
	read int
}

// a step of level 1 across the border of the cluster built, from or into its state of local
// index local
type crossing[T comparable] struct {
	local   int32
	other   T
	key     int
	c       int32 // other's cluster
	refuses uint8
	cost    float64
}

// a twin of a child of the cluster built leading out of it, from the child's node u to v
type crossingK struct {
	u, v       nodeRef
	uKey, vKey int
	up         int32 // v's cluster at the level built
	out, in    float32
}

// a state on a border with its best step across, a candidate entrance
type candidate[T comparable] struct {
	fromKey, toKey int
	x, y           int // from's coordinates
	along, across  int
	key            float64
	// level 1: where the keys of what from and to step to lie in the arena
	fromSteps, toSteps [2]int32
	// above: from's and to's clusters and classes a level down
	fromC, toC, fromClass, toClass int32
	twin                           pair[T]
	keep                           bool
}

// a twin chosen: the node of the cluster built and the state across
type pair[T comparable] struct {
	node     T
	key      int
	local    int32   // level 1
	down     nodeRef // above
	other    int32
	otherKey int
	out, in  float32
}

// members are the states of the cluster scanned, by key: their local index
type members struct {
	keys   []int
	locals []int32
	stamps []uint32
	stamp  uint32
	shift  uint
}

// reset empties the table for n states
func (m *members) reset(n int) {
	size := 16
	for size < 2*n {
		size *= 2
	}
	if len(m.keys) < size {
		m.keys, m.locals, m.stamps = make([]int, size), make([]int32, size), make([]uint32, size)
		m.stamp = 0
	}
	m.shift = uint(64 - bits.TrailingZeros(uint(len(m.keys))))
	if m.stamp++; m.stamp == 0 {
		clear(m.stamps)
		m.stamp = 1
	}
}

func (m *members) slot(key int) int { return int(uint64(key) * 0x9e3779b97f4a7c15 >> m.shift) }

func (m *members) put(key int, local int32) {
	mask := len(m.keys) - 1
	i := m.slot(key)
	for m.stamps[i] == m.stamp && m.keys[i] != key {
		i = (i + 1) & mask
	}
	m.keys[i], m.locals[i], m.stamps[i] = key, local, m.stamp
}

func (m *members) get(key int) (int32, bool) {
	mask := len(m.keys) - 1
	for i := m.slot(key); m.stamps[i] == m.stamp; i = (i + 1) & mask {
		if m.keys[i] == key {
			return m.locals[i], true
		}
	}
	return 0, false
}

// localGraph is a cluster's graph read into arrays, by local index: the steps within it
type localGraph struct {
	start []int32 // by local index and one more: where its steps start
	to    []int32
	cost  []float64
	mask  []uint8 // a build's of several classes: the classes refusing each step
}

// keep copies into dst the steps of g class bit takes
func keep(dst, g *localGraph, bit uint8) {
	n := len(g.start) - 1
	dst.start = grown(dst.start, n+1)
	dst.to, dst.cost = dst.to[:0], dst.cost[:0]
	for l := range n {
		dst.start[l] = int32(len(dst.to))
		for e := g.start[l]; e < g.start[l+1]; e++ {
			if g.mask[e]&bit == 0 {
				dst.to, dst.cost = append(dst.to, g.to[e]), append(dst.cost, g.cost[e])
			}
		}
	}
	dst.start[n] = int32(len(dst.to))
}

// localFill settles a localGraph outward from one local index in order of cost, by Dijkstra
type localFill struct {
	heap                  heap.Heap
	dist                  []float64
	parent                []int32
	seen, settled, target []uint32 // by local index: the fill that reached, settled or aims at it
	fill                  uint32
	left                  int     // the targets not settled yet
	order                 []int32 // settled, in order
}

// run settles g outward from src until every one of targets is settled
func (f *localFill) run(g *localGraph, src int32, targets []int32) {
	f.start(g, targets)
	f.source(src, 0)
	f.settle(g)
}

// start readies a fill of g aiming at targets
func (f *localFill) start(g *localGraph, targets []int32) {
	if n := len(g.start) - 1; len(f.dist) < n {
		f.dist, f.parent = make([]float64, n), make([]int32, n)
		f.seen, f.settled, f.target = make([]uint32, n), make([]uint32, n), make([]uint32, n)
		f.fill = 0
	}
	if f.fill++; f.fill == 0 {
		clear(f.seen)
		clear(f.settled)
		clear(f.target)
		f.fill = 1
	}
	f.heap.Reset()
	f.order, f.left = f.order[:0], 0
	for _, t := range targets {
		if f.target[t] != f.fill {
			f.target[t] = f.fill
			f.left++
		}
	}
}

// source has the fill start from l at cost
func (f *localFill) source(l int32, cost float64) {
	if f.seen[l] != f.fill {
		f.seen[l], f.dist[l], f.parent[l] = f.fill, cost, l
		f.heap.Push(l, cost)
	} else if cost < f.dist[l] {
		f.dist[l], f.parent[l] = cost, l
		f.heap.Fix(l, cost)
	}
}

// settle settles g outward from the sources until every target is settled
func (f *localFill) settle(g *localGraph) {
	fill := f.fill
	for f.left > 0 {
		u, ok := f.heap.Pop()
		if !ok {
			return
		}
		f.settled[u] = fill
		f.order = append(f.order, u)
		if f.target[u] == fill {
			if f.left--; f.left == 0 {
				return
			}
		}
		du := f.dist[u]
		for e := g.start[u]; e < g.start[u+1]; e++ {
			v := g.to[e]
			if f.settled[v] == fill {
				continue
			}
			if d := du + g.cost[e]; f.seen[v] != fill {
				f.seen[v], f.dist[v], f.parent[v] = fill, d, u
				f.heap.Push(v, d)
			} else if d < f.dist[v] {
				f.dist[v], f.parent[v] = d, u
				f.heap.Fix(v, d)
			}
		}
	}
}

// cost is what the last run settled l at, +Inf where it did not
func (f *localFill) cost(l int32) float64 {
	if f.settled[l] == f.fill {
		return f.dist[l]
	}
	return math.Inf(1)
}

// crossingKey is what crossing a border there and back costs, twice the way there where there is
// no way back
func crossingKey(there, back float64) float64 {
	if math.IsInf(back, 1) {
		return 2 * there
	}
	return there + back
}

// grown is s holding n, grown only where it holds fewer
func grown[E any](s []E, n int) []E {
	if cap(s) < n {
		return make([]E, n)
	}
	return s[:n]
}

// prepare1 readies cluster c of level 1 to be filled: its graph read once, the entrances of each
// class chosen on every border, each class's trees kept while its own fit the budget; it reports
// the work done
func (h *Hierarchy[T]) prepare1(c int32) int {
	b := &h.b
	x0, y0, _, _ := h.block1(c)
	work := h.scan(&b.reader, c)
	b.read = h.read1(&b.reader, c, h.graph.Forward, b)
	b.others = b.others[:0]
	for i := range b.outs {
		b.others = append(b.others, b.outs[i].c)
	}
	for i := range b.ins {
		b.others = append(b.others, b.ins[i].c)
	}
	slices.Sort(b.others)
	b.others = slices.Compact(b.others)
	if h.all != 0 {
		b.arena, b.masks, b.lists = b.arena[:0], b.masks[:0], b.lists[:0]
		b.listed.reset(2 * (len(b.outs) + len(b.ins)))
	}
	span := b.layers * h.config.Side * h.config.Side
	width := 1
	if span > 1<<8 {
		width = 2
	}
	for k := range h.views {
		cl := &h.views[k].levels[0].clusters[c]
		cl.layers = int32(b.layers)
		bit := h.bit(k)
		b.pairs = b.pairs[:0]
		for _, o := range b.others {
			h.candidatesOut1(o, x0, y0, bit)
			h.pick(0, bit)
			h.candidatesIn1(o, bit)
			h.pick(0, bit)
		}
		h.assemble(cl, 0)
		work += len(b.pairs)
		h.trees[k] -= cl.bytes
		cl.bytes = 0
		kept := 0
		if need := len(cl.keys) * span * width; span <= 1<<16 &&
			h.trees[k]+need <= h.config.Trees {
			cl.bytes, kept = need, span
			h.trees[k] += need
		}
		fillSetup(cl, kept)
		if h.all != 0 {
			keep(&b.graphs[k], &b.graph, bit)
			work += len(b.graph.to) / workChecks
		}
	}
	return work + workRead*b.read
}

// scan lists the states of cluster c of level 1 into r by their local index, and keeps their
// keys; it reports how many states it listed
func (h *Hierarchy[T]) scan(r *reader[T], c int32) int {
	side := h.config.Side
	x0, y0, x1, y1 := h.block1(c)
	places := side * side
	r.count = grown(r.count, places)
	clear(r.count)
	r.layers = 1
	states := 0
	for y := y0; y < y1; y++ {
		for x := x0; x < x1; x++ {
			r.buf = h.layout.At(x, y, r.buf[:0])
			p := (y-y0)*side + x - x0
			r.count[p] = int32(len(r.buf))
			for len(r.at) < len(r.buf)*places {
				r.at = append(r.at, make([]T, places)...)
			}
			for layer, s := range r.buf {
				r.at[layer*places+p] = s
			}
			r.layers = max(r.layers, len(r.buf))
			states += len(r.buf)
		}
	}
	r.members.reset(states)
	for l := range r.layers * places {
		if int(r.count[l%places]) > l/places {
			r.members.put(h.graph.Index(r.at[l]), int32(l))
		}
	}
	return states
}

// read1 reads the graph of cluster c of level 1, scanned into r, over forward into r's graph;
// a build's b gets the steps leaving the cluster and entering it listed. It reports how many
// transitions it read.
func (h *Hierarchy[T]) read1(r *reader[T], c int32, forward graph.Transitions[T],
	b *builder[T]) (read int) {
	g := &r.graph
	places := h.config.Side * h.config.Side
	span := r.layers * places
	g.start = grown(g.start, span+1)
	g.to, g.cost, g.mask = g.to[:0], g.cost[:0], g.mask[:0]
	masked := b != nil && h.all != 0
	if b != nil {
		b.outs, b.ins = b.outs[:0], b.ins[:0]
	}
	for l := range span {
		g.start[l] = int32(len(g.to))
		if int(r.count[l%places]) <= l/places {
			continue
		}
		s := r.at[l]
		r.steps = forward(s, s, r.steps[:0])
		read += len(r.steps)
		for _, st := range r.steps {
			key := h.graph.Index(st.To)
			if to, in := r.members.get(key); in {
				g.to, g.cost = append(g.to, to), append(g.cost, st.Cost)
				if masked {
					g.mask = append(g.mask, st.Refuses&h.all)
				}
			} else if b != nil {
				b.outs = h.leaving(c, int32(l), st, key, b.outs)
			}
		}
		if b == nil {
			continue
		}
		r.steps = h.layout.Backward(s, s, r.steps[:0])
		read += len(r.steps)
		for _, st := range r.steps {
			if key := h.graph.Index(st.To); !r.hasMember(key) {
				b.ins = h.leaving(c, int32(l), st, key, b.ins)
			}
		}
	}
	g.start[span] = int32(len(g.to))
	return read
}

func (r *reader[T]) hasMember(key int) bool {
	_, in := r.members.get(key)
	return in
}

// leaving appends to list step st of the state of local index local where it leads out of
// cluster c to a cluster of its own
func (h *Hierarchy[T]) leaving(c, local int32, st graph.Transition[T], key int,
	list []crossing[T]) []crossing[T] {
	if o, ok := h.cluster1(st.To); ok && o != c {
		list = append(list, crossing[T]{local: local, other: st.To, key: key, c: o,
			refuses: st.Refuses & h.all, cost: st.Cost})
	}
	return list
}

// crossingCost is what the step listed for the state of local index local and the state of key
// costs class bit, +Inf where it is listed for none
func crossingCost[T comparable](list []crossing[T], local int32, key int, bit uint8) float64 {
	i, _ := slices.BinarySearchFunc(list, local, func(x crossing[T], l int32) int {
		return cmp.Compare(x.local, l)
	})
	for ; i < len(list) && list[i].local == local; i++ {
		if list[i].key == key && list[i].refuses&bit == 0 {
			return list[i].cost
		}
	}
	return math.Inf(1)
}

// candidatesOut1 lists the cluster's border states stepping into cluster o, each with its best
// step there, of the steps class bit takes
func (h *Hierarchy[T]) candidatesOut1(o int32, x0, y0 int, bit uint8) {
	b, side := &h.b, h.config.Side
	b.cands = b.cands[:0]
	if bit == 0 {
		b.arena = b.arena[:0]
	}
	for i := 0; i < len(b.outs); {
		local := b.outs[i].local
		best, bestKey, bestRev := -1, 0.0, 0.0
		for ; i < len(b.outs) && b.outs[i].local == local; i++ {
			x := &b.outs[i]
			if x.c != o || x.refuses&bit != 0 {
				continue
			}
			rev := crossingCost(b.ins, local, x.key, bit)
			key := crossingKey(x.cost, rev)
			if best < 0 || key < bestKey || key == bestKey && x.key < b.outs[best].key {
				best, bestKey, bestRev = i, key, rev
			}
		}
		if best < 0 {
			continue
		}
		x := &b.outs[best]
		s, p := b.at[local], int(local)%(side*side)
		cd := candidate[T]{fromKey: h.graph.Index(s), toKey: x.key, x: x0 + p%side,
			y: y0 + p/side, key: bestKey}
		if bit == 0 {
			cd.fromSteps, cd.toSteps = h.stepsOf(s), h.stepsOf(x.other)
		} else {
			cd.fromSteps, cd.toSteps = h.listed(s, cd.fromKey), h.listed(x.other, x.key)
		}
		cd.twin = pair[T]{node: s, key: cd.fromKey, local: local, other: o, otherKey: x.key,
			out: float32(x.cost), in: float32(bestRev)}
		b.cands = append(b.cands, cd)
	}
}

// candidatesIn1 lists the border states of cluster o stepping into the cluster, each with its
// best step here, of the steps class bit takes
func (h *Hierarchy[T]) candidatesIn1(o int32, bit uint8) {
	b := &h.b
	b.cands = b.cands[:0]
	if bit == 0 {
		b.arena = b.arena[:0]
	}
	b.group = b.group[:0]
	for _, x := range b.ins {
		if x.c == o && x.refuses&bit == 0 {
			b.group = append(b.group, x)
		}
	}
	slices.SortFunc(b.group, func(a, c crossing[T]) int {
		return cmp.Or(cmp.Compare(a.key, c.key), cmp.Compare(a.local, c.local))
	})
	for i := 0; i < len(b.group); {
		from := b.group[i].key
		best, bestKey, bestRev, bestTo := -1, 0.0, 0.0, 0
		for ; i < len(b.group) && b.group[i].key == from; i++ {
			x := &b.group[i]
			rev := crossingCost(b.outs, x.local, from, bit)
			key, to := crossingKey(x.cost, rev), h.graph.Index(b.at[x.local])
			if best < 0 || key < bestKey || key == bestKey && to < bestTo {
				best, bestKey, bestRev, bestTo = i, key, rev, to
			}
		}
		x := &b.group[best]
		fx, fy, _ := h.layout.Coords(x.other)
		to := b.at[x.local]
		cd := candidate[T]{fromKey: from, toKey: bestTo, x: fx, y: fy, key: bestKey}
		if bit == 0 {
			cd.fromSteps, cd.toSteps = h.stepsOf(x.other), h.stepsOf(to)
		} else {
			cd.fromSteps, cd.toSteps = h.listed(x.other, from), h.listed(to, bestTo)
		}
		cd.twin = pair[T]{node: to, key: bestTo, local: x.local, other: o, otherKey: from,
			out: float32(bestRev), in: float32(x.cost)}
		b.cands = append(b.cands, cd)
	}
}

// stepsOf puts the keys of the states s steps to in the arena, and says where
func (h *Hierarchy[T]) stepsOf(s T) [2]int32 {
	b := &h.b
	lo := int32(len(b.arena))
	b.steps = h.graph.Forward(s, s, b.steps[:0])
	b.read += len(b.steps)
	for _, st := range b.steps {
		b.arena = append(b.arena, h.graph.Index(st.To))
	}
	return [2]int32{lo, int32(len(b.arena))}
}

// listed is where the keys of the states s, of key, steps to lie in the arena, the classes
// refusing each step beside them, put there once a job: the steps within the cluster built off
// its graph for a state of it, every step read off the graph for a state across its border
func (h *Hierarchy[T]) listed(s T, key int) [2]int32 {
	b := &h.b
	if i, ok := b.listed.get(key); ok {
		return b.lists[i]
	}
	lo := int32(len(b.arena))
	if l, in := b.members.get(key); in {
		g := &b.graph
		for e := g.start[l]; e < g.start[l+1]; e++ {
			b.arena = append(b.arena, h.graph.Index(b.at[g.to[e]]))
			b.masks = append(b.masks, g.mask[e])
		}
	} else {
		b.steps = h.graph.Forward(s, s, b.steps[:0])
		b.read += len(b.steps)
		for _, st := range b.steps {
			b.arena = append(b.arena, h.graph.Index(st.To))
			b.masks = append(b.masks, st.Refuses&h.all)
		}
	}
	b.listed.put(key, int32(len(b.lists)))
	b.lists = append(b.lists, [2]int32{lo, int32(len(b.arena))})
	return b.lists[len(b.lists)-1]
}

// stepsTo says whether the steps in the arena step to the state of key, of those class bit takes
func (b *builder[T]) stepsTo(steps [2]int32, key int, bit uint8) bool {
	if bit == 0 {
		return slices.Contains(b.arena[steps[0]:steps[1]], key)
	}
	for i := steps[0]; i < steps[1]; i++ {
		if b.arena[i] == key && b.masks[i]&bit == 0 {
			return true
		}
	}
	return false
}

// joined says whether candidates a and c, a before c along the border, lie on one run: on level
// 1 neighbours stepping to each other on both sides of the border by the steps class bit takes,
// above in one class of one cluster a level down on both sides
func (b *builder[T]) joined(first bool, a, c *candidate[T], bit uint8) bool {
	if !first {
		return a.fromC == c.fromC && a.fromClass == c.fromClass && a.toC == c.toC &&
			a.toClass == c.toClass
	}
	if a.across-c.across > 1 || c.across-a.across > 1 {
		return false
	}
	return b.stepsTo(a.fromSteps, c.fromKey, bit) && b.stepsTo(c.fromSteps, a.fromKey, bit) &&
		(a.toKey == c.toKey ||
			b.stepsTo(a.toSteps, c.toKey, bit) && b.stepsTo(c.toSteps, a.toKey, bit))
}

// pick keeps of the candidates of one border of a cluster of level j, crossing it one way, each
// run's ends, its local minima and its cheapest crossing of every span coordinates along it,
// the span doubled a level, as the twins they make; on level 1 its runs join by the steps class
// bit takes
func (h *Hierarchy[T]) pick(j int, bit uint8) {
	b := &h.b
	cs := b.cands
	n := len(cs)
	if n == 0 {
		return
	}
	x0, x1, y0, y1 := cs[0].x, cs[0].x, cs[0].y, cs[0].y
	for i := range cs {
		x0, x1, y0, y1 = min(x0, cs[i].x), max(x1, cs[i].x), min(y0, cs[i].y), max(y1, cs[i].y)
	}
	for i := range cs {
		c := &cs[i]
		c.along, c.across = c.x, c.y
		if y1-y0 >= x1-x0 {
			c.along, c.across = c.y, c.x
		}
	}
	slices.SortFunc(cs, func(a, c candidate[T]) int {
		return cmp.Or(cmp.Compare(a.along, c.along), cmp.Compare(a.across, c.across),
			cmp.Compare(a.fromKey, c.fromKey))
	})
	b.root = grown(b.root, n)
	for i := range b.root {
		b.root[i] = int32(i)
	}
	first := j == 0
	for i := range n {
		for k := i + 1; k < n; k++ {
			if first && cs[k].along-cs[i].along > 1 || !first && k > i+1 {
				break
			}
			if b.joined(first, &cs[i], &cs[k], bit) {
				joinRoots(b.root, int32(i), int32(k))
			}
		}
	}
	// the candidates run by run, each run along the border
	b.label, b.runs = grown(b.label, n), grown(b.runs, n+1)
	clear(b.runs)
	for i := range n {
		b.label[i] = rootOf(b.root, int32(i))
		b.runs[b.label[i]+1]++
	}
	for i := 1; i <= n; i++ {
		b.runs[i] += b.runs[i-1]
	}
	b.cursor = append(b.cursor[:0], b.runs...)
	b.order = grown(b.order, n)
	for i := range n {
		r := b.label[i]
		b.order[b.cursor[r]] = int32(i)
		b.cursor[r]++
	}
	for r := range n {
		if lo, hi := b.runs[r], b.runs[r+1]; lo < hi {
			keepAlong(cs, b.order[lo:hi], h.config.Span<<j)
		}
	}
	for i := range cs {
		if cs[i].keep {
			b.pairs = append(b.pairs, cs[i].twin)
		}
	}
}

// keepAlong marks kept the candidates of one run, along the border: its ends, its local minima
// of the crossing's cost and its cheapest crossing of every gap coordinates along it
func keepAlong[T comparable](cs []candidate[T], run []int32, gap int) {
	m := len(run)
	for p, i := range run {
		keep := p == 0 || p == m-1
		if !keep {
			key := cs[i].key
			keep = key < cs[run[p-1]].key && key <= cs[run[p+1]].key
		}
		cs[i].keep = keep
	}
	lo := cs[run[0]].along
	for p := 0; p < m; {
		best, stretch := run[p], (cs[run[p]].along-lo)/gap
		for p++; p < m && (cs[run[p]].along-lo)/gap == stretch; p++ {
			if cs[run[p]].key < cs[best].key {
				best = run[p]
			}
		}
		cs[best].keep = true
	}
}

func rootOf(root []int32, i int32) int32 {
	for root[i] != i {
		root[i] = root[root[i]]
		i = root[i]
	}
	return i
}

func joinRoots(root []int32, i, k int32) {
	a, b := rootOf(root, i), rootOf(root, k)
	if a > b {
		a, b = b, a
	}
	root[b] = a
}

// assemble makes the cluster's nodes and twins of the twins chosen
func (h *Hierarchy[T]) assemble(cl *cluster[T], j int) {
	b := &h.b
	slices.SortFunc(b.pairs, func(a, c pair[T]) int {
		return cmp.Or(cmp.Compare(a.key, c.key), cmp.Compare(a.otherKey, c.otherKey))
	})
	b.pairs = slices.CompactFunc(b.pairs, func(a, c pair[T]) bool {
		return a.key == c.key && a.otherKey == c.otherKey
	})
	cl.keys, cl.states, cl.local, cl.down = cl.keys[:0], cl.states[:0], cl.local[:0], cl.down[:0]
	cl.twinAt, cl.twins = cl.twinAt[:0], cl.twins[:0]
	for i := range b.pairs {
		p := &b.pairs[i]
		if i == 0 || p.key != b.pairs[i-1].key {
			cl.twinAt = append(cl.twinAt, int32(len(cl.twins)))
			cl.keys, cl.states = append(cl.keys, p.key), append(cl.states, p.node)
			if j == 0 {
				cl.local = append(cl.local, p.local)
			} else {
				cl.down = append(cl.down, p.down)
			}
		}
		cl.twins = append(cl.twins, twin{other: p.other, key: p.otherKey, out: p.out, in: p.in})
	}
	cl.twinAt = append(cl.twinAt, int32(len(cl.twins)))
}

// fillSetup sizes the cluster's costs between its nodes, and its trees over span local indices
// where span is not 0, for its nodes to be filled one a step
func fillSetup[T comparable](cl *cluster[T], span int) {
	n := len(cl.keys)
	cl.intra = grown(cl.intra, n*n)
	cl.span = int32(span)
	cl.tree8, cl.tree16 = cl.tree8[:0], cl.tree16[:0]
	// cleared, as a cluster built afresh holds them, where a fill settles no state
	switch {
	case span > 1<<16:
		cl.span = 0
	case span > 1<<8:
		cl.tree16 = grown(cl.tree16, n*span)
		clear(cl.tree16)
	case span > 0:
		cl.tree8 = grown(cl.tree8, n*span)
		clear(cl.tree8)
	}
}

// fillRow fills g from the cluster's node i, of local index locals[i], to the others of locals:
// their costs, and its tree where the cluster keeps trees; it reports the work done
func (h *Hierarchy[T]) fillRow(cl *cluster[T], g *localGraph, locals []int32, i int) int {
	f, n, span := &h.b.fill, len(locals), int(cl.span)
	f.run(g, locals[i], locals)
	row := cl.intra[i*n : (i+1)*n]
	for k, t := range locals {
		row[k] = float32(f.cost(t))
	}
	if len(cl.tree8) > 0 {
		tree := cl.tree8[i*span : (i+1)*span]
		for _, u := range f.order {
			tree[u] = uint8(f.parent[u])
		}
	} else if len(cl.tree16) > 0 {
		tree := cl.tree16[i*span : (i+1)*span]
		for _, u := range f.order {
			tree[u] = uint16(f.parent[u])
		}
	}
	work := 1 + workSettled*len(f.order)
	for _, u := range f.order {
		work += int(g.start[u+1] - g.start[u])
	}
	return work
}

// sparsifyStart clears the cluster's edges kept, for sparsifyRow to keep them a node at a time
func sparsifyStart[T comparable](cl *cluster[T]) {
	n := len(cl.keys)
	words := (n + 63) >> 6
	cl.out, cl.in = grown(cl.out, n*words), grown(cl.in, n*words)
	clear(cl.out)
	clear(cl.in)
}

// sparsifyRow keeps of the cluster's edges from node u those no two strictly shorter edges
// through a third node make as cheaply, within float32's rounding, so that the paths over the
// edges kept cost what the cheapest do; it reports how many edges it checked
func sparsifyRow[T comparable](cl *cluster[T], u int) (checks int) {
	n := len(cl.keys)
	words := (n + 63) >> 6
	row := cl.intra[u*n : (u+1)*n]
	for w, c := range row {
		if w == u || c == inf32 {
			continue
		}
		made, bound := false, float64(c)*(1+1e-6)
		for v, a := range row {
			checks++
			if b := cl.intra[v*n+w]; a < c && b < c && v != w && v != u &&
				float64(a)+float64(b) <= bound {
				made = true
				break
			}
		}
		if !made {
			cl.out[u*words+w>>6] |= 1 << (w & 63)
			cl.in[w*words+u>>6] |= 1 << (u & 63)
		}
	}
	return checks
}

// classify has each node of the cluster know the least node it reaches and is reached from
func classify[T comparable](cl *cluster[T]) {
	n := len(cl.keys)
	cl.class = grown(cl.class, n)
	for i := range n {
		cl.class[i] = int32(i)
		for k := range i {
			if cl.intra[i*n+k] != inf32 && cl.intra[k*n+i] != inf32 {
				cl.class[i] = cl.class[k]
				break
			}
		}
	}
}

// prepareK readies class k's view of cluster c of level j above 1 to be filled over the graph of
// the level below as the class sees it: its entrances chosen of the twins of its children
// leading out, that graph read; it reports the work done
func (h *Hierarchy[T]) prepareK(k, j int, c int32) int {
	b, v := &h.b, &h.views[k]
	lv, dn := &h.levels[j], &h.levels[j-1]
	cl, down := &v.levels[j].clusters[c], v.levels[j-1].clusters
	f := int32(h.config.Factor)
	cx, cy := c%lv.cols, c/lv.cols
	cl.offset = grown(cl.offset, int(f*f)+1)
	cl.offset[0] = 0
	b.crossK = b.crossK[:0]
	for s := range f * f {
		cl.offset[s+1] = cl.offset[s]
		ccx, ccy := cx*f+s%f, cy*f+s/f
		if ccx >= dn.cols || ccy >= dn.rows {
			continue
		}
		ch := ccy*dn.cols + ccx
		chl := &down[ch]
		cl.offset[s+1] += int32(len(chl.keys))
		for u := range chl.keys {
			for _, tw := range chl.twins[chl.twinAt[u]:chl.twinAt[u+1]] {
				if dn.parent[tw.other] == c {
					continue
				}
				if i, ok := v.nodeIndex(j-1, tw.other, tw.key); ok {
					b.crossK = append(b.crossK, crossingK{u: nodeRef{ch, int32(u)},
						v: nodeRef{tw.other, i}, uKey: chl.keys[u], vKey: tw.key,
						up: dn.parent[tw.other], out: tw.out, in: tw.in})
				}
			}
		}
	}
	b.others = b.others[:0]
	for i := range b.crossK {
		b.others = append(b.others, b.crossK[i].up)
	}
	slices.Sort(b.others)
	b.others = slices.Compact(b.others)
	b.pairs = b.pairs[:0]
	for _, o := range b.others {
		h.candidatesOutK(v, j, o)
		h.pick(j, 0)
		h.candidatesInK(v, j, o)
		h.pick(j, 0)
	}
	h.assemble(cl, j)
	return len(b.crossK) + len(b.pairs) + h.prepareCosts(k, j, c)
}

// prepareCosts readies class k's view of cluster c of level j above 1, its entrances kept, to be
// filled over the graph of the level below as the class sees it, read; it reports the work done
func (h *Hierarchy[T]) prepareCosts(k, j int, c int32) int {
	b, v, dn := &h.b, &h.views[k], &h.levels[j-1]
	cl, g := &v.levels[j].clusters[c], h.fillGraph(k)
	h.readK(v, g, j, c, false)
	b.locals[k] = grown(b.locals[k], len(cl.down))
	for i, r := range cl.down {
		b.locals[k][i] = cl.offset[dn.slot[r.c]] + r.i
	}
	fillSetup(cl, int(cl.offset[h.config.Factor*h.config.Factor]))
	return 1 + len(g.to) + len(cl.down)
}

// readK reads the graph of v's level below within cluster c of level j into g, by local index:
// each node's steps out of it, or with back each node's steps into it
func (h *Hierarchy[T]) readK(v *view[T], g *localGraph, j int, c int32, back bool) {
	lv, dn := &h.levels[j], &h.levels[j-1]
	cl, down := &v.levels[j].clusters[c], v.levels[j-1].clusters
	f := int32(h.config.Factor)
	cx, cy := c%lv.cols, c/lv.cols
	span := cl.offset[f*f]
	g.start = grown(g.start, int(span)+1)
	g.to, g.cost = g.to[:0], g.cost[:0]
	for s := range f * f {
		ccx, ccy := cx*f+s%f, cy*f+s/f
		if ccx >= dn.cols || ccy >= dn.rows {
			continue
		}
		chl := &down[ccy*dn.cols+ccx]
		n, base := int32(len(chl.keys)), cl.offset[s]
		kept := chl.out
		if back {
			kept = chl.in
		}
		for i := range n {
			g.start[base+i] = int32(len(g.to))
			for w, word := range chl.row(kept, i) {
				for ; word != 0; word &= word - 1 {
					k := int32(w<<6 + bits.TrailingZeros64(word))
					cost := chl.intra[i*n+k]
					if back {
						cost = chl.intra[k*n+i]
					}
					g.to, g.cost = append(g.to, base+k), append(g.cost, float64(cost))
				}
			}
			for _, tw := range chl.twins[chl.twinAt[i]:chl.twinAt[i+1]] {
				cost := tw.out
				if back {
					cost = tw.in
				}
				if cost == inf32 || dn.parent[tw.other] != c {
					continue
				}
				if i, ok := v.nodeIndex(j-1, tw.other, tw.key); ok {
					g.to = append(g.to, cl.offset[dn.slot[tw.other]]+i)
					g.cost = append(g.cost, float64(cost))
				}
			}
		}
	}
	g.start[span] = int32(len(g.to))
}

// candidatesOutK lists the nodes of v's children stepping into cluster o of level j, each with
// its best step there
func (h *Hierarchy[T]) candidatesOutK(v *view[T], j int, o int32) {
	b, down := &h.b, v.levels[j-1].clusters
	b.cands = b.cands[:0]
	for i := 0; i < len(b.crossK); {
		u := b.crossK[i].u
		best, bestKey, bestTo := -1, 0.0, 0
		for ; i < len(b.crossK) && b.crossK[i].u == u; i++ {
			x := &b.crossK[i]
			if x.up != o || x.out == inf32 {
				continue
			}
			key := crossingKey(float64(x.out), float64(x.in))
			if best < 0 || key < bestKey || key == bestKey && x.vKey < bestTo {
				best, bestKey, bestTo = i, key, x.vKey
			}
		}
		if best >= 0 {
			x := &b.crossK[best]
			b.cands = append(b.cands, h.candidateK(down, x.u, x.v, bestKey, x.u, o, x.out, x.in))
		}
	}
}

// candidatesInK lists v's nodes of cluster o of level j stepping into the children, each with
// its best step here
func (h *Hierarchy[T]) candidatesInK(v *view[T], j int, o int32) {
	b, down := &h.b, v.levels[j-1].clusters
	b.cands = b.cands[:0]
	b.groupK = b.groupK[:0]
	for _, x := range b.crossK {
		if x.up == o && x.in != inf32 {
			b.groupK = append(b.groupK, x)
		}
	}
	slices.SortFunc(b.groupK, func(a, c crossingK) int {
		return cmp.Or(cmp.Compare(a.vKey, c.vKey), cmp.Compare(a.uKey, c.uKey))
	})
	for i := 0; i < len(b.groupK); {
		v := b.groupK[i].v
		best, bestKey := i, 0.0
		for ; i < len(b.groupK) && b.groupK[i].v == v; i++ {
			x := &b.groupK[i]
			if key := crossingKey(float64(x.in), float64(x.out)); i == best || key < bestKey {
				best, bestKey = i, key
			}
		}
		x := &b.groupK[best]
		b.cands = append(b.cands, h.candidateK(down, x.v, x.u, bestKey, x.u, o, x.out, x.in))
	}
}

// candidateK is the candidate of the step from from to to of the clusters down, the twin of node
// of the cluster built with the node across in cluster o
func (h *Hierarchy[T]) candidateK(down []cluster[T], from, to nodeRef, key float64,
	node nodeRef, o int32, out, in float32) candidate[T] {
	fc, tc := &down[from.c], &down[to.c]
	x, y, _ := h.layout.Coords(fc.states[from.i])
	cd := candidate[T]{fromKey: fc.keys[from.i], toKey: tc.keys[to.i], x: x, y: y, key: key,
		fromC: from.c, toC: to.c, fromClass: fc.class[from.i], toClass: tc.class[to.i]}
	nc, across := &down[node.c], cd.toKey
	if node != from {
		across = cd.fromKey
	}
	cd.twin = pair[T]{node: nc.states[node.i], key: nc.keys[node.i], down: node, other: o,
		otherKey: across, out: out, in: in}
	return cd
}
