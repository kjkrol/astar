package hierarchy

import (
	"math"
	"slices"
	"time"

	"github.com/kjkrol/hpastar/graph"
)

// Hierarchy is HPA* over a graph laid out in a plane: levels of clusters, entrances on their
// borders, the costs and trees between them computed once a cluster, an abstract search on the
// highest level the ends' distance allows and the path refined down the trees, each of these
// for every class of the graph apart. It is built on one goroutine, a budget of work at a time,
// one job after another; its climbs may ask at once.
type Hierarchy[T comparable] struct {
	graph  graph.Implicit[T]
	layout graph.Layout[T]
	config Config
	w, h   int
	levels []level
	// by class: the clusters as its steps make them, and its labels
	views []view[T]
	// the bits of the classes in a step's Refuses; 0 where the graph has one class, whose steps
	// are read as every class's
	all uint8
	// by class: the bytes its trees of level 1 take
	trees  []int
	builds uint64
	b      builder[T]
	job    job
	// the work the last build did, and the work of its last step
	worked, stepWorked int
	// by level from 0: the clusters built whole, and those whose costs alone were filled again
	rebuilds, refills []uint64
	// told how long each cluster built took, by level, and the labels, as level 0; nil but in
	// the benchmarks
	rebuilt func(level int, took time.Duration)
}

// the work a build counts, in steps of about an edge a fill relaxes each: a transition read off
// the graph counts workRead, a state a fill settles workSettled, and workChecks checks of an
// edge sparsify keeps or drops one
const (
	workRead    = 3
	workSettled = 2
	workChecks  = 4
)

// a job is what a build has under way: a cluster built in steps (made ready, filled from one
// node of one class a step, sparsified a node's row a step, finished) or the labels of a class
// worked out
type job struct {
	phase phase
	j     int
	c     int32
	kind  staleness
	// the staleness the cluster was marked with again while the job was under way
	again staleness
	// the class of the next fill or row, or of the labels
	k int
	// the next fill or row, or the next cluster of the labels
	i    int32
	took time.Duration
	// by class: the cluster's view as it was before the job
	was []former
	// the clusters its twins led to before the job, every class's
	across []int32
}

// a cluster of a view as it was before a job built it
type former struct {
	keys   []int
	class  []int32
	twinAt []int32
	twins  []twin
	intra  []float32
}

type phase uint8

const (
	idle phase = iota
	preparing
	filling
	sparsifying
	finishing
	// the labels: the clusters changed alone, or whole, each node its own, joined, named
	relabelling
	rooting
	joining
	naming
)

// Config is the shape of a hierarchy, the distance its climbs start from and the memory of its
// trees.
type Config struct {
	Side   int     // a first-level cluster's side, in coordinates
	Factor int     // the clusters a side of a cluster a level up
	Top    int     // the most clusters the top level may have
	Span   int     // a border keeps its cheapest crossing every Span coordinates, twice a level up
	Trees  int     // the bytes the first level's trees of each class may take
	Near   float64 // how far apart, by the Heuristic, ends lie to climb, Factor times a level up
}

// New is the hierarchy of shape c over g, whose Layout and classes its caller checked, nothing
// built yet.
func New[T comparable](g graph.Implicit[T], c Config) *Hierarchy[T] {
	h := &Hierarchy[T]{graph: g, layout: *g.Layout, config: c}
	h.w, h.h = h.layout.Width, h.layout.Height
	classes := max(g.Classes, 1)
	h.views = make([]view[T], classes)
	if classes > 1 {
		h.all = uint8(1<<classes - 1)
	}
	h.lay()
	h.prepare()
	return h
}

// Stale marks the clusters of level 1 s lies in and those across the borders it lies on to be
// built again; the levels above follow where those change.
func (h *Hierarchy[T]) Stale(s T) {
	x, y, ok := h.layout.Coords(s)
	if !ok || !h.inPlane(x, y) {
		return
	}
	c := h.clusterAt(x, y)
	h.mark(0, c, rebuild)
	for dy := -1; dy <= 1; dy++ {
		for dx := -1; dx <= 1; dx++ {
			if h.inPlane(x+dx, y+dy) {
				h.mark(0, h.clusterAt(x+dx, y+dy), rebuild)
			}
		}
	}
	b := &h.b
	for _, steps := range [2]graph.Transitions[T]{h.graph.Forward, h.layout.Backward} {
		b.steps = steps(s, s, b.steps[:0])
		for _, st := range b.steps {
			if o, ok := h.cluster1(st.To); ok {
				h.mark(0, o, rebuild)
			}
		}
	}
	key := h.graph.Index(s)
	for k := range h.views {
		v := &h.views[k]
		cl := &v.levels[0].clusters[c]
		if i, ok := v.nodeIndex(0, c, key); ok {
			for _, tw := range cl.twins[cl.twinAt[i]:cl.twinAt[i+1]] {
				h.mark(0, tw.other, rebuild)
			}
		}
	}
}

// StaleAll marks every cluster of level 1 to be built again.
func (h *Hierarchy[T]) StaleAll() {
	for c := range h.levels[0].stale {
		h.mark(0, int32(c), rebuild)
	}
}

// Build works through up to budget steps of work, all that is stale for a budget of 0, and
// reports how many clusters it finished and how many stay stale, the labels of every class
// counted as one while they are worked out; it stops after the step that spends the budget.
func (h *Hierarchy[T]) Build(budget int) (built, left int) {
	if budget <= 0 {
		budget = math.MaxInt
	}
	h.worked = 0
	for h.worked < budget && (h.job.phase != idle || h.next()) {
		var began time.Time
		if h.rebuilt != nil {
			began = time.Now()
		}
		work, done := h.step()
		h.worked += work
		h.stepWorked = work
		if h.rebuilt != nil {
			h.job.took += time.Since(began)
		}
		if !done {
			continue
		}
		level := 0
		if h.job.phase < relabelling {
			built++
			level = h.job.j + 1
		}
		if h.rebuilt != nil {
			h.rebuilt(level, h.job.took)
		}
		h.job.phase = idle
	}
	// what the climbs read and would otherwise work out themselves
	for j := range h.levels {
		for k := range h.views {
			h.views[k].based(j)
		}
		left += h.levels[j].stales
	}
	if _, ok := h.pendingLabels(); ok {
		left++
	}
	return built, left
}

// Ready says no cluster is stale and the labels of every class are worked out.
func (h *Hierarchy[T]) Ready() bool {
	for j := range h.levels {
		if h.levels[j].stales > 0 {
			return false
		}
	}
	_, pending := h.pendingLabels()
	return !pending
}

// pendingLabels is the first class whose labels are not worked out for level 1 as it is
func (h *Hierarchy[T]) pendingLabels() (int, bool) {
	for k := range h.views {
		if h.views[k].labels.pending() {
			return k, true
		}
	}
	return 0, false
}

// next starts the job to do next, false where none is left: a stale cluster of level 1, then the
// labels of each class in turn, then a stale cluster of the lowest level above, in cluster order
func (h *Hierarchy[T]) next() bool {
	if c, ok := h.nextStale(0); ok {
		h.begin(0, c)
		return true
	}
	if k, ok := h.pendingLabels(); ok {
		h.beginLabels(k)
		return true
	}
	for j := 1; j < len(h.levels); j++ {
		if c, ok := h.nextStale(j); ok {
			h.begin(j, c)
			return true
		}
	}
	return false
}

// begin starts building cluster c of level j as it was left stale; it stays stale until the job
// is done
func (h *Hierarchy[T]) begin(j int, c int32) {
	jb := &h.job
	jb.phase, jb.j, jb.c, jb.kind, jb.again, jb.k, jb.i, jb.took =
		preparing, j, c, h.levels[j].stale[c], fresh, 0, 0, 0
}

// step does the next step of the job under way, and reports its work and whether the job is done
func (h *Hierarchy[T]) step() (work int, done bool) {
	jb := &h.job
	switch jb.phase {
	case preparing:
		work = h.prepareJob()
		jb.phase, jb.k, jb.i = filling, 0, 0
		if !h.nodesLeft() {
			jb.phase = finishing
		}
	case filling:
		cl := h.under(jb.k)
		work = h.fillRow(cl, h.fillGraph(jb.k), h.locals(jb.k), int(jb.i))
		if jb.i++; int(jb.i) == len(cl.keys) {
			if jb.k, jb.i = jb.k+1, 0; !h.nodesLeft() {
				for k := range h.views {
					if cl := h.under(k); len(cl.keys) > 0 {
						sparsifyStart(cl)
					}
				}
				jb.phase, jb.k = sparsifying, 0
				h.nodesLeft()
			}
		}
	case sparsifying:
		cl := h.under(jb.k)
		work = 1 + sparsifyRow(cl, int(jb.i))/workChecks
		if jb.i++; int(jb.i) == len(cl.keys) {
			if jb.k, jb.i = jb.k+1, 0; !h.nodesLeft() {
				jb.phase = finishing
			}
		}
	case finishing:
		return h.finish(), true
	default:
		return h.labelStep()
	}
	return work, false
}

// under is class k's view of the cluster the job has under way
func (h *Hierarchy[T]) under(k int) *cluster[T] {
	return &h.views[k].levels[h.job.j].clusters[h.job.c]
}

// nodesLeft moves the job on to the first class from its own whose view of the cluster has
// nodes; false where none has
func (h *Hierarchy[T]) nodesLeft() bool {
	jb := &h.job
	for ; jb.k < len(h.views); jb.k++ {
		if len(h.under(jb.k).keys) > 0 {
			return true
		}
	}
	return false
}

// locals are the local indices of class k's nodes of the cluster under way: its own on level 1,
// read off its children's above
func (h *Hierarchy[T]) locals(k int) []int32 {
	if h.job.j == 0 {
		return h.under(k).local
	}
	return h.b.locals[k]
}

// fillGraph is the graph class k's nodes of the cluster under way are filled over: of one class
// the graph read, of several the class's own
func (h *Hierarchy[T]) fillGraph(k int) *localGraph {
	if len(h.views) == 1 {
		return &h.b.graph
	}
	return &h.b.graphs[k]
}

// prepareJob keeps the cluster of the job as every class sees it, then readies it to be filled:
// on level 1 read off the graph once, the entrances of each class chosen; above read off the
// level below as each class sees it, its entrances chosen again unless its children's costs
// alone changed
func (h *Hierarchy[T]) prepareJob() int {
	jb := &h.job
	jb.across = jb.across[:0]
	work := 1
	for k := range h.views {
		cl, w := h.under(k), &jb.was[k]
		w.keys = append(w.keys[:0], cl.keys...)
		w.class = append(w.class[:0], cl.class...)
		w.twinAt = append(w.twinAt[:0], cl.twinAt...)
		w.twins = append(w.twins[:0], cl.twins...)
		w.intra = append(w.intra[:0], cl.intra...)
		for _, tw := range cl.twins {
			jb.across = append(jb.across, tw.other)
		}
		work += len(w.intra) / workChecks
	}
	switch {
	case jb.j == 0:
		work += h.prepare1(jb.c)
	case jb.kind == refill:
		for k := range h.views {
			work += h.prepareCosts(k, jb.j, jb.c)
		}
	default:
		for k := range h.views {
			work += h.prepareK(k, jb.j, jb.c)
		}
	}
	return work
}

// finish classifies the cluster of the job and marks what its change leaves stale: the cluster
// above it whole where the nodes of a class, their classes or their twins changed, that
// cluster's costs where its own costs alone did; the clusters above across its borders where the
// nodes of a class or their classes changed, as they may choose other entrances; on level 1 the
// labels of a class where its connectivity changed
func (h *Hierarchy[T]) finish() int {
	jb := &h.job
	j, c := jb.j, jb.c
	lv := &h.levels[j]
	work := 1
	for k := range h.views {
		cl := h.under(k)
		classify(cl)
		work += len(cl.intra) / workChecks
		if len(jb.was[k].keys) != len(cl.keys) {
			h.views[k].levels[j].based = false
		}
	}
	h.builds++
	if jb.kind == refill {
		h.refills[j]++
	} else {
		h.rebuilds[j]++
	}
	if lv.stale[c] = jb.again; jb.again == fresh {
		lv.stales--
	}
	if j == 0 {
		for k := range h.views {
			work += h.note(k, c)
		}
	}
	if lv.parent == nil {
		return work
	}
	nodes, twins, costs := true, true, true
	for k := range h.views {
		cl, w := h.under(k), &jb.was[k]
		nodes = nodes && slices.Equal(w.keys, cl.keys) && slices.Equal(w.class, cl.class)
		twins = twins && slices.Equal(w.twinAt, cl.twinAt) && slices.Equal(w.twins, cl.twins)
		costs = costs && slices.Equal(w.intra, cl.intra)
	}
	switch {
	case !nodes || !twins:
		h.mark(j+1, lv.parent[c], rebuild)
	case !costs:
		h.mark(j+1, lv.parent[c], refill)
	}
	if nodes {
		return work
	}
	for k := range h.views {
		for _, tw := range h.under(k).twins {
			jb.across = append(jb.across, tw.other)
		}
	}
	for _, o := range jb.across {
		if p := lv.parent[o]; p != lv.parent[c] {
			h.mark(j+1, p, rebuild)
		}
	}
	return work + len(jb.across)
}
