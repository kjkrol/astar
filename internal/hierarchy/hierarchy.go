package hierarchy

import (
	"math"
	"slices"
	"time"

	"github.com/kjkrol/hpastar/graph"
)

// Hierarchy is HPA* over a graph laid out in a plane: levels of clusters, entrances on their
// borders, the costs and trees between them computed once a cluster, an abstract search on the
// highest level the ends' distance allows and the path refined down the trees. It is built on
// one goroutine, a budget of work at a time, one job after another; its climbs may ask at once.
type Hierarchy[T comparable] struct {
	graph  graph.Implicit[T]
	layout graph.Layout[T]
	config Config
	w, h   int
	levels []level[T]
	// the bytes the trees of level 1 take
	trees  int
	builds uint64
	b      builder[T]
	labels labels
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
// node a step, sparsified a node's row a step, finished) or the labels worked out
type job struct {
	phase phase
	j     int
	c     int32
	kind  staleness
	// the staleness the cluster was marked with again while the job was under way
	again staleness
	// the next fill or row, or the next cluster of the labels
	i    int32
	took time.Duration
	// the cluster as it was before the job
	keys   []int
	class  []int32
	twinAt []int32
	twins  []twin
	intra  []float32
	across []int32
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
	Trees  int     // the bytes the first level's trees may take
	Near   float64 // how far apart, by the Heuristic, ends lie to climb, Factor times a level up
}

// New is the hierarchy of shape c over g, whose Layout its caller checked, nothing built yet.
func New[T comparable](g graph.Implicit[T], c Config) *Hierarchy[T] {
	h := &Hierarchy[T]{graph: g, layout: *g.Layout, config: c}
	h.w, h.h = h.layout.Width, h.layout.Height
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
	cl := &h.levels[0].clusters[c]
	if i, ok := h.nodeIndex(0, c, h.graph.Index(s)); ok {
		for _, tw := range cl.twins[cl.twinAt[i]:cl.twinAt[i+1]] {
			h.mark(0, tw.other, rebuild)
		}
	}
}

// StaleAll marks every cluster of level 1 to be built again.
func (h *Hierarchy[T]) StaleAll() {
	for c := range h.levels[0].clusters {
		h.mark(0, int32(c), rebuild)
	}
}

// Build works through up to budget steps of work, all that is stale for a budget of 0, and
// reports how many clusters it finished and how many stay stale, the labels counted as one while
// they are worked out; it stops after the step that spends the budget.
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
		h.based(j)
		left += h.levels[j].stales
	}
	if h.labels.pending() {
		left++
	}
	return built, left
}

// Ready says no cluster is stale and the labels are worked out.
func (h *Hierarchy[T]) Ready() bool {
	for j := range h.levels {
		if h.levels[j].stales > 0 {
			return false
		}
	}
	return !h.labels.pending()
}

// next starts the job to do next, false where none is left: a stale cluster of level 1, then the
// labels, then a stale cluster of the lowest level above, in cluster order
func (h *Hierarchy[T]) next() bool {
	if c, ok := h.nextStale(0); ok {
		h.begin(0, c)
		return true
	}
	if h.labels.pending() {
		h.beginLabels()
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
	jb.phase, jb.j, jb.c, jb.kind, jb.again, jb.i, jb.took =
		preparing, j, c, h.levels[j].stale[c], fresh, 0, 0
}

// step does the next step of the job under way, and reports its work and whether the job is done
func (h *Hierarchy[T]) step() (work int, done bool) {
	jb := &h.job
	switch jb.phase {
	case preparing:
		work = h.prepareJob()
		jb.phase, jb.i = filling, 0
		if len(h.levels[jb.j].clusters[jb.c].keys) == 0 {
			jb.phase = finishing
		}
	case filling:
		cl := &h.levels[jb.j].clusters[jb.c]
		work = h.fillRow(cl, h.locals(), int(jb.i))
		if jb.i++; int(jb.i) == len(cl.keys) {
			sparsifyStart(cl)
			jb.phase, jb.i = sparsifying, 0
		}
	case sparsifying:
		cl := &h.levels[jb.j].clusters[jb.c]
		work = 1 + sparsifyRow(cl, int(jb.i))/workChecks
		if jb.i++; int(jb.i) == len(cl.keys) {
			jb.phase = finishing
		}
	case finishing:
		return h.finish(), true
	default:
		return h.labelStep()
	}
	return work, false
}

// locals are the local indices of the nodes of the cluster under way: its own on level 1, read
// off its children's above
func (h *Hierarchy[T]) locals() []int32 {
	if h.job.j == 0 {
		return h.levels[0].clusters[h.job.c].local
	}
	return h.b.locals
}

// prepareJob keeps the cluster of the job as it is, then readies it to be filled: on level 1
// read off the graph, its entrances chosen; above read off the level below, its entrances chosen
// again unless its children's costs alone changed
func (h *Hierarchy[T]) prepareJob() int {
	jb := &h.job
	cl := &h.levels[jb.j].clusters[jb.c]
	jb.keys = append(jb.keys[:0], cl.keys...)
	jb.class = append(jb.class[:0], cl.class...)
	jb.twinAt = append(jb.twinAt[:0], cl.twinAt...)
	jb.twins = append(jb.twins[:0], cl.twins...)
	jb.intra = append(jb.intra[:0], cl.intra...)
	jb.across = jb.across[:0]
	for _, tw := range cl.twins {
		jb.across = append(jb.across, tw.other)
	}
	work := 1 + len(jb.intra)/workChecks
	switch {
	case jb.j == 0:
		work += h.prepare1(jb.c)
	case jb.kind == refill:
		work += h.prepareCosts(jb.j, jb.c)
	default:
		work += h.prepareK(jb.j, jb.c)
	}
	return work
}

// finish classifies the cluster of the job and marks what its change leaves stale: the cluster
// above it whole where its nodes, their classes or their twins changed, that cluster's costs
// where its own costs alone did; the clusters above across its borders where its nodes or their
// classes changed, as they may choose other entrances; on level 1 the labels where its
// connectivity changed
func (h *Hierarchy[T]) finish() int {
	jb := &h.job
	j, c := jb.j, jb.c
	lv := &h.levels[j]
	cl := &lv.clusters[c]
	classify(cl)
	work := 1 + len(cl.intra)/workChecks
	h.builds++
	if jb.kind == refill {
		h.refills[j]++
	} else {
		h.rebuilds[j]++
	}
	if len(jb.keys) != len(cl.keys) {
		lv.based = false
	}
	if lv.stale[c] = jb.again; jb.again == fresh {
		lv.stales--
	}
	if j == 0 {
		work += h.note(c)
	}
	if lv.parent == nil {
		return work
	}
	nodes := slices.Equal(jb.keys, cl.keys) && slices.Equal(jb.class, cl.class)
	switch {
	case !nodes || !slices.Equal(jb.twinAt, cl.twinAt) || !slices.Equal(jb.twins, cl.twins):
		h.mark(j+1, lv.parent[c], rebuild)
	case !slices.Equal(jb.intra, cl.intra):
		h.mark(j+1, lv.parent[c], refill)
	}
	if nodes {
		return work
	}
	for _, tw := range cl.twins {
		jb.across = append(jb.across, tw.other)
	}
	for _, o := range jb.across {
		if p := lv.parent[o]; p != lv.parent[c] {
			h.mark(j+1, p, rebuild)
		}
	}
	return work + len(jb.across)
}
