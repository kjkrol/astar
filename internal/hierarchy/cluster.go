package hierarchy

import (
	"math"
	"math/bits"
	"slices"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/dijkstra"
	"github.com/kjkrol/hpastar/internal/heap"
)

// a node of one level: its cluster there and its index among the cluster's nodes; a search's
// start and goal are the nodes of cluster -1
type nodeRef struct{ c, i int32 }

var (
	startRef = nodeRef{c: -1}
	goalRef  = nodeRef{c: -1, i: 1}
	inf32    = float32(math.Inf(1))
)

// a level of clusters, cols × rows: on level 1 blocks of side × side coordinates, above blocks
// of factor × factor clusters a level down
type level[T comparable] struct {
	cols, rows int32
	clusters   []cluster[T]
	parent     []int32 // by cluster: the cluster a level up holding it; nil on the top
	slot       []int32 // by cluster: its place in the block of that cluster
	stale      []staleness
	stales     int
	next       int32   // no cluster below it is stale
	base       []int32 // by cluster and one more: its first node's index among the level's
	based      bool    // base is up to date
}

// staleness is what of a cluster a change left to build: nothing, its costs and trees over the
// entrances it keeps, or the whole of it
type staleness uint8

const (
	fresh staleness = iota
	refill
	rebuild
)

// a cluster's nodes, the steps across its border from them, and the cheapest paths between them
// within it: their costs and the trees they were read from
type cluster[T comparable] struct {
	keys   []int     // by node: its state's key, ascending
	states []T       // by node
	local  []int32   // level 1, by node: its state's local index
	down   []nodeRef // above, by node: itself a level down
	class  []int32   // by node: the least node it reaches within the cluster and is reached from
	twinAt []int32   // by node and one more: where its twins start
	twins  []twin
	intra  []float32 // by from node, then to node: the cheapest within the cluster, +Inf none
	// the edges kept, those no two shorter ones through a third node make as cheaply, a bit
	// each: out by from node, then to node; in by to node, then from node; a row whole words
	out, in []uint64
	offset  []int32 // above, by child slot and one more: the local index of its first node
	// by from node, then local index: the parent there on the cheapest path from the node
	tree8  []uint8
	tree16 []uint16
	span   int32   // the local indices of a tree; 0 where the cluster keeps no trees
	layers int32   // level 1: the most states at a place
	bytes  int     // level 1: what its trees take
	label  []int32 // level 1, by node: the name of its component, as labels.alias reads it
}

// a step across a cluster's border between one of its nodes and the state across
type twin struct {
	other   int32   // the cluster across
	key     int     // the state across
	out, in float32 // the step there and the step back; +Inf none
}

// row is node u's row of the bits of kept, out or in
func (cl *cluster[T]) row(kept []uint64, u int32) []uint64 {
	words := int32(len(cl.keys)+63) >> 6
	return kept[u*words : (u+1)*words]
}

func (cl *cluster[T]) parent(from, l int32) int32 {
	if len(cl.tree8) > 0 {
		return int32(cl.tree8[from*cl.span+l])
	}
	return int32(cl.tree16[from*cl.span+l])
}

// place1 is the cluster of level 1 a fill keeps to: its block of coordinates, and the state
// last found in it, whose place the dictionary asks for next
type place1[T comparable] struct {
	coords         func(T) (int, int, bool)
	side           int
	x0, y0, x1, y1 int
	last           T
	lastAt         int
	known          bool
}

func (p *place1[T]) set(x0, y0, x1, y1 int) {
	p.x0, p.y0, p.x1, p.y1, p.known = x0, y0, x1, y1, false
}

func (p *place1[T]) within(s T) bool {
	x, y, ok := p.coords(s)
	if !ok || x < p.x0 || y < p.y0 || x >= p.x1 || y >= p.y1 {
		return false
	}
	p.last, p.lastAt, p.known = s, (y-p.y0)*p.side+x-p.x0, true
	return true
}

func (p *place1[T]) home(s T) int {
	if p.known && s == p.last {
		return p.lastAt
	}
	x, y, _ := p.coords(s)
	return (y-p.y0)*p.side + x - p.x0
}

// placeTop is the level a search runs over, its start and goal past its nodes
type placeTop struct {
	base  []int32
	total int32
}

func (p *placeTop) home(r nodeRef) int {
	if r.c < 0 {
		return int(p.total + r.i)
	}
	return int(p.base[r.c] + r.i)
}

// a dijkstra of level 1 with the cluster it keeps to
type fill1[T comparable] struct {
	d  *dijkstra.Engine[T]
	at *place1[T]
	in func(T) bool
}

func (h *Hierarchy[T]) newFill1() fill1[T] {
	side := h.config.Side
	at := &place1[T]{coords: h.layout.Coords, side: side}
	d := dijkstra.New[T](dict.NewLocal(side*side, at.home), 2*side*side)
	return fill1[T]{d: d, at: at, in: at.within}
}

// lay cuts the plane into levels of clusters, every one stale
func (h *Hierarchy[T]) lay() {
	side, f := h.config.Side, h.config.Factor
	cols, rows := (h.w+side-1)/side, (h.h+side-1)/side
	if h.w <= 0 || h.h <= 0 {
		cols, rows = 0, 0
	}
	for {
		n := cols * rows
		lv := level[T]{cols: int32(cols), rows: int32(rows), clusters: make([]cluster[T], n),
			stale: make([]staleness, n), stales: n, base: make([]int32, n+1)}
		for i := range lv.stale {
			lv.stale[i] = rebuild
		}
		h.levels = append(h.levels, lv)
		if n <= h.config.Top {
			break
		}
		cols, rows = (cols+f-1)/f, (rows+f-1)/f
	}
	for j := 0; j+1 < len(h.levels); j++ {
		lv, up, f := &h.levels[j], &h.levels[j+1], int32(f)
		lv.parent, lv.slot = make([]int32, len(lv.clusters)), make([]int32, len(lv.clusters))
		for c := range int32(len(lv.clusters)) {
			cx, cy := c%lv.cols, c/lv.cols
			lv.parent[c], lv.slot[c] = cy/f*up.cols+cx/f, cy%f*f+cx%f
		}
	}
}

func (h *Hierarchy[T]) prepare() {
	h.b.fill.heap = heap.New(h.config.Side * h.config.Side)
	h.labels.whole = true
	h.labels.at = make([]int32, len(h.levels[0].clusters))
	for c := range h.labels.at {
		h.labels.at[c] = -1
	}
	h.rebuilds, h.refills = make([]uint64, len(h.levels)), make([]uint64, len(h.levels))
}

func (h *Hierarchy[T]) inPlane(x, y int) bool { return x >= 0 && y >= 0 && x < h.w && y < h.h }

func (h *Hierarchy[T]) clusterAt(x, y int) int32 {
	side := h.config.Side
	return int32(y/side)*h.levels[0].cols + int32(x/side)
}

func (h *Hierarchy[T]) cluster1(s T) (int32, bool) {
	x, y, ok := h.layout.Coords(s)
	if !ok || !h.inPlane(x, y) {
		return 0, false
	}
	return h.clusterAt(x, y), true
}

func (h *Hierarchy[T]) block1(c int32) (x0, y0, x1, y1 int) {
	lv, side := &h.levels[0], h.config.Side
	x0, y0 = int(c%lv.cols)*side, int(c/lv.cols)*side
	return x0, y0, min(x0+side, h.w), min(y0+side, h.h)
}

// nodeIndex is the index of the node of key in cluster c of level j
func (h *Hierarchy[T]) nodeIndex(j int, c int32, key int) (int32, bool) {
	i, ok := slices.BinarySearch(h.levels[j].clusters[c].keys, key)
	return int32(i), ok
}

// based is level j with the index of each cluster's first node among the level's up to date
func (h *Hierarchy[T]) based(j int) *level[T] {
	lv := &h.levels[j]
	if !lv.based {
		for c := range lv.clusters {
			lv.base[c+1] = lv.base[c] + int32(len(lv.clusters[c].keys))
		}
		lv.based = true
	}
	return lv
}

// mark leaves cluster c of level j stale as kind says, unless more of it is stale already; the
// cluster a job has under way is marked to be built again once it is done
func (h *Hierarchy[T]) mark(j int, c int32, kind staleness) {
	if jb := &h.job; jb.phase != idle && jb.phase < relabelling && jb.j == j && jb.c == c {
		jb.again = max(jb.again, kind)
		return
	}
	lv := &h.levels[j]
	if lv.stale[c] == fresh {
		lv.stales++
		lv.next = min(lv.next, c)
	}
	lv.stale[c] = max(lv.stale[c], kind)
}

// nextStale is the first stale cluster of level j
func (h *Hierarchy[T]) nextStale(j int) (int32, bool) {
	lv := &h.levels[j]
	if lv.stales == 0 {
		return 0, false
	}
	for lv.stale[lv.next] == fresh {
		lv.next++
	}
	return lv.next, true
}

// edgesOut appends the steps of level j's graph from node from
func (h *Hierarchy[T]) edgesOut(j int, from nodeRef,
	buf []graph.Transition[nodeRef]) []graph.Transition[nodeRef] {
	cl := &h.levels[j].clusters[from.c]
	n := int32(len(cl.keys))
	for w, word := range cl.row(cl.out, from.i) {
		for ; word != 0; word &= word - 1 {
			t := int32(w<<6 + bits.TrailingZeros64(word))
			buf = append(buf, graph.Transition[nodeRef]{To: nodeRef{from.c, t},
				Cost: float64(cl.intra[from.i*n+t])})
		}
	}
	for _, tw := range cl.twins[cl.twinAt[from.i]:cl.twinAt[from.i+1]] {
		if tw.out == inf32 {
			continue
		}
		if v, ok := h.nodeIndex(j, tw.other, tw.key); ok {
			buf = append(buf, graph.Transition[nodeRef]{To: nodeRef{tw.other, v},
				Cost: float64(tw.out)})
		}
	}
	return buf
}
