package hierarchy

import (
	"math"
	"slices"
	"sync"
	"testing"
	"unsafe"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/astar"
	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/dijkstra"
	"github.com/kjkrol/hpastar/internal/testmap"
)

// asked is a hierarchy with a climb of its own over its graph's steps, as the tests ask it
type asked[T comparable] struct {
	*Hierarchy[T]
	own *Climb[T]
}

// testHierarchy is the hierarchy over g, nothing built yet, with a climb of its own
func testHierarchy[T comparable](g graph.Implicit[T], opts ...option) asked[T] {
	h := New(g, configOf(opts...))
	return asked[T]{h, h.NewClimb(g.Forward, g.Layout.Backward)}
}

// option changes the shape of a hierarchy a test builds
type option func(*Config)

// configOf is the shape NewSolver gives a hierarchy without options, as opts change it
func configOf(opts ...option) Config {
	c := Config{Side: 16, Factor: 4, Top: 16, Span: 4, Trees: 64 << 20, Near: 32}
	for _, o := range opts {
		o(&c)
	}
	return c
}

// withClusterSide sets the side of a cluster of level 1; 16 by default
func withClusterSide(side int) option { return func(c *Config) { c.Side = side } }

// withFactor sets how many clusters a side a cluster of the level above spans; 4 by default
func withFactor(f int) option { return func(c *Config) { c.Factor = f } }

// withTop sets the most clusters the top level may have; 16 by default
func withTop(clusters int) option { return func(c *Config) { c.Top = clusters } }

// withNear sets how far apart a path's ends lie to climb level 1; 32 by default
func withNear(distance float64) option { return func(c *Config) { c.Near = distance } }

// withTreeMemory sets the bytes the trees of level 1 may take; 64 MB by default
func withTreeMemory(bytes int) option { return func(c *Config) { c.Trees = bytes } }

// clusters is how many clusters level has, from 1
func (h *Hierarchy[T]) clusters(level int) int {
	if level < 1 || level > len(h.levels) {
		return 0
	}
	return len(h.levels[level-1].clusters)
}

// clusterOf is the cluster of s on level, from 1; false for a state without coordinates
func (h *Hierarchy[T]) clusterOf(level int, s T) (int, bool) {
	if level < 1 || level > len(h.levels) {
		return 0, false
	}
	c, ok := h.cluster1(s)
	if !ok {
		return 0, false
	}
	for j := 0; j+1 < level; j++ {
		c = h.levels[j].parent[c]
	}
	return int(c), true
}

// find writes the route the climb finds into dst, as a Path climbing it does; false where the
// climb refuses it
func (s *Climb[T]) find(from, to T, dst []T) (int, bool) {
	if s.Route(from, to) != Climbed {
		return 0, false
	}
	return s.Refine(dst), true
}

// cost is what the abstract route from from to to costs; false where find would refuse it
func (s *Climb[T]) cost(from, to T) (float64, bool) {
	if s.Route(from, to) != Climbed {
		return 0, false
	}
	return s.routeCost(), true
}

// routeCost is what the route found costs, edge by edge
func (s *Climb[T]) routeCost() float64 {
	q := &s.q
	j := q.level
	lv := &s.h.levels[j]
	total := 0.0
	for t := 0; t+1 < len(q.path); t++ {
		u, v := q.path[t], q.path[t+1]
		switch {
		case u == startRef:
			total += q.startCost[j][v.i]
		case v == goalRef:
			total += q.goalCost[j][u.i]
		case u.c == v.c:
			cl := &lv.clusters[u.c]
			total += float64(cl.intra[int(u.i)*len(cl.keys)+int(v.i)])
		default:
			cl, key := &lv.clusters[u.c], lv.clusters[v.c].keys[v.i]
			for _, tw := range cl.twins[cl.twinAt[u.i]:cl.twinAt[u.i+1]] {
				if tw.other == v.c && tw.key == key {
					total += float64(tw.out)
					break
				}
			}
		}
	}
	return total
}

var builtMaps = map[[4]uint64]asked[int]{}
var builtGrids = map[[4]uint64]*testmap.Map{}

// builtOver is a generated map with its hierarchy, built whole, kept for the tests that only read
// them
func builtOver(seed uint64, size int, walls float64, wrap bool) (*testmap.Map, asked[int]) {
	key := [4]uint64{seed, uint64(size), math.Float64bits(walls), 0}
	if wrap {
		key[3] = 1
	}
	if h, ok := builtMaps[key]; ok {
		return builtGrids[key], h
	}
	m := testmap.New(seed, size, walls, wrap)
	h := testHierarchy(m.Graph())
	h.Build(math.MaxInt)
	builtMaps[key], builtGrids[key] = h, m
	return m, h
}

func mapAStar(m *testmap.Map) *astar.Engine[int] {
	return astar.New[int](m.Octile, dict.NewIndexedSlice(m.W*m.H, testmap.Identity), 0)
}

// nodesByLevel is how many nodes each level has
func nodesByLevel[T comparable](h *Hierarchy[T]) []int {
	counts := make([]int, len(h.levels))
	for j := range h.levels {
		for c := range h.levels[j].clusters {
			counts[j] += len(h.levels[j].clusters[c].keys)
		}
	}
	return counts
}

// bytesOf is what the hierarchy's clusters hold, in bytes
func bytesOf[T comparable](h *Hierarchy[T]) int {
	var state T
	total := 0
	for j := range h.levels {
		for c := range h.levels[j].clusters {
			cl := &h.levels[j].clusters[c]
			total += len(cl.keys)*8 + len(cl.states)*int(unsafe.Sizeof(state)) +
				len(cl.local)*4 + len(cl.down)*8 + len(cl.class)*4 + len(cl.twinAt)*4 +
				len(cl.twins)*int(unsafe.Sizeof(twin{})) + len(cl.intra)*4 + len(cl.offset)*4 +
				len(cl.out)*8 + len(cl.in)*8 + len(cl.tree8) + len(cl.tree16)*2
		}
	}
	return total
}

func TestHierarchy_ClustersTileThePlane(t *testing.T) {
	plane := func(w, h int) graph.Implicit[int] {
		return graph.Implicit[int]{Index: testmap.Identity, Size: w * h, Layout: &graph.Layout[int]{
			Coords: func(s int) (int, int, bool) { return s % w, s / w, s >= 0 && s < w*h },
			At:     func(x, y int, buf []int) []int { return append(buf, y*w+x) },
			Width:  w,
			Height: h,
		}}
	}
	for _, c := range []struct {
		size     int
		clusters []int
	}{
		{256, []int{256, 16}},
		{1024, []int{4096, 256, 16}},
		{4096, []int{65536, 4096, 256, 16}},
	} {
		h := New(plane(c.size, c.size), configOf())
		if len(h.levels) != len(c.clusters) {
			t.Fatalf("%d²: %d levels, want %d", c.size, len(h.levels), len(c.clusters))
		}
		for level, want := range c.clusters {
			if got := h.clusters(level + 1); got != want {
				t.Fatalf("%d²: %d clusters on level %d, want %d", c.size, got, level+1, want)
			}
		}
	}
	// 250 × 100 cut by 16: the last column 10 wide, the last row 4 high
	const w, ht = 250, 100
	h := New(plane(w, ht), configOf())
	if len(h.levels) != 2 || h.clusters(1) != 16*7 || h.clusters(2) != 4*2 {
		t.Fatalf("250×100: %d levels of %d and %d clusters, want 2 of 112 and 8",
			len(h.levels), h.clusters(1), h.clusters(2))
	}
	for s := range w * ht {
		x, y := s%w, s/w
		for level, side, cols := 1, 16, 16; level <= 2; level, side, cols = level+1, 64, 4 {
			if c, ok := h.clusterOf(level, s); !ok || c != y/side*cols+x/side {
				t.Fatalf("(%d, %d) on level %d: cluster %d (%v), want %d",
					x, y, level, c, ok, y/side*cols+x/side)
			}
		}
	}
	if x0, y0, x1, y1 := h.block1(int32(h.clusters(1) - 1)); x1-x0 != 10 || y1-y0 != 4 {
		t.Fatalf("the last cluster spans %d × %d, want 10 × 4", x1-x0, y1-y0)
	}
	if _, ok := h.clusterOf(1, -1); ok {
		t.Fatal("a state without coordinates has a cluster")
	}
	if _, ok := h.clusterOf(3, 0); ok {
		t.Fatal("a cluster on a level the plane does not have")
	}
}

// eachCrossing visits every step of the map from a cluster of level 1 into another
func eachCrossing(m *testmap.Map, h *Hierarchy[int], visit func(a, b int, c, o int32)) {
	var buf []graph.Transition[int]
	for a := range m.W * m.H {
		if m.Share[a] == 0 {
			continue
		}
		c, _ := h.cluster1(a)
		buf = m.Forward(a, a, buf[:0])
		for _, st := range buf {
			if o, _ := h.cluster1(st.To); o != c {
				visit(a, st.To, c, o)
			}
		}
	}
}

// fillWithin is the cheapest from from to each state within cluster c of level 1, by Dijkstra
func fillWithin(h *Hierarchy[int], c int32, from int,
	steps graph.Transitions[int]) *dijkstra.Engine[int] {
	d := dijkstra.New[int](dict.NewMap[int](0), 0)
	d.Fill([]dijkstra.Source[int]{{At: from}}, steps, func(s int) bool {
		o, ok := h.cluster1(s)
		return ok && o == c
	}, nil)
	return d
}

func TestHierarchy_AnEntranceStandsOnEveryRun(t *testing.T) {
	// a crossing is kept in reach: from its border state to an entrance, through the twin, and
	// on to the state across, each within its cluster
	m := testmap.New(4, 64, 0.12, false)
	h := testHierarchy(m.Graph())
	h.Build(math.MaxInt)
	crossings := 0
	eachCrossing(m, h.Hierarchy, func(a, b int, c, o int32) {
		crossings++
		from := fillWithin(h.Hierarchy, c, a, m.Forward)
		to := fillWithin(h.Hierarchy, o, b, m.Backward)
		cl := &h.levels[0].clusters[c]
		for i := range cl.keys {
			if _, ok := from.Cost(cl.states[i]); !ok {
				continue
			}
			for _, tw := range cl.twins[cl.twinAt[i]:cl.twinAt[i+1]] {
				if _, on := to.Cost(tw.key); tw.other == o && tw.out != inf32 && on {
					return
				}
			}
		}
		t.Fatalf("the step from %d to %d across clusters %d and %d is kept by no entrance",
			a, b, c, o)
	})
	if crossings == 0 {
		t.Fatal("the scene has no border crossed")
	}

	// a uniform map with a wall across the border between clusters 0 and 1, at rows 5 to 7:
	// runs of rows 0..4 and 8..15, each with its ends and an entrance every span
	u := testmap.New(1, 64, 0, false)
	for i := range u.Share {
		u.Share[i] = 1
	}
	for y := 5; y <= 7; y++ {
		for x := 14; x <= 17; x++ {
			u.Share[y*64+x] = 0
		}
	}
	hu := testHierarchy(u.Graph())
	hu.Build(math.MaxInt)
	var rows []int
	cl := &hu.levels[0].clusters[0]
	for i, s := range cl.states {
		twins := cl.twins[cl.twinAt[i]:cl.twinAt[i+1]]
		if x, y, _ := u.Coords(s); x == 15 &&
			slices.ContainsFunc(twins, func(tw twin) bool { return tw.other == 1 }) {
			rows = append(rows, y)
		}
	}
	if want := []int{0, 4, 8, 12, 15}; !slices.Equal(rows, want) {
		t.Fatalf("the entrances into cluster 1 stand on rows %v, want %v", rows, want)
	}
}

func TestHierarchy_IntraCostsAreTheDijkstrasCosts(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	lv := &h.levels[0]
	for c := int32(0); c < int32(len(lv.clusters)); c += 7 {
		cl := &lv.clusters[c]
		n := len(cl.keys)
		for i := range n {
			d := fillWithin(h.Hierarchy, c, cl.states[i], m.Forward)
			for k := range n {
				want, ok := d.Cost(cl.states[k])
				got := cl.intra[i*n+k]
				if !ok {
					if got != inf32 {
						t.Fatalf("cluster %d: %d to %d costs %v, the dijkstra reaches none",
							c, i, k, got)
					}
					continue
				}
				if math.Abs(float64(got)-want) > 1e-6*want {
					t.Fatalf("cluster %d: %d to %d costs %v, the dijkstra's %v", c, i, k, got, want)
				}
				// the tree's path, from its end back to its start
				path := []int{cl.states[k]}
				for l := cl.local[k]; l != cl.local[i]; {
					l = cl.parent(int32(i), l)
					path = append(path, h.own.stateAt(c, l))
				}
				slices.Reverse(path)
				cost, ok := testmap.WalkCost(m.Forward, path)
				if !ok || math.Abs(cost-want) > 1e-6*want {
					t.Fatalf("cluster %d: the tree's path from %d to %d costs %v (%v), want %v",
						c, i, k, cost, ok, want)
				}
			}
		}
	}
}

// oneWay refuses the steps leftward across x == 16 on m: no way back into cluster 0's column
func oneWay(m *testmap.Map) graph.Implicit[int] {
	g := m.Graph()
	left := func(from, to int) bool { return from%m.W >= 16 && to%m.W < 16 }
	g.Forward = func(from, prev int, buf []graph.Transition[int]) []graph.Transition[int] {
		return slices.DeleteFunc(m.Forward(from, prev, buf), func(st graph.Transition[int]) bool {
			return left(from, st.To)
		})
	}
	g.Layout.Backward = func(to, next int, buf []graph.Transition[int]) []graph.Transition[int] {
		return slices.DeleteFunc(m.Backward(to, next, buf), func(st graph.Transition[int]) bool {
			return left(st.To, to)
		})
	}
	return g
}

func TestHierarchy_TwinEdgesAreDirected(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	cost := func(steps graph.Transitions[int], a, b int) float32 {
		for _, st := range steps(a, a, nil) {
			if st.To == b {
				return float32(st.Cost)
			}
		}
		return inf32
	}
	lopsided := 0
	for c := range h.levels[0].clusters {
		cl := &h.levels[0].clusters[c]
		for i, a := range cl.states {
			for _, tw := range cl.twins[cl.twinAt[i]:cl.twinAt[i+1]] {
				out, in := cost(m.Forward, a, tw.key), cost(m.Forward, tw.key, a)
				if tw.out != out || tw.in != in {
					t.Fatalf("the twin %d-%d steps there at %v and back at %v; the map's %v, %v",
						a, tw.key, tw.out, tw.in, out, in)
				}
				if out != in {
					lopsided++
				}
			}
		}
	}
	if lopsided == 0 {
		t.Fatal("every twin costs the same both ways on a map priced by the cell entered")
	}

	// no way back leftward across x == 16: the twins there lead one way, and so do the routes
	o := testmap.New(2, 64, 0.05, false)
	g := oneWay(o)
	ho := testHierarchy(g)
	ho.Build(math.MaxInt)
	oneway := 0
	for c := range ho.levels[0].clusters {
		cl := &ho.levels[0].clusters[c]
		for i, a := range cl.states {
			for _, tw := range cl.twins[cl.twinAt[i]:cl.twinAt[i+1]] {
				if a%64 == 15 && tw.key%64 == 16 {
					oneway++
					if tw.out == inf32 || tw.in != inf32 {
						t.Fatalf("the twin %d-%d across the one-way border: %v there, %v back",
							a, tw.key, tw.out, tw.in)
					}
				}
			}
		}
	}
	if oneway == 0 {
		t.Fatal("no twin across the one-way border")
	}
	dst := make([]int, 64*64)
	for _, p := range o.Pairs(3, 40, 24, 90) {
		a, b := p[0], p[1]
		if a%64 >= 16 || b%64 < 16 {
			continue
		}
		n, ok := ho.own.find(a, b, dst)
		if _, walk := testmap.WalkCost(g.Forward, dst[:n]); ok && (!walk || dst[n-1] != b) {
			t.Fatalf("the route from %d to %d over the one-way border does not walk", a, b)
		}
		if _, back := ho.own.find(b, a, dst); back {
			t.Fatalf("a route from %d back to %d over the one-way border", b, a)
		}
	}
}

func TestHierarchy_ALevelAboveIsBuiltOverTheLevelBelow(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	up, dn := &h.levels[1], &h.levels[0]
	for c := range int32(len(up.clusters)) {
		cl := &up.clusters[c]
		n := len(cl.keys)
		for i, r := range cl.down {
			if dn.clusters[r.c].keys[r.i] != cl.keys[i] || dn.parent[r.c] != c {
				t.Fatalf("cluster %d's node %d is no node of its children", c, i)
			}
			if slices.ContainsFunc(cl.twins[cl.twinAt[i]:cl.twinAt[i+1]],
				func(tw twin) bool { return tw.other == c }) {
				t.Fatalf("cluster %d's node %d has a twin within it", c, i)
			}
		}
		for i := range n {
			d := dijkstra.New[nodeRef](dict.NewMap[nodeRef](0), 0)
			d.Fill([]dijkstra.Source[nodeRef]{{At: cl.down[i]}},
				func(from, _ nodeRef, buf []graph.Transition[nodeRef]) []graph.Transition[nodeRef] {
					return h.edgesOut(0, from, buf)
				}, func(r nodeRef) bool { return dn.parent[r.c] == c }, nil)
			base := dijkstra.New[int](dict.NewMap[int](0), 0)
			x0, y0 := int(c%up.cols)*64, int(c/up.cols)*64
			base.Fill([]dijkstra.Source[int]{{At: cl.states[i]}}, m.Forward, func(s int) bool {
				x, y, _ := m.Coords(s)
				return x >= x0 && y >= y0 && x < x0+64 && y < y0+64
			}, nil)
			for k := range n {
				want, ok := d.Cost(cl.down[k])
				got := float64(cl.intra[i*n+k])
				if !ok && !math.IsInf(got, 1) || ok && math.Abs(got-want) > 1e-5*want {
					t.Fatalf("cluster %d: %d to %d costs %v; over the level below %v (%v)",
						c, i, k, got, want, ok)
				}
				if least, _ := base.Cost(cl.states[k]); ok && got < least*(1-1e-6) {
					t.Fatalf("cluster %d: %d to %d costs %v, below the cheapest within it, %v",
						c, i, k, got, least)
				}
			}
		}
	}
}

// boundByAStar checks the climb's routes against A*'s over pairs: walkable from from to to, each
// at most worst times the cheapest and on average at most 1.04; it reports the ratios
func boundByAStar(t *testing.T, m *testmap.Map, h asked[int], g graph.Implicit[int], pairs [][2]int,
	worst float64) (mean, most float64) {
	t.Helper()
	exact := astar.New[int](g.Heuristic, dict.NewIndexedSlice(m.W*m.H, testmap.Identity), 0)
	dst := make([]int, m.W*m.H)
	found := 0
	var path []int
	for _, p := range pairs {
		a, b := p[0], p[1]
		path = exact.SolveInto(path, a, b, g.Forward)
		n, ok := h.own.find(a, b, dst)
		if len(path) == 0 {
			if ok {
				t.Fatalf("a route from %d to %d A* finds unreachable", a, b)
			}
			continue
		}
		if !ok {
			t.Fatalf("no route from %d to %d; A*'s has %d states", a, b, len(path))
		}
		route := dst[:n]
		got, walk := testmap.WalkCost(g.Forward, route)
		if !walk || route[0] != a || route[n-1] != b {
			t.Fatalf("the route from %d to %d does not walk from one to the other", a, b)
		}
		if abs, _ := h.own.cost(a, b); math.Abs(abs-got) > 1e-4*got {
			t.Fatalf("the route from %d to %d costs %v, its Cost %v", a, b, got, abs)
		}
		want, _ := testmap.WalkCost(g.Forward, path)
		r := got / want
		if r > worst || r < 1-1e-9 {
			t.Fatalf("the route from %d to %d costs %.4f of the cheapest, want at most %v",
				a, b, r, worst)
		}
		mean, most = mean+r, max(most, r)
		found++
	}
	if found == 0 {
		t.Fatal("no pair reachable")
	}
	if mean /= float64(found); mean > 1.04 {
		t.Fatalf("the routes cost %.4f of the cheapest on average, want at most 1.04", mean)
	}
	return mean, most
}

// shareCluster says whether a and b lie in one cluster of level 1
func shareCluster[T comparable](h *Hierarchy[T], a, b T) bool {
	ca, _ := h.cluster1(a)
	cb, _ := h.cluster1(b)
	return ca == cb
}

func TestHierarchy_RoutesCostWithinABoundOfAStar(t *testing.T) {
	for _, c := range []struct {
		name        string
		seed        uint64
		size, pairs int
		far, worst  float64
		opts        []option
		levels      int
	}{
		{"one level", 3, 128, 100, math.Inf(1), 1.10, []option{withTop(64)}, 1},
		{"256", 1, 256, 200, math.Inf(1), 1.15, nil, 2},
		{"250, the last clusters thinner", 5, 250, 100, math.Inf(1), 1.15, nil, 2},
		{"1024", 2, 1024, 40, 300, 1.15, nil, 3},
	} {
		t.Run(c.name, func(t *testing.T) {
			var m *testmap.Map
			var h asked[int]
			if c.opts == nil {
				m, h = builtOver(c.seed, c.size, 0.1, false)
			} else {
				m = testmap.New(c.seed, c.size, 0.1, false)
				h = testHierarchy(m.Graph(), c.opts...)
				h.Build(math.MaxInt)
			}
			if len(h.levels) != c.levels {
				t.Fatalf("%d levels, want %d", len(h.levels), c.levels)
			}
			pairs := m.Pairs(c.seed+10, c.pairs, 64, c.far)
			mean, most := boundByAStar(t, m, h, m.Graph(), pairs, c.worst)
			t.Logf("%d²: levels %d, clusters %v, nodes %v, %.1f MB (trees %.1f MB); "+
				"cost ratio mean %.4f, worst %.4f", c.size, len(h.levels),
				clustersByLevel(h.Hierarchy), nodesByLevel(h.Hierarchy),
				float64(bytesOf(h.Hierarchy))/(1<<20), float64(h.trees)/(1<<20), mean, most)
		})
	}
}

func clustersByLevel[T comparable](h *Hierarchy[T]) []int {
	counts := make([]int, len(h.levels))
	for j := range counts {
		counts[j] = h.clusters(j + 1)
	}
	return counts
}

func TestHierarchy_ARouteIsWalkable(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	dst := make([]int, m.W*m.H)
	for _, p := range m.Pairs(21, 300, 20, math.Inf(1)) {
		n, ok := h.own.find(p[0], p[1], dst)
		if !ok {
			continue
		}
		if dst[0] != p[0] || dst[n-1] != p[1] {
			t.Fatalf("the route from %d to %d runs from %d to %d", p[0], p[1], dst[0], dst[n-1])
		}
		if _, walk := testmap.WalkCost(m.Forward, dst[:n]); !walk {
			t.Fatalf("the route from %d to %d takes a step the map has not", p[0], p[1])
		}
	}
}

func TestHierarchy_FindFillsDstLazily(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	bare := testHierarchy(m.Graph(), withTreeMemory(0))
	bare.Build(math.MaxInt)
	pairs := m.Pairs(5, 10, 180, math.Inf(1))
	full, short := make([]int, m.W*m.H), make([]int, 8)
	for _, p := range pairs {
		n, ok := h.own.find(p[0], p[1], full)
		if !ok {
			continue
		}
		cost := func(h asked[int], dst []int) (uint64, int) {
			before := h.own.Expanded()
			k, _ := h.own.find(p[0], p[1], dst)
			return h.own.Expanded() - before, k
		}
		whole, _ := cost(h, full)
		part, k := cost(h, short)
		if k != len(short) || !slices.Equal(short, full[:k]) {
			t.Fatalf("%v: %d states written into 8, %v; the route begins %v", p, k, short, full[:8])
		}
		if part != whole {
			t.Fatalf("%v: %d expansions for 8 states, %d for the whole route; a tree is walked",
				p, part, whole)
		}
		// without trees each leg written is searched, the ones past dst are not
		whole, n0 := cost(bare, full)
		part, _ = cost(bare, short)
		if n0 != n || part >= whole {
			t.Fatalf("%v: no trees, %d expansions for 8 states, %d for all %d", p, part, whole, n0)
		}
	}
}

func TestHierarchy_AReaskContinuesTheRoute(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	exact := mapAStar(m)
	whole, chunk := make([]int, m.W*m.H), make([]int, 16)
	for _, p := range m.Pairs(7, 30, 100, math.Inf(1)) {
		a, b := p[0], p[1]
		n, ok := h.own.find(a, b, whole)
		if !ok {
			continue
		}
		route := []int{a}
		for from := a; ; {
			k, ok := h.own.find(from, b, chunk)
			if !ok {
				// near its goal a route is A*'s
				route = append(route, exact.SolveInto(nil, from, b, m.Forward)[1:]...)
				break
			}
			if chunk[0] != from {
				t.Fatalf("%v: a re-ask from %d starts at %d", p, from, chunk[0])
			}
			if from == a && !slices.Equal(chunk[:k], whole[:min(n, k)]) {
				t.Fatalf("%v: the first piece %v, the route begins %v", p, chunk[:k], whole[:k])
			}
			route = append(route, chunk[1:k]...)
			if chunk[k-1] == b {
				break
			}
			from = chunk[k-1]
			if len(route) > 4*n {
				t.Fatalf("%v: the pieces run on past %d states", p, len(route))
			}
		}
		got, walk := testmap.WalkCost(m.Forward, route)
		want, _ := testmap.WalkCost(m.Forward, exact.SolveInto(nil, a, b, m.Forward))
		if !walk || route[len(route)-1] != b || got > 1.15*want {
			t.Fatalf("%v: the route in pieces walks %v to %d at %.4f of the cheapest",
				p, walk, route[len(route)-1], got/want)
		}
	}
}

// wallIn turns the cells of m from x0, y0 to x1, y1 into walls, a row or a column
func wallIn(m *testmap.Map, x0, y0, x1, y1 int) {
	for y := y0; y <= y1; y++ {
		for x := x0; x <= x1; x++ {
			m.Share[y*m.W+x] = 0
		}
	}
}

func TestHierarchy_ReachesAgreesWithAStar(t *testing.T) {
	m := testmap.New(5, 96, 0.08, false)
	// an island across four clusters, a U within one, open only up, a cell walled in
	wallIn(m, 24, 24, 40, 24)
	wallIn(m, 24, 40, 40, 40)
	wallIn(m, 24, 24, 24, 40)
	wallIn(m, 40, 24, 40, 40)
	wallIn(m, 66, 66, 66, 76)
	wallIn(m, 76, 66, 76, 76)
	wallIn(m, 66, 76, 76, 76)
	wallIn(m, 4, 84, 6, 84)
	wallIn(m, 4, 86, 6, 86)
	wallIn(m, 4, 85, 4, 85)
	wallIn(m, 6, 85, 6, 85)
	m.Share[85*96+5] = 1
	h := testHierarchy(m.Graph())
	h.Build(math.MaxInt)
	exact := mapAStar(m)
	pairs := m.Pairs(9, 600, 0, math.Inf(1))
	for _, special := range [][2]int{{32*96 + 32, 10}, {10, 32*96 + 32}, {70*96 + 70, 5},
		{85*96 + 5, 90*96 + 90}, {90*96 + 90, 85*96 + 5}, {85*96 + 5, 85*96 + 5},
		{70*96 + 70, 71*96 + 71}, {32*96 + 32, 33*96 + 34}} {
		pairs = append(pairs, special)
	}
	unreachable := 0
	for _, p := range pairs {
		want := exact.SolveInto(nil, p[0], p[1], m.Forward) != nil
		if got := h.own.Reaches(p[0], p[1]); got != want {
			t.Fatalf("reaches(%d, %d) %v, A* %v", p[0], p[1], got, want)
		}
		if !want {
			unreachable++
		}
	}
	if unreachable == 0 {
		t.Fatal("no pair unreachable in the scene")
	}
	if !testHierarchy(m.Graph()).own.Reaches(32*96+32, 10) {
		t.Fatal("unbuilt, reaches says no")
	}
}

func TestHierarchy_RoutesAreRefusedForAReason(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	var a, b int
	for _, p := range m.Pairs(13, 1000, 3, 12) {
		if shareCluster(h.Hierarchy, p[0], p[1]) &&
			mapAStar(m).SolveInto(nil, p[0], p[1], m.Forward) != nil {
			a, b = p[0], p[1]
			break
		}
	}
	if v := h.own.Route(a, b); v != TooNear {
		t.Fatalf("a route from %d to %d within one cluster: %v, want too near", a, b, v)
	}
	if _, ok := h.own.cost(a, b); ok {
		t.Fatalf("a cost from %d to %d within one cluster", a, b)
	}
	if !h.own.Reaches(a, b) {
		t.Fatalf("%d does not reach %d within its cluster", a, b)
	}
	far := m.Pairs(14, 1, 100, math.Inf(1))[0]
	if v := h.own.Route(-1, far[1]); v != NoCluster {
		t.Fatalf("a route from a state without coordinates: %v, want no cluster", v)
	}
	if v := h.own.Route(far[0], -1); v != NoCluster {
		t.Fatalf("a route to a state without coordinates: %v, want no cluster", v)
	}
	if v := testHierarchy(m.Graph()).own.Route(far[0], far[1]); v != NotReady {
		t.Fatalf("a route before anything is built: %v, want not ready", v)
	}
	h.Stale(far[0])
	if v := h.own.Route(far[0], far[1]); v != NotReady {
		t.Fatalf("a route while a cluster is stale: %v, want not ready", v)
	}
	h.Build(math.MaxInt)
	if v := h.own.Route(far[0], far[1]); v != Climbed {
		t.Fatalf("no route once built again: %v", v)
	}
}

// sameGraph fails t where h and o differ in a cluster
func sameGraph[T comparable](t *testing.T, h, o *Hierarchy[T]) {
	t.Helper()
	if diff := h.Diff(o); diff != "" {
		t.Fatal(diff)
	}
}

// built is what h built so far: by level, the clusters built whole and those filled again
func built[T comparable](h *Hierarchy[T]) (whole, costs []uint64) {
	return slices.Clone(h.rebuilds), slices.Clone(h.refills)
}

// since is what h built after it had built whole and costs
func since[T comparable](h *Hierarchy[T], whole, costs []uint64) (w, c []uint64) {
	w, c = built(h)
	for j := range w {
		w[j] -= whole[j]
		c[j] -= costs[j]
	}
	return w, c
}

func TestHierarchy_AChangedStateRebuildsItsClusterAndTheOnesAcross(t *testing.T) {
	m := testmap.New(4, 128, 0.05, false)
	h := testHierarchy(m.Graph())
	h.Build(math.MaxInt)
	// a cell on the border of clusters 0 and 1, away from their corners, walled: the entrances
	// there change, so the cluster above is built again whole
	s := 8*128 + 15
	m.Share[s] = 0
	h.Stale(s)
	var stale []int
	for c, st := range h.levels[0].stale {
		if st != fresh {
			stale = append(stale, c)
		}
	}
	if !slices.Equal(stale, []int{0, 1}) || h.levels[1].stales != 0 {
		t.Fatalf("stale: %v on level 1, %d on level 2; want clusters 0 and 1, none above till "+
			"they are built", stale, h.levels[1].stales)
	}
	whole, costs := built(h.Hierarchy)
	h.Build(math.MaxInt)
	if w, c := since(h.Hierarchy, whole, costs); !slices.Equal(w, []uint64{2, 1}) ||
		!slices.Equal(c, []uint64{0, 0}) {
		t.Fatalf("built again whole %v, filled again %v; want the two across the border and the "+
			"one above whole", w, c)
	}
	fresh := testHierarchy(m.Graph())
	fresh.Build(math.MaxInt)
	sameGraph(t, h.Hierarchy, fresh.Hierarchy)

	// a cell within a cluster: that cluster, its costs between its entrances changed, so the one
	// above fills its costs again
	in := 40*128 + 40
	m.Share[in] = 0.5
	h.Stale(in)
	whole, costs = built(h.Hierarchy)
	h.Build(math.MaxInt)
	if w, c := since(h.Hierarchy, whole, costs); !slices.Equal(w, []uint64{1, 0}) ||
		!slices.Equal(c, []uint64{0, 1}) {
		t.Fatalf("built again whole %v, filled again %v; want the cell's cluster whole and the "+
			"one above filled again", w, c)
	}
	fresh = testHierarchy(m.Graph())
	fresh.Build(math.MaxInt)
	sameGraph(t, h.Hierarchy, fresh.Hierarchy)
}

func TestHierarchy_BuildKeepsToItsBudget(t *testing.T) {
	m := testmap.New(4, 128, 0.05, false)
	h := testHierarchy(m.Graph())
	total := h.clusters(1) + h.clusters(2)
	// a budget is spent a step at a time: a call stops on the step that spends it, and the next
	// goes on with the job under way; level 1 first, then the labels, then level 2
	const budget = 3000
	sum, calls := 0, 0
	for !h.Ready() {
		built, left := h.Build(budget)
		calls++
		if h.worked < 1 || h.worked-h.stepWorked >= budget || h.stepWorked < 1 {
			t.Fatalf("a build of %d did %d, its last step %d", budget, h.worked, h.stepWorked)
		}
		if h.rebuilds[1] > 0 && (h.rebuilds[0] != uint64(h.clusters(1)) || h.labels.made == 0) {
			t.Fatal("a cluster of level 2 built before level 1 and its labels are")
		}
		sum += built
		if stale := h.levels[0].stales + h.levels[1].stales; left != stale &&
			left != stale+1 || left == stale+1 && !h.labels.pending() {
			t.Fatalf("%d left with %d clusters stale, labels pending %v", left, stale,
				h.labels.pending())
		}
	}
	if sum != total || h.builds != uint64(total) || calls <= total {
		t.Fatalf("%d built over %d calls, %d counted; want %d, a cluster over several calls",
			sum, calls, h.builds, total)
	}
	// every cluster of level 1 built again on a graph that did not change: level 2 is left as
	// it is, and so are the labels
	h.StaleAll()
	if h.Ready() || h.levels[0].stales != h.clusters(1) || h.levels[1].stales != 0 {
		t.Fatal("staleAll left a cluster of level 1 built, or marked one above")
	}
	if built, left := h.Build(0); built != h.clusters(1) || left != 0 || h.labels.made != 1 ||
		h.labels.local != 0 || h.rebuilds[1]+h.refills[1] != uint64(h.clusters(2)) {
		t.Fatalf("built %d, %d left, labelled %d and %d times, level 2 built %d and filled %d "+
			"times; want level 1 alone", built, left, h.labels.made, h.labels.local,
			h.rebuilds[1], h.refills[1])
	}
}

func TestHierarchy_TheSameGraphWhicheverOrderItIsBuiltIn(t *testing.T) {
	m := testmap.New(6, 128, 0.1, false)
	whole := testHierarchy(m.Graph())
	whole.Build(math.MaxInt)
	piece := testHierarchy(m.Graph())
	for {
		if _, left := piece.Build(1); left == 0 {
			break
		}
	}
	sameGraph(t, whole.Hierarchy, piece.Hierarchy)
	again := testHierarchy(m.Graph())
	again.Build(math.MaxInt)
	rnd := testmap.Splitmix(3)
	for range 40 {
		again.Stale(int(rnd() * float64(m.W*m.H)))
		if rnd() < 0.3 {
			again.Build(int(rnd()*5) + 1)
		}
	}
	again.Build(math.MaxInt)
	sameGraph(t, whole.Hierarchy, again.Hierarchy)
	a, b := make([]int, m.W*m.H), make([]int, m.W*m.H)
	for _, p := range m.Pairs(4, 50, 40, math.Inf(1)) {
		n, ok := whole.own.find(p[0], p[1], a)
		k, ok2 := again.own.find(p[0], p[1], b)
		if ok != ok2 || !slices.Equal(a[:n], b[:k]) {
			t.Fatalf("%v: the routes of two builds of one graph differ", p)
		}
	}
}

func TestHierarchy_TreesOverTheirMemorySearchTheirLegs(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	if h.trees == 0 {
		t.Fatal("no trees kept under the default memory")
	}
	for _, trees := range []int{0, h.trees / 3} {
		o := testHierarchy(m.Graph(), withTreeMemory(trees))
		o.Build(math.MaxInt)
		if o.trees > trees {
			t.Fatalf("%d bytes of trees kept under %d", o.trees, trees)
		}
		a, b := make([]int, m.W*m.H), make([]int, m.W*m.H)
		for _, p := range m.Pairs(15, 60, 64, math.Inf(1)) {
			n, ok := h.own.find(p[0], p[1], a)
			k, ok2 := o.own.find(p[0], p[1], b)
			if ok != ok2 || !slices.Equal(a[:n], b[:k]) {
				t.Fatalf("%d bytes of trees: the route %v differs from the one walked", trees, p)
			}
		}
	}
}

func TestHierarchy_AClimbAllocatesNothingOnceBuilt(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	bare := testHierarchy(m.Graph(), withTreeMemory(0))
	bare.Build(math.MaxInt)
	pairs := m.Pairs(17, 20, 0, math.Inf(1))
	dst, short := make([]int, m.W*m.H), make([]int, 8)
	for _, h := range []asked[int]{h, bare} {
		finds := func() {
			for _, p := range pairs {
				h.own.find(p[0], p[1], dst)
				h.own.find(p[0], p[1], short)
				h.own.cost(p[0], p[1])
				h.own.Reaches(p[0], p[1])
			}
		}
		finds()
		if allocs := testing.AllocsPerRun(5, finds); allocs != 0 {
			t.Fatalf("find, cost and reaches allocate %v times once warmed up", allocs)
		}
	}
}

func TestHierarchy_BuildAllocatesNothingTwice(t *testing.T) {
	m := testmap.New(8, 128, 0.1, false)
	h := testHierarchy(m.Graph())
	h.Build(math.MaxInt)
	// a cell on the border of clusters 0 and 1 walled and opened in turn: every cluster of level
	// 1 built again, the one above whole and the labels of the two changed
	gate := 8*128 + 15
	share := m.Share[gate]
	rebuild := func() {
		if m.Share[gate] == 0 {
			m.Share[gate] = share
		} else {
			m.Share[gate] = 0
		}
		for s := range m.W * m.H {
			h.Stale(s)
		}
		if built, _ := h.Build(math.MaxInt); built < h.clusters(1)+1 {
			t.Fatalf("%d clusters built again of %d", built, h.clusters(1)+1)
		}
	}
	rebuild()
	rebuild()
	if allocs := testing.AllocsPerRun(2, rebuild); allocs != 0 {
		t.Fatalf("a build of every cluster allocates %v times the second time", allocs)
	}
	if h.labels.local == 0 {
		t.Fatal("the labels of the two clusters changed never worked out alone")
	}
}

// hexes walks m's cells as a parallelogram of axial hexagons: six neighbours each, a step
// priced as m prices its own
func hexes(m *testmap.Map) graph.Implicit[int] {
	dirs := [6][2]int{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, -1}, {-1, 1}}
	steps := func(from int, back bool, buf []graph.Transition[int]) []graph.Transition[int] {
		buf = buf[:0]
		q, r := from%m.W, from/m.W
		if back && m.Share[from] == 0 {
			return buf
		}
		for _, d := range dirs {
			nq, nr := q+d[0], r+d[1]
			if nq < 0 || nr < 0 || nq >= m.W || nr >= m.H || m.Share[nr*m.W+nq] == 0 {
				continue
			}
			n, entered := nr*m.W+nq, nr*m.W+nq
			if back {
				entered = from
			}
			buf = append(buf, graph.Transition[int]{To: n, Cost: 1 / float64(m.Share[entered])})
		}
		return buf
	}
	g := m.Graph()
	g.Forward = func(from, _ int, buf []graph.Transition[int]) []graph.Transition[int] {
		return steps(from, false, buf)
	}
	g.Layout.Backward = func(to, _ int, buf []graph.Transition[int]) []graph.Transition[int] {
		return steps(to, true, buf)
	}
	g.Heuristic = func(a, b int) float64 {
		dq, dr := a%m.W-b%m.W, a/m.W-b/m.W
		return float64(max(absInt(dq), absInt(dr), absInt(dq+dr)))
	}
	return g
}

func absInt(x int) int { return max(x, -x) }

func TestHierarchy_RoutesOnAHexGrid(t *testing.T) {
	m := testmap.New(11, 160, 0.08, false)
	g := hexes(m)
	h := testHierarchy(g)
	h.Build(math.MaxInt)
	mean, most := boundByAStar(t, m, h, g, m.Pairs(12, 80, 48, math.Inf(1)), 1.15)
	t.Logf("hex 160²: levels %d, nodes %v; cost ratio mean %.4f, worst %.4f",
		len(h.levels), nodesByLevel(h.Hierarchy), mean, most)
}

func TestHierarchy_RoutesOnATorus(t *testing.T) {
	m := testmap.New(12, 192, 0.08, true)
	h := testHierarchy(m.Graph())
	h.Build(math.MaxInt)
	seams := 0
	for _, tw := range h.levels[0].clusters[0].twins {
		cols := h.levels[0].cols
		if c := tw.other; c == cols-1 || c == int32(h.clusters(1))-cols {
			seams++
		}
	}
	if seams == 0 {
		t.Fatal("no twin across the seams of the torus")
	}
	mean, most := boundByAStar(t, m, h, m.Graph(), m.Pairs(13, 80, 48, math.Inf(1)), 1.15)
	t.Logf("torus 192²: levels %d, nodes %v; cost ratio mean %.4f, worst %.4f",
		len(h.levels), nodesByLevel(h.Hierarchy), mean, most)
}

// decked is m with a deck over the cells of row y from x0 to x1: a second state at each place
// there, keyed past the cells, stepping only along the deck and onto the ground at its ends;
// and a goal of its own for each cell, keyed past the decks, reached from the cell for nothing,
// at the cell's place but not among its states
type decked struct {
	m          *testmap.Map
	y, x0, x1  int
	deck, goal int // the first key of a deck, of a goal
}

func newDecked(m *testmap.Map, y, x0, x1 int) *decked {
	return &decked{m: m, y: y, x0: x0, x1: x1, deck: m.W * m.H, goal: 2 * m.W * m.H}
}

func (d *decked) coords(s int) (int, int, bool) {
	if s >= d.goal {
		s -= d.goal
	} else if s >= d.deck {
		s -= d.deck
	}
	return d.m.Coords(s)
}

func (d *decked) at(x, y int, buf []int) []int {
	buf = d.m.States(x, y, buf)
	if y == d.y && x >= d.x0 && x <= d.x1 {
		buf = append(buf, d.deck+y*d.m.W+x)
	}
	return buf
}

// steps lists every step of the decked map as from, to and cost
func (d *decked) steps(visit func(from, to int, cost float64)) {
	var buf []graph.Transition[int]
	for c := range d.m.W * d.m.H {
		for _, st := range d.m.Forward(c, c, buf[:0]) {
			visit(c, st.To, st.Cost)
		}
		visit(c, d.goal+c, 0)
	}
	row := d.y * d.m.W
	for x := d.x0; x < d.x1; x++ {
		visit(d.deck+row+x, d.deck+row+x+1, 1)
		visit(d.deck+row+x+1, d.deck+row+x, 1)
	}
	for _, end := range [][2]int{{d.x0, d.x0 - 1}, {d.x1, d.x1 + 1}} {
		visit(d.deck+row+end[0], row+end[1], 1)
		visit(row+end[1], d.deck+row+end[0], 1)
	}
}

func (d *decked) graph() graph.Implicit[int] {
	out, in := map[int][]graph.Transition[int]{}, map[int][]graph.Transition[int]{}
	d.steps(func(from, to int, cost float64) {
		if from < d.deck && d.m.Share[from] == 0 {
			return
		}
		out[from] = append(out[from], graph.Transition[int]{To: to, Cost: cost})
		in[to] = append(in[to], graph.Transition[int]{To: from, Cost: cost})
	})
	return graph.Implicit[int]{
		Forward: func(from, _ int, buf []graph.Transition[int]) []graph.Transition[int] {
			return append(buf[:0], out[from]...)
		},
		Heuristic: func(a, b int) float64 {
			ax, ay, _ := d.coords(a)
			bx, by, _ := d.coords(b)
			return d.m.Octile(ay*d.m.W+ax, by*d.m.W+bx)
		},
		Index: testmap.Identity,
		Size:  3 * d.m.W * d.m.H,
		Layout: &graph.Layout[int]{Coords: d.coords, At: d.at, Width: d.m.W, Height: d.m.H,
			Backward: func(to, _ int, buf []graph.Transition[int]) []graph.Transition[int] {
				return append(buf[:0], in[to]...)
			}},
	}
}

func TestHierarchy_StatesSharingAPlaceAreRoutedApart(t *testing.T) {
	m := testmap.New(14, 96, 0.06, false)
	// a river down x 46..49, across the border of two clusters, forded through mud at row 20
	// and crossed over a deck above the ford
	wallIn(m, 46, 0, 49, 95)
	for x := 45; x <= 50; x++ {
		m.Share[20*96+x] = 0.15
	}
	m.Share[20*96+45], m.Share[20*96+50] = 1, 1
	d := newDecked(m, 20, 46, 49)
	g := d.graph()
	h := testHierarchy(g)
	h.Build(math.MaxInt)
	for _, x := range []int{47, 48} {
		c, _ := h.clusterOf(1, 20*96+x)
		if cl := &h.levels[0].clusters[c]; cl.layers != 2 || len(cl.tree16) == 0 {
			t.Fatalf("the cluster under the deck at %d: %d layers, %d trees of two bytes",
				x, cl.layers, len(cl.tree16))
		}
		cl := &h.levels[0].clusters[c]
		if !slices.ContainsFunc(cl.keys, func(k int) bool { return k >= d.deck }) {
			t.Fatalf("no entrance on the deck at %d", x)
		}
	}
	exact := astar.New[int](g.Heuristic, dict.NewIndexedMap(0, testmap.Identity), 0)
	dst := make([]int, 3*96*96)
	decks := 0
	for _, p := range m.Pairs(15, 120, 32, math.Inf(1)) {
		a, b := p[0], d.goal+p[1]
		path := exact.SolveInto(nil, a, b, g.Forward)
		n, ok := h.own.find(a, b, dst)
		reached := path != nil
		if ok != reached || h.own.Reaches(a, b) != reached {
			t.Fatalf("%d to the goal of %d: find %v, reaches %v; A* reaches it %v",
				a, p[1], ok, h.own.Reaches(a, b), reached)
		}
		if !ok {
			continue
		}
		got, walk := testmap.WalkCost(g.Forward, dst[:n])
		want, _ := testmap.WalkCost(g.Forward, path)
		if !walk || dst[n-1] != b || got > 1.15*want {
			t.Fatalf("%d to the goal of %d: walks %v to %d at %.4f of the cheapest",
				a, p[1], walk, dst[n-1], got/want)
		}
		if slices.ContainsFunc(dst[:n], func(s int) bool { return s >= d.deck && s < d.goal }) {
			decks++
		}
	}
	if decks == 0 {
		t.Fatal("no route over the deck")
	}
}

// openCell is the open cell of m at x, y; false where it is a wall or off the map
func openCell(m *testmap.Map, x, y int) (int, bool) {
	if c, ok := m.At(x, y); ok && x >= 0 && y >= 0 && x < m.W && y < m.H && m.Share[c] > 0 {
		return c, true
	}
	return 0, false
}

// refusedOrExact fails t where h finds a route from a to b dearer than A*'s, and reports
// whether h found one at all
func refusedOrExact(t *testing.T, m *testmap.Map, h asked[int], exact *astar.Engine[int], a, b int,
	dst []int) bool {
	t.Helper()
	n, ok := h.own.find(a, b, dst)
	if !ok {
		return false
	}
	got, walk := testmap.WalkCost(m.Forward, dst[:n])
	want, _ := testmap.WalkCost(m.Forward, exact.SolveInto(nil, a, b, m.Forward))
	if !walk || got > want*(1+1e-9) {
		ax, ay, _ := m.Coords(a)
		bx, by, _ := m.Coords(b)
		t.Fatalf("(%d,%d) to (%d,%d), %.1f apart: a route at %.3f, the cheapest %.3f",
			ax, ay, bx, by, m.Octile(a, b), got, want)
	}
	return true
}

func TestHierarchy_ANearPairAcrossAHighBorderIsRefusedOrExact(t *testing.T) {
	m, h := builtOver(2, 1024, 0.1, false)
	exact := mapAStar(m)
	dst := make([]int, m.W*m.H)
	pairs := 0
	// across the borders of levels 1, 2 and 3 along rows and down columns, from neighbours to
	// 31 apart; slantwise across their corners
	for _, x := range []int{16, 64, 256} {
		for y := 3; y < m.H; y += 41 {
			for gap := 1; gap < 32; gap += 2 {
				from, to := x-(gap+1)/2, x-(gap+1)/2+gap
				for _, ends := range [2][4]int{{from, y, to, y}, {y, from, y, to}} {
					a, okA := openCell(m, ends[0], ends[1])
					b, okB := openCell(m, ends[2], ends[3])
					if okA && okB {
						refusedOrExact(t, m, h, exact, a, b, dst)
						pairs++
					}
				}
			}
		}
		for d := 1; d <= 22; d += 3 {
			a, okA := openCell(m, x-(d+1)/2, x-(d+1)/2)
			b, okB := openCell(m, x-(d+1)/2+d, x-(d+1)/2+d)
			if okA && okB {
				refusedOrExact(t, m, h, exact, a, b, dst)
				pairs++
			}
		}
	}
	if pairs < 500 {
		t.Fatalf("%d near pairs asked; the scene wants more", pairs)
	}
}

func TestHierarchy_ANearPairAcrossTheSeamIsRefusedOrExact(t *testing.T) {
	m, h := builtOver(12, 192, 0.08, true)
	exact := mapAStar(m)
	dst := make([]int, m.W*m.H)
	pairs := 0
	for y := 1; y < m.H; y += 5 {
		for gap := 1; gap < 32; gap += 2 {
			from := m.W - (gap+1)/2
			to := (from + gap) % m.W
			for _, ends := range [2][4]int{{from, y, to, y}, {y, from, y, to}} {
				a, okA := openCell(m, ends[0], ends[1])
				b, okB := openCell(m, ends[2], ends[3])
				if okA && okB {
					refusedOrExact(t, m, h, exact, a, b, dst)
					pairs++
				}
			}
		}
	}
	if a, ok := openCell(m, m.W-1, m.H-1); ok {
		if b, ok := openCell(m, 0, 0); ok {
			refusedOrExact(t, m, h, exact, a, b, dst)
		}
	}
	if pairs < 300 {
		t.Fatalf("%d near pairs asked across the seams; the scene wants more", pairs)
	}
}

// searchedOn is the level, from 1, a climb searches the route from a to b on, 0 where it refuses
// it, checked against the level the climb took
func searchedOn(t *testing.T, s *Climb[int], a, b int, dst []int) int {
	t.Helper()
	level, v := s.levelFor(a, b)
	if v != Climbed {
		if _, found := s.find(a, b, dst); found {
			t.Fatalf("a route from %d to %d, which levelFor refuses", a, b)
		}
		return 0
	}
	s.find(a, b, dst)
	if s.q.level != level {
		t.Fatalf("from %d to %d: searched on level %d, levelFor says %d", a, b, s.q.level+1,
			level+1)
	}
	return level + 1
}

func TestHierarchy_TheLevelSearchedFollowsTheDistance(t *testing.T) {
	m, h := builtOver(2, 1024, 0.1, false)
	dst := make([]int, 64)
	// along a row from x as far as d: refused under 32, on level 1 under 128, on level 2 under
	// 512, and never above the highest level where the ends' clusters differ
	for _, c := range []struct{ x, d, want int }{
		{241, 31, 0}, {240, 32, 1}, {193, 127, 1}, {192, 128, 2}, {257, 511, 2}, {256, 512, 3},
		{260, 40, 1}, {280, 129, 2}, {200, 700, 3},
	} {
		asked := 0
		for y := 2; y < m.H && asked < 5; y += 13 {
			a, okA := openCell(m, c.x, y)
			b, okB := openCell(m, c.x+c.d, y)
			if !okA || !okB {
				continue
			}
			if got := searchedOn(t, h.own, a, b, dst); got != c.want {
				t.Fatalf("from (%d,%d) %d along the row: searched on level %d, want %d",
					c.x, y, c.d, got, c.want)
			}
			asked++
		}
		if asked == 0 {
			t.Fatalf("no open pair from x %d, %d apart", c.x, c.d)
		}
	}
	// WithNear moves the distances: 48 on level 1, 192 on level 2; 0 searches the highest
	// level the ends' clusters differ on, however near
	o := testmap.New(3, 256, 0, false)
	for _, c := range []struct {
		near        float64
		x, d, level int
	}{
		{48, 40, 47, 0}, {48, 40, 48, 1}, {48, 40, 191, 1}, {48, 40, 192, 2},
		{0, 63, 1, 2}, {0, 15, 1, 1},
	} {
		ho := testHierarchy(o.Graph(), withNear(c.near))
		ho.Build(math.MaxInt)
		a, _ := openCell(o, c.x, 7)
		b, _ := openCell(o, c.x+c.d, 7)
		if got := searchedOn(t, ho.own, a, b, dst); got != c.level {
			t.Fatalf("WithNear(%v), from x %d %d along: searched on level %d, want %d",
				c.near, c.x, c.d, got, c.level)
		}
	}
}

func TestHierarchy_ClosePairsCostWithinABound(t *testing.T) {
	m, h := builtOver(2, 1024, 0.1, false)
	rnd := testmap.Splitmix(31)
	var pairs [][2]int
	// pairs 32 to 128 apart across a border of each level in turn, along a row or a column
	for len(pairs) < 150 {
		side := []int{16, 64, 256}[len(pairs)%3]
		border := side * (1 + int(rnd()*float64(m.W/side-1)))
		across := 32 + int(rnd()*90)
		left := 1 + int(rnd()*float64(across-1))
		along, slant := int(rnd()*float64(m.H)), int((rnd()-0.5)*float64(across))
		ends := [4]int{border - left, along, border - left + across, along + slant}
		if rnd() < 0.5 {
			ends = [4]int{ends[1], ends[0], ends[3], ends[2]}
		}
		a, okA := openCell(m, ends[0], ends[1])
		b, okB := openCell(m, ends[2], ends[3])
		if d := m.Octile(a, b); okA && okB && d >= 32 && d <= 128 {
			pairs = append(pairs, [2]int{a, b})
		}
	}
	mean, most := boundByAStar(t, m, h, m.Graph(), pairs, 1.10)
	t.Logf("close pairs on 1024²: cost ratio mean %.4f, worst %.4f", mean, most)
}

func TestHierarchy_ClimbsAskAtOnce(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	bare := testHierarchy(m.Graph(), withTreeMemory(0))
	bare.Build(math.MaxInt)
	for _, h := range []asked[int]{h, bare} {
		climbsAskAtOnce(t, m, h)
	}
}

// climbsAskAtOnce has eight goroutines ask h through climbs of their own what h's own climb
// answers one pair after another
func climbsAskAtOnce(t *testing.T, m *testmap.Map, h asked[int]) {
	const goroutines, each = 8, 50
	pairs := m.Pairs(41, goroutines*each, 32, math.Inf(1))
	// what each pair gets asked of the Hierarchy's own Search, one after another
	type answer struct {
		route   []int
		found   bool
		cost    float64
		reached bool
	}
	ask := func(s *Climb[int], p [2]int, dst []int) answer {
		n, found := s.find(p[0], p[1], dst)
		cost, _ := s.cost(p[0], p[1])
		return answer{slices.Clone(dst[:n]), found, cost, s.Reaches(p[0], p[1])}
	}
	want := make([]answer, len(pairs))
	dst := make([]int, m.W*m.H)
	for i, p := range pairs {
		want[i] = ask(h.own, p, dst)
	}
	got := make([]answer, len(pairs))
	var wg sync.WaitGroup
	for g := range goroutines {
		wg.Add(1)
		go func() {
			defer wg.Done()
			// the generator's transitions through closures of the goroutine's own
			s := h.NewClimb(func(from, prev int,
				buf []graph.Transition[int]) []graph.Transition[int] {
				return m.Forward(from, prev, buf)
			}, func(to, next int, buf []graph.Transition[int]) []graph.Transition[int] {
				return m.Backward(to, next, buf)
			})
			dst := make([]int, m.W*m.H)
			// half the goroutines ask theirs the other way round
			for k := range each {
				i := g*each + k
				if g%2 == 1 {
					i = g*each + each - 1 - k
				}
				got[i] = ask(s, pairs[i], dst)
			}
		}()
	}
	wg.Wait()
	found := 0
	for i := range pairs {
		w, g := want[i], got[i]
		if w.found != g.found || w.cost != g.cost || w.reached != g.reached ||
			!slices.Equal(w.route, g.route) {
			t.Fatalf("%v: asked at once %v, %d states at %v, reached %v; one after another "+
				"%v, %d states at %v, reached %v", pairs[i], g.found, len(g.route), g.cost,
				g.reached, w.found, len(w.route), w.cost, w.reached)
		}
		if w.found {
			found++
		}
	}
	if found < len(pairs)/2 {
		t.Fatalf("%d routes found of %d pairs; the scene wants most", found, len(pairs))
	}
}

func TestHierarchy_ANewClimbAllocatesNothingOnceWarm(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	bare := testHierarchy(m.Graph(), withTreeMemory(0))
	bare.Build(math.MaxInt)
	pairs := m.Pairs(43, 20, 0, math.Inf(1))
	dst, short := make([]int, m.W*m.H), make([]int, 8)
	for _, h := range []asked[int]{h, bare} {
		s := h.NewClimb(m.Forward, m.Backward)
		asks := func() {
			for _, p := range pairs {
				s.find(p[0], p[1], dst)
				s.find(p[0], p[1], short)
				s.cost(p[0], p[1])
				s.Reaches(p[0], p[1])
			}
		}
		asks()
		if allocs := testing.AllocsPerRun(5, asks); allocs != 0 {
			t.Fatalf("a climb's find, cost and reaches allocate %v times once warm", allocs)
		}
	}
}

func TestHierarchy_AClimbCountsItsOwn(t *testing.T) {
	m, h := builtOver(1, 256, 0.1, false)
	p := m.Pairs(47, 1, 150, math.Inf(1))[0]
	expansions := h.own.Expanded()
	a, b := h.NewClimb(m.Forward, m.Backward), h.NewClimb(m.Forward, m.Backward)
	dst := make([]int, 64)
	for range 3 {
		a.find(p[0], p[1], dst)
	}
	b.find(p[0], p[1], dst)
	if b.Expanded() == 0 || a.Expanded() != 3*b.Expanded() {
		t.Fatalf("two climbs count %d and %d expansions, want three times the other's",
			a.Expanded(), b.Expanded())
	}
	if h.own.Expanded() != expansions {
		t.Fatal("the hierarchy's own climb counted another climb's expansions")
	}
}

// the labels' names, by cluster of level 1 and node, as the climbs read them
func namesOf[T comparable](h *Hierarchy[T]) [][]int32 {
	var names [][]int32
	for c := range h.levels[0].clusters {
		var row []int32
		for _, n := range h.levels[0].clusters[c].label {
			row = append(row, h.labels.alias[n])
		}
		names = append(names, row)
	}
	return names
}

func TestHierarchy_ReachesReadsLabelsBuilt(t *testing.T) {
	m := testmap.New(16, 128, 0.1, false)
	h := testHierarchy(m.Graph())
	if h.labels.made != 0 {
		t.Fatal("labels worked out before anything is built")
	}
	h.Build(math.MaxInt)
	if h.labels.made != 1 || h.labels.pending() {
		t.Fatalf("labels worked out %d times by the build, pending %v; want once and done",
			h.labels.made, h.labels.pending())
	}
	names := namesOf(h.Hierarchy)
	s := h.NewClimb(m.Forward, m.Backward)
	for _, p := range m.Pairs(49, 200, 0, math.Inf(1)) {
		h.own.Reaches(p[0], p[1])
		s.Reaches(p[0], p[1])
	}
	if h.labels.made != 1 || !slices.EqualFunc(names, namesOf(h.Hierarchy), slices.Equal) {
		t.Fatal("reaches wrote the labels")
	}
	// a changed cell on the corner of four clusters: nothing labelled until all four are built
	// again, then once
	c, _ := openCell(m, 47, 47)
	m.Share[c] = 0
	h.Stale(c)
	h.own.Reaches(c, 0)
	labelled := func() uint64 { return h.labels.made + h.labels.local }
	if labelled() != 1 {
		t.Fatal("reaches labelled while level 1 is stale")
	}
	h.Build(1)
	if labelled() != 1 {
		t.Fatal("labelled while level 1 is still stale")
	}
	h.Build(math.MaxInt)
	if labelled() != 2 {
		t.Fatalf("labels worked out %d times after the cell's build, want 2", labelled())
	}
}

// samePartition fails t where the labels of h and o part the nodes of level 1 otherwise: two
// nodes one names alike the other names apart
func samePartition[T comparable](t *testing.T, h, o *Hierarchy[T]) {
	t.Helper()
	there, back := map[int32]int32{}, map[int32]int32{}
	for c := range h.levels[0].clusters {
		a, b := &h.levels[0].clusters[c], &o.levels[0].clusters[c]
		for i := range a.keys {
			x, y := h.labels.alias[a.label[i]], o.labels.alias[b.label[i]]
			if w, ok := there[x]; ok && w != y {
				t.Fatalf("cluster %d's node %d: named with nodes the labels whole name apart", c, i)
			}
			if w, ok := back[y]; ok && w != x {
				t.Fatalf("cluster %d's node %d: named apart from nodes the labels whole join", c, i)
			}
			there[x], back[y] = y, x
		}
	}
}

// cells walled and opened at random, on the borders of clusters most often and a line at a time
// now and then, and the gate of a ring across clusters shut and opened, built under budgets drawn
// at random: the graph a whole build gives, its labels parting the nodes as a whole labelling
// does and telling apart only ends A* finds apart
func TestHierarchy_LabelsFollowEveryChange(t *testing.T) {
	for _, wrap := range []bool{false, true} {
		const size = 96
		m := testmap.New(21, size, 0.08, wrap)
		m.WallRing(20, 20, 30)
		gate := 20*size + 32
		g := m.Graph()
		h := testHierarchy(g)
		h.Build(0)
		exact := mapAStar(m)
		rnd := testmap.Splitmix(5)
		toggle := func(c int) {
			if m.Share[c] == 0 {
				m.Share[c] = float32(0.3 + 0.7*rnd())
			} else {
				m.Share[c] = 0
			}
			h.Stale(c)
		}
		for round := range 40 {
			if round%3 == 0 {
				toggle(gate)
			}
			for range 1 + int(rnd()*3) {
				x, y := int(rnd()*size), int(rnd()*size)
				if rnd() < 0.6 {
					x = x/16*16 + int(rnd()*2)*15
				}
				toggle(y*size + x)
				if rnd() < 0.2 {
					for k := 1; k < 6 && y+k < size; k++ {
						toggle((y+k)*size + x)
					}
				}
			}
			for calls := 0; !h.Ready(); calls++ {
				if calls > 100000 {
					t.Fatal("the build makes no progress")
				}
				h.Build(1 + int(rnd()*20000))
			}
			fresh := testHierarchy(g)
			fresh.Build(0)
			sameGraph(t, h.Hierarchy, fresh.Hierarchy)
			samePartition(t, h.Hierarchy, fresh.Hierarchy)
			for _, p := range m.Pairs(uint64(round), 30, 0, math.Inf(1)) {
				want := exact.SolveInto(nil, p[0], p[1], m.Forward) != nil
				if got := h.own.Reaches(p[0], p[1]); got != want {
					t.Fatalf("wrap %v, round %d: reaches(%d, %d) %v, A* %v", wrap, round, p[0],
						p[1], got, want)
				}
			}
		}
		if h.labels.local == 0 || h.labels.made < 2 {
			t.Fatalf("wrap %v: labelled alone %d times, whole %d; want both", wrap,
				h.labels.local, h.labels.made)
		}
		t.Logf("wrap %v: labelled alone %d times, whole %d", wrap, h.labels.local, h.labels.made)
	}
}
