package dijkstra

import (
	"math"
	"slices"
	"testing"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/astar"
	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/testmap"
)

// a zero heuristic: the engine expands states in the order a dijkstra settles them
func zeroAStar(d ...dict.Dict[testmap.Point]) *astar.Engine[testmap.Point] {
	return astar.New(func(testmap.Point, testmap.Point) float64 { return 0 }, dictOr(d), 0)
}

func newPointDijkstra(d ...dict.Dict[testmap.Point]) *Engine[testmap.Point] {
	return New(dictOr(d), 0)
}

func dictOr(d []dict.Dict[testmap.Point]) dict.Dict[testmap.Point] {
	if len(d) > 0 {
		return d[0]
	}
	return dict.NewMap[testmap.Point](defaultCapacity)
}

func sourceAt(p testmap.Point) []Source[testmap.Point] {
	return []Source[testmap.Point]{{At: p}}
}

// the states the last fill settled, in the order it settled them
func settledOf[T comparable](d *Engine[T]) []T {
	states := make([]T, len(d.settled))
	for i, n := range d.settled {
		states[i] = d.nodes[n].ID
	}
	return states
}

// the path the last fill keeps to s, from its source along the parents; nil where s was not
// settled
func treePath(d *Engine[testmap.Point], s testmap.Point) []testmap.Point {
	path := []testmap.Point{s}
	for len(path) <= 1<<20 {
		parent, ok := d.Parent(s)
		if !ok {
			return nil
		}
		if parent == s {
			slices.Reverse(path)
			return path
		}
		path, s = append(path, parent), parent
	}
	return nil
}

// the cells of grid, open or walls, a row at a time
func cells(grid [][]float64, open bool) []testmap.Point {
	var found []testmap.Point
	for y, row := range grid {
		for x, cost := range row {
			if (cost < testmap.InsurmountableObstacle) == open {
				found = append(found, testmap.Point{X: x, Y: y})
			}
		}
	}
	return found
}

func TestDijkstra_CostsAreTheEnginesWithAZeroHeuristic(t *testing.T) {
	const size = 20
	for seed := uint64(1); seed <= 3; seed++ {
		grid := testmap.RandomGrid(size, seed)
		transitions := testmap.EightWays(grid)
		open := cells(grid, true)
		a := zeroAStar()
		fills := map[string]*Engine[testmap.Point]{}
		for name, dict := range testmap.Dicts(size) {
			fills[name] = newPointDijkstra(dict())
		}
		for _, from := range []testmap.Point{open[0], open[len(open)/2], open[len(open)-1]} {
			want := map[testmap.Point]float64{}
			for _, p := range open {
				if path := a.SolveInto(nil, from, p, transitions); path != nil {
					want[p] = testmap.PathCost(grid, path)
				}
			}
			for name, d := range fills {
				d.Fill(sourceAt(from), transitions, nil, nil)
				settled := settledOf(d)
				if len(settled) != len(want) {
					t.Fatalf("seed %d %s from %v: %d states settled, the engine reaches %d",
						seed, name, from, len(settled), len(want))
				}
				for _, p := range settled {
					cost, ok := d.Cost(p)
					if solved, reached := want[p]; !ok || !reached || math.Abs(cost-solved) > 1e-9 {
						t.Fatalf("seed %d %s from %v: %v settled at %v (%v); the engine's %v (%v)",
							seed, name, from, p, cost, ok, solved, reached)
					}
					if got := testmap.PathCost(grid, treePath(d, p)); math.Abs(got-cost) > 1e-9 {
						t.Fatalf("seed %d %s from %v: the tree's path to %v costs %v, cost %v: %v",
							seed, name, from, p, got, cost, treePath(d, p))
					}
				}
			}
		}
	}
}

// the states a zero-heuristic engine expands searching from from to to, to last where it is
// reached, each with the path the engine holds to it
func engineExpansions(from, to testmap.Point,
	transitions graph.Transitions[testmap.Point]) ([]testmap.Point, [][]testmap.Point) {
	a := zeroAStar()
	var order []testmap.Point
	var paths [][]testmap.Point
	a.Start(from, to)
	for over := false; !over; {
		over = a.Run(to, transitions, 1)
		if path := a.ResultInto(nil); len(path) > 0 {
			order, paths = append(order, path[len(path)-1]), append(paths, path)
		}
	}
	return order, paths
}

func TestDijkstra_BreaksTiesAsTheEngine(t *testing.T) {
	uniform, walled := testmap.UniformGrid(24), testmap.WalledGrid(32)
	random := testmap.RandomGrid(24, 9)
	scenes := []struct {
		name        string
		size        int
		from        testmap.Point
		transitions graph.Transitions[testmap.Point]
	}{
		{"uniform24-4", 24, testmap.Point{X: 12, Y: 12}, testmap.FourWays(1.0, uniform)},
		{"uniform24-8", 24, testmap.Point{X: 12, Y: 12}, testmap.EightWays(uniform)},
		{"grid32-8", 32, testmap.Point{X: 16, Y: 16}, testmap.EightWays(walled)},
		{"random24-8", 24, testmap.Point{}, testmap.EightWays(random)},
	}
	for _, s := range scenes {
		far := testmap.Point{X: s.size - 1, Y: s.size - 1}
		// the far corner as the one target, then none: the engine's goal out of reach
		for _, to := range []testmap.Point{far, {X: -1, Y: -1}} {
			order, paths := engineExpansions(s.from, to, s.transitions)
			var targets []testmap.Point
			reached := 0
			if to == far {
				targets = []testmap.Point{far}
				if order[len(order)-1] == far {
					reached = 1
				}
			}
			for name, dict := range testmap.Dicts(s.size) {
				d := newPointDijkstra(dict())
				if got := d.Fill(sourceAt(s.from), s.transitions, nil, targets); got != reached {
					t.Fatalf("%s %s to %v: %d targets settled, want %d",
						s.name, name, targets, got, reached)
				}
				settled := settledOf(d)
				for i := range max(len(settled), len(order)) {
					if i >= len(settled) || i >= len(order) || settled[i] != order[i] {
						t.Fatalf("%s %s to %v: %d settled, the engine expanded %d; "+
							"they part at %d", s.name, name, targets, len(settled), len(order), i)
					}
					if got := treePath(d, settled[i]); !slices.Equal(got, paths[i]) {
						t.Fatalf("%s %s to %v: the tree's path to %v is %v, the engine's %v",
							s.name, name, targets, settled[i], got, paths[i])
					}
				}
			}
		}
	}
}

func TestDijkstra_StopsOnceTheTargetsAreSettled(t *testing.T) {
	const size = 24
	grid := testmap.RandomGrid(size, 5)
	transitions := testmap.EightWays(grid)
	start := testmap.Point{}
	wall := cells(grid, false)[0]
	for name, dict := range testmap.Dicts(size) {
		d := newPointDijkstra(dict())
		if got := d.Fill(sourceAt(start), transitions, nil, nil); got != 0 {
			t.Fatalf("%s: no targets, %d reported settled", name, got)
		}
		all := settledOf(d)
		if len(all) < size*size/2 {
			t.Fatalf("%s: %d states reached from %v; the scene wants most of the grid",
				name, len(all), start)
		}
		near, mid, far := all[len(all)/4], all[len(all)/3], all[len(all)/2]

		settles := func(targets []testmap.Point, want int, prefix []testmap.Point) {
			t.Helper()
			if got := d.Fill(sourceAt(start), transitions, nil, targets); got != want {
				t.Fatalf("%s %v: %d targets settled, want %d", name, targets, got, want)
			}
			if got := settledOf(d); !slices.Equal(got, prefix) {
				t.Fatalf("%s %v: %d states settled, want the first %d an exhaustive fill settles",
					name, targets, len(got), len(prefix))
			}
		}
		unsettled := func(p testmap.Point) bool {
			_, costed := d.Cost(p)
			_, parented := d.Parent(p)
			return !costed && !parented
		}
		settles([]testmap.Point{far, near, mid}, 3, all[:len(all)/2+1])
		if past := all[len(all)/2+1]; !unsettled(past) {
			t.Fatalf("%s: %v, past the last target, reads settled", name, past)
		}
		settles([]testmap.Point{near, near}, 1, all[:len(all)/4+1])
		settles([]testmap.Point{near, wall}, 1, all)
		if !unsettled(wall) {
			t.Fatalf("%s: the wall %v reads settled", name, wall)
		}
		settles([]testmap.Point{start}, 1, all[:1])

		if got := d.Fill(nil, transitions, nil, []testmap.Point{near}); got != 0 {
			t.Fatalf("%s: no sources, %d targets settled", name, got)
		}
		if !unsettled(near) || len(settledOf(d)) != 0 {
			t.Fatalf("%s: no sources, yet states read settled", name)
		}
	}
}

func TestDijkstra_StaysWithinTheBound(t *testing.T) {
	const size, lo, hi = 32, 8, 24
	grid := testmap.RandomGrid(size, 7)
	transitions := testmap.EightWays(grid)
	in := func(p testmap.Point) bool { return p.X >= lo && p.X < hi && p.Y >= lo && p.Y < hi }
	bounded := func(from, prev testmap.Point,
		buf []graph.Transition[testmap.Point]) []graph.Transition[testmap.Point] {
		return slices.DeleteFunc(transitions(from, prev, buf),
			func(s graph.Transition[testmap.Point]) bool { return !in(s.To) })
	}
	var start testmap.Point
	for _, p := range cells(grid, true) {
		if in(p) && p.Y >= size/2 {
			start = p
			break
		}
	}
	want := map[testmap.Point]float64{}
	a := zeroAStar()
	for _, p := range cells(grid, true) {
		if path := a.SolveInto(nil, start, p, bounded); path != nil {
			want[p] = testmap.PathCost(grid, path)
		}
	}

	outside := 0
	indexer := func(p testmap.Point) int {
		if !in(p) {
			outside++
		}
		return p.Y*size + p.X
	}
	dicts := map[string]dict.Dict[testmap.Point]{
		"IndexedMapDict":   dict.NewIndexedMap(0, indexer),
		"IndexedSliceDict": dict.NewIndexedSlice(size*size, indexer),
	}
	for name, dict := range dicts {
		d := newPointDijkstra(dict)
		outside = 0
		d.Fill(sourceAt(start), transitions, in, nil)
		if outside != 0 {
			t.Fatalf("%s: the dictionary was asked %d times of a state out of the bound",
				name, outside)
		}
		settled := settledOf(d)
		if len(settled) != len(want) {
			t.Fatalf("%s: %d states settled, the engine reaches %d in the bound",
				name, len(settled), len(want))
		}
		for _, p := range settled {
			cost, _ := d.Cost(p)
			if solved, reached := want[p]; !in(p) || !reached || math.Abs(cost-solved) > 1e-9 {
				t.Fatalf("%s: %v settled at %v, the engine's in the bound %v (%v)",
					name, p, cost, solved, reached)
			}
		}
		d.Fill(sourceAt(start), transitions, nil, nil)
		if !slices.ContainsFunc(settledOf(d), func(p testmap.Point) bool { return !in(p) }) {
			t.Fatalf("%s: with no bound nothing out of it settled", name)
		}
	}

	d := newPointDijkstra()
	edge := testmap.Point{X: lo - 1, Y: lo - 1}
	d.Fill(sourceAt(edge), testmap.EightWays(testmap.UniformGrid(size)), in, nil)
	if cost, ok := d.Cost(edge); !ok || cost != 0 {
		t.Fatalf("a source out of the bound reads %v (%v); want settled at 0", cost, ok)
	}
	if parent, ok := d.Parent(testmap.Point{X: lo, Y: lo}); !ok || parent != edge {
		t.Fatalf("the corner of the bound reached from %v (%v); want the source %v",
			parent, ok, edge)
	}
	for _, p := range settledOf(d) {
		if !in(p) && p != edge {
			t.Fatalf("%v settled out of the bound", p)
		}
	}
}

func TestDijkstra_StartsFromSeveralSourcesAtTheirCosts(t *testing.T) {
	const size = 20
	grid := testmap.RandomGrid(size, 11)
	transitions := testmap.EightWays(grid)
	open := cells(grid, true)
	sources := []Source[testmap.Point]{
		{At: open[0]},
		{At: open[len(open)/2], Cost: 3.5},
		{At: open[len(open)-1], Cost: 9},
	}
	want := map[testmap.Point]float64{}
	a := zeroAStar()
	for _, p := range open {
		for _, s := range sources {
			if path := a.SolveInto(nil, s.At, p, transitions); path != nil {
				if cost, ok := want[p]; !ok || s.Cost+testmap.PathCost(grid, path) < cost {
					want[p] = s.Cost + testmap.PathCost(grid, path)
				}
			}
		}
	}
	for name, dict := range testmap.Dicts(size) {
		d := newPointDijkstra(dict())
		d.Fill(sources, transitions, nil, nil)
		if settled := settledOf(d); len(settled) != len(want) {
			t.Fatalf("%s: %d states settled, the sources reach %d", name, len(settled), len(want))
		}
		for p, cheapest := range want {
			cost, ok := d.Cost(p)
			if !ok || math.Abs(cost-cheapest) > 1e-9 {
				t.Fatalf("%s: %v settled at %v (%v); the cheapest from a source %v",
					name, p, cost, ok, cheapest)
			}
			path := treePath(d, p)
			i := slices.IndexFunc(sources, func(s Source[testmap.Point]) bool {
				return len(path) > 0 && s.At == path[0]
			})
			if i < 0 {
				t.Fatalf("%s: the tree's path to %v starts at no source: %v", name, p, path)
			}
			if got := sources[i].Cost + testmap.PathCost(grid, path); math.Abs(got-cost) > 1e-9 {
				t.Fatalf("%s: from the source %v the tree's path to %v costs %v, cost %v",
					name, sources[i], p, got, cost)
			}
		}
	}

	uniform := testmap.EightWays(testmap.UniformGrid(size))
	d := newPointDijkstra()
	a2, b := testmap.Point{X: 5, Y: 5}, testmap.Point{X: 6, Y: 5}
	d.Fill([]Source[testmap.Point]{{At: a2}, {At: b, Cost: 100}}, uniform, nil, nil)
	if cost, _ := d.Cost(b); cost != 1 {
		t.Fatalf("a source a step from a cheaper one settled at %v; want 1", cost)
	}
	if parent, _ := d.Parent(b); parent != a2 {
		t.Fatalf("a source reached from a cheaper one has the parent %v; want %v", parent, a2)
	}
	for _, twice := range [][]Source[testmap.Point]{
		{{At: a2, Cost: 5}, {At: a2, Cost: 2}},
		{{At: a2, Cost: 2}, {At: a2, Cost: 5}},
	} {
		d.Fill(twice, uniform, nil, nil)
		if cost, _ := d.Cost(a2); cost != 2 {
			t.Fatalf("a source given at %v settled at %v; want the cheaper", twice, cost)
		}
		if parent, _ := d.Parent(a2); parent != a2 {
			t.Fatalf("a source given twice has the parent %v; want itself", parent)
		}
	}
	for _, pair := range [][2]testmap.Point{{a2, b}, {b, a2}} {
		d.Fill([]Source[testmap.Point]{{At: pair[0], Cost: 1}, {At: pair[1], Cost: 1}}, uniform,
			nil, nil)
		if first := settledOf(d)[0]; first != pair[0] {
			t.Fatalf("sources %v at one cost: %v settled first; want the first given", pair, first)
		}
	}
}

func TestDijkstra_TransitionsSeePrevAsTheParent(t *testing.T) {
	const size = 16
	steps := testmap.EightWays(testmap.RandomGrid(size, 3))
	prevOf := map[testmap.Point]testmap.Point{}
	calls := 0
	transitions := func(from, prev testmap.Point,
		buf []graph.Transition[testmap.Point]) []graph.Transition[testmap.Point] {
		calls++
		prevOf[from] = prev
		return steps(from, prev, buf)
	}
	d := newPointDijkstra()
	sources := []Source[testmap.Point]{{At: testmap.Point{}},
		{At: testmap.Point{X: size - 1, Y: size - 1}, Cost: 4}}
	d.Fill(sources, transitions, nil, nil)
	if settled := settledOf(d); calls != len(settled) || len(prevOf) != calls {
		t.Fatalf("%d calls for %d states, %d settled; want one a state settled",
			calls, len(prevOf), len(settled))
	}
	for from, prev := range prevOf {
		if parent, ok := d.Parent(from); !ok || parent != prev {
			t.Fatalf("the steps from %v asked with prev %v; its parent is %v (%v)",
				from, prev, parent, ok)
		}
	}
}

// The slice dictionary alone is pinned: a map's clear draws a new hash seed, so a map dictionary
// may grow its tables on a later fill.
func TestDijkstra_FillAllocatesNothingTwice(t *testing.T) {
	const size, side = 64, 16
	transitions := testmap.EightWays(testmap.WalledGrid(size))
	in := func(p testmap.Point) bool { return p.X < side && p.Y < side }
	sources := []Source[testmap.Point]{{At: testmap.Point{}},
		{At: testmap.Point{X: side - 1}, Cost: 2}}
	targets := []testmap.Point{{X: 0, Y: side - 1}, {X: side - 1, Y: side - 1}}
	d := newPointDijkstra(testmap.Dicts(size)["IndexedSliceDict"]())
	fills := func() {
		d.Fill(sources, transitions, nil, nil)
		if got := d.Fill(sources, transitions, in, targets); got != len(targets) {
			t.Fatalf("%d targets settled, want %d", got, len(targets))
		}
	}
	fills()
	if allocs := testing.AllocsPerRun(10, fills); allocs != 0 {
		t.Fatalf("fill allocates %v times once warmed up; want none", allocs)
	}
	reads := func() {
		for _, n := range d.settled {
			d.Cost(d.nodes[n].ID)
			d.Parent(d.nodes[n].ID)
		}
	}
	if allocs := testing.AllocsPerRun(10, reads); allocs != 0 {
		t.Fatalf("reading a fill allocates %v times; want none", allocs)
	}
}

// what a fill reads: its result, the states settled in order, their costs and parents, and how
// many of the grid's cells read settled
type fillRead struct {
	reported int
	settled  []testmap.Point
	costs    []float64
	parents  []testmap.Point
	known    int
}

func readFill(d *Engine[testmap.Point], reported, size int) fillRead {
	r := fillRead{reported: reported, settled: settledOf(d)}
	for _, p := range r.settled {
		cost, _ := d.Cost(p)
		parent, _ := d.Parent(p)
		r.costs, r.parents = append(r.costs, cost), append(r.parents, parent)
	}
	for p := range size * size {
		if _, ok := d.Cost(testmap.Point{X: p % size, Y: p / size}); ok {
			r.known++
		}
	}
	return r
}

func (r fillRead) equal(o fillRead) bool {
	return r.reported == o.reported && r.known == o.known && slices.Equal(r.settled, o.settled) &&
		slices.Equal(r.costs, o.costs) && slices.Equal(r.parents, o.parents)
}

// fills past many rounds of the slice dictionary's marks, each leaving behind states the next
// must not read, read as a fresh dijkstra's do
func TestDijkstra_ReusedPastTheMarksComingRound(t *testing.T) {
	const size = 32
	transitions := testmap.EightWays(testmap.WalledGrid(size))
	indexer := func(p testmap.Point) int { return p.Y*size + p.X }
	right := func(p testmap.Point) bool { return p.X >= size/2 }
	fill := func(d *Engine[testmap.Point], i int) fillRead {
		if i%2 == 0 {
			return readFill(d, d.Fill(sourceAt(testmap.Point{}), transitions, nil, nil), size)
		}
		far := testmap.Point{X: size - 1, Y: size - 1}
		reported := d.Fill(sourceAt(far), transitions, right, []testmap.Point{{X: size / 2}})
		return readFill(d, reported, size)
	}
	fresh := func() *Engine[testmap.Point] {
		return newPointDijkstra(dict.NewIndexedSlice(size*size, indexer))
	}
	want := []fillRead{fill(fresh(), 0), fill(fresh(), 1)}
	if want[1].reported != 1 || len(want[1].settled) >= len(want[0].settled) {
		t.Fatalf("the scene: %d and %d settled, the second reporting %d targets",
			len(want[0].settled), len(want[1].settled), want[1].reported)
	}
	d := fresh()
	for i := range 3 * 255 {
		if got := fill(d, i); !got.equal(want[i%2]) {
			t.Fatalf("fill %d: %d settled, %d known; a fresh one's %d, %d",
				i, len(got.settled), got.known, len(want[i%2].settled), want[i%2].known)
		}
	}
}
