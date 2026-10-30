package astar

import (
	"slices"
	"testing"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/testmap"
)

// pathFinder is an engine with the Manhattan distance, keeping its nodes in a map unless d gives
// another dictionary
func pathFinder(d ...dict.Dict[testmap.Point]) *Engine[testmap.Point] {
	return New(testmap.Manhattan, dictOr(d), 0)
}

// octileFinder is an engine with the octile distance
func octileFinder(d ...dict.Dict[testmap.Point]) *Engine[testmap.Point] {
	return New(testmap.Octile, dictOr(d), 0)
}

func dictOr(d []dict.Dict[testmap.Point]) dict.Dict[testmap.Point] {
	if len(d) > 0 {
		return d[0]
	}
	return dict.NewMap[testmap.Point](defaultCapacity)
}

// stepped is the path an engine finds expanding a state at a time, read after each step as an
// iteration would read it
func stepped(a *Engine[testmap.Point], from, to testmap.Point,
	transitions graph.Transitions[testmap.Point]) []testmap.Point {
	a.Start(from, to)
	for !a.Run(to, transitions, 1) {
	}
	return a.ResultInto(nil)
}

func TestAStar_FindsAPath(t *testing.T) {
	const size = 64
	grid := testmap.WalledGrid(size)
	start := testmap.Point{X: 0, Y: 0}
	to := testmap.Point{X: size - 1, Y: size - 1}
	for dictName, dict := range testmap.Dicts(size) {
		t.Run(dictName, func(t *testing.T) {
			t.Run("Solve", func(t *testing.T) {
				path := pathFinder(dict()).SolveInto(nil, start, to, testmap.FourWays(1.0, grid))
				verifyPath(t, path, start, to)
			})
			t.Run("Stepped", func(t *testing.T) {
				path := stepped(pathFinder(dict()), start, to, testmap.FourWays(1.0, grid))
				verifyPath(t, path, start, to)
			})
		})
	}
}

func verifyPath(t *testing.T, path []testmap.Point, start, to testmap.Point) {
	t.Helper()
	if len(path) == 0 {
		t.Fatalf("expected a valid path, but got none (nil/empty slice)")
	}
	if path[0] != start {
		t.Errorf("path starts at %v, expected %v", path[0], start)
	}
	if path[len(path)-1] != to {
		t.Errorf("path ends at %v, expected %v", path[len(path)-1], to)
	}
}

// a target walled off on every side: no path, through a search run whole or a step at a time
func TestAStar_UnreachableTarget(t *testing.T) {
	grid := [][]float64{
		{1, testmap.InsurmountableObstacle, 1},
		{testmap.InsurmountableObstacle, 1, testmap.InsurmountableObstacle},
		{1, testmap.InsurmountableObstacle, 1},
	}
	start, to := testmap.Point{X: 0, Y: 0}, testmap.Point{X: 1, Y: 1}
	a := pathFinder()
	if path := a.SolveInto(nil, start, to, testmap.FourWays(1.0, grid)); path != nil {
		t.Fatalf("expected nil for an unreachable target, got %v", path)
	}
	if a.Found() {
		t.Fatal("found an unreachable target")
	}
	if path := stepped(pathFinder(), start, to, testmap.FourWays(1.0, grid)); path != nil {
		t.Fatalf("stepped: expected nil for an unreachable target, got %v", path)
	}
}

func TestAStar_ReusedAcrossSearches(t *testing.T) {
	const size = 32
	grid := testmap.WalledGrid(size)
	transitions := testmap.FourWays(1.0, grid)
	queries := [][2]testmap.Point{
		{{X: 0, Y: 0}, {X: size - 1, Y: size - 1}},
		{{X: size - 1, Y: size - 1}, {X: 0, Y: 0}},
		{{X: 0, Y: 0}, {X: 2, Y: 0}}, // a wall: unreachable
		{{X: 5, Y: 5}, {X: 20, Y: 9}},
		{{X: 0, Y: 0}, {X: size - 1, Y: size - 1}},
		{{X: 3, Y: 3}, {X: 3, Y: 3}},
		{{X: 20, Y: 9}, {X: 5, Y: 5}},
	}
	for dictName, dict := range testmap.Dicts(size) {
		t.Run(dictName, func(t *testing.T) {
			reused := pathFinder(dict())
			for _, q := range queries {
				got := reused.SolveInto(nil, q[0], q[1], transitions)
				want := pathFinder(dict()).SolveInto(nil, q[0], q[1], transitions)
				if !slices.Equal(got, want) {
					t.Fatalf("%v -> %v on a reused engine: got %v, want %v", q[0], q[1], got, want)
				}
			}
		})
	}
}

func TestAStar_KeepsGrownTransitionBuffer(t *testing.T) {
	const size = 8
	var grown, calls int
	// every cell up to two away: more steps than the buffer is first made for
	transitions := func(from, prev testmap.Point,
		buf []graph.Transition[testmap.Point]) []graph.Transition[testmap.Point] {
		calls++
		if len(buf) != 0 {
			t.Fatalf("call %d: the buffer holds %d transitions; want none", calls, len(buf))
		}
		if cap(buf) < grown {
			t.Fatalf("call %d: the buffer's capacity is %d; it had grown to %d",
				calls, cap(buf), grown)
		}
		// a dead end handing back no buffer at all must not lose the grown one
		if from == (testmap.Point{X: 1, Y: 1}) {
			return nil
		}
		for dx := -2; dx <= 2; dx++ {
			for dy := -2; dy <= 2; dy++ {
				to := testmap.Point{X: from.X + dx, Y: from.Y + dy}
				off := to.X < 0 || to.Y < 0 || to.X >= size || to.Y >= size
				if off || to == from || to == prev {
					continue
				}
				buf = append(buf, graph.Transition[testmap.Point]{To: to, Cost: 1})
			}
		}
		grown = max(grown, cap(buf))
		return buf
	}
	a := pathFinder()
	for range 2 {
		verifyPath(t, a.SolveInto(nil, testmap.Point{}, testmap.Point{X: size - 1, Y: size - 1},
			transitions), testmap.Point{}, testmap.Point{X: size - 1, Y: size - 1})
	}
	if grown <= defaultSuccessors {
		t.Fatalf("the buffer never grew past the first capacity: %d", grown)
	}
}

func TestAStar_ResultIntoWritesTheResult(t *testing.T) {
	const size = 32
	grid := testmap.WalledGrid(size)
	transitions := testmap.FourWays(1.0, grid)
	queries := [][2]testmap.Point{
		{{X: 0, Y: 0}, {X: 2, Y: 0}}, // a wall: unreachable
		{{X: 3, Y: 3}, {X: 3, Y: 3}},
		{{X: 5, Y: 5}, {X: 20, Y: 9}},
		{{X: 0, Y: 0}, {X: size - 1, Y: size - 1}},
		{{X: 20, Y: 9}, {X: 5, Y: 5}},
		{{X: 0, Y: 0}, {X: 2, Y: 0}},
	}
	for dictName, dict := range testmap.Dicts(size) {
		t.Run(dictName, func(t *testing.T) {
			a, b := pathFinder(dict()), pathFinder(dict())
			buf := make([]testmap.Point, 1, 2)
			var stepBuf []testmap.Point
			for _, q := range queries {
				want := pathFinder(dict()).SolveInto(nil, q[0], q[1], transitions)
				before := buf
				buf = a.SolveInto(buf, q[0], q[1], transitions)
				if !slices.Equal(buf, want) {
					t.Fatalf("%v -> %v: solveInto gave %v, want %v", q[0], q[1], buf, want)
				}
				if cap(before) >= len(want) && cap(buf) != cap(before) {
					t.Fatalf("%v -> %v: a buffer of capacity %d was replaced for a path of %d",
						q[0], q[1], cap(before), len(want))
				}
				b.Start(q[0], q[1])
				for !b.Run(q[1], transitions, 3) {
				}
				stepBuf = b.ResultInto(stepBuf)
				if !slices.Equal(stepBuf, want) {
					t.Fatalf("%v -> %v: resultInto gave %v, want %v", q[0], q[1], stepBuf, want)
				}
			}
		})
	}
}

func TestAStar_PrefixWritesTheFirstStates(t *testing.T) {
	const size = 32
	grid := testmap.WalledGrid(size)
	transitions := testmap.EightWays(grid)
	a := octileFinder()
	for _, q := range [][2]testmap.Point{
		{{X: 0, Y: 0}, {X: size - 1, Y: size - 1}},
		{{X: 3, Y: 3}, {X: 3, Y: 3}},
		{{X: 0, Y: 0}, {X: 2, Y: 0}},
	} {
		want := octileFinder().SolveInto(nil, q[0], q[1], transitions)
		for _, n := range []int{0, 1, 3, len(want), len(want) + 5} {
			a.Start(q[0], q[1])
			a.Run(q[1], transitions, -1)
			dst := make([]testmap.Point, n)
			k := a.Prefix(dst)
			if k != min(n, len(want)) || !slices.Equal(dst[:k], want[:k]) {
				t.Fatalf("%v -> %v into %d: wrote %d, %v; the path begins %v",
					q[0], q[1], n, k, dst[:k], want[:min(n, len(want))])
			}
		}
	}
}

// The slice dictionary alone is pinned: a map's clear draws a new hash seed, so a map dictionary
// may grow its tables on a later search.
func TestAStar_SolveIntoAllocatesNothing(t *testing.T) {
	const size = 64
	grid := testmap.WalledGrid(size)
	transitions := testmap.FourWays(1.0, grid)
	start, to := testmap.Point{X: 0, Y: 0}, testmap.Point{X: size - 1, Y: size - 1}
	t.Run("IndexedSliceDict", func(t *testing.T) {
		a := pathFinder(testmap.Dicts(size)["IndexedSliceDict"]())
		path := a.SolveInto(nil, start, to, transitions)
		verifyPath(t, path, start, to)
		allocs := testing.AllocsPerRun(10, func() {
			path = a.SolveInto(path, start, to, transitions)
		})
		if allocs != 0 {
			t.Fatalf("solveInto into a buffer big enough allocates %v times; want none", allocs)
		}
	})
	t.Run("Octile", func(t *testing.T) {
		transitions := testmap.EightWays(grid)
		a := octileFinder(testmap.Dicts(size)["IndexedSliceDict"]())
		path := a.SolveInto(nil, start, to, transitions)
		verifyPath(t, path, start, to)
		allocs := testing.AllocsPerRun(10, func() {
			path = a.SolveInto(path, start, to, transitions)
		})
		if allocs != 0 {
			t.Fatalf("solveInto eight ways allocates %v times; want none", allocs)
		}
	})
}

// the marks of the slice dictionary come round every 255 searches: searches past many rounds
// find what a fresh engine finds
func TestAStar_ReusedPastTheMarksComingRound(t *testing.T) {
	const size = 32
	grid := testmap.WalledGrid(size)
	transitions := testmap.EightWays(grid)
	queries := [][2]testmap.Point{
		{{X: 0, Y: 0}, {X: size - 1, Y: size - 1}},
		{{X: size - 1, Y: 0}, {X: 3, Y: size - 2}},
		{{X: 0, Y: 0}, {X: 2, Y: 0}},
	}
	want := make([][]testmap.Point, len(queries))
	for i, q := range queries {
		want[i] = octileFinder().SolveInto(nil, q[0], q[1], transitions)
	}
	a := octileFinder(testmap.Dicts(size)["IndexedSliceDict"]())
	var path []testmap.Point
	for i := range 3 * 255 {
		q := queries[i%len(queries)]
		if path = a.SolveInto(path, q[0], q[1], transitions); !slices.Equal(path, want[i%3]) {
			t.Fatalf("search %d, %v -> %v: %v; a fresh engine finds %v",
				i, q[0], q[1], path, want[i%3])
		}
	}
}
