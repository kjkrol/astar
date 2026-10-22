package astar_test

import (
	"math"
	"slices"
	"testing"

	"github.com/kjkrol/astar"
)

func TestSolver(t *testing.T) {
	const size = 64
	grid := setupGrid(size)
	start := Point{X: 0, Y: 0}
	to := Point{X: size - 1, Y: size - 1}
	indexer := func(p Point) int { return p.Y*size + p.X }

	// Tabela konfiguracji słowników (Dictionaries)
	dictionaries := map[string][]astar.SolverOption[Point]{
		"DefaultDict":      nil,
		"IndexedMapDict":   {astar.WithInitCapacity[Point](size * size), astar.WithIndexedMapDict(indexer)},
		"IndexedSliceDict": {astar.WithIndexedSliceDict(size*size, indexer)},
	}

	for dictName, opts := range dictionaries {
		t.Run(dictName, func(t *testing.T) {

			// 1. Test dla metody Solve
			t.Run("Solve", func(t *testing.T) {
				pathFinder := setupPathFinder(opts...)
				transitions := setupTransitions(1.0, grid)

				path := pathFinder.Solve(start, to, transitions)
				verifyPath(t, path, start, to)
			})

			// 2. Test dla metody Iter
			t.Run("Iter", func(t *testing.T) {
				pathFinder := setupPathFinder(opts...)
				transitions := setupTransitions(1.0, grid)

				// Konsumowanie iteratora do końca
				for range pathFinder.Iter(start, to, transitions) {
				}

				path := pathFinder.Result()
				verifyPath(t, path, start, to)
			})
		})
	}
}

// Generyczny helper do weryfikacji asercji ścieżki
func verifyPath(t *testing.T, path []Point, start, to Point) {
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

// TestSolver_UnreachableTarget guards Solve's documented contract ("returns
// the full sequence of states, or nil if no solution exists") for a target
// walled off on every side — both through Solve directly and through the
// lower-level Iter+Result path it's built on.
func TestSolver_UnreachableTarget(t *testing.T) {
	grid := [][]float64{
		{1, InsurmountableObstacle, 1},
		{InsurmountableObstacle, 1, InsurmountableObstacle},
		{1, InsurmountableObstacle, 1},
	}
	start, to := Point{X: 0, Y: 0}, Point{X: 1, Y: 1}

	t.Run("Solve", func(t *testing.T) {
		pathFinder := setupPathFinder()
		path := pathFinder.Solve(start, to, setupTransitions(1.0, grid))
		if path != nil {
			t.Fatalf("expected nil for an unreachable target, got %v", path)
		}
	})

	t.Run("Iter", func(t *testing.T) {
		pathFinder := setupPathFinder()
		for range pathFinder.Iter(start, to, setupTransitions(1.0, grid)) {
		}
		if path := pathFinder.Result(); path != nil {
			t.Fatalf("expected nil for an unreachable target, got %v", path)
		}
	})
}

func TestSolver_ReusedAcrossSearches(t *testing.T) {
	const size = 32
	grid := setupGrid(size)
	transitions := setupTransitions(1.0, grid)
	indexer := func(p Point) int { return p.Y*size + p.X }
	dictionaries := map[string][]astar.SolverOption[Point]{
		"DefaultDict":      nil,
		"IndexedMapDict":   {astar.WithIndexedMapDict(indexer)},
		"IndexedSliceDict": {astar.WithIndexedSliceDict(size*size, indexer)},
	}
	queries := [][2]Point{
		{{X: 0, Y: 0}, {X: size - 1, Y: size - 1}},
		{{X: size - 1, Y: size - 1}, {X: 0, Y: 0}},
		{{X: 0, Y: 0}, {X: 2, Y: 0}}, // a wall: unreachable
		{{X: 5, Y: 5}, {X: 20, Y: 9}},
		{{X: 0, Y: 0}, {X: size - 1, Y: size - 1}},
		{{X: 3, Y: 3}, {X: 3, Y: 3}},
		{{X: 20, Y: 9}, {X: 5, Y: 5}},
	}

	for dictName, opts := range dictionaries {
		t.Run(dictName, func(t *testing.T) {
			reused := setupPathFinder(opts...)
			for _, q := range queries {
				got := reused.Solve(q[0], q[1], transitions)
				want := setupPathFinder(opts...).Solve(q[0], q[1], transitions)
				if !slices.Equal(got, want) {
					t.Fatalf("%v -> %v on a reused solver: got %v, want %v", q[0], q[1], got, want)
				}
			}
		})
	}
}

func TestSolver_KeepsGrownTransitionBuffer(t *testing.T) {
	const size = 8
	var grown, calls int
	transitions := func(
		from, prev Point,
		buf []astar.Transition[Point],
	) []astar.Transition[Point] {
		calls++
		if len(buf) != 0 {
			t.Fatalf("call %d: the buffer holds %d transitions; want none", calls, len(buf))
		}
		if cap(buf) < grown {
			t.Fatalf("call %d: the buffer's capacity is %d; it had grown to %d",
				calls, cap(buf), grown)
		}
		// a dead end handing back no buffer at all must not lose the grown one
		if from == (Point{X: 1, Y: 1}) {
			return nil
		}
		for dx := -1; dx <= 1; dx++ {
			for dy := -1; dy <= 1; dy++ {
				to := Point{X: from.X + dx, Y: from.Y + dy}
				off := to.X < 0 || to.Y < 0 || to.X >= size || to.Y >= size
				if off || to == from || to == prev {
					continue
				}
				buf = append(buf, astar.Transition[Point]{To: to, Cost: 1})
			}
		}
		grown = max(grown, cap(buf))
		return buf
	}

	solver := setupPathFinder()
	for range 2 {
		verifyPath(t, solver.Solve(Point{}, Point{X: size - 1, Y: size - 1}, transitions),
			Point{}, Point{X: size - 1, Y: size - 1})
	}
	if grown <= 4 {
		t.Fatalf("the buffer never grew past the default capacity: %d", grown)
	}
}

func TestSolver_ResultIntoWritesTheResult(t *testing.T) {
	const size = 32
	grid := setupGrid(size)
	transitions := setupTransitions(1.0, grid)
	indexer := func(p Point) int { return p.Y*size + p.X }
	dictionaries := map[string][]astar.SolverOption[Point]{
		"DefaultDict":      nil,
		"IndexedMapDict":   {astar.WithIndexedMapDict(indexer)},
		"IndexedSliceDict": {astar.WithIndexedSliceDict(size*size, indexer)},
	}
	queries := [][2]Point{
		{{X: 0, Y: 0}, {X: 2, Y: 0}}, // a wall: unreachable
		{{X: 3, Y: 3}, {X: 3, Y: 3}},
		{{X: 5, Y: 5}, {X: 20, Y: 9}},
		{{X: 0, Y: 0}, {X: size - 1, Y: size - 1}},
		{{X: 20, Y: 9}, {X: 5, Y: 5}},
		{{X: 0, Y: 0}, {X: 2, Y: 0}},
	}

	for dictName, opts := range dictionaries {
		t.Run(dictName, func(t *testing.T) {
			solver, stepped := setupPathFinder(opts...), setupPathFinder(opts...)
			buf := make([]Point, 1, 2)
			var iterBuf []Point
			for _, q := range queries {
				want := setupPathFinder(opts...).Solve(q[0], q[1], transitions)
				before := buf
				buf = solver.SolveInto(buf, q[0], q[1], transitions)
				if !slices.Equal(buf, want) {
					t.Fatalf("%v -> %v: SolveInto gave %v, want %v", q[0], q[1], buf, want)
				}
				if cap(before) >= len(want) && cap(buf) != cap(before) {
					t.Fatalf("%v -> %v: a buffer of capacity %d was replaced for a path of %d",
						q[0], q[1], cap(before), len(want))
				}
				for range stepped.Iter(q[0], q[1], transitions) {
				}
				iterBuf = stepped.ResultInto(iterBuf)
				if !slices.Equal(iterBuf, want) {
					t.Fatalf("%v -> %v: ResultInto gave %v, want %v", q[0], q[1], iterBuf, want)
				}
			}
		})
	}
}

func TestSolver_SolveIntoAllocatesNothing(t *testing.T) {
	const size = 64
	grid := setupGrid(size)
	transitions := setupTransitions(1.0, grid)
	start, to := Point{X: 0, Y: 0}, Point{X: size - 1, Y: size - 1}
	indexer := func(p Point) int { return p.Y*size + p.X }
	dictionaries := map[string][]astar.SolverOption[Point]{
		"DefaultDict":      nil,
		"IndexedMapDict":   {astar.WithIndexedMapDict(indexer)},
		"IndexedSliceDict": {astar.WithIndexedSliceDict(size*size, indexer)},
	}

	for dictName, opts := range dictionaries {
		t.Run(dictName, func(t *testing.T) {
			solver := setupPathFinder(opts...)
			path := solver.SolveInto(nil, start, to, transitions)
			verifyPath(t, path, start, to)
			allocs := testing.AllocsPerRun(10, func() {
				path = solver.SolveInto(path, start, to, transitions)
			})
			if allocs != 0 {
				t.Fatalf("SolveInto into a buffer big enough allocates %v times; want none", allocs)
			}
		})
	}
}

func TestSolver_WipeBetweenSearches(t *testing.T) {
	const size = 32
	grid := setupGrid(size)
	transitions := setupTransitions(1.0, grid)
	indexer := func(p Point) int { return p.Y*size + p.X }
	from, to := Point{X: 0, Y: 0}, Point{X: size - 1, Y: size - 1}
	want := setupPathFinder().Solve(from, to, transitions)

	s := setupPathFinder(astar.WithIndexedSliceDict(size*size, indexer))
	for range 190 {
		s.Solve(from, to, transitions)
	}
	if s.Wipe(64) {
		t.Fatal("wiped with 64 searches left before the stamp wraps")
	}
	s.Solve(from, to, transitions)
	if !s.Wipe(64) {
		t.Fatal("not wiped with 63 searches left before the stamp wraps")
	}
	if got := s.Solve(from, to, transitions); !slices.Equal(got, want) {
		t.Fatalf("after the wipe: got %v, want %v", got, want)
	}
	if setupPathFinder().Wipe(math.MaxInt) {
		t.Fatal("a solver over a map wiped")
	}
}
