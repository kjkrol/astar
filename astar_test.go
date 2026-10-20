package astar_test

import (
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
