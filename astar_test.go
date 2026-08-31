package astar_test

import (
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
