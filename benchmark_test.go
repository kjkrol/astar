package astar_test

import (
	"fmt"
	"testing"

	"github.com/kjkrol/astar"
)

func BenchmarkSolver_Solve(b *testing.B) {
	sizes := []int{64, 256, 512, 1024, 2048}

	// Define test scenarios for terrain cost weights
	weights := []struct {
		name  string
		value float64
	}{
		{"CostWeight_1.0", 1.0},
		{"CostWeight_0.5", 0.5},
		{"CostWeight_0.1", 0.1},
	}

	for _, size := range sizes {
		b.Run(fmt.Sprintf("Size_%dx%d", size, size), func(b *testing.B) {
			// Generate the grid once per grid-size scenario to eliminate initialization noise
			grid := setupGrid(size)

			start := Point{X: 0, Y: 0}
			goal := Point{X: size - 1, Y: size - 1}
			indexer := func(p Point) int { return p.Y*size + p.X }
			maxSize := size * size

			for _, w := range weights {
				b.Run(w.name, func(b *testing.B) {
					// Create the execution-specific transition rules using the current weight
					transitions := setupTransitions(w.value, grid)

					b.Run("WithIndexedSliceDict", func(b *testing.B) {
						pathFinder := setupPathFinder(
							astar.WithInitCapacity[Point](maxSize),
							astar.WithIndexedSliceDict(maxSize, indexer),
						)
						b.ReportAllocs()
						b.ResetTimer()
						for i := 0; i < b.N; i++ {
							_ = pathFinder.Solve(start, goal, transitions)
						}
					})

					b.Run("WithIndexedMapDict", func(b *testing.B) {
						pathFinder := setupPathFinder(
							astar.WithInitCapacity[Point](maxSize),
							astar.WithIndexedMapDict(indexer),
						)
						b.ReportAllocs()
						b.ResetTimer()
						for i := 0; i < b.N; i++ {
							_ = pathFinder.Solve(start, goal, transitions)
						}
					})

					b.Run("DefaultMapDict", func(b *testing.B) {
						pathFinder := setupPathFinder(
							astar.WithInitCapacity[Point](maxSize),
						)
						b.ReportAllocs()
						b.ResetTimer()
						for i := 0; i < b.N; i++ {
							_ = pathFinder.Solve(start, goal, transitions)
						}
					})
				})
			}
		})
	}
}

// the caller's shape: a route across the grid at weight 1.0 written into a reused buffer
func BenchmarkSolver_SolveInto(b *testing.B) {
	for _, size := range []int{64, 256, 512, 1024, 2048} {
		b.Run(fmt.Sprintf("Size_%dx%d/WithIndexedSliceDict", size, size), func(b *testing.B) {
			grid := setupGrid(size)
			transitions := setupTransitions(1.0, grid)
			start, goal := Point{}, Point{X: size - 1, Y: size - 1}
			indexer := func(p Point) int { return p.Y*size + p.X }
			solver := setupPathFinder(
				astar.WithInitCapacity[Point](size*size),
				astar.WithIndexedSliceDict(size*size, indexer),
			)
			var path []Point
			b.ReportAllocs()
			for b.Loop() {
				path = solver.SolveInto(path, start, goal, transitions)
			}
		})
	}
}

// a grid walked eight ways with the octile distance, the shape of a board's planner
func BenchmarkSolver_Octile(b *testing.B) {
	for _, size := range []int{64, 256, 512, 1024} {
		b.Run(fmt.Sprintf("Size_%dx%d/WithIndexedSliceDict", size, size), func(b *testing.B) {
			grid := setupGrid(size)
			transitions := setupTransitions8(grid)
			start, goal := Point{}, Point{X: size - 1, Y: size - 1}
			indexer := func(p Point) int { return p.Y*size + p.X }
			solver := setupOctileFinder(
				astar.WithInitCapacity[Point](size*size),
				astar.WithIndexedSliceDict(size*size, indexer),
				astar.WithSuccessorCapacity[Point](8),
			)
			var path []Point
			b.ReportAllocs()
			for b.Loop() {
				path = solver.SolveInto(path, start, goal, transitions)
			}
		})
	}
}
