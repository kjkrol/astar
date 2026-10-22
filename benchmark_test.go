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
						b.ResetTimer()
						for i := 0; i < b.N; i++ {
							_ = pathFinder.Solve(start, goal, transitions)
						}
					})

					b.Run("DefaultMapDict", func(b *testing.B) {
						pathFinder := setupPathFinder(
							astar.WithInitCapacity[Point](maxSize),
						)
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
