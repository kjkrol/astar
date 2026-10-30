package astar

import (
	"fmt"
	"testing"

	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/testmap"
)

// The A* engine's benchmarks keep the names they had in v1, Solver_*, for their history in
// BENCHMARKS.md.

func BenchmarkSolver_Solve(b *testing.B) {
	sizes := []int{64, 256, 512, 1024, 2048}
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
			// the grid made once a size, out of the clock
			grid := testmap.WalledGrid(size)
			start := testmap.Point{X: 0, Y: 0}
			goal := testmap.Point{X: size - 1, Y: size - 1}
			indexer := func(p testmap.Point) int { return p.Y*size + p.X }
			maxSize := size * size
			for _, w := range weights {
				b.Run(w.name, func(b *testing.B) {
					transitions := testmap.FourWays(w.value, grid)
					dicts := []struct {
						name string
						dict func() dict.Dict[testmap.Point]
					}{
						{"WithIndexedSliceDict", func() dict.Dict[testmap.Point] {
							return dict.NewIndexedSlice(maxSize, indexer)
						}},
						{"WithIndexedMapDict", func() dict.Dict[testmap.Point] {
							return dict.NewIndexedMap(maxSize, indexer)
						}},
						{"DefaultMapDict", func() dict.Dict[testmap.Point] {
							return dict.NewMap[testmap.Point](maxSize)
						}},
					}
					for _, d := range dicts {
						b.Run(d.name, func(b *testing.B) {
							a := New(testmap.Manhattan, d.dict(), maxSize)
							b.ReportAllocs()
							b.ResetTimer()
							for i := 0; i < b.N; i++ {
								_ = a.SolveInto(nil, start, goal, transitions)
							}
						})
					}
				})
			}
		})
	}
}

// the caller's shape: a route across the grid at weight 1.0 written into a reused buffer
func BenchmarkSolver_SolveInto(b *testing.B) {
	for _, size := range []int{64, 256, 512, 1024, 2048} {
		b.Run(fmt.Sprintf("Size_%dx%d/WithIndexedSliceDict", size, size), func(b *testing.B) {
			grid := testmap.WalledGrid(size)
			transitions := testmap.FourWays(1.0, grid)
			start, goal := testmap.Point{}, testmap.Point{X: size - 1, Y: size - 1}
			indexer := func(p testmap.Point) int { return p.Y*size + p.X }
			a := New(testmap.Manhattan, dict.NewIndexedSlice(size*size, indexer), size*size)
			var path []testmap.Point
			b.ReportAllocs()
			for b.Loop() {
				path = a.SolveInto(path, start, goal, transitions)
			}
		})
	}
}

// a grid walked eight ways with the octile distance, the shape of a board's planner
func BenchmarkSolver_Octile(b *testing.B) {
	for _, size := range []int{64, 256, 512, 1024} {
		b.Run(fmt.Sprintf("Size_%dx%d/WithIndexedSliceDict", size, size), func(b *testing.B) {
			grid := testmap.WalledGrid(size)
			transitions := testmap.EightWays(grid)
			start, goal := testmap.Point{}, testmap.Point{X: size - 1, Y: size - 1}
			indexer := func(p testmap.Point) int { return p.Y*size + p.X }
			a := New(testmap.Octile, dict.NewIndexedSlice(size*size, indexer), size*size)
			var path []testmap.Point
			b.ReportAllocs()
			for b.Loop() {
				path = a.SolveInto(path, start, goal, transitions)
			}
		})
	}
}

// the route of Hierarchy_Find/whole searched by the A* engine eight ways, its fair sibling
func BenchmarkSolver_OctileFar(b *testing.B) {
	for _, size := range []int{256, 1024} {
		b.Run(fmt.Sprintf("Size_%dx%d/WithIndexedSliceDict", size, size), func(b *testing.B) {
			m := testmap.New(1, size, 0.05, false)
			far := m.FarPair()
			from, to := far[0], far[1]
			a := New[int](m.Octile, dict.NewIndexedSlice(size*size, testmap.Identity), size*size)
			path := a.SolveInto(nil, from, to, m.Forward)
			if len(path) == 0 {
				b.Fatal("no route between the far pair")
			}
			expansions := a.Expanded()
			b.ReportAllocs()
			for b.Loop() {
				path = a.SolveInto(path, from, to, m.Forward)
			}
			b.ReportMetric(float64(a.Expanded()-expansions)/float64(b.N), "expansions/op")
		})
	}
}
