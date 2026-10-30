package hpastar_test

import (
	"fmt"
	"runtime"
	"testing"
	"time"

	"github.com/kjkrol/hpastar"
	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/testmap"
)

const benchWalls = 0.05

// classed is m's graph serving classes classes, the steps into its states refused as masks says:
// to none, in a quarter of the states of a twentieth of the first level's clusters, or in 2% of
// the states anywhere; and the bars it reads, each refusing a class and those above it
func classed(m *testmap.Map, classes int, masks string) (graph.Implicit[int], []uint8) {
	var bars []uint8
	switch masks {
	case "none":
		bars = make([]uint8, m.W*m.H)
	case "local":
		bars = m.Bars(2, classes, 0.25, func(x, y int) bool {
			return testmap.Hashed(3, x/16, y/16) < 0.05
		})
	default:
		bars = m.Bars(2, classes, 0.02, nil)
	}
	return m.Barred(classes, bars), bars
}

// the path between the far pair of a map, the whole of it: astar through a Solver without a
// Layout, every path the cheapest, hpa through one with it, built whole, climbing; of 2 and 8
// classes, the steps into 2% of the states refused, its first class, which none refuses, or its
// last, which most refuse, the Solver built once for both
func BenchmarkPath(b *testing.B) {
	for _, mode := range []string{"astar", "hpa"} {
		for _, size := range []int{256, 1024, 4096} {
			b.Run(fmt.Sprintf("%s/Size_%dx%d", mode, size, size), func(b *testing.B) {
				testmap.SkipLarge(b, size)
				m := testmap.New(1, size, benchWalls, false)
				benchPath(b, m, solverFor(b, m.Graph(), mode), mode, 0)
			})
		}
		for _, classes := range []int{2, 8} {
			for _, size := range []int{256, 1024} {
				var m *testmap.Map
				var s *hpastar.Solver[int]
				for k, class := range [2]string{"first", "last"} {
					name := fmt.Sprintf("%s/Classes_%d/%s/Size_%dx%d", mode, classes, class, size,
						size)
					k *= classes - 1
					b.Run(name, func(b *testing.B) {
						if s == nil {
							m = testmap.New(1, size, benchWalls, false)
							g, bars := classed(m, classes, "scattered")
							far := m.FarPair()
							bars[far[0]], bars[far[1]] = 0, 0
							s = solverFor(b, g, mode)
						}
						benchPath(b, m, s, mode, k)
					})
				}
			}
		}
	}
}

// solverFor is a Solver over g built whole, without g's Layout for astar
func solverFor(b *testing.B, g graph.Implicit[int], mode string) *hpastar.Solver[int] {
	if mode == "astar" {
		g.Layout = nil
	}
	s := hpastar.NewSolver(g)
	if _, left := s.Update(0); left != 0 {
		b.Fatalf("Update(0) left %d stale", left)
	}
	return s
}

// benchPath asks class k of s, over m, its path between m's far pair
func benchPath(b *testing.B, m *testmap.Map, s *hpastar.Solver[int], mode string, k int) {
	far := m.FarPair()
	q := s.Search()
	q.Class(k)
	// room for the whole path at once: an A* search across 4096x4096 takes long
	route, ok := path(q, far[0], far[1], make([]int, 4*m.W))
	if !ok {
		b.Fatal("no path between the far pair")
	}
	if st := q.Stats(); mode == "hpa" && st.Climbs == 0 {
		b.Fatal("the far pair not climbed")
	}
	cost, _ := testmap.WalkCost(m.Forward, route)
	before := q.Stats().Expansions
	b.ReportAllocs()
	for b.Loop() {
		q.Path(far[0], far[1], route)
	}
	b.ReportMetric(float64(q.Stats().Expansions-before)/float64(b.N), "expansions/op")
	b.ReportMetric(cost, "cost")
	b.ReportMetric(float64(len(route)), "states")
}

// the hierarchy of a map built whole from nothing, and built again after a state changed: amid
// a cluster of the first level near the map's middle, and on the borders of every level there;
// the state walled off and opened again in turn; of 2, 4 and 8 classes, the steps into its
// states refused to none, in a few clusters or anywhere
func BenchmarkUpdate(b *testing.B) {
	for _, size := range []int{256, 1024, 4096} {
		b.Run(fmt.Sprintf("build/Size_%dx%d", size, size), func(b *testing.B) {
			testmap.SkipLarge(b, size)
			benchBuild(b, testmap.New(1, size, benchWalls, false).Graph(), size)
		})
		for _, rung := range []string{"change", "change-border"} {
			b.Run(fmt.Sprintf("%s/Size_%dx%d", rung, size, size), func(b *testing.B) {
				testmap.SkipLarge(b, size)
				m := testmap.New(1, size, benchWalls, false)
				benchChange(b, m, m.Graph(), rung)
			})
		}
	}
	for _, classes := range []int{2, 4, 8} {
		for _, masks := range []string{"none", "local", "scattered"} {
			for _, size := range []int{256, 1024} {
				name := fmt.Sprintf("Classes_%d/%s/Size_%dx%d", classes, masks, size, size)
				b.Run("build/"+name, func(b *testing.B) {
					g, _ := classed(testmap.New(1, size, benchWalls, false), classes, masks)
					benchBuild(b, g, size)
				})
				for _, rung := range []string{"change", "change-border"} {
					b.Run(rung+"/"+name, func(b *testing.B) {
						m := testmap.New(1, size, benchWalls, false)
						g, _ := classed(m, classes, masks)
						benchChange(b, m, g, rung)
					})
				}
			}
		}
	}
}

// benchBuild builds a Solver's hierarchy over g, of size × size states, whole from nothing
func benchBuild(b *testing.B, g graph.Implicit[int], size int) {
	var held uint64
	steps := 0
	b.ReportAllocs()
	for i := 0; i < b.N; i++ {
		var before runtime.MemStats
		if i == 0 {
			b.StopTimer()
			runtime.GC()
			runtime.ReadMemStats(&before)
			b.StartTimer()
		}
		s := hpastar.NewSolver(g)
		s.Update(0)
		work, _ := hpastar.Worked(s)
		steps += work
		if i == 0 {
			b.StopTimer()
			var after runtime.MemStats
			runtime.GC()
			runtime.ReadMemStats(&after)
			runtime.KeepAlive(s)
			held = after.HeapAlloc - before.HeapAlloc
			b.StartTimer()
		}
	}
	b.ReportMetric(float64(held)/float64(size*size), "bytes/cell")
	b.ReportMetric(float64(steps)/float64(b.N), "steps/op")
}

// benchChange builds a Solver's hierarchy over g, of m, whole, then again after a state of m
// changes as rung says
func benchChange(b *testing.B, m *testmap.Map, g graph.Implicit[int], rung string) {
	size := m.W
	s := hpastar.NewSolver(g)
	s.Update(0)
	x := size/2 + 8
	if rung == "change-border" {
		x = size / 2
	}
	state := m.OpenNear(x, size/2+8)
	share := m.Share[state]
	var took [8]time.Duration
	clusters := 0
	hpastar.TimeRebuilds(s, func(level int, d time.Duration) {
		took[level] += d
		if level > 0 {
			clusters++
		}
	})
	steps := 0
	b.ReportAllocs()
	for i := 0; b.Loop(); i++ {
		m.Share[state] = float32(i%2) * share
		s.Changed(state)
		s.Update(0)
		work, _ := hpastar.Worked(s)
		steps += work
	}
	perOp := func(d time.Duration) float64 {
		return d.Seconds() * 1000 / float64(b.N)
	}
	b.ReportMetric(float64(clusters)/float64(b.N), "clusters/op")
	b.ReportMetric(float64(steps)/float64(b.N), "steps/op")
	for level := 1; level < len(took) && took[level] > 0; level++ {
		b.ReportMetric(perOp(took[level]), fmt.Sprintf("l%d-ms/op", level))
	}
	b.ReportMetric(perOp(took[0]), "labels-ms/op")
}
