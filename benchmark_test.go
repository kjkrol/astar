package hpastar_test

import (
	"fmt"
	"runtime"
	"testing"
	"time"

	"github.com/kjkrol/hpastar"
	"github.com/kjkrol/hpastar/internal/testmap"
)

const benchWalls = 0.05

// the path between the far pair of a map, the whole of it: astar through a Solver without a
// Layout, every path the cheapest, hpa through one with it, built whole, climbing
func BenchmarkPath(b *testing.B) {
	for _, mode := range []string{"astar", "hpa"} {
		for _, size := range []int{256, 1024, 4096} {
			b.Run(fmt.Sprintf("%s/Size_%dx%d", mode, size, size), func(b *testing.B) {
				testmap.SkipLarge(b, size)
				m := testmap.New(1, size, benchWalls, false)
				g := m.Graph()
				if mode == "astar" {
					g.Layout = nil
				}
				s := hpastar.NewSolver(g)
				if _, left := s.Update(0); left != 0 {
					b.Fatalf("Update(0) left %d stale", left)
				}
				far := m.FarPair()
				q := s.Search()
				// room for the whole path at once: an A* search across 4096x4096 takes long
				route, ok := path(q, far[0], far[1], make([]int, 4*size))
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
			})
		}
	}
}

// the hierarchy of a map built whole from nothing, and built again after a state changed: amid
// a cluster of the first level near the map's middle, and on the borders of every level there;
// the state walled off and opened again in turn
func BenchmarkUpdate(b *testing.B) {
	for _, size := range []int{256, 1024, 4096} {
		b.Run(fmt.Sprintf("build/Size_%dx%d", size, size), func(b *testing.B) {
			testmap.SkipLarge(b, size)
			g := testmap.New(1, size, benchWalls, false).Graph()
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
		})
		for _, rung := range []string{"change", "change-border"} {
			b.Run(fmt.Sprintf("%s/Size_%dx%d", rung, size, size), func(b *testing.B) {
				testmap.SkipLarge(b, size)
				m := testmap.New(1, size, benchWalls, false)
				s := hpastar.NewSolver(m.Graph())
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
			})
		}
	}
}
