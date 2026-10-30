package hierarchy

import (
	"fmt"
	"runtime"
	"sync/atomic"
	"testing"

	"github.com/kjkrol/hpastar/internal/testmap"
)

const benchWalls = 0.05

// benchReady is a hierarchy over a generated map built whole, with the map's far pair
func benchReady(b *testing.B, size int) (asked[int], [2]int) {
	m := testmap.New(1, size, benchWalls, false)
	h := testHierarchy(m.Graph())
	if _, left := h.Build(0); left > 0 {
		b.Fatal("build left clusters stale")
	}
	return h, m.FarPair()
}

// benchWhole is a dst as long as the whole route the climb writes from far[0] to far[1]
func benchWhole(b *testing.B, h asked[int], far [2]int) []int {
	for dst := make([]int, 1024); len(dst) <= 1<<24; dst = make([]int, 2*len(dst)) {
		n, ok := h.own.find(far[0], far[1], dst)
		if !ok {
			b.Fatal("no route between the far pair")
		}
		if n < len(dst) || dst[n-1] == far[1] {
			return dst[:n]
		}
	}
	b.Fatal("the route between the far pair outgrows a map of 4096x4096")
	return nil
}

// a hierarchy over a generated map, at the default side, factor and top, built whole from
// nothing
func BenchmarkHierarchy_Build(b *testing.B) {
	for _, size := range []int{256, 1024, 4096} {
		b.Run(fmt.Sprintf("whole/Size_%dx%d", size, size), func(b *testing.B) {
			testmap.SkipLarge(b, size)
			g := testmap.New(1, size, benchWalls, false).Graph()
			var h *Hierarchy[int]
			b.ReportAllocs()
			for b.Loop() {
				h = New(g, configOf())
				h.Build(0)
			}
			if !h.Ready() {
				b.Fatal("build left clusters stale")
			}
		})
	}
}

// the clusters of a state in the middle of a level-1 cluster built again after it is staled:
// that cluster and the ones above it
func BenchmarkHierarchy_Rebuild(b *testing.B) {
	const size, side = 1024, 16
	b.Run(fmt.Sprintf("cluster/Size_%dx%d", size, size), func(b *testing.B) {
		h, _ := benchReady(b, size)
		s := (size/2+side/2)*size + size/2 + side/2
		if h.Stale(s); h.Ready() {
			b.Fatal("stale left nothing to build")
		}
		h.Build(0)
		builds := h.builds
		b.ReportAllocs()
		for b.Loop() {
			h.Stale(s)
			h.Build(0)
		}
		b.ReportMetric(float64(h.builds-builds)/float64(b.N), "clusters/op")
	})
}

// the route between a map's far pair over a hierarchy built whole: its first 64 states, a
// caller's piece, and the whole of it
func BenchmarkHierarchy_Find(b *testing.B) {
	for _, size := range []int{256, 1024, 4096} {
		var h asked[int]
		var far [2]int
		rungs := []string{"chunk", "whole"}
		if size == 1024 {
			rungs = append(rungs, "parallel")
		}
		for _, rung := range rungs {
			b.Run(fmt.Sprintf("%s/Size_%dx%d", rung, size, size), func(b *testing.B) {
				testmap.SkipLarge(b, size)
				if h.Hierarchy == nil {
					h, far = benchReady(b, size)
				}
				dst := make([]int, 64)
				if rung == "whole" {
					dst = benchWhole(b, h, far)
				}
				if n, ok := h.own.find(far[0], far[1], dst); !ok || n != len(dst) {
					b.Fatalf("the climb wrote %d states of %d (%v)", n, len(dst), ok)
				}
				if rung == "parallel" {
					benchParallel(b, h, far)
					return
				}
				expansions := h.own.Expanded()
				b.ReportAllocs()
				for b.Loop() {
					h.own.find(far[0], far[1], dst)
				}
				b.ReportMetric(float64(h.own.Expanded()-expansions)/float64(b.N),
					"expansions/op")
			})
		}
	}
}

// benchParallel asks Hierarchy_Find/chunk of h on every goroutine at once, each through a climb
// of its own made and warmed before the clock starts
func benchParallel(b *testing.B, h asked[int], far [2]int) {
	type asker struct {
		s   *Climb[int]
		dst []int
	}
	askers := make(chan asker, runtime.GOMAXPROCS(0))
	for range cap(askers) {
		a := asker{h.NewClimb(h.graph.Forward, h.layout.Backward), make([]int, 64)}
		a.s.find(far[0], far[1], a.dst)
		askers <- a
	}
	var expanded atomic.Uint64
	b.ReportAllocs()
	b.ResetTimer()
	b.RunParallel(func(pb *testing.PB) {
		a := <-askers
		before := a.s.Expanded()
		for pb.Next() {
			a.s.find(far[0], far[1], a.dst)
		}
		expanded.Add(a.s.Expanded() - before)
	})
	b.ReportMetric(float64(expanded.Load())/float64(b.N), "expansions/op")
}
