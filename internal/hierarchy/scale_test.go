package hierarchy

import (
	"fmt"
	"runtime"
	"slices"
	"testing"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/dijkstra"
	"github.com/kjkrol/hpastar/internal/testmap"
)

const scaleWalls = 0.05

// scaleBuilt is a hierarchy over m built whole.
func scaleBuilt(t *testing.T, m *testmap.Map, opts ...option) asked[int] {
	t.Helper()
	h := testHierarchy(m.Graph(), opts...)
	if _, left := h.Build(0); left > 0 || !h.Ready() {
		t.Fatal("build left clusters stale")
	}
	return h
}

// scaleRoute is the whole route a climb writes from from to to, dst grown until it holds it or
// outgrows a map of 4096x4096.
func scaleRoute(h asked[int], from, to int, dst []int) ([]int, bool) {
	for {
		n, ok := h.own.find(from, to, dst)
		if !ok || n == 0 {
			return nil, false
		}
		if n < len(dst) || dst[n-1] == to || len(dst) > 1<<24 {
			return dst[:n], true
		}
		dst = make([]int, 2*len(dst))
	}
}

// scaleCost is what route costs a step at a time; false where a step is none of m's.
func scaleCost(m *testmap.Map, route []int) (float64, bool) {
	var cost float64
	var buf []graph.Transition[int]
	for i := 1; i < len(route); i++ {
		buf = m.Forward(route[i-1], route[i-1], buf)
		j := slices.IndexFunc(buf, func(s graph.Transition[int]) bool { return s.To == route[i] })
		if j < 0 {
			return 0, false
		}
		cost += buf[j].Cost
	}
	return cost, true
}

// scaleHeld is how many bytes of the heap what build makes holds.
func scaleHeld(build func() any) int64 {
	var before, after runtime.MemStats
	runtime.GC()
	runtime.ReadMemStats(&before)
	held := build()
	runtime.GC()
	runtime.ReadMemStats(&after)
	runtime.KeepAlive(held)
	return int64(after.HeapAlloc) - int64(before.HeapAlloc)
}

func TestScale_LevelsGrowWithTheMap(t *testing.T) {
	for _, c := range []struct {
		size     int
		clusters []int // on each level, from the first up
	}{
		{256, []int{256, 16}},
		{1024, []int{4096, 256, 16}},
		{4096, []int{65536, 4096, 256, 16}},
	} {
		t.Run(fmt.Sprintf("Size_%dx%d", c.size, c.size), func(t *testing.T) {
			testmap.SkipLarge(t, c.size)
			m := testmap.New(1, c.size, scaleWalls, false)
			h := New(m.Graph(), configOf(withClusterSide(16), withFactor(4), withTop(16)))
			got := make([]int, len(h.levels))
			for level := range got {
				got[level] = h.clusters(level + 1)
			}
			if !slices.Equal(got, c.clusters) {
				t.Errorf("clusters by level %v, want %v", got, c.clusters)
			}
		})
	}
}

func TestScale_RoutesCostWithinABound(t *testing.T) {
	for _, c := range []struct {
		size, sources, targets int
		// the farthest a target lies from its source, 0 anywhere: a fill reaching a target
		// anywhere on 4096x4096 settles most of 16.7M states
		far float64
	}{
		{1024, 5, 20, 0},
		{4096, 2, 10, 1024},
	} {
		t.Run(fmt.Sprintf("Size_%dx%d", c.size, c.size), func(t *testing.T) {
			testmap.SkipLarge(t, c.size)
			m := testmap.New(1, c.size, scaleWalls, false)
			h := scaleBuilt(t, m)
			// a fill from a source settles its targets at their cheapest costs at once
			d := dijkstra.New[int](dict.NewIndexedSlice(c.size*c.size, testmap.Identity), 0)
			rnd := testmap.Splitmix(2)
			dst := make([]int, 4*c.size)
			var routes int
			var sum, worst float64
			for range c.sources {
				from := m.AnyOpen(rnd)
				targets := m.Apart(rnd, from, c.targets, 64, c.far)
				d.Fill([]dijkstra.Source[int]{{At: from}}, m.Forward, nil, targets)
				for _, to := range targets {
					cheapest, reached := d.Cost(to)
					if !reached {
						if h.own.Reaches(from, to) {
							t.Errorf("reaches %d → %d, which no route joins", from, to)
						}
						continue
					}
					route, ok := scaleRoute(h, from, to, dst)
					if !ok {
						t.Errorf("no route %d → %d, the cheapest at %.1f", from, to, cheapest)
						continue
					}
					cost, walks := scaleCost(m, route)
					if !walks || route[0] != from || route[len(route)-1] != to {
						t.Errorf("the route %d → %d is no walk from one to the other", from, to)
						continue
					}
					ratio := cost / cheapest
					if ratio > 1.15 || ratio < 1-1e-9 {
						t.Errorf("the route %d → %d costs %.1f, %.3f of the cheapest",
							from, to, cost, ratio)
					}
					routes++
					sum, worst = sum+ratio, max(worst, ratio)
				}
			}
			if pairs := c.sources * c.targets; routes < pairs*9/10 {
				t.Fatalf("%d routes of %d pairs", routes, pairs)
			}
			mean := sum / float64(routes)
			t.Logf("%d routes over the cheapest: mean %.4f, max %.4f", routes, mean, worst)
			if mean > 1.04 {
				t.Errorf("the routes cost %.4f of the cheapest on the mean, want at most 1.04",
					mean)
			}
		})
	}
}

func TestScale_MemoryIsLinear(t *testing.T) {
	perCell := make(map[int]float64)
	for _, size := range []int{256, 1024} {
		m := testmap.New(1, size, scaleWalls, false)
		held := scaleHeld(func() any {
			// the trees kept on both maps, or the larger would drop them and look the leaner
			return scaleBuilt(t, m, withTreeMemory(1<<30))
		})
		perCell[size] = float64(held) / float64(size*size)
		t.Logf("%dx%d: %.1f MiB, %.1f bytes a cell", size, size, float64(held)/(1<<20),
			perCell[size])
		// the hierarchy as it came held 44 and 48 bytes a cell, 25 and 28 without its trees
		if perCell[size] < 32 || perCell[size] > 64 {
			t.Errorf("%dx%d holds %.1f bytes a cell, want 32 to 64", size, size, perCell[size])
		}
	}
	// the larger map's level more adds a little a cell; memory growing as n log n would add a
	// quarter
	if r := perCell[1024] / perCell[256]; r < 0.9 || r > 1.2 {
		t.Errorf("a cell of 1024x1024 holds %.2f of what one of 256x256 does, want 0.9 to 1.2", r)
	}
}
