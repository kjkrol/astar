package testmap

import (
	"slices"
	"testing"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/astar"
	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/dijkstra"
)

func TestMaps_AreReproducibleBySeed(t *testing.T) {
	a, b := New(7, 256, 0.1, true), New(7, 256, 0.1, true)
	if !slices.Equal(a.Share, b.Share) {
		t.Fatal("two maps from seed 7 differ")
	}
	if c := New(8, 256, 0.1, true); slices.Equal(a.Share, c.Share) {
		t.Fatal("seeds 7 and 8 made the same map")
	}
}

func TestMaps_SharesLieInRange(t *testing.T) {
	m := New(1, 256, 0, false)
	lo, hi := float32(1), float32(0)
	for _, s := range m.Share {
		lo, hi = min(lo, s), max(hi, s)
	}
	if lo < 0.13 || hi > 1 {
		t.Fatalf("shares from %v to %v, want within [0.13, 1]", lo, hi)
	}
	// the noise spans the range, rough ground and roads both
	if lo > 0.2 || hi < 0.8 {
		t.Fatalf("shares from %v to %v, want below 0.2 and above 0.8 somewhere", lo, hi)
	}
}

func TestMaps_WallsAreImpassable(t *testing.T) {
	const size, walls = 256, 0.1
	m := New(3, size, walls, false)
	var buf []graph.Transition[int]
	zeros := 0
	for c, s := range m.Share {
		if s == 0 {
			zeros++
			if buf = m.Backward(c, c, buf); len(buf) > 0 {
				t.Fatalf("a step from %d leads into the wall at %d", buf[0].To, c)
			}
		}
		buf = m.Forward(c, c, buf)
		for _, step := range buf {
			if m.Share[step.To] == 0 {
				t.Fatalf("a step from %d enters the wall at %d", c, step.To)
			}
		}
	}
	// lines overlapping and cut at the edges leave a little fewer than asked for
	if got := float64(zeros) / (size * size); got < 0.8*walls || got > walls {
		t.Fatalf("%.4f of the cells walls, want a little under %v", got, walls)
	}
	const x0, y0, side = 100, 100, 20
	m.WallRing(x0, y0, side)
	inside := m.OpenNear(x0+side/2, y0+side/2)
	within := func(c int) bool {
		x, y, _ := m.Coords(c)
		return x > x0 && y > y0 && x < x0+side-1 && y < y0+side-1
	}
	if !within(inside) {
		t.Fatalf("no open cell inside the ring")
	}
	d := dijkstra.New[int](dict.NewIndexedSlice(size*size, Identity), 0)
	d.Fill([]dijkstra.Source[int]{{At: inside}}, m.Forward, nil, nil)
	for c := range size * size {
		if _, settled := d.Cost(c); settled && !within(c) {
			x, y, _ := m.Coords(c)
			t.Fatalf("a route from inside the ring reaches (%d, %d)", x, y)
		}
	}
}

func TestMaps_TheTorusWraps(t *testing.T) {
	const size = 64
	edged, torus := New(2, size, 0, false), New(2, size, 0, true)
	if !slices.Equal(edged.Share, torus.Share) {
		t.Fatal("the torus has other ground than the map with edges")
	}
	joins := func(m *Map, a, b int) bool {
		for _, s := range m.Forward(a, a, nil) {
			if s.To == b {
				return true
			}
		}
		return false
	}
	west, east := 10*size, 10*size+size-1
	corner, across := 0, size*size-1
	if !joins(torus, west, east) || !joins(torus, corner, across) {
		t.Fatal("no step across the torus's seam")
	}
	if joins(edged, west, east) || joins(edged, corner, across) {
		t.Fatal("a step across the edge of a map with edges")
	}
	if got := torus.Octile(west, east); got != 1 {
		t.Fatalf("the octile distance across the seam %v, want 1", got)
	}
	if got := edged.Octile(west, east); got != size-1 {
		t.Fatalf("the octile distance across a map with edges %v, want %d", got, size-1)
	}
	a := astar.New[int](torus.Octile, dict.NewMap[int](0), 0)
	if path := a.SolveInto(nil, west, east, torus.Forward); len(path) != 2 {
		t.Fatalf("the route across the seam %v, want one step", path)
	}
}

func TestMaps_BackwardMirrorsForward(t *testing.T) {
	for _, wrap := range []bool{false, true} {
		m := New(5, 64, 0.1, wrap)
		into := make(map[[2]int]float64)
		for c, s := range m.Share {
			if s == 0 {
				continue
			}
			for _, step := range m.Forward(c, c, nil) {
				into[[2]int{c, step.To}] = step.Cost
			}
		}
		mirrored := 0
		for c := range m.Share {
			for _, step := range m.Backward(c, c, nil) {
				if cost, ok := into[[2]int{step.To, c}]; !ok || cost != step.Cost {
					t.Fatalf("wrap %v: backward has %d → %d at %v, forward at %v (%v)",
						wrap, step.To, c, step.Cost, cost, ok)
				}
				mirrored++
			}
		}
		if mirrored != len(into) {
			t.Fatalf("wrap %v: %d steps backward, %d forward", wrap, mirrored, len(into))
		}
	}
}

func TestMaps_OctileNeverOverestimatesAStep(t *testing.T) {
	for _, wrap := range []bool{false, true} {
		m := New(6, 64, 0.1, wrap)
		for c := range m.Share {
			for _, step := range m.Forward(c, c, nil) {
				if h := m.Octile(c, step.To); h > step.Cost {
					t.Fatalf("wrap %v: octile %d → %d is %v, the step costs %v",
						wrap, c, step.To, h, step.Cost)
				}
			}
		}
	}
}
