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

// mirrored fails t where g's Backward does not list every step of its Forward between the
// states its Layout lists, at its cost and refused to its classes, and nothing more
func mirrored(t *testing.T, name string, g graph.Implicit[int]) {
	t.Helper()
	var states []int
	for y := range g.Layout.Height {
		for x := range g.Layout.Width {
			states = g.Layout.At(x, y, states)
		}
	}
	into := make(map[[2]int]graph.Transition[int])
	for _, s := range states {
		for _, step := range g.Forward(s, s, nil) {
			into[[2]int{s, step.To}] = step
		}
	}
	mirrored := 0
	for _, s := range states {
		for _, step := range g.Layout.Backward(s, s, nil) {
			fwd, ok := into[[2]int{step.To, s}]
			if !ok || fwd.Cost != step.Cost || fwd.Refuses != step.Refuses {
				t.Fatalf("%s: backward has %d → %d at %v refused %b, forward %+v (%v)", name,
					step.To, s, step.Cost, step.Refuses, fwd, ok)
			}
			mirrored++
		}
	}
	if mirrored != len(into) {
		t.Fatalf("%s: %d steps backward, %d forward", name, mirrored, len(into))
	}
}

func TestMaps_MasksMirrorBothWays(t *testing.T) {
	m := New(7, 64, 0.1, false)
	bars := m.Bars(1, 3, 0.2, nil)
	refuse := func(from, to int) uint8 { return uint8(Hashed(2, from, to) * 8) }
	mirrored(t, "barred", m.Barred(3, bars))
	mirrored(t, "hex", Masked(m.Hex(), 3, refuse))
	mirrored(t, "layered", Masked(m.Layered(20, 10, 30), 3, refuse))
	mirrored(t, "a class of hex", Filtered(Masked(m.Hex(), 3, refuse), 2))
}

func TestMaps_FilteredTakesWhatItsClassTakes(t *testing.T) {
	m := New(8, 64, 0.1, false)
	g := Masked(m.Graph(), 3, func(from, to int) uint8 { return uint8(Hashed(3, from, to) * 8) })
	for k := range 3 {
		f := Filtered(g, k)
		if f.Classes != 0 {
			t.Fatalf("class %d: a graph of %d classes", k, f.Classes)
		}
		for s := range 64 * 64 {
			var kept []graph.Transition[int]
			for _, step := range g.Forward(s, s, nil) {
				if step.Refuses&(1<<k) == 0 {
					step.Refuses = 0
					kept = append(kept, step)
				}
			}
			if got := f.Forward(s, s, nil); !slices.Equal(got, kept) {
				t.Fatalf("class %d, from %d: %v, want %v", k, s, got, kept)
			}
		}
	}
}

func TestMaps_BarsSpareClass0AndNest(t *testing.T) {
	m := New(9, 128, 0, false)
	bars := m.Bars(4, 4, 0.3, func(x, y int) bool { return x < 64 })
	barred := 0
	for s, b := range bars {
		x, _, _ := m.Coords(s)
		if b&1 != 0 || b != 0 && x >= 64 {
			t.Fatalf("state %d barred %04b", s, b)
		}
		// every class from the least barred up
		if b != 0 && b|(b-1) != 0b1111 {
			t.Fatalf("state %d barred %04b, classes apart", s, b)
		}
		if b != 0 {
			barred++
		}
	}
	if got := float64(barred) / float64(64*128); got < 0.25 || got > 0.35 {
		t.Fatalf("%.3f of the states within barred, want about 0.3", got)
	}
}
