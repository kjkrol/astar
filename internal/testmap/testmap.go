package testmap

import (
	"math"
	"os"
	"slices"
	"testing"

	"github.com/kjkrol/hpastar/graph"
)

// Map is a square map of cells for the tests and the benchmarks: each cell's share of full pace
// (0 a wall), reproducible from a seed; Wrap makes it a torus.
type Map struct {
	W, H  int
	Share []float32
	Wrap  bool
}

// New is a map of size × size cells: ground from smooth value noise with shares between 0.13
// and 1 (as rough ground to a road), walls on the share of cells walls says, laid as short lines
// so that pockets and detours form.
func New(seed uint64, size int, walls float64, wrap bool) *Map {
	m := &Map{W: size, H: size, Share: make([]float32, size*size), Wrap: wrap}
	const lattice = 16
	rnd := Splitmix(seed)
	cols := size/lattice + 2
	knots := make([]float32, cols*cols)
	for i := range knots {
		knots[i] = float32(rnd())
	}
	for y := range size {
		for x := range size {
			fx, fy := float32(x)/lattice, float32(y)/lattice
			ix, iy := int(fx), int(fy)
			tx, ty := fx-float32(ix), fy-float32(iy)
			k := func(dx, dy int) float32 { return knots[(iy+dy)*cols+ix+dx] }
			v := (k(0, 0)*(1-tx)+k(1, 0)*tx)*(1-ty) + (k(0, 1)*(1-tx)+k(1, 1)*tx)*ty
			m.Share[y*size+x] = 0.13 + 0.87*v*v
		}
	}
	for n := int(float64(size*size) * walls / 6); n > 0; n-- {
		x, y := int(rnd()*float64(size)), int(rnd()*float64(size))
		dx, dy := 0, 1
		if rnd() < 0.5 {
			dx, dy = 1, 0
		}
		for i := 0; i < 6 && x < size && y < size; i++ {
			m.Share[y*size+x] = 0
			x, y = x+dx, y+dy
		}
	}
	return m
}

// Splitmix is a deterministic source of numbers in [0, 1) for the maps.
func Splitmix(seed uint64) func() float64 {
	return func() float64 {
		seed += 0x9e3779b97f4a7c15
		z := seed
		z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9
		z = (z ^ (z >> 27)) * 0x94d049bb133111eb
		return float64(z^(z>>31)) / (1 << 64)
	}
}

// At is the cell at x, y, across the seams of a torus; false off a map with edges.
func (m *Map) At(x, y int) (int, bool) {
	if m.Wrap {
		x, y = (x+m.W)%m.W, (y+m.H)%m.H
	} else if x < 0 || y < 0 || x >= m.W || y >= m.H {
		return 0, false
	}
	return y*m.W + x, true
}

// Coords is where cell c lies; false for no cell of the map.
func (m *Map) Coords(c int) (x, y int, ok bool) {
	if c < 0 || c >= len(m.Share) {
		return 0, 0, false
	}
	return c % m.W, c / m.W, true
}

var octileSteps = [8][2]int{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}

// Forward is the map's transitions: eight directions, a step priced by its length over the
// share of the cell it enters, none into a wall.
func (m *Map) Forward(from, _ int, buf []graph.Transition[int]) []graph.Transition[int] {
	buf = buf[:0]
	x, y, _ := m.Coords(from)
	for i, s := range octileSteps {
		n, ok := m.At(x+s[0], y+s[1])
		if !ok || m.Share[n] == 0 {
			continue
		}
		length := 1.0
		if i >= 4 {
			length = math.Sqrt2
		}
		buf = append(buf, graph.Transition[int]{To: n, Cost: length / float64(m.Share[n])})
	}
	return buf
}

// Backward is Forward seen from the destination: the states a step leads from into to, each at
// the cost of that step.
func (m *Map) Backward(to, _ int, buf []graph.Transition[int]) []graph.Transition[int] {
	buf = buf[:0]
	if m.Share[to] == 0 {
		return buf
	}
	x, y, _ := m.Coords(to)
	for i, s := range octileSteps {
		n, ok := m.At(x-s[0], y-s[1])
		if !ok || m.Share[n] == 0 {
			continue
		}
		length := 1.0
		if i >= 4 {
			length = math.Sqrt2
		}
		buf = append(buf, graph.Transition[int]{To: n, Cost: length / float64(m.Share[to])})
	}
	return buf
}

// Octile is the octile distance, which never overestimates: no step costs less than its length.
func (m *Map) Octile(a, b int) float64 {
	ax, ay, _ := m.Coords(a)
	bx, by, _ := m.Coords(b)
	dx, dy := math.Abs(float64(ax-bx)), math.Abs(float64(ay-by))
	if m.Wrap {
		dx, dy = math.Min(dx, float64(m.W)-dx), math.Min(dy, float64(m.H)-dy)
	}
	return math.Max(dx, dy) + (math.Sqrt2-1)*math.Min(dx, dy)
}

// Graph is the map's graph laid out in the plane, a state y·W + x.
func (m *Map) Graph() graph.Implicit[int] {
	return graph.Implicit[int]{Forward: m.Forward, Heuristic: m.Octile, Index: Identity,
		Size: m.W * m.H, Layout: &graph.Layout[int]{Coords: m.Coords, At: m.States, Width: m.W,
			Height: m.H, Backward: m.Backward}}
}

// Identity is a cell's index, the cell itself.
func Identity(s int) int { return s }

// States appends the cell at x, y to buf, none for a wall.
func (m *Map) States(x, y int, buf []int) []int {
	if x >= 0 && y >= 0 && x < m.W && y < m.H && m.Share[y*m.W+x] > 0 {
		buf = append(buf, y*m.W+x)
	}
	return buf
}

// WallRing walls off the square of side cells at x0, y0 with a ring a cell thick, which no step
// crosses, slantwise either: its inside reaches nothing outside.
func (m *Map) WallRing(x0, y0, side int) {
	last := side - 1
	for i := range side {
		for _, c := range [4][2]int{{i, 0}, {i, last}, {0, i}, {last, i}} {
			if n, ok := m.At(x0+c[0], y0+c[1]); ok {
				m.Share[n] = 0
			}
		}
	}
}

// OpenNear is the open cell nearest x, y, looked for a ring of cells at a time; -1 where the map
// has none.
func (m *Map) OpenNear(x, y int) int {
	for r := range max(m.W, m.H) {
		for dy := -r; dy <= r; dy++ {
			for dx := -r; dx <= r; dx++ {
				if max(dx, -dx, dy, -dy) != r {
					continue
				}
				if n, ok := m.At(x+dx, y+dy); ok && m.Share[n] > 0 {
					return n
				}
			}
		}
	}
	return -1
}

// AnyOpen is an open cell drawn from rnd.
func (m *Map) AnyOpen(rnd func() float64) int {
	for {
		if c := int(rnd() * float64(len(m.Share))); m.Share[c] > 0 {
			return c
		}
	}
}

// Apart draws n open cells from rnd, each at least near from from by the octile distance and,
// where far is above 0, at most far.
func (m *Map) Apart(rnd func() float64, from, n int, near, far float64) []int {
	fx, fy, _ := m.Coords(from)
	draw := func() (int, bool) {
		if far <= 0 {
			return m.AnyOpen(rnd), true
		}
		// from the square round from: most draws from the whole map would land beyond far
		c, ok := m.At(fx+int((2*rnd()-1)*far), fy+int((2*rnd()-1)*far))
		return c, ok && m.Share[c] > 0
	}
	found := make([]int, 0, n)
	for len(found) < n {
		c, ok := draw()
		if !ok {
			continue
		}
		if d := m.Octile(from, c); d >= near && (far <= 0 || d <= far) {
			found = append(found, c)
		}
	}
	return found
}

// FarPair is two open cells near opposite corners, an eighth of the side in from each: a route
// across most of a map with edges.
func (m *Map) FarPair() [2]int {
	in := m.W / 8
	return [2]int{m.OpenNear(in, in), m.OpenNear(m.W-1-in, m.H-1-in)}
}

// Pairs are n pairs of open cells drawn from seed, lo to hi apart by the octile distance.
func (m *Map) Pairs(seed uint64, n int, lo, hi float64) [][2]int {
	rnd := Splitmix(seed)
	var pairs [][2]int
	for len(pairs) < n {
		a, b := int(rnd()*float64(m.W*m.H)), int(rnd()*float64(m.W*m.H))
		if d := m.Octile(a, b); m.Share[a] > 0 && m.Share[b] > 0 && d >= lo && d <= hi {
			pairs = append(pairs, [2]int{a, b})
		}
	}
	return pairs
}

// Gated is the map's graph with a gate past its cells, a state off the plane stepping only to
// at, and the gate.
func (m *Map) Gated(at int) (graph.Implicit[int], int) {
	g, gate := m.Graph(), m.W*m.H
	forward := g.Forward
	g.Forward = func(from, prev int, buf []graph.Transition[int]) []graph.Transition[int] {
		if from == gate {
			return append(buf[:0], graph.Transition[int]{To: at, Cost: 1})
		}
		return forward(from, prev, buf)
	}
	g.Size++
	return g, gate
}

// WalkCost is what route costs stepped along steps; false where a step is not a transition.
func WalkCost(steps graph.Transitions[int], route []int) (float64, bool) {
	total := 0.0
	var buf []graph.Transition[int]
	for i := 0; i+1 < len(route); i++ {
		buf = steps(route[i], route[i], buf[:0])
		k := slices.IndexFunc(buf, func(st graph.Transition[int]) bool {
			return st.To == route[i+1]
		})
		if k < 0 {
			return total, false
		}
		total += buf[k].Cost
	}
	return total, true
}

// SkipLarge skips tb on a map of 4096x4096 or more unless HPASTAR_SCALE=4096 asks for one: a
// hierarchy over it takes a minute and most of a gigabyte to build.
func SkipLarge(tb testing.TB, size int) {
	tb.Helper()
	if size >= 4096 && os.Getenv("HPASTAR_SCALE") != "4096" {
		tb.Skipf("a map of %dx%d: set HPASTAR_SCALE=4096 to run it", size, size)
	}
}
