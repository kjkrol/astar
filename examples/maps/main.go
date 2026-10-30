// Maps sets a Solver with a Layout, whose long paths HPA* finds, against one without, whose
// paths A* finds, on a map generated from a seed: the hierarchy's build, then each pair's time,
// expansions and cost, and the pairs no path joins.
package main

import (
	"flag"
	"fmt"
	"math"
	"os"
	"runtime"
	"slices"
	"time"

	"github.com/kjkrol/hpastar"
	"github.com/kjkrol/hpastar/graph"
)

// the least octile distance between the two cells of a pair: nearer, HPA* leaves a path to A*
const apart = 64

func main() {
	size := flag.Int("size", 1024, "the map's side in cells")
	side := flag.Int("side", 16, "the side of a cluster of the hierarchy's first level, in cells")
	pairs := flag.Int("pairs", 100,
		"random pairs of open cells at least 64 apart; A* takes seconds a pair on 4096")
	seed := flag.Uint64("seed", 1, "the seed of the map and of the pairs")
	walls := flag.Float64("walls", 0.05, "the share of the cells that are walls")
	islands := flag.Int("islands", 4,
		"squares walled off, each the end of a pair no path joins, which A* floods for")
	wrap := flag.Bool("wrap", false, "join the map's opposite edges: a torus")
	flag.Parse()
	if *size < 2*apart {
		fmt.Fprintf(os.Stderr, "maps: a map below %d cells a side leaves no pair %d apart\n",
			2*apart, apart)
		os.Exit(2)
	}

	m := genMap(*seed, *size, *walls, *wrap)
	rnd := splitmix(*seed + 1)
	isles := m.isles(rnd, *islands)
	m.describe(*seed, len(isles))

	plain := m.graph()
	plain.Layout = nil
	exact := hpastar.NewSolver(plain).Search()
	fmt.Printf("HPA*, clusters of %d cells a side: ", *side)
	hpa := build(m, *side).Search()

	all := m.pairs(rnd, *pairs, isles)
	fmt.Printf("%d pairs at least %d apart, %d of them into an island:\n",
		len(all), apart, len(isles))
	var c comparison
	c.run(m, exact, hpa, all)
	c.print(m)
}

// build makes the Solver with m's Layout and builds its hierarchy whole, printing the time it
// took and the memory it holds.
func build(m *gridMap, side int) *hpastar.Solver[int] {
	var before, after runtime.MemStats
	runtime.GC()
	runtime.ReadMemStats(&before)
	start := time.Now()
	s := hpastar.NewSolver(m.graph(), hpastar.WithClusterSide(side))
	s.Update(0)
	took := time.Since(start)
	runtime.GC()
	runtime.ReadMemStats(&after)
	held := max(int64(after.HeapAlloc)-int64(before.HeapAlloc), 0)
	ready := "ready"
	if !s.Ready() {
		ready = "not ready"
	}
	fmt.Printf("built in %v, %s; %.1f MiB held, %.1f bytes a cell; %.1f MiB allocated\n",
		short(took), ready, mib(held), float64(held)/float64(len(m.share)),
		mib(int64(after.TotalAlloc-before.TotalAlloc)))
	return s
}

// tally is what a kind of search took over the pairs it ran on.
type tally struct {
	n          int
	took       time.Duration
	expansions uint64
}

func (t *tally) add(took time.Duration, expansions uint64) {
	t.n++
	t.took += took
	t.expansions += expansions
}

func (t tally) mean() time.Duration {
	return t.took / time.Duration(max(t.n, 1))
}

func (t tally) String() string {
	if t.n == 0 {
		return "none"
	}
	return fmt.Sprintf("%v and %d expansions a pair", short(t.mean()), t.expansions/uint64(t.n))
}

// comparison is what A* and HPA* made of the pairs.
type comparison struct {
	joined, unjoined tally  // A* on the pairs a path joins and on the others
	whole, first     tally  // HPA*, the whole path and its first 64 states
	denied           tally  // HPA* on the pairs no path joins
	asked, climbs    uint64 // the paths asked of HPA*, and those it climbed
	exact            uint64 // the paths it left to A*
	broken, wrong    int    // paths HPA* wrote that are no walk, answers that differ from A*'s
	reached          int    // pairs a path joins that Reaches joins
	sum, worst       float64
	worstPair        [2]int
}

func (c *comparison) run(m *gridMap, exact, hpa *hpastar.Search[int], all [][2]int) {
	var path, route []int
	first := make([]int, apart)
	before := hpa.Stats()
	for _, p := range all {
		path = path[:cap(path)]
		start, e := time.Now(), exact.Stats().Expansions
		path, joined := whole(exact, p[0], p[1], path)
		took, expanded := time.Since(start), exact.Stats().Expansions-e

		start, e = time.Now(), hpa.Stats().Expansions
		route, found := whole(hpa, p[0], p[1], route)
		climbing, climbed := time.Since(start), hpa.Stats().Expansions-e
		if found != joined {
			c.wrong++
		}
		if !joined {
			c.unjoined.add(took, expanded)
			c.denied.add(climbing, climbed)
			continue
		}
		c.joined.add(took, expanded)
		c.whole.add(climbing, climbed)
		if hpa.Reaches(p[0], p[1]) {
			c.reached++
		}
		start, e = time.Now(), hpa.Stats().Expansions
		hpa.Path(p[0], p[1], first)
		c.first.add(time.Since(start), hpa.Stats().Expansions-e)

		cost, walks := m.cost(route)
		if !walks || route[0] != p[0] || route[len(route)-1] != p[1] {
			c.broken++
			continue
		}
		cheapest, _ := m.cost(path)
		ratio := cost / cheapest
		c.sum += ratio
		if ratio > c.worst {
			c.worst, c.worstPair = ratio, p
		}
	}
	after := hpa.Stats()
	c.asked, c.climbs = after.Paths-before.Paths, after.Climbs-before.Climbs
	c.exact = after.Exact - before.Exact
}

func (c *comparison) print(m *gridMap) {
	fmt.Printf("  %d joined, A*: %v\n", c.joined.n, c.joined)
	fmt.Printf("    HPA*, the whole path: %v, %.0fx as fast\n", c.whole,
		float64(c.joined.mean())/float64(max(c.whole.mean(), 1)))
	fmt.Printf("    HPA*, its first %d states: %v\n", apart, c.first)
	if c.broken > 0 || c.wrong > 0 {
		fmt.Printf("    HPA* wrote %d paths that are no walk, answered %d pairs unlike A*\n",
			c.broken, c.wrong)
	}
	if n := c.whole.n - c.broken; n > 0 {
		x0, y0, _ := m.coords(c.worstPair[0])
		x1, y1, _ := m.coords(c.worstPair[1])
		fmt.Printf("    cost over the cheapest: mean %.4f, max %.4f", c.sum/float64(n), c.worst)
		fmt.Printf(" from (%d, %d) to (%d, %d)\n", x0, y0, x1, y1)
	}
	fmt.Printf("    of the %d paths asked of HPA*: %d climbed, %d left to A*\n",
		c.asked, c.climbs, c.exact)
	fmt.Printf("    Reaches: joined for %d of %d\n", c.reached, c.joined.n)
	fmt.Printf("  %d unjoined, A* to learn it: %v\n", c.unjoined.n, c.unjoined)
	if c.unjoined.n > 0 {
		fmt.Printf("    HPA* to learn it, off the labels: %v\n", c.denied)
	}
}

// whole is the whole path s writes from from to to, into dst grown until it holds it.
func whole(s *hpastar.Search[int], from, to int, dst []int) ([]int, bool) {
	dst = dst[:cap(dst)]
	if len(dst) == 0 {
		dst = make([]int, 1024)
	}
	for {
		n, ok := s.Path(from, to, dst)
		if !ok || n < len(dst) || dst[n-1] == to {
			return dst[:n], ok
		}
		dst = make([]int, 2*len(dst))
	}
}

// short is d to three significant digits.
func short(d time.Duration) time.Duration {
	unit := time.Duration(1)
	for d/unit >= 1000 {
		unit *= 10
	}
	return d.Round(unit)
}

func mib(bytes int64) float64 { return float64(bytes) / (1 << 20) }

// island is a square walled off by a ring, and an open cell inside it.
type island struct {
	x0, y0, side, inside int
}

func (is island) holds(m *gridMap, c int) bool {
	x, y, _ := m.coords(c)
	return x >= is.x0 && y >= is.y0 && x < is.x0+is.side && y < is.y0+is.side
}

// isles walls off n squares at random, each with an open cell inside; a square whose inside
// another's ring walled up is dropped.
func (m *gridMap) isles(rnd func() float64, n int) []island {
	isles := make([]island, 0, n)
	for range n {
		side := 12 + int(rnd()*20)
		x0, y0 := int(rnd()*float64(m.w-side)), int(rnd()*float64(m.h-side))
		m.wallRing(x0, y0, side)
		isles = append(isles, island{x0: x0, y0: y0, side: side, inside: -1})
	}
	for i, is := range isles {
		for y := is.y0 + 1; y < is.y0+is.side-1 && isles[i].inside < 0; y++ {
			for x := is.x0 + 1; x < is.x0+is.side-1; x++ {
				if c := y*m.w + x; m.share[c] > 0 {
					isles[i].inside = c
					break
				}
			}
		}
	}
	return slices.DeleteFunc(isles, func(is island) bool { return is.inside < 0 })
}

// pairs draws n pairs of open cells at least apart, then one from out of every island into it.
func (m *gridMap) pairs(rnd func() float64, n int, isles []island) [][2]int {
	all := make([][2]int, 0, n+len(isles))
	for len(all) < n {
		if from, to := m.anyOpen(rnd), m.anyOpen(rnd); m.octile(from, to) >= apart {
			all = append(all, [2]int{from, to})
		}
	}
	for _, is := range isles {
		for {
			if from := m.anyOpen(rnd); !is.holds(m, from) && m.octile(from, is.inside) >= apart {
				all = append(all, [2]int{from, is.inside})
				break
			}
		}
	}
	return all
}

func (m *gridMap) describe(seed uint64, isles int) {
	walls := 0
	for _, s := range m.share {
		if s == 0 {
			walls++
		}
	}
	edges := "edges"
	if m.wrap {
		edges = "a torus"
	}
	fmt.Printf("map %dx%d from seed %d, %s: %.1f%% of the cells walls, %d islands\n",
		m.w, m.h, seed, edges, 100*float64(walls)/float64(len(m.share)), isles)
}

// cost is what route costs a step at a time; false where a step is none of the map's.
func (m *gridMap) cost(route []int) (float64, bool) {
	var cost float64
	var buf []graph.Transition[int]
	for i := 1; i < len(route); i++ {
		buf = m.forward(route[i-1], route[i-1], buf)
		j := slices.IndexFunc(buf, func(s graph.Transition[int]) bool { return s.To == route[i] })
		if j < 0 {
			return 0, false
		}
		cost += buf[j].Cost
	}
	return cost, true
}

// What follows is a copy of the tests' maps (internal/testmap), so that the example stands on the
// public API alone: keep the two in step, so a seed makes the same map here as in the tests.

// gridMap is a square map of cells: each cell's share of full pace (0 a wall); wrap makes it a
// torus.
type gridMap struct {
	w, h  int
	share []float32
	wrap  bool
}

// genMap is a map of size × size cells: ground from smooth value noise with shares between
// 0.13 and 1 (as rough ground to a road), walls on the share of cells walls says, laid as
// short lines so that pockets and detours form.
func genMap(seed uint64, size int, walls float64, wrap bool) *gridMap {
	m := &gridMap{w: size, h: size, share: make([]float32, size*size), wrap: wrap}
	const lattice = 16
	rnd := splitmix(seed)
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
			m.share[y*size+x] = 0.13 + 0.87*v*v
		}
	}
	for n := int(float64(size*size) * walls / 6); n > 0; n-- {
		x, y := int(rnd()*float64(size)), int(rnd()*float64(size))
		dx, dy := 0, 1
		if rnd() < 0.5 {
			dx, dy = 1, 0
		}
		for i := 0; i < 6 && x < size && y < size; i++ {
			m.share[y*size+x] = 0
			x, y = x+dx, y+dy
		}
	}
	return m
}

// splitmix is a deterministic source of numbers in [0, 1) for the maps.
func splitmix(seed uint64) func() float64 {
	return func() float64 {
		seed += 0x9e3779b97f4a7c15
		z := seed
		z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9
		z = (z ^ (z >> 27)) * 0x94d049bb133111eb
		return float64(z^(z>>31)) / (1 << 64)
	}
}

func (m *gridMap) at(x, y int) (int, bool) {
	if m.wrap {
		x, y = (x+m.w)%m.w, (y+m.h)%m.h
	} else if x < 0 || y < 0 || x >= m.w || y >= m.h {
		return 0, false
	}
	return y*m.w + x, true
}

func (m *gridMap) coords(c int) (x, y int, ok bool) {
	if c < 0 || c >= len(m.share) {
		return 0, 0, false
	}
	return c % m.w, c / m.w, true
}

var octileSteps = [8][2]int{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}

// forward is the map's transitions: eight directions, a step priced by its length over the
// share of the cell it enters, none into a wall.
func (m *gridMap) forward(from, _ int, buf []graph.Transition[int]) []graph.Transition[int] {
	buf = buf[:0]
	x, y, _ := m.coords(from)
	for i, s := range octileSteps {
		n, ok := m.at(x+s[0], y+s[1])
		if !ok || m.share[n] == 0 {
			continue
		}
		length := 1.0
		if i >= 4 {
			length = math.Sqrt2
		}
		buf = append(buf, graph.Transition[int]{To: n, Cost: length / float64(m.share[n])})
	}
	return buf
}

// backward is forward seen from the destination: the states a step leads from into to, each at
// the cost of that step.
func (m *gridMap) backward(to, _ int, buf []graph.Transition[int]) []graph.Transition[int] {
	buf = buf[:0]
	if m.share[to] == 0 {
		return buf
	}
	x, y, _ := m.coords(to)
	for i, s := range octileSteps {
		n, ok := m.at(x-s[0], y-s[1])
		if !ok || m.share[n] == 0 {
			continue
		}
		length := 1.0
		if i >= 4 {
			length = math.Sqrt2
		}
		buf = append(buf, graph.Transition[int]{To: n, Cost: length / float64(m.share[to])})
	}
	return buf
}

// octile never overestimates: no step costs less than its length.
func (m *gridMap) octile(a, b int) float64 {
	ax, ay, _ := m.coords(a)
	bx, by, _ := m.coords(b)
	dx, dy := math.Abs(float64(ax-bx)), math.Abs(float64(ay-by))
	if m.wrap {
		dx, dy = math.Min(dx, float64(m.w)-dx), math.Min(dy, float64(m.h)-dy)
	}
	return math.Max(dx, dy) + (math.Sqrt2-1)*math.Min(dx, dy)
}

// graph is the map's graph laid out in the plane, a state y·w + x.
func (m *gridMap) graph() graph.Implicit[int] {
	return graph.Implicit[int]{Forward: m.forward, Heuristic: m.octile,
		Index: func(c int) int { return c }, Size: m.w * m.h,
		Layout: &graph.Layout[int]{Coords: m.coords, At: m.states, Width: m.w, Height: m.h,
			Backward: m.backward}}
}

// states appends the cell at x, y to buf, none for a wall
func (m *gridMap) states(x, y int, buf []int) []int {
	if x >= 0 && y >= 0 && x < m.w && y < m.h && m.share[y*m.w+x] > 0 {
		buf = append(buf, y*m.w+x)
	}
	return buf
}

// wallRing walls off the square of side cells at x0, y0 with a ring a cell thick, which no step
// crosses, slantwise either: its inside reaches nothing outside.
func (m *gridMap) wallRing(x0, y0, side int) {
	last := side - 1
	for i := range side {
		for _, c := range [4][2]int{{i, 0}, {i, last}, {0, i}, {last, i}} {
			if n, ok := m.at(x0+c[0], y0+c[1]); ok {
				m.share[n] = 0
			}
		}
	}
}

// anyOpen is an open state drawn from rnd.
func (m *gridMap) anyOpen(rnd func() float64) int {
	for {
		if c := int(rnd() * float64(len(m.share))); m.share[c] > 0 {
			return c
		}
	}
}
