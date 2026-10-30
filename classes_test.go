package hpastar_test

import (
	"fmt"
	"math"
	"slices"
	"sync"
	"testing"

	"github.com/kjkrol/hpastar"
	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/testmap"
)

// what a Search answers of a pair: the path into a dst that holds it and into a piece of 8, the
// reach, and what each of the three asks did
type answer struct {
	path, piece    []int
	found, reached bool
	did            [3]hpastar.Stats
}

func since(before, after hpastar.Stats) hpastar.Stats {
	return hpastar.Stats{Paths: after.Paths - before.Paths, Climbs: after.Climbs - before.Climbs,
		Exact: after.Exact - before.Exact, Expansions: after.Expansions - before.Expansions}
}

func ask(q *hpastar.Search[int], p [2]int, dst, piece []int) answer {
	var a answer
	s0 := q.Stats()
	n, found := q.Path(p[0], p[1], dst)
	s1 := q.Stats()
	k, _ := q.Path(p[0], p[1], piece)
	s2 := q.Stats()
	a.reached = q.Reaches(p[0], p[1])
	s3 := q.Stats()
	a.path, a.piece, a.found = slices.Clone(dst[:n]), slices.Clone(piece[:k]), found
	a.did = [3]hpastar.Stats{since(s0, s1), since(s1, s2), since(s2, s3)}
	return a
}

func (a answer) differs(b answer) string {
	switch {
	case a.found != b.found || !slices.Equal(a.path, b.path):
		return fmt.Sprintf("found %v, %d states; alone %v, %d states", a.found, len(a.path),
			b.found, len(b.path))
	case !slices.Equal(a.piece, b.piece):
		return fmt.Sprintf("a piece of %v; alone %v", a.piece, b.piece)
	case a.reached != b.reached:
		return fmt.Sprintf("reached %v; alone %v", a.reached, b.reached)
	case a.did != b.did:
		return fmt.Sprintf("did %+v; alone %+v", a.did, b.did)
	}
	return ""
}

// eachClassAlone fails t where a Solver over g asks a class otherwise than a Solver of one class
// over the steps that class takes: its hierarchy, and each pair's paths, reach and work. It
// reports how many pairs some class answered otherwise than class 0.
func eachClassAlone(t *testing.T, g graph.Implicit[int], pairs [][2]int, states int,
	opts ...hpastar.Option) int {
	t.Helper()
	all := hpastar.NewSolver(g, opts...)
	all.Update(0)
	q := all.Search()
	dst, piece := make([]int, states), make([]int, 8)
	first := make([]answer, len(pairs))
	apart := map[int]bool{}
	for k := range g.Classes {
		one := hpastar.NewSolver(testmap.Filtered(g, k), opts...)
		one.Update(0)
		if g.Layout != nil {
			if diff := hpastar.ClassDiffers(all, k, one); diff != "" {
				t.Fatalf("class %d: %s", k, diff)
			}
		}
		q.Class(k)
		alone := one.Search()
		for i, p := range pairs {
			got := ask(q, p, dst, piece)
			if diff := got.differs(ask(alone, p, dst, piece)); diff != "" {
				t.Fatalf("class %d, %d to %d: %s", k, p[0], p[1], diff)
			}
			if k == 0 {
				first[i] = got
			} else if !slices.Equal(got.path, first[i].path) || got.found != first[i].found {
				apart[i] = true
			}
		}
	}
	return len(apart)
}

// randomBars is a set of g's classes a state of m drawn from seed, any of them, for the states
// in where it says so with chance share
func randomBars(m *testmap.Map, seed uint64, classes int, share float64,
	in func(x, y int) bool) []uint8 {
	bars := make([]uint8, m.W*m.H)
	for s := range bars {
		if x, y, _ := m.Coords(s); in(x, y) && testmap.Hashed(seed, s, s) < share {
			bars[s] = uint8(testmap.Hashed(seed+1, s, 0) * float64(int(1)<<classes))
		}
	}
	return bars
}

func anywhere(_, _ int) bool { return true }

// the pairs a scene asks: of every distance, the far pair, and a few near ones
func scenePairs(m *testmap.Map, seed uint64) [][2]int {
	pairs := append(m.Pairs(seed, 40, 0, math.Inf(1)), m.FarPair())
	return append(pairs, m.Pairs(seed+1, 10, 2, 20)...)
}

func TestSolver_EachClassAsAGraphOfItsOwnSteps(t *testing.T) {
	const side = 16
	for _, c := range []struct {
		name    string
		classes int
		scene   func(classes int) (graph.Implicit[int], *testmap.Map)
		opts    []hpastar.Option
	}{
		{"within clusters", 3, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(31, 128, 0.06, false)
			bars := randomBars(m, 1, classes, 0.3, func(x, y int) bool {
				return min(x%side, side-1-x%side) >= 2 && min(y%side, side-1-y%side) >= 2
			})
			return m.Barred(classes, bars), m
		}, nil},
		{"on the crossings", 2, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(32, 128, 0.06, false)
			return testmap.Masked(m.Graph(), classes, func(from, to int) uint8 {
				if from%m.W/side == to%m.W/side && from/m.W/side == to/m.W/side ||
					testmap.Hashed(2, from, to) > 0.5 {
					return 0
				}
				return 0b10
			}), m
		}, nil},
		{"across the border alone", 2, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(33, 128, 0.02, false)
			// the steps along the first column and the first row of each cluster: what the
			// cluster before it sees across its border
			return testmap.Masked(m.Graph(), classes, func(from, to int) uint8 {
				fx, fy, _ := m.Coords(from)
				tx, ty, _ := m.Coords(to)
				if fx == tx && fx%side == 0 || fy == ty && fy%side == 0 {
					return 0b10
				}
				return 0
			}), m
		}, nil},
		{"one way", 3, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(34, 128, 0.06, false)
			return testmap.Masked(m.Graph(), classes, func(from, to int) uint8 {
				if testmap.Hashed(3, from, to) < 0.15 {
					return uint8(testmap.Hashed(4, from, to) * float64(int(1)<<classes))
				}
				return 0
			}), m
		}, nil},
		{"a ring shut to a class", 2, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(35, 128, 0.04, false)
			m.WallRing(40, 40, 40)
			gate := 40*m.W + 56
			m.Share[gate] = 1
			return testmap.Masked(m.Graph(), classes, func(_, to int) uint8 {
				if to == gate {
					return 0b10
				}
				return 0
			}), m
		}, nil},
		{"hex", 2, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(36, 128, 0.06, false)
			bars := randomBars(m, 5, classes, 0.08, anywhere)
			return testmap.Masked(m.Hex(), classes, func(_, to int) uint8 { return bars[to] }), m
		}, nil},
		{"torus", 2, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(37, 128, 0.06, true)
			return m.Barred(classes, m.Bars(6, classes, 0.08, nil)), m
		}, nil},
		{"8 classes", 8, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(38, 96, 0.06, false)
			return m.Barred(classes, m.Bars(7, classes, 0.15, nil)), m
		}, nil},
		{"no layout", 3, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(39, 64, 0.06, false)
			g := m.Barred(classes, randomBars(m, 8, classes, 0.1, anywhere))
			g.Layout = nil
			return g, m
		}, nil},
		{"no trees", 3, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(40, 128, 0.06, false)
			return m.Barred(classes, randomBars(m, 9, classes, 0.1, anywhere)), m
		}, []hpastar.Option{hpastar.WithTreeMemory(0)}},
		{"a third of the trees", 3, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(42, 128, 0.06, false)
			return m.Barred(classes, randomBars(m, 11, classes, 0.1, anywhere)), m
		}, []hpastar.Option{hpastar.WithTreeMemory(100 << 10)}},
		{"three levels", 3, func(classes int) (graph.Implicit[int], *testmap.Map) {
			m := testmap.New(41, 128, 0.06, false)
			return m.Barred(classes, m.Bars(10, classes, 0.1, nil)), m
		}, []hpastar.Option{hpastar.WithClusterSide(4)}},
	} {
		t.Run(c.name, func(t *testing.T) {
			g, m := c.scene(c.classes)
			pairs := scenePairs(m, 11)
			if c.name == "a ring shut to a class" {
				inside, outside := m.OpenNear(60, 60), m.OpenNear(10, 10)
				pairs = append(pairs, [2]int{inside, outside}, [2]int{outside, inside},
					[2]int{inside, m.OpenNear(70, 50)})
			}
			if apart := eachClassAlone(t, g, pairs, 4*m.W*m.H, c.opts...); apart == 0 {
				t.Fatal("every class answered every pair alike; the scene wants classes apart")
			}
		})
	}
}

func TestSolver_TwoStatesAtAPlaceApartByClass(t *testing.T) {
	m := testmap.New(14, 96, 0.06, false)
	// a river down x 46..49, forded at row 20 below a stretch of states over it: class 1 takes
	// the ford alone, class 2 the stretch above it alone
	for y := range m.H {
		for x := 46; x <= 49; x++ {
			m.Share[y*m.W+x] = 0
		}
	}
	for x := 45; x <= 50; x++ {
		m.Share[20*m.W+x] = 0.15
	}
	m.Share[20*m.W+45], m.Share[20*m.W+50] = 1, 1
	above := m.W * m.H
	g := testmap.Masked(m.Layered(20, 46, 49), 3, func(_, to int) uint8 {
		switch x := to % m.W; {
		case to >= above:
			return 0b010
		case to/m.W == 20 && x >= 46 && x <= 49:
			return 0b100
		}
		return 0
	})
	var pairs [][2]int
	for _, p := range m.Pairs(15, 200, 32, math.Inf(1)) {
		if a, b := p[0]%m.W, p[1]%m.W; (a < 46 && b > 49 || a > 49 && b < 46) && len(pairs) < 40 {
			pairs = append(pairs, p)
		}
	}
	if apart := eachClassAlone(t, g, pairs, 2*m.W*m.H); apart == 0 {
		t.Fatal("every class crossed the river alike")
	}
	q := hpastar.NewSolver(g)
	q.Update(0)
	dst := make([]int, 2*m.W*m.H)
	for k, wants := range []func(s int) bool{nil, func(s int) bool { return s >= above },
		func(s int) bool { return s < above && s/m.W == 20 && s%m.W >= 46 && s%m.W <= 49 }} {
		s := q.Search()
		s.Class(k)
		for _, p := range pairs {
			n, _ := s.Path(p[0], p[1], dst)
			if wants != nil && slices.ContainsFunc(dst[:n], wants) {
				t.Fatalf("class %d, %d to %d: a path through the states it refuses", k, p[0],
					p[1])
			}
		}
	}
}

func TestSolver_OneClassReadsNoRefuses(t *testing.T) {
	m := testmap.New(41, 128, 0.06, false)
	plain := m.Graph()
	for _, classes := range []int{0, 1} {
		g := testmap.Masked(m.Graph(), classes, func(from, to int) uint8 {
			return uint8(testmap.Hashed(9, from, to) * 256)
		})
		a, b := hpastar.NewSolver(g), hpastar.NewSolver(plain)
		a.Update(0)
		b.Update(0)
		if diff := hpastar.GraphsDiffer(a, b); diff != "" {
			t.Fatalf("%d classes: %s against the graph with no step refused", classes, diff)
		}
		qa, qb := a.Search(), b.Search()
		dst, piece := make([]int, 4*m.W*m.H), make([]int, 8)
		for _, p := range scenePairs(m, 12) {
			if diff := ask(qa, p, dst, piece).differs(ask(qb, p, dst, piece)); diff != "" {
				t.Fatalf("%d classes, %d to %d: %s", classes, p[0], p[1], diff)
			}
		}
	}
}

func TestSolver_TheBitsPastTheClassesAreUnread(t *testing.T) {
	m := testmap.New(42, 128, 0.06, false)
	refuse := func(from, to int) uint8 {
		if testmap.Hashed(10, from, to) < 0.2 {
			return 0b10
		}
		return 0
	}
	noisy := testmap.Masked(m.Graph(), 2, func(from, to int) uint8 {
		return refuse(from, to) | uint8(testmap.Hashed(11, from, to)*256)&^0b11
	})
	a, b := hpastar.NewSolver(noisy), hpastar.NewSolver(testmap.Masked(m.Graph(), 2, refuse))
	a.Update(0)
	b.Update(0)
	if diff := hpastar.GraphsDiffer(a, b); diff != "" {
		t.Fatalf("against the bits of the classes alone: %s", diff)
	}
	qa, qb := a.Search(), b.Search()
	dst, piece := make([]int, 4*m.W*m.H), make([]int, 8)
	for k := range 2 {
		qa.Class(k)
		qb.Class(k)
		for _, p := range scenePairs(m, 13) {
			if diff := ask(qa, p, dst, piece).differs(ask(qb, p, dst, piece)); diff != "" {
				t.Fatalf("class %d, %d to %d: %s", k, p[0], p[1], diff)
			}
		}
	}
}

func TestSolver_ClassesOutOfRangePanic(t *testing.T) {
	m := testmap.New(43, 32, 0, false)
	panics := func(what string, f func()) {
		t.Helper()
		defer func() {
			if recover() == nil {
				t.Fatalf("%s: no panic", what)
			}
		}()
		f()
	}
	for _, classes := range []int{-1, 9} {
		g := m.Graph()
		g.Classes = classes
		panics(fmt.Sprintf("a graph of %d classes", classes), func() { hpastar.NewSolver(g) })
	}
	for _, c := range []struct{ classes, k int }{{0, 1}, {1, 1}, {3, 3}, {3, -1}, {8, 8}} {
		g := m.Graph()
		g.Classes = c.classes
		q := hpastar.NewSolver(g).Search()
		panics(fmt.Sprintf("class %d of %d", c.k, c.classes), func() { q.Class(c.k) })
	}
	g := m.Graph()
	g.Classes = 8
	q := hpastar.NewSolver(g).Search()
	q.Class(7)
	q.Class(0)
}

func TestSearch_ClassesAllocateNothingOnceWarm(t *testing.T) {
	const classes = 3
	m := testmap.New(44, 128, 0.05, false)
	pairs := pairsOf(m, 16)
	g := m.Barred(classes, m.Bars(14, classes, 0.1, nil))
	plain := g
	plain.Layout = nil
	for name, s := range map[string]*hpastar.Solver[int]{
		"HPA*": hpastar.NewSolver(g),
		"A*":   hpastar.NewSolver(plain),
	} {
		s.Update(0)
		q := s.Search()
		dst, short := make([]int, 4*128), make([]int, 8)
		asks := func() {
			for k := range classes {
				q.Class(k)
				for _, p := range pairs {
					q.Path(p[0], p[1], dst)
					q.Path(p[0], p[1], short)
					q.Reaches(p[0], p[1])
				}
			}
		}
		asks()
		if allocs := testing.AllocsPerRun(2, asks); allocs != 0 {
			t.Fatalf("%s: Path, Reaches and Class allocate %v times once warm", name, allocs)
		}
		if st := q.Stats(); name == "HPA*" && (st.Climbs == 0 || st.Exact == 0) {
			t.Fatalf("%s: the pairs asked %+v; want some climbed and some found by A*", name, st)
		}
	}
}

func TestSearch_ClassesAskAtOnce(t *testing.T) {
	const goroutines, each, classes = 8, 30, 3
	m := testmap.New(45, 128, 0.08, false)
	pairs := pairsOf(m, goroutines*each)
	g := m.Barred(classes, m.Bars(15, classes, 0.1, nil))
	s := hpastar.NewSolver(g)
	s.Update(0)
	want := make([]answer, len(pairs))
	q := s.Search()
	dst, piece := make([]int, 4*128), make([]int, 8)
	for i, p := range pairs {
		q.Class(i / each % classes)
		want[i] = ask(q, p, dst, piece)
	}
	got := make([]answer, len(pairs))
	var wg sync.WaitGroup
	for r := range goroutines {
		wg.Go(func() {
			// half the goroutines read the graph through transitions of their own
			q := s.Search()
			if r%2 == 1 {
				own := m.Barred(classes, m.Bars(15, classes, 0.1, nil))
				q = s.SearchVia(own.Forward, own.Layout.Backward)
			}
			q.Class(r % classes)
			dst, piece := make([]int, 4*128), make([]int, 8)
			for i := r * each; i < (r+1)*each; i++ {
				got[i] = ask(q, pairs[i], dst, piece)
			}
		})
	}
	wg.Wait()
	for i, p := range pairs {
		if diff := got[i].differs(want[i]); diff != "" {
			t.Fatalf("class %d, %v asked at once: %s", i/each%classes, p, diff)
		}
	}
}
