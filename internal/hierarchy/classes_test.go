package hierarchy

import (
	"math"
	"slices"
	"testing"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/testmap"
)

// classClimb is a climb of h for class k over the steps of g that k takes
func classClimb[T comparable](h *Hierarchy[T], g graph.Implicit[T], k int) *Climb[T] {
	f := testmap.Filtered(g, k)
	s := h.NewClimb(f.Forward, f.Layout.Backward)
	s.Class(k)
	return s
}

// states walled and opened, and the classes a state is closed to changed, at random, on the
// borders of clusters most often and a line at a time now and then, the gate of a ring across
// clusters shut to a class and opened, built under budgets drawn at random: the graph a whole
// build gives every class, the labels of each parting its nodes as a whole labelling does and
// telling apart only ends A* over the class's steps finds apart
func TestHierarchy_ClassesFollowEveryChange(t *testing.T) {
	for _, wrap := range []bool{false, true} {
		const size, classes = 80, 3
		m := testmap.New(21, size, 0.08, wrap)
		m.WallRing(20, 20, 30)
		gate := 20*size + 32
		m.Share[gate] = 1
		// a state closed to a class both ways, as a wall is: the labels join steps either way
		bars := make([]uint8, size*size)
		g := testmap.Masked(m.Graph(), classes, func(from, to int) uint8 {
			return bars[from] | bars[to]
		})
		h := testHierarchy(g)
		h.Build(0)
		exact := mapAStar(m)
		rnd := testmap.Splitmix(5)
		change := func(s int) {
			if rnd() < 0.5 {
				bars[s] = uint8(rnd() * (1 << classes))
			} else if m.Share[s] == 0 {
				m.Share[s] = float32(0.3 + 0.7*rnd())
			} else {
				m.Share[s] = 0
			}
			h.Stale(s)
		}
		for round := range 24 {
			if round%3 == 0 {
				bars[gate] ^= 1 << (round / 3 % classes)
				h.Stale(gate)
			}
			for range 1 + int(rnd()*3) {
				x, y := int(rnd()*size), int(rnd()*size)
				if rnd() < 0.6 {
					x = x/16*16 + int(rnd()*2)*15
				}
				change(y*size + x)
				if rnd() < 0.2 {
					for k := 1; k < 6 && y+k < size; k++ {
						change((y+k)*size + x)
					}
				}
			}
			for calls := 0; !h.Ready(); calls++ {
				if calls > 100000 {
					t.Fatal("the build makes no progress")
				}
				h.Build(1 + int(rnd()*20000))
			}
			fresh := testHierarchy(g)
			fresh.Build(0)
			sameGraph(t, h.Hierarchy, fresh.Hierarchy)
			pairs := m.Pairs(uint64(round), 10, 0, math.Inf(1))
			pairs = append(pairs, [2]int{m.OpenNear(35, 35), m.OpenNear(5, 5)})
			for k := range classes {
				samePartition(t, h.Hierarchy, k, fresh.Hierarchy, k)
				s, steps := classClimb(h.Hierarchy, g, k), testmap.Filtered(g, k).Forward
				for _, p := range pairs {
					want := exact.SolveInto(nil, p[0], p[1], steps) != nil
					if got := s.Reaches(p[0], p[1]); got != want {
						t.Fatalf("wrap %v, round %d, class %d: reaches(%d, %d) %v, A* %v", wrap,
							round, k, p[0], p[1], got, want)
					}
				}
			}
		}
		for k := range classes {
			if l := &h.views[k].labels; l.local == 0 || l.made < 2 {
				t.Fatalf("wrap %v, class %d: labelled alone %d times, whole %d; want both", wrap,
					k, l.local, l.made)
			}
		}
	}
}

// a hierarchy of one level, each step of its build done by a call of its own: a cluster made
// ready, a fill from each node of each class, a row of each, finished; the labels of each class
// worked out whole in three
func TestHierarchy_AStepFillsANodeOfAClass(t *testing.T) {
	m := testmap.New(23, 48, 0.05, false)
	for _, classes := range []int{1, 2, 3} {
		g := m.Barred(classes, m.Bars(16, classes, 0.2, nil))
		h := testHierarchy(g, withTop(64))
		calls := 0
		for ; !h.Ready(); calls++ {
			h.Build(1)
			if h.stepWorked != h.worked {
				t.Fatalf("%d classes: a build of 1 did %d over more than one step", classes,
					h.worked)
			}
		}
		want := 3 * classes
		for c := range h.levels[0].stale {
			want += 2
			for k := range h.views {
				want += 2 * len(h.views[k].levels[0].clusters[c].keys)
			}
		}
		if len(h.levels) != 1 || calls != want {
			t.Fatalf("%d classes: %d levels built in %d steps, want one in %d", classes,
				len(h.levels), calls, want)
		}
	}
}

// each class keeps the trees a hierarchy of one class over its steps alone keeps, under the same
// memory, whatever the others keep: a budget holding one class's trees whole holds every class's,
// and one holding a third of them cuts each class where it would alone
func TestHierarchy_EachClassKeepsTheTreesItWouldAlone(t *testing.T) {
	const classes = 4
	m := testmap.New(24, 128, 0.05, false)
	g := m.Barred(classes, m.Bars(17, classes, 0.1, nil))
	most := 0
	for k := range classes {
		one := testHierarchy(testmap.Filtered(g, k))
		one.Build(0)
		most = max(most, one.trees[0])
	}
	for _, trees := range []int{most / 3, most} {
		h := testHierarchy(g, withTreeMemory(trees))
		h.Build(0)
		total, cut := 0, false
		for k := range classes {
			one := testHierarchy(testmap.Filtered(g, k), withTreeMemory(trees))
			one.Build(0)
			if diff := h.DiffView(k, one.Hierarchy, 0); diff != "" {
				t.Fatalf("%d bytes, class %d: %s", trees, k, diff)
			}
			for c := range h.levels[0].stale {
				a, b := &h.views[k].levels[0].clusters[c], &one.views[0].levels[0].clusters[c]
				if a.span != b.span || a.bytes != b.bytes {
					t.Fatalf("%d bytes, class %d: cluster %d keeps trees of span %d in %d bytes, "+
						"%d in %d alone", trees, k, c, a.span, a.bytes, b.span, b.bytes)
				}
				cut = cut || a.span == 0
			}
			if h.trees[k] != one.trees[0] || h.trees[k] > trees {
				t.Fatalf("%d bytes, class %d: the trees take %d, %d alone", trees, k,
					h.trees[k], one.trees[0])
			}
			total += h.trees[k]
		}
		if cut != (trees < most) || total <= trees {
			t.Fatalf("%d bytes: a cluster without trees %v; the classes' trees take %d in all, "+
				"want more than one budget", trees, cut, total)
		}
	}
}

func TestHierarchy_ClassesBuildAllocatesNothingTwice(t *testing.T) {
	m := testmap.New(8, 128, 0.1, false)
	bars := m.Bars(18, 4, 0.1, nil)
	g := m.Barred(4, bars)
	h := testHierarchy(g)
	h.Build(math.MaxInt)
	// a state on the border of clusters 0 and 1 walled and opened in turn and its classes
	// changed: every cluster of level 1 built again, the one above whole and the labels
	gate := 8*128 + 15
	share := m.Share[gate]
	rebuild := func() {
		if m.Share[gate] == 0 {
			m.Share[gate] = share
		} else {
			m.Share[gate] = 0
		}
		bars[gate+1] ^= 0b1100
		for s := range m.W * m.H {
			h.Stale(s)
		}
		if built, _ := h.Build(math.MaxInt); built < h.clusters(1)+1 {
			t.Fatalf("%d clusters built again of %d", built, h.clusters(1)+1)
		}
	}
	rebuild()
	rebuild()
	if allocs := testing.AllocsPerRun(2, rebuild); allocs != 0 {
		t.Fatalf("a build of every cluster allocates %v times the second time", allocs)
	}
}

// a map of open ground, a step costing 1, walled where walls says, of two classes, a state
// closed both ways to the classes bars has for it
func plainGround(size int, walls [][4]int) ([]uint8, graph.Implicit[int]) {
	m := testmap.New(1, size, 0, false)
	for i := range m.Share {
		m.Share[i] = 1
	}
	for _, w := range walls {
		wallIn(m, w[0], w[1], w[2], w[3])
	}
	bars := make([]uint8, size*size)
	return bars, testmap.Masked(m.Graph(), 2, func(from, to int) uint8 {
		return bars[from] | bars[to]
	})
}

func TestHierarchy_AChangeOfOneClassRebuildsWhatItSees(t *testing.T) {
	// a state on the border of clusters 0 and 1 closed to class 1: the entrances of class 1
	// there change, class 0's stay, and the cluster above is built again whole
	bars, g := plainGround(128, nil)
	h := testHierarchy(g)
	h.Build(0)
	s := 8*128 + 15
	bars[s] = 0b10
	h.Stale(s)
	whole, costs := built(h.Hierarchy)
	h.Build(0)
	if w, c := since(h.Hierarchy, whole, costs); !slices.Equal(w, []uint64{2, 1}) ||
		!slices.Equal(c, []uint64{0, 0}) {
		t.Fatalf("built again whole %v, filled again %v; want the two across the border and the "+
			"one above whole", w, c)
	}
	fresh := testHierarchy(g)
	fresh.Build(0)
	sameGraph(t, h.Hierarchy, fresh.Hierarchy)

	// the cluster at 48..63 × 16..31, its right border the border of two clusters above, cut
	// across at row 24 for class 1 where no step crosses its borders: the nodes of class 1 there
	// fall into two classes, its entrances stay, and both clusters above are built again whole,
	// the one across choosing other entrances
	bars, g = plainGround(256, [][4]int{{48, 15, 63, 15}, {48, 32, 63, 32}, {47, 23, 47, 25},
		{64, 23, 64, 25}})
	h = testHierarchy(g)
	h.Build(0)
	cut, across := 1*16+3, int32(1)
	keys := slices.Clone(h.views[1].levels[0].clusters[cut].keys)
	before := slices.Clone(h.views[1].levels[1].clusters[across].keys)
	for x := 48; x <= 63; x++ {
		bars[24*256+x] = 0b10
		h.Stale(24*256 + x)
	}
	whole, costs = built(h.Hierarchy)
	h.Build(0)
	if w, _ := since(h.Hierarchy, whole, costs); w[1] != 2 {
		t.Fatalf("built again whole %v; want both clusters above the cut's ends", w)
	}
	fresh = testHierarchy(g)
	fresh.Build(0)
	sameGraph(t, h.Hierarchy, fresh.Hierarchy)
	if !slices.Equal(keys, h.views[1].levels[0].clusters[cut].keys) ||
		slices.Equal(before, h.views[1].levels[1].clusters[across].keys) {
		t.Fatal("the scene: the cut changed the entrances of its cluster, or not the ones of " +
			"the cluster above across it")
	}
}

func TestHierarchy_ATwinOfOneClassAloneRebuildsTheClusterAbove(t *testing.T) {
	// the step from u to v across the border of clusters 0 and 1, and the step back, priced for
	// class 1 apart, 1.5 there and 0.5 back where both cost 1: the crossing costs it 2 as it did,
	// so every class keeps its entrances, and the twins of class 1 alone change
	_, base := plainGround(128, nil)
	u, v := 8*128+15, 8*128+16
	apart := false
	priced := func(steps graph.Transitions[int], back bool) graph.Transitions[int] {
		return func(s, prev int, buf []graph.Transition[int]) []graph.Transition[int] {
			buf = steps(s, prev, buf)
			if !apart || s != u && s != v {
				return buf
			}
			for i := range buf {
				if buf[i].To == u || buf[i].To == v {
					buf[i].Refuses = 0b10
				}
			}
			across, cost := v, 1.5
			if s == v {
				across, cost = u, 0.5
			}
			if back {
				cost = 2 - cost
			}
			return append(buf, graph.Transition[int]{To: across, Cost: cost, Refuses: 0b01})
		}
	}
	g := base
	g.Forward = priced(base.Forward, false)
	layout := *base.Layout
	layout.Backward = priced(base.Layout.Backward, true)
	g.Layout = &layout
	h := testHierarchy(g)
	h.Build(0)
	var zero, one [2][]twin
	for c := range 2 {
		zero[c] = slices.Clone(h.views[0].levels[0].clusters[c].twins)
		one[c] = slices.Clone(h.views[1].levels[0].clusters[c].twins)
	}
	entrances := slices.Clone(h.views[1].levels[0].clusters[0].keys)
	apart = true
	h.Stale(u)
	h.Stale(v)
	whole, costs := built(h.Hierarchy)
	h.Build(0)
	if w, c := since(h.Hierarchy, whole, costs); !slices.Equal(w, []uint64{2, 1}) ||
		!slices.Equal(c, []uint64{0, 0}) {
		t.Fatalf("built again whole %v, filled again %v; want the two across the border and the "+
			"one above whole", w, c)
	}
	fresh := testHierarchy(g)
	fresh.Build(0)
	sameGraph(t, h.Hierarchy, fresh.Hierarchy)
	if !slices.Equal(entrances, h.views[1].levels[0].clusters[0].keys) {
		t.Fatal("the scene: the entrances of class 1 changed")
	}
	for c := range 2 {
		if slices.Equal(one[c], h.views[1].levels[0].clusters[c].twins) ||
			!slices.Equal(zero[c], h.views[0].levels[0].clusters[c].twins) {
			t.Fatalf("the scene: the twins of cluster %d of class 1 kept, or class 0's changed", c)
		}
	}
}

func TestHierarchy_StaleFollowsTheTwinsOfEveryClass(t *testing.T) {
	// a step from a to b, clusters apart, refused to class 0: the twins of class 1 alone join
	// them; the step taken away and a alone staled, b's cluster is built again too
	m := testmap.New(25, 128, 0.02, false)
	a, b := m.OpenNear(10, 10), m.OpenNear(100, 90)
	jump := true
	base := m.Graph()
	g := base
	g.Classes = 2
	g.Forward = func(from, prev int, buf []graph.Transition[int]) []graph.Transition[int] {
		buf = base.Forward(from, prev, buf)
		if from == a && jump {
			buf = append(buf, graph.Transition[int]{To: b, Cost: 3, Refuses: 0b01})
		}
		return buf
	}
	layout := *base.Layout
	layout.Backward = func(to, next int, buf []graph.Transition[int]) []graph.Transition[int] {
		buf = base.Layout.Backward(to, next, buf)
		if to == b && jump {
			buf = append(buf, graph.Transition[int]{To: a, Cost: 3, Refuses: 0b01})
		}
		return buf
	}
	g.Layout = &layout
	h := testHierarchy(g)
	h.Build(0)
	ca, _ := h.cluster1(a)
	cb, _ := h.cluster1(b)
	if !slices.ContainsFunc(h.views[1].levels[0].clusters[ca].twins,
		func(tw twin) bool { return tw.other == cb }) {
		t.Fatal("the scene: no twin of class 1 across the step")
	}
	jump = false
	h.Stale(a)
	if h.levels[0].stale[cb] == fresh {
		t.Fatal("staling a left the cluster its twin of class 1 leads to fresh")
	}
	h.Build(0)
	o := testHierarchy(g)
	o.Build(0)
	sameGraph(t, h.Hierarchy, o.Hierarchy)
}
