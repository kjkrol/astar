package hpastar_test

import (
	"math"
	"slices"
	"sync"
	"testing"

	"github.com/kjkrol/hpastar"
	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/testmap"
)

// path is the whole path q writes from from to to, dst grown until it holds it
func path(q *hpastar.Search[int], from, to int, dst []int) ([]int, bool) {
	for {
		n, ok := q.Path(from, to, dst)
		if !ok || n < len(dst) || dst[n-1] == to {
			return dst[:n], ok
		}
		dst = make([]int, 2*len(dst))
	}
}

// exactOf is a Search over m's graph without its Layout: every path the cheapest, by A*
func exactOf(m *testmap.Map) *hpastar.Search[int] {
	g := m.Graph()
	g.Layout = nil
	return hpastar.NewSolver(g).Search()
}

// walks fails t where route is no walk from from to to on m, and reports what it costs
func walks(t *testing.T, m *testmap.Map, route []int, from, to int) float64 {
	t.Helper()
	cost, ok := testmap.WalkCost(m.Forward, route)
	if !ok || len(route) == 0 || route[0] != from || route[len(route)-1] != to {
		t.Fatalf("the path from %d to %d is no walk between them: %v", from, to, route)
	}
	return cost
}

func TestSolver_ReadyWithoutALayout(t *testing.T) {
	m := testmap.New(1, 64, 0.05, false)
	g := m.Graph()
	g.Layout = nil
	s := hpastar.NewSolver(g)
	if !s.Ready() {
		t.Fatal("a Solver without a Layout is not ready")
	}
	far := m.FarPair()
	s.Changed(far[0])
	s.ChangedAll()
	if _, left := s.Update(0); left != 0 || !s.Ready() {
		t.Fatalf("Update left %d, ready %v; without a Layout there is nothing to build",
			left, s.Ready())
	}
	q := s.Search()
	route, ok := path(q, far[0], far[1], make([]int, 64*64))
	if !ok {
		t.Fatal("no path between the far pair")
	}
	walks(t, m, route, far[0], far[1])
	if !q.Reaches(far[0], far[1]) {
		t.Fatal("Reaches says no path joins the far pair")
	}
	st := q.Stats()
	if st.Exact != 1 || st.Climbs != 0 || st.Paths != 1 || st.Expansions == 0 {
		t.Fatalf("stats %+v; want one path asked, found by A*", st)
	}
}

func TestSolver_PathAnswersEveryPair(t *testing.T) {
	const size = 256
	m := testmap.New(3, size, 0.05, false)
	// an island within one cluster of the first level, another across many
	m.WallRing(97, 97, 12)
	m.WallRing(150, 30, 40)
	inside, across := m.OpenNear(102, 102), m.OpenNear(170, 50)
	outside := m.OpenNear(90, 102)
	for _, c := range []struct {
		s      int
		x0, x1 int
	}{{inside, 98, 108}, {across, 151, 189}, {outside, 0, 96}} {
		if x, y := c.s%size, c.s/size; x < c.x0 || x > c.x1 || y < 0 {
			t.Fatalf("the scene: the open state %d lies at (%d, %d)", c.s, x, y)
		}
	}
	far := m.FarPair()
	g, gate := m.Gated(far[0])
	plain := g
	plain.Layout = nil
	s := hpastar.NewSolver(g)
	q, exact := s.Search(), hpastar.NewSolver(plain).Search()
	// room for any path at once: a Path asked again with more room is another call
	dst := make([]int, 4*size)
	cheapest := func(from, to int) float64 {
		route, ok := path(exact, from, to, dst)
		if !ok {
			return math.Inf(1)
		}
		cost, _ := testmap.WalkCost(m.Forward, route)
		return cost
	}
	asked := func(from, to int) (route []int, found bool, st hpastar.Stats) {
		before := q.Stats()
		route, found = path(q, from, to, dst)
		st = q.Stats()
		st.Paths -= before.Paths
		st.Climbs -= before.Climbs
		st.Exact -= before.Exact
		st.Expansions -= before.Expansions
		return route, found, st
	}

	// before the hierarchy is built every path is the cheapest
	route, found, st := asked(far[0], far[1])
	if !found || st.Exact != 1 || st.Climbs != 0 {
		t.Fatalf("not ready: found %v, stats %+v; want A*'s path", found, st)
	}
	if got, want := walks(t, m, route, far[0], far[1]), cheapest(far[0], far[1]); got != want {
		t.Fatalf("not ready: a path of %v, the cheapest %v", got, want)
	}
	if _, left := s.Update(0); left != 0 || !s.Ready() {
		t.Fatalf("Update(0) left %d", left)
	}

	// ends far apart: climbed
	route, found, st = asked(far[0], far[1])
	if !found || st.Climbs != 1 || st.Exact != 0 {
		t.Fatalf("far: found %v, stats %+v; want a climb", found, st)
	}
	got, want := walks(t, m, route, far[0], far[1]), cheapest(far[0], far[1])
	if got > 1.15*want {
		t.Fatalf("far: a path of %v, %.3f of the cheapest", got, got/want)
	}

	// ends near, or one without a place in the plane: the cheapest, A*'s own path
	near := m.OpenNear(far[0]%size+5, far[0]/size+3)
	for _, ends := range [][2]int{{far[0], near}, {gate, near}, {near, near}} {
		route, found, st = asked(ends[0], ends[1])
		want, _ := path(exact, ends[0], ends[1], make([]int, 4*size))
		if !found || st.Exact != 1 || st.Climbs != 0 || !slices.Equal(route, want) {
			t.Fatalf("%d to %d: found %v, stats %+v, %v; want A*'s path %v",
				ends[0], ends[1], found, st, route, want)
		}
	}

	// no path: denied off the labels, no flood of all the start reaches
	for _, ends := range [][2]int{{far[0], inside}, {inside, far[1]}, {outside, inside},
		{far[1], across}} {
		_, found, st = asked(ends[0], ends[1])
		if found || st.Expansions > 2500 {
			t.Fatalf("%d to %d, apart: found %v after %d expansions; want none, few",
				ends[0], ends[1], found, st.Expansions)
		}
		if q.Reaches(ends[0], ends[1]) {
			t.Fatalf("Reaches %d to %d, apart", ends[0], ends[1])
		}
	}
	if !q.Reaches(far[0], far[1]) || !q.Reaches(inside, inside) {
		t.Fatal("Reaches denies a pair a path joins")
	}

	// a path asked a piece at a time, each from where the last ended, comes to the goal
	piece := make([]int, 16)
	whole := []int{far[0]}
	for from := far[0]; from != far[1]; from = whole[len(whole)-1] {
		n, ok := q.Path(from, far[1], piece)
		if !ok || n < 2 || piece[0] != from {
			t.Fatalf("a piece from %d: %d states (%v)", from, n, ok)
		}
		if whole = append(whole, piece[1:n]...); len(whole) > 4*size {
			t.Fatal("the pieces run on past the goal")
		}
	}
	walks(t, m, whole, far[0], far[1])
	if n, ok := q.Path(far[0], far[1], nil); n != 0 || !ok {
		t.Fatalf("into no room: %d states (%v); want none and found", n, ok)
	}
}

func TestSolver_AChangeIsFollowed(t *testing.T) {
	const size, wallX = 256, 128
	m := testmap.New(5, size, 0.02, false)
	// a wall down the map, open at its foot alone
	for y := range size - 8 {
		m.Share[y*size+wallX] = 0
	}
	from, to := m.OpenNear(40, 20), m.OpenNear(216, 20)
	s := hpastar.NewSolver(m.Graph())
	s.Update(0)
	q := s.Search()
	dst := make([]int, 64)
	around, ok := path(q, from, to, dst)
	if !ok {
		t.Fatal("no path round the wall")
	}
	before := walks(t, m, around, from, to)

	// a gap opened near the pair: a new step, and a path through it
	for y := 18; y <= 22; y++ {
		m.Share[y*size+wallX] = 1
		s.Changed(y*size + wallX)
	}
	if s.Ready() {
		t.Fatal("ready after a change")
	}
	// while it is rebuilt a path is the cheapest, over the graph as it is
	stale, ok := path(q, from, to, dst)
	if exact, _ := path(exactOf(m), from, to, dst); !ok || !slices.Equal(stale, exact) {
		t.Fatalf("while rebuilt: %v, A*'s %v", stale, exact)
	}
	if _, left := s.Update(0); left != 0 {
		t.Fatalf("Update(0) left %d stale", left)
	}
	through, ok := path(q, from, to, dst)
	if !ok {
		t.Fatal("no path once the gap opened")
	}
	if after := walks(t, m, through, from, to); after > before/2 ||
		!slices.ContainsFunc(through, func(s int) bool { return s%size == wallX }) {
		t.Fatalf("through the gap: a path of %v, round the wall %v", after, before)
	}

	// the gap walled again but for a cell: the path keeps off the walls
	for y := 18; y <= 22; y++ {
		if y != 21 {
			m.Share[y*size+wallX] = 0
			s.Changed(y*size + wallX)
		}
	}
	s.Update(0)
	again, ok := path(q, from, to, dst)
	if !ok {
		t.Fatal("no path through the cell left open")
	}
	walks(t, m, again, from, to)
	if !slices.Contains(again, 21*size+wallX) {
		t.Fatalf("the path keeps off the one cell left open in the wall: %v", again)
	}

	// the whole graph changed: every cluster rebuilt, the same paths
	s.ChangedAll()
	if s.Ready() {
		t.Fatal("ready after ChangedAll")
	}
	s.Update(0)
	if all, _ := path(q, from, to, dst); !slices.Equal(all, again) {
		t.Fatalf("rebuilt whole: %v, rebuilt where changed %v", all, again)
	}
}

// pairsOf are n pairs of m, of a 128x128 map: far and near ones, and ones into a walled island
func pairsOf(m *testmap.Map, n int) [][2]int {
	m.WallRing(40, 40, 10)
	inside := m.OpenNear(44, 44)
	pairs := m.Pairs(41, n, 0, math.Inf(1))
	for i := 0; i < len(pairs); i += 4 {
		if from := pairs[i][0]; i%8 == 0 {
			pairs[i][1] = inside
		} else {
			pairs[i][1] = m.OpenNear(min(from%128+5, 127), min(from/128+3, 127))
		}
	}
	return pairs
}

func TestSearch_ManyGoroutinesAskAtOnce(t *testing.T) {
	const goroutines, each = 8, 30
	m := testmap.New(1, 128, 0.08, false)
	pairs := pairsOf(m, goroutines*each)
	plain := m.Graph()
	plain.Layout = nil
	for name, s := range map[string]*hpastar.Solver[int]{
		"HPA*": hpastar.NewSolver(m.Graph()),
		"A*":   hpastar.NewSolver(plain),
	} {
		s.Update(0)
		type answer struct {
			route   []int
			found   bool
			reached bool
		}
		ask := func(q *hpastar.Search[int], p [2]int, dst []int) answer {
			route, found := path(q, p[0], p[1], dst)
			return answer{slices.Clone(route), found, q.Reaches(p[0], p[1])}
		}
		want := make([]answer, len(pairs))
		q, dst := s.Search(), make([]int, 64)
		for i, p := range pairs {
			want[i] = ask(q, p, dst)
		}
		got := make([]answer, len(pairs))
		var wg sync.WaitGroup
		for g := range goroutines {
			wg.Go(func() {
				// half the goroutines read the map through closures of their own
				q := s.Search()
				if g%2 == 1 {
					own := m.Graph()
					q = s.SearchVia(func(from, prev int,
						buf []graph.Transition[int]) []graph.Transition[int] {
						return own.Forward(from, prev, buf)
					}, func(to, next int, buf []graph.Transition[int]) []graph.Transition[int] {
						return own.Layout.Backward(to, next, buf)
					})
				}
				dst := make([]int, 64)
				for k := range each {
					got[g*each+k] = ask(q, pairs[g*each+k], dst)
				}
			})
		}
		wg.Wait()
		for i, p := range pairs {
			w, g := want[i], got[i]
			if w.found != g.found || w.reached != g.reached || !slices.Equal(w.route, g.route) {
				t.Fatalf("%s %v: asked at once %v, %d states, reached %v; one after another "+
					"%v, %d states, reached %v", name, p, g.found, len(g.route), g.reached,
					w.found, len(w.route), w.reached)
			}
		}
	}
}

func TestSearch_PathAllocatesNothingOnceWarm(t *testing.T) {
	m := testmap.New(2, 128, 0.05, false)
	pairs := pairsOf(m, 24)
	plain := m.Graph()
	plain.Layout = nil
	for name, s := range map[string]*hpastar.Solver[int]{
		"HPA*": hpastar.NewSolver(m.Graph()),
		"A*":   hpastar.NewSolver(plain),
	} {
		s.Update(0)
		q := s.Search()
		dst, short := make([]int, 4*128), make([]int, 8)
		asks := func() {
			for _, p := range pairs {
				q.Path(p[0], p[1], dst)
				q.Path(p[0], p[1], short)
				q.Reaches(p[0], p[1])
			}
		}
		asks()
		if allocs := testing.AllocsPerRun(5, asks); allocs != 0 {
			t.Fatalf("%s: Path and Reaches allocate %v times once warm", name, allocs)
		}
		if st := q.Stats(); name == "HPA*" && (st.Climbs == 0 || st.Exact == 0) {
			t.Fatalf("%s: the pairs asked %+v; want some climbed and some found by A*", name, st)
		}
	}
}

// rebuiltSince is what s's Updates built since they had built before
func rebuiltSince(s *hpastar.Solver[int], before hpastar.Rebuilds) hpastar.Rebuilds {
	now := hpastar.CountRebuilds(s)
	for j := range now.Whole {
		now.Whole[j] -= before.Whole[j]
		now.Costs[j] -= before.Costs[j]
	}
	now.Labelled -= before.Labelled
	now.Relabelled -= before.Relabelled
	return now
}

// sameAsWhole fails t where s's hierarchy differs from one a Solver over g builds whole
func sameAsWhole(t *testing.T, s *hpastar.Solver[int], g graph.Implicit[int],
	opts ...hpastar.Option) {
	t.Helper()
	fresh := hpastar.NewSolver(g, opts...)
	fresh.Update(0)
	if diff := hpastar.GraphsDiffer(s, fresh); diff != "" {
		t.Fatalf("against a hierarchy built whole: %s", diff)
	}
}

// a step's cost changed in a pocket walled off amid a cluster, where no path between its
// entrances runs
func TestSolver_AChangeInsideAClusterRebuildsItAlone(t *testing.T) {
	const size = 256
	m := testmap.New(7, size, 0.05, false)
	m.WallRing(86, 86, 4)
	pocket := 87*size + 87
	m.Share[pocket] = 0.5
	s := hpastar.NewSolver(m.Graph())
	s.Update(0)
	before := hpastar.CountRebuilds(s)
	m.Share[pocket] = 0.25
	s.Changed(pocket)
	if built, left := s.Update(0); built != 1 || left != 0 {
		t.Fatalf("built %d clusters, %d left; want the pocket's alone", built, left)
	}
	if got := rebuiltSince(s, before); !slices.Equal(got.Whole, []uint64{1, 0}) ||
		!slices.Equal(got.Costs, []uint64{0, 0}) || got.Labelled+got.Relabelled != 0 {
		t.Fatalf("built whole by level %v, costs alone %v, labelled %d and %d times; want the "+
			"pocket's cluster alone, the one above untouched, the labels kept", got.Whole,
			got.Costs, got.Labelled, got.Relabelled)
	}
	sameAsWhole(t, s, m.Graph())
}

// the cells within a cluster made dearer, its border kept: its entrances, their classes and
// their twins stay, the costs between them change
func TestSolver_ACostOnlyChangeRefillsTheParent(t *testing.T) {
	const size = 256
	m := testmap.New(7, size, 0.05, false)
	s := hpastar.NewSolver(m.Graph())
	s.Update(0)
	far := m.FarPair()
	q := s.Search()
	dst := make([]int, 4*size)
	before := hpastar.CountRebuilds(s)
	for y := 81; y <= 94; y++ {
		for x := 81; x <= 94; x++ {
			if c := y*size + x; m.Share[c] > 0 {
				m.Share[c] = m.Share[c] / 2
				s.Changed(c)
			}
		}
	}
	if built, left := s.Update(0); built != 2 || left != 0 {
		t.Fatalf("built %d clusters, %d left; want the cells' cluster and the one above", built,
			left)
	}
	if got := rebuiltSince(s, before); !slices.Equal(got.Whole, []uint64{1, 0}) ||
		!slices.Equal(got.Costs, []uint64{0, 1}) || got.Labelled+got.Relabelled != 0 {
		t.Fatalf("built whole by level %v, costs alone %v, labelled %d and %d times; want the "+
			"cells' cluster whole, the costs of the one above filled again, the labels kept",
			got.Whole, got.Costs, got.Labelled, got.Relabelled)
	}
	sameAsWhole(t, s, m.Graph())
	route, ok := path(q, far[0], far[1], dst)
	if !ok || q.Stats().Climbs == 0 {
		t.Fatal("the far pair not climbed once the costs changed")
	}
	cheapest, _ := path(exactOf(m), far[0], far[1], make([]int, 4*size))
	if got, want := walks(t, m, route, far[0], far[1]), walks(t, m, cheapest, far[0],
		far[1]); got > 1.15*want {
		t.Fatalf("a path of %v, %.3f of the cheapest", got, got/want)
	}
}

// a ring across clusters with a gate: shut, its inside an island; opened, one with the rest
func TestSolver_AConnectivityChangeRelabels(t *testing.T) {
	const size = 256
	m := testmap.New(3, size, 0.02, false)
	m.WallRing(100, 100, 40)
	gate := 100*size + 112
	inside, outside := m.OpenNear(120, 120), m.OpenNear(60, 60)
	if x, y := inside%size, inside/size; x <= 100 || y <= 100 || x >= 139 || y >= 139 {
		t.Fatalf("the scene: the open state inside the ring lies at (%d, %d)", x, y)
	}
	s := hpastar.NewSolver(m.Graph())
	s.Update(0)
	q := s.Search()
	pairs := [][2]int{{inside, outside}, {outside, inside}, {inside, m.OpenNear(130, 108)},
		{outside, m.OpenNear(200, 30)}}
	reaches := func(what string) {
		t.Helper()
		exact := exactOf(m)
		for _, p := range pairs {
			_, want := path(exact, p[0], p[1], make([]int, 4*size))
			if got := q.Reaches(p[0], p[1]); got != want {
				t.Fatalf("%s: Reaches(%d, %d) %v, A* %v", what, p[0], p[1], got, want)
			}
		}
	}
	reaches("shut")
	if q.Reaches(inside, outside) {
		t.Fatal("the scene: the ring shut, its inside reaches out")
	}
	for _, c := range []struct {
		what                 string
		share                float32
		labelled, relabelled uint64
	}{
		{"opened: the components joined, the clusters changed labelled alone", 1, 0, 1},
		{"shut: a component come apart, labelled whole", 0, 1, 0},
		{"opened again", 1, 0, 1},
	} {
		before := hpastar.CountRebuilds(s)
		m.Share[gate] = c.share
		s.Changed(gate)
		if _, left := s.Update(0); left != 0 {
			t.Fatalf("%s: %d left", c.what, left)
		}
		if got := rebuiltSince(s, before); got.Labelled != c.labelled ||
			got.Relabelled != c.relabelled {
			t.Fatalf("%s: labelled whole %d times and alone %d; want %d and %d", c.what,
				got.Labelled, got.Relabelled, c.labelled, c.relabelled)
		}
		reaches(c.what)
		sameAsWhole(t, s, m.Graph())
	}
}

// a small budget spent call by call: each call stops on the step that spends it, and the
// clusters' fills run on across calls
func TestSolver_TheBudgetBoundsTheWork(t *testing.T) {
	const budget = 2000
	m := testmap.New(1, 256, 0.05, false)
	s := hpastar.NewSolver(m.Graph(), hpastar.WithClusterSide(8))
	calls, built, idle := 0, 0, 0
	for !s.Ready() {
		finished, _ := s.Update(budget)
		work, step := hpastar.Worked(s)
		if work < 1 || step < 1 || work > budget+step || work-step >= budget {
			t.Fatalf("an Update of %d did %d, its last step %d; want a step past the budget at "+
				"most, and progress", budget, work, step)
		}
		calls, built = calls+1, built+finished
		if finished == 0 {
			idle++
		}
	}
	if idle == 0 || built != 32*32+8*8+2*2 {
		t.Fatalf("%d clusters built over %d calls, %d finishing none; want every one, some "+
			"over several calls", built, calls, idle)
	}
	sameAsWhole(t, s, m.Graph(), hpastar.WithClusterSide(8))
}

// a cell on the borders of every level walled: rebuilt by Updates of a small budget, a step of
// the cluster above it at a time, or by one Update of all
func TestSolver_AResumedRebuildEqualsAWholeOne(t *testing.T) {
	const size = 256
	m := testmap.New(9, size, 0.05, false)
	cell := 132*size + 128
	m.Share[cell] = 1
	g := m.Graph()
	one := hpastar.NewSolver(g, hpastar.WithClusterSide(8))
	steps := hpastar.NewSolver(g, hpastar.WithClusterSide(8))
	one.Update(0)
	steps.Update(0)
	m.Share[cell] = 0
	one.Changed(cell)
	steps.Changed(cell)
	one.Update(0)
	before := hpastar.CountRebuilds(steps)
	calls, built := 0, 0
	for left := 1; left > 0; calls++ {
		var finished int
		finished, left = steps.Update(300)
		built += finished
	}
	if got := rebuiltSince(steps, before); got.Whole[2]+got.Costs[2] == 0 || calls <= built {
		t.Fatalf("built whole by level %v, costs alone %v, %d clusters over %d calls; want the "+
			"cluster of level 3 built, a step at a time", got.Whole, got.Costs, built, calls)
	}
	if diff := hpastar.GraphsDiffer(one, steps); diff != "" {
		t.Fatalf("rebuilt a step at a time, against at once: %s", diff)
	}
	sameAsWhole(t, steps, g, hpastar.WithClusterSide(8))
	a, b := one.Search(), steps.Search()
	for _, p := range m.Pairs(11, 40, 40, math.Inf(1)) {
		x, okx := path(a, p[0], p[1], make([]int, 64))
		y, oky := path(b, p[0], p[1], make([]int, 64))
		if okx != oky || !slices.Equal(x, y) {
			t.Fatalf("%v: the paths of the two rebuilds differ", p)
		}
	}
}

// two Solvers over maps alike, built and changed by the same calls
func TestSolver_TheSameCallsGiveTheSameProgress(t *testing.T) {
	run := func() []int {
		m := testmap.New(4, 256, 0.05, false)
		s := hpastar.NewSolver(m.Graph())
		var progress []int
		update := func(budget int) {
			for left := 1; left > 0; {
				var built int
				built, left = s.Update(budget)
				work, step := hpastar.Worked(s)
				progress = append(progress, built, left, work, step)
			}
		}
		update(50000)
		for _, c := range []int{40*256 + 47, 128*256 + 128, 200*256 + 16} {
			m.Share[c] = 0
			s.Changed(c)
		}
		update(777)
		s.ChangedAll()
		update(123456)
		return progress
	}
	a, b := run(), run()
	if !slices.Equal(a, b) {
		t.Fatal("the same calls made other progress")
	}
	if len(a) < 4*10 {
		t.Fatalf("%d calls; want many", len(a)/4)
	}
}
