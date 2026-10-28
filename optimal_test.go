package astar_test

import (
	"container/heap"
	"math"
	"slices"
	"testing"

	"github.com/kjkrol/astar"
)

// dijkstraCost is the cheapest cost from from to to over transitions, +Inf unreachable: the
// reference the solver's paths are judged by
func dijkstraCost(from, to Point, transitions astar.Transitions[Point]) float64 {
	best := map[Point]float64{from: 0}
	q := &costQueue{{from, 0}}
	var buf []astar.Transition[Point]
	for q.Len() > 0 {
		cur := heap.Pop(q).(costed)
		if cur.cost > best[cur.p] {
			continue
		}
		if cur.p == to {
			return cur.cost
		}
		buf = transitions(cur.p, cur.p, buf[:0])
		for _, t := range buf {
			cost := cur.cost + t.Cost
			if known, ok := best[t.To]; !ok || cost < known {
				best[t.To] = cost
				heap.Push(q, costed{t.To, cost})
			}
		}
	}
	return math.Inf(1)
}

type costed struct {
	p    Point
	cost float64
}

type costQueue []costed

func (q costQueue) Len() int           { return len(q) }
func (q costQueue) Less(i, j int) bool { return q[i].cost < q[j].cost }
func (q costQueue) Swap(i, j int)      { q[i], q[j] = q[j], q[i] }
func (q *costQueue) Push(x any)        { *q = append(*q, x.(costed)) }
func (q *costQueue) Pop() (last any)   { last, *q = (*q)[len(*q)-1], (*q)[:len(*q)-1]; return }

func TestSolver_FindsTheCheapestPath(t *testing.T) {
	const size = 24
	indexer := func(p Point) int { return p.Y*size + p.X }
	dictionaries := map[string][]astar.SolverOption[Point]{
		"DefaultDict":      nil,
		"IndexedMapDict":   {astar.WithIndexedMapDict(indexer)},
		"IndexedSliceDict": {astar.WithIndexedSliceDict(size*size, indexer)},
	}
	for seed := uint64(1); seed <= 5; seed++ {
		grid := randomGrid(size, seed)
		queries := goldenQueries(grid, seed+100)
		walks := map[string]struct {
			transitions astar.Transitions[Point]
			finder      func(...astar.SolverOption[Point]) *astar.Solver[Point]
		}{
			"four":  {setupTransitions(1.0, grid), setupPathFinder},
			"eight": {setupTransitions8(grid), setupOctileFinder},
		}
		for walk, w := range walks {
			for dictName, opts := range dictionaries {
				solver := w.finder(opts...)
				for _, q := range queries {
					want := dijkstraCost(q[0], q[1], w.transitions)
					path := solver.Solve(q[0], q[1], w.transitions)
					if math.IsInf(want, 1) {
						if path != nil {
							t.Fatalf("seed %d %s %s %v->%v: unreachable, got %v",
								seed, walk, dictName, q[0], q[1], path)
						}
						continue
					}
					if path == nil {
						t.Fatalf("seed %d %s %s %v->%v: no path, Dijkstra finds one of %v",
							seed, walk, dictName, q[0], q[1], want)
					}
					if got := pathCost(grid, path); math.Abs(got-want) > 1e-9 {
						t.Fatalf("seed %d %s %s %v->%v: a path of %v, the cheapest is %v: %v",
							seed, walk, dictName, q[0], q[1], got, want, path)
					}
				}
			}
		}
	}
}

// a graph of four states: S 0, A 1, B 2, G 3; the heuristic never overestimates (h(A) 5 under
// the 11 left) but is inconsistent (h(A) 5 over the step to B, 1, plus h(B) 0), so B is closed
// through S at 5 before A reaches it at 2 and must be opened again
func TestSolver_ReopensAClosedState(t *testing.T) {
	edges := map[int][]astar.Transition[int]{
		0: {{To: 1, Cost: 1}, {To: 2, Cost: 5}},
		1: {{To: 2, Cost: 1}},
		2: {{To: 3, Cost: 10}},
	}
	estimates := [4]float64{0, 5, 0, 0}
	heuristic := func(from, to int) float64 { return estimates[from] }
	transitions := func(from, prev int, buf []astar.Transition[int]) []astar.Transition[int] {
		return append(buf, edges[from]...)
	}
	solver := astar.New(heuristic)

	expansions := 0
	for reached := range solver.Iter(0, 3, transitions) {
		if !reached {
			expansions++
		}
	}
	if got, want := solver.Result(), []int{0, 1, 2, 3}; !slices.Equal(got, want) {
		t.Fatalf("path %v, want %v: the cheapest, through A", got, want)
	}
	if expansions != 4 {
		t.Fatalf("%d states expanded; want 4: S, B, A, then B again", expansions)
	}
}

func TestSolver_ZeroCostSteps(t *testing.T) {
	heuristic := func(int, int) float64 { return 0 }
	t.Run("a chain of free steps beats one step of cost 1", func(t *testing.T) {
		edges := map[int][]astar.Transition[int]{
			0: {{To: 3, Cost: 1}, {To: 1, Cost: 0}},
			1: {{To: 2, Cost: 0}},
			2: {{To: 3, Cost: 0}},
		}
		transitions := func(from, prev int, buf []astar.Transition[int]) []astar.Transition[int] {
			return append(buf, edges[from]...)
		}
		got := astar.New(heuristic).Solve(0, 3, transitions)
		if want := []int{0, 1, 2, 3}; !slices.Equal(got, want) {
			t.Fatalf("path %v, want %v", got, want)
		}
	})
	t.Run("a goal reached by a free step from the last cell", func(t *testing.T) {
		const goal = 99
		transitions := func(from, prev int, buf []astar.Transition[int]) []astar.Transition[int] {
			if from == 2 {
				return append(buf, astar.Transition[int]{To: goal, Cost: 0})
			}
			return append(buf, astar.Transition[int]{To: from + 1, Cost: 1})
		}
		got := astar.New(heuristic).Solve(0, goal, transitions)
		if want := []int{0, 1, 2, goal}; !slices.Equal(got, want) {
			t.Fatalf("path %v, want %v", got, want)
		}
	})
}

func TestSolver_FirstStepSeesPrevAsFrom(t *testing.T) {
	grid := setupGrid(8)
	var first *[2]Point
	transitions := func(from, prev Point, buf []astar.Transition[Point]) []astar.Transition[Point] {
		if first == nil {
			first = &[2]Point{from, prev}
		}
		return setupTransitions(1.0, grid)(from, prev, buf)
	}
	start := Point{X: 3, Y: 3}
	setupPathFinder().Solve(start, Point{X: 7, Y: 7}, transitions)
	if first == nil || *first != [2]Point{start, start} {
		t.Fatalf("the first call saw %v; want from and prev both %v", first, start)
	}
}

func TestSolver_IterBrokenEarly(t *testing.T) {
	const size = 32
	grid := setupGrid(size)
	transitions := setupTransitions(1.0, grid)
	start, to := Point{}, Point{X: size - 1, Y: size - 1}
	want := setupPathFinder().Solve(start, to, transitions)
	solver := setupPathFinder()

	for range solver.Iter(start, to, transitions) {
		break
	}
	if got := solver.Result(); !slices.Equal(got, []Point{start}) {
		t.Fatalf("after the first expansion: Result %v, want [%v]", got, start)
	}
	yields := 0
	for range solver.Iter(start, to, transitions) {
		if yields++; yields == 3 {
			break
		}
	}
	got := solver.Result()
	if len(got) < 2 || got[0] != start {
		t.Fatalf("after three expansions: Result %v, want a path from %v", got, start)
	}
	if got := solver.Solve(start, to, transitions); !slices.Equal(got, want) {
		t.Fatalf("Solve after a broken Iter: %v, want %v", got, want)
	}
}

func TestWithIndexedSliceDict_PanicsOutsideMaxSize(t *testing.T) {
	const size = 4
	grid := setupGrid(size)
	// the indexer of a grid one column wider than the dictionary was sized for
	indexer := func(p Point) int { return p.Y*(size+1) + p.X }
	solver := setupPathFinder(astar.WithIndexedSliceDict(size*size, indexer))
	defer func() {
		if recover() == nil {
			t.Fatal("an index at maxSize did not panic")
		}
	}()
	solver.Solve(Point{}, Point{X: size - 1, Y: size - 1}, setupTransitions(1.0, grid))
}
