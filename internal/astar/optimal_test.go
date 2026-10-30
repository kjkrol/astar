package astar

import (
	"container/heap"
	"math"
	"slices"
	"testing"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/testmap"
)

// dijkstraCost is the cheapest cost from from to to over transitions, +Inf unreachable: the
// reference the engine's paths are judged by
func dijkstraCost(from, to testmap.Point, transitions graph.Transitions[testmap.Point]) float64 {
	best := map[testmap.Point]float64{from: 0}
	q := &costQueue{{from, 0}}
	var buf []graph.Transition[testmap.Point]
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
	p    testmap.Point
	cost float64
}

type costQueue []costed

func (q costQueue) Len() int           { return len(q) }
func (q costQueue) Less(i, j int) bool { return q[i].cost < q[j].cost }
func (q costQueue) Swap(i, j int)      { q[i], q[j] = q[j], q[i] }
func (q *costQueue) Push(x any)        { *q = append(*q, x.(costed)) }
func (q *costQueue) Pop() (last any)   { last, *q = (*q)[len(*q)-1], (*q)[:len(*q)-1]; return }

func TestAStar_FindsTheCheapestPath(t *testing.T) {
	const size = 24
	for seed := uint64(1); seed <= 5; seed++ {
		grid := testmap.RandomGrid(size, seed)
		queries := testmap.Queries(grid, seed+100)
		walks := map[string]struct {
			transitions graph.Transitions[testmap.Point]
			finder      func(...dict.Dict[testmap.Point]) *Engine[testmap.Point]
		}{
			"four":  {testmap.FourWays(1.0, grid), pathFinder},
			"eight": {testmap.EightWays(grid), octileFinder},
		}
		for walk, w := range walks {
			for dictName, dict := range testmap.Dicts(size) {
				a := w.finder(dict())
				for _, q := range queries {
					want := dijkstraCost(q[0], q[1], w.transitions)
					path := a.SolveInto(nil, q[0], q[1], w.transitions)
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
					if got := testmap.PathCost(grid, path); math.Abs(got-want) > 1e-9 {
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
func TestAStar_ReopensAClosedState(t *testing.T) {
	edges := map[int][]graph.Transition[int]{
		0: {{To: 1, Cost: 1}, {To: 2, Cost: 5}},
		1: {{To: 2, Cost: 1}},
		2: {{To: 3, Cost: 10}},
	}
	estimates := [4]float64{0, 5, 0, 0}
	heuristic := func(from, to int) float64 { return estimates[from] }
	transitions := func(from, prev int, buf []graph.Transition[int]) []graph.Transition[int] {
		return append(buf, edges[from]...)
	}
	a := New[int](heuristic, dict.NewMap[int](0), 0)
	got := a.SolveInto(nil, 0, 3, transitions)
	if want := []int{0, 1, 2, 3}; !slices.Equal(got, want) {
		t.Fatalf("path %v, want %v: the cheapest, through A", got, want)
	}
	if expansions := a.Expanded(); expansions != 4 {
		t.Fatalf("%d states expanded; want 4: S, B, A, then B again", expansions)
	}
}

func TestAStar_ZeroCostSteps(t *testing.T) {
	heuristic := func(int, int) float64 { return 0 }
	solve := func(to int, transitions graph.Transitions[int]) []int {
		return New[int](heuristic, dict.NewMap[int](0), 0).SolveInto(nil, 0, to, transitions)
	}
	t.Run("a chain of free steps beats one step of cost 1", func(t *testing.T) {
		edges := map[int][]graph.Transition[int]{
			0: {{To: 3, Cost: 1}, {To: 1, Cost: 0}},
			1: {{To: 2, Cost: 0}},
			2: {{To: 3, Cost: 0}},
		}
		transitions := func(from, prev int, buf []graph.Transition[int]) []graph.Transition[int] {
			return append(buf, edges[from]...)
		}
		if got, want := solve(3, transitions), []int{0, 1, 2, 3}; !slices.Equal(got, want) {
			t.Fatalf("path %v, want %v", got, want)
		}
	})
	t.Run("a goal reached by a free step from the last cell", func(t *testing.T) {
		const goal = 99
		transitions := func(from, prev int, buf []graph.Transition[int]) []graph.Transition[int] {
			if from == 2 {
				return append(buf, graph.Transition[int]{To: goal, Cost: 0})
			}
			return append(buf, graph.Transition[int]{To: from + 1, Cost: 1})
		}
		got := solve(goal, transitions)
		if want := []int{0, 1, 2, goal}; !slices.Equal(got, want) {
			t.Fatalf("path %v, want %v", got, want)
		}
	})
}

func TestAStar_FirstStepSeesPrevAsFrom(t *testing.T) {
	grid := testmap.WalledGrid(8)
	var first *[2]testmap.Point
	transitions := func(from, prev testmap.Point,
		buf []graph.Transition[testmap.Point]) []graph.Transition[testmap.Point] {
		if first == nil {
			first = &[2]testmap.Point{from, prev}
		}
		return testmap.FourWays(1.0, grid)(from, prev, buf)
	}
	start := testmap.Point{X: 3, Y: 3}
	pathFinder().SolveInto(nil, start, testmap.Point{X: 7, Y: 7}, transitions)
	if first == nil || *first != [2]testmap.Point{start, start} {
		t.Fatalf("the first call saw %v; want from and prev both %v", first, start)
	}
}

func TestAStar_StoppedEarly(t *testing.T) {
	const size = 32
	grid := testmap.WalledGrid(size)
	transitions := testmap.FourWays(1.0, grid)
	start, to := testmap.Point{}, testmap.Point{X: size - 1, Y: size - 1}
	want := pathFinder().SolveInto(nil, start, to, transitions)
	a := pathFinder()

	a.Start(start, to)
	a.Run(to, transitions, 1)
	if got := a.ResultInto(nil); !slices.Equal(got, []testmap.Point{start}) {
		t.Fatalf("after the first expansion: the path %v, want [%v]", got, start)
	}
	a.Start(start, to)
	a.Run(to, transitions, 3)
	if got := a.ResultInto(nil); len(got) < 2 || got[0] != start {
		t.Fatalf("after three expansions: the path %v, want one from %v", got, start)
	}
	if got := a.SolveInto(nil, start, to, transitions); !slices.Equal(got, want) {
		t.Fatalf("a search after one stopped early: %v, want %v", got, want)
	}
}

func TestIndexedSliceDict_PanicsOutsideMaxSize(t *testing.T) {
	const size = 4
	grid := testmap.WalledGrid(size)
	// the indexer of a grid one column wider than the dictionary was sized for
	indexer := func(p testmap.Point) int { return p.Y*(size+1) + p.X }
	a := pathFinder(dict.NewIndexedSlice(size*size, indexer))
	defer func() {
		if recover() == nil {
			t.Fatal("an index at maxSize did not panic")
		}
	}()
	a.SolveInto(nil, testmap.Point{}, testmap.Point{X: size - 1, Y: size - 1},
		testmap.FourWays(1.0, grid))
}
