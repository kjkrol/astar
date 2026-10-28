# kjkrol/astar

<p align="center">
  <img src=".github/docs/img/logo.png" alt="kjkrol/astar Logo" width="100">
  <br>
    <a href="https://go.dev">
    <img src="https://img.shields.io/badge/Go-1.25+-00ADD8?style=flat-square&logo=go" alt="Go Version">
  </a>
  <a href="https://pkg.go.dev/github.com/kjkrol/astar">
    <img src="https://img.shields.io/badge/GoDoc-Reference-007d9c?style=flat-square&logo=go" alt="GoDoc">
  </a>
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg?style=flat-square" alt="License">
  </a>
  <a href="https://github.com/kjkrol/astar/actions">
    <img src="https://github.com/kjkrol/astar/actions/workflows/go.yml/badge.svg" alt="Go Quality Check">
  </a>
</p>

**kjkrol/astar** is a fast, fully generic A* solver for Go: the cheapest sequence of states from
one state to another, over any state space a heuristic and a transition function describe.

## What is A*?

The [A* (A-star) algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) is a graph
traversal and state-space search algorithm heavily used in computer science due to its
completeness, optimality, and optimal efficiency. It is widely considered the industry standard
for finding the shortest path or optimal sequence of transitions between nodes.

While most commonly used as a **pathfinder**, this library provides a completely domain-agnostic
`Solver`. You can use it to solve complex puzzles, optimize network routing, or navigate grids,
simply by providing your own `Heuristic` and `Transitions` logic. The Solver remains strictly
agnostic of the underlying graph structure or domain-specific cost metrics, making it a universal
state-space resolution engine.

## Installation

astar requires **Go 1.25** or newer and depends on nothing but the standard library.

```bash
go get github.com/kjkrol/astar
```

## The contract

A* finds the cheapest path when, and only when, what it is given keeps to four rules:

1. A `Transition`'s `Cost` is added to the path's cost and is never negative.
2. The `Heuristic` never estimates more than the cheapest cost left to the goal; one that does
   still finds a path, but not the cheapest.
3. A consistent `Heuristic` — never more than a step's cost plus the estimate from the step's
   end — has no state expanded twice; an inconsistent one may reopen a closed state, which the
   solver does, keeping the path the cheapest.
4. A step that cannot be taken is a `Transition` not generated: there is no cost that means
   "never".

The solver is reused across searches and is not safe for concurrent use: one solver per
goroutine.

## Getting started

A grid whose cells carry a *share of full speed*: 1 open ground, 0.3 mud, 0 a wall. A step costs
its length divided by the share, so no step costs less than its length and the Manhattan
distance never overestimates: rule 2 holds by construction.

```go
type Point struct{ X, Y int }

// a cell's share of full speed: 1 open ground, 0.3 mud, 0 a wall
var shares = [][]float64{
	{1, 1, 1},
	{1, 0.3, 1},
	{1, 0, 1},
}

func main() {
	width, height := len(shares[0]), len(shares)
	// the Manhattan distance never exceeds the cost left: no share exceeds 1, so no step costs
	// less than its length
	manhattan := func(a, b Point) float64 {
		return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y))
	}
	solver := astar.New(manhattan,
		astar.WithIndexedSliceDict(width*height, func(p Point) int { return p.Y*width + p.X }),
		astar.WithSuccessorCapacity[Point](4))

	dirs := [4]Point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
	steps := func(from, prev Point, buf []astar.Transition[Point]) []astar.Transition[Point] {
		for _, d := range dirs {
			to := Point{X: from.X + d.X, Y: from.Y + d.Y}
			if to.X < 0 || to.Y < 0 || to.X >= width || to.Y >= height || to == prev {
				continue
			}
			share := shares[to.Y][to.X]
			if share == 0 { // a wall: a step not generated
				continue
			}
			buf = append(buf, astar.Transition[Point]{To: to, Cost: 1 / share})
		}
		return buf
	}

	fmt.Println(solver.Solve(Point{X: 0, Y: 2}, Point{X: 2, Y: 2}, steps))
	// [{0 2} {0 1} {0 0} {1 0} {2 0} {2 1} {2 2}]
}
```

Over the top costs 6; through the mud 1 + 3.33 + 1 + 1 = 6.33, so the path goes round. Lower
the mud to 0.5 and it wades through: `[{0 2} {0 1} {1 1} {2 1} {2 2}]`.

Three things to know as you scale up:

- `WithIndexedSliceDict(maxSize, indexer)` is the fastest dictionary when every state maps to a
  small integer: the indexer must return a number in `[0, maxSize)`, any other panics.
- `SolveInto(dst, …)` writes the path into your buffer; a search reusing it allocates nothing.
- `Wipe(within)` lets a solver that is idle take its once-in-255-searches housekeeping now.

## Performance

Inside the solver nothing points anywhere: a search's states live in arrays indexed by an
integer id, the open set is a binary heap of keys and ids written by hand, sifting its node as
a hole, and one dictionary holds every state's id, open or closed, under a stamp that a new
search moves on instead of clearing. The paths are the ones v1.1.3 found, to the step.

The lower of six runs on an Intel Core i5-8265U, Linux, v1.1.3 against v1.2.0 alternately: a
full search across a grid with walls from a corner to the far corner (`CostWeight_1.0`), then a
short one (`CostWeight_0.1`, the heuristic dominating), then the grid eight ways with the
octile distance, a board's shape. `BENCHMARKS.md` has every regime, the steps, what was tried
and dropped (a four-ary heap with its least child picked in SIMD among them) and the method.

| Grid | `IndexedSliceDict` | `IndexedMapDict` | `DefaultMapDict` |
| :--- | :--- | :--- | :--- |
| **256x256** | 11.7 → 6.72 ms (−43%) | 20.5 → 11.3 ms (−45%) | 26.1 → 14.8 ms (−43%) |
| **1024x1024** | 208 → 123 ms (−41%) | 487 → 277 ms (−43%) | 663 → 365 ms (−45%) |
| **2048x2048** | 937 → 547 ms (−42%) | | |

| Grid | `CostWeight_0.1`, `IndexedSliceDict` | `Octile` |
| :--- | :--- | :--- |
| **256x256** | 779 → 89 µs (−89%) | 21.6 → 13.3 ms (−38%) |
| **1024x1024** | 1.92 → 0.47 ms (−76%) | 413 → 249 ms (−40%) |
| **2048x2048** | 5.37 → 1.07 ms (−80%) | |

A `Solve` allocates once, the path, where v1.1.3 allocated twice; a `SolveInto` nothing, where
it allocated once.

Memory: `IndexedSliceDict` holds one dictionary of `maxSize` slots, 5 bytes each (an id and a
stamp): 20 MiB for 2048x2048, where v1.1.3 held two of 9 bytes a slot, 72 MiB. A search's nodes
take `sizeof(T) + 16` bytes each, grown by doubling from `WithInitCapacity`.

## Weighted A*

The contract's second rule is a trade you may make knowingly: a heuristic scaled above the
cheapest cost left (`h × w`, `w > 1`, or step costs scaled down) expands far fewer states and
gives up the cheapest path for one found sooner — Weighted A*. The `CostWeight_0.5` and
`CostWeight_0.1` benchmarks run in that regime.

## License

**kjkrol/astar** is licensed under the MIT License. See the LICENSE [file](./LICENSE) for more details.
