# kjkrol/hpastar

<p align="center">
  <img src=".github/docs/img/logo.png" alt="kjkrol/hpastar Logo" width="100">
  <br>
    <a href="https://go.dev">
    <img src="https://img.shields.io/badge/Go-1.25+-00ADD8?style=flat-square&logo=go" alt="Go Version">
  </a>
  <a href="https://pkg.go.dev/github.com/kjkrol/hpastar">
    <img src="https://img.shields.io/badge/GoDoc-Reference-007d9c?style=flat-square&logo=go" alt="GoDoc">
  </a>
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg?style=flat-square" alt="License">
  </a>
  <a href="https://github.com/kjkrol/hpastar/actions">
    <img src="https://github.com/kjkrol/hpastar/actions/workflows/go.yml/badge.svg" alt="Go Quality Check">
  </a>
</p>

**kjkrol/hpastar** is a universal path solver for Go: the cheapest sequence of states from one
state to another, found by A* over any state space a transition function and a heuristic
describe — a puzzle, a plan, a network or a map. Where the states lie in a plane, long paths are
found by HPA* over a hierarchy that follows the graph as it changes.

## What is A*?

The [A* (A-star) algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) is a graph
traversal and state-space search algorithm heavily used in computer science due to its
completeness, optimality, and optimal efficiency. It is widely considered the industry standard
for finding the shortest path or optimal sequence of transitions between nodes.

The solver knows nothing of the domain: you give it the steps a state may take (`Transitions`)
and an estimate of the cost left (`Heuristic`), both of the package `graph`, and it is a
pathfinder, a puzzle solver or a planner alike.

## Installation

hpastar requires **Go 1.25** or newer and depends on nothing but the standard library.

```bash
go get github.com/kjkrol/hpastar
```

The module's root package `hpastar` holds the `Solver`, its `Search` and the options; the
package `graph` holds what a solver is given: the graph, `graph.Implicit`, and its steps and
estimate.

```go
import (
	"github.com/kjkrol/hpastar"
	"github.com/kjkrol/hpastar/graph"
)
```

## The contract

A* finds the cheapest path when, and only when, what it is given keeps to four rules, which the
package `graph` states:

1. A `Transition`'s `Cost` is added to the path's cost and is never negative.
2. The `Heuristic` never estimates more than the cheapest cost left to the goal; one that does
   still finds a path, but not the cheapest.
3. A consistent `Heuristic` — never more than a step's cost plus the estimate from the step's
   end — has no state expanded twice; an inconsistent one may reopen a closed state, which the
   search does, keeping the path the cheapest.
4. A step that cannot be taken is a `Transition` not generated: there is no cost that means
   "never".

## Getting started

A `Solver` is made once over a `graph.Implicit` and shared; each goroutine asks it through a
`Search` of its own, whose `Path` writes the path into your buffer.

A grid whose cells carry a *share of full speed*: 1 open ground, 0.3 mud, 0 a wall. A step costs
its length divided by the share, so no step costs less than its length and the Manhattan
distance never overestimates: rule 2 holds by construction.

```go
package main

import (
	"fmt"
	"math"

	"github.com/kjkrol/hpastar"
	"github.com/kjkrol/hpastar/graph"
)

type point struct{ X, Y int }

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
	manhattan := func(a, b point) float64 {
		return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y))
	}
	dirs := [4]point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
	steps := func(from, prev point, buf []graph.Transition[point]) []graph.Transition[point] {
		for _, d := range dirs {
			to := point{X: from.X + d.X, Y: from.Y + d.Y}
			if to.X < 0 || to.Y < 0 || to.X >= width || to.Y >= height || to == prev {
				continue
			}
			share := shares[to.Y][to.X]
			if share == 0 { // a wall: a step not generated
				continue
			}
			buf = append(buf, graph.Transition[point]{To: to, Cost: 1 / share})
		}
		return buf
	}
	solver := hpastar.NewSolver(graph.Implicit[point]{
		Forward:   steps,
		Heuristic: manhattan,
		Index:     func(p point) int { return p.Y*width + p.X },
		Size:      width * height,
	})

	search := solver.Search()
	path := make([]point, width*height)
	n, found := search.Path(point{X: 0, Y: 2}, point{X: 2, Y: 2}, path)
	fmt.Println(found, path[:n])
	// true [{0 2} {0 1} {0 0} {1 0} {2 0} {2 1} {2 2}]
}
```

Over the top costs 6; through the mud 1 + 3.33 + 1 + 1 = 6.33, so the path goes round. Lower
the mud to 0.5 and it wades through: `[{0 2} {0 1} {1 1} {2 1} {2 2}]`.

Three things to know as you scale up:

- `Index` and `Size` give the fastest dictionary, a slot a state, 5 bytes each: the index must
  lie in `[0, Size)`, any other panics. With an `Index` and no `Size` the indices key a map, and
  with no `Index` the states themselves do.
- `Path` writes as many states as your buffer holds and reports how many; a `Search` reusing its
  memory allocates nothing once warm, and keeps its own house — there is nothing to wipe.
- One `Search` a goroutine: the `Searches` of one `Solver` may ask at once, and the graph's
  functions are called by all of them; `SearchVia` gives a goroutine's `Search` its own
  `Transitions` where it reads the graph its own way. `ExampleSolver_Search` asks from a pool
  of workers, a `Search` each, and changes the graph between two batches. `Stats` counts what
  a `Search` did.

## Long paths: HPA*

Give the graph a `Layout` — where each state lies on a plane of integer coordinates, the
states at a place, and the same steps seen from where they end — and paths between ends far
apart are found by **HPA*** (Hierarchical Path-Finding A*: A. Botea, M. Müller, J. Schaeffer,
*Near Optimal Hierarchical Path-Finding*, Journal of Game Development 1(1), 2004), not A*:

- the plane is cut into clusters of `WithClusterSide` coordinates a side (16), and those into
  clusters of four by four, level over level, until the top has 16 clusters or fewer: 256x256
  has two levels, 1024x1024 three, 4096x4096 four;
- each cluster keeps a few entrances on its borders — the ends of every stretch of the border
  crossed alike, the cheapest crossing every few states, every local minimum of the crossing's
  cost (a road) — and the cheapest paths between them within it, with the trees that walk them;
- `Path` inserts its ends into their clusters, searches the abstract graph of entrances on the
  highest level the ends' distance allows, and refines the path down the trees only as far as
  your buffer holds, so a path asked a piece at a time costs a search of the abstract graph a
  piece.

Its paths cost about 1.02 times the cheapest on average, 1.05 at worst, on the tests' maps.
`Path` decides by itself: ends nearer than `WithNear` (two clusters' sides), a state the
`Layout` places nowhere and any path asked while the hierarchy is not `Ready` are found by A*,
the cheapest; ends no path joins are told off the hierarchy's labels of connected entrances in
microseconds, where A* would search all the start reaches. `Reaches` answers from the same
labels. A square grid, a hex grid and a torus (through its `Transitions`) are all planes; two
states may share a place (a bridge over a ford).

```go
g.Layout = &graph.Layout[point]{
	Coords:   func(p point) (int, int, bool) { return p.X, p.Y, true },
	At:       statesAt, // every state at x, y
	Width:    width,
	Height:   height,
	Backward: stepsInto, // the steps into a state, from where they start
}
solver := hpastar.NewSolver(g)
solver.Update(0) // build the hierarchy whole
```

With a `Layout` the graph needs an `Index`. `At` lists every state `Coords` places at a place,
but for ends of your own (a goal no state is) whose steps stay within their cluster, and
`Backward` mirrors `Forward`. The package's example (`Example_layout`) asks a path a piece at a
time and opens a gap in a wall.

## A graph that changes

The graph may change while its `Solver` lives: a step added or taken away, the cost of a step
changed. Tell the `Solver` which states' steps changed, between asks, and rebuild:

```go
solver.Changed(state)                // its steps, or the steps into it, changed
built, left := solver.Update(budget) // budget steps of work; 0 rebuilds all that is stale
```

`Changed` marks the state's first-level cluster and the ones across the borders it lies on;
`Update` rebuilds them, the lowest level first, and the clusters above only as far as the change
reaches: a cluster whose entrances, their twins and the costs between them come out the same
leaves the one above it as it is; one whose costs alone changed has the one above fill its costs
again over the entrances it keeps; one whose entrances changed has it choose its own again. The
labels of connected entrances stay where a cluster's connectivity holds, the clusters whose
connectivity changed are named again from their neighbours', and the whole is worked out anew
only where a component may have come apart.

The budget counts work, not time: a step is about an edge a fill relaxes (a transition read off
the graph, a state settled and the checks of an edge kept weighed alike), some 0.01 µs on the
test maps. A cluster is built in steps — its graph read and its entrances chosen, a fill from
each entrance, its edges kept a node at a time — and a call stops on the step that spends its
budget, the next going on from there, so a cluster of a high level is spread over as many calls
as its work asks. `built` counts the clusters finished, `left` those still stale, the labels as
one while they are worked out. The same calls make the same progress on any machine: whether a
path climbs the hierarchy or is found by A* never hangs on the machine's speed. While anything
is stale `Path` falls back to A*, so every answer is right in the meantime. On 1024x1024 a whole
build takes 3 s; a cell changed amid the map is followed in 16 ms (its first-level cluster
0.64 ms, the clusters above it the rest, the labels kept), one on the borders of every level in
65 ms; on 4096x4096 either in 17 ms, the fourth level left as it is (`BENCHMARKS.md` has every
level's share).

## Performance

One run a scenario on an Intel Core i5-8265U (Go 1.27.1, `-cpu=1`), the far pair of a map
generated from a seed (shares 0.13–1, 5% walls in short lines), an eighth of the side in from
opposite corners, eight ways: v1.2.0's `SolveInto` against v1.3.0's `Path` without a `Layout`
(A*) and with one (HPA*), time a path · states expanded.

| Map | v1.2.0 `SolveInto` | `Path`, no `Layout` (A*) | `Path`, a `Layout` (HPA*) |
| :--- | ---: | ---: | ---: |
| **256x256** | 21.9 ms · 52,153 | 22.0 ms · 52,153 | 0.562 ms (39×) · 1,560 |
| **1024x1024** | 347 ms · 736,844 | 350 ms · 736,844 | 1.38 ms (251×) · 3,319 |
| **4096x4096** | 7.57 s · 12,450,466 | 7.51 s · 12,450,466 | 28.8 ms (263×) · 47,221 |

Every path allocates nothing once warm. Without a `Layout`, `Path` is v1.2.0's search to the
expansion, within one run's noise of its time; with one, the paths cost 1.016–1.019× the
cheapest here. The hierarchy holds 44–48 bytes a cell (2.9 MB on 256x256, 50 MB on 1024x1024;
34 on 4096x4096, 567 MB, its trees capped at 64 MB), built whole in 0.16 s, 3.0 s and 50 s; a
`Search` adds a dictionary of 5 bytes a slot once it searches by A*. On 1024x1024 the example
below finds 100 random paths 107× faster on average at 1.017× the cheapest cost, and denies a
pair no path joins in 114 µs where A* searches 995,034 states in 464 ms.

`BENCHMARKS.md` has every regime, the method and the history: the A* engine's own benchmarks
keep their v1 names (`Solver_*`, in `internal/astar`), and v1.2.0's search in arrays by an
integer id, a heap of keys and ids and one dictionary is where its speed comes from.

## The example

`examples/maps` sets a `Solver` with a `Layout` against one without on a map generated from a
seed (ground of shares 0.13–1 from smooth noise, walls in short lines, squares walled off as
islands): the hierarchy's build, its time and the memory it holds, then for random pairs at
least 64 apart each one's time and expansions by A* and by HPA* (the whole path and its first
64 states), the cost over the cheapest, and the pairs no path joins, which A* searches the whole
map to deny.

`make example-maps` (1024x1024, 100 pairs) · `go run ./examples/maps -size 256` ·
`go run ./examples/maps -size 4096 -pairs 10 -islands 1` (A* takes seconds a pair there).
Flags: `-size` (from 128; 1024), `-side` 16, `-pairs` 100, `-seed` 1, `-walls` 0.05,
`-islands` 4, `-wrap`.

## Packages

| Package | What it holds |
| :--- | :--- |
| `hpastar` | `Solver`, `NewSolver`, `Search`, `Stats` and the options |
| `hpastar/graph` | the contract: `Implicit`, `Layout`, `Transition`, `Transitions`, `Heuristic` |
| `internal/astar` | the A* engine |
| `internal/dijkstra` | the fill from several sources inserting a path's ends into the hierarchy |
| `internal/hierarchy` | HPA*: the clusters, their labels, their build in steps of work, the climb |
| `internal/heap`, `internal/dict` | the open set and the dictionaries the searches share |

`graph` imports nothing of the module; `heap` and `dict` serve `astar` and `dijkstra`, which
serve `hierarchy`, which the root calls with them.

## Weighted A*

The contract's second rule is a trade you may make knowingly: a heuristic scaled above the
cheapest cost left (`h × w`, `w > 1`, or step costs scaled down) expands far fewer states and
gives up the cheapest path for one found sooner — Weighted A*.

## License

**kjkrol/hpastar** is licensed under the MIT License. See the LICENSE [file](./LICENSE) for more
details.
