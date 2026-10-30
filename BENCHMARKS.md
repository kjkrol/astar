# Benchmarks

The machine is an idle Intel Core i5-8265U (4 cores, 8 threads, AVX2, no AVX-512), Linux, Go
1.27.1. The v1.3.0 numbers are one run a scenario (the user's rule since 2026-10-29); the earlier
ones the lower of six runs (`-count=2`, three rounds) of a baseline copy of the tree run
alternately with the working tree, `-benchtime=10x -cpu=1`, v1.1.3 against v1.2.0 on 2026-10-28
unless said otherwise. The suite is `go test -run '^$' -bench . -benchmem` (`make bench`);
`make bench-save` keeps five runs under `bench_results/`.

## v1.3.0: one Solver, A* and HPA*

One run a scenario, `-count=1 -cpu=1`, a plain build, the test binaries compiled before the
first run, the 1-minute load 0.44 as it began, 2026-10-29, about four minutes in all. The map is
`testmap.New(1, N, 0.05, false)` (`internal/testmap`): shares 0.13–1 from smooth noise, 5% of
the cells walls in short lines; the pair is `FarPair`, an eighth of the side in from opposite
corners, eight ways, the octile distance. The old side is a throwaway program outside the
repository with v1.2.0 from the proxy and a copy of the generator (`SolveInto` a reused buffer,
`WithIndexedSliceDict`, `WithInitCapacity` of the map, as `Solver_OctileFar` ran); the new side
is `Path/{astar,hpa}`. One build cannot hold two versions of one module path, so the two ran one
after the other; A* on both expands the same states to the same path, so the map and the pair
are the same.

A path between the far pair, its time · the states expanded:

| Map | v1.2.0 `SolveInto` | v1.3.0 `Path`, no `Layout` (A*) | v1.3.0 `Path`, a `Layout` (HPA*) |
| :--- | ---: | ---: | ---: |
| **256x256** | 21.9 ms · 52,153 | 22.0 ms (+0.6%) · 52,153 | 0.562 ms (39×) · 1,560 |
| **1024x1024** | 347 ms · 736,844 | 350 ms (+0.8%) · 736,844 | 1.38 ms (251×) · 3,319 |
| **4096x4096** | 7.57 s · 12,450,466 | 7.51 s (−0.8%) · 12,450,466 | 28.8 ms (263×) · 47,221 |

Every row allocates nothing, 0 B a path. A* costs 666.8, 2,429 and 9,340 over 239, 1,007 and
4,238 states; HPA* 679.2 (1.019×), 2,467 (1.016×) and 9,519 (1.019×) over 252, 1,035 and 4,454.
The A* rows of 4096x4096 ran one path each (`-benchtime=1x`). `Path` without a `Layout` is
v1.2.0's search to the expansion, within one run's noise of its time.

The memory held, the heap after a collection with the Solver built whole (`Update/build`): 43.7
bytes a cell on 256x256 (2.9 MB), 47.8 on 1024x1024 (50.1 MB), 33.4 on 4096x4096 (560 MB, its
first-level trees capped by `WithTreeMemory`'s 64 MB). A `Search` adds, once it searches by A*,
a dictionary of `Size` slots, 5 bytes each (0.3, 5.2 and 84 MB), and its nodes as a search
grows them; v1.2.0's `Solver` held the same dictionary and its nodes sized up front.

The build and the rebuild after one cell changed, walled off and opened again in turn
(`Update/change`: amid a first-level cluster near the map's middle; `Update/change-border`: on
the borders of every level there):

| Map | Built whole | Allocated by the build | A change amid a cluster | A change on every border |
| :--- | ---: | ---: | ---: | ---: |
| **256x256** | 163 ms | 3.55 MB, 9,061 allocations | 3.79 ms, 2 clusters | 12.2 ms, 6 clusters |
| **1024x1024** | 2.99 s | 62.1 MB, 144,412 | 24.4 ms, 3 | 73.2 ms, 10 |
| **4096x4096** | 49.9 s | 741 MB, 2,264,183 | 200 ms, 4 | 265 ms, 8 |

The rebuild by level, milliseconds an update (`l1-ms/op`… and `labels-ms/op`, the first level's
labels of connected entrances, worked out whole after any first-level rebuild):

| Map, change | Level 1 | Level 2 | Level 3 | Level 4 | Labels |
| :--- | ---: | ---: | ---: | ---: | ---: |
| **256x256**, amid | 0.57 | 2.80 | | | 0.42 |
| **256x256**, borders | 1.19 | 10.6 | | | 0.43 |
| **1024x1024**, amid | 0.65 | 2.96 | 13.0 | | 7.75 |
| **1024x1024**, borders | 1.30 | 11.3 | 52.7 | | 7.90 |
| **4096x4096**, amid | 0.54 | 2.78 | 13.5 | 49.7 | 133 |
| **4096x4096**, borders | 1.10 | 5.34 | 27.5 | 102 | 129 |

The first-level cluster a change lies in costs about half a millisecond on every map; the
clusters above it and the labels are the rest, and they grow with the map: 97% of a change amid
a cluster on 1024x1024 (the third level 53%, the labels 32%), 99.7% on 4096x4096 (the labels
66%, the fourth level 25%). A budget of clusters weighs a fourth-level cluster, 50–100 ms, as a
first-level one, half a millisecond.

The engines against v1.2.0, `Solver_Octile/Size_256x256` once a side: 12.46 ms on the v1.2.0
copy, 11.80 ms in the tree (−5%, 0 allocations both); the search is the same code but for a
count of the states expanded and the marks wiped a part a search, so the gap is one run's
noise.

### The rebuild only as far as a change reaches, in a budget of work

One run a scenario on 2026-10-30, `-count=1 -cpu=1`, a plain build, the test binaries compiled
before the first run, the 1-minute load 0.41 as it began, about eight minutes in all: the base a
copy of the tree at `aae2aec` (above), the tree `Update(budget)` counting work. A cluster built
again now leaves the one above as it is where its entrances, their classes and twins and its
costs between them come out the same, has it fill its costs and trees again over the entrances
it keeps where its costs alone changed, and has it build whole otherwise; the labels stay where
a first-level cluster's connectivity holds, and are named again for the changed clusters alone
where it does not (`labels-ms/op` 0.026 on the borders). The rebuild by level, milliseconds an
update, base → tree:

| Map, change | Level 1 | Level 2 | Level 3 | Level 4 | Labels | Update |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: |
| 256x256, amid | 0.57 → 0.58 | 2.80 → 2.76 | | | 0.42 → 0 | 3.79 → 3.34 |
| 256x256, borders | 1.21 → 1.21 | 10.8 → 10.6 | | | 0.43 → 0.03 | 12.4 → 11.9 |
| 1024x1024, amid | 0.65 → 0.64 | 2.98 → 2.91 | 13.0 → 12.9 | | 7.80 → 0 | 24.5 → 16.5 |
| 1024x1024, borders | 1.32 → 1.30 | 11.5 → 11.1 | 56.2 → 52.2 | | 8.02 → 0.03 | 77.1 → 64.7 |
| 4096x4096, amid | 0.53 → 0.54 | 2.79 → 2.74 | 13.5 → 13.4 | 50.1 → 0 | 129 → 0 | 196 → 16.7 |
| 4096x4096, borders | 1.08 → 1.11 | 5.35 → 2.51 | 27.3 → 13.6 | 102 → 0 | 129 → 0 | 266 → 17.2 |

An update costs 12%, 33% and 92% less amid a cluster (256x256, 1024x1024, 4096x4096), 4%, 16%
and 94% less on the borders. The labels are the gain: kept where a change cuts no connection,
named again for two clusters in 26 µs where they were worked out whole. A cluster whose costs
alone changed fills them again for as long as it built them whole (the third level 13.0 →
12.9 ms): its fills are its cost, the choice of entrances a few per cent. Its level above is
left as it is where its costs between its entrances come out the same: on 4096x4096 the
third-level cluster's do, so the fourth level, 50–100 ms, is not touched; there the cell on the
borders changes no entrance, and the cluster on its other side comes out the same, so one
cluster a level above is filled again where two were built. Every rebuild equals a whole build
of the changed map, to the bit (tested; at 4096x4096 checked once for both cells, walled and
opened).

The work is counted in steps of about an edge a fill relaxes (a transition read off the graph
weighs 3, a state settled 2, four checks of an edge kept 1): 8.2–10.2 ns a step here, the
changes 0.34–7.6 M steps, a whole build 17.5 M (256x256) and 322 M (1024x1024). A cluster is
built in steps (read and its entrances chosen, a fill from each entrance, a node's row of edges
kept, finished), so a budget spreads a high cluster over calls; a call stops on the step that
spends it. A whole build costs what it did, 162 → 164 ms, 2.97 → 3.00 s and 49.9 → 49.6 s (one
run's noise; 4096x4096 once, alone, 5.41 G steps), with 0.35–0.43 bytes a cell more (33.4 → 33.8
on 4096x4096) and one allocation a first-level cluster more (its labels); the labels worked out
whole, the fallback where a component may have come apart, take 4.6 ms on 1024x1024 and 74 ms
on 4096x4096 (7.8 and 129 ms before), a chunk of clusters a step.

In gram (`plugins/navigation`, its board of 128x128 cells over relief and its routing board of
200x200, the same runs): `PathFinder_Hierarchy/rebuild/relief` 3.00 → 2.88 ms, `Routing_Path`
3.64 → 1.91 ms and `Routing_Long` 5.46 → 3.66 ms, whose `Touch(0)` an op now rebuilds its cluster
alone where it rebuilt the one above and the labels too. A step there costs about 33 ns on a
first-level cluster, its board's steps read over relief, and 11 ns above.

### The module as hpastar, a package a function

One run a side on 2026-10-30, `-count=1 -cpu=1`, a plain build, the test binaries compiled before
the first run, the base (a copy of the tree at `31d48dc`, the module still `astar`) then the
tree, the 1-minute load 1.01 as it began (a browser at a third of a core), half a minute a side.
The code moved between packages and none of it changed: both sides expand the same states to the
same costs and lengths, count the same steps of work and allocate as often.

| Benchmark | Base | Tree | |
| :--- | ---: | ---: | ---: |
| `Path/astar/Size_256x256` | 25.0 ms | 25.7 ms | +2.9% |
| `Path/astar/Size_1024x1024` | 408 ms | 416 ms | +2.1% |
| `Path/hpa/Size_256x256` | 0.614 ms | 0.630 ms | +2.5% |
| `Path/hpa/Size_1024x1024` | 1.560 ms | 1.563 ms | +0.2% |
| `Update/build/Size_256x256` | 181 ms | 179 ms | −1.1% |
| `Update/change/Size_256x256` | 3.65 ms | 3.62 ms | −0.7% |
| `Update/change-border/Size_256x256` | 13.0 ms | 12.7 ms | −2.3% |
| `Update/build/Size_1024x1024` | 3.38 s | 3.24 s | −4.2% |
| `Update/change/Size_1024x1024` | 18.2 ms | 17.7 ms | −2.4% |
| `Update/change-border/Size_1024x1024` | 71.9 ms | 68.9 ms | −4.2% |

Every benchmark lies within 5% of the base, so no second pair ran; this run's times are higher
than the table above on both sides (the machine, not the code). A path allocates nothing either
side. A build allocates as often (9,321 and 148,512 times) and 32 bytes fewer (3,579,834 →
3,579,797 and 62,593,856 → 62,593,824 B): the root's config now holds only what the public
options set, and it escapes once a `NewSolver`, in 32 bytes instead of 64. The updates' bytes
an op move with their count of runs, a state walled and opened in turn.

## The regimes

`Path` and `Update` are the public API's, in the root. The engines' benchmarks are in their
packages: `Solver_*` the A* engine's, under the names they had in v1, in `internal/astar`;
`Dijkstra_Fill` in `internal/dijkstra`; `Hierarchy_*` in `internal/hierarchy`.

- `Solver_Solve/Size_NxN/CostWeight_W/<dictionary>`: a route from a corner to the far corner of
  a grid of costs 1–5 with walls (`testmap.WalledGrid`), four ways, the Manhattan distance, the
  path allocated each search. Weight 1.0 is a full search: the heuristic admissible, every
  alternative weighed. Weights 0.5 and 0.1 scale the step costs down under the same heuristic,
  so it overestimates (Weighted A*): a short search where the reset and the first expansions
  dominate; the walls, tested after the weight, are passable there at 50 and 10.
- `Solver_SolveInto/Size_NxN/WithIndexedSliceDict`: weight 1.0 into a reused buffer, the
  caller's shape: no allocation.
- `Solver_Octile/Size_NxN/WithIndexedSliceDict`: the same grid eight ways, a slantwise step √2
  of the cell's cost, the octile distance, eight successors a state: a board's planner.

## v1.1.3 → v1.2.0

A full search (`CostWeight_1.0`), `Solve`:

| Grid | `IndexedSliceDict` | `IndexedMapDict` | `DefaultMapDict` |
| :--- | :--- | :--- | :--- |
| **256x256** | 11.7 → 6.72 ms (−43%) | 20.5 → 11.3 ms (−45%) | 26.1 → 14.8 ms (−43%) |
| **1024x1024** | 208 → 123 ms (−41%) | 487 → 277 ms (−43%) | 663 → 365 ms (−45%) |
| **2048x2048** | 937 → 547 ms (−42%) | not measured | not measured |

The medians of the six lie within 1% of these minima on the larger grids and within 3% at
256x256; the short searches spread more (medians 4–11% over the minima). `Solve` allocates 1 a
search (the path) instead of 2.

A short search (`CostWeight_0.1`), `Solve`, `IndexedSliceDict`: 256x256 779 → 89 µs (−89%),
1024x1024 1.92 → 0.47 ms (−76%), 2048x2048 5.37 → 1.07 ms (−80%).

`SolveInto`, `IndexedSliceDict`: 256x256 11.6 → 6.67 ms (−43%), 1024x1024 204 → 123 ms (−40%),
2048x2048 909 → 545 ms (−40%); 1 → 0 allocations a search.

`Octile`, `IndexedSliceDict`: 256x256 21.6 → 13.3 ms (−38%), 1024x1024 413 → 249 ms (−40%);
1 → 0 allocations a search.

The SIMD build (`GOEXPERIMENT=simd`) runs the same code: v1.2.0 has no kernel (below).

## The hierarchy as first built (v1.3.0's draft, before the one Solver)

One run a side, `-count=1 -cpu=1`, a plain build, 2026-10-29, through the draft's public
`Hierarchy` and `Solver` (now the climb and the A* engine). The maps are the generator's
(`internal/testmap`): shares 0.13–1 from smooth noise, walls in short lines; the pair is
`FarPair`, across the map.

| Benchmark | Time | States expanded | Allocations |
| :--- | ---: | ---: | ---: |
| `Hierarchy_Find/whole/Size_1024x1024` | 1.35 ms | 3,319 | 0 |
| `Solver_OctileFar/Size_1024x1024` (the same pair) | 358 ms | 736,844 | 0 |
| `Hierarchy_Build/whole/Size_256x256` | 168 ms | — | 9,113 / 3.57 MB |

The hierarchy finds the route across 1024x1024 266× faster than the Solver, expanding 222×
fewer states, at 1.02× the cheapest cost on average over the tests' maps (1.05 at worst, pairs
from 32 apart). Building a whole 1024x1024 takes about 4 s (4,368 clusters, three levels) and
holds about 48 bytes a cell; a 4096x4096 about a minute and 0.5 GB (four levels, behind
`HPASTAR_SCALE=4096`). `Solver_Octile/Size_256x256` against v1.2.0: −2.1%, the Solver's files
untouched (an earlier +5–7% did not come back: the base's first round ran in the CPU's boost).
The example (then `examples/hpa`, 1024x1024, 100 pairs) prints the same picture: a route 122×
faster than the Solver's on average, `Reaches` denying a pair no route joins in 138 µs where
the Solver floods 995,034 states in 621 ms to learn it.

## The steps, each against the tree before it

1. **Nodes in arrays by an integer id, the open set a heap of keys and ids** (no pointer in the
   search; the heap written by hand after `container/heap`, so the paths stayed the same to the
   step): `Solve` weight 1.0 −26…−31% (256x256 10.72 → 7.35 ms, 2048x2048 826 → 603 ms),
   weight 0.1 −70…−85% (256x256 708 → 104 µs), `Octile` −28%, `SolveInto` −26…−31%, the map
   dictionaries at 256x256 −13% (default) and −16% (indexed).
2. **One dictionary for the open and the closed states** (a closed state's id stored as its
   complement; four reads an expansion instead of eight, `pop` touching no dictionary;
   `IndexedSliceDict` 2048x2048 40 → 20 MiB): `Solve` weight 1.0 −12…−14%, weight 0.1 −7…−10%,
   `SolveInto` −10…−12%, `Octile` −12%, the map dictionaries at 256x256 −31…−32%.
3. **The sift as a hole** (the node sifted written once where it lands, the same decisions as
   the swaps, so the paths stayed): `Solve` weight 1.0 −4.6…−5.7%, weight 0.1 −11…−12%,
   `SolveInto` −7…−8%, `Octile` −4%.

## Where the time goes

A CPU profile of `Solve/Size_1024x1024/CostWeight_1.0/WithIndexedSliceDict` after step 2
(760 ms of samples): the caller's `Transitions` 21%, `expandCurrent` itself 18% (the loop, the
dictionary's decode, the heuristic 4% + `math.Abs` 3%), the heap 33% (`swap` 14.5%, `down`
10.5%, `push` 4%, `up` 2.6%, `pop` 1.3%), the slice dictionary's `get` 12% and `set` 1.3%.
Step 3 took the swaps' writes out of the heap's share; what is left is the caller's callbacks,
the dictionary's random reads and the loop.

## Tried and dropped

- **A four-ary heap, scalar** (children `4i+1..4i+4`, the first least on ties), plain build
  against step 3: `Solve` weight 1.0 +0.7…+1.4%, `SolveInto` +0.2…+2.4%, `Octile`
  +1.4…+2.1%, weight 0.1 −8…−11%. The first least of four children is three unpredictable
  branches a level; half the levels do not pay for them on a full search.
- **The least child picked four keys at a time** on that heap (`LoadFloat64x4Array`, the least
  spread over the lanes by two AVX shuffles and a `Min` each, `LessEqual` to bits, the first
  set bit; `GOEXPERIMENT=simd`, `archsimd.X86.AVX2()`, a scalar twin), the SIMD build at
  `-cpu=1` against the binary heap: `Octile` −10% (1024x1024 227 → 204 ms), weight 0.1
  −6…−14%, `Solve` weight 1.0 −0.7…−4.9%, `SolveInto` −3.1…+0.4%; the plain build keeps the
  scalar pick's +1–2%. The same paths in both builds, but another of several equal cheapest
  than the binary heap's: in gram's `Navigation_Tick/crowded,entities=1000` (1,000 walkers
  through one gap, the same gram either way) the units replanned 55% more with it, 7.2 → 11.2
  ms a tick. A gain in the solver's own regime that loses in the caller's is no gain: dropped,
  the heap binary, the paths v1.1.3's. The kernel and its tests are kept outside the repo.
- The successors' G and F arithmetic in SIMD: two additions and a heuristic callback a
  successor, under 16 flops of an expansion of ~200 ns; a batched heuristic would move the work
  into the caller's API for four lanes. Not coded; the profile above is the number.
- The dictionary's compares of up to eight successors in SIMD: archsimd has no gather, so the
  eight random loads of `nodes[n].G` stay scalar and are the cost; the compare they feed is
  free. Not coded.

## How to benchmark

```
make bench            # every package's benchmarks once, with allocations, no test
make bench-simd       # the same built with GOEXPERIMENT=simd
make bench-save       # five runs into bench_results/ (bench-save-simd: the SIMD build)
go test -run '^$' -bench . -benchmem ./internal/astar   # one package's: the A* engine's
```

A change is judged on a copy of the baseline tree run alternately with the working tree on an
idle machine (`cut -d' ' -f1 /proc/loadavg` under 1.5), the same build both sides, `-count=2`,
three rounds, `-benchtime=10x`, `-cpu=1`; gram's `compare.py`
(`/home/karol/CODE/gram/.claude/skills/gram-perf/compare.py`) prints the min and the median a
benchmark with the change in percent. Go matches `-bench` level by level, and a benchmark
shallower than the pattern does not run: `Solver_Solve/…/CostWeight_1.0/WithIndexedSliceDict`
(four levels) and `Solver_(SolveInto|Octile)/…/WithIndexedSliceDict` (three) take two runs. A
gain counts from 3% of the median in the regime it aims at, the two sides' runs apart, no regime
losing — the caller's regimes included: gram's `Navigation_Tick` and `Routing_*` run against
the working tree through `/home/karol/CODE/go.work`. A change to the sift that keeps
`container/heap`'s decisions keeps `testdata/paths.golden`; one that changes which of several
cheapest paths is found rewrites it with `-update`, says why, and changes gram's routes and
crowds with it.
