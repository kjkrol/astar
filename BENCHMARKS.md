# Benchmarks

Every number here is the lower of six runs (`-count=2`, three rounds) of a baseline copy of the
tree run alternately with the working tree, on an idle Intel Core i5-8265U (4 cores, 8 threads,
AVX2, no AVX-512), Linux, Go 1.27.1, `-benchtime=10x -cpu=1`, v1.1.3 against v1.2.0 on
2026-10-28 unless said otherwise. The suite is `go test -run '^$' -bench . -benchmem`
(`make bench`); `make bench-save` keeps five runs under `bench_results/`.

## The regimes

- `Solver_Solve/Size_NxN/CostWeight_W/<dictionary>`: a route from a corner to the far corner of
  a grid of costs 1–5 with walls (`setupGrid`), four ways, the Manhattan distance, the path
  allocated each search. Weight 1.0 is a full search: the heuristic admissible, every
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
make bench            # every benchmark once, with allocations, no test
make bench-simd       # the same built with GOEXPERIMENT=simd
make bench-save       # five runs into bench_results/ (bench-save-simd: the SIMD build)
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
