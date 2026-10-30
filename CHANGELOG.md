# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [v1.4.0]

### Added

- Classes: one `Solver` serves up to 8 classes of search, each exactly as a `Solver` over the
  steps it takes alone would, to the path, the reach and the states expanded. A graph's
  `Classes` says how many (0 and 1 one class), a `Transition`'s `Refuses` has the bit of every
  class that cannot take the step (the bits from `Classes` up unread), `Backward` mirrors it, and
  `Search.Class(k)` asks the next `Path` and `Reaches` for class k, 0 at first, allocating
  nothing; `SearchVia`'s transitions may leave out the steps the `Search`'s class refuses. With a
  `Layout` the hierarchy keeps a whole view of each class (its entrances, the costs and trees
  between them, its labels), the graph read once a cluster, and `WithTreeMemory` bounds each
  class's trees on its own; a change to the classes a step refuses is a change of steps for
  `Changed`; the budget counts a fill of a class's node as a step. `ExampleSearch_Class`.
- Tests: each class against a `Solver` of one class over its steps alone, its hierarchy, paths,
  pieces, reaches and expansions, on steps refused within clusters, on crossings, across a border
  alone, one way, a ring shut to a class, two states at a place, hex, torus, without a `Layout`,
  without trees, on three levels, of 2, 3 and 8 classes; changes of classes and walls under
  random budgets against a whole build, each class's labels as A* finds; a change one class alone
  sees rebuilding what it sees; one class reading no `Refuses`, the bits past the classes unread;
  nothing allocated; classes asked at once. Benchmarks `Update/{build,change,change-border}/
  Classes_{2,4,8}/{none,local,scattered}` and `Path/{astar,hpa}/Classes_{2,8}/{first,last}`.

### Changed

- `graph.Transition` has a field more, `Refuses`, a byte: 8 bytes more a transition with its
  padding. A graph of one class reads no `Refuses`, and builds, finds and counts as v1.3.0 did,
  to the byte.

## [v1.3.0]

The public API breaks, yet the version stays 1.x: the module has no user but its author's own.

### Breaking

- The module is `github.com/kjkrol/hpastar`, renamed from `github.com/kjkrol/astar`, and its root
  package `hpastar`. The graph's contract lives in the package `graph`: `Graph` →
  `graph.Implicit`, its fields the same; `Layout`, `Transition`, `Transitions` and `Heuristic` →
  `graph.Layout`, `graph.Transition`, `graph.Transitions` and `graph.Heuristic`.
- `New(heuristic, opts...)` and the `Solver` it made are gone: a `Solver` is now made once over a
  graph by `hpastar.NewSolver(graph.Implicit[T]{Forward: transitions, Heuristic: heuristic})`
  and asked through a `Search` a goroutine (`solver.Search()`, or
  `solver.SearchVia(forward, backward)` where a goroutine reads the graph through transitions of
  its own).
- `Solve`, `SolveInto` → `Search.Path(from, to, dst)`: the path written into `dst` from `from`,
  as many states as `dst` holds, with how many it wrote and whether a path joins the ends (where
  `SolveInto` grew `dst` to the whole path and returned it).
- `Iter`, `Result`, `ResultInto` → none: `Path` writes the path, `Stats` counts the states
  expanded.
- `Wipe` → none: the slot dictionary's marks are wiped a 255th part a search, so no search wipes
  them all.
- `WithIndexedSliceDict(maxSize, indexer)` → `graph.Implicit`'s `Index` and `Size`;
  `WithIndexedMapDict(indexer)` → its `Index` with no `Size`; `Indexer` → `func(T) int`.
- `SolverOption`, `WithInitCapacity`, `WithSuccessorCapacity` → none: the engines size their
  memory themselves and keep it.

### Added

- `Solver[T]` and `NewSolver(graph, opts...)`: one solver over a `graph.Implicit` (`Forward`,
  `Heuristic`, `Index` and `Size`, `Layout`), made once and shared; `Search[T]`, one a
  goroutine, with `Path`, `Reaches` and `Stats` (`Paths`, `Climbs`, `Exact`, `Expansions`). A
  warm `Search` over `Index` and `Size` allocates nothing, with a `Layout` or without;
  `ExampleSolver_Search` asks from a pool of workers, a `Search` each, in batches, the graph
  changed between two.
- HPA* (Hierarchical Path-Finding A*, Botea, Müller and Schaeffer, 2004) where a graph has a
  `Layout`: `Coords`, `At`, `Width` and `Height`, `Backward`. The plane is cut into clusters of
  `WithClusterSide` states a side (16), four by four a level up until the top holds 16 or fewer
  (256x256 two levels, 1024x1024 three, 4096x4096 four); each cluster keeps entrances on its
  borders (the ends of every run of crossings, the cheapest crossing every few states, every
  local minimum of the crossing's cost) and the cheapest paths between them with their trees,
  the dominated edges left out. `Path` climbs the highest level its ends' distance allows (level
  k from `WithNear`, two first-level sides, times four to the k−1) and refines down the trees
  only as far as `dst` holds, so a path asked again from its last state goes on cheaply; its
  paths cost about 1.02× the cheapest on average and 1.05× at worst on the tests' maps. Ends
  nearer than that, a state off the plane and any path asked while the hierarchy is not ready
  are found by A*, the cheapest; ends no path joins are denied off the first level's labels of
  connected entrances (an A* search asks them once it has expanded four clusters' worth of
  states), never by a flood of all the start reaches. `Reaches` reads the same labels.
- A graph that changes: `Changed(state)` marks the first-level clusters a change to the steps
  from or into a state touches, `ChangedAll` every one, and `Update(budget)` rebuilds them, the
  lowest level first, and the clusters above only as far as the change reaches: one whose child's
  entrances, their classes and twins and its costs between them come out the same is left as it
  is, one whose child's costs alone changed fills its own again over the entrances it keeps. The
  budget counts work, not time, in steps of about an edge a fill relaxes (0.01 µs on the test
  maps): a cluster is built in steps, a fill from an entrance a step, so a call stops on the step
  that spends its budget and a large cluster is spread over calls; the same calls make the same
  progress on any machine. `Update` reports the clusters it finished and those left stale; `Ready`.
  The labels of connected entrances stay where a cluster's connectivity holds, are named again
  for the clusters whose connectivity changed alone, and are worked out whole, a chunk at a time,
  only where a component may have come apart. Any order of rebuilds gives the graph a whole build
  gives. `WithTreeMemory` (64 MB) bounds the first level's trees, past it a leg is searched in
  its cluster, to the same states.
- `examples/maps` (`make example-maps`, `MAPS_FLAGS`): a `Solver` with a `Layout` against one
  without on a map from a seed, on the public API alone.
- Tests: the golden paths through `Path` without a `Layout` and through each dictionary, `Path`
  answering every kind of pair, a change followed, many goroutines asking at once, nothing
  allocated once warm; a change inside a cluster rebuilding it alone, a change of costs alone
  refilling the cluster above, a change of connectivity relabelling, the budget bounding the
  work, a rebuild resumed over calls equal to a whole one, the same calls giving the same
  progress, the labels following every change; the engines' own tests kept, in the engines'
  packages. Benchmarks `Path/{astar,hpa}`, `Update/{build,change,change-border}` (bytes a cell
  held, clusters, milliseconds a level and steps an update), the engines' kept under their v1
  names in the engines' packages.

### Changed

- A package a function: the root `hpastar` holds the `Solver`, the `Search` and the options,
  `graph` the contract, and the engines are internal, a package each (`internal/astar` the A*
  search, `internal/heap` its open set, `internal/dict` its dictionaries, `internal/dijkstra`
  the fill, `internal/hierarchy` HPA*), the root calling them and the hierarchy calling A* and
  the fill; `graph` imports nothing of the module, and no package imports one above it. The A*
  engine is v1.2.0's search; its paths are the golden ones, to the step.

### Improved

- A path between ends far apart through a `Layout`, against v1.2.0's `SolveInto` on the far
  pair of a generated map (one run each, `BENCHMARKS.md`): 256x256 21.9 → 0.562 ms (39×),
  1024x1024 347 → 1.38 ms (251×), 4096x4096 7.57 s → 28.8 ms (263×), 52,153 → 1,560,
  736,844 → 3,319 and 12,450,466 → 47,221 states expanded, at 1.016–1.019× the cheapest cost;
  0 B and no allocation a path either way. Without a `Layout`, `Path` is v1.2.0's search to the
  expansion, its time within one run's noise (22.0, 350 ms, 7.51 s).
- The hierarchy holds 44.1, 48.2 and 33.8 bytes a cell (2.9 MB, 50 MB, 567 MB with the trees
  capped), built whole in 164 ms, 3.0 s and 49.6 s; a cell changed amid the map is followed in
  3.3, 16 and 17 ms, on the borders of every level in 12, 65 and 17 ms (one run each, where a
  hierarchy that rebuilt every cluster above the change and its labels whole took 3.8, 24 and
  196 ms, and 12, 77 and 266 ms): the labels kept, or named again for the changed clusters
  alone in 26 µs (worked out whole they took 8 ms on 1024x1024 and 129 ms on 4096x4096, now 4.6
  and 74 ms), and on 4096x4096 the fourth level, 50–100 ms a cluster, left as it is.
- A pair no path joins is denied in microseconds: on 1024x1024 in 114 µs against A*'s 464 ms and
  995,034 states (`examples/maps`).

## [v1.2.0]

### Changed

- `Solver`: a search's states live in arrays indexed by an integer id and nothing in the search
  points anywhere: a node is `sizeof(T) + 16` bytes instead of `+ 32`, the open set is a heap of
  keys and ids written by hand instead of `container/heap` over pointers, sifting its node as a
  hole, and one dictionary holds every state's id, open or closed (the complement), instead of
  two: `WithIndexedSliceDict` keeps 5 bytes a slot instead of two dictionaries of 9 (2048×2048:
  72 → 20 MiB). The paths are the ones v1.1.3 found, to the step. `WithIndexedSliceDict`, cost
  weight 1.0: 256×256 11.7 → 6.72 ms, 1024×1024 208 → 123 ms, 2048×2048 937 → 547 ms
  (−41…−43%); weight 0.1 (a short search, the reset dominating): −76…−89% (256×256 779 → 89
  µs); the grid eight ways with the octile distance: −38…−40% (1024×1024 413 → 249 ms); the map
  dictionaries at 256×256 and 1024×1024: −43…−45% (the lower of six runs, alternately;
  `BENCHMARKS.md`). A `Solve` allocates once, the path, instead of twice; a `SolveInto` nothing
  instead of once.
- Tried and dropped (`BENCHMARKS.md`): a four-ary heap, alone 1–2% slower on the full searches,
  and with its least child picked four keys at a time under `GOEXPERIMENT=simd` 10% faster on
  the eight-way grid; it pops another of several equal cheapest first, and a crowd of 1,000
  agents planning over it replanned 55% more. The heap stays binary and the paths
  v1.1.3's.
- The package, `Heuristic`, `Transition.Cost` and `Transitions` state the A* contract: costs
  added and never negative, the heuristic never more than the cheapest cost left, a consistent
  one never reopening a state, a refused step a transition not generated; Weighted A* stated as
  a trade of the cheapest path for a quicker one.
- `WithIndexedSliceDict`: an index outside `[0, maxSize)` panics (documented).

### Added

- `BENCHMARKS.md`: the method, every regime in both builds, the profile's shares, what was tried
  and dropped.
- `make test-simd`, `bench-simd`, `bench-save` and `bench-save-simd`; `bench` runs the benchmarks
  without the tests. Benchmarks `Solver_SolveInto` (a reused buffer, 0 allocations) and
  `Solver_Octile` (eight ways, the octile distance: a board's shape); `Solver_Solve` reports
  allocations.
- Tests: the paths of six scenes pinned in `testdata/paths.golden` (`-update` rewrites), the
  cheapest path against a Dijkstra reference, a closed state reopened under an inconsistent
  heuristic, zero-cost steps, `prev` at the start, `Iter` broken early, the Indexer's bounds,
  and the heap's order against `container/heap`'s, ties included.
- CI runs the tests on Go 1.25 plain and on Go 1.27 with `GOEXPERIMENT=simd`.
- A skill for the repo, `.claude/skills/astar/SKILL.md`.

### Fixed

- README: the install section named another module and Go 1.23, the Go badge said 1.23+, the
  example priced cells by arbitrary weights; it now shows shares of full speed and the contract.

## [v1.1.3]

### Added

- `Solver.Wipe(within)`: a `WithIndexedSliceDict` solver wipes its stamps between two searches,
  where fewer than `within` are left before a search would wipe them, so a caller with the solver
  idle takes the wipe once every 255 searches off its next search.

## [v1.1.2]

### Added

- `Solver.SolveInto(dst, from, to, transitions)` and `Solver.ResultInto(dst)`: the path written
  over the caller's `dst`, grown only when shorter than the path (`dst` emptied when no path
  exists), so a search reusing its buffer allocates nothing for its result.

### Changed

- `Solve` and `SolveInto` run the search without `Iter`'s range-over-func, and `Result` sizes the
  path once instead of growing it by `append`: `Solve` across 64×64 allocates once instead of 11
  times (256×256: once instead of 13); `SolveInto` into a buffer big enough, not at all.

- `WithIndexedSliceDict`: a new search no longer clears the dictionaries' slices. A byte beside
  each slot holds the stamp of the search that wrote it, so a reset moves the stamp on whatever
  `maxSize`; the stamps are wiped once every 255 searches, when the stamp wraps. A slot takes 9
  bytes instead of 8. A search visiting few nodes on a large grid is much faster (2048×2048, cost
  weight 0.1: 5.33 → 2.20 ms), one visiting many within the noise to 5% slower (each read checks
  the stamp; 64×64, weight 1.0: 555 → 582 µs).
- The solver keeps the transitions buffer `Transitions` returns when it outgrew its own, so
  without `WithSuccessorCapacity` retrieving successors stops allocating once the buffer has
  grown; a `nil` or smaller result leaves it as it was.

## [v1.1.1]

### Fixed

- `Solver.Iter`/`Solve` no longer return a bogus, non-`nil` path when `to` is
  unreachable. `Iter` now clears `current` when the search space is
  exhausted without ever reaching the target, so `Result()` correctly
  returns `nil` — matching `Solve`'s documented contract ("returns the full
  sequence of states, or nil if no solution exists"), which the previous
  behavior silently violated.

## [v1.1.0]

### Changed

- Unified naming conventions and fixed examples.

## [v1.0.4]

### Added

- Introduced the `Transitions` function.

## [v1.0.3]

### Changed

- Pass parent to `SuccessorsFunc` to prevent backtracking.

## [v1.0.1]

### Changed

- Updated public API.

## [v1.0.0]

### Added

- Initial version.
