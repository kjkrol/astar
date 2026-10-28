# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

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
  the eight-way grid; it pops another of several equal cheapest first, and in gram's crowded
  tick of 1,000 walkers the units then replan 55% more. The heap stays binary and the paths
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
