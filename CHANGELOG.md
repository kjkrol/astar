# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

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
