# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [v1.1.2]

### Changed

- `WithIndexedSliceDict`: a new search no longer clears the dictionaries' slices. Each slot
  carries the stamp of the search that wrote it, so a reset is O(1) whatever `maxSize` (the
  slots are wiped once in 2^32 searches, when the stamp wraps); a slot takes 16 bytes instead of
  8. A short search on a large grid is much faster (2048×2048, cost weight 0.1: 4.48 → 2.08 ms),
  a long one 4–10% slower (each read checks the stamp; 256×256, weight 1.0: 9.55 → 10.33 ms).
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
