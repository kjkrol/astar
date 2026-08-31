# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

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
