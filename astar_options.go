package astar

// ---------------
// Solver Options
// ---------------

// SolverOption configures the internal behavior and data structures of the Solver.
type SolverOption[T comparable] func(*config[T])

// WithInitCapacity sets the initial capacity for the solver's internal data structures,
// such as dictionaries and memory arenas. Providing an accurate estimate prevents costly
// dynamic allocations during the search execution. If the capacity is exceeded,
// the underlying structures will automatically grow (typically doubling their size).
func WithInitCapacity[T comparable](capacity int) SolverOption[T] {
	return func(cfg *config[T]) {
		cfg.capacity = capacity
	}
}

// WithIndexedSliceDict configures the solver to use slice-based dictionaries for its
// internal "open" and "closed" sets. Because the underlying slice does not grow dynamically,
// the provided maxSize must represent the absolute maximum number of possible states (nodes).
// It also requires an Indexer function to map node properties to contiguous integer indices.
// Starting a search does not touch the slice, so it costs the same whatever maxSize is.
//
// This is the fastest dictionary implementation available, typically doubling the solver's
// overall performance by completely eliminating map hashing overhead and reducing memory allocations.
func WithIndexedSliceDict[T comparable](maxSize int, indexer Indexer[T]) SolverOption[T] {
	return func(cfg *config[T]) {
		cfg.dictFactory = func(_ int) nodeDict[T] {
			return newIndexedSliceDict[T](maxSize, indexer)
		}
	}
}

// WithIndexedMapDict configures the solver to use standard Go maps backed by an Indexer function.
// By mapping complex state types to simple integer keys, it bypasses Go's interface boxing
// and complex hashing overhead.
//
// This provides a noticeable performance boost (often in the double-digit percentages)
// compared to the default map implementation, while remaining more memory-efficient
// than slice-based dictionaries for problems with sparse state spaces.
func WithIndexedMapDict[T comparable](indexer Indexer[T]) SolverOption[T] {
	return func(cfg *config[T]) {
		cfg.dictFactory = func(capacity int) nodeDict[T] {
			return newIndexedMapDict[T](capacity, indexer)
		}
	}
}

// WithSuccessorCapacity pre-allocates the internal buffer used for fetching neighbor nodes.
// Setting this to the maximum expected number of successors per node (e.g., 4 or 8 for 2D grids)
// guarantees zero allocations for successor retrieval from the very first search iteration.
func WithSuccessorCapacity[T comparable](capacity int) SolverOption[T] {
	return func(cfg *config[T]) {
		cfg.transitionsCap = capacity
	}
}
