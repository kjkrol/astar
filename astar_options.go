package astar

// SolverOption configures a Solver as New makes it.
type SolverOption[T comparable] func(*config[T])

// WithInitCapacity is the first size of a search's node storage and of a map dictionary, grown
// by doubling past it; an estimate of the states a search visits spares it the growing.
func WithInitCapacity[T comparable](capacity int) SolverOption[T] {
	return func(cfg *config[T]) {
		cfg.capacity = capacity
	}
}

// WithIndexedSliceDict keeps the states' nodes in a slice of maxSize slots, 5 bytes each, under
// indexer: the fastest dictionary, and the one Wipe serves. An index outside [0, maxSize) panics.
func WithIndexedSliceDict[T comparable](maxSize int, indexer Indexer[T]) SolverOption[T] {
	return func(cfg *config[T]) {
		cfg.dictFactory = func(_ int) nodeDict[T] {
			return newIndexedSliceDict(maxSize, indexer)
		}
	}
}

// WithIndexedMapDict keeps the states' nodes in a map keyed by indexer's integers instead of by
// the states themselves, sparing the hashing of a wide state where the state space is too sparse
// for a slice.
func WithIndexedMapDict[T comparable](indexer Indexer[T]) SolverOption[T] {
	return func(cfg *config[T]) {
		cfg.dictFactory = func(capacity int) nodeDict[T] {
			return newIndexedMapDict(capacity, indexer)
		}
	}
}

// WithSuccessorCapacity sizes the buffer Transitions fills, the most steps a state has (4 or 8
// on a grid), so retrieving successors allocates nothing from the first search.
func WithSuccessorCapacity[T comparable](capacity int) SolverOption[T] {
	return func(cfg *config[T]) {
		cfg.transitionsCap = capacity
	}
}
