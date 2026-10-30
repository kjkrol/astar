package graph

// Transition is one step a search may take: the state it reaches and its Cost, added to the
// path's and never negative.
type Transition[T comparable] struct {
	To   T
	Cost float64
	// Refuses has bit k set where class k cannot take the step, 0 where every class can; a graph
	// of one class never reads it.
	Refuses uint8
}

// Transitions appends to buffer every step from from, prev being the state from was reached from
// (from itself at the start), and returns it; a step refused is one not appended. A larger buffer
// returned is kept for the next call.
type Transitions[T comparable] func(from, prev T, buffer []Transition[T]) []Transition[T]

// Heuristic estimates the cheapest cost left from one state to another. It never estimates more
// than that, or the path found is not the cheapest; consistent (see the package), it has no
// state expanded twice.
type Heuristic[T comparable] func(from, to T) float64

// Implicit is a state space to search, told by the steps from each state rather than listed: its
// steps and the estimate of the cost left, with an Index for the fastest dictionary and a Layout
// for HPA* where the states lie in a plane. Its functions are called by every search at once.
type Implicit[T comparable] struct {
	Forward   Transitions[T]
	Heuristic Heuristic[T]
	// Index gives each state an integer of its own, in [0, Size) for a dictionary of a slot a
	// state (5 bytes each, an index outside panicking); with no Size the integers key a map, and
	// with no Index the states do.
	Index func(T) int
	Size  int
	// Layout lays the states out in a plane; with one, a path between ends far apart is found by
	// HPA* over a hierarchy of clusters, nil keeping every path the cheapest, found by A*.
	Layout *Layout[T]
	// Classes is how many classes the graph serves, up to 8, each taking the steps whose Refuses
	// leaves its bit clear, the bits from Classes up unread; 0 and 1 are one class.
	Classes int
}

// Layout is how a graph's states lie in a plane of Width × Height integer coordinates, for HPA*;
// it needs the graph's Index. Its functions are called by every search at once.
type Layout[T comparable] struct {
	// Coords is where a state lies; a state it places nowhere has its paths found by A*.
	Coords func(T) (x, y int, ok bool)
	// At appends to buf every state Coords places at x, y, but for the ends a caller makes of
	// its own, which step only within their cluster.
	At            func(x, y int, buf []T) []T
	Width, Height int
	// Backward is Forward seen from where its steps end: the states a step leads from into to,
	// each at that step's cost and refused to the classes it refuses.
	Backward Transitions[T]
}
