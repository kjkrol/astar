package astar

import (
	"container/heap"
	"iter"
	"slices"
)

// Heuristic defines the estimated cost to travel from one state to another.
//
// Note on Performance vs. Optimality:
// To drastically narrow the search space and speed up the solver, the heuristic's
// contribution should dominate the actual step cost. You can achieve this by multiplying
// the heuristic result by a weight (e.g., > 1.0) or by reducing the step cost.
// This effectively turns the algorithm into Weighted A*, which visits far fewer nodes
// but sacrifices the guarantee of finding the absolute shortest path.
type Heuristic[T comparable] func(from, to T) float64

// Transition represents a valid movement or mutation in the state space,
// capturing the destination state and the cost incurred to reach it.
type Transition[T comparable] struct {
	To   T       // The destination state of this transition.
	Cost float64 // The edge weight or cost associated with this state change.
}

// Transitions defines how steps between states are discovered and how their costs are calculated.
// It populates the provided reusable buffer with all valid, directly reachable transitions
// from the 'from' state in a single pass.
//
// This decoupled design keeps the Solver strictly generic and agnostic of the underlying
// graph structure, state representation, or domain-specific cost metrics. It is the
// ideal place to:
//  1. Evaluate valid neighbor nodes and compute dynamic transition costs (edge weights).
//  2. Filter out invalid, blocked, or out-of-bounds destinations.
//  3. Prevent immediate backtracking or simple cycles by utilizing the 'prev' state.
//  4. Perform early pruning by excluding transitions whose cost exceeds custom thresholds.
//
// Returns the sliced buffer containing the valid transitions.
type Transitions[T comparable] func(from, prev T, buffer []Transition[T]) []Transition[T]

// Indexer maps a complex state of type T to a unique, contiguous integer identifier.
// It is required when using highly optimized internal structures like IndexedSliceDict.
type Indexer[T comparable] func(T) int

// Solver is a generic, high-performance pathfinding and state-space search engine
// based on the A* algorithm. It is entirely agnostic to the underlying domain problem.
type Solver[T comparable] struct {
	open          *open[T]
	closed        *closed[T]
	heuristic     Heuristic[T]
	transitionBuf []Transition[T]
	current       *node[T]
}

// New initializes and returns a new Solver configured with the provided
// heuristic and optional configuration parameters.
func New[T comparable](
	heuristic Heuristic[T],
	opts ...SolverOption[T],
) *Solver[T] {
	cfg := newConfig[T]()

	for _, opt := range opts {
		opt(&cfg)
	}

	openDict := cfg.dictFactory(cfg.capacity)
	closedDict := cfg.dictFactory(cfg.capacity)

	return &Solver[T]{
		open:          newOpen[T](cfg.capacity, openDict),
		closed:        newClosed[T](closedDict),
		heuristic:     heuristic,
		transitionBuf: make([]Transition[T], 0, cfg.transitionsCap),
	}
}

// Solves the optimal transition sequence between the 'from' and 'to' states.
// It returns the full sequence of states, or nil if no solution exists.
func (a *Solver[T]) Solve(from, to T, transitions Transitions[T]) []T {
	for range a.Iter(from, to, transitions) {
	}
	return a.Result()
}

// Iter yields false while searching and true once the 'to' state is reached.
// The sequence terminates immediately after reaching the target, or ends
// without ever yielding true if 'to' is unreachable — Result then returns nil.
func (a *Solver[T]) Iter(from, to T, transitions Transitions[T]) iter.Seq[bool] {
	a.reset()
	a.open.insert(from, nil, 0, a.heuristic(from, to))

	return func(yield func(bool) bool) {
		for {
			node, ok := a.open.pop()
			if !ok {
				a.current = nil
				return
			}
			a.current = node

			if a.current.ID == to {
				yield(true)
				return
			}

			a.expandCurrent(to, transitions)

			if !yield(false) {
				return
			}
		}
	}
}

func (a *Solver[T]) expandCurrent(to T, transitions Transitions[T]) {
	parentID := a.current.ID
	if a.current.Parent != nil {
		parentID = a.current.Parent.ID
	}
	successors := transitions(a.current.ID, parentID, a.transitionBuf[:0])
	if cap(successors) > cap(a.transitionBuf) {
		a.transitionBuf = successors[:0]
	}
	for _, transition := range successors {
		G := a.current.G + transition.Cost
		F := G + a.heuristic(transition.To, to)

		inOpen, hasBetter := a.open.containsBetterOrEqual(transition.To, G)
		if hasBetter {
			continue
		}

		inClosed, hasBetter := a.closed.containsBetterOrEqual(transition.To, G)
		if hasBetter {
			continue
		}

		if inClosed {
			a.closed.remove(transition.To)
		}

		if inOpen {
			a.open.update(transition.To, a.current, G, F)
		} else {
			a.open.insert(transition.To, a.current, G, F)
		}
	}
	a.closed.insert(a.current)
}

// Result reconstructs and returns the path from the starting state to the current state.
// It is typically called immediately after the Iter sequence yields 'true'.
func (a *Solver[T]) Result() []T {
	if a.current == nil {
		return nil
	}

	var path []T
	node := a.current

	for node != nil {
		path = append(path, node.ID)
		node = node.Parent
	}

	slices.Reverse(path)

	return path
}

func (a *Solver[T]) reset() {
	a.closed.reset()
	a.open.reset()
	a.current = nil
}

// ---------------
// node
// ---------------
type node[T comparable] struct {
	ID     T
	G, F   float64
	Parent *node[T]
	Index  int
}

// ---------------
// Closed Nodes
// ---------------
type closed[T comparable] struct {
	dict nodeDict[T]
}

func newClosed[T comparable](dict nodeDict[T]) *closed[T] {
	return &closed[T]{dict: dict}
}

func (c *closed[T]) insert(node *node[T]) {
	c.dict.set(node.ID, node)
}

func (c *closed[T]) containsBetterOrEqual(id T, newG float64) (exists, hasBetter bool) {
	if existingNode, ok := c.dict.get(id); ok {
		exists = true
		hasBetter = existingNode.G <= newG
	}
	return
}

func (c *closed[T]) remove(id T) {
	c.dict.remove(id)
}

func (c *closed[T]) reset() {
	c.dict.clear()
}

// ---------------
// Open Nodes
// ---------------
type open[T comparable] struct {
	openPQ *openNodesPriorityQueue[T]
	dict   nodeDict[T]
	arena  *nodeArena[T]
}

func newOpen[T comparable](capacity int, dict nodeDict[T]) *open[T] {
	openPQ := &openNodesPriorityQueue[T]{}
	heap.Init(openPQ)

	return &open[T]{
		openPQ: openPQ,
		dict:   dict,
		arena:  newNodeArena[T](capacity),
	}
}

func (o *open[T]) insert(id T, parent *node[T], g, f float64) {
	node := o.arena.Get()
	node.ID = id
	node.Parent = parent
	node.G = g
	node.F = f
	node.Index = -1
	heap.Push(o.openPQ, node)
	o.dict.set(node.ID, node)
}

func (o *open[T]) update(id T, parent *node[T], g, f float64) {
	if x, ok := o.dict.get(id); ok {
		x.Parent = parent
		x.G = g
		x.F = f
		heap.Fix(o.openPQ, x.Index)
	}
}

func (o *open[T]) pop() (*node[T], bool) {
	if o.openPQ.Len() == 0 {
		return nil, false
	}
	node := heap.Pop(o.openPQ).(*node[T])
	o.dict.remove(node.ID)
	return node, true
}

func (o *open[T]) containsBetterOrEqual(id T, newG float64) (exists, hasBetter bool) {
	if existingNode, ok := o.dict.get(id); ok {
		exists = true
		hasBetter = existingNode.G <= newG
	}
	return
}

func (o *open[T]) reset() {
	o.dict.clear()
	if o.openPQ != nil {
		clear(*o.openPQ)
		*o.openPQ = (*o.openPQ)[:0]
	}
	o.arena.Reset()
}

// ---------------
// (internal) Open Nodes Priority (by Node.F) Queue
// ---------------
type openNodesPriorityQueue[T comparable] []*node[T]

var _ (heap.Interface) = (*openNodesPriorityQueue[any])(nil)

func (q *openNodesPriorityQueue[T]) Push(x any) {
	newNode := x.(*node[T])
	newNode.Index = len(*q)
	*q = append(*q, newNode)
}
func (q *openNodesPriorityQueue[T]) Pop() any {
	old := *q
	n := len(old)
	node := old[n-1]
	old[n-1] = nil
	node.Index = -1
	*q = old[0 : n-1]
	return node
}
func (q *openNodesPriorityQueue[T]) Len() int           { return len(*q) }
func (q *openNodesPriorityQueue[T]) Less(i, j int) bool { return (*q)[i].F < (*q)[j].F }
func (q *openNodesPriorityQueue[T]) Swap(i, j int) {
	(*q)[i].Index = j
	(*q)[j].Index = i
	(*q)[i], (*q)[j] = (*q)[j], (*q)[i]
}
