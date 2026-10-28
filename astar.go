package astar

import (
	"iter"
	"slices"
)

// Heuristic estimates the cheapest cost left from one state to another. It never estimates more
// than that, or the path found is not the cheapest; consistent (see the package), it has no
// state expanded twice.
type Heuristic[T comparable] func(from, to T) float64

// Transition is one step the search may take: the state it reaches and its Cost, added to the
// path's and never negative.
type Transition[T comparable] struct {
	To   T
	Cost float64
}

// Transitions appends to buffer every step from from, prev being the state from was reached from
// (from itself at the start), and returns it; a step refused is one not appended. A larger buffer
// returned is kept by the Solver.
type Transitions[T comparable] func(from, prev T, buffer []Transition[T]) []Transition[T]

// Indexer maps a state to an integer of its own, in [0, maxSize) for WithIndexedSliceDict.
type Indexer[T comparable] func(T) int

// Solver searches a state space by A*: made once by New for a Heuristic, reused across searches,
// never concurrently.
type Solver[T comparable] struct {
	nodes         []node[T]
	heap          openHeap
	dict          nodeDict[T]
	heuristic     Heuristic[T]
	transitionBuf []Transition[T]
	current       int32
}

// a state reached, by the id of the node it was reached from; -1 the start
type node[T comparable] struct {
	ID     T
	G      float64
	Parent int32
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

	capacity := cfg.capacity
	if capacity <= 0 {
		capacity = defaultCapacity
	}

	return &Solver[T]{
		nodes:         make([]node[T], 0, capacity),
		heap:          newOpenHeap(capacity),
		dict:          cfg.dictFactory(cfg.capacity),
		heuristic:     heuristic,
		transitionBuf: make([]Transition[T], 0, cfg.transitionsCap),
		current:       -1,
	}
}

// Wipe wipes the solver's memory of its searches now, between two, where fewer than within
// searches are left before one would wipe it; true where it did. Only a WithIndexedSliceDict
// solver wipes its memory, once every 255 searches.
func (a *Solver[T]) Wipe(within int) bool {
	return a.dict.wipe(within)
}

// Solve is the cheapest sequence of states from from to to, from and to included; nil where to
// is unreachable.
func (a *Solver[T]) Solve(from, to T, transitions Transitions[T]) []T {
	return a.SolveInto(nil, from, to, transitions)
}

// SolveInto is Solve writing the sequence into dst as ResultInto does, so a reused dst spares
// the search the allocation of its result.
func (a *Solver[T]) SolveInto(dst []T, from, to T, transitions Transitions[T]) []T {
	a.start(from, to)
	for !a.next(to, transitions) {
	}
	return a.ResultInto(dst)
}

// Iter runs the search a state at a time, yielding false after each expansion and true once to
// is reached, nothing more where it is unreachable. Result then is the path to the state last
// expanded, nil where none was.
func (a *Solver[T]) Iter(from, to T, transitions Transitions[T]) iter.Seq[bool] {
	a.start(from, to)

	return func(yield func(bool) bool) {
		for {
			if a.next(to, transitions) {
				if a.current >= 0 {
					yield(true)
				}
				return
			}
			if !yield(false) {
				return
			}
		}
	}
}

func (a *Solver[T]) start(from, to T) {
	a.reset()
	n := a.alloc(from, -1, 0)
	a.heap.push(n, a.heuristic(from, to))
	a.dict.set(from, n)
}

// next visits the open node of least F; true once the search is over, current then the target
// reached or -1
func (a *Solver[T]) next(to T, transitions Transitions[T]) bool {
	n, ok := a.heap.pop()
	if !ok {
		a.current = -1
		return true
	}
	a.current = n
	if a.nodes[n].ID == to {
		return true
	}
	a.expandCurrent(to, transitions)
	return false
}

// The dictionary holds a state's node: open as its id, closed as the id's complement, ^id. A
// closed state reached cheaper is opened again as a fresh node, the old one left behind for
// whoever was reached through it. The current node's fields are copied out first: alloc may
// move the nodes.
func (a *Solver[T]) expandCurrent(to T, transitions Transitions[T]) {
	cur := a.current
	curID, curG := a.nodes[cur].ID, a.nodes[cur].G
	parentID := curID
	if p := a.nodes[cur].Parent; p >= 0 {
		parentID = a.nodes[p].ID
	}
	successors := transitions(curID, parentID, a.transitionBuf[:0])
	if cap(successors) > cap(a.transitionBuf) {
		a.transitionBuf = successors[:0]
	}
	for _, transition := range successors {
		G := curG + transition.Cost
		F := G + a.heuristic(transition.To, to)

		if slot, found := a.dict.get(transition.To); found {
			n := slot
			if slot < 0 {
				n = ^slot
			}
			if a.nodes[n].G <= G {
				continue
			}
			if slot >= 0 {
				a.nodes[n].Parent, a.nodes[n].G = cur, G
				a.heap.fix(n, F)
				continue
			}
		}
		n := a.alloc(transition.To, cur, G)
		a.heap.push(n, F)
		a.dict.set(transition.To, n)
	}
	a.dict.set(curID, ^cur)
}

func (a *Solver[T]) alloc(id T, parent int32, g float64) int32 {
	n := int32(len(a.nodes))
	a.nodes = append(a.nodes, node[T]{ID: id, G: g, Parent: parent})
	return n
}

// Result is the path from the start to the state Iter last expanded, the goal once it yielded
// true.
func (a *Solver[T]) Result() []T {
	return a.ResultInto(nil)
}

// ResultInto is Result written over dst, which grows only when shorter than the path; with no
// path it returns dst emptied.
func (a *Solver[T]) ResultInto(dst []T) []T {
	n := 0
	for i := a.current; i >= 0; i = a.nodes[i].Parent {
		n++
	}
	dst = slices.Grow(dst[:0], n)[:n]
	for i, k := a.current, n-1; i >= 0; i, k = a.nodes[i].Parent, k-1 {
		dst[k] = a.nodes[i].ID
	}
	return dst
}

func (a *Solver[T]) reset() {
	a.nodes = a.nodes[:0]
	a.heap.reset()
	a.dict.clear()
	a.current = -1
}
