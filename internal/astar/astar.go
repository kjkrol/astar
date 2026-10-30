package astar

import (
	"slices"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/heap"
)

// Engine searches a state space by A*, reused across searches and never by two at once.
type Engine[T comparable] struct {
	// a search's states by an integer id: nothing in a search points anywhere, so the collector
	// scans no node of a state without pointers
	nodes []node[T]
	heap  heap.Heap
	// every state's node, open or closed
	dict       dict.Dict[T]
	heuristic  graph.Heuristic[T]
	steps      []graph.Transition[T]
	current    int32
	expansions uint64
}

// a state reached, by the id of the node it was reached from; -1 the start
type node[T comparable] struct {
	ID     T
	G      float64
	Parent int32
}

// the nodes an engine is first sized for and the steps a state is first expected to have, each
// grown by doubling past them and kept
const (
	defaultCapacity   = 1024
	defaultSuccessors = 8
)

// New is an engine estimating by heuristic, its nodes kept in d, first sized for capacity
// nodes, 1024 where capacity is not above 0.
func New[T comparable](heuristic graph.Heuristic[T], d dict.Dict[T], capacity int) *Engine[T] {
	if capacity <= 0 {
		capacity = defaultCapacity
	}
	return &Engine[T]{
		nodes:     make([]node[T], 0, capacity),
		heap:      heap.New(capacity),
		dict:      d,
		heuristic: heuristic,
		steps:     make([]graph.Transition[T], 0, defaultSuccessors),
		current:   -1,
	}
}

// SolveInto writes the cheapest path from from to to into dst, grown only where it is shorter,
// and returns it; dst emptied where to is unreachable.
func (a *Engine[T]) SolveInto(dst []T, from, to T, transitions graph.Transitions[T]) []T {
	a.Start(from, to)
	a.Run(to, transitions, -1)
	return a.ResultInto(dst)
}

// Start begins a search from from to to.
func (a *Engine[T]) Start(from, to T) {
	a.reset()
	n := a.alloc(from, -1, 0)
	a.heap.Push(n, a.heuristic(from, to))
	a.dict.Set(from, n)
}

// Run expands up to n states of the search started, as many as it takes where n is below 0;
// true once the search is over.
func (a *Engine[T]) Run(to T, transitions graph.Transitions[T], n int) bool {
	for ; n != 0; n-- {
		if a.next(to, transitions) {
			return true
		}
	}
	return false
}

// Found says the search run over reached its goal.
func (a *Engine[T]) Found() bool { return a.current >= 0 }

// Expanded is how many states the engine has expanded, its searches together.
func (a *Engine[T]) Expanded() uint64 { return a.expansions }

// next visits the open node of least F; true once the search is over, current then the target
// reached or -1
func (a *Engine[T]) next(to T, transitions graph.Transitions[T]) bool {
	n, ok := a.heap.Pop()
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
func (a *Engine[T]) expandCurrent(to T, transitions graph.Transitions[T]) {
	a.expansions++
	cur := a.current
	curID, curG := a.nodes[cur].ID, a.nodes[cur].G
	parentID := curID
	if p := a.nodes[cur].Parent; p >= 0 {
		parentID = a.nodes[p].ID
	}
	successors := transitions(curID, parentID, a.steps[:0])
	if cap(successors) > cap(a.steps) {
		a.steps = successors[:0]
	}
	for _, transition := range successors {
		G := curG + transition.Cost
		F := G + a.heuristic(transition.To, to)

		if slot, found := a.dict.Get(transition.To); found {
			n := slot
			if slot < 0 {
				n = ^slot
			}
			if a.nodes[n].G <= G {
				continue
			}
			if slot >= 0 {
				a.nodes[n].Parent, a.nodes[n].G = cur, G
				a.heap.Fix(n, F)
				continue
			}
		}
		n := a.alloc(transition.To, cur, G)
		a.heap.Push(n, F)
		a.dict.Set(transition.To, n)
	}
	a.dict.Set(curID, ^cur)
}

func (a *Engine[T]) alloc(id T, parent int32, g float64) int32 {
	n := int32(len(a.nodes))
	a.nodes = append(a.nodes, node[T]{ID: id, G: g, Parent: parent})
	return n
}

// ResultInto writes the path from the start to the state the search is at over dst, grown only
// where it is shorter than the path; with none it returns dst emptied.
func (a *Engine[T]) ResultInto(dst []T) []T {
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

// Prefix writes as many states of the path found as dst holds, from the start, and reports how
// many.
func (a *Engine[T]) Prefix(dst []T) int {
	n := 0
	for i := a.current; i >= 0; i = a.nodes[i].Parent {
		n++
	}
	k := min(n, len(dst))
	i := a.current
	for range n - k {
		i = a.nodes[i].Parent
	}
	for w := k - 1; w >= 0; w-- {
		dst[w] = a.nodes[i].ID
		i = a.nodes[i].Parent
	}
	return k
}

func (a *Engine[T]) reset() {
	a.nodes = a.nodes[:0]
	a.heap.Reset()
	a.dict.Clear()
	a.current = -1
}
