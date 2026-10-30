package dijkstra

import (
	"math"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/heap"
)

// Source is where a fill starts: a state and the cost it is reached at.
type Source[T comparable] struct {
	At   T
	Cost float64
}

// Engine settles states outward from its sources in order of cost over a graph's Transitions,
// within a bound, until its targets are settled, keeping each settled state's cost and parent.
// It is reused across fills and never by two at once.
type Engine[T comparable] struct {
	nodes   []node[T]
	heap    heap.Heap
	dict    dict.Dict[T]
	steps   []graph.Transition[T]
	settled []int32 // nodes, in the order they settled
}

// a state reached, by the id of the node it was reached from; -1 a source, the A* engine's node
type node[T comparable] struct {
	ID     T
	G      float64
	Parent int32
}

// the Parent of a target's node no step has reached yet: out of the heap, at a G above any cost
const unreached = -2

// the nodes a fill is first sized for and the steps a state is first expected to have, each
// grown by doubling past them and kept, as the A* engine's
const (
	defaultCapacity   = 1024
	defaultSuccessors = 8
)

// New is a fill keeping its nodes in d, first sized for capacity nodes, 1024 where capacity is
// not above 0.
func New[T comparable](d dict.Dict[T], capacity int) *Engine[T] {
	if capacity <= 0 {
		capacity = defaultCapacity
	}
	return &Engine[T]{
		nodes:   make([]node[T], 0, capacity),
		heap:    heap.New(capacity),
		dict:    d,
		steps:   make([]graph.Transition[T], 0, defaultSuccessors),
		settled: make([]int32, 0, capacity),
	}
}

// Fill settles states outward from sources over transitions, a step to a state out of within
// not taken (a nil within bounds nothing), until every one of targets is settled or, with no
// targets, all it reaches; it reports how many distinct targets it settled.
func (d *Engine[T]) Fill(
	sources []Source[T],
	transitions graph.Transitions[T],
	within func(T) bool,
	targets []T,
) int {
	d.nodes, d.settled = d.nodes[:0], d.settled[:0]
	d.heap.Reset()
	d.dict.Clear()
	// a target is marked by a node of its own made before any other, so the nodes below marked
	// are the targets'
	for _, t := range targets {
		if _, found := d.dict.Get(t); !found {
			d.dict.Set(t, d.alloc(t, unreached, math.Inf(1)))
		}
	}
	marked := int32(len(d.nodes))
	for _, s := range sources {
		slot, found := d.dict.Get(s.At)
		if !found {
			n := d.alloc(s.At, -1, s.Cost)
			d.heap.Push(n, s.Cost)
			d.dict.Set(s.At, n)
		} else if !(d.nodes[slot].G <= s.Cost) {
			d.reach(slot, -1, s.Cost)
		}
	}
	left := marked
	for {
		cur, ok := d.heap.Pop()
		if !ok {
			break
		}
		d.settled = append(d.settled, cur)
		d.dict.Set(d.nodes[cur].ID, ^cur)
		if cur < marked {
			if left--; left == 0 {
				break
			}
		}
		d.expand(cur, transitions, within)
	}
	return int(marked - left)
}

// expand reaches the states a step from cur, just settled. A settled state is passed by: over
// costs never negative, none is reached cheaper. The current node's fields are copied out
// first: alloc may move the nodes.
func (d *Engine[T]) expand(cur int32, transitions graph.Transitions[T], within func(T) bool) {
	id, g := d.nodes[cur].ID, d.nodes[cur].G
	prev := id
	if p := d.nodes[cur].Parent; p >= 0 {
		prev = d.nodes[p].ID
	}
	steps := transitions(id, prev, d.steps[:0])
	if cap(steps) > cap(d.steps) {
		d.steps = steps[:0]
	}
	for _, step := range steps {
		if within != nil && !within(step.To) {
			continue
		}
		G := g + step.Cost
		if slot, found := d.dict.Get(step.To); found {
			if slot >= 0 && !(d.nodes[slot].G <= G) {
				d.reach(slot, cur, G)
			}
			continue
		}
		n := d.alloc(step.To, cur, G)
		d.heap.Push(n, G)
		d.dict.Set(step.To, n)
	}
}

// reach has node n reached from parent at g, cheaper than before: an open node moves up the
// heap, a target's node no step had reached enters it
func (d *Engine[T]) reach(n, parent int32, g float64) {
	open := d.nodes[n].Parent != unreached
	d.nodes[n].Parent, d.nodes[n].G = parent, g
	if open {
		d.heap.Fix(n, g)
	} else {
		d.heap.Push(n, g)
	}
}

func (d *Engine[T]) alloc(id T, parent int32, g float64) int32 {
	n := int32(len(d.nodes))
	d.nodes = append(d.nodes, node[T]{ID: id, G: g, Parent: parent})
	return n
}

// Cost is the cost the last fill settled s at; false where it did not settle s.
func (d *Engine[T]) Cost(s T) (float64, bool) {
	if slot, found := d.dict.Get(s); found && slot < 0 {
		return d.nodes[^slot].G, true
	}
	return 0, false
}

// Parent is the state the last fill reached s from, s itself where s was settled at its cost as
// a source; false where the fill did not settle s.
func (d *Engine[T]) Parent(s T) (T, bool) {
	slot, found := d.dict.Get(s)
	if !found || slot >= 0 {
		var none T
		return none, false
	}
	if p := d.nodes[^slot].Parent; p >= 0 {
		return d.nodes[p].ID, true
	}
	return s, true
}
