package hpastar

import (
	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/hierarchy"
)

// Solver finds paths over one graph, made once and shared by the Searches of many goroutines.
// With a Layout it keeps the graph's HPA* hierarchy, rebuilt where the graph changes.
type Solver[T comparable] struct {
	graph graph.Implicit[T]
	h     *hierarchy.Hierarchy[T] // nil without a Layout
}

// NewSolver is a Solver over g, its hierarchy laid out where g has a Layout but built by Update
// alone. It panics where g lacks Forward or Heuristic, or has a Layout lacking Coords, At,
// Backward, a plane or the graph's Index.
func NewSolver[T comparable](g graph.Implicit[T], opts ...Option) *Solver[T] {
	if g.Forward == nil || g.Heuristic == nil {
		panic("hpastar: a graph needs Forward and Heuristic")
	}
	s := &Solver[T]{graph: g}
	if l := g.Layout; l != nil {
		if l.Coords == nil || l.At == nil || l.Backward == nil || l.Width <= 0 ||
			l.Height <= 0 || g.Index == nil {
			panic("hpastar: a Layout needs Coords, At, Backward, a plane and the graph's Index")
		}
		s.h = hierarchy.New(g, configOf(opts))
	}
	return s
}

// Changed marks for Update the clusters a change to the steps from or into state may alter, the
// graph already changed: state's own and those across the borders it lies on. Without a Layout
// it does nothing.
func (s *Solver[T]) Changed(state T) {
	if s.h != nil {
		s.h.Stale(state)
	}
}

// ChangedAll marks every cluster of the hierarchy's first level to be rebuilt, the levels above
// following where those change.
func (s *Solver[T]) ChangedAll() {
	if s.h != nil {
		s.h.StaleAll()
	}
}

// Update rebuilds what changes left stale, budget steps of work (a step about an edge a fill
// relaxes) and the step that spends them, all of it for a budget of 0, and reports how many
// clusters it finished and how many are left stale. The same calls make the same progress on any
// machine; without a Layout it does nothing.
func (s *Solver[T]) Update(budget int) (built, left int) {
	if s.h == nil {
		return 0, 0
	}
	return s.h.Build(budget)
}

// Ready says nothing is left stale, so a Path between ends far apart climbs the hierarchy;
// without a Layout it is always true.
func (s *Solver[T]) Ready() bool { return s.h == nil || s.h.Ready() }

// Search is a Search over the graph's own steps.
func (s *Solver[T]) Search() *Search[T] { return s.SearchVia(nil, nil) }

// SearchVia is a Search over forward and backward, read as its goroutine reads them, which step
// and price as the graph's do, in the same order; a nil one takes the graph's.
func (s *Solver[T]) SearchVia(forward, backward graph.Transitions[T]) *Search[T] {
	if forward == nil {
		forward = s.graph.Forward
	}
	q := &Search[T]{solver: s, forward: forward}
	if s.h != nil {
		if backward == nil {
			backward = s.graph.Layout.Backward
		}
		q.climb = s.h.NewClimb(forward, backward)
	}
	return q
}
