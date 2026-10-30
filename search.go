package hpastar

import (
	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/astar"
	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/hierarchy"
)

// Search asks a Solver's paths on one goroutine, with memory and counters of its own. The
// Searches of one Solver may ask at once, never while it changes or updates.
type Search[T comparable] struct {
	solver  *Solver[T]
	forward graph.Transitions[T]
	exact   *astar.Engine[T]    // made the first time a path is searched exactly
	climb   *hierarchy.Climb[T] // nil without a Layout
	stats   Stats
}

// Stats counts what a Search has done.
type Stats struct {
	Paths      uint64 // the calls of Path
	Climbs     uint64 // the paths found by climbing the hierarchy, HPA*
	Exact      uint64 // the paths found by A*, the cheapest
	Expansions uint64 // the states expanded, the hierarchy's nodes among them
}

// Path writes the path from from to to into dst, from first, as many states as dst holds, and
// reports how many; false only where no path joins them. It is the cheapest, by A*, unless the
// hierarchy is Ready and the ends far apart, when HPA* climbs it, refined as far as dst holds.
func (q *Search[T]) Path(from, to T, dst []T) (int, bool) {
	q.stats.Paths++
	if c := q.climb; c != nil {
		switch c.Route(from, to) {
		case hierarchy.Climbed:
			q.stats.Climbs++
			return c.Refine(dst), true
		case hierarchy.NoPath:
			return 0, false
		}
	}
	e := q.engine()
	e.Start(from, to)
	over := false
	if c := q.climb; c != nil {
		// past a few clusters' worth of states the labels may tell no path joins the ends,
		// sparing a flood of all from reaches
		if over = e.Run(to, q.forward, c.Patience()); !over && !c.Reaches(from, to) {
			return 0, false
		}
	}
	if !over {
		e.Run(to, q.forward, -1)
	}
	if !e.Found() {
		return 0, false
	}
	q.stats.Exact++
	return e.Prefix(dst), true
}

// Reaches says whether to can be reached from from, by A* without a Layout. With one it reads
// the hierarchy's labels, false only where it cannot: true while they are rebuilt or where an
// end lies off the plane.
func (q *Search[T]) Reaches(from, to T) bool {
	if q.climb != nil {
		return q.climb.Reaches(from, to)
	}
	e := q.engine()
	e.Start(from, to)
	e.Run(to, q.forward, -1)
	return e.Found()
}

// Stats is what the Search has done so far.
func (q *Search[T]) Stats() Stats {
	st := q.stats
	if q.exact != nil {
		st.Expansions += q.exact.Expanded()
	}
	if q.climb != nil {
		st.Expansions += q.climb.Expanded()
	}
	return st
}

func (q *Search[T]) engine() *astar.Engine[T] {
	if q.exact == nil {
		g := &q.solver.graph
		q.exact = astar.New(g.Heuristic, dict.New(g.Index, g.Size), 0)
	}
	return q.exact
}
