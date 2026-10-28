// Package astar finds the cheapest sequence of states from one state to another by A*, over any
// state space a Heuristic and a Transitions describe.
//
// The contract: a Transition's Cost is added to the path's cost and is never negative; the
// Heuristic never estimates more than the cheapest cost left to the goal, or the path found is
// not the cheapest; a consistent Heuristic, never more than a step's cost plus the estimate from
// the step's end, has no state expanded twice, while an inconsistent one may have a closed state
// opened again, which the Solver does; a step refused is a Transition not generated.
//
// A Heuristic scaled above the cheapest cost left (Weighted A*) expands fewer states and trades
// the cheapest path for one found sooner.
//
// A Solver is reused across searches and is not safe for concurrent use.
package astar
