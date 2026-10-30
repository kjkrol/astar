// Package graph is the contract between a state space and the solver of package hpastar: an
// Implicit graph, the steps from each state (Transitions, each a Transition) and the estimate of
// the cost left (Heuristic), with an Index for the fastest dictionary and a Layout placing the
// states in a plane for HPA*.
//
// A path is the cheapest where the graph keeps to four rules: a Transition's Cost is added to
// the path's cost and is never negative; the Heuristic never estimates more than the cheapest
// cost left to the goal, or the path found is not the cheapest; a consistent Heuristic, never
// more than a step's cost plus the estimate from the step's end, has no state expanded twice,
// while an inconsistent one may have a closed state opened again, which the search does; a step
// refused is a Transition not generated. A Heuristic scaled above the cheapest cost left
// (Weighted A*) expands fewer states and trades the cheapest path for one found sooner.
//
// A graph may serve several classes of search at once (Implicit.Classes): a step some of them
// cannot take is a Transition with their bits in its Refuses, and a search of class k finds the
// paths a graph of the steps k takes alone would give.
package graph
