// Package hpastar is a universal solver of paths over a state space: the cheapest sequence of
// states from one state to another, found by A* over any graph a Transitions and a Heuristic
// describe, a puzzle, a plan or a grid alike, and by HPA* between ends far apart where the states
// lie in a plane.
//
// A Solver is made once over a graph.Implicit by NewSolver and shared; each goroutine asks it
// through a Search of its own, whose Path writes the path into the caller's buffer and allocates
// nothing once warm. Reaches says whether a path exists at all, and Stats counts what a Search did.
//
//	import (
//		"github.com/kjkrol/hpastar"
//		"github.com/kjkrol/hpastar/graph"
//	)
//
//	solver := hpastar.NewSolver(graph.Implicit[point]{Forward: steps, Heuristic: manhattan})
//	search := solver.Search()
//	n, found := search.Path(from, to, dst) // dst[:n], from first
//
// A path is the cheapest where its graph keeps the contract package graph states: a Transition's
// Cost added to the path's and never negative, a Heuristic never estimating more than the
// cheapest cost left, a step refused a Transition not generated. A Heuristic scaled above the
// cheapest cost left (Weighted A*) expands fewer states and trades the cheapest path for one found
// sooner.
//
// # HPA*
//
// Where the states lie at integer coordinates in a plane (a square or a hex grid, a torus
// through its Transitions), a graph's Layout has paths between ends far apart found by HPA*, not
// A*: Hierarchical Path-Finding A* (Botea, Müller and Schaeffer, 2004). The plane is cut into
// clusters, and those into clusters of clusters, level over level; each cluster keeps entrances
// on its borders and the cheapest paths between them within it; a path is searched over the
// entrances of the highest level its ends' distance allows and refined down the paths kept only
// as far as the caller's buffer holds, so a path asked again from its last state goes on from
// there cheaply. Its paths cost about 1.02 times the cheapest on average and 1.05 at worst on the
// tests' maps. Ends nearer than WithNear asks, a state Coords places nowhere and any path asked
// while the hierarchy is not Ready are found by A*, the cheapest; ends no path joins are told
// apart off the hierarchy's labels of connected entrances, not by searching all a start reaches.
//
// The graph may change while its Solver lives: a step added or taken away, the cost of a step or
// the classes it refuses changed. Changed marks the first-level clusters a changed state's steps
// run through, ChangedAll every one, and Update rebuilds them, and the clusters above only as far
// as the change reaches, a budget of work at a time: the same calls make the same progress on any
// machine. Paths asked meanwhile are found by A*.
//
// # Classes
//
// A graph may serve up to 8 classes of search at once, each with steps of its own: a step some
// classes cannot take carries their bits in its Refuses (graph.Implicit's Classes). A Search asks
// for one class at a time (Search.Class), its paths, reaches and expansions those of a Solver over
// the steps the class takes alone; the hierarchy keeps a view of each class, read off the graph
// once a cluster, so its memory and its fills grow with the classes. A graph of one class reads
// no Refuses.
//
// # Concurrency
//
// The Searches of one Solver may ask at once, each on its own goroutine, and the functions of
// its graph and Layout are called by all of them at once; Changed, ChangedAll and Update run
// between their asks, never during one.
//
// # Packages
//
// The module's root holds the Solver, the Search and their options, and package graph the
// contract they take. The engines behind them are internal, a package each: the A* search, its
// heap and its dictionaries, the fill from several sources, and the hierarchy of HPA* with its
// build and its climbs. The Solver and the Search call the engines, the hierarchy calls A* and
// the fill, and neither of those knows the hierarchy.
package hpastar
