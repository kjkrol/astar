// Package hierarchy is the HPA* of package hpastar (Hierarchical Path-Finding A*: Botea, Müller
// and Schaeffer, 2004): a graph laid out in a plane cut into clusters, level over level, each
// keeping entrances on its borders and the cheapest paths between them, rebuilt in steps of work
// as far as a change reaches, and the climbs that search a level and refine a path down them.
package hierarchy
