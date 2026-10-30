package hpastar

import "github.com/kjkrol/hpastar/internal/hierarchy"

// Option configures a Solver as NewSolver makes it.
type Option func(*config)

// what NewSolver's options set: the side of the hierarchy's first-level clusters, how far apart a
// path's ends lie for it to climb and the memory of each class's trees
type config struct {
	side, trees int
	near        float64
	nearSet     bool
}

const (
	defaultSide   = 16
	defaultFactor = 4
	defaultTop    = 16
	defaultSpan   = 4
	defaultTrees  = 64 << 20
)

func configOf(opts []Option) hierarchy.Config {
	c := config{side: defaultSide, trees: defaultTrees}
	for _, o := range opts {
		o(&c)
	}
	c.side = max(c.side, 4)
	if !c.nearSet {
		c.near = 2 * float64(c.side)
	}
	return hierarchy.Config{Side: c.side, Factor: defaultFactor, Top: defaultTop,
		Span: defaultSpan, Trees: c.trees, Near: c.near}
}

// WithClusterSide sets the side of a cluster of the hierarchy's first level, in coordinates: 16
// by default, 4 the least.
func WithClusterSide(side int) Option { return func(c *config) { c.side = side } }

// WithNear sets how far apart, by the Heuristic, a path's ends must lie for HPA* to find it,
// four times farther a level up, nearer paths found by A*. It is twice a first-level cluster's
// side by default, for a Heuristic counting a step as 1.
func WithNear(distance float64) Option {
	return func(c *config) { c.near, c.nearSet = distance, true }
}

// WithTreeMemory sets the memory the first level's trees of each class may take, in bytes, 64 MB
// by default; past it a leg is searched within its cluster instead of walked, to the same states.
func WithTreeMemory(bytes int) Option { return func(c *config) { c.trees = bytes } }
