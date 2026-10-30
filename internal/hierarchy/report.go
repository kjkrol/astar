package hierarchy

import (
	"fmt"
	"slices"
	"time"
)

// TimeBuilds has Build tell each how long every cluster it builds took, by its level from 1, and
// the labels of level 1, as level 0; nil stops it.
func (h *Hierarchy[T]) TimeBuilds(each func(level int, took time.Duration)) { h.rebuilt = each }

// Rebuilds is what the builds built so far: by level from 0, the clusters built whole and those
// whose costs alone were filled again; the labellings whole and those of changed clusters alone.
func (h *Hierarchy[T]) Rebuilds() (whole, costs []uint64, labelled, relabelled uint64) {
	return slices.Clone(h.rebuilds), slices.Clone(h.refills), h.labels.made, h.labels.local
}

// Worked is the work the last Build did, and the work of its last step.
func (h *Hierarchy[T]) Worked() (build, step int) { return h.worked, h.stepWorked }

// Diff says which cluster of h differs from o's, "" where none does: its nodes, their twins,
// classes and costs, and its trees where both keep them.
func (h *Hierarchy[T]) Diff(o *Hierarchy[T]) string {
	if len(h.levels) != len(o.levels) {
		return fmt.Sprintf("%d levels against %d", len(h.levels), len(o.levels))
	}
	for j := range h.levels {
		for c := range h.levels[j].clusters {
			a, b := &h.levels[j].clusters[c], &o.levels[j].clusters[c]
			if !slices.Equal(a.keys, b.keys) || !slices.Equal(a.states, b.states) ||
				!slices.Equal(a.local, b.local) || !slices.Equal(a.down, b.down) ||
				!slices.Equal(a.class, b.class) || !slices.Equal(a.twinAt, b.twinAt) ||
				!slices.Equal(a.twins, b.twins) || !slices.Equal(a.intra, b.intra) ||
				!slices.Equal(a.offset, b.offset) || a.layers != b.layers {
				return fmt.Sprintf("cluster %d of level %d differs", c, j+1)
			}
			if a.span > 0 && b.span > 0 && (!slices.Equal(a.tree8, b.tree8) ||
				!slices.Equal(a.tree16, b.tree16)) {
				return fmt.Sprintf("the trees of cluster %d of level %d differ", c, j+1)
			}
		}
	}
	return ""
}
