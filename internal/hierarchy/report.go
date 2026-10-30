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
// whose costs alone were filled again; the labellings whole and those of changed clusters alone,
// every class's.
func (h *Hierarchy[T]) Rebuilds() (whole, costs []uint64, labelled, relabelled uint64) {
	for k := range h.views {
		labelled += h.views[k].labels.made
		relabelled += h.views[k].labels.local
	}
	return slices.Clone(h.rebuilds), slices.Clone(h.refills), labelled, relabelled
}

// Worked is the work the last Build did, and the work of its last step.
func (h *Hierarchy[T]) Worked() (build, step int) { return h.worked, h.stepWorked }

// Diff says which cluster of h differs from o's, as any class sees it, "" where none does: its
// nodes, their twins, classes and costs, and its trees where both keep them.
func (h *Hierarchy[T]) Diff(o *Hierarchy[T]) string {
	if len(h.views) != len(o.views) {
		return fmt.Sprintf("%d classes against %d", len(h.views), len(o.views))
	}
	for k := range h.views {
		if diff := h.DiffView(k, o, k); diff != "" {
			return diff
		}
	}
	return ""
}

// DiffView says which cluster of h as class k sees it differs from o's as its class ko sees it,
// "" where none does, as Diff tells.
func (h *Hierarchy[T]) DiffView(k int, o *Hierarchy[T], ko int) string {
	if len(h.levels) != len(o.levels) {
		return fmt.Sprintf("%d levels against %d", len(h.levels), len(o.levels))
	}
	of := ""
	if len(h.views) > 1 {
		of = fmt.Sprintf(" of class %d", k)
	}
	for j := range h.levels {
		as, bs := h.views[k].levels[j].clusters, o.views[ko].levels[j].clusters
		for c := range as {
			a, b := &as[c], &bs[c]
			if !slices.Equal(a.keys, b.keys) || !slices.Equal(a.states, b.states) ||
				!slices.Equal(a.local, b.local) || !slices.Equal(a.down, b.down) ||
				!slices.Equal(a.class, b.class) || !slices.Equal(a.twinAt, b.twinAt) ||
				!slices.Equal(a.twins, b.twins) || !slices.Equal(a.intra, b.intra) ||
				!slices.Equal(a.offset, b.offset) || a.layers != b.layers {
				return fmt.Sprintf("cluster %d of level %d%s differs", c, j+1, of)
			}
			if a.span > 0 && b.span > 0 && (!slices.Equal(a.tree8, b.tree8) ||
				!slices.Equal(a.tree16, b.tree16)) {
				return fmt.Sprintf("the trees of cluster %d of level %d%s differ", c, j+1, of)
			}
		}
	}
	return ""
}
