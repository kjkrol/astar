package hpastar

import "time"

// TimeRebuilds has s's Update tell each how long every cluster it builds took, by its level from
// 1, and the labels of level 1, as level 0; nil stops it.
func TimeRebuilds[T comparable](s *Solver[T], each func(level int, took time.Duration)) {
	s.h.TimeBuilds(each)
}

// Rebuilds counts what s's Updates built: by level from 1, the clusters built whole and those
// whose costs alone were filled again; the labels worked out whole and those of the clusters
// changed alone.
type Rebuilds struct {
	Whole, Costs         []uint64
	Labelled, Relabelled uint64
}

// CountRebuilds is what s's Updates built so far.
func CountRebuilds[T comparable](s *Solver[T]) Rebuilds {
	var r Rebuilds
	r.Whole, r.Costs, r.Labelled, r.Relabelled = s.h.Rebuilds()
	return r
}

// Worked is the work s's last Update did, and the work of its last step.
func Worked[T comparable](s *Solver[T]) (update, step int) { return s.h.Worked() }

// GraphsDiffer says where the hierarchies of a and b differ, "" where they do not.
func GraphsDiffer[T comparable](a, b *Solver[T]) string { return a.h.Diff(b.h) }
