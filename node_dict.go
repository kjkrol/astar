package astar

import "math"

// ---------------
// Internal Node Dictionary
// ---------------
type nodeDict[T comparable] interface {
	get(id T) (*node[T], bool)
	set(id T, node *node[T])
	remove(id T)
	clear()
	// wipe wipes now what a clear would, where fewer than within clears are left before one
	// would; true where it did
	wipe(within int) bool
}

// ---------------
// Map-based Node Dictionary
// ---------------
type mapDict[T comparable] struct {
	m map[T]*node[T]
}

func newMapDict[T comparable](capacity int) *mapDict[T] {
	return &mapDict[T]{m: make(map[T]*node[T], capacity)}
}
func (d *mapDict[T]) get(id T) (*node[T], bool) { n, ok := d.m[id]; return n, ok }
func (d *mapDict[T]) set(id T, node *node[T])   { d.m[id] = node }
func (d *mapDict[T]) remove(id T)               { delete(d.m, id) }
func (d *mapDict[T]) clear()                    { clear(d.m) }
func (d *mapDict[T]) wipe(int) bool             { return false }

// ---------------
// Indexed Map-based Node Dictionary
// ---------------
type indexedMapDict[T comparable] struct {
	m       map[int]*node[T]
	indexOf func(T) int
}

func newIndexedMapDict[T comparable](capacity int, indexer func(T) int) *indexedMapDict[T] {
	return &indexedMapDict[T]{
		m:       make(map[int]*node[T], capacity),
		indexOf: indexer,
	}
}

func (d *indexedMapDict[T]) get(id T) (*node[T], bool) { n, ok := d.m[d.indexOf(id)]; return n, ok }
func (d *indexedMapDict[T]) set(id T, node *node[T])   { d.m[d.indexOf(id)] = node }
func (d *indexedMapDict[T]) remove(id T)               { delete(d.m, d.indexOf(id)) }
func (d *indexedMapDict[T]) clear()                    { clear(d.m) }
func (d *indexedMapDict[T]) wipe(int) bool             { return false }

// ---------------
// Slice-based Node Dictionary (for fixed-size, integer-indexable IDs)
// ---------------
type indexedSliceDict[T comparable] struct {
	nodes   []*node[T]
	stamps  []uint8 // by slot: the search that wrote it, 0 none
	stamp   uint8
	indexOf func(T) int
}

func newIndexedSliceDict[T comparable](maxSize int, indexer func(T) int) *indexedSliceDict[T] {
	return &indexedSliceDict[T]{
		nodes:   make([]*node[T], maxSize),
		stamps:  make([]uint8, maxSize),
		stamp:   1,
		indexOf: indexer,
	}
}
func (d *indexedSliceDict[T]) get(id T) (*node[T], bool) {
	i := d.indexOf(id)
	if d.stamps[i] != d.stamp {
		return nil, false
	}
	return d.nodes[i], true
}
func (d *indexedSliceDict[T]) set(id T, node *node[T]) {
	i := d.indexOf(id)
	d.nodes[i], d.stamps[i] = node, d.stamp
}
func (d *indexedSliceDict[T]) remove(id T) { d.stamps[d.indexOf(id)] = 0 }

// clear moves the stamp on, leaving the slots be; only a stamp wrapping to 0 wipes the stamps
func (d *indexedSliceDict[T]) clear() {
	if d.stamp++; d.stamp == 0 {
		clear(d.stamps)
		d.stamp = 1
	}
}

func (d *indexedSliceDict[T]) wipe(within int) bool {
	if math.MaxUint8-int(d.stamp) >= within {
		return false
	}
	clear(d.stamps)
	d.stamp = 1
	return true
}
