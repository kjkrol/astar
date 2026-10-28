package astar

import "math"

// a dictionary from a state to the id of its node, the solver's to read as it will
type nodeDict[T comparable] interface {
	get(id T) (int32, bool)
	set(id T, n int32)
	clear()
	// wipe wipes now what a clear would, where fewer than within clears are left before one
	// would; true where it did
	wipe(within int) bool
}

type mapDict[T comparable] struct {
	m map[T]int32
}

func newMapDict[T comparable](capacity int) *mapDict[T] {
	return &mapDict[T]{m: make(map[T]int32, capacity)}
}
func (d *mapDict[T]) get(id T) (int32, bool) { n, ok := d.m[id]; return n, ok }
func (d *mapDict[T]) set(id T, n int32)      { d.m[id] = n }
func (d *mapDict[T]) clear()                 { clear(d.m) }
func (d *mapDict[T]) wipe(int) bool          { return false }

type indexedMapDict[T comparable] struct {
	m       map[int]int32
	indexOf func(T) int
}

func newIndexedMapDict[T comparable](capacity int, indexer func(T) int) *indexedMapDict[T] {
	return &indexedMapDict[T]{
		m:       make(map[int]int32, capacity),
		indexOf: indexer,
	}
}

func (d *indexedMapDict[T]) get(id T) (int32, bool) { n, ok := d.m[d.indexOf(id)]; return n, ok }
func (d *indexedMapDict[T]) set(id T, n int32)      { d.m[d.indexOf(id)] = n }
func (d *indexedMapDict[T]) clear()                 { clear(d.m) }
func (d *indexedMapDict[T]) wipe(int) bool          { return false }

// a slot a state, by the Indexer: the node's id and the stamp of the search that wrote it
type indexedSliceDict[T comparable] struct {
	ids     []int32
	stamps  []uint8 // by slot: the search that wrote it, 0 none
	stamp   uint8
	indexOf func(T) int
}

func newIndexedSliceDict[T comparable](maxSize int, indexer func(T) int) *indexedSliceDict[T] {
	return &indexedSliceDict[T]{
		ids:     make([]int32, maxSize),
		stamps:  make([]uint8, maxSize),
		stamp:   1,
		indexOf: indexer,
	}
}
func (d *indexedSliceDict[T]) get(id T) (int32, bool) {
	i := d.indexOf(id)
	if d.stamps[i] != d.stamp {
		return -1, false
	}
	return d.ids[i], true
}
func (d *indexedSliceDict[T]) set(id T, n int32) {
	i := d.indexOf(id)
	d.ids[i], d.stamps[i] = n, d.stamp
}

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
