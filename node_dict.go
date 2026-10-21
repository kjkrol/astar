package astar

// ---------------
// Internal Node Dictionary
// ---------------
type nodeDict[T comparable] interface {
	get(id T) (*node[T], bool)
	set(id T, node *node[T])
	remove(id T)
	clear()
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

// ---------------
// Slice-based Node Dictionary (for fixed-size, integer-indexable IDs)
// ---------------
type indexedSliceDict[T comparable] struct {
	slots   []slot[T]
	stamp   uint32
	indexOf func(T) int
}

// a slot holds its node only while its stamp is the dictionary's
type slot[T comparable] struct {
	node  *node[T]
	stamp uint32
}

func newIndexedSliceDict[T comparable](maxSize int, indexer func(T) int) *indexedSliceDict[T] {
	return &indexedSliceDict[T]{
		slots:   make([]slot[T], maxSize),
		stamp:   1,
		indexOf: indexer,
	}
}
func (d *indexedSliceDict[T]) get(id T) (*node[T], bool) {
	s := d.slots[d.indexOf(id)]
	if s.stamp != d.stamp {
		return nil, false
	}
	return s.node, true
}
func (d *indexedSliceDict[T]) set(id T, node *node[T]) {
	d.slots[d.indexOf(id)] = slot[T]{node: node, stamp: d.stamp}
}
func (d *indexedSliceDict[T]) remove(id T) { d.slots[d.indexOf(id)] = slot[T]{} }

// clear moves the stamp on, leaving the slots be; only a stamp wrapping to 0 wipes them
func (d *indexedSliceDict[T]) clear() {
	if d.stamp++; d.stamp == 0 {
		clear(d.slots)
		d.stamp = 1
	}
}
