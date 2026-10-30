package dict

// Dict is a dictionary from a state to the id of its node, an engine's to read as it will; Clear
// readies it for a new search.
type Dict[T comparable] interface {
	Get(id T) (int32, bool)
	Set(id T, n int32)
	Clear()
}

// the states a map is first sized for, the nodes an engine is first sized for
const defaultCapacity = 1024

// New is the dictionary an index and a size ask for: a slot a state by the index, a map keyed by
// the index without a size, a map keyed by the states without an index.
func New[T comparable](index func(T) int, size int) Dict[T] {
	switch {
	case index != nil && size > 0:
		return NewIndexedSlice(size, index)
	case index != nil:
		return NewIndexedMap(defaultCapacity, index)
	}
	return NewMap[T](defaultCapacity)
}

type mapDict[T comparable] struct {
	m map[T]int32
}

// NewMap is a dictionary keyed by the states, a map first sized for capacity of them.
func NewMap[T comparable](capacity int) Dict[T] {
	return &mapDict[T]{m: make(map[T]int32, capacity)}
}
func (d *mapDict[T]) Get(id T) (int32, bool) { n, ok := d.m[id]; return n, ok }
func (d *mapDict[T]) Set(id T, n int32)      { d.m[id] = n }
func (d *mapDict[T]) Clear()                 { clear(d.m) }

type indexedMapDict[T comparable] struct {
	m       map[int]int32
	indexOf func(T) int
}

// NewIndexedMap is a dictionary keyed by indexer, a map first sized for capacity states.
func NewIndexedMap[T comparable](capacity int, indexer func(T) int) Dict[T] {
	return &indexedMapDict[T]{
		m:       make(map[int]int32, capacity),
		indexOf: indexer,
	}
}

func (d *indexedMapDict[T]) Get(id T) (int32, bool) { n, ok := d.m[d.indexOf(id)]; return n, ok }
func (d *indexedMapDict[T]) Set(id T, n int32)      { d.m[d.indexOf(id)] = n }
func (d *indexedMapDict[T]) Clear()                 { clear(d.m) }

// a slot a state, by the index: the node's id and the mark of the search that wrote it
type indexedSliceDict[T comparable] struct {
	ids     []int32
	marks   marks
	indexOf func(T) int
}

// NewIndexedSlice is a dictionary of a slot a state by indexer, maxSize slots of 5 bytes; an index
// outside them panics.
func NewIndexedSlice[T comparable](maxSize int, indexer func(T) int) Dict[T] {
	return &indexedSliceDict[T]{
		ids:     make([]int32, maxSize),
		marks:   newMarks(maxSize),
		indexOf: indexer,
	}
}
func (d *indexedSliceDict[T]) Get(id T) (int32, bool) {
	i := d.indexOf(id)
	if d.marks.of[i] != d.marks.now {
		return -1, false
	}
	return d.ids[i], true
}
func (d *indexedSliceDict[T]) Set(id T, n int32) {
	i := d.indexOf(id)
	d.ids[i], d.marks.of[i] = n, d.marks.now
}

// Clear moves the mark on, leaving the slots be.
func (d *indexedSliceDict[T]) Clear() { d.marks.next() }

// marks are a byte a slot: the search that wrote it, 0 none, a slot of another search's mark
// reading as empty. A mark comes round every 255 searches and each search wipes a 255th part of
// the slots as it starts, so every slot is wiped between a mark and its coming round, and no
// search wipes them all.
type marks struct {
	of  []uint8
	now uint8
}

func newMarks(slots int) marks { return marks{of: make([]uint8, slots), now: 1} }

// next moves on to a new search's mark, wiping that mark's part of the slots
func (m *marks) next() {
	if m.now++; m.now == 0 {
		m.now = 1
	}
	part := (len(m.of) + 254) / 255
	lo := min(int(m.now-1)*part, len(m.of))
	clear(m.of[lo:min(lo+part, len(m.of))])
}
