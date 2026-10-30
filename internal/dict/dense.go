package dict

// Dense is a dictionary of states with a slot of their own each, by home, as many slots as Fit
// asks for: the nodes of a level of a hierarchy.
type Dense[T comparable] struct {
	home  func(T) int
	ids   []int32
	marks marks
}

// NewDense is a Dense dictionary placing a state at home, holding no slot until Fit.
func NewDense[T comparable](home func(T) int) *Dense[T] { return &Dense[T]{home: home} }

// Fit has the dictionary hold slots slots, forgetting what it held where it grows.
func (d *Dense[T]) Fit(slots int) {
	if slots > len(d.ids) {
		d.ids, d.marks = make([]int32, slots), newMarks(slots)
	}
}

// Get is the id of s's node; false where the search has none.
func (d *Dense[T]) Get(s T) (int32, bool) {
	i := d.home(s)
	if d.marks.of[i] != d.marks.now {
		return -1, false
	}
	return d.ids[i], true
}

// Set keeps n as the id of s's node.
func (d *Dense[T]) Set(s T, n int32) {
	i := d.home(s)
	d.ids[i], d.marks.of[i] = n, d.marks.now
}

// Clear readies the dictionary for a new search, wiping a part of its marks.
func (d *Dense[T]) Clear() { d.marks.next() }
