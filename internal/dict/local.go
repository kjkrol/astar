package dict

// localDict keeps the nodes of the states at places home numbers: a slot a place, and slots
// chained past the places for the states sharing one
type localDict[T comparable] struct {
	home   func(T) int
	keys   []T
	ids    []int32
	next   []int32
	stamps []uint8
	stamp  uint8
	places int
	spill  int
}

// NewLocal is a dictionary of the states at the places home numbers from 0 to places−1, a slot a
// place and more chained past them for the states sharing one: the states of a cluster.
func NewLocal[T comparable](places int, home func(T) int) Dict[T] {
	return &localDict[T]{home: home, stamp: 1, places: places, keys: make([]T, places),
		ids: make([]int32, places), next: make([]int32, places), stamps: make([]uint8, places)}
}

func (d *localDict[T]) Get(s T) (int32, bool) {
	i := d.home(s)
	if d.stamps[i] != d.stamp {
		return -1, false
	}
	for d.keys[i] != s {
		if i = int(d.next[i]); i < 0 {
			return -1, false
		}
	}
	return d.ids[i], true
}

func (d *localDict[T]) Set(s T, n int32) {
	i := d.home(s)
	if d.stamps[i] != d.stamp {
		d.keys[i], d.ids[i], d.next[i], d.stamps[i] = s, n, -1, d.stamp
		return
	}
	for d.keys[i] != s {
		if d.next[i] < 0 {
			k := d.places + d.spill
			d.spill++
			if k == len(d.keys) {
				var none T
				d.keys, d.ids = append(d.keys, none), append(d.ids, 0)
				d.next, d.stamps = append(d.next, 0), append(d.stamps, 0)
			}
			d.keys[k], d.next[k], d.stamps[k] = s, -1, d.stamp
			d.next[i] = int32(k)
		}
		i = int(d.next[i])
	}
	d.ids[i] = n
}

func (d *localDict[T]) Clear() {
	d.spill = 0
	if d.stamp++; d.stamp == 0 {
		clear(d.stamps)
		d.stamp = 1
	}
}
