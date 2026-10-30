package dict

import (
	"math/rand/v2"
	"testing"
)

func TestIndexedSliceDict_ClearLeavesSlotsBe(t *testing.T) {
	d := NewIndexedSlice(8, func(i int) int { return i }).(*indexedSliceDict[int])
	for i := range 8 {
		d.Set(i, int32(i))
	}
	d.Set(3, ^int32(3)) // closed: the complement, kept as it is
	d.Clear()

	for i := range 8 {
		if n, ok := d.Get(i); ok || n != -1 {
			t.Fatalf("slot %d after clear: got %v, %v; want -1, false", i, n, ok)
		}
		if want := int32(i); (i == 3 && d.ids[i] != ^want) || (i != 3 && d.ids[i] != want) {
			t.Fatalf("clear wrote slot %d; it should only move the mark on", i)
		}
	}

	d.Set(5, 40)
	if n, ok := d.Get(5); !ok || n != 40 {
		t.Fatalf("slot 5 set after clear: got %v, %v; want 40, true", n, ok)
	}
	d.Set(5, ^int32(40))
	if n, ok := d.Get(5); !ok || n != ^int32(40) {
		t.Fatalf("slot 5 closed: got %v, %v; want %d, true", n, ok, ^int32(40))
	}
}

// searches of random slots past many rounds of the marks: a slot reads as written only in the
// search that wrote it, on dictionaries of fewer slots than marks and of many more
func TestIndexedSliceDict_MarksComingRoundFindTheirSlotsWiped(t *testing.T) {
	for _, slots := range []int{4, 255, 1000, 4096} {
		d := NewIndexedSlice(slots, func(i int) int { return i })
		r := rand.New(rand.NewPCG(uint64(slots), 3))
		written := make(map[int]int32)
		for search := range 4 * 255 {
			d.Clear()
			clear(written)
			for range 1 + r.IntN(slots) {
				i := r.IntN(slots)
				written[i] = int32(search)
				d.Set(i, int32(search))
			}
			for i := range slots {
				n, ok := d.Get(i)
				if want, wrote := written[i]; ok != wrote || ok && n != want {
					t.Fatalf("%d slots, search %d: slot %d reads %v, %v; written %v, %v",
						slots, search, i, n, ok, want, wrote)
				}
			}
		}
	}
}

func TestMarks_ASearchWipesAPartOfThem(t *testing.T) {
	m := newMarks(1000)
	for i := range m.of {
		m.of[i] = 7
	}
	m.next()
	wiped := 0
	for _, mark := range m.of {
		if mark == 0 {
			wiped++
		}
	}
	// a 255th part, rounded up
	if wiped != 4 {
		t.Fatalf("a search wiped %d of 1000 slots, want 4", wiped)
	}
}
