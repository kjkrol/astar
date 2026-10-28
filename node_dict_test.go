package astar

import (
	"math"
	"testing"
)

func TestIndexedSliceDict_ClearLeavesSlotsBe(t *testing.T) {
	d := newIndexedSliceDict(8, func(i int) int { return i })
	for i := range 8 {
		d.set(i, int32(i))
	}
	d.set(3, ^int32(3)) // closed: the complement, kept as it is
	d.clear()

	for i := range 8 {
		if n, ok := d.get(i); ok || n != -1 {
			t.Fatalf("slot %d after clear: got %v, %v; want -1, false", i, n, ok)
		}
		if want := int32(i); (i == 3 && d.ids[i] != ^want) || (i != 3 && d.ids[i] != want) {
			t.Fatalf("clear wrote slot %d; it should only move the stamp on", i)
		}
	}

	d.set(5, 40)
	if n, ok := d.get(5); !ok || n != 40 {
		t.Fatalf("slot 5 set after clear: got %v, %v; want 40, true", n, ok)
	}
	d.set(5, ^int32(40))
	if n, ok := d.get(5); !ok || n != ^int32(40) {
		t.Fatalf("slot 5 closed: got %v, %v; want %d, true", n, ok, ^int32(40))
	}
}

func TestIndexedSliceDict_StampWraps(t *testing.T) {
	d := newIndexedSliceDict(4, func(i int) int { return i })
	d.set(0, 10)
	d.stamp = math.MaxUint8
	d.set(1, 11)
	d.clear()

	if d.stamp != 1 {
		t.Fatalf("stamp after wrapping: got %d, want 1", d.stamp)
	}
	// slot 0 still carried stamp 1: only wiping the stamps keeps it from coming back
	for i := range 2 {
		if n, ok := d.get(i); ok || n != -1 {
			t.Fatalf("slot %d after the stamp wrapped: got %v, %v; want -1, false", i, n, ok)
		}
	}
}

func TestIndexedSliceDict_WipesWithinSoManyClearsOfTheWrap(t *testing.T) {
	d := newIndexedSliceDict(4, func(i int) int { return i })
	d.stamp = math.MaxUint8 - 64
	d.set(0, 10)
	if d.wipe(64) {
		t.Fatal("64 clears left: wiped within 64")
	}
	d.stamp++
	d.set(1, 11)
	if !d.wipe(64) {
		t.Fatal("63 clears left: not wiped within 64")
	}
	if d.stamp != 1 || d.stamps[0] != 0 || d.stamps[1] != 0 {
		t.Fatalf("after the wipe: stamp %d, stamps %v; want 1 and none", d.stamp, d.stamps)
	}
	if _, ok := d.get(1); ok {
		t.Fatal("slot 1 still found after the wipe")
	}
}
