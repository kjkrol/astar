package astar

import (
	"math"
	"testing"
)

func TestIndexedSliceDict_ClearLeavesSlotsBe(t *testing.T) {
	d := newIndexedSliceDict[int](8, func(i int) int { return i })
	nodes := make([]node[int], 8)
	for i := range nodes {
		d.set(i, &nodes[i])
	}
	d.remove(3)
	d.clear()

	for i := range nodes {
		if n, ok := d.get(i); ok || n != nil {
			t.Fatalf("slot %d after clear: got %v, %v; want nil, false", i, n, ok)
		}
		if i != 3 && d.nodes[i] != &nodes[i] {
			t.Fatalf("clear wrote slot %d; it should only move the stamp on", i)
		}
	}

	d.set(5, &nodes[0])
	if n, ok := d.get(5); !ok || n != &nodes[0] {
		t.Fatalf("slot 5 set after clear: got %v, %v; want node 0, true", n, ok)
	}
	d.remove(5)
	if _, ok := d.get(5); ok {
		t.Fatal("slot 5 removed: still found")
	}
}

func TestIndexedSliceDict_StampWraps(t *testing.T) {
	d := newIndexedSliceDict[int](4, func(i int) int { return i })
	var early, late node[int]
	d.set(0, &early)
	d.stamp = math.MaxUint8
	d.set(1, &late)
	d.clear()

	if d.stamp != 1 {
		t.Fatalf("stamp after wrapping: got %d, want 1", d.stamp)
	}
	// slot 0 still carried stamp 1: only wiping the stamps keeps it from coming back
	for i := range 2 {
		if n, ok := d.get(i); ok || n != nil {
			t.Fatalf("slot %d after the stamp wrapped: got %v, %v; want nil, false", i, n, ok)
		}
	}
}
