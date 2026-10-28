package astar

import (
	"container/heap"
	"math/rand/v2"
	"testing"
)

// the open set as it was: nodes on pointers under container/heap, the reference the hand-written
// heap is held to, ties included
type refNode struct {
	id    int32
	key   float64
	index int
}

type refQueue []*refNode

func (q refQueue) Len() int           { return len(q) }
func (q refQueue) Less(i, j int) bool { return q[i].key < q[j].key }
func (q refQueue) Swap(i, j int) {
	q[i].index, q[j].index = j, i
	q[i], q[j] = q[j], q[i]
}
func (q *refQueue) Push(x any) {
	n := x.(*refNode)
	n.index = len(*q)
	*q = append(*q, n)
}
func (q *refQueue) Pop() any {
	old := *q
	n := old[len(old)-1]
	n.index = -1
	*q = old[:len(old)-1]
	return n
}

func TestHeap_PopsAsContainerHeap(t *testing.T) {
	for seed := uint64(1); seed <= 20; seed++ {
		r := rand.New(rand.NewPCG(seed, 99))
		h := newOpenHeap(8)
		ref := &refQueue{}
		var inRef []*refNode
		var next int32
		for op := range 10000 {
			switch choice := r.Uint64() % 10; {
			case choice < 5 || len(inRef) == 0:
				key := float64(r.Uint64() % 4)
				n := &refNode{id: next, key: key}
				heap.Push(ref, n)
				h.push(next, key)
				inRef = append(inRef, n)
				next++
			case choice < 7:
				n := inRef[r.Uint64()%uint64(len(inRef))]
				n.key = float64(r.Uint64() % 4)
				heap.Fix(ref, n.index)
				h.fix(n.id, n.key)
			default:
				want := heap.Pop(ref).(*refNode)
				got, ok := h.pop()
				if !ok || got != want.id {
					t.Fatalf("seed %d op %d: popped %d (%v), the reference %d", seed, op, got, ok,
						want.id)
				}
				for i, n := range inRef {
					if n == want {
						inRef = append(inRef[:i], inRef[i+1:]...)
						break
					}
				}
			}
		}
		for ref.Len() > 0 {
			want := heap.Pop(ref).(*refNode)
			if got, ok := h.pop(); !ok || got != want.id {
				t.Fatalf("seed %d draining: popped %d (%v), the reference %d",
					seed, got, ok, want.id)
			}
		}
		if _, ok := h.pop(); ok {
			t.Fatalf("seed %d: the heap popped past the reference's end", seed)
		}
	}
}
