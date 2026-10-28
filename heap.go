package astar

// openHeap is the open set: the node of the least key first, the keys kept apart from the
// nodes so a sift reads keys alone, and each node's position kept by its id for fix. A sift
// carries the node sifted as a hole, writing it once where it lands, and takes the way
// container/heap's takes, so ties break as they always have.
type openHeap struct {
	keys  []float64
	nodes []int32
	pos   []int32
}

// a heap for as many nodes as the solver sizes itself for, the frontier far fewer
func newOpenHeap(nodes int) openHeap {
	frontier := min(nodes, 4096)
	return openHeap{
		keys:  make([]float64, 0, frontier),
		nodes: make([]int32, 0, frontier),
		pos:   make([]int32, 0, nodes),
	}
}

func (h *openHeap) push(n int32, key float64) {
	for int(n) >= len(h.pos) {
		h.pos = append(h.pos, -1)
	}
	j := len(h.keys)
	h.keys, h.nodes = append(h.keys, key), append(h.nodes, n)
	h.up(j, key, n)
}

func (h *openHeap) pop() (int32, bool) {
	last := len(h.keys) - 1
	if last < 0 {
		return -1, false
	}
	n := h.nodes[0]
	key, node := h.keys[last], h.nodes[last]
	h.keys, h.nodes = h.keys[:last], h.nodes[:last]
	if last > 0 {
		h.down(0, last, key, node)
	}
	h.pos[n] = -1
	return n, true
}

// fix moves n, in the heap, to where its new key puts it
func (h *openHeap) fix(n int32, key float64) {
	i := int(h.pos[n])
	if !h.down(i, len(h.keys), key, n) {
		h.up(i, key, n)
	}
}

func (h *openHeap) reset() {
	h.keys, h.nodes = h.keys[:0], h.nodes[:0]
}

// up sifts node, of key, up from the hole at j
func (h *openHeap) up(j int, key float64, node int32) {
	for {
		i := (j - 1) / 2
		if i == j || !(key < h.keys[i]) {
			break
		}
		h.keys[j], h.nodes[j] = h.keys[i], h.nodes[i]
		h.pos[h.nodes[j]] = int32(j)
		j = i
	}
	h.keys[j], h.nodes[j] = key, node
	h.pos[node] = int32(j)
}

// down sifts node, of key, down from the hole at i0 over the first n; true where it moved
func (h *openHeap) down(i0, n int, key float64, node int32) bool {
	i := i0
	for {
		j1 := 2*i + 1
		if j1 >= n {
			break
		}
		j := j1
		if j2 := j1 + 1; j2 < n && h.keys[j2] < h.keys[j1] {
			j = j2
		}
		if !(h.keys[j] < key) {
			break
		}
		h.keys[i], h.nodes[i] = h.keys[j], h.nodes[j]
		h.pos[h.nodes[i]] = int32(i)
		i = j
	}
	h.keys[i], h.nodes[i] = key, node
	h.pos[node] = int32(i)
	return i > i0
}
