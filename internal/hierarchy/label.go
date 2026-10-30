package hierarchy

import (
	"cmp"
	"slices"
)

// labels name the components of level 1's graph, its steps taken either way: ends whose nodes'
// names differ are joined by no path. A cluster's nodes keep their names while its connectivity
// holds; the clusters whose connectivity changed are named again off the names across their
// borders, and the whole is worked out anew only where a component may have come apart.
type labels struct {
	// by name: the name it goes by now, the components joined since it was given
	alias []int32
	// how many names the last labelling whole gave
	given int
	// the labels are to be worked out whole
	whole bool
	// the clusters of level 1 whose connectivity changed since the labels were made, each as it
	// was then, and by cluster its place among them, -1 for none
	changed []snapshot
	at      []int32
	// the labellings whole, and those of the changed clusters alone
	made, local uint64
	// a labelling whole: by node across level 1, its parent in the union, then its name
	root []int32
	// the changed clusters' labelling: by changed cluster where its vertices start, now and as
	// it was; its pieces now; the nodes across its borders; the unions now and then; by vertex
	// as it was the root now, and by root now its name; one cluster's pieces, now and as it was
	from, then     []int32
	pieces         []int32
	ends           []nodeRef
	now, was       []int32
	bound, named   []int32
	cluster, older []int32
}

// snapshot is a cluster of level 1's connectivity as it was: its nodes' keys, the piece of the
// cluster each lay in, and where its twins led, by node
type snapshot struct {
	c      int32
	keys   []int
	piece  []int32
	twinAt []int32
	other  []int32
	across []int
}

// the most clusters named again alone, past which the labels are worked out whole
const relabelled = 64

// the work a step of a labelling whole does at least
const labelChunk = 1 << 12

// pending says the labels are not worked out for level 1 as it is
func (l *labels) pending() bool { return l.whole || len(l.changed) > 0 }

// forget lets go of the clusters noted changed
func (l *labels) forget() {
	for _, e := range l.changed {
		l.at[e.c] = -1
	}
	l.changed = l.changed[:0]
}

// join has the components of names a and b go by one name, and reports the work done
func (l *labels) join(a, b int32) int {
	x, y := l.alias[a], l.alias[b]
	if x == y {
		return 1
	}
	won, lost := min(x, y), max(x, y)
	for i, n := range l.alias {
		if n == lost {
			l.alias[i] = won
		}
	}
	return 1 + len(l.alias)
}

// note follows cluster c of level 1, just built by the job, with the labels: where its nodes,
// the nodes their twins lead to or the pieces they lie in changed, the cluster as it was is kept
// for it to be named again; it reports the work done
func (h *Hierarchy[T]) note(c int32) int {
	l, jb := &h.labels, &h.job
	if l.whole {
		return 1
	}
	cl := &h.levels[0].clusters[c]
	n, m := len(cl.keys), len(jb.keys)
	work := 1 + n + m + len(cl.twins) + len(jb.twins)
	l.pieces = piecesOf(l.pieces, cl.class, cl.intra, n)
	l.older = piecesOf(l.older, jb.class, jb.intra, m)
	if slices.Equal(jb.keys, cl.keys) && slices.Equal(l.older, l.pieces) &&
		slices.Equal(jb.twinAt, cl.twinAt) && slices.EqualFunc(jb.twins, cl.twins, sameEnd) {
		return work
	}
	if l.at[c] >= 0 {
		return work
	}
	if len(l.changed) == relabelled {
		l.forget()
		l.whole = true
		return work
	}
	if len(l.changed) < cap(l.changed) {
		l.changed = l.changed[:len(l.changed)+1]
	} else {
		l.changed = append(l.changed, snapshot{})
	}
	e := &l.changed[len(l.changed)-1]
	e.c = c
	e.keys = append(e.keys[:0], jb.keys...)
	e.piece = append(e.piece[:0], l.older...)
	e.twinAt = append(e.twinAt[:0], jb.twinAt...)
	e.other, e.across = e.other[:0], e.across[:0]
	for _, tw := range jb.twins {
		e.other, e.across = append(e.other, tw.other), append(e.across, tw.key)
	}
	l.at[c] = int32(len(l.changed) - 1)
	return work
}

func sameEnd(a, b twin) bool { return a.other == b.other && a.key == b.key }

// piecesOf lists into dst, by node of a cluster of n nodes of classes class and costs intra, the
// least node of its piece: the nodes it reaches or is reached from, joined
func piecesOf(dst, class []int32, intra []float32, n int) []int32 {
	dst = grown(dst, n)
	for i := range dst {
		dst[i] = int32(i)
	}
	joinPieces(dst, 0, class, intra, n)
	for i := range dst {
		dst[i] = rootOf(dst, int32(i))
	}
	return dst
}

// joinPieces joins in root, its nodes from base on, the nodes of a cluster of n nodes, of
// classes class and costs intra, that one reaches from another; it reports the checks it made
func joinPieces(root []int32, base int32, class []int32, intra []float32, n int) (checks int) {
	for i := range n {
		if k := class[i]; k != int32(i) {
			joinRoots(root, base+int32(i), base+k)
		}
	}
	// a node reaches every node of its class and is reached from them, so the classes one
	// reaches from another join them all
	for i := range n {
		if class[i] != int32(i) {
			continue
		}
		for k := i + 1; k < n; k++ {
			checks++
			if class[k] == int32(k) && (intra[i*n+k] != inf32 || intra[k*n+i] != inf32) {
				joinRoots(root, base+int32(i), base+int32(k))
			}
		}
	}
	return checks
}

// beginLabels starts working the labels out: the changed clusters alone, unless the whole is
// due or too many names were given since the last labelling whole
func (h *Hierarchy[T]) beginLabels() {
	l, jb := &h.labels, &h.job
	jb.phase, jb.i, jb.took = relabelling, 0, 0
	if len(l.alias) > 2*l.given+1024 {
		l.whole = true
	}
	if l.whole {
		h.beginWhole()
	}
}

// beginWhole starts a labelling whole: each node its own root, then the joins, then the names,
// a chunk of clusters a step
func (h *Hierarchy[T]) beginWhole() {
	l, jb := &h.labels, &h.job
	l.forget()
	l.whole = true
	lv := h.based(0)
	l.root = grown(l.root, int(lv.base[len(lv.clusters)]))
	jb.phase, jb.i = rooting, 0
}

// labelStep does the next step of the labels' job, and reports its work and whether it is done
func (h *Hierarchy[T]) labelStep() (work int, done bool) {
	l, jb := &h.labels, &h.job
	lv := &h.levels[0]
	clusters := int32(len(lv.clusters))
	switch jb.phase {
	case relabelling:
		work, done = h.relabel()
		if done {
			l.forget()
			l.local++
			return work, true
		}
		h.beginWhole()
	case rooting:
		for ; jb.i < clusters && work < labelChunk; jb.i++ {
			for i := lv.base[jb.i]; i < lv.base[jb.i+1]; i++ {
				l.root[i] = i
			}
			work += 1 + int(lv.base[jb.i+1]-lv.base[jb.i])
		}
		if jb.i == clusters {
			jb.phase, jb.i = joining, 0
		}
	case joining:
		for ; jb.i < clusters && work < labelChunk; jb.i++ {
			work += h.joinLabels(jb.i)
		}
		if jb.i == clusters {
			jb.phase, jb.i = naming, 0
			l.alias = l.alias[:0]
		}
	case naming:
		for ; jb.i < clusters && work < labelChunk; jb.i++ {
			work += h.nameLabels(jb.i)
		}
		if jb.i == clusters {
			l.given, l.whole = len(l.alias), false
			l.made++
			return work, true
		}
	}
	return work, false
}

// joinLabels joins in the labelling whole the nodes of cluster c of level 1 that one reaches
// from another within it, and each with the nodes its twins lead to; it reports the work done
func (h *Hierarchy[T]) joinLabels(c int32) int {
	l, lv := &h.labels, &h.levels[0]
	cl := &lv.clusters[c]
	base, n := lv.base[c], len(cl.keys)
	work := 1 + n + joinPieces(l.root, base, cl.class, cl.intra, n)
	for i := range n {
		for _, tw := range cl.twins[cl.twinAt[i]:cl.twinAt[i+1]] {
			if v, ok := h.nodeIndex(0, tw.other, tw.key); ok {
				joinRoots(l.root, base+int32(i), lv.base[tw.other]+v)
			}
		}
	}
	return work + 2*len(cl.twins)
}

// nameLabels names the nodes of cluster c of level 1 in the labelling whole: the least node of a
// component, its root, a name of its own, each other its root's. The nodes are named in order,
// and a node's parent in the union is a node before it, named already: root keeps a name as its
// complement.
func (h *Hierarchy[T]) nameLabels(c int32) int {
	l, lv := &h.labels, &h.levels[0]
	cl := &lv.clusters[c]
	base := lv.base[c]
	cl.label = grown(cl.label, len(cl.keys))
	for i := range cl.keys {
		g := base + int32(i)
		if r := l.root[g]; r == g {
			name := int32(len(l.alias))
			l.alias = append(l.alias, name)
			l.root[g] = ^name
		} else {
			l.root[g] = l.root[r]
		}
		cl.label[i] = ^l.root[g]
	}
	return 1 + len(cl.keys)
}

// relabel names the nodes of the clusters changed since the labels were made, off the names of
// the nodes across their borders (their ends), joining the components they now join; false
// where a component they held may have come apart, two ends they joined as they were held apart
// now, which only a labelling whole tells. It reports the work done.
func (h *Hierarchy[T]) relabel() (work int, ok bool) {
	l, lv := &h.labels, &h.levels[0]
	k := len(l.changed)
	l.from, l.then = grown(l.from, k+1), grown(l.then, k+1)
	l.from[0], l.then[0] = 0, 0
	l.pieces, l.ends = l.pieces[:0], l.ends[:0]
	for x := range l.changed {
		e := &l.changed[x]
		cl := &lv.clusters[e.c]
		n := len(cl.keys)
		l.cluster = piecesOf(l.cluster, cl.class, cl.intra, n)
		l.pieces = append(l.pieces, l.cluster...)
		l.from[x+1], l.then[x+1] = l.from[x]+int32(n), l.then[x]+int32(len(e.keys))
		for _, tw := range cl.twins {
			h.end(tw.other, tw.key)
		}
		for t, o := range e.other {
			h.end(o, e.across[t])
		}
		work += 1 + n + len(e.keys) + len(cl.twins) + len(e.other)
	}
	slices.SortFunc(l.ends, byRef)
	l.ends = slices.Compact(l.ends)
	now, was, ends := l.from[k], l.then[k], int32(len(l.ends))
	l.now, l.was = roots(l.now, now+ends), roots(l.was, was+ends)
	for x := range l.changed {
		e := &l.changed[x]
		cl := &lv.clusters[e.c]
		for i := range cl.keys {
			u := l.from[x] + l.pieces[l.from[x]+int32(i)]
			for _, tw := range cl.twins[cl.twinAt[i]:cl.twinAt[i+1]] {
				v, found := h.nodeIndex(0, tw.other, tw.key)
				if y := l.at[tw.other]; found && y >= 0 {
					joinRoots(l.now, u, l.from[y]+l.pieces[l.from[y]+v])
				} else if found {
					joinRoots(l.now, u, now+l.endOf(tw.other, v))
				}
			}
		}
		for i := range e.keys {
			u := l.then[x] + e.piece[i]
			for t := e.twinAt[i]; t < e.twinAt[i+1]; t++ {
				o, key := e.other[t], e.across[t]
				if y := l.at[o]; y >= 0 {
					f := &l.changed[y]
					if v, found := slices.BinarySearch(f.keys, key); found {
						joinRoots(l.was, u, l.then[y]+f.piece[v])
					}
				} else if v, found := h.nodeIndex(0, o, key); found {
					joinRoots(l.was, u, was+l.endOf(o, v))
				}
			}
		}
	}
	l.bound = filled(l.bound, int(was+ends))
	for e := range ends {
		then, held := rootOf(l.was, was+e), rootOf(l.now, now+e)
		if b := l.bound[then]; b < 0 {
			l.bound[then] = held
		} else if b != held {
			return work + int(ends), false
		}
	}
	l.named = filled(l.named, int(now+ends))
	for e, end := range l.ends {
		r := rootOf(l.now, now+int32(e))
		name := lv.clusters[end.c].label[end.i]
		if l.named[r] < 0 {
			l.named[r] = name
		} else {
			work += l.join(l.named[r], name)
		}
	}
	for x := range l.changed {
		cl := &lv.clusters[l.changed[x].c]
		cl.label = grown(cl.label, len(cl.keys))
		for i := range cl.keys {
			r := rootOf(l.now, l.from[x]+l.pieces[l.from[x]+int32(i)])
			if l.named[r] < 0 {
				l.named[r] = int32(len(l.alias))
				l.alias = append(l.alias, l.named[r])
			}
			cl.label[i] = l.named[r]
		}
	}
	return work + 2*int(ends), true
}

// end lists the node of key in cluster o of level 1 among the ends, where o did not change
func (h *Hierarchy[T]) end(o int32, key int) {
	l := &h.labels
	if l.at[o] >= 0 {
		return
	}
	if v, ok := h.nodeIndex(0, o, key); ok {
		l.ends = append(l.ends, nodeRef{o, v})
	}
}

// endOf is the place among the ends of node v of cluster o
func (l *labels) endOf(o, v int32) int32 {
	e, _ := slices.BinarySearchFunc(l.ends, nodeRef{o, v}, byRef)
	return int32(e)
}

func byRef(a, b nodeRef) int { return cmp.Or(cmp.Compare(a.c, b.c), cmp.Compare(a.i, b.i)) }

// roots is s holding n, each its own root
func roots(s []int32, n int32) []int32 {
	s = grown(s, int(n))
	for i := range s {
		s[i] = int32(i)
	}
	return s
}

// filled is s holding n, each -1
func filled(s []int32, n int) []int32 {
	s = grown(s, n)
	for i := range s {
		s[i] = -1
	}
	return s
}
