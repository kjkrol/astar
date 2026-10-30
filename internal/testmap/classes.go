package testmap

import "github.com/kjkrol/hpastar/graph"

// Masked is g serving classes classes, a step from from into to refused to the classes whose bits
// refuse(from, to) has, Forward and Backward alike.
func Masked[T comparable](g graph.Implicit[T], classes int,
	refuse func(from, to T) uint8) graph.Implicit[T] {
	forward := g.Forward
	g.Forward = func(from, prev T, buf []graph.Transition[T]) []graph.Transition[T] {
		n := len(buf)
		buf = forward(from, prev, buf)
		for i := n; i < len(buf); i++ {
			buf[i].Refuses = refuse(from, buf[i].To)
		}
		return buf
	}
	if g.Layout != nil {
		l := *g.Layout
		backward := l.Backward
		l.Backward = func(to, next T, buf []graph.Transition[T]) []graph.Transition[T] {
			n := len(buf)
			buf = backward(to, next, buf)
			for i := n; i < len(buf); i++ {
				buf[i].Refuses = refuse(buf[i].To, to)
			}
			return buf
		}
		g.Layout = &l
	}
	g.Classes = classes
	return g
}

// Filtered is g as its class k steps it, a graph of one class: the steps k refuses left out, the
// others refused to none.
func Filtered[T comparable](g graph.Implicit[T], k int) graph.Implicit[T] {
	bit := uint8(1) << k
	taken := func(steps graph.Transitions[T]) graph.Transitions[T] {
		return func(from, prev T, buf []graph.Transition[T]) []graph.Transition[T] {
			n := len(buf)
			buf = steps(from, prev, buf)
			kept := buf[:n]
			for _, st := range buf[n:] {
				if st.Refuses&bit == 0 {
					st.Refuses = 0
					kept = append(kept, st)
				}
			}
			return kept
		}
	}
	g.Forward = taken(g.Forward)
	if g.Layout != nil {
		l := *g.Layout
		l.Backward = taken(l.Backward)
		g.Layout = &l
	}
	g.Classes = 0
	return g
}

// Bars draws from seed, for each state of the map with chance share, the classes barred from
// stepping into it: every class from one drawn up, so that class 0 is never barred and a class
// is barred wherever one below it is. Within, where not nil, keeps the bars to its places.
func (m *Map) Bars(seed uint64, classes int, share float64, within func(x, y int) bool) []uint8 {
	bars := make([]uint8, len(m.Share))
	rnd := Splitmix(seed)
	for s := range bars {
		barred, from := rnd() < share, 1+int(rnd()*float64(classes-1))
		if x, y, _ := m.Coords(s); !barred || within != nil && !within(x, y) {
			continue
		}
		bars[s] = uint8(1<<classes-1) &^ uint8(1<<from-1)
	}
	return bars
}

// Barred is the map's graph serving classes classes, a step into a state refused to the classes
// bars has for it.
func (m *Map) Barred(classes int, bars []uint8) graph.Implicit[int] {
	return Masked(m.Graph(), classes, func(_, to int) uint8 { return bars[to] })
}

// Hex is the map's graph walked as a parallelogram of axial hexagons: six neighbours a state, a
// step priced as the map prices its own, the hex distance its estimate.
func (m *Map) Hex() graph.Implicit[int] {
	dirs := [6][2]int{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, -1}, {-1, 1}}
	steps := func(from int, back bool, buf []graph.Transition[int]) []graph.Transition[int] {
		buf = buf[:0]
		q, r := from%m.W, from/m.W
		if back && m.Share[from] == 0 {
			return buf
		}
		for _, d := range dirs {
			nq, nr := q+d[0], r+d[1]
			if nq < 0 || nr < 0 || nq >= m.W || nr >= m.H || m.Share[nr*m.W+nq] == 0 {
				continue
			}
			n, entered := nr*m.W+nq, nr*m.W+nq
			if back {
				entered = from
			}
			buf = append(buf, graph.Transition[int]{To: n, Cost: 1 / float64(m.Share[entered])})
		}
		return buf
	}
	g := m.Graph()
	g.Forward = func(from, _ int, buf []graph.Transition[int]) []graph.Transition[int] {
		return steps(from, false, buf)
	}
	g.Layout.Backward = func(to, _ int, buf []graph.Transition[int]) []graph.Transition[int] {
		return steps(to, true, buf)
	}
	g.Heuristic = func(a, b int) float64 {
		dq, dr := a%m.W-b%m.W, a/m.W-b/m.W
		return float64(max(dq, -dq, dr, -dr, dq+dr, -dq-dr))
	}
	return g
}

// Layered is the map's graph with a second state at each place of row y from x0 to x1, keyed
// past the map's own: those step along the row both ways at cost 1, and at its ends onto the
// map's states next to them and back.
func (m *Map) Layered(y, x0, x1 int) graph.Implicit[int] {
	above, row := m.W*m.H, y*m.W
	out, in := map[int][]graph.Transition[int]{}, map[int][]graph.Transition[int]{}
	step := func(from, to int) {
		out[from] = append(out[from], graph.Transition[int]{To: to, Cost: 1})
		in[to] = append(in[to], graph.Transition[int]{To: from, Cost: 1})
	}
	for x := x0; x < x1; x++ {
		step(above+row+x, above+row+x+1)
		step(above+row+x+1, above+row+x)
	}
	for _, end := range [2][2]int{{x0, x0 - 1}, {x1, x1 + 1}} {
		if s, ok := m.At(end[1], y); ok && m.Share[s] > 0 {
			step(above+row+end[0], s)
			step(s, above+row+end[0])
		}
	}
	place := func(s int) int {
		if s >= above {
			return s - above
		}
		return s
	}
	g := m.Graph()
	g.Forward = func(from, prev int, buf []graph.Transition[int]) []graph.Transition[int] {
		buf = buf[:0]
		if from < above {
			buf = m.Forward(from, prev, buf)
		}
		return append(buf, out[from]...)
	}
	g.Heuristic = func(a, b int) float64 { return m.Octile(place(a), place(b)) }
	g.Size = 2 * above
	g.Layout = &graph.Layout[int]{
		Coords: func(s int) (int, int, bool) { return m.Coords(place(s)) },
		At: func(x, ay int, buf []int) []int {
			buf = m.States(x, ay, buf)
			if ay == y && x >= x0 && x <= x1 {
				buf = append(buf, above+row+x)
			}
			return buf
		},
		Width:  m.W,
		Height: m.H,
		Backward: func(to, next int, buf []graph.Transition[int]) []graph.Transition[int] {
			buf = buf[:0]
			if to < above {
				buf = m.Backward(to, next, buf)
			}
			return append(buf, in[to]...)
		},
	}
	return g
}

// Hashed is a number in [0, 1) of the step from from into to, drawn from seed, another for the
// step back.
func Hashed(seed uint64, from, to int) float64 {
	z := seed ^ uint64(from)*0x9e3779b97f4a7c15 ^ uint64(to)*0xc2b2ae3d27d4eb4f
	z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9
	z = (z ^ (z >> 27)) * 0x94d049bb133111eb
	return float64(z^(z>>31)) / (1 << 64)
}
