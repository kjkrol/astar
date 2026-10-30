package hpastar_test

import (
	"fmt"
	"math"
	"sync"

	"github.com/kjkrol/hpastar"
	"github.com/kjkrol/hpastar/graph"
)

type point struct{ X, Y int }

// a cell's share of full speed: 1 open ground, 0.3 mud, 0 a wall
var shares = [][]float64{
	{1, 1, 1},
	{1, 0.3, 1},
	{1, 0, 1},
}

func ExampleSearch_Path() {
	width, height := len(shares[0]), len(shares)
	// the Manhattan distance never exceeds the cost left: no share exceeds 1, so no step costs
	// less than its length
	manhattan := func(a, b point) float64 {
		return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y))
	}
	dirs := [4]point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
	steps := func(from, prev point, buf []graph.Transition[point]) []graph.Transition[point] {
		for _, d := range dirs {
			to := point{X: from.X + d.X, Y: from.Y + d.Y}
			if to.X < 0 || to.Y < 0 || to.X >= width || to.Y >= height || to == prev {
				continue
			}
			share := shares[to.Y][to.X]
			if share == 0 { // a wall: a step not generated
				continue
			}
			buf = append(buf, graph.Transition[point]{To: to, Cost: 1 / share})
		}
		return buf
	}
	solver := hpastar.NewSolver(graph.Implicit[point]{
		Forward:   steps,
		Heuristic: manhattan,
		Index:     func(p point) int { return p.Y*width + p.X },
		Size:      width * height,
	})

	search := solver.Search()
	path := make([]point, width*height)
	n, found := search.Path(point{X: 0, Y: 2}, point{X: 2, Y: 2}, path)
	fmt.Println(found, path[:n])
	// Output: true [{0 2} {0 1} {0 0} {1 0} {2 0} {2 1} {2 2}]
}

func Example_layout() {
	const size = 48
	// open ground, a wall down the middle open at the bottom, mud past it
	wall := make(map[point]bool)
	for y := range 40 {
		wall[point{X: size / 2, Y: y}] = true
	}
	share := func(p point) float64 {
		switch {
		case p.X < 0 || p.Y < 0 || p.X >= size || p.Y >= size || wall[p]:
			return 0
		case p.X > 30 && p.Y > 30:
			return 0.3
		}
		return 1
	}
	dirs := [8]point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}
	length := func(i int) float64 { return 1 + (math.Sqrt2-1)*float64(i/4) }
	// a step costs its length over the share of the cell it enters
	forward := func(from, _ point, buf []graph.Transition[point]) []graph.Transition[point] {
		for i, d := range dirs {
			if to := (point{X: from.X + d.X, Y: from.Y + d.Y}); share(to) > 0 {
				buf = append(buf, graph.Transition[point]{To: to, Cost: length(i) / share(to)})
			}
		}
		return buf
	}
	// the same steps seen from where they end
	backward := func(to, _ point, buf []graph.Transition[point]) []graph.Transition[point] {
		for i, d := range dirs {
			if from := (point{X: to.X - d.X, Y: to.Y - d.Y}); share(from) > 0 && share(to) > 0 {
				buf = append(buf, graph.Transition[point]{To: from, Cost: length(i) / share(to)})
			}
		}
		return buf
	}
	octile := func(a, b point) float64 {
		dx, dy := math.Abs(float64(a.X-b.X)), math.Abs(float64(a.Y-b.Y))
		return max(dx, dy) + (math.Sqrt2-1)*min(dx, dy)
	}
	g := graph.Implicit[point]{
		Forward:   forward,
		Heuristic: octile,
		Index:     func(p point) int { return p.Y*size + p.X },
		Size:      size * size,
	}
	exact := hpastar.NewSolver(g).Search()
	g.Layout = &graph.Layout[point]{
		Coords: func(p point) (int, int, bool) { return p.X, p.Y, true },
		At: func(x, y int, buf []point) []point {
			if p := (point{X: x, Y: y}); share(p) > 0 {
				buf = append(buf, p)
			}
			return buf
		},
		Width:    size,
		Height:   size,
		Backward: backward,
	}
	solver := hpastar.NewSolver(g, hpastar.WithClusterSide(8))
	solver.Update(0)
	search := solver.Search()
	cost := func(path []point) (sum float64) {
		for i := 1; i < len(path); i++ {
			step := 1.0
			if path[i].X != path[i-1].X && path[i].Y != path[i-1].Y {
				step = math.Sqrt2
			}
			sum += step / share(path[i])
		}
		return sum
	}

	// the path asked a piece at a time, each from where the last ended: climbed while the goal
	// is far, the last piece near it found by A*
	start, goal := point{X: 2, Y: 2}, point{X: 45, Y: 3}
	route, piece := []point{start}, make([]point, 24)
	for from := start; from != goal; from = route[len(route)-1] {
		n, _ := search.Path(from, goal, piece)
		route = append(route, piece[1:n]...)
	}
	cheapest := make([]point, size*size)
	n, _ := exact.Path(start, goal, cheapest)
	st := search.Stats()
	fmt.Printf("%d states at %.2f, the cheapest at %.2f; %d pieces climbed, %d by A*\n",
		len(route), cost(route), cost(cheapest[:n]), st.Climbs, st.Exact)

	// a gap opened in the wall near the top: the graph changed there and is rebuilt
	for y := 2; y <= 4; y++ {
		delete(wall, point{X: size / 2, Y: y})
		solver.Changed(point{X: size / 2, Y: y})
	}
	solver.Update(0)
	whole := make([]point, size*size)
	n, _ = search.Path(start, goal, whole)
	k, _ := exact.Path(start, goal, cheapest)
	fmt.Printf("through the gap: %d states at %.2f, the cheapest at %.2f\n", n, cost(whole[:n]),
		cost(cheapest[:k]))
	fmt.Println("reaches the goal:", search.Reaches(start, goal))
	// Output:
	// 76 states at 92.81, the cheapest at 92.81; 3 pieces climbed, 1 by A*
	// through the gap: 44 states at 44.24, the cheapest at 43.41
	// reaches the goal: true
}

func ExampleSolver_Search() {
	const size = 32
	door := point{X: size / 2, Y: 4}
	wall := make(map[point]bool)
	for y := range 24 {
		if y != door.Y {
			wall[point{X: size / 2, Y: y}] = true
		}
	}
	open := func(p point) bool {
		return p.X >= 0 && p.Y >= 0 && p.X < size && p.Y < size && !wall[p]
	}
	dirs := [4]point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
	steps := func(at, _ point, buf []graph.Transition[point]) []graph.Transition[point] {
		for _, d := range dirs {
			if to := (point{X: at.X + d.X, Y: at.Y + d.Y}); open(to) {
				buf = append(buf, graph.Transition[point]{To: to, Cost: 1})
			}
		}
		return buf
	}
	solver := hpastar.NewSolver(graph.Implicit[point]{
		Forward: steps,
		Heuristic: func(a, b point) float64 {
			return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y))
		},
		Index: func(p point) int { return p.Y*size + p.X },
		Size:  size * size,
		Layout: &graph.Layout[point]{
			Coords: func(p point) (int, int, bool) { return p.X, p.Y, true },
			At: func(x, y int, buf []point) []point {
				if p := (point{X: x, Y: y}); open(p) {
					buf = append(buf, p)
				}
				return buf
			},
			Width:    size,
			Height:   size,
			Backward: steps,
		},
	}, hpastar.WithClusterSide(8))
	solver.Update(0)

	pairs := []struct{ from, to point }{
		{point{X: 2, Y: 2}, point{X: 29, Y: 2}},
		{point{X: 4, Y: 12}, point{X: 27, Y: 8}},
		{point{X: 10, Y: 1}, point{X: 22, Y: 16}},
		{point{X: 3, Y: 28}, point{X: 28, Y: 30}},
	}
	lengths := make([]int, len(pairs))
	queries := make(chan int)
	var done sync.WaitGroup
	for range 3 {
		// each worker asks through a Search of its own, made once and reused for every query
		search, path := solver.Search(), make([]point, size*size)
		go func() {
			for i := range queries {
				lengths[i], _ = search.Path(pairs[i].from, pairs[i].to, path)
				done.Done()
			}
		}()
	}
	ask := func() {
		done.Add(len(pairs))
		for i := range pairs {
			queries <- i
		}
		done.Wait()
	}

	ask()
	fmt.Println("through the door:", lengths)
	// the graph changes, and Changed and Update run, between batches, never during one
	wall[door] = true
	solver.Changed(door)
	solver.Update(0)
	ask()
	fmt.Println("round the wall:", lengths)
	close(queries)
	// Output:
	// through the door: [32 36 28 28]
	// round the wall: [72 52 44 28]
}
