package astar_test

import (
	"fmt"
	"math"

	"github.com/kjkrol/astar"
)

// a cell's share of full speed: 1 open ground, 0.3 mud, 0 a wall
var shares = [][]float64{
	{1, 1, 1},
	{1, 0.3, 1},
	{1, 0, 1},
}

func ExampleSolver_Solve() {
	width, height := len(shares[0]), len(shares)
	// the Manhattan distance never exceeds the cost left: no share exceeds 1, so no step costs
	// less than its length
	manhattan := func(a, b Point) float64 {
		return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y))
	}
	solver := astar.New(manhattan,
		astar.WithIndexedSliceDict(width*height, func(p Point) int { return p.Y*width + p.X }),
		astar.WithSuccessorCapacity[Point](4))

	dirs := [4]Point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
	steps := func(from, prev Point, buf []astar.Transition[Point]) []astar.Transition[Point] {
		for _, d := range dirs {
			to := Point{X: from.X + d.X, Y: from.Y + d.Y}
			if to.X < 0 || to.Y < 0 || to.X >= width || to.Y >= height || to == prev {
				continue
			}
			share := shares[to.Y][to.X]
			if share == 0 { // a wall: a step not generated
				continue
			}
			buf = append(buf, astar.Transition[Point]{To: to, Cost: 1 / share})
		}
		return buf
	}

	fmt.Println(solver.Solve(Point{X: 0, Y: 2}, Point{X: 2, Y: 2}, steps))
	// Output: [{0 2} {0 1} {0 0} {1 0} {2 0} {2 1} {2 2}]
}
