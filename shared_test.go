package astar_test

import (
	"math"

	"github.com/kjkrol/astar"
)

type Point struct {
	X, Y int
}

const InsurmountableObstacle = 100.0

// setupPathFinder creates a deterministic grid of a given size, applies a cost multiplier,
// and returns the initialized solver along with boundaries and transition rules.

func setupGrid(size int) [][]float64 {
	// Allocate grid dynamically based on the requested benchmark/test size
	grid := make([][]float64, size)
	for y := range size {
		grid[y] = make([]float64, size)
		for x := range size {
			val := float64(((x + y) % 5) + 1)

			if x%4 == 2 && y%4 != 2 {
				val = InsurmountableObstacle
			}

			if (x == 0 && y == 0) || (x == size-1 && y == size-1) {
				val = 1
			}

			grid[y][x] = val
		}
	}

	return grid
}

func setupPathFinder(
	opts ...astar.SolverOption[Point],
) *astar.Solver[Point] {

	heuristic := func(p, g Point) float64 {
		return math.Abs(float64(g.X-p.X)) + math.Abs(float64(g.Y-p.Y))
	}

	pathFinder := astar.New(heuristic, opts...)

	return pathFinder
}

// setupOctileFinder is a solver with the octile distance: no step across grid costs less than
// its length, so it never overestimates
func setupOctileFinder(opts ...astar.SolverOption[Point]) *astar.Solver[Point] {
	return astar.New(func(p, g Point) float64 {
		dx, dy := math.Abs(float64(g.X-p.X)), math.Abs(float64(g.Y-p.Y))
		return dx + dy + (math.Sqrt2-2)*min(dx, dy)
	}, opts...)
}

// setupTransitions8 walks grid eight ways: a wall is never entered, a step costs the cell
// entered, a slantwise one √2 of it
func setupTransitions8(grid [][]float64) astar.Transitions[Point] {
	size := len(grid)
	dirs := [8]Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}}
	return func(from, prev Point, buf []astar.Transition[Point]) []astar.Transition[Point] {
		for i, d := range dirs {
			nx, ny := from.X+d.X, from.Y+d.Y
			if nx < 0 || nx >= size || ny < 0 || ny >= size || (nx == prev.X && ny == prev.Y) {
				continue
			}
			cost := grid[ny][nx]
			if cost >= InsurmountableObstacle {
				continue
			}
			if i >= 4 {
				cost *= math.Sqrt2
			}
			buf = append(buf, astar.Transition[Point]{To: Point{X: nx, Y: ny}, Cost: cost})
		}
		return buf
	}
}

func setupTransitions(costWeight float64, grid [][]float64) astar.Transitions[Point] {
	size := len(grid)
	dirs := []Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}
	return func(
		from, prev Point,
		buf []astar.Transition[Point],
	) []astar.Transition[Point] {
		for _, d := range dirs {
			nx, ny := from.X+d.X, from.Y+d.Y
			if nx >= 0 && nx < size && ny >= 0 && ny < size {
				if nx == prev.X && ny == prev.Y {
					continue
				}

				// Apply the dynamic scenario weight to the terrain cost
				cost := grid[ny][nx] * costWeight
				if cost >= InsurmountableObstacle {
					continue
				}

				buf = append(buf, astar.Transition[Point]{
					To:   Point{X: nx, Y: ny},
					Cost: cost,
				})
			}
		}
		return buf
	}
}
