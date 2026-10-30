package testmap

import (
	"math"
	"math/rand/v2"

	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/dict"
)

// Point is a cell of a grid of costs.
type Point struct {
	X, Y int
}

// InsurmountableObstacle is the cost of a wall of a grid, a cell no step enters.
const InsurmountableObstacle = 100.0

// WalledGrid is a grid of costs 1 to 5 by the cells' diagonals, walled on most of every fourth
// column, its corners open.
func WalledGrid(size int) [][]float64 {
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

// RandomGrid is a grid of costs of 1 to 5 and a wall on a fifth of the cells, the corners kept
// open.
func RandomGrid(size int, seed uint64) [][]float64 {
	r := rand.New(rand.NewPCG(seed, seed*7+1))
	grid := make([][]float64, size)
	for y := range size {
		grid[y] = make([]float64, size)
		for x := range size {
			grid[y][x] = float64(1 + r.Uint64()%5)
			if r.Uint64()%100 < 20 {
				grid[y][x] = InsurmountableObstacle
			}
		}
	}
	grid[0][0], grid[size-1][size-1] = 1, 1
	return grid
}

// UniformGrid is a grid of cells of cost 1.
func UniformGrid(size int) [][]float64 {
	grid := make([][]float64, size)
	for y := range size {
		grid[y] = make([]float64, size)
		for x := range size {
			grid[y][x] = 1
		}
	}
	return grid
}

// Manhattan is the Manhattan distance.
func Manhattan(p, g Point) float64 {
	return math.Abs(float64(g.X-p.X)) + math.Abs(float64(g.Y-p.Y))
}

// Octile is the octile distance, which never overestimates on a grid where no step costs less
// than its length.
func Octile(p, g Point) float64 {
	dx, dy := math.Abs(float64(g.X-p.X)), math.Abs(float64(g.Y-p.Y))
	return dx + dy + (math.Sqrt2-2)*min(dx, dy)
}

// Dicts are the dictionaries an engine over a grid of size × size may keep its nodes in, by
// name, each made anew.
func Dicts(size int) map[string]func() dict.Dict[Point] {
	indexer := func(p Point) int { return p.Y*size + p.X }
	return map[string]func() dict.Dict[Point]{
		"DefaultDict": func() dict.Dict[Point] { return dict.NewMap[Point](1024) },
		"IndexedMapDict": func() dict.Dict[Point] {
			return dict.NewIndexedMap(1024, indexer)
		},
		"IndexedSliceDict": func() dict.Dict[Point] {
			return dict.NewIndexedSlice(size*size, indexer)
		},
	}
}

// EightWays walks grid eight ways: a wall is never entered, a step costs the cell entered, a
// slantwise one √2 of it.
func EightWays(grid [][]float64) graph.Transitions[Point] {
	size := len(grid)
	dirs := [8]Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}}
	return func(from, prev Point, buf []graph.Transition[Point]) []graph.Transition[Point] {
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
			buf = append(buf, graph.Transition[Point]{To: Point{X: nx, Y: ny}, Cost: cost})
		}
		return buf
	}
}

// FourWays walks grid four ways, a step costing the cell entered times costWeight; a step of 100
// or more is not taken.
func FourWays(costWeight float64, grid [][]float64) graph.Transitions[Point] {
	size := len(grid)
	dirs := []Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}
	return func(from, prev Point, buf []graph.Transition[Point]) []graph.Transition[Point] {
		for _, d := range dirs {
			nx, ny := from.X+d.X, from.Y+d.Y
			if nx >= 0 && nx < size && ny >= 0 && ny < size {
				if nx == prev.X && ny == prev.Y {
					continue
				}
				cost := grid[ny][nx] * costWeight
				if cost >= InsurmountableObstacle {
					continue
				}
				buf = append(buf, graph.Transition[Point]{To: Point{X: nx, Y: ny}, Cost: cost})
			}
		}
		return buf
	}
}

// Queries are the corners of grid both ways, a cell to itself, the first wall if any, then forty
// pairs of open cells drawn from seed.
func Queries(grid [][]float64, seed uint64) [][2]Point {
	size := len(grid)
	last := Point{X: size - 1, Y: size - 1}
	queries := [][2]Point{{{}, last}, {last, {}}, {{X: 3, Y: 3}, {X: 3, Y: 3}}}
	var open []Point
	wall, walled := Point{}, false
	for y := range size {
		for x := range size {
			if grid[y][x] >= InsurmountableObstacle {
				if !walled {
					wall, walled = Point{X: x, Y: y}, true
				}
				continue
			}
			open = append(open, Point{X: x, Y: y})
		}
	}
	if walled {
		queries = append(queries, [2]Point{{}, wall})
	}
	r := rand.New(rand.NewPCG(seed, seed*7+1))
	for range 40 {
		from := open[r.Uint64()%uint64(len(open))]
		to := open[r.Uint64()%uint64(len(open))]
		queries = append(queries, [2]Point{from, to})
	}
	return queries
}

// PathCost is the cost of path over grid as FourWays and EightWays price it: the cell entered, a
// slantwise step √2 of it.
func PathCost(grid [][]float64, path []Point) float64 {
	cost := 0.0
	for i := 1; i < len(path); i++ {
		step := grid[path[i].Y][path[i].X]
		if path[i].X != path[i-1].X && path[i].Y != path[i-1].Y {
			step *= math.Sqrt2
		}
		cost += step
	}
	return cost
}
