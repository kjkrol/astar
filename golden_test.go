package astar_test

import (
	"bytes"
	"flag"
	"fmt"
	"math"
	"math/rand/v2"
	"os"
	"slices"
	"testing"

	"github.com/kjkrol/astar"
)

var updatePaths = flag.Bool("update", false, "rewrite testdata/paths.golden")

const pathsGolden = "testdata/paths.golden"

// a scene of the golden: a grid, whether it is walked eight ways, and its queries
type goldenScene struct {
	name    string
	grid    [][]float64
	eight   bool
	queries [][2]Point
}

func goldenScenes() []goldenScene {
	var scenes []goldenScene
	add := func(name string, grid [][]float64, eight bool, seed uint64) {
		scenes = append(scenes, goldenScene{name, grid, eight, goldenQueries(grid, seed)})
	}
	random := randomGrid(48, 3)
	add("grid32", setupGrid(32), false, 1)
	add("grid64", setupGrid(64), false, 2)
	add("random48-4", random, false, 4)
	add("random48-8", random, true, 5)
	add("uniform24-4", uniformGrid(24), false, 6)
	add("uniform24-8", uniformGrid(24), true, 7)
	return scenes
}

// the corners both ways, a cell to itself, the first wall if any, then forty pairs of open cells
func goldenQueries(grid [][]float64, seed uint64) [][2]Point {
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

// costs of 1 to 5 and a wall on a fifth of the cells, the corners kept open
func randomGrid(size int, seed uint64) [][]float64 {
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

func uniformGrid(size int) [][]float64 {
	grid := make([][]float64, size)
	for y := range size {
		grid[y] = make([]float64, size)
		for x := range size {
			grid[y][x] = 1
		}
	}
	return grid
}

// the cost of path over grid as the scene's transitions price it: the cell entered, a
// slantwise step √2 of it
func pathCost(grid [][]float64, path []Point) float64 {
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

// every path of every scene, a line a query, equal through all three dictionaries
func goldenPaths(t *testing.T) []byte {
	t.Helper()
	var buf bytes.Buffer
	for _, s := range goldenScenes() {
		size := len(s.grid)
		indexer := func(p Point) int { return p.Y*size + p.X }
		var transitions astar.Transitions[Point]
		var finder func(...astar.SolverOption[Point]) *astar.Solver[Point]
		if s.eight {
			transitions, finder = setupTransitions8(s.grid), setupOctileFinder
		} else {
			transitions, finder = setupTransitions(1.0, s.grid), setupPathFinder
		}
		solvers := map[string]*astar.Solver[Point]{
			"DefaultDict":      finder(),
			"IndexedMapDict":   finder(astar.WithIndexedMapDict(indexer)),
			"IndexedSliceDict": finder(astar.WithIndexedSliceDict(size*size, indexer)),
		}
		for _, q := range s.queries {
			path := solvers["IndexedSliceDict"].Solve(q[0], q[1], transitions)
			for name, solver := range solvers {
				if got := solver.Solve(q[0], q[1], transitions); !slices.Equal(got, path) {
					t.Fatalf("%s %v->%v: %s gave %v, the slice dictionary %v",
						s.name, q[0], q[1], name, got, path)
				}
			}
			fmt.Fprintf(&buf, "%s %d,%d->%d,%d ", s.name, q[0].X, q[0].Y, q[1].X, q[1].Y)
			if path == nil {
				buf.WriteString("none\n")
				continue
			}
			fmt.Fprintf(&buf, "cost=%.6g [", pathCost(s.grid, path))
			for i, p := range path {
				if i > 0 {
					buf.WriteByte(' ')
				}
				fmt.Fprintf(&buf, "%d,%d", p.X, p.Y)
			}
			buf.WriteString("]\n")
		}
	}
	return buf.Bytes()
}

func TestSolver_PathsAsPinned(t *testing.T) {
	got := goldenPaths(t)
	if *updatePaths {
		if err := os.MkdirAll("testdata", 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(pathsGolden, got, 0o644); err != nil {
			t.Fatal(err)
		}
		return
	}
	want, err := os.ReadFile(pathsGolden)
	if err != nil {
		t.Fatalf("%v; run with -update to pin the paths", err)
	}
	if bytes.Equal(got, want) {
		return
	}
	gotLines, wantLines := bytes.Split(got, []byte("\n")), bytes.Split(want, []byte("\n"))
	for i := range max(len(gotLines), len(wantLines)) {
		var g, w []byte
		if i < len(gotLines) {
			g = gotLines[i]
		}
		if i < len(wantLines) {
			w = wantLines[i]
		}
		if !bytes.Equal(g, w) {
			t.Fatalf("line %d differs from %s:\n got %s\nwant %s\n"+
				"a change meant to keep the paths keeps it; one meant to change them runs "+
				"with -update and says why", i+1, pathsGolden, g, w)
		}
	}
}
