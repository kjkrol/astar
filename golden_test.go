package hpastar_test

import (
	"bytes"
	"flag"
	"fmt"
	"os"
	"slices"
	"testing"

	"github.com/kjkrol/hpastar"
	"github.com/kjkrol/hpastar/graph"
	"github.com/kjkrol/hpastar/internal/astar"
	"github.com/kjkrol/hpastar/internal/testmap"
)

var updatePaths = flag.Bool("update", false, "rewrite testdata/paths.golden")

const pathsGolden = "testdata/paths.golden"

// a scene of the golden: a grid, whether it is walked eight ways, and its queries
type goldenScene struct {
	name    string
	grid    [][]float64
	eight   bool
	queries [][2]testmap.Point
}

func goldenScenes() []goldenScene {
	var scenes []goldenScene
	add := func(name string, grid [][]float64, eight bool, seed uint64) {
		scenes = append(scenes, goldenScene{name, grid, eight, testmap.Queries(grid, seed)})
	}
	random := testmap.RandomGrid(48, 3)
	add("grid32", testmap.WalledGrid(32), false, 1)
	add("grid64", testmap.WalledGrid(64), false, 2)
	add("random48-4", random, false, 4)
	add("random48-8", random, true, 5)
	add("uniform24-4", testmap.UniformGrid(24), false, 6)
	add("uniform24-8", testmap.UniformGrid(24), true, 7)
	return scenes
}

// the scene's steps and the distance that never overestimates them
func (s goldenScene) graph() graph.Implicit[testmap.Point] {
	if s.eight {
		return graph.Implicit[testmap.Point]{Forward: testmap.EightWays(s.grid),
			Heuristic: testmap.Octile}
	}
	return graph.Implicit[testmap.Point]{Forward: testmap.FourWays(1.0, s.grid),
		Heuristic: testmap.Manhattan}
}

// goldenPaths is every path of every scene, a line a query; solvers are the ways of finding a
// scene's paths, by name, each making its finder anew for a scene, and must agree to the step
func goldenPaths(t *testing.T,
	solvers map[string]func(goldenScene) func(from, to testmap.Point) []testmap.Point) []byte {
	t.Helper()
	var buf bytes.Buffer
	for _, s := range goldenScenes() {
		finders := map[string]func(from, to testmap.Point) []testmap.Point{}
		var first string
		for name, solver := range solvers {
			finders[name] = solver(s)
			if first == "" || name < first {
				first = name
			}
		}
		for _, q := range s.queries {
			path := finders[first](q[0], q[1])
			for name, find := range finders {
				if got := find(q[0], q[1]); !slices.Equal(got, path) {
					t.Fatalf("%s %v->%v: %s gave %v, %s %v",
						s.name, q[0], q[1], name, got, first, path)
				}
			}
			fmt.Fprintf(&buf, "%s %d,%d->%d,%d ", s.name, q[0].X, q[0].Y, q[1].X, q[1].Y)
			if len(path) == 0 {
				buf.WriteString("none\n")
				continue
			}
			fmt.Fprintf(&buf, "cost=%.6g [", testmap.PathCost(s.grid, path))
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

// matchesGolden fails t where got differs from the paths pinned, or pins got with -update
func matchesGolden(t *testing.T, got []byte) {
	t.Helper()
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

func TestAStar_PathsAsPinned(t *testing.T) {
	solvers := map[string]func(goldenScene) func(from, to testmap.Point) []testmap.Point{}
	for name := range testmap.Dicts(0) {
		solvers[name] = func(s goldenScene) func(from, to testmap.Point) []testmap.Point {
			g := s.graph()
			a := astar.New(g.Heuristic, testmap.Dicts(len(s.grid))[name](), 0)
			return func(from, to testmap.Point) []testmap.Point {
				return a.SolveInto(nil, from, to, g.Forward)
			}
		}
	}
	matchesGolden(t, goldenPaths(t, solvers))
}

// the paths of a Solver without a Layout are the engine's, to the step, whichever dictionary its
// Graph asks for
func TestSolver_PathWithoutALayoutIsTheEnginesPath(t *testing.T) {
	graphs := map[string]func(g graph.Implicit[testmap.Point],
		size int) graph.Implicit[testmap.Point]{
		"no Index": func(g graph.Implicit[testmap.Point], _ int) graph.Implicit[testmap.Point] {
			return g
		},
		"an Index": func(g graph.Implicit[testmap.Point], size int) graph.Implicit[testmap.Point] {
			g.Index = func(p testmap.Point) int { return p.Y*size + p.X }
			return g
		},
		"an Index and a Size": func(g graph.Implicit[testmap.Point],
			size int) graph.Implicit[testmap.Point] {
			g.Index, g.Size = func(p testmap.Point) int { return p.Y*size + p.X }, size*size
			return g
		},
	}
	solvers := map[string]func(goldenScene) func(from, to testmap.Point) []testmap.Point{}
	for name, indexed := range graphs {
		solvers[name] = func(s goldenScene) func(from, to testmap.Point) []testmap.Point {
			q := hpastar.NewSolver(indexed(s.graph(), len(s.grid))).Search()
			dst := make([]testmap.Point, 4*len(s.grid)*len(s.grid))
			return func(from, to testmap.Point) []testmap.Point {
				n, ok := q.Path(from, to, dst)
				if !ok {
					return nil
				}
				return slices.Clone(dst[:n])
			}
		}
	}
	matchesGolden(t, goldenPaths(t, solvers))
}
