package dijkstra

import (
	"fmt"
	"testing"

	"github.com/kjkrol/hpastar/internal/dict"
	"github.com/kjkrol/hpastar/internal/testmap"
)

// fills eight ways over Solver_Octile's grid: octile settles all of it from a corner; bounded
// fills each 16x16 cluster from its corner to its other three, a hierarchy's build, under the
// grid's dictionary, and local the same under a cluster's
func BenchmarkDijkstra_Fill(b *testing.B) {
	for _, size := range []int{64, 256, 512, 1024} {
		b.Run(fmt.Sprintf("octile/Size_%dx%d", size, size), func(b *testing.B) {
			transitions := testmap.EightWays(testmap.WalledGrid(size))
			indexer := func(p testmap.Point) int { return p.Y*size + p.X }
			d := New(dict.NewIndexedSlice(size*size, indexer), size*size)
			sources := []Source[testmap.Point]{{}}
			b.ReportAllocs()
			for b.Loop() {
				d.Fill(sources, transitions, nil, nil)
			}
		})
	}
	const side = 16
	for _, size := range []int{256, 1024} {
		for _, rung := range []string{"bounded", "local"} {
			b.Run(fmt.Sprintf("%s/Size_%dx%d", rung, size, size), func(b *testing.B) {
				transitions := testmap.EightWays(testmap.WalledGrid(size))
				slots := dict.NewIndexedSlice(size*size, func(p testmap.Point) int {
					return p.Y*size + p.X
				})
				if rung == "local" {
					// a fill asks of no state out of its cluster, so a cluster's slots serve
					slots = dict.NewIndexedSlice(side*side, func(p testmap.Point) int {
						return p.Y%side*side + p.X%side
					})
				}
				d := New(slots, side*side)
				var x0, y0 int
				in := func(p testmap.Point) bool {
					return p.X >= x0 && p.X < x0+side && p.Y >= y0 && p.Y < y0+side
				}
				sources := make([]Source[testmap.Point], 1)
				targets := make([]testmap.Point, 3)
				clusters := func() (settled int) {
					for y0 = 0; y0 < size; y0 += side {
						for x0 = 0; x0 < size; x0 += side {
							sources[0].At = testmap.Point{X: x0, Y: y0}
							targets[0] = testmap.Point{X: x0 + side - 1, Y: y0}
							targets[1] = testmap.Point{X: x0, Y: y0 + side - 1}
							targets[2] = testmap.Point{X: x0 + side - 1, Y: y0 + side - 1}
							settled += d.Fill(sources, transitions, in, targets)
						}
					}
					return settled
				}
				if got, want := clusters(), 3*(size/side)*(size/side); got != want {
					b.Fatalf("%d corners settled, want %d", got, want)
				}
				b.ReportAllocs()
				for b.Loop() {
					clusters()
				}
			})
		}
	}
}
