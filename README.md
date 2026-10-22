# kjkrol/astar

<p align="center">
  <img src=".github/docs/img/logo.png" alt="kjkrol/astar Logo" width="100">
  <br>
    <a href="https://go.dev">
    <img src="https://img.shields.io/badge/Go-1.23+-00ADD8?style=flat-square&logo=go" alt="Go Version">
  </a>
  <a href="https://pkg.go.dev/github.com/kjkrol/astar">
    <img src="https://img.shields.io/badge/GoDoc-Reference-007d9c?style=flat-square&logo=go" alt="GoDoc">
  </a>
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg?style=flat-square" alt="License">
  </a>
  <a href="https://github.com/kjkrol/astar/actions">
    <img src="https://github.com/kjkrol/astar/actions/workflows/go.yml/badge.svg" alt="Go Quality Check">
  </a>
</p>

**kjkrol/astart** is a **highly performant, fully generic A*** solver for Go. Perfect for optimal pathfinding and abstract state-space search.

## What is A*?

The [A* (A-star) algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) is a graph traversal and state-space search algorithm heavily used in computer science due to its completeness, optimality, and optimal efficiency. It is widely considered the industry standard for finding the shortest path or optimal sequence of transitions between nodes.

While most commonly used as a **Pathfinder**, this library provides a completely domain-agnostic `Solver`. You can use it to solve complex puzzles, optimize network routing, or navigate grids, simply by providing your own `Heuristic` and `Transitions` logic. The Solver remains strictly agnostic of the underlying graph structure or domain-specific cost metrics, making it a universal state-space resolution engine.

# 📦 Installation

GOKe requires **Go 1.23** or newer.

```bash
go get github.com/kjkrol/goke
```

---

# ⏱️ Performance

This solver is built for extreme performance. By utilizing Go 1.18+ Generics, custom memory arenas, and highly optimized data structures, we drastically reduce memory allocations and prevent heap escapes during the hot path of the search.

**Key Optimizations:**
* **`WithIndexedSliceDict`:** Replaces standard Go maps with a pre-allocated slice for tracking visited nodes. It completely eliminates hashing overhead and interface boxing. A byte beside each slot holds the stamp of the search that wrote it, so a new search resets the dictionary by moving the stamp on instead of clearing `maxSize` slots; the stamps are wiped once every 255 searches, or ahead of time, between two, by `Solver.Wipe`.
* **Node Arena:** An internal chunk-based memory arena ensures that millions of nodes can be processed with near-zero allocations after the initial setup.
* **`SolveInto`:** Writes the path into the caller's buffer; a search reusing its buffer allocates nothing.

## Benchmarks (Intel Core i5-8265U, Linux)

The first table represents execution metrics under standard A* operational stress (`CostWeight_1.0`), where the algorithm fully evaluates path costs and alternative routes. Medians of 5 runs of `go test -run '^$' -bench 'Solve/./CostWeight_(1.0|0.1)' -benchmem`.

As the state space expands, the `IndexedSliceDict` provides massive scaling advantages over map-based lookups. For a large **2048x2048** environment, it reduces execution time from **3.00 seconds down to just 1.02 seconds**—outperforming the standard Go map implementation by **2.9x**.

| Grid Size | Dictionary Type | Time (ms/op) | Memory (kB/op) | Allocs (allocs/op) |
| :--- | :--- | :--- | :--- | :--- |
| **64x64** | `IndexedSliceDict` | 0.59 ms | 2.0 kB | 1 |
| | `IndexedMapDict` | 0.99 ms | 2.0 kB | 1 |
| | `DefaultMapDict` | 1.18 ms | 2.0 kB | 1 |
| **256x256** | `IndexedSliceDict` | 10.79 ms | 8.3 kB | 1 |
| | `IndexedMapDict` | 19.01 ms | 8.4 kB | 1 |
| | `DefaultMapDict` | 24.03 ms | 8.4 kB | 1 |
| **512x512** | `IndexedSliceDict` | 43.90 ms | 16.8 kB | 1 |
| | `IndexedMapDict` | 96.32 ms | 17.2 kB | 1 |
| | `DefaultMapDict` | 126.26 ms | 17.4 kB | 2 |
| **1024x1024** | `IndexedSliceDict` | 190.80 ms | 37.7 kB | 3 |
| | `IndexedMapDict` | 459.66 ms | 42.7 kB | 5 |
| | `DefaultMapDict` | 607.95 ms | 47.7 kB | 7 |
| **2048x2048** | `IndexedSliceDict` | **1,016.47 ms** | 138.4 kB | **15** |
| | `IndexedMapDict` | 2,287.90 ms | 138.4 kB | 15 |
| | `DefaultMapDict` | 2,997.18 ms | 138.4 kB | 15 |

Memory and allocations are per search, on a solver sized by `WithInitCapacity`: mostly the returned path, which `SolveInto` with a reused buffer spares. The dictionaries are allocated once, by `New`: `IndexedSliceDict` holds two dictionaries of `maxSize` slots, 9 bytes each: a pointer and a stamp (2048x2048: 72 MiB).

A search with `CostWeight_0.1` (the heuristic dominating) visits few nodes, so on a large grid the reset of `IndexedSliceDict` by a stamp matters most. Each allocates once, for the path.

| Grid Size | `IndexedSliceDict` | `IndexedMapDict` | `DefaultMapDict` |
| :--- | :--- | :--- | :--- |
| **64x64** | 0.04 ms | 0.07 ms | 0.08 ms |
| **256x256** | 0.18 ms | 0.38 ms | 0.45 ms |
| **512x512** | 0.38 ms | 1.65 ms | 1.99 ms |
| **1024x1024** | 0.87 ms | 4.88 ms | 5.84 ms |
| **2048x2048** | **2.06 ms** | 12.36 ms | 14.73 ms |

---

# 🚀 Getting Started (Pathfinding Example)

Here is a step-by-step example of how to configure the solver to find the optimal path on a 2D terrain grid.

### Define your domain state and world grid
The solver is generic, so you define the state representation. For a grid map, a Point struct represents coordinates, and a 2D slice simulates the world terrain.

```go
type Point struct {
	X, Y int
}

const (
	GridSize = 64
	
	// Terrain types and their cost/weights
	TerrainWalkway = 1.0
	TerrainMud     = 3.5
	TerrainWall    = 999.0 // Insurmountable obstacle
)

// Example world grid layout (pre-allocated or loaded from game data)
var grid [GridSize][GridSize]float64
```

Additionally, you need to define a heuristic function (e.g., Manhattan distance) to estimate the remaining distance to the target:

```go
// Heuristic is part of your domain definition
heuristic := func(from, to Point) float64 {
	dx := math.Abs(float64(to.X - from.X))
	dy := math.Abs(float64(to.Y - from.Y))
	return dx + dy
}
```

### Initialize the Solver

Pass your static heuristic into astar.New. To unlock maximum performance on fixed state spaces, use WithIndexedSliceDict by providing an indexer function mapped to your grid dimensions.

**Note:** The solver allocates its internal memory structures once during initialization. This instance is designed to be reused sequentially across multiple distinct pathfinding queries to avoid GC pressure. It is not thread-safe; if you need concurrent pathfinding, use separate solver instances per goroutine or orchestrate them via a pool.
```go
// The Indexer maps a 2D coordinate to a unique 1D slice index
indexer := func(p Point) int { 
	return p.Y * GridSize + p.X 
}
maxNodes := GridSize * GridSize

// Initialize the Solver once with static configuration
solver := astar.New(
	heuristic,
	astar.WithIndexedSliceDict(maxNodes, indexer),
)
```

### Define Rules & Execute Search (Reusing Buffers)

Every time you call Solve(), you pass a transition rule. This allows you to dynamicly change movement logic on the fly without reallocating solver internal buffers, ensuring zero-allocation hot paths.

```go
// Transitions: Populates the pre-allocated buffer with valid moves and terrain costs in a single pass.
dirs := []Point{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}

transitions := func(from, prev Point, buffer []astar.Transition[Point]) []astar.Transition[Point] {
	for _, d := range dirs {
		nx, ny := from.X+d.X, from.Y+d.Y
		
		// 1. Boundary check
		if nx >= 0 && nx < GridSize && ny >= 0 && ny < GridSize {
			
			// 2. Prevent immediate backtracking to the parent node
			if nx == prev.X && ny == prev.Y {
				continue
			}
			
			// 3. Static obstacle pruning (e.g., skip walls entirely)
			terrainCost := grid[ny][nx]
			if terrainCost >= TerrainWall {
				continue
			}
			
			// 4. Register valid transition with its intrinsic edge weight
			buffer = append(buffer, astar.Transition[Point]{
				To:   Point{X: nx, Y: ny},
				Cost: terrainCost, 
			})
		}
	}
	return buffer
}

start := Point{X: 0, Y: 0}
target := Point{X: 63, Y: 63}

// Execute the search by injecting the grid transition rules into the reused solver
path := solver.Solve(start, target, transitions)

if path != nil {
	fmt.Println("Found optimal path with steps:", len(path))
}
```

# License

**kjkrol/astar** is licensed under the MIT License. See the LICENSE [file](./LICENSE) for more details.
