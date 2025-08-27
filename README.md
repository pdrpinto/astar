# astar

A tiny, generic, and concurrent A* pathfinding library for Go, plus a step-by-step visualizer example.

- Generic API using Go 1.18+ type parameters. Works with any comparable node type.
- Concurrent neighbor expansion with a worker pool.
- Deterministic orchestrator with pluggable heuristics.
- Stepper API for driving UIs, debuggers, or tutorials one iteration at a time.

## Requirements

- Go 1.18 or newer (uses generics)

## Install and use

This repository is a small library and example. You can:

- Vendor/clone it locally and import from your project while using traditional GOPATH mode, or
- Use it directly within this repository (examples below), or
- Adapt the code into your own module if you prefer a canonical import path.

Because the example imports the library via a relative path (`astar "../.."`), it is easiest to run the example from inside this repository with modules disabled.

## Quick start (library)

Minimal example that searches a 2D grid with 4-neighborhood movement using Manhattan distance.

```go
package main

import (
    "context"
    "fmt"

    astar "path/to/your/local/astar" // if inside this repo, you can also build from ./examples
)

type point = [2]int

type grid struct {
    W, H  int
    Walls map[point]bool
}

func (g grid) in(p point) bool { return p[0] >= 0 && p[0] < g.W && p[1] >= 0 && p[1] < g.H }

// Satisfy astar.Graph[point]
func (g grid) Neighbors(p point) []astar.Neighbor[point] {
    dirs := []point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
    out := make([]astar.Neighbor[point], 0, 4)
    for _, d := range dirs {
        q := point{p[0] + d[0], p[1] + d[1]}
        if g.in(q) && !g.Walls[q] {
            out = append(out, astar.Neighbor[point]{ID: q, Cost: 1})
        }
    }
    return out
}

func manhattan(a, b point) float64 {
    dx := a[0] - b[0]
    if dx < 0 { dx = -dx }
    dy := a[1] - b[1]
    if dy < 0 { dy = -dy }
    return float64(dx + dy)
}

func main() {
    g := grid{W: 5, H: 5, Walls: map[point]bool{{1,1}: true, {1,2}: true}}
    start, goal := point{0, 0}, point{4, 4}

    res, err := astar.Search(context.Background(), g, start, goal, manhattan, astar.WithWorkers(4))
    if err != nil {
        fmt.Println("no path:", err)
        return
    }
    fmt.Println("found:", res.Found, "cost:", res.TotalCost, "expanded:", res.ExpandedNodes)
    fmt.Println("path:", res.Path)
}
```

## Step-by-step visualization (Stepper)

If you want to drive a UI or debugger one expansion at a time, use `Stepper`:

```go
s := astar.NewStepper(ctx, g, start, goal, manhattan, astar.WithWorkers(4))
for {
    snap, err := s.Step()
    if err != nil { /* handle ctx cancel, etc. */ }
    // snap exposes: Current, Open, Closed, CameFrom, Done, Found, Path, StepIndex
    if snap.Done { break }
}
s.Close()
```

The included `examples/vizweb` demonstrates this and renders each iteration in the browser.

## Run the web visualizer example (Windows PowerShell)

- From the repository root, run:

```powershell
go run ./examples/vizweb
```

- The server tries port 8080 first. If it's taken, it falls back to a random free port and prints the URL. Look for a line like:

```
GUI: http://localhost:8080 (rate adjustable)
```

- Open the printed URL in your browser. You should see the grid, tweak parameters, and play/pause the search.

Troubleshooting:
- If you saw a 404 before, it was likely due to the working directory; the example now looks for `index.html` in a few common locations so it works from the repo root or the example folder.

## API overview

- `type Graph[N comparable] interface { Neighbors(node N) []Neighbor[N] }`
  - Your graph type implements this method to return reachable neighbors and their costs.
- `type Neighbor[N comparable] struct { ID N; Cost float64 }`
- `type Heuristic[N comparable] func(from N, to N) float64`
  - For admissible A*, ensure the heuristic never overestimates the true cost.
- `func Search[N comparable](ctx context.Context, g Graph[N], start, goal N, h Heuristic[N], opts ...Option) (Result[N], error)`
- `type Result[N comparable] struct { Path []N; TotalCost float64; ExpandedNodes int; Found bool }`
- `type Option = func(*Options)`; `func WithWorkers(n int) Option`
- `type Stepper[N comparable]` with `NewStepper`, `(*Stepper).Step()`, and `(*Stepper).Close()`.

## Concurrency model

- A single orchestrator goroutine pops the next best node from a priority queue.
- A worker pool computes tentative relaxations for neighbors in parallel.
- Proposals flow back to the orchestrator which updates the open set and g-scores.

This keeps correctness with a clear owner of the frontier while still parallelizing expensive neighbor evaluations.

## Notes and next steps

- The example uses relative imports to keep things simple within this repo. If you publish as a module, update imports and add a `go.mod` with your module path.
- Embedding the static files with Go's `embed` package would remove working-directory concerns a potential future improvement.

## License

Add your preferred license here.
