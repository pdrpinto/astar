package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"math/rand"
	"net"
	"net/http"
	"os"
	"strconv"
	"time"

	astar "../.."
)

type point = [2]int

type grid struct {
	W, H  int
	Walls map[point]bool
}

func (g grid) in(p point) bool { return p[0] >= 0 && p[0] < g.W && p[1] >= 0 && p[1] < g.H }

func (g grid) neighbors(p point) []struct {
	ID   point
	Cost float64
} {
	dirs := []point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
	res := make([]struct {
		ID   point
		Cost float64
	}, 0, 4)
	for _, d := range dirs {
		np := point{p[0] + d[0], p[1] + d[1]}
		if g.in(np) && !g.Walls[np] {
			res = append(res, struct {
				ID   point
				Cost float64
			}{ID: np, Cost: 1})
		}
	}
	return res
}

func manhattan(a, b point) float64 {
	dx := a[0] - b[0]
	if dx < 0 {
		dx = -dx
	}
	dy := a[1] - b[1]
	if dy < 0 {
		dy = -dy
	}
	return float64(dx + dy)
}

// clustered random walls via random walks
func genWalls(w, h, clusters, steps int, density float64, start, goal point) map[point]bool {
	r := rand.New(rand.NewSource(time.Now().UnixNano()))
	walls := map[point]bool{}
	for c := 0; c < clusters; c++ {
		p := point{r.Intn(w), r.Intn(h)}
		for s := 0; s < steps; s++ {
			if r.Float64() < density && p != start && p != goal {
				walls[p] = true
			}
			d := []point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}[r.Intn(4)]
			np := point{p[0] + d[0], p[1] + d[1]}
			if np[0] >= 0 && np[0] < w && np[1] >= 0 && np[1] < h {
				p = np
			}
		}
	}
	return walls
}

// --- Root A* stepper integration ---

// adapt grid to astar.Graph
type gridGraph struct{ g grid }

func (gg gridGraph) Neighbors(p point) []astar.Neighbor[point] {
	src := gg.g.neighbors(p)
	out := make([]astar.Neighbor[point], 0, len(src))
	for _, n := range src {
		out = append(out, astar.Neighbor[point]{ID: n.ID, Cost: n.Cost})
	}
	return out
}

type snapshot struct {
	Step    int      `json:"step"`
	W       int      `json:"w"`
	H       int      `json:"h"`
	Walls   [][2]int `json:"walls"`
	Open    [][2]int `json:"open,omitempty"`
	Closed  [][2]int `json:"closed,omitempty"`
	Current [2]int   `json:"current"`
	Start   [2]int   `json:"start"`
	Goal    [2]int   `json:"goal"`
	Done    bool     `json:"done"`
	Found   bool     `json:"found"`
	Path    [][2]int `json:"path,omitempty"`
}

func mapToList(m map[point]bool) [][2]int {
	res := make([][2]int, 0, len(m))
	for p := range m {
		res = append(res, [2]int{p[0], p[1]})
	}
	return res
}
func setToList(m map[point]bool) [][2]int {
	res := make([][2]int, 0, len(m))
	for p, ok := range m {
		if ok {
			res = append(res, [2]int{p[0], p[1]})
		}
	}
	return res
}

// convert a map[point]bool to list
func mapKeysBool(m map[point]bool) [][2]int {
	res := make([][2]int, 0, len(m))
	for p := range m {
		res = append(res, [2]int{p[0], p[1]})
	}
	return res
}

var (
	gState                grid
	startState, goalState point
	stepper               *astar.Stepper[point]
)

func handleInit(w http.ResponseWriter, r *http.Request) {
	q := r.URL.Query()
	wStr, hStr := q.Get("w"), q.Get("h")
	clustersStr, stepsStr, densStr := q.Get("clusters"), q.Get("steps"), q.Get("density")
	wVal, hVal := 40, 24
	clusters, steps := 8, 200
	density := 0.25
	if v, err := strconv.Atoi(wStr); err == nil && v > 4 {
		wVal = v
	}
	if v, err := strconv.Atoi(hStr); err == nil && v > 4 {
		hVal = v
	}
	if v, err := strconv.Atoi(clustersStr); err == nil && v > 0 {
		clusters = v
	}
	if v, err := strconv.Atoi(stepsStr); err == nil && v > 0 {
		steps = v
	}
	if v, err := strconv.ParseFloat(densStr, 64); err == nil && v >= 0 && v <= 1 {
		density = v
	}
	// random start/goal not on walls
	rsrc := rand.New(rand.NewSource(time.Now().UnixNano()))
	start, goal := point{}, point{}
	for {
		start = point{rsrc.Intn(wVal), rsrc.Intn(hVal)}
		goal = point{rsrc.Intn(wVal), rsrc.Intn(hVal)}
		if start != goal {
			break
		}
	}
	g := grid{W: wVal, H: hVal, Walls: genWalls(wVal, hVal, clusters, steps, density, start, goal)}
	// ensure start/goal are not walls
	delete(g.Walls, start)
	delete(g.Walls, goal)
	gState = g
	startState, goalState = start, goal
	// create stepper
	// stop previous stepper if any
	if stepper != nil {
		stepper.Close()
	}
	stepper = astar.NewStepper(context.Background(), gridGraph{g}, start, goal, manhattan, astar.WithWorkers(4))
	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(map[string]any{"ok": true, "w": wVal, "h": hVal})
}

func handleNext(w http.ResponseWriter, r *http.Request) {
	if stepper == nil {
		http.Error(w, "engine not initialized", http.StatusBadRequest)
		return
	}
	st, err := stepper.Step()
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	// convert snapshot
	s := snapshot{
		Step: st.StepIndex,
		W:    gState.W, H: gState.H,
		Walls: mapToList(gState.Walls),
		Start: startState, Goal: goalState,
		Done: st.Done, Found: st.Found,
	}
	s.Current = st.Current
	if st.Open != nil {
		s.Open = mapKeysBool(st.Open)
	}
	if st.Closed != nil {
		s.Closed = setToList(st.Closed)
	}
	if st.Found && len(st.Path) > 0 {
		s.Path = make([][2]int, 0, len(st.Path))
		for _, p := range st.Path {
			s.Path = append(s.Path, p)
		}
	}
	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(s)
}

func handleStatic(w http.ResponseWriter, r *http.Request) {
	if r.URL.Path != "/" {
		http.NotFound(w, r)
		return
	}
	// Try a few likely locations so it works when run from repo root or from the example folder
	candidates := []string{
		"examples/vizweb/static/index.html", // when run from repo root: go run ./examples/vizweb
		"vizweb/static/index.html",          // legacy relative
		"static/index.html",                 // when run from examples/vizweb working dir
	}
	for _, p := range candidates {
		if _, err := os.Stat(p); err == nil {
			http.ServeFile(w, r, p)
			return
		}
	}
	http.Error(w, "index.html not found", http.StatusInternalServerError)
}

func main() {
	mux := http.NewServeMux()
	mux.HandleFunc("/", handleStatic)
	mux.HandleFunc("/init", handleInit)
	mux.HandleFunc("/next", handleNext)
	srv := &http.Server{Handler: mux}
	// Try 8080 first, then fall back to a random free port
	ln, err := net.Listen("tcp", ":8080")
	if err != nil {
		ln, err = net.Listen("tcp", "127.0.0.1:0")
		if err != nil {
			log.Fatal(err)
		}
	}
	addr := ln.Addr().String()
	host := "localhost"
	_, port, _ := net.SplitHostPort(addr)
	if port == "" {
		fmt.Printf("GUI serving on %s\n", addr)
	} else {
		fmt.Printf("GUI: http://%s:%s (rate adjustable)\n", host, port)
	}
	if err := srv.Serve(ln); err != nil {
		log.Fatal(err)
	}
}
