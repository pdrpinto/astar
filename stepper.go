package astar

import (
	"container/heap"
	"context"
	"runtime"
)

// StepSnapshot exposes the per-iteration state of the search
type StepSnapshot[NodeType comparable] struct {
	Current   NodeType
	Open      map[NodeType]bool
	Closed    map[NodeType]bool
	CameFrom  map[NodeType]NodeType
	Done      bool
	Found     bool
	Path      []NodeType
	StepIndex int
}

// Stepper provides a step-by-step orchestrator over the concurrent workers
type Stepper[NodeType comparable] struct {
	ctx       context.Context
	cancel    context.CancelFunc
	graph     Graph[NodeType]
	goal      NodeType
	heuristic Heuristic[NodeType]
	workers   int

	openSet    PriorityQueue[NodeType]
	openSetMap map[NodeType]*PriorityQueueItem[NodeType]
	closedSet  map[NodeType]bool
	cameFrom   map[NodeType]NodeType
	gScore     map[NodeType]float64

	expandCh chan ExpandTask[NodeType]
	relaxCh  chan RelaxProposal[NodeType]

	stepCount int
	done      bool
	found     bool
}

// NewStepper creates a new stepper using the same worker-based expansion logic as Search
func NewStepper[NodeType comparable](
	parent context.Context,
	graph Graph[NodeType],
	startNode NodeType,
	goalNode NodeType,
	heuristic Heuristic[NodeType],
	options ...Option,
) *Stepper[NodeType] {
	// options
	opts := Options{NumberOfWorkers: runtime.NumCPU()}
	for _, o := range options {
		o(&opts)
	}

	ctx, cancel := context.WithCancel(parent)
	s := &Stepper[NodeType]{
		ctx: ctx, cancel: cancel,
		graph: graph, goal: goalNode, heuristic: heuristic,
		workers:    opts.NumberOfWorkers,
		openSet:    make(PriorityQueue[NodeType], 0),
		openSetMap: make(map[NodeType]*PriorityQueueItem[NodeType]),
		closedSet:  make(map[NodeType]bool),
		cameFrom:   make(map[NodeType]NodeType),
		gScore:     map[NodeType]float64{startNode: 0},
		expandCh:   make(chan ExpandTask[NodeType]),
		relaxCh:    make(chan RelaxProposal[NodeType]),
	}

	heap.Init(&s.openSet)
	startItem := &PriorityQueueItem[NodeType]{Node: startNode, GScore: 0, FCost: heuristic(startNode, goalNode)}
	heap.Push(&s.openSet, startItem)
	s.openSetMap[startNode] = startItem

	// start workers
	for i := 0; i < s.workers; i++ {
		go func() {
			for {
				select {
				case <-s.ctx.Done():
					return
				case task := <-s.expandCh:
					tentativeG := task.CurrentGScore + task.Neighbor.Cost
					f := tentativeG + task.HeuristicFunc(task.Neighbor.ID, task.GoalNode)
					s.relaxCh <- RelaxProposal[NodeType]{
						FromNode: task.FromNode,
						ToNode:   task.Neighbor.ID,
						GScore:   tentativeG,
						FCost:    f,
					}
				}
			}
		}()
	}

	return s
}

// Close stops the workers
func (s *Stepper[NodeType]) Close() {
	if s.cancel != nil {
		s.cancel()
	}
}

// Step advances the search by one node expansion and returns a snapshot
func (s *Stepper[NodeType]) Step() (StepSnapshot[NodeType], error) {
	if s.done {
		return StepSnapshot[NodeType]{
			Done:      true,
			Found:     s.found,
			Open:      copyBoolMap(s.openSetToBoolMap()),
			Closed:    copyBoolMap(s.closedSet),
			CameFrom:  copyCameFrom(s.cameFrom),
			Path:      nil,
			StepIndex: s.stepCount,
		}, nil
	}
	if s.openSet.Len() == 0 {
		s.done = true
		return StepSnapshot[NodeType]{
			Done:      true,
			Found:     false,
			Open:      copyBoolMap(s.openSetToBoolMap()),
			Closed:    copyBoolMap(s.closedSet),
			CameFrom:  copyCameFrom(s.cameFrom),
			StepIndex: s.stepCount,
		}, nil
	}

	s.stepCount++
	currentItem := heap.Pop(&s.openSet).(*PriorityQueueItem[NodeType])
	current := currentItem.Node
	delete(s.openSetMap, current)
	if s.closedSet[current] {
		return s.Step()
	}
	s.closedSet[current] = true

	if current == s.goal {
		s.done = true
		s.found = true
		return StepSnapshot[NodeType]{
			Current:   current,
			Open:      copyBoolMap(s.openSetToBoolMap()),
			Closed:    copyBoolMap(s.closedSet),
			CameFrom:  copyCameFrom(s.cameFrom),
			Done:      true,
			Found:     true,
			Path:      reconstructPath(s.cameFrom, current, inferStartFromCameFrom(s.cameFrom, current)),
			StepIndex: s.stepCount,
		}, nil
	}

	neighbors := s.graph.Neighbors(current)
	for _, nb := range neighbors {
		s.expandCh <- ExpandTask[NodeType]{
			FromNode:      current,
			Neighbor:      nb,
			CurrentGScore: currentItem.GScore,
			GoalNode:      s.goal,
			HeuristicFunc: s.heuristic,
		}
	}
	for i := 0; i < len(neighbors); i++ {
		select {
		case <-s.ctx.Done():
			s.done = true
			return StepSnapshot[NodeType]{Done: true, Found: false, StepIndex: s.stepCount}, s.ctx.Err()
		case p := <-s.relaxCh:
			if s.closedSet[p.ToNode] {
				continue
			}
			if gPrev, ok := s.gScore[p.ToNode]; !ok || p.GScore < gPrev {
				s.gScore[p.ToNode] = p.GScore
				s.cameFrom[p.ToNode] = p.FromNode
				if it, ok := s.openSetMap[p.ToNode]; !ok {
					it = &PriorityQueueItem[NodeType]{Node: p.ToNode, GScore: p.GScore, FCost: p.FCost}
					heap.Push(&s.openSet, it)
					s.openSetMap[p.ToNode] = it
				} else if p.FCost < it.FCost {
					it.GScore = p.GScore
					it.FCost = p.FCost
					heap.Fix(&s.openSet, it.IndexInQueue)
				}
			}
		}
	}

	return StepSnapshot[NodeType]{
		Current:   current,
		Open:      copyBoolMap(s.openSetToBoolMap()),
		Closed:    copyBoolMap(s.closedSet),
		CameFrom:  copyCameFrom(s.cameFrom),
		Done:      false,
		Found:     false,
		StepIndex: s.stepCount,
	}, nil
}

func (s *Stepper[NodeType]) openSetToBoolMap() map[NodeType]bool {
	m := make(map[NodeType]bool, len(s.openSetMap))
	for k := range s.openSetMap {
		m[k] = true
	}
	return m
}

func copyBoolMap[T comparable](m map[T]bool) map[T]bool {
	if m == nil {
		return nil
	}
	c := make(map[T]bool, len(m))
	for k, v := range m {
		c[k] = v
	}
	return c
}
func copyCameFrom[T comparable](m map[T]T) map[T]T {
	if m == nil {
		return nil
	}
	c := make(map[T]T, len(m))
	for k, v := range m {
		c[k] = v
	}
	return c
}

// inferStartFromCameFrom tries to find the start by walking backwards until a node with no predecessor
func inferStartFromCameFrom[T comparable](came map[T]T, current T) T {
	cur := current
	for {
		prev, ok := came[cur]
		if !ok {
			return cur
		}
		cur = prev
	}
}
