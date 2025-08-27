package astar

import (
	"container/heap"
	"context"
	"errors"
	"runtime"
)

// Graph is generic over node type N.
// N must be comparable so it can be used in maps.

type Graph[NodeType comparable] interface {
	Neighbors(node NodeType) []Neighbor[NodeType]
}

// Neighbor represents a reachable node with a cost.
type Neighbor[NodeType comparable] struct {
	ID   NodeType
	Cost float64
}

// Heuristic returns the estimated cost from node a to node b
type Heuristic[NodeType comparable] func(from NodeType, to NodeType) float64

// Result contains the outcome of a search
type Result[NodeType comparable] struct {
	Path          []NodeType
	TotalCost     float64
	ExpandedNodes int
	Found         bool
}

// Options defines parameters for the search.
type Options struct {
	NumberOfWorkers int
}

// Option is a function that modifies Options.
type Option func(*Options)

// WithWorkers specifies how many worker goroutines should expand neighbors.
func WithWorkers(numberOfWorkers int) Option {
	return func(options *Options) { options.NumberOfWorkers = numberOfWorkers }
}

// Search executes the concurrent A* search algorithm.
func Search[NodeType comparable](
	contextObject context.Context,
	graph Graph[NodeType],
	startNode NodeType,
	goalNode NodeType,
	heuristic Heuristic[NodeType],
	options ...Option,
) (Result[NodeType], error) {

	// --- Apply options ---
	searchOptions := Options{
		NumberOfWorkers: runtime.NumCPU(),
	}
	for _, option := range options {
		option(&searchOptions)
	}

	// --- Initialize state ---
	openSet := make(PriorityQueue[NodeType], 0)
	heap.Init(&openSet)

	startItem := &PriorityQueueItem[NodeType]{
		Node:   startNode,
		GScore: 0.0,
		FCost:  heuristic(startNode, goalNode),
	}
	heap.Push(&openSet, startItem)

	cameFrom := make(map[NodeType]NodeType)
	pathCostFromStart := map[NodeType]float64{startNode: 0.0}
	closedSet := make(map[NodeType]bool)
	openSetMap := make(map[NodeType]*PriorityQueueItem[NodeType])
	openSetMap[startNode] = startItem

	// Channels for communication
	expandTaskChannel := make(chan ExpandTask[NodeType])
	relaxProposalChannel := make(chan RelaxProposal[NodeType])

	// --- Start worker pool ---
	for i := 0; i < searchOptions.NumberOfWorkers; i++ {
		go func() {
			for {
				select {
				case <-contextObject.Done():
					return
				case task := <-expandTaskChannel:
					tentativeG := task.CurrentGScore + task.Neighbor.Cost
					f := tentativeG + task.HeuristicFunc(task.Neighbor.ID, task.GoalNode)
					proposal := RelaxProposal[NodeType]{
						FromNode: task.FromNode,
						ToNode:   task.Neighbor.ID,
						GScore:   tentativeG,
						FCost:    f,
					}
					relaxProposalChannel <- proposal
				}
			}
		}()
	}

	// --- Orchestrator loop ---
	expandedNodes := 0
	for {
		if openSet.Len() == 0 {
			return Result[NodeType]{
				Path:          nil,
				TotalCost:     0,
				ExpandedNodes: expandedNodes,
				Found:         false,
			}, errors.New("no path found")
		}

		currentItem := heap.Pop(&openSet).(*PriorityQueueItem[NodeType])
		currentNode := currentItem.Node
		delete(openSetMap, currentNode)

		// Skip if already closed
		if closedSet[currentNode] {
			continue
		}
		closedSet[currentNode] = true
		expandedNodes++

		// Goal check
		if currentNode == goalNode {
			return Result[NodeType]{
				Path:          reconstructPath(cameFrom, currentNode, startNode),
				TotalCost:     currentItem.GScore,
				ExpandedNodes: expandedNodes,
				Found:         true,
			}, nil
		}

		// Send tasks to workers for each neighbor
		neighbors := graph.Neighbors(currentNode)
		for _, neighbor := range neighbors {
			task := ExpandTask[NodeType]{
				FromNode:      currentNode,
				Neighbor:      neighbor,
				CurrentGScore: currentItem.GScore,
				GoalNode:      goalNode,
				HeuristicFunc: heuristic,
			}
			expandTaskChannel <- task
		}

		// Collect proposals for all neighbors of current node
		for i := 0; i < len(neighbors); i++ {
			select {
			case <-contextObject.Done():
				return Result[NodeType]{}, contextObject.Err()
			case proposal := <-relaxProposalChannel:
				if closedSet[proposal.ToNode] {
					continue
				}
				currentG, exists := pathCostFromStart[proposal.ToNode]
				if !exists || proposal.GScore < currentG {
					pathCostFromStart[proposal.ToNode] = proposal.GScore
					cameFrom[proposal.ToNode] = proposal.FromNode
					if item, inOpen := openSetMap[proposal.ToNode]; !inOpen {
						item = &PriorityQueueItem[NodeType]{
							Node:   proposal.ToNode,
							GScore: proposal.GScore,
							FCost:  proposal.FCost,
						}
						heap.Push(&openSet, item)
						openSetMap[proposal.ToNode] = item
					} else if proposal.FCost < item.FCost {
						item.GScore = proposal.GScore
						item.FCost = proposal.FCost
						heap.Fix(&openSet, item.IndexInQueue)
					}
				}
			}
		}
	}
}

// reconstructPath is internal to the orchestrator.
func reconstructPath[NodeType comparable](
	cameFrom map[NodeType]NodeType,
	current NodeType,
	start NodeType,
) []NodeType {
	path := []NodeType{current}
	for current != start {
		previousNode, exists := cameFrom[current]
		if !exists {
			break
		}
		path = append(path, previousNode)
		current = previousNode
	}
	// reverse path
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return path
}
