package astar

// ExpandTask represents a request from the orchestrator to the workers.
type ExpandTask[NodeType comparable] struct {
	FromNode      NodeType
	Neighbor      Neighbor[NodeType]
	CurrentGScore float64
	GoalNode      NodeType
	HeuristicFunc Heuristic[NodeType]
}

// RelaxProposal is the worker's suggestion for updating a path
type RelaxProposal[NodeType comparable] struct {
	FromNode NodeType
	ToNode   NodeType
	GScore   float64
	FCost    float64
}
