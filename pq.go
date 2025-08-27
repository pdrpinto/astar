package astar

type PriorityQueueItem[NodeType comparable] struct {
	Node         NodeType
	GScore       float64
	FCost        float64
	IndexInQueue int
}

type PriorityQueue[NodeType comparable] []*PriorityQueueItem[NodeType]

func (queue PriorityQueue[NodeType]) Len() int           { return len(queue) }
func (queue PriorityQueue[NodeType]) Less(i, j int) bool { return queue[i].FCost < queue[j].FCost }
func (queue PriorityQueue[NodeType]) Swap(i, j int) {
	queue[i], queue[j] = queue[j], queue[i]
	queue[i].IndexInQueue = i
	queue[j].IndexInQueue = j
}

func (queue *PriorityQueue[NodeType]) Push(x any) {
	*queue = append(*queue, x.(*PriorityQueueItem[NodeType]))
}

func (queue *PriorityQueue[NodeType]) Pop() any {
	oldQueue := *queue
	n := len(oldQueue)
	item := oldQueue[n-1]
	*queue = oldQueue[:n-1]
	// TODO: better efficiency for popping elements
	return item
}
