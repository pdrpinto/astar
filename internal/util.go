package internal

// ReconstructPath rebuilds the path from the cameFrom map.
func ReconstructPath[NodeType comparable](
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

		// TODO: better optimization for appending stuff
		path = append(path, previousNode)
		current = previousNode
	}
	// reverse path
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}

	return path
}
