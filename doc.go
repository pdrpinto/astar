// Package astar provides a generic and concurrent A* pathfinding implementation.
//
// It exposes two main entry points:
//
//   - Search: run the algorithm to completion and get a Result.
//   - Stepper: iterate the search one expansion at a time to drive UIs or debugging tools.
//
// The library is generic over node type and uses a worker pool to parallelize
// neighbor expansion while keeping a single orchestrator that owns the frontier.
package astar
