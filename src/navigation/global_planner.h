#ifndef __SRC_NAVIGATION_GLOBAL_PLANNER__
#define __SRC_NAVIGATION_GLOBAL_PLANNER__

#include "graph.h"
#include <cmath>

// The graph needs to be a global variable. This may not be the best place for it, but it's here temporarily.
vector<Node*> graph;

// Comparator object used in the priority_queue
// Note that if we want n1 to appear before n2, the function must return false
struct CompareNode{
    bool operator()(Node* n1, Node* n2){
        return (n1->cost + n1->heuristic > n2->cost + n2->heuristic);
    }
};

// Calculates the euclidean distance between two nodes.
float distance(Node* n1, Node* n2) {
    float dx = n1->x - n2->x;
    float dy = n1->y - n2->y;
    return sqrt((dx * dx) + (dy * dy));
}

// Prepares all nodes to run A*.
void a_star_prep(Node* goal, std::map<std::string, Node*> Nodes) {
    for (auto it = Nodes.begin(); it != Nodes.end(); it++) {
        // Add heuristic costs to every node in the graph
        // The heuristic is inversely proportional to the distance from the goal.
        it->second->heuristic = 1 / distance(it->second, goal);
        // Reset the nodes, as well
       // When going through all the nodes, clear their "path" vectors
        it->second->path.clear();
        // And set all their "visited" markers to be false
        it->second->cost = -1;
    }
}

// Uses the A* algorithm to find the shortest path between two nodes,
// Using a heuristic weight for each node inversely proportional to the distance from the goal.
    std::priority_queue<Node*>, vector<Node*>, CompareNode> PQ;
    a_star_prep(goal, Nodes);

    //Add start to the priorityqueue
    PQ.push(start);

    //While queue isn't empty
    while (!PQ.empty()) {
        Node *currentNode = PQ.top();
        // If first node in queue is the goal, we've found the best path!
        if (currentNode == goal) {
            return goal.path;
        }

        // If the first node isn't "visited"
        if (!currentNode->marked) {
            // Note that it is possible for the node to be "visited", 
            // as there can be duplicates in the PQ if multiple paths get to the same node
            currentNode->marked = true;
            for (String n : currentNode->neighbors) {
                Node* neighbor = Nodes[n];
                if (!neighbor->marked) {
                    if (neighbor->cost == -1 || ->cost > currentNode->cost + 1) {
                        // If they aren't in the PQ, or if we've just found a better path, add them to the PQ
                        // Don't worry about duplicates, we will only use each node once 
                        // (when we've found the optimal path to it)
                        neighbor->cost = currentNode->cost + 1;
                        neighbor->path_ids = currentNode->path_ids;
                        neighbor->path_ids.push(currentNode->id)
                        PQ.push(neighbor);
                    }
                }
            }
        }
        // Pop the currentNode to remove it from the queue
        PQ.pop();
    }
    // If we get to this part of the code, the queue is empty. This means there is no path to the goal.
}

// Uses the A* algorithm to find the shortest path between two nodes,
// Using a heuristic weight for each node inversely proportional to the distance from the goal.
vector<Node*> a_star_local_vars(Node* start, Node* goal, std::map<std::string, Node*> &Nodes, int x_dim, int y_dim) {

    std::vector<float> costs = std::vector<float>(x_dim * y_dim, 0.0);
    std::vector<std::vector<String>> paths = std::vector<std::vector<String>>(x_dim * y_dim, std::vector<String>());
    std::vector<bool> markings = std::vector<bool>(x_dim * y_dim, false);
    std::vector<float> heuristics = std::vector<float>(x_dim * y_dim, 0.0);

    //Create priority queue
    std::priority_queue<Node*, vector<Node*>, CompareNode> PQ;

    //Add start to the priorityqueue
    PQ.push(start);

    //While queue isn't empty
    while (!PQ.empty()) {
        Node *currentNode = PQ.top();
        int cnIndex = PointToIndex(currentNode->x, currentNode->y, x_dim, y_dim);
        // If first node in queue is the goal, we've found the best path!
        if (currentNode == goal) {
            return paths[cnIndex];
        }

        // If the first node isn't "visited"
        if (!markings[cnIndex]) {
            // Note that it is possible for the node to be "visited", 
            // as there can be duplicates in the PQ if multiple paths get to the same node
            markings[cnIndex] = true;
            for (String n : currentNode->neighbors) {
                Node* neighbor = Nodes[n];
                int neighborIndex = PointToIndex(neighbor->x, neighbor->y, x_dim, y_dim);
                if (!markings[neighborIndex]) {
                    if (costs[neighborIndex] == -1 || costs[neighborIndex] > costs[cnIndex] + 1) {
                        // If they aren't in the PQ, or if we've just found a better path, add them to the PQ
                        // Don't worry about duplicates, we will only use each node once 
                        // (when we've found the optimal path to it)
                        //Update cost, then update the path to include the current best path
                        costs[neighborIndex] = costs[cnIndex] + 1;
                        paths[neighborIndex] = paths[cnIndex];
                        paths[neighborIndex].push(currentNode->id)
                        PQ.push(neighbor);
                    }
                }
            }
        }
        // Pop the currentNode to remove it from the queue
        PQ.pop();
    }
    // If we get to this part of the code, the queue is empty. This means there is no path to the goal.
}

// Returns the index in a vector of a matrix element at location (x, y)
// Doesn't matter whether we consider row-based or column-based indexing as long as we're consistent
int PointToIndex(int x, int y, x_dim, y_dim){
  return x * y_dim + y; 
}


#endif //__SRC_NAVIGATION_GLOBAL_PLANNER__