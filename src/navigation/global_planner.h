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
    float dx = n1.x – n2.x;
    float dy = n1.y – n2.y;
    return sqrt((dx * dx) + (dy * dy));
}

// Prepares all nodes to run A*.
void a_star_prep(Node* goal) {
    for (Node* node : graph) {
        // Add heuristic costs to every node in the graph
        // The heuristic is inversely proportional to the distance from the goal.
        node->heuristic = 1 / distance(node, goal);
        // Reset the nodes, as well
            // When going through all the nodes, clear their "path" vectors
        node->path.clear();
            // And set all their "visited" markers to be false
        node->marked = false;
            // And set their costs to -1
        node->cost = -1;
    }
}

// Uses the A* algorithm to find the shortest path between two nodes,
// Using a heuristic weight for each node inversely proportional to the distance from the goal.
vector<Node*> a_star(Node* start, Node* goal) {
    //Create priority queue
    std::priority_queue<Node, vector<Node>, CompareNode> PQ;
    a_star_prep(goal);

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
            for (Node* neighbor : currentNode->neighbors) {
                if (!neighbor->marked) {
                    if (neighbor->cost == -1 || ->cost > currentNode->cost + 1) {
                        // If they aren't in the PQ, or if we've just found a better path, add them to the PQ
                        // Don't worry about duplicates, we will only use each node once 
                        // (when we've found the optimal path to it)
                        neighbor->cost = currentNode->cost + 1;
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


