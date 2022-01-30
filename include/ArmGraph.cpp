#include "../include/ArmGraph.h"
#include "ButtonHandler.h"


ArmGraph::ArmGraph(ButtonHandler* bh) {
  bHandler = bh;

  // Initialize teleop button mappings. Add / remove / change mappings as needed.
  std::fill_n(teleopMap, NUM_NODES, -1);
  teleopMap[ButtonHandler::DOWN] = INTAKING;
  teleopMap[ButtonHandler::Y] = RING_FRONT;
  teleopMap[ButtonHandler::A] = ABOVE_MIDDLE;
  teleopMap[ButtonHandler::X] = RING_BACK;
  teleopMap[ButtonHandler::B] = PLACE_GOAL;
  teleopMap[ButtonHandler::RIGHT] = PLATFORM_LEVEL;

}

// LOOK HOW FUCKING SHORT AND CLEAN THIS IS
void ArmGraph::armMovement() {

  if (arrived) {

    // b is the button that was just pressed, or NONE if no button pressed this frame
    ButtonHandler::Button b = bHandler->getButtonPressed();

    // a relevant button was pressed, so set arm destination
    if (b != ButtonHandler::NONE && teleopMap[b] != -1) {
      generateShortestPath(targetNode, teleopMap[b]);
      targetArmPathIndex = 1;
    }

    // not at final destination, so since it's arrived at the current one, set the new target destination to the next on the route
    if (targetArmPathIndex != armPath.size() - 1) {
        targetArmPathIndex++;
    }
  }

  targetNode = armPath.at(targetArmPathIndex);

  /* WRITE MOTOR MOVEMENT CODE TO TARGETNODE HERE */

}

void ArmGraph::addEdge(int u, int v) {

  adj[u].push_back(v);
  adj[v].push_back(u);

}

// a modified version of BFS that stores predecessor
// of each vertex in array p
bool ArmGraph::BFS(std::vector<int> adj[], int src, int dest, int pred[]) {
    // a queue to maintain queue of vertices whose
    // adjacency list is to be scanned as per normal
    std::list<int> queue;
 
    // boolean array visited[] which stores the
    // information whether ith vertex is reached
    // at least once in the Breadth first search
    bool visited[NUM_NODES];
 
    // initially all vertices are unvisited
    // so v[i] for all i is false
    // and as no path is yet constructed
    for (int i = 0; i < NUM_NODES; i++) {
        visited[i] = false;
        pred[i] = -1;
    }
 
    // now source is first to be visited
    visited[src] = true;
    queue.push_back(src);
 
    // standard BFS algorithm
    while (!queue.empty()) {
        int u = queue.front();
        queue.pop_front();
        for (int i = 0; i < adj[u].size(); i++) {

            int n = adj[u][i]; // current neighbor

            // Skip disabled nodes
            if (!nodeEnabled[n]) continue;

            if (!visited[n]) {
                visited[n] = true;
                pred[n] = u;
                queue.push_back(n);
 
                // We stop BFS when we find
                // destination.
                if (n == dest)
                    return true;
            }
        }
    }
 
    return false;
}
 
// utility function to print the shortest distance
// between source vertex and destination vertex
void ArmGraph::generateShortestPath(int start, int dest) {

  // predecessor[i] array stores predecessor of
  // i and distance array stores distance of i
  // from s
  int pred[NUM_NODES];

  if (!BFS(adj, start, dest, pred)) {
      return;
  }

  // vector path stores the shortest path
  int crawl = dest;
  armPath.clear();
  armPath.push_back(crawl);
  while (pred[crawl] != -1) {
      armPath.push_back(pred[crawl]);
      crawl = pred[crawl];
  }

}