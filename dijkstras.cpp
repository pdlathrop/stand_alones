/* Standalone program to implement Dijkstra's graph search algorithm on a weighted digraph
Author: Paul Lathrop
Date last edited: 4/11/24
*/

#include <cstdlib>
#include <iostream>
#include <locale>
#include <vector>
#include <cmath>
#include <stack>
#include <unordered_map>

struct node{
  //node class contains x and y coordinates and vector int of indices of nodes that are connected. Each constructed node gets a unique id via the static member field uniqueId
  int nodeId;
  float x;
  float y;
  inline static int uniqueId = 0;
  node(float xCoord, float yCoord){
    x = xCoord;
    y = yCoord;
    nodeId = uniqueId++; //unique node ids
  }
  node(){ //empty constructor
    x = -1;
    y = -1;
    nodeId = uniqueId++; //unique node ids
  }
};

struct connection{
  int parentId; //id of parent node
  int connectionId; //id of the node that is connected to parent node
  float cost; //cost of connection
  connection(int tempId1, int tempId2, float tempCost){
    parentId = tempId1;
    connectionId = tempId2;
    cost = tempCost;
  }
};

struct roadmap{
  std::vector<node> graph;
  int oracleCount;
  std::unordered_map<int, std::vector<connection>> connections; //key (int) is node id, connections vector is list of all that node's connections OUT
};

struct path{
  std::vector<int> indexPath;
  float cost;
}

float randFloat(){
  //between 0 and 1, some graininess based on what RAND_MAX is (32767?) but should be ok
  return (float)(rand()) / (float)(RAND_MAX);
}

float nodeDist(node node1, node node2){
  return sqrt(pow(node1.x-node2.x,2)+pow(node1.y-node2.y,2));
}

void printPath(std::vector<node>& path){
  //method to print x and y values of all nodes in path 
  for(int i = 0; i < path.size(); i++){
    std::cout << "[" << path[i].x << "," << path[i].y << "]" << std::endl;
  }
  return;
}

roadmap createWeightedGraph(node start, int numNodes, float bound, float connecBound){
  /*
  Method to create a weighted graph/roadmap with random weights and random node locations. Roadmap object is returned.
  Inputs:
  node start: starting node
  int numNodes: desired size of returned graph
  float bound: environment size
  float connecBound: try to connect all nodes within this radius to the current node
  */
  roadmap graphReturn;
  graphReturn.graph.push_back(start);
  while(graphReturn.graph.size() <= numNodes){
    node randNode = node(randFloat()*bound, randFloat()*bound);
    for(const auto& element:graphReturn.graph){ //element are nodes passed by reference
      if(nodeDist(element, randNode) <= connecBound){ //make connection with random weight to close enough nodes
        graphReturn.connections[element.nodeId].push_back(connection(randNode.nodeId, element.nodeId, 10*randFloat()));
        graphReturn.connections[randNode.nodeId].push_back(connection(element.nodeId, randNode.nodeId, 10*randFloat()));
      }
    }
    if(!graphReturn.connections[randNode.nodeId].empty()){
      graphReturn.graph.push_back(randNode);
    }
  }
  return graphReturn;
}

path dijkstrasSearch(roadmap envGraph, int start, int goal){
  /*
  Method to perform Dijkstra's algorithm on a roadmap envGraph from node of unique index start to goal
  Inputs:
  roadmap envGraph: weighted graph to be searched
  int start: unique start node index that is in envGraph.graph
  int goal: unique goal node index that is in envGraph.graph
  Output:
  path object, vector list of node indices (to change:: could be vector list of nodes themselves)
  */
  //initializations
  int currentNode = start;
  std::unordered_map<int, bool> visited;
  std::unordered_map<int, float> minCost;
  std::unordered_map<int, int> minParent;
  for(const auto& element:envGraph.graph){ //init maps of visited and cost to defaults
    visited[element.nodeId] = false;
    minCost[element.nodeId] = 10^10;
    minParent[element.nodeId] = -1;
  }
  minCost[start] = 0;
  finished = false;

  //search
  while(!finished){
    for(const auto& connectionTemp:envGraph.connections[currentNode]){ //go through currentNode's connections
      if(visited[connectionTemp.connectionId] == false && (minCost[currentNode] + connectionTemp.cost < minCost[connectionTemp.connectionId])){ //if unvisited and shorter path
        minCost[connectionTemp.connectionId] = minCost[currentNode] + connectionTemp.cost; //overwrite minCost
        minParent[connectionTemp.connectionId] = currentNode; //overwrite min parent index
      }
    }
    visited[currentNode] = true; 

    //find next lowest element
    int nextLowestCostNode = -1;
    int lowestCostSeenSoFar = 10^10;
    for(const auto& element:minCost){ //loop and find the next lowest cost node
      if(visited[element.first] == false){ 
        if(element.second < lowestCostSeenSoFar){ //if lowest cost so far
          lowestCostSeenSoFar = element.second;
          nextLowestCostNode = element.first; //overwrite and select
        }
      }
    }

    if(nextLowestCostNode == -1){ //if no overwrite, all nodes are visited, so stop
      finished = true;
    }
    currentNode = nextLowestCostNode;
  }

  //unwrap path
  path minPath;
  minPath.cost = minCost[start]; //calculated minimum cost
  int currentIndex = start;
  minPath.graph.push_back(currentIndex);
  while(!finished){
    currentIndex = minParent[currentIndex]; //go to next element
    minPath.graph.push_back(currentIndex);
    if(currentIndex == goal){
      finished = true;
    }
  }

  return minPath; 
}


int main(){
  std::srand(time(NULL)); //set new random seed
  // create sample random graph with weights
  float bound = 10;
  int numNodes = 50;
  float connecBound = 2.5;
  node start = node(0,0);
  roadmap graph1 = createWeightedGraph(start, numNodes, bound, connecBound);
  path returnPath = dijkstrasSearch(graph1, 40, 12);
  

  return 0;
}
