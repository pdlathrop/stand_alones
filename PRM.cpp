/* Standalone program to create a 2D obstacle environment, create a probabilistic roadmap (PRM) performing connection tests with straight-line rasterizations, connect an initial and goal node to the PRM, and returning a traversible path via a graph search
Author: Paul Lathrop
Date last edited: 3/25/24
*/


#include <cstdlib>
#include <iostream>
#include <locale>
#include <vector>
#include <cmath>
#include <stack>
#include <unordered_map>

struct node{
  //node class contains x and y coordinates and vector int of indices of nodes that are connected. Works when the tree/graph is not order-changed. Would be more robust to use a unique numbering system separate to index (or other key)
  float x;
  float y;
  std::vector<int> connectionInd;
  node(float xCoord, float yCoord){
    x = xCoord;
    y = yCoord;
  }
  node(){ //empty constructor
    x = -1;
    y = -1;
  }
};

struct roadmap{
  //for PRM return, tree and oracleCount being number of connections tested, for comparison purposes. Tree not technically correct term, graph is more appropriate.
  std::vector<node> tree;
  int oracleCount;
};

struct obstacle{
  //rectangular obstacles defined by bounds
  float x1;
  float x2;
  float y1; 
  float y2;
};

float randFloat(){
  //between 0 and 1, some graininess based on what RAND_MAX is (32767?) but should be ok
  return (float)(rand()) / (float)(RAND_MAX);
}

float nodeDist(node node1, node node2){
  return sqrt(pow(node1.x-node2.x,2)+pow(node1.y-node2.y,2));
}

int findParent(node randNode, std::vector<node> tree, int bound){
  //returns index of closest parent node in graph to randNode
  float minDist = 2*bound;
  int  chosenParentInd;
  for(int i = 0; i < tree.size(); i++){
    if(nodeDist(randNode, tree[i]) < minDist){
      minDist = nodeDist(randNode, tree[i]);
      chosenParentInd = i;
    }
  }
  return chosenParentInd;
}

bool testConnection(node node1, node node2, std::vector<obstacle>& obstacles, int rastNum){
  //simple connection test based on rasterizing straight line between node1 and node2 (with rastNum equally spread rasterizations), and checking all points for obstacle impact. Not perfect, better (and slower) with higher rastNum. Returns true if obstacle free straight line path exists, false otherwise.

  //create rasterizations 
  float xRange = node2.x - node1.x;
  float yRange = node2.y - node1.y;
  for(int i = 0; i < rastNum; i++){
    node testNode{node1.x + xRange*i/rastNum, node1.y + yRange*i/rastNum};
    for(int j = 0; j < obstacles.size(); j++){
      if(testNode.x >= obstacles[j].x1 && testNode.x <= obstacles[j].x2 && testNode.y >= obstacles[j].y1 && testNode.y <= obstacles[j].y2){
        return false;
      }
    }
  }
  return true;
}

bool testNode(node testNode,std::vector<obstacle>& obstacles){
  //boolean test of whether testNode is within an obstacle
  //returns true if free, false if not
  for(int j = 0; j < obstacles.size(); j++){
    if(testNode.x >= obstacles[j].x1 && testNode.x <= obstacles[j].x2 && testNode.y >= obstacles[j].y1 && testNode.y <= obstacles[j].y2){
      return false;
    }
  }
  return true;
}

void populateStartEndNodes(node& startNode, node& endNode, std::vector<obstacle>& obstacles, float bound){
  //method to edit startNode and endNode such that they are both outside of obstacles
  //float bound is size of environment (assumed square)
  //no return as nodes are edited by reference
  bool validStart = false;
  while(!validStart){
    node randNode{randFloat()*bound,randFloat()*bound};
    if(testNode(randNode, obstacles)){
      startNode = randNode;
      validStart = true;
    }
  }
  bool validEnd = false;
  while(!validEnd){
    node randNode{randFloat()*bound,randFloat()*bound};
    if(testNode(randNode, obstacles)){
      endNode = randNode;
      validEnd = true;
    }
  }
  return;
}

std::vector<obstacle> createObstacles(int numObs, float minSide, float maxSide, int bound){
  //create a numObs-long random obstacle list defining rectangular obstacles of minimum side length minSide and max length maxSide
  std::vector<obstacle> obstacles;
  float obsDiff = maxSide - minSide;
  for(int i = 0; i < numObs; i++){
    float x1 = randFloat()*bound;
    float y1 = randFloat()*bound;
    obstacle newObs{x1, x1+randFloat()*obsDiff, y1, y1+randFloat()*obsDiff};
    obstacles.push_back(newObs);
  }
  return obstacles;
}

roadmap createPRM(std::vector<obstacle>& obstacles, int numNodes, float bound, float connecThreshold, int rastNum){
  /*
  Method to create a roadmap of size numNodes within 2D obstacle environment as defined by obstacles
  Inputs:
  std::vector<obstacle>& obstacles: list of rectangular obstacles (obstacle objects) by reference
  int numNodes: number of nodes in graph to return
  float bound: side length of square environment (width and height equal)
  float connecThreshold: when a random node is drawn, attempts are made to connect all existing nodes within a distance connecThreshold
  int rastNum: number of equally spaced rasterizations to check between a node and a possible parent node, when testing connection feasibility within obstacle environment
  Return:
  roadmap object with fields
  std::vector<node> tree: list of nodes and their connections (by index)
  oracleCount: number of total connection tests tried (for comparison purposes)
  */
  int nodeCount = 0;
  int oracleCount = 0; 
  std::vector<node> tree;

  //initialize first node
  bool foundFreeInit = false;
  while(!foundFreeInit){
    node randNode{randFloat()*bound,randFloat()*bound};
    if(testNode(randNode, obstacles)){ //randnode is obstacle free
       tree.push_back(randNode); 
       foundFreeInit = true;
       nodeCount++;
    }
  }
  //add nodes until reached desired size
  while(nodeCount < numNodes){
    node randNode{randFloat()*bound,randFloat()*bound};
    std::vector<int> closeNodeInd;
    for(int i = 0; i < tree.size(); i++){ //find nodes that are close enough
      if(nodeDist(randNode,tree[i]) <= connecThreshold){
        closeNodeInd.push_back(i);
      }
    }
    for(int i = 0; i < closeNodeInd.size(); i++){ //for close enough nodes
      oracleCount++; //every call to testconnection increases oracleCount by 1
      if(testConnection(randNode,tree[closeNodeInd[i]],obstacles,rastNum)){
        randNode.connectionInd.push_back(closeNodeInd[i]); //mark connection in randnode
        tree[closeNodeInd[i]].connectionInd.push_back(tree.size()); //also mark connection in node, index will be current size - 1 + 1 (due to 0-index)
      }
    }
    if(!randNode.connectionInd.empty()){ //if some connections
      tree.push_back(randNode);
      nodeCount++;
    }
  }
  return roadmap{tree,oracleCount};
}

int addPRM(roadmap& PRM, std::vector<obstacle>& obstacles, node startNode, node goalNode, float connecThreshold, int rastNum){
  /*
  Method to add start and goal nodes to an existing PRM within an obstacle environment
  Inputs:
  roadmap& PRM: existing roadmap by reference
  std::vector<obstacle>& obstacles: list of rectangular obstacles (obstacle objects) by reference
  node startNode: start node to add to the PRM
  node goalNode: goal node to add to the PRM
  float connecThreshold: attempts are made to connect all existing nodes within a distance connecThreshold to start and goal nodes
  int rastNum: number of equally spaced rasterizations to check between a node and a possible parent node, when testing connection feasibility within obstacle environment
  Return:
  PRM edited by reference
  int is 0 for when neither start nor goal can be connected, 1 for when only start connected, -1 for when only end connected, and 2 for when both are. PRM is edited by reference when int is 1 or 2.
  */

  //find close enough nodes
  std::vector<int> closeNodeIndStart;
  std::vector<int> closeNodeIndGoal;
  for(int i = 0; i < PRM.tree.size(); i++){
    if(nodeDist(startNode,PRM.tree[i]) <= connecThreshold){
        closeNodeIndStart.push_back(i);
    }
    if(nodeDist(goalNode,PRM.tree[i]) <= connecThreshold){
        closeNodeIndGoal.push_back(i);
    }
  }
  for(int i = 0; i < closeNodeIndStart.size(); i++){
    PRM.oracleCount++;
    if(testConnection(startNode, PRM.tree[closeNodeIndStart[i]], obstacles, rastNum)){
      startNode.connectionInd.push_back(closeNodeIndStart[i]); //not marking node connections yet within PRM
    }
  }
  for(int i = 0; i < closeNodeIndGoal.size(); i++){
    PRM.oracleCount++;
    if(testConnection(goalNode, PRM.tree[closeNodeIndGoal[i]], obstacles, rastNum)){
      goalNode.connectionInd.push_back(closeNodeIndGoal[i]); //not marking node connections yet within PRM
    }
  }

  //deal with partial connections, all statements return
  if(!startNode.connectionInd.empty() && goalNode.connectionInd.empty()){ //start connected, goal is empty
    PRM.tree.push_back(startNode);
    for(int i = 0; i < startNode.connectionInd.size(); i++){
      PRM.tree[startNode.connectionInd[i]].connectionInd.push_back(PRM.tree.size() - 1); //start node's index is size - 1, as 1 node was added
    }
    return 1; 
  }
  if(!goalNode.connectionInd.empty() && startNode.connectionInd.empty()){ //goal connected, start is empty
    PRM.tree.push_back(goalNode);
    for(int i = 0; i < goalNode.connectionInd.size(); i++){
      PRM.tree[goalNode.connectionInd[i]].connectionInd.push_back(PRM.tree.size() - 1); //goal node's index is size - 1
    }
    return -1; 
  }
  if(goalNode.connectionInd.empty() && startNode.connectionInd.empty()){
    return 0; 
  }
  //else able to add both: add both then mark connections from existing node connection indices
  PRM.tree.push_back(startNode);
  PRM.tree.push_back(goalNode);
  for(int i = 0; i < startNode.connectionInd.size(); i++){
    PRM.tree[startNode.connectionInd[i]].connectionInd.push_back(PRM.tree.size() - 2); //start node's index is size - 2, as 2 nodes were added
  }
  for(int i = 0; i < goalNode.connectionInd.size(); i++){
    PRM.tree[goalNode.connectionInd[i]].connectionInd.push_back(PRM.tree.size() - 1); //goal node's index is size - 1
  }
  return 2;
}

int forceExpand(roadmap& PRM, node addNode, std::vector<obstacle>& obstacles, int rastNum, int connectAttempts, float bound, float connecThreshold){
  /*
  Method forces PRM expansion toward and around addNode in an attempt to connect it with the existing PRM (for use if addPRM returns -1, 1, and 0). Returns 1 when addNode successfully added, 0 if not
  */
  int attempts = 0;
  int batchsize = 10;
  bool connected = false;
  while(!connected || attempts != connectAttempts){//until we connect or timeout
    for(int i = 0; i < batchsize; i++){ //connect 10 new nodes to the PRM in direction of the new node
      bool make1Connection = false;
      while(!make1Connection){
        node randNode;
        if(i % 2 == 0){
          node randNode{randFloat()*bound,randFloat()*bound}; //true random node
        }else{
          node randNode{addNode.x + (1-2*randFloat()), addNode.y + (1-2*randFloat())}; //else direct rand node to be near addNode
        }
        connectAttempts++; //iterate this here, will overflow by the time mother while is reached

        std::vector<int> closeNodeInd;
        for(int i = 0; i < PRM.tree.size(); i++){ //find nodes that are close enough
          if(nodeDist(randNode,PRM.tree[i]) <= connecThreshold){
            closeNodeInd.push_back(i);
          }
        }
        for(int i = 0; i < closeNodeInd.size(); i++){ //for close enough nodes
          if(testConnection(randNode,PRM.tree[closeNodeInd[i]],obstacles,rastNum)){
            randNode.connectionInd.push_back(closeNodeInd[i]); //mark connection in randnode
            PRM.tree[closeNodeInd[i]].connectionInd.push_back(PRM.tree.size()); //also mark connection in node, index will be current size - 1 + 1 (due to 0-index)
          }
        }
        if(!randNode.connectionInd.empty()){ //if some connections
          PRM.tree.push_back(randNode);
          make1Connection = true; //to escape most recent while
        }
      }
      //for loop will time out if connections are made
    }
    //every 10 connections to PRM, attempt to connect to addNode via new connections
    for(int i = 0; i < batchsize; i++){
      if(testConnection(addNode, PRM.tree[PRM.tree.size()-1-i], obstacles, rastNum)){
        addNode.connectionInd.push_back(PRM.tree.size()-1-i); //this will iterate the last batchsize elements, backwards
        PRM.tree[PRM.tree.size()-1-i].connectionInd.push_back(PRM.tree.size()-1); //it will be last element
      }
    }
    if(!addNode.connectionInd.empty()){
      PRM.tree.push_back(addNode);
      connected = true; //we did it!
    }
  }
  if(connected){
    return 1;
  }
  return 0;
}

std::vector<node> searchPRM(roadmap& PRM, int startInd, int goalInd, int rastNum){
  /*
  Method to search via DFS (stack) an existing PRM for a path between index startInd and index goalInd. Path of nodes is returned.
  Inputs:
  roadmap& PRM: existing roadmap passed by reference
  int startInd: index of start node within PRM
  int goalInd: index of goal node within PRM
  int rastNum: number of equally spaced rasterizations to check between a node and a possible parent node, when testing connection feasibility within obstacle environment
  Returns:
  std::vector<node> : with path of nodes (in reverse order) from start index to goal index (including both start and goal nodes)
  */

  //init a search, also create an unordered map of parents to return path
  std::unordered_map<int,int> parentMap;
  std::stack<int> searchList;
  std::vector<int> visited(PRM.tree.size(),0);
  searchList.push(startInd);//push start node
  while(!searchList.empty()){
    int indexTemp = searchList.top(); 
    //std::cout << "index of current check: " << indexTemp << std::endl; //debug line
    searchList.pop();
    visited[indexTemp] = 1;
    if(indexTemp == goalInd){ //if its the index of goal, quit loop
      break; //leave while
    }
    for(int i = 0; i < PRM.tree[indexTemp].connectionInd.size(); i++){ //for all indexTemp's connections
      if(visited[PRM.tree[indexTemp].connectionInd[i]] == 0){ //if not visited
        searchList.push(PRM.tree[indexTemp].connectionInd[i]); //add to stack
        //not sure if next qualifier is necessary, as if node is unvisited then it doesn't exist as a key
        if(parentMap.count(PRM.tree[indexTemp].connectionInd[i]) == 0){ //if child doesn't exist as key
          parentMap[PRM.tree[indexTemp].connectionInd[i]] = indexTemp; //add child key, index temp parent
        }
      }
    }
  }

  //unwrap and return path
  std::vector<node> returnPath;
  int tempInd = goalInd; //goal ind is the start
  returnPath.push_back(PRM.tree[tempInd]);
  while(tempInd != startInd){ //until we find the start
    tempInd = parentMap[tempInd];
    returnPath.push_back(PRM.tree[tempInd]);
  }
  return returnPath;
}

void printPath(std::vector<node>& path){
  //method to print x and y values of all nodes in path 
  for(int i = 0; i < path.size(); i++){
    std::cout << "[" << path[i].x << "," << path[i].y << "]" << std::endl;
  }
  return;
}

int main() {
  std::srand(time(NULL)); //set new random seed
  int bound = 10;
  int rastNum = 13;
  //create some obstacles
  std::vector<obstacle> obstacles = createObstacles(10,.5,2,bound);

  roadmap createdPRM = createPRM(obstacles, 50, bound, 2, rastNum);
  std::cout << createdPRM.tree.size() << std::endl;
  node startNode; node endNode;
  populateStartEndNodes(startNode, endNode, obstacles, bound);
  std::cout << "startNode: [" << startNode.x << "," << startNode.y << "]" << std::endl;
  std::cout << "endNode: [" << endNode.x << "," << endNode.y << "]" << std::endl;

  int addReturn = addPRM(createdPRM, obstacles,  startNode, endNode, 3, rastNum);
  int success = 1;
  if(addReturn == -1){
    success = forceExpand(createdPRM, startNode, obstacles, rastNum, 250, bound, 2.5);
  }else if(addReturn == 1){
    success = forceExpand(createdPRM, endNode, obstacles, rastNum, 250, bound, 2.5);
  }else if(addReturn == 0){
    success = forceExpand(createdPRM, startNode, obstacles, rastNum, 250, bound, 2.5);
    success = forceExpand(createdPRM, endNode, obstacles, rastNum, 250, bound, 2.5);
  }

  if(success){
    std::vector<node> returnPath = searchPRM(createdPRM, createdPRM.tree.size()-2,  createdPRM.tree.size()-1, rastNum);
    std::cout << returnPath.size() << std::endl;
    printPath(returnPath);
  }
  

  return 0;
}
