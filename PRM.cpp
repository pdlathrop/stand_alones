#include <cstdlib>
#include <iostream>
#include <locale>
#include <vector>
#include <cmath>
#include <stack>
#include <unordered_map>

struct node{
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
  std::vector<node> tree;
  int oracleCount;
};

struct obstacle{
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
  //returns true if free, false if not
  for(int j = 0; j < obstacles.size(); j++){
    if(testNode.x >= obstacles[j].x1 && testNode.x <= obstacles[j].x2 && testNode.y >= obstacles[j].y1 && testNode.y <= obstacles[j].y2){
      return false;
    }
  }
  return true;
}

void populateStartEndNodes(node& startNode, node& endNode, std::vector<obstacle>& obstacles, float bound){
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
      startNode.connectionInd.push_back(closeNodeIndStart[i]); //not marking node connections yet
    }
  }
  for(int i = 0; i < closeNodeIndGoal.size(); i++){
    PRM.oracleCount++;
    if(testConnection(goalNode, PRM.tree[closeNodeIndGoal[i]], obstacles, rastNum)){
      goalNode.connectionInd.push_back(closeNodeIndGoal[i]); //not marking node connections yet
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
    return 1; 
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

std::vector<node> searchPRM(roadmap& PRM, int startInd, int goalInd, int rastNum){
  //init a search, also create an unordered map of parents to return path
  std::unordered_map<int,int> parentMap;
  std::stack<int> searchList;
  std::vector<int> visited(PRM.tree.size(),0);
  searchList.push(startInd);//push start node
  while(!searchList.empty()){
    int indexTemp = searchList.top(); 
    //std::cout << "index of current check: " << indexTemp << std::endl;
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
  //now unwrap and return path
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
  if(addReturn == 2){
     std::vector<node> returnPath = searchPRM(createdPRM, createdPRM.tree.size()-2,  createdPRM.tree.size()-1, rastNum);
    std::cout << returnPath.size() << std::endl;
    printPath(returnPath);
  }
  return 0;
}
