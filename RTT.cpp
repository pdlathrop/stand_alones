/* Standalone program to create a 2D obstacle environment and search it with Rapidly Exploring Random Trees algorithm, performing connection tests with straight-line rasterizations, and returning a traversible path 
Author: Paul Lathrop
Date last edited: 3/25/24
*/

#include <cstdlib>
#include <iostream>
#include <locale>
#include <vector>
#include <cmath>

struct node{
  //node class contains x and y coordinates and a parent index for use in trees (not graphs)
  float x;
  float y;
  int parentInd;
  node(float xCoord, float yCoord){
    //parent-less constructor
    x = xCoord;
    y = yCoord;
  }
};

struct searchResult{
  //for RRT return, tree and oracleCount being number of connections tested, for comparison purposes
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
  //between 0 and 1, some graininess based on what RAND_MAX is (32767?) but should be ok for most applications
  return (float)(rand()) / (float)(RAND_MAX);
}

float nodeDist(node node1, node node2){
  return sqrt(pow(node1.x-node2.x,2)+pow(node1.y-node2.y,2));
}

int findParent(node randNode, std::vector<node> tree, int bound){
  //returns index of closest parent node in tree to randNode
  float minDist = 2*bound;
  int chosenParentInd;
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

std::vector<node> unwrap(searchResult lastRun, int goalNodeIndex){
  //takes in searchResult object with tree field, traces back via parents from 0 indexed start to index goalNodeIndex and returns reverse order path of nodes as a list
  //assuming index 0 is the start. returns vector of nodes, 0 index is goal, last index is start (reverse order)
  std::vector<node> returnList;
  bool reachedStart = false;
  int currIndex = goalNodeIndex;
  std::vector<node> searchTree = lastRun.tree;
  while(!reachedStart){
    returnList.push_back(searchTree[currIndex]);
    currIndex = searchTree[currIndex].parentInd;
    if(currIndex == 0){ //when 0 index is start
      reachedStart = true;
    }
  }
  //could reverse it but no real need
  return returnList;
}

std::vector<node> unwrap(searchResult lastRun, int startNodeIndex, int goalNodeIndex){
  //takes in searchResult object with tree field, traces back via parents from startNodeIndex start to index goalNodeIndex and returns reverse order path of nodes as a list
  //for when 0 index is not the start. returns vector of nodes, 0 index is goal, last index is start (reverse order)
  std::vector<node> returnList;
  bool reachedStart = false;
  int currIndex = goalNodeIndex;
  std::vector<node> searchTree = lastRun.tree;
  while(!reachedStart){
    returnList.push_back(searchTree[currIndex]);
    currIndex = searchTree[currIndex].parentInd;
    if(currIndex == startNodeIndex){ 
      reachedStart = true;
    }
  }
  return returnList;
}

searchResult RRT(std::vector<obstacle>& obstacles, node xInit, node xGoal, float goalRad, int rastNum, int bound){
  //performs Rapidly-Exploring Random Tree search of bound*bound 2D space with obstacles defined in obstacles, initial node xInit, goal node xGoal, a radius of acceptible solution around goal goalRad
  //returns searchResult object made of tree and number of times connections are tested (oracleCount). A node within goalRad of xGoal is last element of tree.
  bool finished = false;
  std::vector<node> tree;
  xInit.parentInd = 0;
  tree.push_back(xInit);
  int oracleCount = 0;

  while(!finished){
    node randNode{randFloat()*bound,randFloat()*bound};
    randNode.parentInd = findParent(randNode, tree, bound);
    bool connectible = testConnection(tree[randNode.parentInd], randNode, obstacles, rastNum);
    oracleCount++;
    if(connectible){
      tree.push_back(randNode);
      if(nodeDist(randNode, xGoal) <= goalRad){
        finished = true;
      }
    }
  }
  return searchResult{tree, oracleCount};
}

int main() {
  std::srand(time(NULL)); //set new random seed
  int bound = 10;
  int rastNum = 17;
  //create some obstacles
  std::vector<obstacle> obstacles = createObstacles(10,.5,2,bound);
  searchResult run1 = RRT(obstacles, node{0,0}, node{9,9}, 0.5, rastNum, bound);
  std::cout << run1.tree.size() << std::endl;
  std::vector<node> path = unwrap(run1, run1.tree.size()); //returns reverse order
  return 0;
}
