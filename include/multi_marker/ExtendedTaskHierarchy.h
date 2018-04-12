#include <iostream>
#include <Eigen/Core>
#include <LLkinematics.h>
//#include <Task.h>
#include <TaskHierarchy.h>
#include <vector>

using namespace std;
using namespace Eigen;

struct node{

TaskHierarchy h;

node *left;
node *right;

};

class ExtendedTaskHierarchy{

private:

TaskHierarchy taskHierarchy;
TaskHierarchy activeTasks;



public:

node *tree;
ExtendedTaskHierarchy();
~ExtendedTaskHierarchy();

TaskHierarchy getTaskHierarchy();
TaskHierarchy getActiveTasks();
void setTaskHierarchy(TaskHierarchy h);
void setActiveTasks(VectorXd q);
void clearActiveTasks();
int size();
void printTaskHierarchy();
void printActiveTasks();
int numberofSetBased();
void setTaskParameters(int i, vector<double> p);

VectorXd getTaskError(int id, VectorXd q);
VectorXd getTaskValue(int id, VectorXd q);
Task getTask(int i);

vector<VectorXd> computeSolutions(TaskHierarchy h);

void buildTree(TaskHierarchy hierarchy, int i, node *cur);
void printSolutions(node *t);
void fillSolutionsVector(node *t, vector<TaskHierarchy> &v, vector<VectorXd> &solutions, VectorXd q);
vector<VectorXd> evaluateProjections(TaskHierarchy h, vector<VectorXd> &solutions, vector<TaskHierarchy> v, VectorXd q);
VectorXd chooseSolution(vector<VectorXd> sol);
VectorXd optimize(TaskHierarchy in, VectorXd q);

VectorXd computeDq(VectorXd q);

int treeNodes(node *cur);
int treeLeaves(node *cur);
void deleteTree(node *n);
void printTree(node *cur);



};




