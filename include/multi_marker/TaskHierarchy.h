#include <iostream>
#include <Eigen/Core>
#include <LLkinematics.h>
#include <Task.h>
#include <vector>
#include <list>


using namespace std;
using namespace Eigen;



class TaskHierarchy{

/*

	TaskHierarchy class definition.
	
	Attributes:
	
	- vector<Task> hierarchy		vector of Task objects representing the prioritized task hierarchy


*/

private:

vector<Task> hierarchy;


public:

TaskHierarchy();
~TaskHierarchy();

void print();
void setHierarchy(vector<Task> h);
vector<Task> getHierarchy();
int size();
void clear();
Task operator[](int i);
void insert(Task t);

VectorXd getTaskError(int i, VectorXd q);
VectorXd getTaskValue(int i, VectorXd q);

void setTaskParameters(int i, vector<double> p);
VectorXd computeNSBsolution(VectorXd q);
bool findTask(Task t);


};


