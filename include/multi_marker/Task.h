#include <iostream>
#include <Eigen/Core>
#include <LLkinematics.h>
#include <vector>
#include <list>
#include <TaskDefinitions.h>

using namespace std;
using namespace Eigen;

/*
	Task class definition. 
	
	Attributes:
	
	int type			type of the task. Refer to the definitions above
	Eigen::MatrixXd K		gain matrix
	int task_dim			task dimension
	vector<double> params		parameters vector (see below)
	
	
	
	The parameters vector contains the desired values and other informations for the computation of the task solution. Its dimension depends from the specific task:
	
	- Tasks from 1 to 7:		dim: 1		params[0] = desired joint position
	
	- Task 8:			dim: 3		params[0:2] = desired end-effector position
	
	- Task 9:			dim: 4		params[0:3] = desired end-effector quaternion
	
	- Task 10:			dim: 7		params[0:3] = desired end-effector quaternion
							params[4:6] = desired end-effector position
							
	- Task 11: 			dim: 1		params[0] = desired manipulability measure
	
	- Task 12: 			dim: 4		params[0] = desired distance from the obstacle
							params[1:3] = obstacle position
							
	- Task 13: 			dim: 10		params[0] = desired distance from the plane
							params[1:3] = coordinates of a first point belonging to the plane
							params[4:6] = coordinates of a second point belonging to the plane
							params[7:9] = coordinates of a third point belonging to the plane

*/

class Task{

private:

int type; 
MatrixXd K;
vector<double> params;
int task_dim;
int set_based;
int threshold_violated;
int number;

public:

Task();
Task(int type, MatrixXd K, vector<double> params);
~Task();

MatrixXd Jacobian(VectorXd q);
VectorXd Error(VectorXd q);
MatrixXd NullJ(VectorXd q);
VectorXd computeSolution(VectorXd q);
VectorXd taskValue(VectorXd q);

int getType();
MatrixXd getK();
int getTaskDim();
vector<double> getParams();
void setParams(vector<double> p);
void print();
void setThresholdViolated(int t);
int getThresholdViolated();
int isSetBased();
int getNumber();

};
