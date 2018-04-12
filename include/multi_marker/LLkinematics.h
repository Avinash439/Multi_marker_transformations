#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>


using namespace std;
using namespace Eigen;

#define PI 3.1415926535


double **elaborateDH(int joints, double *q, double **DH);
double **readDH(int joints, char file_txt[]);
double **readDH1(int joints);
double **readDH2(int joints);

double ***elaborateT(double **DH, int joints);
double ***Homogeneous_dh(int joints, double **DH);
double ***Rot_dh(double *alpha, double *theta, int joints);

double ***elaborateT0(double ***T, int joints);

void printArray3d(int row, int col, int depth, double ***array3d);
void printArray2d(int row, int col, double **array2d);
void printArray2d_from_Array3d(int row, int col, int joint, double ***array3d);

double ***allocateArray3d(int row, int col, int depth);
double **allocateArray2d(int row, int col);

void deallocateArray3d(int rows, int col, double ***array3d);
void deallocateArray2d(int rows, double **array3d);

double *q_init(int joints);

double ***jaco_directkinematics(VectorXd q);

MatrixXd directkinematics(VectorXd q);
MatrixXd directkinematics(VectorXd q, int id);

double **jacobian(VectorXd q);

MatrixXd J_oripos(VectorXd q);
MatrixXd J_ori(VectorXd q);
MatrixXd J_pos(VectorXd q);
MatrixXd J_obst_wrist(Vector3d p_obst, VectorXd q);
MatrixXd J_obst_elbow(Vector3d p_obst, VectorXd q, int id);
MatrixXd J_jointlimit(VectorXd q, int id);
MatrixXd J_manipulability(VectorXd q);
MatrixXd J_wall(Vector3d p1, Vector3d p2, Vector3d p3, Vector3d p_d,  VectorXd q);
MatrixXd J_fov(Vector3d p,  VectorXd q);

MatrixXd J_vehicle_oripos(VectorXd q);
MatrixXd J_vehicle_pos(VectorXd q);
MatrixXd J_vehicle_ori(VectorXd q);
MatrixXd J_vehicle_attitude(VectorXd q);
MatrixXd J_vehicle_altitude(VectorXd q);
MatrixXd J_vehicle_obstacle(VectorXd q, Vector3d p_obst);
MatrixXd inv_J_vehicle_oripos(VectorXd q);

MatrixXd T_uvms();
MatrixXd J_uvms_ee_pos(VectorXd q);
MatrixXd J_uvms_vehicle_pos(VectorXd q);


VectorXd value_oripos(VectorXd q);
VectorXd value_ori(VectorXd q);
VectorXd value_pos(VectorXd q);
VectorXd value_obst_wrist(Vector3d p_obst, VectorXd q);
VectorXd value_obst_elbow(Vector3d p_obst, VectorXd q, int id);
VectorXd value_jointlimit(VectorXd q, int id);
VectorXd value_manipulability(VectorXd q);
VectorXd value_wall(Vector3d p1, Vector3d p2, Vector3d p3, Vector3d p_d, VectorXd q);
VectorXd value_fov(Vector3d p, VectorXd q);

VectorXd value_vehicle_pos(VectorXd q);
VectorXd value_vehicle_ori(VectorXd q);
VectorXd value_vehicle_oripos(VectorXd q);
VectorXd value_vehicle_attitude(VectorXd q);
VectorXd value_vehicle_altitude(VectorXd q);
VectorXd value_vehicle_obstacle(VectorXd q, Vector3d p_obst);
VectorXd value_uvms_ee_pos(VectorXd q);
VectorXd value_uvms_vehicle_pos(VectorXd q);



double *LinearAndAngular(double **p_J, double **z, int joint);

int countRows(char file_txt[]);

int sign(double x);

VectorXd rot2quat(Matrix3d R);

Matrix3d quat2rot(VectorXd e);

Matrix3d rpy2rot(Vector3d rpy);
VectorXd R2axis(Matrix3d R);

VectorXd rpy2quat(Vector3d rpy);
Vector3d quat2rpy(VectorXd quat);

Vector3d rot2rpy_1(Matrix3d R);
double *rot2rpy_2(double **R);

Matrix3d skew(VectorXd v);

void trapezoidal(double qi, double qf, double tf, double t, double &q, double &dq, double &ddq);

MatrixXd array2dtoMatrixXd(double **M, int rows, int cols);
VectorXd array1dtoVectorXd(double *m, int size);
double *VectorXdtoarray1d(VectorXd m);

VectorXd quatError(VectorXd ed, VectorXd e);

VectorXd inverseKinematics(Vector3d xd, VectorXd ed, VectorXd q, Vector3d &eo, Vector3d &error_x, char algorithm[], VectorXd gain, double &t, VectorXd &ve);

double *mask_Vrep2DH_Jaco6dof(double *qVrep, int joints);
double *mask_DH2Vrep_Jaco6dof(double *qDH, int joints);

VectorXd mask_Vrep2DH_Jaco7dof(VectorXd qVrep, int joints);
VectorXd mask_DH2Vrep_Jaco7dof(VectorXd qDH, int joints);

VectorXd mask_DH2Gazebo(VectorXd qDH, int joints);
VectorXd mask_Gazebo2DH(VectorXd qVrep, int joints);
VectorXd mask_DH_Gazebo_velocity(VectorXd qdot, int joints);

VectorXd mask_Gazebo2DH_sim(VectorXd qVrep, int joints);
VectorXd mask_DH2Gazebo_sim(VectorXd qDH, int joints);
VectorXd mask_DH_Gazebo_velocity_sim(VectorXd qdot, int joints);

VectorXd mask_Gazebo2DH_right(VectorXd qVrep, int joints);
VectorXd mask_DH_Gazebo_velocity_right(VectorXd qdot, int joints);

Vector3d GetEEPosition(VectorXd q);
VectorXd GetEEQuaternion(VectorXd q);
VectorXd VelocitySaturation(VectorXd dq);



MatrixXd mypinv(MatrixXd Jac);
MatrixXd mypinvScaled(MatrixXd Jac, MatrixXd W);
bool any(MatrixXd J);
MatrixXd mypinv1(MatrixXd J, MatrixXd W, int flag_algorithm, double error_norm, double max_out, int flag_sigma, double &d, double &l, double &sigma_min);
VectorXd bubble_sort(VectorXd v);
MatrixXd dls_pinv(MatrixXd J, MatrixXd W, double error_norm, double max_out, int flag_sigma);

MatrixXd J01(Vector3d eta_ee1d, VectorXd q);
MatrixXd J02(Vector3d p_obst, VectorXd q);
MatrixXd J03(VectorXd q);
MatrixXd J04(VectorXd q);
MatrixXd J05(VectorXd q);
MatrixXd manipulability_jacobian(VectorXd q);
MatrixXd J_obstacle_elbow(Vector3d p_obst, VectorXd q);
MatrixXd J_plane(Vector3d p1, Vector3d p2, Vector3d p3, VectorXd q);


double measure_of_Manipulability(VectorXd q);

double sigma_tilde01(Vector3d eta_ee1d, VectorXd q);
double sigma_tilde02(Vector3d p_obst, VectorXd q, double d);
Vector3d sigma_tilde03(Vector3d eta_ee1d, VectorXd q);
Vector3d sigma_tilde04(VectorXd eta_ee_quat_d, VectorXd q);
VectorXd sigma_tilde05(Vector3d eta_ee1d, VectorXd eta_ee_quat_d, VectorXd q);
double sigmatilde_plane(Vector3d p1, Vector3d p2, Vector3d p3, VectorXd q);


MatrixXd J_joint1_limit(VectorXd q);
MatrixXd J_joint2_limit(VectorXd q);
MatrixXd J_joint3_limit(VectorXd q);
MatrixXd J_joint4_limit(VectorXd q);
MatrixXd J_joint5_limit(VectorXd q);
MatrixXd J_joint6_limit(VectorXd q);
MatrixXd J_joint7_limit(VectorXd q);

Matrix3d S(Vector3d x);

MatrixXd armbase2ik(MatrixXd T_in);
MatrixXd ik2armbase(MatrixXd T_in);

VectorXd odom2ik(VectorXd pos_in_odom);
VectorXd ik2odom(VectorXd pos_in_ik);
VectorXd ik2odom_rotation(VectorXd quaternion_in_ik);
VectorXd odom2ik_rotation(VectorXd quaternion_in_odom);


MatrixXd J_oripos_arm_floating_base(Vector3d vehicle_p,Matrix3d vehicle_rot,VectorXd q,int arm_id); // Author: E.Cataldi, 2017, SevilleExp
MatrixXd J_oripos_vehicle_manipulator(Vector3d vehicle_p,Matrix3d vehicle_rot,VectorXd q); // Author: E.Cataldi, 2017, Planner
MatrixXd J_vehicle_posori_C(Vector3d vehicle_p,Matrix3d vehicle_rot,VectorXd q, int arm_id); // Author: E.Cataldi, 2017, SevilleExp
Vector3d rot2rpy_Eigen(Matrix3d R); // Author: E.Cataldi, 2017, SevilleExp
Matrix3d tran_omega2drpy(Vector3d rpy); // Author: E.Cataldi, 2017, SevilleExp
MatrixXd directkinematics(Vector3d vehicle_p,Matrix3d vehicle_rot,VectorXd q,int arm_id); // Author: E.Cataldi, 2017, SevilleExp
MatrixXd directkinematics(Vector3d vehicle_p,Matrix3d vehicle_rot,VectorXd q,int arm_id,int id); // Author: E.Cataldi, 2017, SevilleExp

