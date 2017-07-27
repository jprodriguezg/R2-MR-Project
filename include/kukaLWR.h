#ifndef _KUKA_LWR_H
#define _KUKA_LWR_H

#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen\Dense>
#include <LWR_Dynamic_Model_Lib.h>

#define NUMBER_OF_JOINTS	7
#define CONDITIONAL_NUMBER_LIMIT 800
#define TABLE_LENGHT 0.0
#define LINK1_LENGHT 0.326//0.326+0.018
#define LINK2_LENGHT 0.2
#define LINK3_LENGHT 0.2
#define LINK4_LENGHT 0.2
#define LINK5_LENGHT 0.19
#define LINK6_LENGHT 0.078

#define LINK1 1
#define LINK2 2
#define LINK3 3
#define LINK4 4
#define LINK5 5
#define LINK6 6
#define END_EFFECTOR 7

#define FULL_JACOBIAN 0
#define LINEAR_JACOBIAN 1
#define ANGULAR_JACOBIAN 2

extern bool NEW_DYNAMIC_MODEL;
extern bool W_FRICTION;

extern float CYCLE_TIME;
extern float dyn_pars_tip[10];
extern float EE_Z_OFFSET;

class kukaLWR
{
	
	Eigen::VectorXf jointPosition;
	Eigen::VectorXf jointVelocity;
	Eigen::VectorXf jointAcceleration;
	Eigen::VectorXf EEPosition;

	Eigen::VectorXf mmntVec;
	Eigen::VectorXf sumDyn;
	Eigen::VectorXf sumRes;
	Eigen::VectorXf output;

	Eigen::MatrixXf Mv[NUMBER_OF_JOINTS];
	Eigen::MatrixXf Rv[NUMBER_OF_JOINTS];

	CLWR_Dynamic_Model_Lib dyn;
	float	jointPos[NUMBER_OF_JOINTS];
	float	jointVel[NUMBER_OF_JOINTS];
	float	jointAcc[NUMBER_OF_JOINTS];
	float	matG[NUMBER_OF_JOINTS];
	float	matF[NUMBER_OF_JOINTS];
	float	matT[NUMBER_OF_JOINTS];
	float**	matB;
	float**	matC;


public:

	Eigen::VectorXf gravVec;
	Eigen::VectorXf fricVec;
	Eigen::VectorXf tauVec;
	Eigen::VectorXf resVecKuka;
	Eigen::VectorXf resVec;
	Eigen::MatrixXf coriolMatrix;
	Eigen::MatrixXf massMatrix;
	Eigen::MatrixXf kineWToFrame[NUMBER_OF_JOINTS];
	Eigen::MatrixXf rotationWToFrame[NUMBER_OF_JOINTS];

	kukaLWR();
	~kukaLWR(){};
	
	void setJointPosition(Eigen::VectorXf jointPos);
	void setJointVelocity(Eigen::VectorXf jointVel);
	void setJointAcceleration(Eigen::VectorXf jointAcc);
	void computeKinematic();
	void computeDynamic();
	void computeResidualFull(Eigen::VectorXf q_dot, Eigen::VectorXf trq, float gain = 5.0);
	
	Eigen::VectorXf getEEPosition();
	Eigen::VectorXf getLinkPosition(int link = END_EFFECTOR);
	Eigen::MatrixXf computeFullJacobian(int link = END_EFFECTOR);
	Eigen::MatrixXf computeLinearJacobian(int link = END_EFFECTOR);
	Eigen::MatrixXf computeAngularJacobian(int link = END_EFFECTOR);
	Eigen::MatrixXf getLinearPartJacobian(Eigen::MatrixXf A);
	Eigen::MatrixXf getAngularPartJacobian(Eigen::MatrixXf A);
	Eigen::MatrixXf computeDampedPinv(Eigen::MatrixXf A);

private:

	Eigen::MatrixXf computeJacobian(int link = END_EFFECTOR, int part = FULL_JACOBIAN);

};
#endif
