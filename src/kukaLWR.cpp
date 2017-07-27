#include "stdafx.h"
#include "kukaLWR.h"
#include <Windows.h>

bool NEW_DYNAMIC_MODEL;
bool W_FRICTION;

float CYCLE_TIME;
float dyn_pars_tip[10];
float EE_Z_OFFSET;

using namespace Eigen;

kukaLWR::kukaLWR()
{
	jointPosition.setZero(7);
	jointVelocity.setZero(7);
	jointAcceleration.setZero(7);
	EEPosition.setZero(3);

	gravVec.setZero(7);
	fricVec.setZero(7);
	tauVec.setZero(7);
	massMatrix.setZero(7,7);
	coriolMatrix.setZero(7,7);

	matC = new float* [7];
    matB = new float* [7];

	for(int i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		matC[i] = new float[7];
        matB[i] = new float[7];
		kineWToFrame[i].setZero(4,4);
		rotationWToFrame[i].setZero(3,3);
		Mv[i].setZero(4,4);
        Rv[i].setZero(3,3);
	}

	resVec.setZero(7);
	resVecKuka.setZero(7);

	mmntVec.setZero(7);
	sumDyn.setZero(7);
	sumRes.setZero(7);
	output.setZero(7);

}

void kukaLWR::setJointPosition(VectorXf jointPos)
{
	jointPosition = jointPos;
	return;
}

void kukaLWR::setJointVelocity(VectorXf jointVel)
{
	jointVelocity = jointVel;
	return;
}

void kukaLWR::setJointAcceleration(VectorXf jointAcc)
{
	jointAcceleration = jointAcc;
	return;
}

void kukaLWR::computeKinematic()
{
	float C1,S1,C2,S2,C3,S3,C4,S4,C5,S5,C6,S6,C7,S7;

    C1=cos(jointPosition(0));
    S1=sin(jointPosition(0));
    C2=cos(jointPosition(1));
    S2=sin(jointPosition(1));
    C3=cos(jointPosition(2));
    S3=sin(jointPosition(2));
    C4=cos(jointPosition(3));
    S4=sin(jointPosition(3));
    C5=cos(jointPosition(4));
    S5=sin(jointPosition(4));
    C6=cos(jointPosition(5));
    S6=sin(jointPosition(5));
    C7=cos(jointPosition(6));
    S7=sin(jointPosition(6));

    Mv[0] << C1,0,S1,0,
          S1,0,-C1,0,
          0,1,0,(float)(LINK1_LENGHT+TABLE_LENGHT),
          0,0,0,1;
    Mv[1] << C2,0,-S2,0,
          S2,0,C2,0,
          0,-1,0,0,
          0,0,0,1;
    Mv[2] << C3,0,-S3,0,
          S3,0,C3,0,
          0,-1,0,(float)(LINK2_LENGHT+LINK3_LENGHT),
          0,0,0,1;
    Mv[3] << C4,0,S4,0,
          S4,0,-C4,0,
          0,1,0,0,
          0,0,0,1;
    Mv[4] << C5,0,S5,0,
          S5,0,-C5,0,
          0,1,0,(float)(LINK4_LENGHT+LINK5_LENGHT),
          0,0,0,1;
    Mv[5] << C6,0,-S6,0,
          S6,0,C6,0,
          0,-1,0,0,
          0,0,0,1;
    Mv[6] << C7,-S7,0,0,
          S7,C7,0,0,
          0,0,1,(float)(LINK6_LENGHT + EE_Z_OFFSET),
          0,0,0,1;

    kineWToFrame[0] = Mv[0];
    kineWToFrame[1] = kineWToFrame[0]*Mv[1];
    kineWToFrame[2] = kineWToFrame[1]*Mv[2];
    kineWToFrame[3] = kineWToFrame[2]*Mv[3];
    kineWToFrame[4] = kineWToFrame[3]*Mv[4];
    kineWToFrame[5] = kineWToFrame[4]*Mv[5];
    kineWToFrame[6] = kineWToFrame[5]*Mv[6];

	Rv[0] << C1,0,S1,
          S1,0,-C1,
          0,1,0;
    Rv[1] << C2,0,-S2,
          S2,0,C2,
          0,-1,0;
    Rv[2] << C3,0,-S3,
          S3,0,C3,
          0,-1,0;
    Rv[3] << C4,0,S4,
          S4,0,-C4,
          0,1,0;
    Rv[4] << C5,0,S5,
          S5,0,-C5,
          0,1,0;
    Rv[5] << C6,0,-S6,
          S6,0,C6,
          0,-1,0;
    Rv[6] << C7,-S7,0,
          S7,C7,0,
          0,0,1;

    rotationWToFrame[0] = Rv[0];
    rotationWToFrame[1] = rotationWToFrame[0]*Rv[1];
    rotationWToFrame[2] = rotationWToFrame[1]*Rv[2];
    rotationWToFrame[3] = rotationWToFrame[2]*Rv[3];
    rotationWToFrame[4] = rotationWToFrame[3]*Rv[4];
    rotationWToFrame[5] = rotationWToFrame[4]*Rv[5];
    rotationWToFrame[6] = rotationWToFrame[5]*Rv[6];

	return;
}

VectorXf kukaLWR::getEEPosition()
{
	VectorXf EEPosition;
	EEPosition.setZero(3);
	
	EEPosition(0) = kineWToFrame[6](0,3);
	EEPosition(1) = kineWToFrame[6](1,3);
	EEPosition(2) = kineWToFrame[6](2,3);

	return EEPosition;
}

VectorXf kukaLWR::getLinkPosition(int link)
{
	int linkTh = link - 1;
	VectorXf LinkPosition;
	LinkPosition.setZero(3);

	if(linkTh < 0 || linkTh > 6)
		return LinkPosition;
	
	LinkPosition(0) = kineWToFrame[linkTh](0,3);
	LinkPosition(1) = kineWToFrame[linkTh](1,3);
	LinkPosition(2) = kineWToFrame[linkTh](2,3);

	return LinkPosition;
}

MatrixXf kukaLWR::computeJacobian(int link, int part)
{
	float C1,S1,C2,S2,C3,S3,C4,S4,C5,S5,C6,S6;

    float l2 = LINK2_LENGHT;
    float l3 = LINK3_LENGHT;
    float l4 = LINK4_LENGHT;
    float l5 = LINK5_LENGHT;
    float l6 = LINK6_LENGHT + EE_Z_OFFSET;

	MatrixXf jacobianAtLink;
	jacobianAtLink.setZero(6,7);

	if( link < LINK2 || link > END_EFFECTOR)
		return jacobianAtLink;

    C1=cos(jointPosition(0));
    S1=sin(jointPosition(0));
    C2=cos(jointPosition(1));
    S2=sin(jointPosition(1));
    C3=cos(jointPosition(2));
    S3=sin(jointPosition(2));
    C4=cos(jointPosition(3));
    S4=sin(jointPosition(3));
    C5=cos(jointPosition(4));
    S5=sin(jointPosition(4));
    C6=cos(jointPosition(5));
    S6=sin(jointPosition(5));

    if(link == LINK2)
    {
        jacobianAtLink(0,0)=0;
        jacobianAtLink(1,0)=0;
        jacobianAtLink(2,0)=0;
        jacobianAtLink(3,0)=0;
        jacobianAtLink(4,0)=0;
        jacobianAtLink(5,0)=1;

        jacobianAtLink(0,1)=0;
        jacobianAtLink(1,1)=0;
        jacobianAtLink(2,1)=0;
        jacobianAtLink(3,1)=S1;
        jacobianAtLink(4,1)=-C1;
        jacobianAtLink(5,1)=0;
    }

    else if(link == LINK3)
    {
        jacobianAtLink(0,0)=(l2 + l3)*S1*S2;
        jacobianAtLink(1,0)=-(l2 + l3)*C1*S2;
        jacobianAtLink(2,0)=0;
        jacobianAtLink(3,0)=0;
        jacobianAtLink(4,0)=0;
        jacobianAtLink(5,0)=1;

        jacobianAtLink(0,1)=-(l2 + l3)*C1*C2;
        jacobianAtLink(1,1)=-(l2 + l3)*S1*C2;
        jacobianAtLink(2,1)=-(l2 + l3)*S2;
        jacobianAtLink(3,1)=S1;
        jacobianAtLink(4,1)=-C1;
        jacobianAtLink(5,1)=0;

        jacobianAtLink(0,2)=0;
        jacobianAtLink(1,2)=0;
        jacobianAtLink(2,2)=0;
        jacobianAtLink(3,2)=-C1*S2;
        jacobianAtLink(4,2)=-S1*S2;
        jacobianAtLink(5,2)=C2;
    }

    else if(link == LINK4)
    {
        jacobianAtLink(0,0)=(l2 + l3)*S1*S2;
        jacobianAtLink(1,0)=-(l2 + l3)*C1*S2;
        jacobianAtLink(2,0)=0;
        jacobianAtLink(3,0)=0;
        jacobianAtLink(4,0)=0;
        jacobianAtLink(5,0)=1;

        jacobianAtLink(0,1)=-(l2 + l3)*C1*C2;
        jacobianAtLink(1,1)=-(l2 + l3)*S1*C2;
        jacobianAtLink(2,1)=-(l2 + l3)*S2;
        jacobianAtLink(3,1)=S1;
        jacobianAtLink(4,1)=-C1;
        jacobianAtLink(5,1)=0;

        jacobianAtLink(0,2)=0;
        jacobianAtLink(1,2)=0;
        jacobianAtLink(2,2)=0;
        jacobianAtLink(3,2)=-C1*S2;
        jacobianAtLink(4,2)=-S1*S2;
        jacobianAtLink(5,2)=C2;

        jacobianAtLink(0,3)=0;
        jacobianAtLink(1,3)=0;
        jacobianAtLink(2,3)=0;
        jacobianAtLink(3,3)=-C3*S1 - C1*C2*S3;
        jacobianAtLink(4,3)=-C2*S1*S3 + C1*C3;
        jacobianAtLink(5,3)=-S2*S3;
    }

    else if(link == LINK5)
    {
        jacobianAtLink(0,0)=-(l4 + l5)*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) + (l2 + l3)*S1*S2;
        jacobianAtLink(1,0)=-(l4 + l5)*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - (l2 + l3)*C1*S2;
        jacobianAtLink(2,0)=0;
        jacobianAtLink(3,0)=0;
        jacobianAtLink(4,0)=0;
        jacobianAtLink(5,0)=1;

        jacobianAtLink(0,1)=-C1*((l4 + l5)*(C2*C4 + C3*S2*S4) + (l2 + l3)*C2);
        jacobianAtLink(1,1)=-S1*((l4 + l5)*(C2*C4 + C3*S2*S4) + (l2 + l3)*C2);
        jacobianAtLink(2,1)=-(l2 + l3)*S2 - (l4 + l5)*C4*S2 + (l4 + l5)*C2*C3*S4;
        jacobianAtLink(3,1)=S1;
        jacobianAtLink(4,1)=-C1;
        jacobianAtLink(5,1)=0;

        jacobianAtLink(0,2)=-(l4 + l5)*S4*(C3*S1 + C1*C2*S3);
        jacobianAtLink(1,2)=(l4 + l5)*S4*(C1*C3 - C2*S1*S3);
        jacobianAtLink(2,2)=-(l4 + l5)*S2*S3*S4;
        jacobianAtLink(3,2)=-C1*S2;
        jacobianAtLink(4,2)=-S1*S2;
        jacobianAtLink(5,2)=C2;

        jacobianAtLink(0,3)=(l4 + l5)*C1*S2*S4 - (l4 + l5)*C4*S1*S3 + (l4 + l5)*C1*C2*C3*C4;
        jacobianAtLink(1,3)=(l4 + l5)*S1*S2*S4 + (l4 + l5)*C1*C4*S3 + (l4 + l5)*C2*C3*C4*S1;
        jacobianAtLink(2,3)=-(l4 + l5)*C2*S4 + (l4 + l5)*C3*C4*S2;
        jacobianAtLink(3,3)=-C3*S1 - C1*C2*S3;
        jacobianAtLink(4,3)=-C2*S1*S3 + C1*C3;
        jacobianAtLink(5,3)=-S2*S3;

        jacobianAtLink(0,4)=0;
        jacobianAtLink(1,4)=0;
        jacobianAtLink(2,4)=0;
        jacobianAtLink(3,4)=-S4*(S1*S3 - C1*C2*C3) - C1*C4*S2;
        jacobianAtLink(4,4)=S4*(C1*S3 + C2*C3*S1) - C4*S1*S2;
        jacobianAtLink(5,4)=C2*C4 + C3*S2*S4;
    }

    else if(link == LINK6)
    {
        jacobianAtLink(0,0)=-(l4 + l5)*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) + (l2 + l3)*S1*S2;
        jacobianAtLink(1,0)=-(l4 + l5)*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - (l2 + l3)*C1*S2;
        jacobianAtLink(2,0)=0;
        jacobianAtLink(3,0)=0;
        jacobianAtLink(4,0)=0;
        jacobianAtLink(5,0)=1;

        jacobianAtLink(0,1)=-C1*((l4 + l5)*(C2*C4 + C3*S2*S4) + (l2 + l3)*C2);
        jacobianAtLink(1,1)=-S1*((l4 + l5)*(C2*C4 + C3*S2*S4) + (l2 + l3)*C2);
        jacobianAtLink(2,1)=-(l2 + l3)*S2 - (l4 + l5)*C4*S2 + (l4 + l5)*C2*C3*S4;
        jacobianAtLink(3,1)=S1;
        jacobianAtLink(4,1)=-C1;
        jacobianAtLink(5,1)=0;

        jacobianAtLink(0,2)=-(l4 + l5)*S4*(C3*S1 + C1*C2*S3);
        jacobianAtLink(1,2)=(l4 + l5)*S4*(C1*C3 - C2*S1*S3);
        jacobianAtLink(2,2)=-(l4 + l5)*S2*S3*S4;
        jacobianAtLink(3,2)=-C1*S2;
        jacobianAtLink(4,2)=-S1*S2;
        jacobianAtLink(5,2)=C2;

        jacobianAtLink(0,3)=(l4 + l5)*C1*S2*S4 - (l4 + l5)*C4*S1*S3 + (l4 + l5)*C1*C2*C3*C4;
        jacobianAtLink(1,3)=(l4 + l5)*S1*S2*S4 + (l4 + l5)*C1*C4*S3 + (l4 + l5)*C2*C3*C4*S1;
        jacobianAtLink(2,3)=-(l4 + l5)*C2*S4 + (l4 + l5)*C3*C4*S2;
        jacobianAtLink(3,3)=-C3*S1 - C1*C2*S3;
        jacobianAtLink(4,3)=-C2*S1*S3 + C1*C3;
        jacobianAtLink(5,3)=-S2*S3;

        jacobianAtLink(0,4)=0;
        jacobianAtLink(1,4)=0;
        jacobianAtLink(2,4)=0;
        jacobianAtLink(3,4)=-S4*(S1*S3 - C1*C2*C3) - C1*C4*S2;
        jacobianAtLink(4,4)=S4*(C1*S3 + C2*C3*S1) - C4*S1*S2;
        jacobianAtLink(5,4)=C2*C4 + C3*S2*S4;

        jacobianAtLink(0,5)=0;
        jacobianAtLink(1,5)=0;
        jacobianAtLink(2,5)=0;
        jacobianAtLink(3,5)=C5*(C3*S1 + C1*C2*S3) - S5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4);
        jacobianAtLink(4,5)=S5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) - C5*(C1*C3 - C2*S1*S3);
        jacobianAtLink(5,5)=-S5*(C2*S4 - C3*C4*S2) + C5*S2*S3;
    }

    else if(link == END_EFFECTOR)
    {
        jacobianAtLink(0,0)=-(l4 + l5)*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) - l6*(C6*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) - S6*(C5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) + (l2 + l3)*S1*S2;
        jacobianAtLink(1,0)=-(l4 + l5)*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - l6*(C6*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - S6*(C5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - (l2 + l3)*C1*S2;
        jacobianAtLink(2,0)=0;
        jacobianAtLink(3,0)=0;
        jacobianAtLink(4,0)=0;
        jacobianAtLink(5,0)=1;

        jacobianAtLink(0,1)=-l6*C1*(S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5) + C6*(C2*C4 + C3*S2*S4)) - (l2 + l3)*C1*C2 - (l4 + l5)*C1*(C2*C4 + C3*S2*S4);
        jacobianAtLink(1,1)=-l6*S1*(S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5) + C6*(C2*C4 + C3*S2*S4)) - (l2 + l3)*C2*S1 - (l4 + l5)*S1*(C2*C4 + C3*S2*S4);
        jacobianAtLink(2,1)=l6*C2*S3*S5*S6 - (l4 + l5)*C4*S2 + (l4 + l5)*C2*C3*S4 - l6*C4*C6*S2 + l6*C2*C3*C6*S4 - (l2 + l3)*S2 - l6*C5*S2*S4*S6 - l6*C2*C3*C4*C5*S6;
        jacobianAtLink(3,1)=S1;
        jacobianAtLink(4,1)=-C1;
        jacobianAtLink(5,1)=0;

        jacobianAtLink(0,2)=-(l4 + l5)*C3*S1*S4 - (l4 + l5)*C1*C2*S3*S4 + l6*C3*C6*S1*S4 - l6*S1*S3*S5*S6 - l6*C1*C2*C6*S3*S4 + l6*C1*C2*C3*S5*S6 + l6*C3*C4*C5*S1*S6 + l6*C1*C2*C4*C5*S3*S6;
        jacobianAtLink(1,2)=-(l4 + l5)*C2*S1*S3*S4 + l6*C1*C3*C6*S4 + (l4 + l5)*C1*C3*S4 + l6*C1*S3*S5*S6 - l6*C1*C3*C4*C5*S6 - l6*C2*C6*S1*S3*S4 + l6*C2*C3*S1*S5*S6 + l6*C2*C4*C5*S1*S3*S6;
        jacobianAtLink(2,2)=-S2*((l4 + l5)*S3*S4 + l6*C6*S3*S4 - l6*C3*S5*S6 - l6*C4*C5*S3*S6);
        jacobianAtLink(3,2)=-C1*S2;
        jacobianAtLink(4,2)=-S1*S2;
        jacobianAtLink(5,2)=C2;

        jacobianAtLink(0,3)=(l4 + l5)*C1*S2*S4 - (l4 + l5)*C4*S1*S3 + (l4 + l5)*C1*C2*C3*C4 + l6*C1*C6*S2*S4 - l6*C4*C6*S1*S3 - l6*C1*C4*C5*S2*S6 - l6*C5*S1*S3*S4*S6 + l6*C1*C2*C3*C4*C6 + l6*C1*C2*C3*C5*S4*S6;
        jacobianAtLink(1,3)=(l4 + l5)*S1*S2*S4 + (l4 + l5)*C1*C4*S3 + (l4 + l5)*C2*C3*C4*S1 + l6*C1*C4*C6*S3 + l6*C6*S1*S2*S4 + l6*C2*C3*C4*C6*S1 - l6*C4*C5*S1*S2*S6 + l6*C1*C5*S3*S4*S6 + l6*C2*C3*C5*S1*S4*S6;
        jacobianAtLink(2,3)=l6*C3*C5*S2*S4*S6 + (l4 + l5)*C3*C4*S2 - l6*C2*C6*S4 + l6*C3*C4*C6*S2 + l6*C2*C4*C5*S6 - (l4 + l5)*C2*S4;
        jacobianAtLink(3,3)=-C3*S1 - C1*C2*S3;
        jacobianAtLink(4,3)=-C2*S1*S3 + C1*C3;
        jacobianAtLink(5,3)=-S2*S3;

        jacobianAtLink(0,4)=l6*S6*(C3*C5*S1 + C1*C2*C5*S3 + C1*S2*S4*S5 - C4*S1*S3*S5 + C1*C2*C3*C4*S5);
        jacobianAtLink(1,4)=l6*S6*(C2*C5*S1*S3 - C1*C3*C5 + C1*C4*S3*S5 + S1*S2*S4*S5 + C2*C3*C4*S1*S5);
        jacobianAtLink(2,4)=l6*S6*(C5*S2*S3 - C2*S4*S5 + C3*C4*S2*S5);
        jacobianAtLink(3,4)=-S4*(S1*S3 - C1*C2*C3) - C1*C4*S2;
        jacobianAtLink(4,4)=S4*(C1*S3 + C2*C3*S1) - C4*S1*S2;
        jacobianAtLink(5,4)=C2*C4 + C3*S2*S4;

        jacobianAtLink(0,5)=l6*C1*C4*S2*S6 + l6*C3*C6*S1*S5 + l6*S1*S3*S4*S6 - l6*C1*C2*C3*S4*S6 + l6*C1*C2*C6*S3*S5 - l6*C1*C5*C6*S2*S4 + l6*C4*C5*C6*S1*S3 - l6*C1*C2*C3*C4*C5*C6;
        jacobianAtLink(1,5)=l6*C4*S1*S2*S6 - l6*C1*C3*C6*S5 - l6*C1*S3*S4*S6 - l6*C1*C4*C5*C6*S3 - l6*C2*C3*S1*S4*S6 + l6*C2*C6*S1*S3*S5 - l6*C5*C6*S1*S2*S4 - l6*C2*C3*C4*C5*C6*S1;
        jacobianAtLink(2,5)=-l6*C3*S2*S4*S6 + l6*C2*C5*C6*S4 - l6*C2*C4*S6 + l6*C6*S2*S3*S5 - l6*C3*C4*C5*C6*S2;
        jacobianAtLink(3,5)=C5*(C3*S1 + C1*C2*S3) - S5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4);
        jacobianAtLink(4,5)=S5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) - C5*(C1*C3 - C2*S1*S3);
        jacobianAtLink(5,5)=-S5*(C2*S4 - C3*C4*S2) + C5*S2*S3;

        jacobianAtLink(0,6)=0;
        jacobianAtLink(1,6)=0;
        jacobianAtLink(2,6)=0;
        jacobianAtLink(3,6)=-C6*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) + S6*(C5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4) + S5*(C3*S1 + C1*C2*S3));
        jacobianAtLink(4,6)=C6*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) - S6*(C5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) + S5*(C1*C3 - C2*S1*S3));
        jacobianAtLink(5,6)=C6*(C2*C4 + C3*S2*S4) + S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5);
    }

	if(part == FULL_JACOBIAN)
		return jacobianAtLink;

	else if(part == LINEAR_JACOBIAN)
	{
		MatrixXf output;
		output.setZero(3,7);
		
		output.row(0) = jacobianAtLink.row(0);
		output.row(1) = jacobianAtLink.row(1);
		output.row(2) = jacobianAtLink.row(2);
		return output;
	}
	else if(part == ANGULAR_JACOBIAN)
	{
		MatrixXf output;
		output.setZero(3,7);
		
		output.row(0) = jacobianAtLink.row(3);
		output.row(1) = jacobianAtLink.row(4);
		output.row(2) = jacobianAtLink.row(5);

		return output;
	}

}

MatrixXf kukaLWR::computeFullJacobian(int link)
{
	return computeJacobian(link, FULL_JACOBIAN);
}

MatrixXf kukaLWR::computeLinearJacobian(int link)
{
	return computeJacobian(link, LINEAR_JACOBIAN);
}

MatrixXf kukaLWR::computeAngularJacobian(int link)
{
	return computeJacobian(link, ANGULAR_JACOBIAN);
}

MatrixXf kukaLWR::getLinearPartJacobian(MatrixXf A)
{
		MatrixXf output;
		output.setZero(3,7);

		if(A.rows() != 6)
			return output;

		output.row(0) = A.row(0);
		output.row(1) = A.row(1);
		output.row(2) = A.row(2);

		return output;

}

MatrixXf kukaLWR::getAngularPartJacobian(MatrixXf A)
{
		MatrixXf output;
		output.setZero(3,7);

		if(A.rows() != 6)
			return output;

		output.row(0) = A.row(3);
		output.row(1) = A.row(4);
		output.row(2) = A.row(5);

		return output;

}

MatrixXf kukaLWR::computeDampedPinv(MatrixXf A)
{
    int rows = A.rows();
    float sigMin,sigMax,sigMin2,sigMax2,damping;
	
    VectorXf sig;
    MatrixXf Winv;
    MatrixXf ut;
    MatrixXf vtt;
    MatrixXf vt;
	MatrixXf output;

    JacobiSVD<MatrixXf> svd(A, ComputeFullU | ComputeFullV);

	output.setZero(7,rows);

    sig.resize(rows);
    sig = svd.singularValues();

    Winv.resize(7,rows);
    Winv.setZero(7,rows);

    ut.resize(rows,rows);
    ut = svd.matrixU();
    ut.transposeInPlace();

    vtt = svd.matrixV();

    sigMin = sig(rows - 1);
    sigMax = sig(0);
    sigMax2=sigMax*sigMax;
    sigMin2=sigMin*sigMin;


    if ((sigMax2/sigMin2) > CONDITIONAL_NUMBER_LIMIT)
    {
       damping=(CONDITIONAL_NUMBER_LIMIT*sigMin2 - sigMax2)/(1 - CONDITIONAL_NUMBER_LIMIT);
    } else
        {
            damping=0;
        }

    for (int i=0; i<rows; i++)
    {
       Winv(i,i)=sig(i) / (sig(i)*sig(i) + damping);
    }

    return output = vtt*Winv*ut;
}

void kukaLWR::computeDynamic()
{
	
	for(int i=0; i<7;i++)
    {
		jointPos[i] = jointPosition(i);
		jointVel[i] = jointVelocity(i);
		jointAcc[i] = jointAcceleration(i);
	}

	if (NEW_DYNAMIC_MODEL)
	{
		dyn.get_g(matG, jointPos, dyn_pars_tip);
		dyn.get_B(matB, jointPos, dyn_pars_tip);
		dyn.get_S(matC, jointPos, jointVel, dyn_pars_tip);
		dyn.get_tau(matT, jointPos, jointVel, dyn_pars_tip);
		dyn.get_friction(matF, jointVel);
	}
	else
	{
		dyn.get_g(matG, jointPos);
		dyn.get_B(matB, jointPos);
		dyn.get_S(matC, jointPos, jointVel);
		dyn.get_tau(matT, jointPos, jointVel, jointAcc);
		dyn.get_friction(matF, jointVel);
	}

    for( int i = 0; i < NUMBER_OF_JOINTS; i++)
    {
        gravVec(i) = matG[i];
        fricVec(i) = matF[i];
		if (W_FRICTION)
		{
			tauVec(i) = matT[i];
		}
		else
		{
			tauVec(i) = matT[i] - fricVec(i);
		}
        for( int j = 0; j < NUMBER_OF_JOINTS; j++)
        {
            coriolMatrix(i,j) = matC[i][j];
            massMatrix(i,j) = matB[i][j];
        }
    }

    coriolMatrix(6,6) = 0;

    return;
}

void kukaLWR::computeResidualFull(VectorXf q_dot, VectorXf trq, float gain)
{

	mmntVec = massMatrix*q_dot;
	if (W_FRICTION)
	{
		sumDyn += (trq + coriolMatrix.transpose().eval()*q_dot - gravVec - fricVec)*CYCLE_TIME;
	}
	else
	{
		sumDyn += (trq + coriolMatrix.transpose().eval()*q_dot - gravVec)*CYCLE_TIME;
	}
	sumRes +=output*CYCLE_TIME;
	output = gain*(mmntVec - sumDyn - sumRes) / (1 + gain*CYCLE_TIME);
	resVec << output;

	return;
}