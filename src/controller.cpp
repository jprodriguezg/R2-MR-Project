#include "stdafx.h"
#include "controller.h"



Controller::Controller(){

	// Init stringstream objects for data logging
	cmdJointPosSS.str("");
	msrJointPosSS.str("");
	cmdJointVelSS.str("");
	msrJointVelSS.str("");
	msrJointAccSS.str("");
	refCartVelSS.str("");
	refCartPosSS.str("");
	msrJointTorquesSS.str("");
	externalTorquesSS.str("");
	residualVectorSS.str("");
	appliedForceSS.str("");

	globalPlotSS.str("");

	// Dynamically instantiate an object kukaLWR
	robot = new kukaLWR;

	// Member variables initialization
	FRIResultValue = 0;

	cmdFloatJointPosition = new float[NUMBER_OF_JOINTS];
	std::fill_n(cmdFloatJointPosition, NUMBER_OF_JOINTS, 0.0);

	msrFloatJointPosition = new float[NUMBER_OF_JOINTS];
	std::fill_n(msrFloatJointPosition, NUMBER_OF_JOINTS, 0.0);

	msrFloatJointTorques = new float[NUMBER_OF_JOINTS];
	std::fill_n(msrFloatJointTorques, NUMBER_OF_JOINTS, 0.0);

	msrFloatExternalTorques = new float[NUMBER_OF_JOINTS];
	std::fill_n(msrFloatExternalTorques, NUMBER_OF_JOINTS, 0.0);

	cmdEigJointPosition.setZero(NUMBER_OF_JOINTS);
	msrEigJointPosition.setZero(NUMBER_OF_JOINTS);
	msrEigJointPositionPrev.setZero(NUMBER_OF_JOINTS);
	msrEigJointVelocity.setZero(NUMBER_OF_JOINTS);
	msrEigJointVelocityPrev.setZero(NUMBER_OF_JOINTS);
	msrEigJointAcceleration.setZero(NUMBER_OF_JOINTS);
	msrEigJointTorques.setZero(NUMBER_OF_JOINTS);
	externalEigTorques.setZero(NUMBER_OF_JOINTS);
	startEigJointPosition.setZero(NUMBER_OF_JOINTS);
	refEigCartesianVelocity.setZero(POSITION_DIM);
	refEigCartesianPosition.setZero(POSITION_DIM);
	eigEEPosition.setZero(POSITION_DIM);
	eigEEOrientation.setZero(ORIENTATION_DIM, ORIENTATION_DIM);
	Jee.setZero(TASK_DIM, NUMBER_OF_JOINTS);
	pinvJee.setZero(NUMBER_OF_JOINTS, TASK_DIM);

	residualVector.setZero(NUMBER_OF_JOINTS);
	prevResidualVector.setZero(NUMBER_OF_JOINTS);
	appliedForce.setZero(6);
	startJointPosition.setZero(NUMBER_OF_JOINTS);

	eye7.setIdentity(NUMBER_OF_JOINTS, NUMBER_OF_JOINTS);
	
	touchingLinkPosition.setZero(POSITION_DIM);
	Jtl.setZero(TASK_DIM, NUMBER_OF_JOINTS);
	pinvJtl.setZero(NUMBER_OF_JOINTS, TASK_DIM);
	
	// RML pointers initialization
	RML = new TypeIRML(NUMBER_OF_JOINTS, CYCLE_TIME);
	IP = new TypeIRMLInputParameters(NUMBER_OF_JOINTS);
	OP = new TypeIRMLOutputParameters(NUMBER_OF_JOINTS);

	firstTime = true;
}




Controller::~Controller(){
	
	// Delete dynamic structures
	delete cmdFloatJointPosition;
	delete msrFloatJointPosition;
	delete msrFloatJointTorques;
	delete msrFloatExternalTorques;
	delete robot;

}


void Controller::threadCallback(){

	std::cout << "Starting Controller callback ... " << std::endl;

	// Initialize the communication with the robot and instantiate an object kukaLWR to compute kinematics and dynamics
	std::cout << "Initializing FRI and the robot... " << std::endl;
	initFRIandRobot();

	Sleep(1);

	// Start the main loop
	bool whileCond = (REAL_ROBOT) ? (!close && FRI->IsMachineOK()) : (!close);

	firstTime = true;
	/*std::cout << "about to enter in the main loop" << std::endl;
	std::cout << close << std::endl;
	std::cout << FRI->IsMachineOK() << std::endl;*/
	run = true;
	while (whileCond){	// Until a stop signal is not sent, and the machine is successfully running ...

		catchKeyboardInput();

		if (run){
			mainLoop();
		}

		// Evaluate ending condition
		whileCond = (REAL_ROBOT) ? (!close && FRI->IsMachineOK()) : (!close);//*/
	}

	std::cout << "Controller callback is going to terminate ... " << std::endl;
	if (REAL_ROBOT) {
		FRI->StopRobot();
	}

}

void Controller::mainLoop(){

	if (REAL_ROBOT){
		FRI->WaitForKRCTick();
		FRI->GetMeasuredJointPositions(msrFloatJointPosition);

		/* Please uncomment this couple of lines if you need to get access to the torque information */
		FRI->GetMeasuredJointTorques(msrFloatJointTorques);
		FRI->GetEstimatedExternalJointTorques(msrFloatExternalTorques);
		

		std::memcpy(msrEigJointPosition.data(), msrFloatJointPosition, sizeof(float)*NUMBER_OF_JOINTS);

		std::memcpy(msrEigJointPosition.data(), msrFloatJointTorques, sizeof(float)*NUMBER_OF_JOINTS);

		std::memcpy(externalEigTorques.data(), msrFloatExternalTorques, sizeof(float)*NUMBER_OF_JOINTS);

	}
	else{
		Sleep(5);
	}

	msrJointPosSS << globalTime << ", " << msrEigJointPosition(0) << ", " << msrEigJointPosition(1) << ", " << msrEigJointPosition(2) << ", "
										<< msrEigJointPosition(3) << ", " << msrEigJointPosition(4) << ", " << msrEigJointPosition(5) << ", " << msrEigJointPosition(6) << "; " << std::endl;

	msrJointTorquesSS << globalTime << ", " << msrEigJointTorques(0) << ", " << msrEigJointTorques(1) << ", " << msrEigJointTorques(2) << ", "
												<< msrEigJointTorques(3) << ", " << msrEigJointTorques(4) << ", " << msrEigJointTorques(5) << ", " << msrEigJointTorques(6) << "; " << std::endl;
	externalTorquesSS << globalTime << ", " << externalEigTorques(0) << ", " << externalEigTorques(1) << ", " << externalEigTorques(2) << ", "
												<< externalEigTorques(3) << ", " << externalEigTorques(4) << ", " << externalEigTorques(5) << ", " << externalEigTorques(6) << "; " << std::endl;

	if(firstTime){
		firstTime = false;
		std::cout << "Entering in the first Main Loop" << std::endl;
		startJointPosition = msrEigJointPosition;
		//Beep(523,500);
		debugVariable = 0;
		
		//Variable Initialization (To move at the beginning)
		freeMotion = false;
		isXaxisBlocked = false;
		isYaxisBlocked = false;
		isZaxisBlocked = false;
		touchingLink = 7;
		radiusOffset = 0;

		commandedForceX = 0;

	}

	// Update kinematic and dynamic information of the robot
	// a. Get the current EE position
	std::memcpy(msrEigJointPosition.data(), msrFloatJointPosition, sizeof(float)*NUMBER_OF_JOINTS);
	robot->setJointPosition(msrEigJointPosition);
	robot->computeKinematic();
	eigEEPosition = robot->getEEPosition();
	msrEigJointVelocity = (msrEigJointPosition - msrEigJointPositionPrev) / CYCLE_TIME;
	msrEigJointAcceleration = (msrEigJointVelocity - msrEigJointVelocityPrev) / CYCLE_TIME;
	robot->setJointVelocity(msrEigJointVelocity);
	robot->setJointAcceleration(msrEigJointAcceleration);
	robot->computeDynamic();

	Jee = robot->computeFullJacobian();
	pinvJee = robot->computeDampedPinv(Jee);

	msrJointVelSS << globalTime << ", " << msrEigJointVelocity(0) << ", " << msrEigJointVelocity(1) << ", " << msrEigJointVelocity(2)
								<< ", " << msrEigJointVelocity(3) << ", " << msrEigJointVelocity(4) << ", " << msrEigJointVelocity(5) << ", " << msrEigJointVelocity(6) << "; " << std::endl;

	msrJointAccSS << globalTime << ", " << msrEigJointAcceleration(0) << ", " << msrEigJointAcceleration(1) << ", " << msrEigJointAcceleration(2)
								<< ", " << msrEigJointAcceleration(3) << ", " << msrEigJointAcceleration(4) << ", " << msrEigJointAcceleration(5) << ", " << msrEigJointAcceleration(6) << "; " << std::endl;

	/***************************************************** ---------- TESTING CODE ----------- ****************************************************************************/
	
	//Get the residual vector
	//robot->computeResidualFull(msrEigJointVelocity, msrEigJointTorques, 5.0);
	//residualVector = robot->resVec;
	bool isKukaResidual;
	isKukaResidual = true;
	if(isKukaResidual){
		//Get the residual Vector
		residualVector = externalEigTorques;
		//Filter the residual vector (low pass filter)
		float alpha = 0.75;
		float generalGain = 0.01;
		Eigen::VectorXf gainVector(NUMBER_OF_JOINTS);
		gainVector << 2, 2, 2, 2, 2, 2, 2;
		//residualVector = generalGain*(alpha*residualVector + (1 - alpha)*prevResidualVector);
		residualVector = generalGain*gainVector.cwiseProduct(alpha*residualVector + (1 - alpha)*prevResidualVector);
		prevResidualVector = (-1)*residualVector;
	}else{
		//Get the residual Vector
		robot->computeResidualFull(msrEigJointVelocity, msrEigJointTorques, 5.0);
		residualVector = robot->resVec;
		//Filter the residual vector (low pass filter)
		float alpha = 0.75;
		float generalGain = -0.0001 + debugVariable/10;
		Eigen::VectorXf gainVector(NUMBER_OF_JOINTS);
		gainVector << 2, 2, 2, 2, 2, 2, 2;
		//residualVector = generalGain*(alpha*residualVector + (1 - alpha)*prevResidualVector);
		residualVector = generalGain*gainVector.cwiseProduct(alpha*residualVector + (1 - alpha)*prevResidualVector);
		prevResidualVector = (-1)*residualVector;
	}
	

	residualVectorSS << /*globalTime << ", " <<*/ residualVector(0) << ", " << residualVector(1) << ", " << residualVector(2)
								<< ", " << residualVector(3) << ", " << residualVector(4) << ", " << residualVector(5) << ", " << residualVector(6) << "; " << std::endl;

	//Compute tool force
	//Eigen::VectorXf toolGravity(6);
	//toolGravity << 0, 0, dyn_pars_tip[0]*9.81, 0, 0, 0;

	//Compute the jacobian to the touching link
		Jtl = robot->computeFullJacobian(touchingLink);
		pinvJtl = robot->computeDampedPinv(Jtl);
	
	//Compute the applied force to the touching link
	///appliedForce = pinvJee.transpose()*(residualVector);
	appliedForce = (-1)*pinvJtl.transpose() * residualVector.head(touchingLink);
	appliedForceSS << /*globalTime << ", " <<*/ appliedForce(0) << ", " << appliedForce(1) << ", " << appliedForce(2)
								<< ", " << appliedForce(3) << ", " << appliedForce(4) << ", " << appliedForce(5) << ", " << appliedForce(6) << "; " << std::endl;

	//Add simulated Force
	appliedForce(0) += commandedForceX;
	commandedForceX = 0;

	//Erase blocked components of the appliedForce
	if(isXaxisBlocked){
		appliedForce(0) =  0;
		appliedForce(3) =  0;
		appliedForce(4) =  0;
		appliedForce(5) =  0;
	}
	if(isYaxisBlocked){
		appliedForce(1) =  0;
		appliedForce(4) =  0;
	}
	if(isZaxisBlocked){
		appliedForce(2) =  0;
		appliedForce(5) =  0;
	}
	
	//Compute touching link position
	touchingLinkPosition = robot->getLinkPosition(touchingLink);
		
	//Variable set (move at the beginning of the code)
	float delta_h = 0.45;
	float radiusExternalSphere =1.2162/2+0.05+radiusOffset;
	float radiusInternalSphere =0.55+radiusOffset-0.15;
	Eigen::VectorXf sphereCenter(3);	
	sphereCenter<<0.0,0.0, /*radiusExternalSphere-*/delta_h;	
		
	//Compute distance between the touching link and the Sphere Center
	float distance_TouchingLink_SphereCenter = (touchingLinkPosition-sphereCenter).norm();

	//Compute the nearest point to the external Sphere
	Eigen::VectorXf nearestPointToExternalSphere = (radiusExternalSphere-distance_TouchingLink_SphereCenter)*((touchingLinkPosition-sphereCenter)/distance_TouchingLink_SphereCenter)+touchingLinkPosition;

	Eigen::VectorXf surfaceDirectionalVector = (nearestPointToExternalSphere - sphereCenter)/(nearestPointToExternalSphere - sphereCenter).norm();
	bool goingOutside = appliedForce.dot(surfaceDirectionalVector) > 0 ? true : false; 


	Eigen::VectorXf normalForce = appliedForce.dot(surfaceDirectionalVector)*surfaceDirectionalVector;
	Eigen::VectorXf tangentialForce = appliedForce - normalForce;
	Eigen::VectorXf realForce = appliedForce;

	if(distance_TouchingLink_SphereCenter < radiusInternalSphere){ //ZONE I - SAFE
		if(zone != 1){
			zone = 1;
			//Beep(523,100);
			std::cout << "ZONE I" << std::endl;
		}
		cmdEigJointVelocity = Jtl.transpose()*appliedForce;
	}else{
		if(distance_TouchingLink_SphereCenter > radiusInternalSphere && distance_TouchingLink_SphereCenter < radiusExternalSphere){ //ZONE II - MEDIUM
			if(zone != 2){
				zone = 2;
				//Beep(1023,100);
				std::cout << "ZONE II" << std::endl;
			}
			if(!goingOutside){
				cmdEigJointVelocity = Jtl.transpose()*appliedForce;
			}else{
				float interpolationValue = ((nearestPointToExternalSphere-touchingLinkPosition).norm())/(radiusExternalSphere - radiusInternalSphere);
				cmdEigJointVelocity = Jtl.transpose()*(tangentialForce + interpolationValue*normalForce);
				realForce = tangentialForce + interpolationValue*normalForce;
				//cmdEigJointVelocity = Jtl.transpose()*(appliedForce*interpolationValue);
			}
		}else{ //ZONE III - FORBIDDEN
			if(zone != 3){
				zone = 3;
				//Beep(2048,100);
				std::cout << "ZONE III" << std::endl;
			}
			if(!goingOutside){
				cmdEigJointVelocity = Jtl.transpose()*appliedForce;
			}else{
				//cmdEigJointVelocity = Jtl.transpose()*appliedForce*0.0001;
				cmdEigJointVelocity = Jtl.transpose()*(tangentialForce);
				realForce = tangentialForce;
			}
		}
	}
	
	if(freeMotion){
		cmdEigJointVelocity = Jtl.transpose()*appliedForce;
	}

	//Set commanded velocity
	//cmdEigJointVelocity = residualVector*(-0.015) + (eye7 - pinvJtl*Jtl)*0.7*(startJointPosition - msrEigJointPosition);
	cmdJointVelSS << globalTime << ", " << cmdEigJointVelocity(0) << ", " << cmdEigJointVelocity(1) << ", " << cmdEigJointVelocity(2)
								<< ", " << cmdEigJointVelocity(3) << ", " << cmdEigJointVelocity(4) << ", " << cmdEigJointVelocity(5) << ", " << cmdEigJointVelocity(6) << "; " << std::endl;

	// Integrate velocity to get next joint positions
	cmdEigJointPosition += cmdEigJointVelocity*CYCLE_TIME;
	cmdJointPosSS << globalTime << ", " << cmdEigJointPosition(0) << ", " << cmdEigJointPosition(1) << ", " << cmdEigJointPosition(2)
								<< ", " << cmdEigJointPosition(3) << ", " << cmdEigJointPosition(4) << ", " << cmdEigJointPosition(5) << ", " << cmdEigJointPosition(6) << "; " << std::endl;

	
	/********** 3D PLOT  *************************/

	globalPlotSS << globalTime  << "," << touchingLinkPosition(0) << "," << touchingLinkPosition(1) << "," << touchingLinkPosition(2)
								<< "," << appliedForce(0) << "," << appliedForce(1) << "," << appliedForce(2)
								<< "," << realForce(0) << "," << realForce(1) << "," << realForce(2)
							    << "," << radiusInternalSphere << "," << radiusExternalSphere 
								<< "," << msrEigJointPosition(0) << "," << msrEigJointPosition(1) << "," << msrEigJointPosition(2) << ","
								<< "," << msrEigJointPosition(3) << "," << msrEigJointPosition(4) << "," << msrEigJointPosition(5) << "," << msrEigJointPosition(6)
								<< ";" << std::endl;
	
	/*/// Please replace this part of code with your own controller

	// Set the current Cartesian Velocity reference (in EE frame)
	refEigCartesianVelocity(0) = 0.0;
	refEigCartesianVelocity(1) = getCosVelocity(globalTime, T, A);
	refEigCartesianVelocity(2) = 0.0;

	// Rotate the velocity reference in the base frame
	refEigCartesianVelocity = robot->rotationWToFrame[6] * refEigCartesianVelocity;
	refCartVelSS << globalTime << ", " << refEigCartesianVelocity(0) << ", " << refEigCartesianVelocity(1) << ", " << refEigCartesianVelocity(2) << "; " << std::endl;

	// Get the current Cartesian position reference
	refEigCartesianPosition(0) = eigEEPosition(0);
	refEigCartesianPosition(1) = 0.0;
	refEigCartesianPosition(2) = eigEEPosition(2);
	refCartPosSS << globalTime << ", " << refEigCartesianPosition(0) << ", " << refEigCartesianPosition(1) << ", " << refEigCartesianPosition(2) << "; " << std::endl;


	// Compute the commanded joint velocities
	cmdEigJointVelocity = pinvJee*(refEigCartesianVelocity);// +Kp*(refEigCartesianPosition - eigEEPosition));
	cmdJointVelSS << globalTime << ", " << cmdEigJointVelocity(0) << ", " << cmdEigJointVelocity(1) << ", " << cmdEigJointVelocity(2)
								<< ", " << cmdEigJointVelocity(3) << ", " << cmdEigJointVelocity(4) << ", " << cmdEigJointVelocity(5) << ", " << cmdEigJointVelocity(6) << "; " << std::endl;

	// Integrate velocity to get next joint positions
	cmdEigJointPosition += cmdEigJointVelocity*CYCLE_TIME;
	cmdJointPosSS << globalTime << ", " << cmdEigJointPosition(0) << ", " << cmdEigJointPosition(1) << ", " << cmdEigJointPosition(2)
								<< ", " << cmdEigJointPosition(3) << ", " << cmdEigJointPosition(4) << ", " << cmdEigJointPosition(5) << ", " << cmdEigJointPosition(6) << "; " << std::endl;

	/*************************************************************************************************************************************************************************/
	// Apply next joint positions on the robot
	if (REAL_ROBOT){
		FRI->SetCommandedJointPositions(cmdEigJointPosition.data());
	}

	// Increase global time
	globalTime += CYCLE_TIME;

	// Update joint positions and velocities
	msrEigJointPositionPrev = msrEigJointPosition;
	msrEigJointVelocityPrev = msrEigJointVelocity;

}


void Controller::initFRIandRobot(){

	if (REAL_ROBOT)
	{

		_sleep(1000);

		//WaitForSingleObject(h, INFINITE);

		// Initialize Fast Research Interface object from file
		FRI = new FastResearchInterface(".\\res\\980039-FRI-Driver.init");
		
		// Check if FRI is working and start the robot
		if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_POSITION_CONTROL) || (!FRI->IsMachineOK()))
		{
			blue();
			printf("Program is going to stop the robot.\n");
			FRI->StopRobot();
			printf("Restarting the joint position control scheme.\n");
			reset();
			FRIResultValue = FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
			if ((FRIResultValue != EOK) && (FRIResultValue != EALREADY))
			{
				red();
				printf("An error occurred during starting up the robot...\n");
				reset();
				std::exit(1);
			}
			else{
				green();
				printf("Robot started up.\n");
				reset();
			}
		}

		// Get the initial joint positions
		FRI->GetMeasuredJointPositions(msrFloatJointPosition);
		FRI->SetCommandedJointPositions(msrFloatJointPosition); // <--- (ONLY HERE) NEVER, NEVER, NEVER PUT SOME CODE BETWEEN THESE TWO LINES, BECAUSE THE ROBOT GOES CRAZY (aka reaches joint speed limit and turns motors off)

		// Copy data in required variables
		std::memcpy(cmdFloatJointPosition, msrFloatJointPosition, sizeof(float)*NUMBER_OF_JOINTS);
		std::memcpy(cmdEigJointPosition.data(), cmdFloatJointPosition, sizeof(float)*NUMBER_OF_JOINTS);

	}

	// set kukaLWR kinematic variables
	std::memcpy(msrEigJointPosition.data(), msrFloatJointPosition, sizeof(float)*NUMBER_OF_JOINTS);
	robot->setJointPosition(msrEigJointPosition);
	robot->computeKinematic();
	eigEEPosition = robot->getEEPosition();
	msrEigJointVelocity = (msrEigJointPosition - msrEigJointPositionPrev) / CYCLE_TIME;
	msrEigJointAcceleration = (msrEigJointVelocity - msrEigJointVelocityPrev) / CYCLE_TIME;
	robot->setJointVelocity(msrEigJointVelocity);
	robot->setJointAcceleration(msrEigJointAcceleration);
	robot->computeDynamic();

	// Compute Jacobian and its Pseudo-inverse
	Jee = robot->computeFullJacobian();
	pinvJee = robot->computeDampedPinv(Jee);

	// Move the robot to an initial desired Cartesian position using Reflexes Motion Libraries
	// (Note that the Cartesian desired position is set in the initialization file)
	startEigJointPosition = RMLMotionFromDesiredCartPose(initialCartesianPosition);
	//std::cout << "end of initrobot" << std::endl;
}



Eigen::VectorXf Controller::RMLMotionFromDesiredCartPose(Eigen::VectorXf eeOffsetPosition){

	TypeIRML					*RML = NULL;
	TypeIRMLInputParameters		*IP = NULL;
	TypeIRMLOutputParameters	*OP = NULL;
	int	ResultValue;

	RML = new TypeIRML(NUMBER_OF_JOINTS, CYCLE_TIME);
	IP = new TypeIRMLInputParameters(NUMBER_OF_JOINTS);
	OP = new TypeIRMLOutputParameters(NUMBER_OF_JOINTS);
	ResultValue = 0;
	//	memset(JointValuesInRad, 0x0, NUMBER_OF_JOINTS * sizeof(float));

	if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_POSITION_CONTROL) || (!FRI->IsMachineOK()))
	{
		std::cout << "Program is going to stop the robot." << std::endl;
		FRI->StopRobot();

		FRI->GetMeasuredJointPositions(msrFloatJointPosition);
		FRI->SetCommandedJointPositions(msrFloatJointPosition);

		std::cout << "Restarting the joint position control scheme." << std::endl;
		ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);

		if ((ResultValue != EOK) && (ResultValue != EALREADY))
		{
			std::cout << "An error occurred during starting up the robot..." << std::endl;
			delete	RML;
			delete	IP;
			delete	OP;

			std::exit(1);
		}
	}

	std::cout << "Moving towards the Cartesian point [" << eeOffsetPosition(0) << ", " << eeOffsetPosition(1) << ", " << eeOffsetPosition(2) << "] ..." << std::endl;

	FRI->GetMeasuredJointPositions(msrFloatJointPosition);

	float initFloatJointPosition[7];
	std::memcpy(initFloatJointPosition, initialJointPosition.data(), sizeof(float)*NUMBER_OF_JOINTS);

	for (int i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		IP->CurrentPosition->VecData[i] = (double)DEG(msrFloatJointPosition[i]);
		IP->TargetPosition->VecData[i] = (double)DEG(initFloatJointPosition[i]);
		IP->MaxVelocity->VecData[i] = (double)50.0;
		IP->MaxAcceleration->VecData[i] = (double)50.0;
		IP->SelectionVector->VecData[i] = true;
	}

	ResultValue = TypeIRML::RML_WORKING;

	while ((FRI->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
	{
		FRI->WaitForKRCTick();

		ResultValue = RML->GetNextMotionState_Position(*IP, OP);

		if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
		{
			std::cout << "ERROR during trajectory generation (" << ResultValue << "). " << std::endl;
		}

		for (int i = 0; i < NUMBER_OF_JOINTS; i++)
		{
			cmdFloatJointPosition[i] = RAD((double)(OP->NewPosition->VecData[i]));
		}

		FRI->SetCommandedJointPositions(cmdFloatJointPosition);

		*(IP->CurrentPosition) = *(OP->NewPosition);
		*(IP->CurrentVelocity) = *(OP->NewVelocity);
	}

	if (!FRI->IsMachineOK())
	{
		std::cout << "ERROR, machine is not ready." << std::endl;

		delete	RML;
		delete	IP;
		delete	OP;

		std::exit(1);
	}

	if (REAL_ROBOT){
		FRI->WaitForKRCTick();
		FRI->GetMeasuredJointPositions(msrFloatJointPosition);
		std::memcpy(cmdEigJointPosition.data(), msrFloatJointPosition, sizeof(float)*NUMBER_OF_JOINTS);
	}
	return initialJointPosition;

}

int Controller::logData(){

	saveToFile(cmdJointVelSS, curPath + "cmdJointVel.dat");
	saveToFile(msrJointVelSS, curPath + "msrJointVel.dat");
	saveToFile(cmdJointPosSS, curPath + "cmdJointPos.dat");
	saveToFile(msrJointPosSS, curPath + "msrJointPos.dat");
	saveToFile(msrJointAccSS, curPath + "msrJointAcc.dat");
	saveToFile(refCartVelSS, curPath + "refCartVel.dat");
	saveToFile(refCartPosSS, curPath + "refCarttPos.dat");
	saveToFile(msrJointTorquesSS, curPath + "msrJointTorques.dat");
	saveToFile(externalTorquesSS, curPath + "externalTorques.dat");
	saveToFile(residualVectorSS, curPath + "residualVector.dat");
	saveToFile(appliedForceSS, curPath + "appliedForce.dat");

	saveToFile(globalPlotSS, curPath + "globalPlot.dat");
	

	return 0;

}


void Controller::catchKeyboardInput(){
	int chr;
	if (_kbhit()){

		chr = getch();
		chr = toupper(chr);
		//std::cout << chr << std::endl;
		switch (chr){
			case 13:{ //ENTER KEY
				// Start main loop
				run = !run;

				// Awake Force Sensor callback
				/*if (FORCE_SENSOR){
					SetEvent(h);
				}//*/
			}
				break;
			case 65:{ //Letter A
					debugVariable += 0.001;
					std::cout << "Debug Variable: " << debugVariable << std::endl;
			}
				break;
			//TOUCHING LINK CHOICE
			case 51:{ //Link 3
					touchingLink = 3;
					std::cout << "Selected Link: " << touchingLink << std::endl;
			}
				break;
			case 52:{ //Link 4
					touchingLink = 4;
					std::cout << "Selected Link: " << touchingLink << std::endl;
			}
				break;
			case 53:{ //Link 5
					touchingLink = 5;
					std::cout << "Selected Link: " << touchingLink << std::endl;
				}
				break;
			case 54:{ //Link 6
					touchingLink = 6;
					std::cout << "Selected Link: " << touchingLink << std::endl;
			}
				break;
			case 55:{ //Link 7
					touchingLink = 7;
					std::cout << "Selected Link: " << touchingLink << std::endl;
			}
				break;
			//CONSTRAINT MOTION
			case 70:{ //Letter F
					freeMotion = !freeMotion;
					std::cout << "Zone free motion: " << freeMotion << std::endl;
			}
				break;
			//BLOCKING AXIS
			case 88:{ //Letter X
					isXaxisBlocked = !isXaxisBlocked;
					std::cout << "Blocked Motion on X: " << isXaxisBlocked << std::endl;
			}
				break;
			case 89:{ //Letter Y
					isYaxisBlocked = !isYaxisBlocked;
					std::cout << "Blocked Motion on Y: " << isYaxisBlocked << std::endl;
			}
				break;
			case 90:{ //Letter Z
					isZaxisBlocked = !isZaxisBlocked;
					std::cout << "Blocked Motion on Z: " << isZaxisBlocked << std::endl;
			}
				break;
			//RADIUS OFFSET
			case 43:{ //Letter +
					radiusOffset += 0.05;
					std::cout << "Radius offeset: " << radiusOffset << std::endl;
			}
				break;
			case 45:{ //Letter -
					radiusOffset -= 0.05;
					std::cout << "Radius offeset: " << radiusOffset << std::endl;
			}
				break;
			//KEYBOARD FORCE
			case 87:{ //Letter W
					commandedForceX = 0.2+debugVariable;
			}
				break;
			case 83:{ //Letter S
					commandedForceX -= 2;
			}
				break;
			case 68:{ //Letter D
					debugVariable -= 0.001;
			}
				break;
		}
	}
}




