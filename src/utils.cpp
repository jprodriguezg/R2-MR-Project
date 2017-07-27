#include "stdafx.h"
#include "utils.h"

// Function implementations

void inthand(int signum){
	close = true;
}

void saveToFile(std::stringstream& buff, std::string name){
	//---------------------------------------------
	//Input/Output State Bits
	//---------------------------------------------
	//| Goodbit | false/true | ERROS/NO ERRORS
	//| Eofbit  | 1          | End of File bit SET
	//| Failbit | 2          | Fail bit SET
	//| Badbit  | 4          | Bad bit SET
	//---------------------------------------------
	//| E+F     | 3
	//| E+B     | 5
	//| F+B     | 6
	//| E+F+B   | 7
	//---------------------------------------------
	std::ofstream data;
	data.open(name, std::ofstream::app);
	int i = 0;
	int j = 0;
	if (data.is_open()){
		blue();
		std::cout << "Writing to " << name;
		reset();
		do{
			if (i>3) break;
			else{
				i++;
				blue();
				std::cout << ".";
				reset();
				data.write(buff.str().c_str(), buff.str().length());
			}
		} while (!data.good());
		if (data.good()){
			green();
			std::cout << "OK";
			reset();
			blue();
			std::cout << "\nClosing file";
			reset();
		}
		else{
			red();
			std::cout << "ERRORS ON STREAM(" << data.rdstate() << ")";
			reset();
			blue();
			std::cout << "\nClosing file";
			data.clear();
		}
		do{
			blue();
			std::cout << ".";
			reset();
			data.close();
		} while (data.fail());
		green();
		std::cout << "OK\n";
		reset();
	}
	else{
		red();
		std::cout << "Error while opening file " << name << "!";
		reset();
	}
}


float lowpassfilter(float in, float old, float T, float f){
	float tau = 1.0 / f;
	float alpha = T / tau;
	return in*alpha + (1 - alpha)*old;
}


int initFromFile(){

	std::ifstream initF(".\\res\\KukaSkeleton.init", std::ifstream::in);			// define the object in charge of reading the input file

	if (!initF.is_open())	// check successful file open
	{
		red();								// show next message on the console in red
		std::cout << "Cannot open init file!\n";
		reset();							// reset default console message color
		return -1;							// exit if the file is not found or cannot be opened
	}

	std::string initVrbls;
	std::stringstream strVrbls("");				// define and initialize a string variable where the whole file content is stored
	while (!initF.eof())					// until the end of the file ...
	{
		getline(initF, initVrbls);			// put each content (i.e., each word)in the string variable initVrbls
		if (!(initVrbls[0] == '#'))			// if the current content is valid (i.e., it's not a comment)
		{
			strVrbls << initVrbls << " ";	// store that content in stringstream variable, followed by a space
		}
	}
	// At this point, the stringstream object contains all the content of the file. Store the data in the corresponding variables
	strVrbls
		>> REAL_ROBOT																								        // Use real robot						  (Flag)
		>> FORCE_SENSOR																									    // Use Force Sensor						  (Flag)
		>> LOG_DATA																										    // Enable data logging				      (Flag)
		>> SINUSOIDAL																										// Enable sinusoidal Trajectory		      (Flag)
		>> CYCLE_TIME																										// Cycle Time [ms]
		>> NEW_DYNAMIC_MODEL																								// Use new dynamic model				  (Flag)
		>> W_FRICTION																										// Consider friction in the dynamic model (Flag)
		>> EE_Z_OFFSET																										// Tool offset [m]
		>> dyn_pars_tip[0]																									// Tool weight [kg]
		>> dyn_pars_tip[3]																									// Tool CoM [kg]
		>> initialJointPosition(0)																							// ...
		>> initialJointPosition(1)																							// ...
		>> initialJointPosition(2)																							// ...
		>> initialJointPosition(3)																							// ...
		>> initialJointPosition(4)																							// ...
		>> initialJointPosition(5)																							// ...
		>> initialJointPosition(6)																							// ... Initial joint reference positions
		>> initialCartesianPosition(0)																						// ...																							
		>> initialCartesianPosition(1)																						// ...																							
		>> initialCartesianPosition(2)																						// ... Initial Cartesian reference position
		>> T
		>> A;

	yellow();								// show next message on the console in yellow

	std::cout
		<< "\nREAL_ROBOT              = " << ((REAL_ROBOT) ? ("true") : ("false"))
		<< "\nFORCE_SENSOR            = " << ((FORCE_SENSOR) ? ("true") : ("false"))
		<< "\nLOG_DATA                = " << ((LOG_DATA) ? ("true") : ("false"))
		<< "\nSINUSOIDAL              = " << ((SINUSOIDAL) ? ("true") : ("false"))
		<< "\nCYCLE_TIME              = " << CYCLE_TIME
		<< "\nNEW_DYNAMIC_MODEL       = " << ((NEW_DYNAMIC_MODEL) ? ("true") : ("false"))
		<< "\nW_FRICTION              = " << ((W_FRICTION) ? ("true") : ("false"))
		<< "\nEE_Z_OFFSET             = " << EE_Z_OFFSET
		<< "\nTOOL_WEIGHT             = " << dyn_pars_tip[0]
		<< "\nTOOL_COM                = " << dyn_pars_tip[3]
		<< "\nJOINT_INIT              = " << initialJointPosition(0) << ", "
		<< initialJointPosition(1) << ", "
		<< initialJointPosition(2) << ", "
		<< initialJointPosition(3) << ", "
		<< initialJointPosition(4) << ", "
		<< initialJointPosition(5) << ", "
		<< initialJointPosition(6) << ", "
		<< "\nCART_INIT               = " << initialCartesianPosition(0) << ", "
		<< initialCartesianPosition(1) << ", "
		<< initialCartesianPosition(2) << ", "
		<< "\nT                       = " << T
		<< "\nA                       = " << A
		<< "\n";

	reset();
	green();						// show next message on the console in green
	std::cout << "Init file loaded!\n";
	reset();

	return 1;
}


int red()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		std::cout << "Error while getting input handle" << std::endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_INTENSITY);
}

int green()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		std::cout << "Error while getting input handle" << std::endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_GREEN | FOREGROUND_INTENSITY);
}

int blue()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		std::cout << "Error while getting input handle" << std::endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_BLUE | FOREGROUND_INTENSITY);
}

int yellow()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		std::cout << "Error while getting input handle" << std::endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY);
}

int magenta()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		std::cout << "Error while getting input handle" << std::endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
}

int cyan()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		std::cout << "Error while getting input handle" << std::endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
}

int reset()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		std::cout << "Error while getting input handle" << std::endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
}

float getCosVelocity(float t, float T, float A){
	float ret;
	float s = t / T;
	ret = -A*sin(2 * M_PI*s) * 2 * M_PI / T;
	return ret;
}

float getCosTrajectory(float t, float T, float A){
	float ret;
	float s = t / T;
	ret = A*cos(2 * M_PI*s);
	return ret;
}



// Global Variables initialization
bool REAL_ROBOT;
bool FORCE_SENSOR;
bool SINUSOIDAL;
bool LOG_DATA;

bool close = false; // signal-triggered flag allowing threads to terminate
Eigen::VectorXf initialCartesianPosition;
Eigen::VectorXf initialJointPosition;

float T;
float A;
std::string curPath;
