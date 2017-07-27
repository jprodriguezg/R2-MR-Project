#ifndef UTILS_H_
#define UTILS_H_

// WIN includes
#include <Windows.h>

/// System include
#include <iostream>
#include <signal.h>
#include <fstream>

/// Project header includes
#include "kukaLWR.h"

// defines 
#define POSITION_DIM 3
#define ORIENTATION_DIM 3
#define TASK_DIM 6
#define METAL_BASE 0.016

#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / PI )
#endif


/// Function declarations

/** 
 Set program termination condition to true.
 */
void inthand(int signum);

/**
Initialize a set of variables and structures from an initialization file
*/
int initFromFile();

/**
Change the font color in the command prompt
*/
int red();
int green();
int blue();
int yellow();
int magenta();
int cyan();
int reset();

/**
Cosine trajectory
@param float [in]: current sample time
@param float [in]: Time period
@param float [in]: Sinusoidal magnitude
*/
float getCosTrajectory(float t, float T, float A);

/**
Cosine trajectory derivative
@param float [in]: current sample time
@param float [in]: Time period
@param float [in]: Sinusoidal magnitude
*/
float getCosVelocity(float t, float T, float A);

/*
Save the specified stringstream object in the string file
@param std::stringstream [in]: data do be stored
@param std::string [in]: desired output file name
*/
void saveToFile(std::stringstream&, std::string);

/*
Filter the input signal with a first order lowpass filter
*/
float lowpassfilter(float, float, float, float);

/// Global Variables definitions
extern bool REAL_ROBOT;
extern bool FORCE_SENSOR;
extern bool LOG_DATA;
extern bool SINUSOIDAL;
extern bool close;									// Program termination condition
extern Eigen::VectorXf initialCartesianPosition;	// Initial Cartesian EE Position (filled by file)
extern Eigen::VectorXf initialJointPosition;		// Initial Joint Position (filled by file)
extern float globalTime;
extern float T;
extern float A;
extern std::string curPath;

#endif //UTILS_H_