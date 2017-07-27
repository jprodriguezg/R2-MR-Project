#include "stdafx.h"

// Boost headers
#define BOOST_THREAD_DYN_LINK
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/filesystem.hpp>

// Project header
#include "utils.h"
#include "controller.h"


float globalTime = 0;
std::string seqFileName = "sequenceFile.txt";
long double fileSeq = 0;

void createLogFolder(){

	std::ifstream seqFileIn;
	const long curSize = 1024;
	char curDirAndFile[curSize];

	GetCurrentDirectory(curSize, curDirAndFile);
	curPath = std::string(curDirAndFile);

	curPath += std::string("\\res\\log");

	seqFileIn.open(seqFileName, std::ios::in);

	// If "sequenceFile.txt" exists, read the last sequence from it and increment it by 1.
	if (seqFileIn.is_open())
	{
		seqFileIn >> fileSeq;
		fileSeq++;
	}
	else{
		fileSeq = 1; // if it does not exist, start from sequence 1.
	}

	curPath += std::to_string(fileSeq) + "\\";//*/
	std::cout << "curPath: " << curPath << std::endl;
	boost::filesystem::path dir(curPath);
	if (boost::filesystem::create_directory(dir)) {
		std::cout << "Log directory successfully created." << "\n";
	}
	else{
		std::cout << "Error in creating the log data folder. Exiting ... " << std::endl;
		std::exit(1);
	}
}

void closeLog(){

	std::ofstream seqFileOut;

	seqFileOut.open(seqFileName, std::ios::out);
	seqFileOut << fileSeq;
	seqFileOut.close();

}


int main(int argc, char** argv){

	std::cout << "Starting program ... " << std::endl;

	// Bind the termination signal with the desired function
	signal(SIGINT, inthand);

	// Instantiate Controller object
	Controller ctrl;

	// Initialize variables required to be read from file
	std::fill_n(dyn_pars_tip, 10, 0.0);
	initialCartesianPosition.setZero(POSITION_DIM);
	initialJointPosition.setZero(NUMBER_OF_JOINTS);

	std::cout << "Reading init file... " << std::endl;
	int res = initFromFile();
	if (res == 1){
		std::cout << "[main] Init file successfully opened. " << std::endl;
	}
	else{
		std::cout << "[main] Could not open the init file. Exiting ... " << std::endl;
		std::exit(1);
	}

	std::cout << "Launching threads ... " << std::endl;
	// Create and launch the threads 
	boost::thread_group tg;
	tg.create_thread(boost::bind(&Controller::threadCallback, &ctrl));
	/* ADD HERE ANY OTHER THREADS THAT YOU MAY NEED*/
	// tg.create_thread(boost::bind(&CallbackFnc));
	// or
	// tg.create_thread(boost::bind(&ClassName::CallbackFncn, &ClassObject));

	std::cout << "Waiting for the threads to terminate ... " << std::endl;
	// Wait for termination of spawn threads
	tg.join_all();

	// If enabled, log data
	if (LOG_DATA){
		std::cout << "Creating Log Folder... " << std::endl;

		// Create a new folder for the current experiment
		createLogFolder();

		// Log data
		std::cout << "Logging data ... " << std::endl;
		ctrl.logData();

		// Close Logging (update fileSequence)
		closeLog();
	}


	return 0;

}