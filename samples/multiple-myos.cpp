// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.

// This sample illustrates how to interface with multiple Myo armbands and distinguish between them.

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <array>
#include <sstream>
#include <ctime>
#include <time.h>
#include <fstream>

#include <myo/myo.hpp>
#include <dtw.h>
class PrintMyoEvents : public myo::DeviceListener {
public:
	PrintMyoEvents()
	{
		openFiles();
	}
	void openFiles() {
		time_t timestamp = std::time(0);

		for (int i = 0; i < 2; i++) {
			// Open file for EMG log
			if (emgFiles[i].is_open()) {
				emgFiles[i].close();
			}
			std::ostringstream emgFileString;
			emgFileString << "data/emg-" << i << "-" << timestamp << ".csv";
			emgFiles[i].open(emgFileString.str(), std::ios::out);
			emgFiles[i] << "timestamp,emg1,emg2,emg3,emg4,emg5,emg6,emg7,emg8" << std::endl;

			// Open file for gyroscope log
			if (gyroFiles[i].is_open()) {
				gyroFiles[i].close();
			}
			std::ostringstream gyroFileString;
			gyroFileString << "data/gyro-" << i << "-" << timestamp << ".csv";
			gyroFiles[i].open(gyroFileString.str(), std::ios::out);
			gyroFiles[i] << "timestamp,x,y,z" << std::endl;

			// Open file for accelerometer log
			if (accelerometerFiles[i].is_open()) {
				accelerometerFiles[i].close();
			}
			std::ostringstream accelerometerFileString;
			accelerometerFileString << "data/accelerometer-" << i << "-" << timestamp << ".csv";
			accelerometerFiles[i].open(accelerometerFileString.str(), std::ios::out);
			accelerometerFiles[i] << "timestamp,x,y,z" << std::endl;

			// Open file for orientation log
			if (orientationFiles[i].is_open()) {
				orientationFiles[i].close();
			}
			std::ostringstream orientationFileString;
			orientationFileString << "data/orientation-" << i << "-" << timestamp << ".csv";
			orientationFiles[i].open(orientationFileString.str(), std::ios::out);
			orientationFiles[i] << "timestamp,x,y,z,w" << std::endl;

			// Open file for orientation (Euler angles) log
			if (orientationEulerFiles[i].is_open()) {
				orientationEulerFiles[i].close();
			}
			std::ostringstream orientationEulerFileString;
			orientationEulerFileString << "data/orientationEuler-" << i << "-" << timestamp << ".csv";
			orientationEulerFiles[i].open(orientationEulerFileString.str(), std::ios::out);
			orientationEulerFiles[i] << "timestamp,roll,pitch,yaw" << std::endl;
		}
		

	}
		
    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
    {
        
        knownMyos.push_back(myo);
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
        
        std::cout << "Paired with " << identifyMyo(myo) << "." << std::endl;
    }

    void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
    {
        std::cout << "Myo " << identifyMyo(myo) << " has connected." << std::endl;
    }

    void onDisconnect(myo::Myo* myo, uint64_t timestamp)
    {
        std::cout << "Myo " << identifyMyo(myo) << " has disconnected." << std::endl;
    }

	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
	{
		int m = identifyMyo(myo);
		emgFiles[m] << timestamp;
		for (size_t i = 0; i < 8; i++) {
			emgFiles[m] << ',' << static_cast<int>(emg[i]);

		}
		emgFiles[m] << std::endl;
	}

	// onOrientationData is called whenever new orientation data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onOrientationData(myo::Myo *myo, uint64_t timestamp, const myo::Quaternion< float > &rotation) {
		int m = identifyMyo(myo);
		orientationFiles[m] << timestamp
			<< ',' << rotation.x()
			<< ',' << rotation.y()
			<< ',' << rotation.z()
			<< ',' << rotation.w()
			<< std::endl;

		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;

		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		float roll = atan2(2.0f * (rotation.w() * rotation.x() + rotation.y() * rotation.z()),
			1.0f - 2.0f * (rotation.x() * rotation.x() + rotation.y() * rotation.y()));
		float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (rotation.w() * rotation.y() - rotation.z() * rotation.x()))));
		float yaw = atan2(2.0f * (rotation.w() * rotation.z() + rotation.x() * rotation.y()),
			1.0f - 2.0f * (rotation.y() * rotation.y() + rotation.z() * rotation.z()));

		orientationEulerFiles[m] << timestamp
			<< ',' << roll
			<< ',' << pitch
			<< ',' << yaw
			<< std::endl;
	}

	// onAccelerometerData is called whenever new acceleromenter data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onAccelerometerData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &accel) {
		int m = identifyMyo(myo);
		printVector(accelerometerFiles[m], timestamp, accel);

	}

	// onGyroscopeData is called whenever new gyroscope data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onGyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &gyro) {
		int m = identifyMyo(myo);
		printVector(gyroFiles[m], timestamp, gyro);

	}

	// Helper to print out accelerometer and gyroscope vectors
	void printVector(std::ofstream &file, uint64_t timestamp, const myo::Vector3< float > &vector) {
		file << timestamp
			<< ',' << vector.x()
			<< ',' << vector.y()
			<< ',' << vector.z()
			<< std::endl;
	}
    // This is a utility function implemented for this sample that maps a myo::Myo* to a unique ID starting at 1.
    // It does so by looking for the Myo pointer in knownMyos, which onPair() adds each Myo into as it is paired.
    size_t identifyMyo(myo::Myo* myo) {
        // Walk through the list of Myo devices that we've seen pairing events for.
        for (size_t i = 0; i < knownMyos.size(); ++i) {
            // If two Myo pointers compare equal, they refer to the same Myo device.
            if (knownMyos[i] == myo) {
                return i;
            }
        }

        return 0;
    }
	
    // We store each Myo pointer that we pair with in this list, so that we can keep track of the order we've seen
    // each Myo and give it a unique short identifier (see onPair() and identifyMyo() above).
    std::vector<myo::Myo*> knownMyos;
	std::array<std::ofstream, 2> emgFiles;
	std::array<std::ofstream, 2> gyroFiles;
	std::array<std::ofstream, 2> orientationFiles;
	std::array<std::ofstream, 2> orientationEulerFiles;
	std::array<std::ofstream, 2> accelerometerFiles;
};

double euclidean_distance(std::vector<double> P1, std::vector<double> P2)
{
	double total = 0.0;
	for (unsigned int i = 0; i < P1.size(); i++)
	{
		total = total + pow((P1[i] - P2[i]), 2);
	}
	return sqrt(total);
}

int compare_performance(int traj_length, int iterations)
{
	struct timespec bstv, betv;
	printf("Building test arrays\n");
	std::vector< std::vector<double> > test_vec_1;
	for (int i = 0; i < traj_length; i++)
	{
		std::vector<double> state;
		state.push_back(0.0);
		state.push_back(0.0);
		state.push_back(0.0);
		test_vec_1.push_back(state);
	}
	std::vector< std::vector< std::vector<double> > > test_vec_2;
	for (int i = 0; i < iterations; i++)
	{
		std::vector< std::vector<double> > traj;
		for (int j = 0; j < traj_length; j++)
		{
			std::vector<double> state2;
			state2.push_back((double)rand());
			state2.push_back((double)rand());
			state2.push_back((double)rand());
			traj.push_back(state2);
		}
		test_vec_2.push_back(traj);
	}
	DTW::SimpleDTW my_eval = DTW::SimpleDTW(traj_length, traj_length, euclidean_distance);
	printf("Evaluating\n");
	//Run tests
	printf("-----Test single-threaded version-----\n");
	printf("Testing vector variant\n");
	clock_t a;
	a = clock();
	//clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bstv);
	double scost = 0.0;
	for (int i = 0; i < iterations; i++)
	{
		scost = my_eval.EvaluateWarpingCost(test_vec_1, test_vec_2[i]);
	}
	//clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &betv);
	clock_t b;
	b = clock();
	//---------------------------------------------
	//-----Compute runtimes (single-threaded)--------;
	float bsecsv = (float)(betv.tv_sec - bstv.tv_sec);
	bsecsv = bsecsv + (float)(betv.tv_nsec - bstv.tv_nsec) / 1000000000.0;
	printf("Final cost: %f\n", scost);
	printf("SINGLE (vector): %f\n", bsecsv);
	return 0;
}
/*
int main()
{
	myo::Hub hub("com.example.multiple-myos");
	PrintMyoEvents printer;
	std::cout << "Running performance tests..." << std::endl;
	compare_performance(1000, 1000);
	std::cout << "...done performance testing" << std::endl;
	return 0;
}
*/
int main(int argc, char** argv)
{
    try {
		myo::Hub hub("com.example.multiple-myos");
		// Instantiate the PrintMyoEvents class we defined above, and attach it as a listener to our Hub.
		PrintMyoEvents printer;
		hub.addListener(&printer);

		while (1) {
			// Process events for 10 milliseconds at a time.
			hub.run(1);
		}


    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
