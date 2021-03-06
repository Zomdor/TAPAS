/*
	Copyright (c) 2014,	TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
	THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef POSITIONESTIMATION_H_
#define POSITIONESTIMATION_H_

#include <opencv2/opencv.hpp>
#include "Encoders/Encoders.h"
#include "GPS/GPS.h"
#include "IMU/IMU.h"
#include "Util/ExtendedKalmanFilter.h"
#include <thread>
#include <fstream>

//TinyXML
#include <tinyxml.h>

#include <chrono>
#include <iostream>
#include <mutex>

class Robot;
class Debug;


class PositionEstimation {
	friend class Debug;

private:
	struct Parameters {
		int runThread;
		double processingFrequency;
		int debug;
		int encoderTicksPerRev;
		double wheelDiameter;
		double wheelBase;
		double predictionVariance;
		double gpsVariance;
		double imuVariance;
		double encoderVariance;
		double startLatitude, startLongitude, goalLatitude, goalLongitude;
	};

	Parameters parameters;

	// unique pointer to the PositionEstimation thread
	std::thread estimationThread;
	std::mutex positionEstimationMtx;


	// Kalman filter to gather position information
	ExtendedKalmanFilter *EKF;
	std::chrono::high_resolution_clock::time_point lastUpdateTimestamp,
			lastEncoderTimestamp, lastGpsTimestamp, lastImuTimestamp;
	cv::Mat state;

	// GPS
	GPS gps;

	// IMU
	IMU imu;

	// Encoders
	int lastLeft, lastRight;
	float imuZeroAngle;
	bool encoderStart, imuStart;
	Encoders encoders;

	//Parent class Robot
	Robot* robot;

	// Logger
	std::ofstream logStream, logGpxStream;

	// The cycle of the position estimation thread
	void run();

public:

	PositionEstimation(Robot* irobot, TiXmlElement* settings);
	virtual ~PositionEstimation();

	void readSettings(TiXmlElement* settings);

	// Stopping the position estimation thread
	void stopThread();

	// Saving current estimate to file
	void saveTrajectoryToFile();

	// Initializing the filter
	void kalmanSetup();

	// Update Kalman - updates on GPS
	void KalmanLoop();

	// Encoders - predict
	void KalmanPredict();

	// Zeroes the current position estimate
	void setZeroPosition();
	bool isSetZero();

	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32SC1 2x1: left, right encoder
	cv::Mat getEncoderData(
			std::chrono::high_resolution_clock::time_point &timestamp);

	//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
	cv::Mat getImuData(std::chrono::high_resolution_clock::time_point &timestamp);

	float getImuAccVariance();

	//----------------------ACCESS TO COMPUTED DATA
	//CV_32SC1 3x1: x, y, fi
	const cv::Mat getEstimatedPosition();

	//----------------------MENAGMENT OF PositionEstimation DEVICES
	//Gps
	void openGps(std::string port);

	void closeGps();

	bool isGpsOpen();

	int gpsGetFixStatus();

	bool isGpsDataValid();

	double getPosX(double longitude);
	double getPosLongitude(double X);

	double getPosY(double latitude);
	double getPosLatitude(double Y);

	void fakeGPSStart(double lat, double lon);

	//Imu
	void openImu(std::string port);
	void closeImu();
	bool isImuOpen();
	bool isImuDataValid();

	//Encoders
	void openEncoders(std::string port);

	void closeEncoders();

	bool isEncodersOpen();
};

#include "../Robot/Robot.h"

#endif /* POSITIONESTIMATION_H_ */
