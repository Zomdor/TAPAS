/*
 * PositionEstimation.h
 *
 * In this class, we integrate data from different sensors using Kalman Filter
 *
 * TO THINK: - find Kalman implementation and maybe put it in another class
 */

#ifndef POSITIONESTIMATION_H_
#define POSITIONESTIMATION_H_

#include <opencv2/opencv.hpp>
#include "Encoders/Encoders.h"
#include "GPS/GPS.h"
#include "IMU/IMU.h"

class Robot;

class PositionEstimation {

private:
	// Encoders - moved to GlobalPlanner
	//Encoders encoders;

	// GPS
	GPS gps;

	// IMU
	IMU imu;

	//Parent class Robot
	Robot* robot;

public:
	PositionEstimation(Robot* irobot);
	virtual ~PositionEstimation();

	// Update Kalman - updates on GPS
	void KalmanUpdate();

	// Encoders - predict
	void KalmanPredict();

	//----------------------ACCESS TO COMPUTED DATA
	//CV_32SC1 3x1: x, y, fi
	cv::Mat getEstimatedPosition();

	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32SC1 2x1: x, y position
	cv::Mat getGpsData();

	//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
	cv::Mat getImuData();

	//----------------------MENAGMENT OF PositionEstimation DEVICES
	//Gps
	void openGps(std::string port);

	void closeGps();

	bool isGpsOpen();

	//Imu
	void openImu(std::string port);

	void closeImu();

	bool isImuOpen();
};

#include "../Robot/Robot.h"

#endif /* POSITIONESTIMATION_H_ */
