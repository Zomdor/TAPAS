/*
 * debug.cpp
 *
 *  Created on: 04-10-2013
 *      Author: jachu
 */

#include "Debug.h"
//STL
#include <chrono>
//OpenCV
#include <opencv2/opencv.hpp>
//Boost
#include <boost/filesystem.hpp>
//RobotsIntellect
#include "../Robot/Robot.h"
#include "../MovementConstraints/MovementConstraints.h"
#include "../MovementConstraints/Camera/Camera.h"
#include "../MovementConstraints/Hokuyo/Hokuyo.h"
#include "../Planning/GlobalPlanner.h"
#include "../PositionEstimation/PositionEstimation.h"
#include "../PositionEstimation/GPS/GPS.h"
#include "../PositionEstimation/IMU/IMU.h"

//ROS_GPS service
#include "TAPAS/GPSsetZeroXY.h"

using namespace cv;
using namespace std;
using namespace boost;

Debug::Debug(Robot* irobot) : robot(irobot), it(nh), hokuyoActive(false){
	cout << "Debug::Debug" << endl;
	image_sub = it.subscribe("camera_image", 10, &Debug::imageCallback, this);
	// classified_sub = it.subscribe("camera_classified", 10, &Debug::classifiedCallback, this);
	coords_sub = nh.subscribe("camera_coords", 10, &Debug::coordsCallback, this);
	colors_sub = nh.subscribe("camera_colors", 10, &Debug::colorsCallback, this);
	hokuyo_sub = nh.subscribe("hokuyo_data", 10, &Debug::hokuyoCallback, this);
	constraints_sub = nh.subscribe("movement_constraints", 10, &Debug::constraintsCallback, this);
}

//----------------------MODES OF OPERATION
void Debug::switchMode(OperationMode mode){
	robot->globalPlanner->switchMode(mode);
}

void Debug::setMotorsVel(float motLeft, float motRight){
	robot->globalPlanner->setMotorsVel(motLeft, motRight);
}

bool Debug::isHokuyoActive() {
	return hokuyoActive;
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32SC1 2x1: left, right encoder
const cv::Mat Debug::getEncoderData(){
	std::chrono::high_resolution_clock::time_point timestamp;
	return robot->positionEstimation->getEncoderData(timestamp);
}

//CV_32SC1 4x1: x, y, lat, lon position
const cv::Mat Debug::getGpsData(){
	Mat ret(4, 1, CV_32FC1);
	ret.at<float>(0) = robot->positionEstimation->varGPSgetPosX;
	ret.at<float>(1) = robot->positionEstimation->varGPSgetPosY;
	ret.at<float>(2) = robot->positionEstimation->varGPSgetLat;
	ret.at<float>(3) = robot->positionEstimation->varGPSgetLon;
	return ret;
}

//1 - no fix, 2 - 2D, 3 - 3D
int Debug::getGpsFixStatus(){
	return robot->positionEstimation->varGPSgetFixStatus;
}

int Debug::getGpsSatelitesUsed(){
	return robot->positionEstimation->varGPSgetSatelitesUsed;
}

void Debug::setGpsZeroPoint(double lat, double lon){
	TAPAS::GPSsetZeroXY srv;
	srv.request.Lat = robot->positionEstimation->varGPSgetLat;
	srv.request.Lon = robot->positionEstimation->varGPSgetLon;
	robot->positionEstimation->clientSetZeroXY.call(srv);
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
const cv::Mat Debug::getImuData(){
	std::chrono::high_resolution_clock::time_point timestamp;
	return robot->positionEstimation->imu.getData(timestamp);
}

//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
const cv::Mat Debug::getHokuyoData(){
	return hokuyoData;
}

void Debug::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cameraImage = RosHelpers::readImageMsg(msg);
}

void Debug::classifiedCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	classifiedImage = cv_ptr->image;
}

void Debug::coordsCallback(const TAPAS::Matrix msg) {
	pixelCoords = RosHelpers::readMatrixMsg(msg);
}

void Debug::colorsCallback(const TAPAS::Matrix msg) {
	pixelColors = RosHelpers::readMatrixMsg(msg);
}

void Debug::hokuyoCallback(const TAPAS::Matrix msg) {
	hokuyoActive = true;
	hokuyoData = RosHelpers::readMatrixMsg(msg);
}

void Debug::constraintsCallback(const TAPAS::Matrix msg) {
	movementConstraints = RosHelpers::readMatrixMsg(msg);
}

//CV_8UC3 2x640x480: left, right image
const std::vector<cv::Mat> Debug::getCameraData(){
	return vector<Mat>(1, cameraImage);
}

Mat Debug::colorImage(Mat image) {
	ros::NodeHandle nh;

	ros::ServiceClient segmentClient = nh.serviceClient<TAPAS::SegmentImage>("segment_image");
	TAPAS::SegmentImage segmentSrv;
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr->image = image;
	cv_ptr->toImageMsg(segmentSrv.request.image);
	segmentSrv.request.kCurSegment = -1;
	segmentClient.call(segmentSrv);
	
	ros::ServiceClient colorClient = nh.serviceClient<TAPAS::ColorSegments>("color_segments");
	TAPAS::ColorSegments colorSrv;
	colorSrv.request.segmentsRows = segmentSrv.response.segmentsRows;
	colorSrv.request.segmentsCols = segmentSrv.response.segmentsCols;
	colorSrv.request.segments = segmentSrv.response.segments;
	colorClient.call(colorSrv);
	cv_ptr = cv_bridge::toCvCopy(colorSrv.response.image);
	Mat colored = cv_ptr->image;

	return colored;
}

void Debug::testSegmentation(boost::filesystem::path dir){
	filesystem::directory_iterator endIt;
	namedWindow("segmented");
	namedWindow("original");
	for(filesystem::directory_iterator dirIt(dir); dirIt != endIt; dirIt++){
		cout << dirIt->path().string() << endl;
		if(dirIt->path().filename().string().find(".jpg") != string::npos){
			cout << "Processing image " << dirIt->path().string() << endl;
			Mat image = imread(dirIt->path().string());
			Mat colored = colorImage(image);

			imshow("original", image);
			imshow("segmented", colored);
			waitKey();
		}
	}
}

void Debug::testEncoders(){
	robot->openEncoders("/dev/robots/encoders");
	while(true){
		std::chrono::high_resolution_clock::time_point timestamp;
		cout << robot->getEncoderData(timestamp) << endl;
		char a = waitKey(500);
		if(a == 'q'){
			break;
		}
	}
}

void Debug::testDrivers(){
	robot->openRobotsDrive("/dev/robots/driver1", "/dev/robots/driver2");

	while(true){

		robot->globalPlanner->setMotorsVel(1000, 1000);
		char a = waitKey(500);
		if(a == 'q'){
			break;
		}
		robot->globalPlanner->setMotorsVel(0, 0);
		}
	}


//----------------------ACCESS TO COMPUTED DATA
//CV_32SC1 3x1: x, y, fi
const cv::Mat Debug::getEstimatedPosition(){
	return robot->positionEstimation->getEstimatedPosition();
}

//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
const cv::Mat Debug::getMovementConstraints(){
	return movementConstraints;
}

std::vector<cv::Point2f> Debug::getPointCloudCamera(cv::Mat& image){
	Mat curPosMapCenter;
	Mat pointCloudCameraMapCenter = robot->movementConstraints->getPointCloud(curPosMapCenter);
	//cout << "Got point cloud" << endl;
	image = cameraImage;

	vector<Point2f> pointsImage;
	//cout << curPosMapCenter.size() << ", " << pointCloudCameraMapCenter.size() << endl;
	if(!curPosMapCenter.empty() && !pointCloudCameraMapCenter.empty()){
		Mat cameraOrigImu = robot->movementConstraints->cameraOrigImu;
		Mat allPointsCamera = (curPosMapCenter*cameraOrigImu).inv()*pointCloudCameraMapCenter.rowRange(0, 4);
		//cout << "Computing point projection" << endl;

		projectPoints(	allPointsCamera.rowRange(0, 3).t(),
						Matx<float, 3, 1>(0, 0, 0),
						Matx<float, 3, 1>(0, 0, 0),
						RosHelpers::readMatrixParam(nh, "cameraMatrix", 3, 3),
						RosHelpers::readMatrixParam(nh, "distCoeffs", 1, 5),
						pointsImage);
	}
	return pointsImage;
}


cv::Mat Debug::getPointCloudImu(cv::Mat& curImuOrigMapCenter, cv::Mat& curMapCenterOrigGlobal){
	Mat pointCloudOrigMapCenter = robot->movementConstraints->getPointCloud(curImuOrigMapCenter);
	curMapCenterOrigGlobal = robot->movementConstraints->getCurMapCenterOrigGlobal();
	return pointCloudOrigMapCenter;
}

cv::Mat Debug::getClassifiedImage(){
	return classifiedImage;
}

GlobalPlanner::GlobalPlanInfo Debug::getGlobalPlan(){
	return robot->globalPlanner->getGlobalPlan();
}

void Debug::getVecFieldHist(std::vector<float>& retVecFieldHist,
		float& retGoalDirection,
		float& retBestDirection)
{
	robot->globalPlanner->localPlanner->getVecFieldHist(retVecFieldHist, retGoalDirection, retBestDirection);
}

float Debug::getHeadingToGoal(){
	return robot->globalPlanner->getHeadingToGoal();
}

void Debug::getPixelPointCloud(cv::Mat& retPixelCoords,
							cv::Mat& retPixelColors)
{
	retPixelCoords = pixelCoords;
	retPixelColors = pixelColors;
}

float Debug::getImuAccVariance(){
	return robot->getImuAccVariance();
}

void Debug::getTransformationMatrices(cv::Mat& retImuOrigRobot,
									cv::Mat& retCameraOrigLaser,
									cv::Mat& retCameraOrigImu)
{
	robot->movementConstraints->imuOrigRobot.copyTo(retImuOrigRobot);
	robot->movementConstraints->cameraOrigLaser.copyTo(retCameraOrigLaser);
	robot->movementConstraints->cameraOrigImu.copyTo(retCameraOrigImu);
}
