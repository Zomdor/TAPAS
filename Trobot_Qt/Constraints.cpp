/*
 * Constraints.cpp
 *
 *  Created on: 09-07-2014
 *      Author: jachu
 */

#include "Constraints.h"
//Qt
#include <QtGui/QPainter>
#include <QtGui/QColor>
#include <QtGui/QPixmap>
//OpenCV
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Constraints::Constraints(Ui::TrobotQtClass* iui, Debug* idebug) :
	ui(iui),
	debug(idebug)
{
	QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(updateViews()));

	viewer = new Viewer();
	ui->constraintMapViewScrollArea->setWidget(viewer);

	//timer.setInterval(100);
	timer.start(100);
}

Constraints::~Constraints(){

}

void Constraints::updateCameraView(){
	//cout << "Updating camera view" << endl;
	Mat image;
	vector<Point2f> pointCloudCamera = debug->getPointCloudCamera(image);
	//cout << "Got point cloud camera" << endl;
	QPixmap map;
	double scaleX = 640 / ui->constraintCameraViewLabel->width();
	double scaleY = 480 / ui->constraintCameraViewLabel->height();

	if(!image.empty()){
		cvtColor(image, image, CV_BGR2RGB);
		Mat resizedImage;
		cv::resize(image, resizedImage, cv::Size(ui->constraintCameraViewLabel->width(), ui->constraintCameraViewLabel->height()));
		scaleX = (double)image.cols / ui->constraintCameraViewLabel->width();
		scaleY = (double)image.rows / ui->constraintCameraViewLabel->height();
		map = QPixmap::fromImage(QImage(resizedImage.ptr(), resizedImage.cols, resizedImage.rows, QImage::Format_RGB888));
	}
	else{
		map = QPixmap(ui->constraintCameraViewLabel->width(), ui->constraintCameraViewLabel->height());
		map.fill(Qt::white);
	}

	QPainter painter(&map);
	painter.setPen(Qt::red);
	for(int p = 0; p < pointCloudCamera.size(); p++){
		if(pointCloudCamera[p].x >= 0 && pointCloudCamera[p].x < image.cols &&
				pointCloudCamera[p].y >= 0 && pointCloudCamera[p].x < image.rows)
		{
			painter.drawPoint(pointCloudCamera[p].x / scaleX, pointCloudCamera[p].y / scaleY);
		}
	}
	painter.end();

	ui->constraintCameraViewLabel->setPixmap(map);
	ui->constraintCameraViewLabel->update();
}

void Constraints::updateMapView(){
	//cout << "updateMapView()" << endl;
	Mat curPosImuMapCenter;
	Mat pointCloudImuMapCenter = debug->getPointCloudImu(curPosImuMapCenter);

	//cout << "curPosImuMapCenter.size() = " << curPosImuMapCenter.size() << endl;
	if(!curPosImuMapCenter.empty() && !pointCloudImuMapCenter.empty()){
		viewer->updatePointCloud(pointCloudImuMapCenter);
		viewer->updateRobotPos(curPosImuMapCenter);
		viewer->rysuj();
	}
}

void Constraints::updateViews(){
	//cout << "Updating views" << endl;
	if(ui->constraintCameraViewLabel->isVisible()){
		updateCameraView();
	}
	if(ui->constraintMapViewScrollArea->isVisible()){
		updateMapView();
	}
}