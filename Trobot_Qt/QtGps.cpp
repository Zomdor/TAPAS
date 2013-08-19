/*
 * QtGps.cpp
 *
 *  Created on: 15-07-2013
 *      Author: jachu
 */

//OpenCV
#include <opencv2/opencv.hpp>
//TrobotQt
#include "QtGps.h"

using namespace std;
using namespace cv;

QtGps::QtGps(Ui::TrobotQtClass* iui, Robot* irobot) : ui(iui), robot(irobot) {
	QObject::connect(ui->gpsConnectButton, SIGNAL(clicked()), this, SLOT(connect()));
	QObject::connect(ui->gpsDisconnectButton, SIGNAL(clicked()), this, SLOT(disconnect()));
	QObject::connect(&(this->refreshTimer), SIGNAL(timeout()), this, SLOT(refresh()));
	refreshTimer.setInterval(500);
}

QtGps::~QtGps(){
	disconnect();
}

void QtGps::connect(){
	if(ui->gpsPortCombo->count() != 0){
		robot->openGps(ui->gpsPortCombo->currentText().toAscii().data());
		refreshTimer.start();
	}
}

void QtGps::disconnect(){
	refreshTimer.stop();
	robot->closeGps();
}

void QtGps::refresh(){
	Mat tmp = robot->getGpsData();
	ui->gpsTimeLabel->setText(QString("%1").arg(tmp.at<float>(0)));
}

