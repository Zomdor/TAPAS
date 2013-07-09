#ifndef QT_ROBOT_DRIVE
#define QT_ROBOT_DRIVE

//#define DRIVE_DBG

#include <QtCore/QObject>
#include <string>
#include "../Robot/Robot.h"
#include "ui_trobotqt.h"

enum Action {
	Nothing,
	UserDefined,
	Forward,
	Backward,
	Left,
	Right
};

class QtRobotDrive : public QObject
{

	Q_OBJECT

public:
	QtRobotDrive(Robot* irobot,  Ui::TrobotQtClass* iui);
	~QtRobotDrive();
	Action getState();
	bool isOpen();
private:
	Ui::TrobotQtClass* ui;
	void setButtonsEnabled(bool state);

	Action driveState;
	const int speed;
	int motorVal[2];
	Robot* robot;
public slots:
	void goForward();
	void goBackward();
	void goLeft();
	void goRight();
	void leftMotorStop();
	void rightMotorStop();
	void stop();
	void motorValChanged(int val);
	void openRobotDrive();
	void closeRobotDrive();
	void searchRobotDrive();
};

#endif //QT_ROBOT_DRIVE