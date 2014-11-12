#ifndef _ROSARNL_ROBOTMONITOR_H_
#define _ROSARNL_ROBOTMONITOR_H_ 

#include "Aria/Aria.h"
#include "ArNetworking/ArNetworking.h"

class RobotMonitor {
protected:
  ArRobot *robot;
  ArServerHandlerPopup *popupServer;
  ArTypes::Byte4 motorsDisabledPopupID;
  ArServerHandlerPopupInfo motorsDisabledPopupInfo;
  ArFunctor2C<RobotMonitor, ArTypes::Byte4, int> handleMotorsDisabledPopupResponseCB;
  ArFunctorC<RobotMonitor> robotMonitorCB;

public:
  RobotMonitor(ArRobot *r, ArServerHandlerPopup *ps);
  ~RobotMonitor();

protected:
  void handleMotorsDisabledResponse(ArTypes::Byte4 popupID, int button);

  // This function is called as a robot task (every 100ms) to check on the robot
  // state and perform feedback and interact with user as needed.
  void robotMonitorTask();
};

#endif
