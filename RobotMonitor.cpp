

#include "RobotMonitor.h"

RobotMonitor::RobotMonitor(ArRobot *r, ArServerHandlerPopup *ps) :
  robot(r),
  popupServer(ps),
  motorsDisabledPopupID(0),
  motorsDisabledPopupInfo( NULL,
    "Robot Motors Are Disabled",
    "The robot's motors are disabled", 
    ArServerHandlerPopup::WARNING,
    1,  // default button ID
    0,  // escape button ID
    -1, // timeout
    NULL, // timeout message
    "Enable Motors", "Enabling Motors...",
    "Ignore", "Ignore"
  ),
  handleMotorsDisabledPopupResponseCB(this, &RobotMonitor::handleMotorsDisabledResponse),
  robotMonitorCB(this, &RobotMonitor::robotMonitorTask)
{
  robot->addUserTask("arnlServerRobotMonitor", 30, &robotMonitorCB);
}

RobotMonitor::~RobotMonitor()
{
  robot->remUserTask(&robotMonitorCB);
}

void RobotMonitor::handleMotorsDisabledResponse(ArTypes::Byte4 popupID, int button)
{
  if(button == 0)
  {
    ArLog::log(ArLog::Normal, "Enabling motors...");
    robot->enableMotors();
  }
  popupServer->closePopup(motorsDisabledPopupID, "Closing motor disable popup.");
  motorsDisabledPopupID = 0;
}


// This function is called as a robot task (every 100ms) to check on the robot
// state and perform feedback and interact with user as needed.
void RobotMonitor::robotMonitorTask()
{

  // a way for user to re-enable motors if disabled -- show a popup dialog in
  // MobileEyes.
  if(motorsDisabledPopupID == 0 && robot && !robot->areMotorsEnabled() && robot->isConnected())
  {
    motorsDisabledPopupID = popupServer->createPopup(&motorsDisabledPopupInfo, &handleMotorsDisabledPopupResponseCB);
  }

  // Set LX wheel light pattern based on robot activity. You could add more
  // conditions/light patterns here if you want.
  if(robot->isEStopPressed())
    robot->comDataN(ArCommands::WHEEL_LIGHT, "\x02\0\0\0", 4); // pattern #2, flash red
  else if(!robot->areMotorsEnabled())
    robot->comDataN(ArCommands::WHEEL_LIGHT, "\x03\0\0\0", 4); // pattern #3, flash yellow
  else if(fabs(robot->getVel()) < 5)
    robot->comDataN(ArCommands::WHEEL_LIGHT, "\x0A\0\0\0", 4);  // pattern #10, slow blue flash
  else
    robot->comDataN(ArCommands::WHEEL_LIGHT, "\x09\0\0\0", 4);  // pattern 9, blue sweep.
  
}

