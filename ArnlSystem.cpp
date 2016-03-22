
#include <assert.h>

#include "ArnlSystem.h"

#include "Aria/Aria.h"
#include "ArNetworking/ArNetworking.h"
#include "Arnl.h"
#include "ArPathPlanningInterface.h"
#include "ArLocalizationTask.h"
#include "ArDocking.h"
#include "Aria/ArSystemStatus.h"
#include "ArNetworking/ArServerModeJogPosition.h"
#include "ArNetworking/ArServerModeRatioDrive.h"

#include "RobotMonitor.h"




ArnlSystem::ArnlSystem(const char *_logprefix) : 
    robot(0),
    pathTask(0),
    locTask(0),
    modeStop(0),
    modeGoto(0),
    modeWander(0),
    logprefix(_logprefix)
{
}

ArnlSystem::~ArnlSystem()
{
  // todo delete more objects objects
  if(robot)
  {
    robot->disableMotors();
    robot->disableSonar();
    robot->stopRunning();
    robot->waitForRunExit();
  }
  if(pathTask) delete pathTask;
  if(locTask) delete locTask;
  if(modeStop) delete modeStop;
  if(modeGoto) delete modeGoto;
  if(modeWander) delete modeWander;
  if(robot) delete robot;
}

ArnlSystem::Error ArnlSystem::setup()
{
  // Note, various objects are allocated here which are never deleted (freed),
  // if you want to destroy and create new ArnlSystem objects in the future
  // these need to be stored and deleted in the constructor.  If you use it once
  // and keep it for the lifetime of the process, its ok.

  robot = new ArRobot();
  ArArgumentBuilder *args = new ArArgumentBuilder(); 
  ArArgumentParser *argparser = new ArArgumentParser(args);
  argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)
  argparser->addDefaultArgument("-connectLaser");

  ArRobotConnector *robotConnector = new ArRobotConnector(argparser, robot);

  if(!robotConnector->connectRobot())
  {
    ArLog::log(ArLog::Normal, "%sError: could not connect to robot.", logprefix);
    return RobotConnectError;
  }

  robot->addPacketHandler(new ArRetFunctor1C<bool, ArnlSystem, ArRobotPacket*>(this, &ArnlSystem::handleDebugMessage));

  // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
  //robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);


  char fileDir[1024];
  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(), 
			 "examples");
  
  
  ArLog::addToConfig(Aria::getConfig());

  ArAnalogGyro *gyro = new ArAnalogGyro(robot); // for old robots with separate gyro communication

  serverBase = new ArServerBase;

  ArServerSimpleOpener *simpleOpener = new ArServerSimpleOpener(argparser);

  ArLaserConnector *laserConnector = new ArLaserConnector(argparser, robot, robotConnector);

  if(!Aria::parseArgs() || !argparser->checkHelpAndWarnUnparsed())
  {
    return ParseArgumentsError;
  }
     


  ArSonarDevice *sonarDev = new ArSonarDevice;
  robot->addRangeDevice(sonarDev);

  ArRobotConfig *robotConfig = new ArRobotConfig(robot);

  robotConfig->addAnalogGyro(gyro);

  robot->runAsync(true);

  
  ArLog::log(ArLog::Normal, "%sConnecting to laser(s) configured in parameters...", logprefix);
  if (!laserConnector->connectLasers())
  {
    ArLog::log(ArLog::Terse, "%sError: Could not connect to laser(s). Exiting.", logprefix);
    return LaserConnectError;
  }
  ArLog::log(ArLog::Normal, "%sDone connecting to laser(s).", logprefix);

  robot->lock();
  ArLaser *firstLaser = robot->findLaser(1);
  assert(firstLaser);
  if (firstLaser == NULL || !firstLaser->isConnected())
  {
    ArLog::log(ArLog::Terse, "%sDid not have laser 1 or it is not connected, cannot start localization and/or mapping... exiting", logprefix);
    return LaserConnectError;
  }
  robot->unlock();


  map = new ArMap(fileDir);
  map->setIgnoreEmptyFileName(true);
  map->setIgnoreCase(true);
  //map->setIgnoreBadFile(true);

    
  pathTask = new ArPathPlanningTask (robot, sonarDev, map);

  locTask = new ArLocalizationTask (robot, firstLaser, map);
  creationTime.setToNow();
  

  std::map<int, ArLaser *>::iterator laserIt;
  for (laserIt = robot->getLaserMap()->begin();
       laserIt != robot->getLaserMap()->end();
       laserIt++)
  {
    int laserNum = (*laserIt).first;
    ArLaser *laser = (*laserIt).second;

    if(!laser->isConnected())
      continue;

    laser->setCumulativeBufferSize(200);
    pathTask->addRangeDevice(laser, ArPathPlanningTask::BOTH);
    laser->setCumulativeCleanOffset(laserNum * 100);
    laser->resetLastCumulativeCleanTime();

    std::string laserPacketCountName;
    laserPacketCountName = laser->getName();
    laserPacketCountName += " Packet Count";
    Aria::getInfoGroup()->addStringInt(
	    laserPacketCountName.c_str(), 10, 
	    new ArRetFunctorC<int, ArLaser>(laser, 
					 &ArLaser::getReadingCount));
  }




  if (!simpleOpener->open(serverBase, fileDir, 240))
  {
    ArLog::log(ArLog::Normal, "%sError: Could not open server.", logprefix);
    exit(2);
  }




  ArBumpers *bumpers = new ArBumpers;
  robot->addRangeDevice(bumpers);
  pathTask->addRangeDevice(bumpers, ArPathPlanningTask::CURRENT);

  ArForbiddenRangeDevice *forbidden = new ArForbiddenRangeDevice(map);
  robot->addRangeDevice(forbidden);
  pathTask->addRangeDevice(forbidden, ArPathPlanningTask::CURRENT);

  robot->unlock();


  ArActionSlowDownWhenNotCertain *actionSlowDown = new ArActionSlowDownWhenNotCertain(locTask);
  pathTask->getPathPlanActionGroup()->addAction(actionSlowDown, 140);

  ArActionLost *actionLostPath = new ArActionLost(locTask, pathTask);
  pathTask->getPathPlanActionGroup()->addAction(actionLostPath, 150);

  ArGlobalReplanningRangeDevice *replanDev = new ArGlobalReplanningRangeDevice(pathTask);

  
  drawings = new ArServerInfoDrawings(serverBase);
  drawings->addRobotsRangeDevices(robot);
  drawings->addRangeDevice(replanDev);

  drawings->addDrawing( 
    new ArDrawingData("polyLine", ArColor(200,200,200), 1, 75),
    "Local Plan Area", 
    new ArFunctor2C<ArPathPlanningTask, ArServerClient*, ArNetPacket*>(pathTask, &ArPathPlanningTask::drawSearchRectangle)
  );

  drawings->addDrawing(
    new ArDrawingData("polySegments", ArColor(166, 166, 166), 1, 60, 100, "DefaultOn"),
    "Path Planning Clearances",
    new ArFunctor2C<ArPathPlanningTask, ArServerClient*, ArNetPacket*>(pathTask, &ArPathPlanningTask::drawRobotBounds)
  );

  drawings->addDrawing(
    new ArDrawingData("polyDots", ArColor(0, 255, 0), 100, 75), 
    "Localization Points", 
    new ArFunctor2C<ArLocalizationTask, ArServerClient*, ArNetPacket*>(locTask, &ArLocalizationTask::drawRangePoints)
  );


  ArServerHandlerCommands *commands = new ArServerHandlerCommands(serverBase);


  new ArServerInfoRobot(serverBase, robot);
  new ArServerInfoSensor(serverBase, robot);
  ArServerInfoPath *serverInfoPath = new ArServerInfoPath(serverBase, robot, pathTask);
  serverInfoPath->addSearchRectangleDrawing(drawings);
  serverInfoPath->addControlCommands(commands);

  new ArServerInfoLocalization(serverBase, robot, locTask);
  ArServerHandlerLocalization *serverLocHandler = new ArServerHandlerLocalization(serverBase, robot, locTask);

  new ArServerHandlerMap(serverBase, map); 
  new ArServerSimpleComUC(commands, robot);                  
  new ArServerSimpleComMovementLogging(commands, robot); 
  new ArServerSimpleComLogRobotConfig(commands, robot); 
  new ArServerSimpleServerCommands(commands, serverBase); 
  new ArServerHandlerCommMonitor(serverBase);

  new ArServerHandlerConfig (serverBase, Aria::getConfig(),
				      Arnl::getTypicalDefaultParamFileName(),
				      Aria::getDirectory());


  ArServerHandlerPopup *popupServer = new ArServerHandlerPopup(serverBase);

  new RobotMonitor(robot, popupServer);



  modeGoto = new ArServerModeGoto (serverBase, robot, pathTask, map);


  modeStop = new ArServerModeStop(serverBase, robot);

  new ArSonarAutoDisabler(robot);

  modeRatioDrive = new ArServerModeRatioDrive(serverBase, robot);  


  ArActionLost *actionLostRatioDrive = new ArActionLost(locTask, pathTask, modeRatioDrive);
  modeRatioDrive->getActionGroup()->addAction(actionLostRatioDrive, 110);

  modeRatioDrive->addToConfig(Aria::getConfig(), "Teleop settings");
  modeRatioDrive->addControlCommands(commands);

  //Wander Mode//
  modeWander = new ArServerModeWander(serverBase, robot);
  ArActionLost *actionLostWander = new ArActionLost(locTask,pathTask,modeWander);
  modeWander->getActionGroup()->addAction(actionLostWander, 110);

  //Jog position Mode//
  modeJogPosition = new ArServerModeJogPosition(serverBase, robot, "jogPositionMode", commands);
  modeJogPosition->addToConfig(Aria::getConfig());
  ArActionLost *actionLostJogPosition = new ArActionLost(locTask,pathTask,modeJogPosition);
  modeJogPosition->getActionGroup()->addAction(actionLostJogPosition, 110);

  // Tool to log data periodically to a file
  //ArDataLogger dataLogger(&robot, "datalog.txt");
  //dataLogger.addToConfig(Aria::getConfig()); // make it configurable through ArConfig

  // Automatically add anything from the global info group to the data logger.
  //Aria::getInfoGroup()->addAddStringCallback(dataLogger.getAddStringFunctor());

  ArServerInfoStrings *stringInfo = new ArServerInfoStrings(serverBase);
  Aria::getInfoGroup()->addAddStringCallback(stringInfo->getAddStringFunctor());
  
  Aria::getInfoGroup()->addStringInt(
	  "Motor Packet Count", 10, 
	  new ArConstRetFunctorC<int, ArRobot>(robot, 
					       &ArRobot::getMotorPacCount));

  Aria::getInfoGroup()->addStringDouble(
	  "Laser Localization Score", 8, 
	  new ArRetFunctorC<double, ArLocalizationTask>(
		  locTask, &ArLocalizationTask::getLocalizationScore),
	  "%.03f");
  Aria::getInfoGroup()->addStringInt(
	  "Laser Loc Num Samples", 8, 
	  new ArRetFunctorC<int, ArLocalizationTask>(
		  locTask, &ArLocalizationTask::getCurrentNumSamples),
	  "%4d");


  ArSystemStatus::startPeriodicUpdate(1000); // update every 1 second
  Aria::getInfoGroup()->addStringDouble("CPU Use", 10, ArSystemStatus::getCPUPercentFunctor(), "% 4.0f%%");
  Aria::getInfoGroup()->addStringInt("Wireless Link Quality", 9, ArSystemStatus::getWirelessLinkQualityFunctor(), "%d");
  Aria::getInfoGroup()->addStringInt("Wireless Link Noise", 9, ArSystemStatus::getWirelessLinkNoiseFunctor(), "%d");
  Aria::getInfoGroup()->addStringInt("Wireless Signal", 9, ArSystemStatus::getWirelessLinkSignalFunctor(), "%d");
  


  modeDock = NULL;
  modeDock = ArServerModeDock::createDock(serverBase, robot, locTask, pathTask);
  if (modeDock != NULL)
  {
    modeDock->checkDock();
    modeDock->addAsDefaultMode();
    modeDock->addToConfig(Aria::getConfig());
    modeDock->addControlCommands(commands);
  }



  modeStop->addAsDefaultMode();



  ArServerHandlerMapping *handlerMapping = new ArServerHandlerMapping(serverBase, robot, firstLaser, 
					fileDir, "", true);

  // make laser localization stop while mapping
  handlerMapping->addMappingStartCallback(
	  new ArFunctor1C<ArLocalizationTask, bool>
	  (locTask, &ArLocalizationTask::setIdleFlag, true));

  // and then make it start again when we're doine
  handlerMapping->addMappingEndCallback(
	  new ArFunctor1C<ArLocalizationTask, bool>
	  (locTask, &ArLocalizationTask::setIdleFlag, false));


  // Make it so our "lost" actions don't stop us while mapping
  handlerMapping->addMappingStartCallback(actionLostPath->getDisableCB());
  handlerMapping->addMappingStartCallback(actionLostRatioDrive->getDisableCB());

  // And then let them make us stop as usual when done mapping
  handlerMapping->addMappingEndCallback(actionLostPath->getEnableCB());
  handlerMapping->addMappingEndCallback(actionLostRatioDrive->getEnableCB());

  // don't let forbidden lines show up as obstacles while mapping
  // (they'll just interfere with driving while mapping, and localization is off anyway)
  handlerMapping->addMappingStartCallback(forbidden->getDisableCB());

  // let forbidden lines show up as obstacles again as usual after mapping
  handlerMapping->addMappingEndCallback(forbidden->getEnableCB());


  // create a pose storage class, this will let the program keep track
  // of where the robot is between runs...  after we try and restore
  // from this file it will start saving the robot's pose into the
  // file
  ArPoseStorage *poseStorage = new ArPoseStorage(robot);
  /// if we could restore the pose from then set the sim there (this
  /// won't do anything to the real robot)... if we couldn't restore
  /// the pose then just reset the position of the robot (which again
  /// won't do anything to the real robot)
  if (poseStorage->restorePose("robotPose"))
    serverLocHandler->setSimPose(robot->getPose());
  //else
 //   robot->com(ArCommands::SIM_RESET);



  /* File transfer services: */
  
#ifdef WIN32
  // Not implemented for Windows yet.
  ArLog::log(ArLog::Terse, "%sfile upload/download services are not implemented for Windows; not enabling them.", logprefix);
#else
  new ArServerFileLister (serverBase, fileDir);
  new ArServerFileToClient (serverBase, fileDir);
  new ArServerFileFromClient (serverBase, fileDir, "/tmp");
  new ArServerDeleteFileOnServer (serverBase, fileDir);
#endif

  
  // When parsing the configuration file, also look at the program's command line options 
  // from the command-line argument parser as well as the configuration file.
  // (So you can use any argument on the command line, namely -map.) 
  Aria::getConfig()->useArgumentParser(argparser);

  // Read in parameter files.
  ArLog::log(ArLog::Normal,  "%sLoading config file %s%s into ArConfig...", logprefix,  Aria::getDirectory(), Arnl::getTypicalParamFileName());
  if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    ArLog::log(ArLog::Normal,  "%sCould not load ARNL configuration file. Set ARNL environment variable to use non-default installation directory.", logprefix);
    return ConfigError;
  }

  if (!simpleOpener->checkAndLog() || !argparser->checkHelpAndWarnUnparsed())
  {
    return ParseArgumentsError;
  }

  // Warn if there is no map
  if (map->getFileName() == NULL || strlen(map->getFileName()) <= 0)
  {
    ArLog::log(ArLog::Terse,  "%s### No map file is set up, you can make a map with the following procedure", logprefix);
    ArLog::log(ArLog::Terse,  "%s   0) You can find this information in README.txt or docs/Mapping.txt", logprefix);
    ArLog::log(ArLog::Terse,  "%s   1) Connect to this server with MobileEyes", logprefix);
    ArLog::log(ArLog::Terse,  "%s   2) Go to Tools->Map Creation->Start Scan", logprefix);
    ArLog::log(ArLog::Terse,  "%s   3) Give the map a name and hit okay", logprefix);
    ArLog::log(ArLog::Terse,  "%s   4) Drive the robot around your space (see docs/Mapping.txt", logprefix);
    ArLog::log(ArLog::Terse,  "%s   5) Go to Tools->Map Creation->Stop Scan", logprefix);
    ArLog::log(ArLog::Terse,  "%s   6) Start up Mapper3", logprefix);
    ArLog::log(ArLog::Terse,  "%s   7) Go to File->Open on Robot", logprefix);
    ArLog::log(ArLog::Terse,  "%s   8) Select the .2d you created", logprefix);
    ArLog::log(ArLog::Terse,  "%s   9) Create a .map", logprefix);
    ArLog::log(ArLog::Terse,  "%s  10) Go to File->Save on Robot", logprefix);
    ArLog::log(ArLog::Terse,  "%s  11) In MobileEyes, go to Tools->Robot Config", logprefix);
    ArLog::log(ArLog::Terse,  "%s  12) Choose the Files section", logprefix);
    ArLog::log(ArLog::Terse,  "%s  13) Enter the path and name of your new .map file for the value of the Map parameter.", logprefix);
    ArLog::log(ArLog::Terse,  "%s  14) Press OK and your new map should become the map used", logprefix);
  }

  // Print a log message notifying user of the directory for map files
  ArLog::log(ArLog::Normal,  "%sDirectory for maps and file serving: %s", logprefix,  fileDir);
  
  ArLog::log(ArLog::Normal,  "%sSee the ARNL README.txt for more information", logprefix);

  // Do an initial localization of the robot-> ARNL and SONARNL try all the home points
  // in the map, as well as the robot's current odometric position, as possible
  // places the robot is likely to be at startup.   If successful, it will
  // also save the position it found to be the best localized position as the
  // "Home" position, which can be obtained from the localization task (and is
  // used by the "Go to home" network request).
  // MOGS instead just initializes at the current GPS position.
  // (You will stil have to drive the robot so it can determine the robot's
  // heading, however. See GPS Mapping instructions.)
  locTask->localizeRobotAtHomeBlocking();
  
  // Start the networking server's thread
  serverBase->runAsync();


  ArLog::log(ArLog::Normal, "%sARNL server is now running. You may connect with MobileEyes and Mapper3.", logprefix);

  robot->lock();
  robot->enableMotors();
  robot->disableSonar();
  robot->unlock();


  return OK;
}



/// Log messages from robot controller
bool ArnlSystem::handleDebugMessage(ArRobotPacket *pkt)
{
  if(pkt->getID() != ArCommands::MARCDEBUG) return false;
  char msg[256];
  pkt->bufToStr(msg, sizeof(msg));
  msg[255] = 0;
  ArLog::log(ArLog::Terse, "Robot Controller Said: %s", msg);
  return true;
}


const char* ArnlSystem::getServerMode() const {
  return ArServerMode::getActiveModeModeString();
}

const char* ArnlSystem::getServerStatus() const {
  return ArServerMode::getActiveModeStatusString();
}

const char* ArnlSystem::getPathStateName() const
{
  switch(pathTask->getState())
  {
    case ArPathPlanningTask::NOT_INITIALIZED: return "NOT_INITIALIZED";
    case ArPathPlanningTask::PLANNING_PATH: return "PLANNING_PATH";
    case ArPathPlanningTask::MOVING_TO_GOAL: return "MOVING_TO_GOAL";
    case ArPathPlanningTask::REACHED_GOAL: return "REACHED_GOAL";
    case ArPathPlanningTask::FAILED_PLAN: return "FAILED_PLAN";
    case ArPathPlanningTask::FAILED_MOVE: return "FAILED_MOVE";
    case ArPathPlanningTask::ABORTED_PATHPLAN: return "ABORTED_PATHPLAN";
    case ArPathPlanningTask::INVALID:
    default:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

