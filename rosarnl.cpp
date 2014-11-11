
#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"
#include "ArPathPlanningInterface.h"
#include "ArLocalizationTask.h"
#include "ArDocking.h"
#include "ArSystemStatus.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"  //for tf::getPrefixParam
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <dynamic_reconfigure/server.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"

#include <sstream>


class RosArnlNode
{
  public:
    RosArnlNode(ros::NodeHandle n);
    virtual ~RosArnlNode();
    
  public:
    int Setup();
    void spin();
    void publish();

  protected:
    ros::NodeHandle n;
    ros::ServiceServer enable_srv;
    ros::ServiceServer disable_srv;
    bool enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    //std::string serial_port;

    ArRobotConnector *conn;
    ArRobot *robot;
    ArAnalogGyro gyro;
    ArServerBase server;
    ArServerSimpleOpener simpleOpener;
    ArLaserConnector laserConnector;
    ArSonarDevice sonarDev;
    ArRobotConfig robotConfig;
    ArBumpers bumpers;
   
    ArPathPlanningTask *pathTask;
    ArLocalizationTask *locTask;

    ArServerModeStop *modeStop;

    bool handleDebugMessage(ArRobotPacket *pkt);
};


RosArnlNode::RosArnlNode(ros::NodeHandle nh) 
{
  // read in config options
  n = nh;

  //n.param( "port", serial_port, std::string("/dev/ttyUSB0") );
  //ROS_INFO( "rosarnl: using port: [%s]", serial_port.c_str() );


  // Figure out what frame_id's to use. if a tf_prefix param is specified,
  // it will be added to the beginning of the frame_ids.
  //
  // e.g. rosrun ... _tf_prefix:=MyRobot (or equivalently using <param>s in
  // roslaunch files)
  // will result in the frame_ids being set to /MyRobot/odom etc,
  // rather than /odom. This is useful for Multi Robot Systems.
  // See ROS Wiki for further details.
/*
  tf_prefix = tf::getPrefixParam(n);
  frame_id_odom = tf::resolve(tf_prefix, "odom");
  frame_id_base_link = tf::resolve(tf_prefix, "base_link");
  frame_id_bumper = tf::resolve(tf_prefix, "bumpers_frame");
  frame_id_sonar = tf::resolve(tf_prefix, "sonar_frame");
*/


  // advertise enable/disable services
  enable_srv = n.advertiseService("enable_motors", &RosArnlNode::enable_motors_cb, this);
  disable_srv = n.advertiseService("disable_motors", &RosArnlNode::disable_motors_cb, this);
  
}

RosArnlNode::~RosArnlNode()
{
  // disable motors and sonar.
  robot->disableMotors();
  robot->disableSonar();

  robot->stopRunning();
  robot->waitForRunExit();
  Aria::shutdown();
}

int RosArnlNode::Setup()
{
  // Note, various objects are allocated here which are never deleted (freed), since Setup() is only supposed to be
  // called once per instance, and these objects need to persist until the process terminates.

  robot = new ArRobot();
  ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
  ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)

  // handle messages from robot controller firmware and log the contents
  robot->addPacketHandler(new ArGlobalRetFunctor1<bool, ArRobotPacket*>(&handleDebugMessage));

  // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
  //robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);


  // Set up where we'll look for files. Arnl::init() set Aria's default
  // directory to Arnl's default directory; addDirectories() appends this
  // "examples" directory.
  char fileDir[1024];
  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(), 
			 "examples");
  
  
  // Add a section to the configuration to change ArLog parameters
  ArLog::addToConfig(Aria::getConfig());

  // Parse arguments 
  if (!Aria::parseArgs() || !argparser->checkHelpAndWarnUnparsed())
  {
    Aria::exit(1);
  }
  

  // This causes Aria::exit(9) to be called if the robot unexpectedly
  // disconnects
  ArGlobalFunctor1<int> shutdownFunctor(&Aria::exit, 9);
  robot->addDisconnectOnErrorCB(&shutdownFunctor);


  // Create an ArSonarDevice object (ArRangeDevice subclass) and 
  // connect it to the robot->
  robot->addRangeDevice(&sonarDev);

  // Include gyro configuration options in the robot configuration section.
  robotConfig.addAnalogGyro(&gyro);

  // Start the robot thread.
  robot->runAsync(true);

  
  // connect the laser(s) if it was requested, this adds them to the
  // robot too, and starts them running in their own threads
  ArLog::log(ArLog::Normal, "Connecting to laser(s) configured in parameters...");
  if (!laserConnector.connectLasers())
  {
    ArLog::log(ArLog::Normal, "Error: Could not connect to laser(s). Exiting.");
    Aria::exit(2);
  }
  ArLog::log(ArLog::Normal, "Done connecting to laser(s).");

  // find the laser we should use for localization and/or mapping,
  // which will be the first laser
  robot->lock();
  ArLaser *firstLaser = robot->findLaser(1);
  if (firstLaser == NULL || !firstLaser->isConnected())
  {
    ArLog::log(ArLog::Normal, "Did not have laser 1 or it is not connected, cannot start localization and/or mapping... exiting");
    Aria::exit(2);
  }
  robot->unlock();


    /* Create and set up map object */
  
  // Set up the map object, this will look for files in the examples
  // directory (unless the file name starts with a /, \, or .
  // You can take out the 'fileDir' argument to look in the program's current directory
  // instead.
  // When a configuration file is loaded into ArConfig later, if it specifies a
  // map file, then that file will be loaded as the map.
  ArMap *map = new ArMap(fileDir);
  // set it up to ignore empty file names (otherwise if a configuration omits
  // the map file, the whole configuration change will fail)
  map->setIgnoreEmptyFileName(true);
  // ignore the case, so that if someone is using MobileEyes or
  // MobilePlanner from Windows and changes the case on a map name,
  // it will still work.
  map->setIgnoreCase(true);

    
    /* Create localization and path planning threads */


  pathTask = new ArPathPlanningTask (robot, &sonarDev, map);

  locTask = new ArLocalizationTask (robot, firstLaser, map);
  

  // Set some options  and callbacks on each laser that the laser connector 
  // connected to.
  std::map<int, ArLaser *>::iterator laserIt;
  for (laserIt = robot->getLaserMap()->begin();
       laserIt != robot->getLaserMap()->end();
       laserIt++)
  {
    int laserNum = (*laserIt).first;
    ArLaser *laser = (*laserIt).second;

    // Skip lasers that aren't connected
    if(!laser->isConnected())
      continue;

    // add the disconnectOnError CB to shut things down if the laser
    // connection is lost
    laser->addDisconnectOnErrorCB(&shutdownFunctor);
    // set the number of cumulative readings the laser will take
    laser->setCumulativeBufferSize(200);
    // add the lasers to the path planning task
    pathTask.addRangeDevice(laser, ArPathPlanningTask::BOTH);
    // set the cumulative clean offset (so that they don't all fire at once)
    laser->setCumulativeCleanOffset(laserNum * 100);
    // reset the cumulative clean time (to make the new offset take effect)
    laser->resetLastCumulativeCleanTime();

    // Add the packet count to the Aria info strings (It will be included in
    // MobileEyes custom details so you can monitor whether the laser data is
    // being received correctly)
    std::string laserPacketCountName;
    laserPacketCountName = laser->getName();
    laserPacketCountName += " Packet Count";
    Aria::getInfoGroup()->addStringInt(
	    laserPacketCountName.c_str(), 10, 
	    new ArRetFunctorC<int, ArLaser>(laser, 
					 &ArLaser::getReadingCount));
  }





    /* Start the server */

  // Open the networking server
  if (!simpleOpener.open(&server, fileDir, 240))
  {
    ArLog::log(ArLog::Normal, "Error: Could not open server.");
    exit(2);
  }




  /* Add additional range devices to the robot and path planning task (so it
     avoids obstacles detected by these devices) */
  
  // Add bumpers range device to robot and path planning task (so it avoids obstacles
  // detected by this device)
  robot->addRangeDevice(&bumpers);
  pathTask->addRangeDevice(&bumpers, ArPathPlanningTask::CURRENT);

  // Add range device which uses forbidden regions given in the map to give virtual
  // range device readings to ARNL.  (so it avoids obstacles
  // detected by this device)
  ArForbiddenRangeDevice *forbidden = new ArForbiddenRangeDevice(map);
  robot->addRangeDevice(forbidden);
  pathTask->addRangeDevice(forbidden, ArPathPlanningTask::CURRENT);

  robot->unlock();


  // Action to slow down robot when localization score drops but not lost.
  ArActionSlowDownWhenNotCertain *actionSlowDown = new ArActionSlowDownWhenNotCertain(locTask);
  pathTask->getPathPlanActionGroup()->addAction(actionSlowDown, 140);

  // Action to stop the robot when localization is "lost" (score too low)
  ArActionLost *actionLostPath = new ArActionLost(locTask, pathTask);
  pathTask->getPathPlanActionGroup()->addAction(actionLostPath, 150);

  // Arnl uses this object when it must replan its path because its
  // path is completely blocked.  It will use an older history of sensor
  // readings to replan this new path.  This should not be used with SONARNL
  // since sonar readings are not accurate enough and may prevent the robot
  // from planning through space that is actually clear.
  ArGlobalReplanningRangeDevice *replanDev = new ArGlobalReplanningRangeDevice(pathTask);

  
  // Service to provide drawings of data in the map display :
  ArServerInfoDrawings *drawings = new ArServerInfoDrawings(&server);
  drawings->addRobotsRangeDevices(robot);
  drawings->addRangeDevice(replanDev);

  /* Draw a box around the local path planning area use this 
    (You can enable this particular drawing from custom commands 
    which is set up down below in ArServerInfoPath) */
  ArDrawingData *drawingDataP = new ArDrawingData("polyLine", ArColor(200,200,200), 1, 75);
  ArFunctor2C<ArPathPlanningTask, ArServerClient *, ArNetPacket *> *drawingFunctorP = new ArFunchtor2C<ArPathPlanningTask, ArServerClient*, ArNetPacket*>(pathTask, &ArPathPlanningTask::drawSearchRectangle);
  drawings->addDrawing(drawingDataP, "Local Plan Area", drawingFunctorP); 

  /* Draw a box showing the safety clearance used by the path planning task to */
  drawings->addDrawing(
    new ArDrawingData("polySegments", ArColor(166, 166, 166), 1, 60, 100, "DefaultOn"),
    "Path Planning Clearances",
    new ArFunctor2C<ArPathPlanningTask, ArServerClient*, ArNetPacket*>(&pathTask, &ArPathPlanningTask::drawRobotBounds)
  );

  /* Show the sample points used by MCL */
  drawings->addDrawing(new ArDrawingData("polyDots", ArColor(0, 255, 0), 100, 75), "Localization Points", new ArFunctor2C<ArLocalizationTask, ArServerClient*, ArNetPacket*>(locTask, &ArLocalizationTask::drawRangePoints);


  // "Custom" commands. You can add your own custom commands here, they will
  // be available in MobileEyes' custom commands (enable in the toolbar or
  // access through Robot Tools)
  ArServerHandlerCommands *commands = new ArServerHandlerCommands(&server);


  // These provide various kinds of information to the client:
  new ArServerInfoRobot(&server, robot);
  new ArServerInfoSensor(&server, robot);
  ArServerInfoPath *serverInfoPath = new ArServerInfoPath(&server, robot, pathTask);
  serverInfoPath->addSearchRectangleDrawing(drawings);
  serverInfoPath->addControlCommands(commands);

  // Provides localization info and allows the client (MobileEyes) to relocalize at a given
  // pose:
  new ArServerInfoLocalization(&server, robot, locTask);
  new ArServerHandlerLocalization(&server, robot, locTask);

  // If you're using MobileSim, ArServerHandlerLocalization sends it a command
  // to move the robot's true pose if you manually do a localization through 
  // MobileEyes.  To disable that behavior, use this constructor call instead:
  // ArServerHandlerLocalization serverLocHandler(&server, &robot, true, false);
  // The fifth argument determines whether to send the command to MobileSim.

  // Provide the map to the client (and related controls):
  new ArServerHandlerMap(&server, map);

  // These objects add some simple (custom) commands to 'commands' for testing and debugging:
  new ArServerSimpleComUC(commands, robot);                   // Send any command to the microcontroller
  new ArServerSimpleComMovementLogging(commands, robot); // configure logging
  new ArServerSimpleComLogRobotConfig(commands, robot);   // trigger logging of the robot config parameters
  new ArServerSimpleServerCommands(commands, &server);     // monitor networking behavior (track packets sent etc.)


  // service that allows the client to monitor the communication link status
  // between the robot and the client.
  //
  new ArServerHandlerCommMonitor(&server);



  // service that allows client to change configuration parameters in ArConfig 
  new ArServerHandlerConfig (&server, Aria::getConfig(),
				      Arnl::getTypicalDefaultParamFileName(),
				      Aria::getDirectory());


  // This service causes the client to show simple dialog boxes
  ArServerHandlerPopup *popupServer = new ArServerHandlerPopup(&server);

  // Monitor the robot for current state such as if the motors are disabled,
  // indicate this state by various means and show a popup dialog on clients
  // if user confirmation is needed.
  // (Namely, some robots  disable motors automatically on various error conditions,
  // and we want the user to manually re-enable them after resolving the
  // problem)
  RobotMonitor *robotMonitor = new RobotMonitor(robot, popupServer);



  /* Set up the possible modes for remote control from a client such as
   * MobileEyes:
   */

  // Mode To go to a goal or other specific point:
  new ArServerModeGoto (&server, robot, pathTask, map);


  // Mode To stop and remain stopped:
  modeStop = new ArServerModeStop(&server, robot);

  // Cause the sonar to turn off automatically
  // when the robot is stopped, and turn it back on when commands to move
  // are sent. (Note, if using SONARNL to localize, then don't do this
  // since localization may get lost)
  new ArSonarAutoDisabler(&robot);

  // Teleoperation modes To drive by keyboard, joystick, etc:
  ArServerModeRatioDrive *modeRatioDrive = new ArServerModeRatioDrive(&server, robot);  



  // Prevent normal teleoperation driving if localization is lost using
  // a high-priority action, which enables itself when the particular mode is
  // active.
  // (You have to enter unsafe drive mode to drive when lost.)
  ArActionLost *actionLostRatioDrive = new ArActionLost(locTask, pathTask, modeRatioDrive);
  modeRatioDrive->getActionGroup()->addAction(actionLostRatioDrive, 110);

  // Add drive mode section to the configuration, and also some custom (simple) commands:
  modeRatioDrive->addToConfig(Aria::getConfig(), "Teleop settings");
  modeRatioDrive->addControlCommands(commands);

  // Tool to log data periodically to a file
  //ArDataLogger dataLogger(&robot, "datalog.txt");
  //dataLogger.addToConfig(Aria::getConfig()); // make it configurable through ArConfig

  // Automatically add anything from the global info group to the data logger.
  //Aria::getInfoGroup()->addAddStringCallback(dataLogger.getAddStringFunctor());

  // This provides a small table of interesting information for the client
  // to display to the operator. You can add your own callbacks to show any
  // data you want.
  ArServerInfoStrings *stringInfo = new ArServerInfoStrings(&server);
  Aria::getInfoGroup()->addAddStringCallback(stringInfo->getAddStringFunctor());
  
  // The following statements add fields to a set of informational data called
  // the InfoGroup. These are served to MobileEyes for displayi (turn on by enabling Details
  // and Custom Details in the View menu of MobileEyes.)

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


  // Display gyro status if gyro is enabled and is being handled by the firmware (gyro types 2, 3, or 4).
  // (If the firmware detects an error communicating with the gyro or IMU it
  // returns a flag, and stops using it.)
  // (This gyro type parameter, and fault flag, are only in ARCOS, not Seekur firmware)
  if(robot->getOrigRobotConfig() && robot->getOrigRobotConfig()->getGyroType() > 1)
  {
    Aria::getInfoGroup()->addStringString(
          "Gyro/IMU Status", 10,
          new ArGlobalRetFunctor1<const char*, ArRobot*>(&getGyroStatusString,robot)
      );
  }

  // Display system CPU and wireless network status
  ArSystemStatus::startPeriodicUpdate(1000); // update every 1 second
  Aria::getInfoGroup()->addStringDouble("CPU Use", 10, ArSystemStatus::getCPUPercentFunctor(), "% 4.0f%%");
  Aria::getInfoGroup()->addStringInt("Wireless Link Quality", 9, ArSystemStatus::getWirelessLinkQualityFunctor(), "%d");
  Aria::getInfoGroup()->addStringInt("Wireless Link Noise", 9, ArSystemStatus::getWirelessLinkNoiseFunctor(), "%d");
  Aria::getInfoGroup()->addStringInt("Wireless Signal", 9, ArSystemStatus::getWirelessLinkSignalFunctor(), "%d");
  

  // stats on how far its driven since software started
  Aria::getInfoGroup()->addStringDouble("Distance Travelled (m)", 20, new ArRetFunctorC<double, ArRobot>(robot, &ArRobot::getOdometerDistanceMeters), "%.2f");
  Aria::getInfoGroup()->addStringDouble("Run time (min)", 20, new
ArRetFunctorC<double, ArRobot>(robot, &ArRobot::getOdometerTimeMinutes),
"%.2f");


  // Setup the dock if there is a docking system on board.
  ArServerModeDock *modeDock = NULL;
  modeDock = ArServerModeDock::createDock(&server, robot, locTask, pathTask);
  if (modeDock != NULL)
  {
    modeDock->checkDock();
    modeDock->addAsDefaultMode();
    modeDock->addToConfig(Aria::getConfig());
    modeDock->addControlCommands(commands);
  }




  // Make Stop mode the default (If current mode deactivates without entering
  // a new mode, then Stop Mode will be selected)
  modeStop->addAsDefaultMode();



  /* Services that allow the client to initiate scanning with the laser to
     create maps in Mapper3 (So not possible with SONARNL): */

  ArServerHandlerMapping *handlerMapping = new ArServerHandlerMapping(&server, &robot, firstLaser, 
					fileDir, "", true);

  // make laser localization stop while mapping
  handlerMapping->addMappingStartCallback(
	  new ArFunctor1C<ArLocalizationTask, bool>
	  (locTask, &ArLocalizationTask::setIdleFlag, true));

  // and then make it start again when we're doine
  handlerMapping.addMappingEndCallback(
	  new ArFunctor1C<ArLocalizationTask, bool>
	  (locTask, &ArLocalizationTask::setIdleFlag, false));


  // Make it so our "lost" actions don't stop us while mapping
  handlerMapping->addMappingStartCallback(actionLostPath->getDisableCB());
  handlerMapping->addMappingStartCallback(actionLostRatioDrive->getDisableCB());
  handlerMapping->addMappingStartCallback(actionLostWander->getDisableCB());

  // And then let them make us stop as usual when done mapping
  handlerMapping->addMappingEndCallback(actionLostPath->getEnableCB());
  handlerMapping->addMappingEndCallback(actionLostRatioDrive->getEnableCB());
  handlerMapping->addMappingEndCallback(actionLostWander->getEnableCB());

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
  ArLog::log(ArLog::Normal, "Note, file upload/download services are not implemented for Windows; not enabling them.");
#else
  new ArServerFileLister (&server, fileDir);
  new ArServerFileToClient (&server, fileDir);
  new ArServerFileFromClient (&server, fileDir, "/tmp");
  new ArServerDeleteFileOnServer (&server, fileDir);
#endif

    /* Load configuration values, map, and begin! */

  
  // When parsing the configuration file, also look at the program's command line options 
  // from the command-line argument parser as well as the configuration file.
  // (So you can use any argument on the command line, namely -map.) 
  Aria::getConfig()->useArgumentParser(argparser);

  // Read in parameter files.
  ArLog::log(ArLog::Normal, "Loading config file %s%s into ArConfig...", Aria::getDirectory(), Arnl::getTypicalParamFileName());
  if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    ArLog::log(ArLog::Normal, "Could not load ARNL configuration file. Set ARNL environment variable to use non-default installation director.y");
    Aria::exit(5);
  }

  // Warn about unknown params.
  if (!simpleOpener->checkAndLog() || !argparser->checkHelpAndWarnUnparsed())
  {
    logOptions(argv[0]);
    Aria::exit(6);
  }

  // Warn if there is no map
  if (map->getFileName() == NULL || strlen(map->getFileName()) <= 0)
  {
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "### No map file is set up, you can make a map with the following procedure");
    ArLog::log(ArLog::Normal, "   0) You can find this information in README.txt or docs/Mapping.txt");
    ArLog::log(ArLog::Normal, "   1) Connect to this server with MobileEyes");
    ArLog::log(ArLog::Normal, "   2) Go to Tools->Map Creation->Start Scan");
    ArLog::log(ArLog::Normal, "   3) Give the map a name and hit okay");
    ArLog::log(ArLog::Normal, "   4) Drive the robot around your space (see docs/Mapping.txt");
    ArLog::log(ArLog::Normal, "   5) Go to Tools->Map Creation->Stop Scan");
    ArLog::log(ArLog::Normal, "   6) Start up Mapper3");
    ArLog::log(ArLog::Normal, "   7) Go to File->Open on Robot");
    ArLog::log(ArLog::Normal, "   8) Select the .2d you created");
    ArLog::log(ArLog::Normal, "   9) Create a .map");
    ArLog::log(ArLog::Normal, "  10) Go to File->Save on Robot");
    ArLog::log(ArLog::Normal, "  11) In MobileEyes, go to Tools->Robot Config");
    ArLog::log(ArLog::Normal, "  12) Choose the Files section");
    ArLog::log(ArLog::Normal, "  13) Enter the path and name of your new .map file for the value of the Map parameter.");
    ArLog::log(ArLog::Normal, "  14) Press OK and your new map should become the map used");
    ArLog::log(ArLog::Normal, "");    
  }

  // Print a log message notifying user of the directory for map files
  ArLog::log(ArLog::Normal, "");
  ArLog::log(ArLog::Normal, 
	     "Directory for maps and file serving: %s", fileDir);
  
  ArLog::log(ArLog::Normal, "See the ARNL README.txt for more information");
  ArLog::log(ArLog::Normal, "");

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
  server->runAsync();


  robot->lock();
  robot->enableMotors();
  robot->disableSonar();
  robot->unlock();


  return 0;
}

void RosArnlNode::spin()
{
  ros::spin();
}

void RosArnlNode::publish()
{
  // Note, this is called via SensorInterpTask callback (myPublishCB, named "ROSPublishingTask"). ArRobot object 'robot' sholud not be locked or unlocked.
  pos = robot->getPose();
  tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(pos.getTh()*M_PI/180), tf::Vector3(pos.getX()/1000,
    pos.getY()/1000, 0)), position.pose.pose); //Aria returns pose in mm.
  position.twist.twist.linear.x = robot->getVel()/1000; //Aria returns velocity in mm/s.
  position.twist.twist.linear.y = robot->getLatVel()/1000.0;
  position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;
  
  position.header.frame_id = frame_id_odom;
  position.child_frame_id = frame_id_base_link;
  position.header.stamp = ros::Time::now();
  pose_pub.publish(position);

  ROS_DEBUG("rosarnl: publish: (time %f) pose x: %f, y: %f, angle: %f; linear vel x: %f, y: %f; angular vel z: %f", 
    position.header.stamp.toSec(), 
    (double)position.pose.pose.position.x,
    (double)position.pose.pose.position.y,
    (double)position.pose.pose.orientation.w,
    (double) position.twist.twist.linear.x,
    (double) position.twist.twist.linear.y,
    (double) position.twist.twist.angular.z
  );


  // publishing transform odom->base_link
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = frame_id_odom;
  odom_trans.child_frame_id = frame_id_base_link;
  
  odom_trans.transform.translation.x = pos.getX()/1000;
  odom_trans.transform.translation.y = pos.getY()/1000;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pos.getTh()*M_PI/180);
  
  odom_broadcaster.sendTransform(odom_trans);
  
  // getStallValue returns 2 bytes with stall bit and bumper bits, packed as (00 00 FrontBumpers RearBumpers)
  int stall = robot->getStallValue();
  unsigned char front_bumpers = (unsigned char)(stall >> 8);
  unsigned char rear_bumpers = (unsigned char)(stall);

  bumpers.header.frame_id = frame_id_bumper;
  bumpers.header.stamp = ros::Time::now();

  std::stringstream bumper_info(std::stringstream::out);
  // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
  for (unsigned int i=0; i<robot->getNumFrontBumpers(); i++)
  {
    bumpers.front_bumpers[i] = (front_bumpers & (1 << (i+1))) == 0 ? 0 : 1;
    bumper_info << " " << (front_bumpers & (1 << (i+1)));
  }
  ROS_DEBUG("rosarnl: Front bumpers:%s", bumper_info.str().c_str());

  bumper_info.str("");
  // Rear bumpers have reverse order (rightmost is LSB)
  unsigned int numRearBumpers = robot->getNumRearBumpers();
  for (unsigned int i=0; i<numRearBumpers; i++)
  {
    bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
    bumper_info << " " << (rear_bumpers & (1 << (numRearBumpers-i)));
  }
  ROS_DEBUG("rosarnl: Rear bumpers:%s", bumper_info.str().c_str());
  
  bumpers_pub.publish(bumpers);

  //Publish battery information
  // TODO: Decide if BatteryVoltageNow (normalized to (0,12)V)  is a better option
  std_msgs::Float64 batteryVoltage;
  batteryVoltage.data = robot->getRealBatteryVoltageNow();
  voltage_pub.publish(batteryVoltage);

  if(robot->haveStateOfCharge())
  {
    std_msgs::Float32 soc;
    soc.data = robot->getStateOfCharge()/100.0;
    state_of_charge_pub.publish(soc);
  }

  // publish recharge state if changed
  char s = robot->getChargeState();
  if(s != recharge_state.data)
  {
    ROS_INFO("rosarnl: publishing new recharge state %d.", s);
    recharge_state.data = s;
    recharge_state_pub.publish(recharge_state);
  }

  // publish motors state if changed
  bool e = robot->areMotorsEnabled();
  if(e != motors_state.data || !published_motors_state)
  {
	ROS_INFO("rosarnl: publishing new motors state %d.", e);
	motors_state.data = e;
	motors_state_pub.publish(motors_state);
	published_motors_state = true;
  }

  // Publish sonar information, if enabled.
  if (use_sonar) {
    sensor_msgs::PointCloud cloud;	//sonar readings.
    cloud.header.stamp = position.header.stamp;	//copy time.
    // sonar sensors relative to base_link
    cloud.header.frame_id = frame_id_sonar;
    

    // Log debugging info
    std::stringstream sonar_debug_info;
    sonar_debug_info << "Sonar readings: ";
    for (int i = 0; i < robot->getNumSonar(); i++) {
      ArSensorReading* reading = NULL;
      reading = robot->getSonarReading(i);
      if(!reading) {
        ROS_WARN("rosarnl: Did not receive a sonar reading.");
        continue;
      }
      
      // getRange() will return an integer between 0 and 5000 (5m)
      sonar_debug_info << reading->getRange() << " ";

      // local (x,y). Appears to be from the centre of the robot, since values may
      // exceed 5000. This is good, since it means we only need 1 transform.
      // x & y seem to be swapped though, i.e. if the robot is driving north
      // x is north/south and y is east/west.
      //
      //ArPose sensor = reading->getSensorPosition();  //position of sensor.
      // sonar_debug_info << "(" << reading->getLocalX() 
      //                  << ", " << reading->getLocalY()
      //                  << ") from (" << sensor.getX() << ", " 
      //                  << sensor.getY() << ") ;; " ;
      
      //add sonar readings (robot-local coordinate frame) to cloud
      geometry_msgs::Point32 p;
      p.x = reading->getLocalX() / 1000.0;
      p.y = reading->getLocalY() / 1000.0;
      p.z = 0.0;
      cloud.points.push_back(p);
    }
    ROS_DEBUG_STREAM(sonar_debug_info.str());
    
    sonar_pub.publish(cloud);
  }

}

bool RosArnlNode::enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("rosarnl: Enable motors request.");
    robot->lock();
    if(robot->isEStopPressed())
        ROS_WARN("rosarnl: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");
    robot->enableMotors();
    robot->unlock();
	// todo could wait and see if motors do become enabled, and send a response with an error flag if not
    return true;
}

bool RosArnlNode::disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("rosarnl: Disable motors request.");
    robot->lock();
    robot->disableMotors();
    robot->unlock();
	// todo could wait and see if motors do become disabled, and send a response with an error flag if not
    return true;
}

void
RosArnlNode::cmdvel_cb( const geometry_msgs::TwistConstPtr &msg)
{
  veltime = ros::Time::now();
  ROS_INFO( "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.toSec() );

  robot->lock();
  robot->setVel(msg->linear.x*1e3);
  if(robot->hasLatVel())
    robot->setLatVel(msg->linear.y*1e3);
  robot->setRotVel(msg->angular.z*180/M_PI);
  robot->unlock();
  ROS_DEBUG("rosarnl: sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.toSec(),
    (double) msg->linear.x * 1e3, (double) msg->linear.y * 1.3, (double) msg->angular.z * 180/M_PI);
}


/// Log messages from robot controller
bool RosArnlNode::handleDebugMessage(ArRobotPacket *pkt)
{
  if(pkt->getID() != ArCommands::MARCDEBUG) return false;
  char msg[256];
  pkt->bufToStr(msg, sizeof(msg));
  msg[255] = 0;
  ArLog::log(ArLog::Terse, "Controller Firmware: %s", msg);
  return true;
}

int main( int argc, char** argv )
{
  ros::init(argc,argv, "rosarnl");
  ros::NodeHandle n(std::string("~"));
  Aria::init();
  Arnl::init();

  RosArnlNode *node = new RosArnlNode(n);

  if( node->Setup() != 0 )
  {
    ROS_FATAL( "rosarnl: ROS node setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  ROS_INFO( "rosarnl: Quitting... \n" );
  return 0;
  
}


