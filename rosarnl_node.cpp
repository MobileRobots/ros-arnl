
#include "Aria/Aria.h"
#include "ArNetworking/ArNetworking.h"
#include "Arnl.h"
#include "ArPathPlanningInterface.h"
#include "ArLocalizationTask.h"
#include "ArDocking.h"
#include "Aria/ArSystemStatus.h"

#include "RobotMonitor.h"

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

    ros::Publisher motors_state_pub;
    std_msgs::Bool motors_state;
    bool published_motors_state;

    //std::string serial_port;

    ArRobot *robot;
    ArSonarDevice *sonarDev;
   
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

  motors_state_pub = n.advertise<std_msgs::Bool>("motors_state", 5, true /*latch*/ );
  motors_state.data = false;
  published_motors_state = false;
  
  // subscribe to services
  //cmdvel_sub = n.subscribe( "cmd_vel", 1, (boost::function <void(const geometry_msgs::TwistConstPtr&)>)
  //  boost::bind(&RosAriaNode::cmdvel_cb, this, _1 ));

  // advertise enable/disable services
  enable_srv = n.advertiseService("enable_motors", &RosArnlNode::enable_motors_cb, this);
  disable_srv = n.advertiseService("disable_motors", &RosArnlNode::disable_motors_cb, this);
  
}

RosArnlNode::~RosArnlNode()
{
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
  ArArgumentBuilder *args = new ArArgumentBuilder(); 
  ArArgumentParser *argparser = new ArArgumentParser(args);
  argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)
  argparser->addDefaultArgument("-connectLaser");

  ArRobotConnector *robotConnector = new ArRobotConnector(argparser, robot);

  if(!robotConnector->connectRobot())
  {
    ROS_INFO_NAMED("rosarnl_node", "rosarnl_node: Error: could not connect to robot.  Exiting.");
    Aria::exit(3);
  }

  robot->addPacketHandler(new ArRetFunctor1C<bool, RosArnlNode, ArRobotPacket*>(this, &RosArnlNode::handleDebugMessage));

  // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
  //robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);


  char fileDir[1024];
  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(), 
			 "examples");
  
  
  ArLog::addToConfig(Aria::getConfig());

  ArAnalogGyro *gyro = new ArAnalogGyro(robot); // for old robots with separate gyro communication

  ArServerBase *server = new ArServerBase;

  ArServerSimpleOpener *simpleOpener = new ArServerSimpleOpener(argparser);

  ArLaserConnector *laserConnector = new ArLaserConnector(argparser, robot, robotConnector);

  if(!Aria::parseArgs() || !argparser->checkHelpAndWarnUnparsed())
  {
    Aria::exit(1);
  }
     

  ArGlobalFunctor1<int> *ariaExitF = new ArGlobalFunctor1<int>(&Aria::exit, 9);
  robot->addDisconnectOnErrorCB(ariaExitF);

  robot->addRangeDevice(new ArSonarDevice);

  ArRobotConfig *robotConfig = new ArRobotConfig(robot);

  robotConfig->addAnalogGyro(gyro);

  robot->runAsync(true);

  
  ROS_INFO_NAMED("rosarnl_node" , "Connecting to laser(s) configured in parameters...");
  if (!laserConnector->connectLasers())
  {
    ROS_INFO_NAMED("rosarnl_node" , "Error: Could not connect to laser(s). Exiting.");
    Aria::exit(2);
  }
  ROS_INFO_NAMED("rosarnl_node" , "Done connecting to laser(s).");

  robot->lock();
  ArLaser *firstLaser = robot->findLaser(1);
  assert(firstLaser);
  if (firstLaser == NULL || !firstLaser->isConnected())
  {
    ROS_INFO_NAMED("rosarnl_node" , "Did not have laser 1 or it is not connected, cannot start localization and/or mapping... exiting");
    Aria::exit(2);
  }
  robot->unlock();


  ArMap *map = new ArMap(fileDir);
  map->setIgnoreEmptyFileName(true);
  map->setIgnoreCase(true);
  map->setIgnoreBadFile(true);

    
  pathTask = new ArPathPlanningTask (robot, sonarDev, map);

  locTask = new ArLocalizationTask (robot, firstLaser, map);
  

  std::map<int, ArLaser *>::iterator laserIt;
  for (laserIt = robot->getLaserMap()->begin();
       laserIt != robot->getLaserMap()->end();
       laserIt++)
  {
    int laserNum = (*laserIt).first;
    ArLaser *laser = (*laserIt).second;

    if(!laser->isConnected())
      continue;

    laser->addDisconnectOnErrorCB(ariaExitF);
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




  if (!simpleOpener->open(server, fileDir, 240))
  {
    ROS_INFO_NAMED("rosarnl_node" , "Error: Could not open server.");
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

  
  ArServerInfoDrawings *drawings = new ArServerInfoDrawings(server);
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


  ArServerHandlerCommands *commands = new ArServerHandlerCommands(server);


  new ArServerInfoRobot(server, robot);
  new ArServerInfoSensor(server, robot);
  ArServerInfoPath *serverInfoPath = new ArServerInfoPath(server, robot, pathTask);
  serverInfoPath->addSearchRectangleDrawing(drawings);
  serverInfoPath->addControlCommands(commands);

  new ArServerInfoLocalization(server, robot, locTask);
  ArServerHandlerLocalization *serverLocHandler = new ArServerHandlerLocalization(server, robot, locTask);

  new ArServerHandlerMap(server, map);

  new ArServerSimpleComUC(commands, robot);                  
  new ArServerSimpleComMovementLogging(commands, robot); 
  new ArServerSimpleComLogRobotConfig(commands, robot); 
  new ArServerSimpleServerCommands(commands, server); 

  new ArServerHandlerCommMonitor(server);

  new ArServerHandlerConfig (server, Aria::getConfig(),
				      Arnl::getTypicalDefaultParamFileName(),
				      Aria::getDirectory());


  ArServerHandlerPopup *popupServer = new ArServerHandlerPopup(server);

  new RobotMonitor(robot, popupServer);



  new ArServerModeGoto (server, robot, pathTask, map);


  modeStop = new ArServerModeStop(server, robot);

  new ArSonarAutoDisabler(robot);

  ArServerModeRatioDrive *modeRatioDrive = new ArServerModeRatioDrive(server, robot);  


  ArActionLost *actionLostRatioDrive = new ArActionLost(locTask, pathTask, modeRatioDrive);
  modeRatioDrive->getActionGroup()->addAction(actionLostRatioDrive, 110);

  modeRatioDrive->addToConfig(Aria::getConfig(), "Teleop settings");
  modeRatioDrive->addControlCommands(commands);

  // Tool to log data periodically to a file
  //ArDataLogger dataLogger(&robot, "datalog.txt");
  //dataLogger.addToConfig(Aria::getConfig()); // make it configurable through ArConfig

  // Automatically add anything from the global info group to the data logger.
  //Aria::getInfoGroup()->addAddStringCallback(dataLogger.getAddStringFunctor());

  ArServerInfoStrings *stringInfo = new ArServerInfoStrings(server);
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
  


  ArServerModeDock *modeDock = NULL;
  modeDock = ArServerModeDock::createDock(server, robot, locTask, pathTask);
  if (modeDock != NULL)
  {
    modeDock->checkDock();
    modeDock->addAsDefaultMode();
    modeDock->addToConfig(Aria::getConfig());
    modeDock->addControlCommands(commands);
  }



  modeStop->addAsDefaultMode();



  ArServerHandlerMapping *handlerMapping = new ArServerHandlerMapping(server, robot, firstLaser, 
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
  ROS_WARN_NAMED("rosarnl_node" , "rosarnl_node: file upload/download services are not implemented for Windows; not enabling them.");
#else
  new ArServerFileLister (server, fileDir);
  new ArServerFileToClient (server, fileDir);
  new ArServerFileFromClient (server, fileDir, "/tmp");
  new ArServerDeleteFileOnServer (server, fileDir);
#endif

  
  // When parsing the configuration file, also look at the program's command line options 
  // from the command-line argument parser as well as the configuration file.
  // (So you can use any argument on the command line, namely -map.) 
  Aria::getConfig()->useArgumentParser(argparser);

  // Read in parameter files.
  ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node: Loading config file %s%s into ArConfig...", Aria::getDirectory(), Arnl::getTypicalParamFileName());
  if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node: Could not load ARNL configuration file. Set ARNL environment variable to use non-default installation director.y");
    Aria::exit(5);
  }

  if (!simpleOpener->checkAndLog() || !argparser->checkHelpAndWarnUnparsed())
  {
    Aria::exit(6);
  }

  // Warn if there is no map
  if (map->getFileName() == NULL || strlen(map->getFileName()) <= 0)
  {
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node: ### No map file is set up, you can make a map with the following procedure");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:    0) You can find this information in README.txt or docs/Mapping.txt");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:    1) Connect to this server with MobileEyes");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:    2) Go to Tools->Map Creation->Start Scan");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:    3) Give the map a name and hit okay");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:    4) Drive the robot around your space (see docs/Mapping.txt");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:    5) Go to Tools->Map Creation->Stop Scan");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:    6) Start up Mapper3");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:    7) Go to File->Open on Robot");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:    8) Select the .2d you created");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:    9) Create a .map");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:   10) Go to File->Save on Robot");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:   11) In MobileEyes, go to Tools->Robot Config");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:   12) Choose the Files section");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:   13) Enter the path and name of your new .map file for the value of the Map parameter.");
    ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node:   14) Press OK and your new map should become the map used");
  }

  // Print a log message notifying user of the directory for map files
  ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node: Directory for maps and file serving: %s", fileDir);
  
  ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node: See the ARNL README.txt for more information");

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


  ROS_INFO_NAMED("rosarnl_node", "rosarnl_node: ARNL server is now running. You may connect with MobileEyes and Mapper3.");

  robot->lock();
  robot->enableMotors();
  robot->disableSonar();
  robot->unlock();


  return 0;
}

void RosArnlNode::spin()
{
  ROS_INFO_NAMED("rosarnl_node", "rosarnl_node: Running ROS node...");
  ros::spin();
}

void RosArnlNode::publish()
{
/*
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
*/
  
  // publish motors state if changed
  bool e = robot->areMotorsEnabled();
  if(e != motors_state.data || !published_motors_state)
  {
    ROS_INFO_NAMED("rosarnl_node", "rosarnl_node: publishing new motors state %d.", e);
    motors_state.data = e;
    motors_state_pub.publish(motors_state);
    published_motors_state = true;
  }

}

bool RosArnlNode::enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO_NAMED("rosarnl_node", "rosarnl_node: Enable motors request.");
    robot->lock();
    if(robot->isEStopPressed())
        ROS_WARN_NAMED("rosarnl_node", "rosarnl_node: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");
    robot->enableMotors();
    robot->unlock();
	// todo could wait and see if motors do become enabled, and send a response with an error flag if not
    return true;
}

bool RosArnlNode::disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO_NAMED("rosarnl_node", "rosarnl_node: Disable motors request.");
    robot->lock();
    robot->disableMotors();
    robot->unlock();
	// todo could wait and see if motors do become disabled, and send a response with an error flag if not
    return true;
}


bool RosArnlNode::handleDebugMessage(ArRobotPacket *pkt)
{
  if(pkt->getID() != ArCommands::MARCDEBUG) return false;
  char msg[256];
  pkt->bufToStr(msg, sizeof(msg));
  msg[255] = 0;
  ROS_WARN_NAMED("rosarnl_node", "rosarnl_node: Robot Firmware: %s", msg);
  return true;
}

void ariaLogHandler(const char *msg, ArLog::LogLevel level)
{
  // node that ARIA logging is normally limited at Normal and Terse only. Set
  // ARLOG_LEVEL environment variable to override.
  switch(level)
  {
    case ArLog::Normal:
      ROS_INFO_NAMED("ARNL", "ARNL: %s", msg);
      return;
    case ArLog::Terse:
      ROS_WARN_NAMED("ARNL", "ARNL: %s", msg);
      return;
    case ArLog::Verbose:
      ROS_DEBUG_NAMED("ARNL", "ARNL: %s", msg);
      return;
  }
}

  

int main( int argc, char** argv )
{
  ros::init(argc,argv, "rosarnl_node");

  Aria::init();
  Arnl::init();
 /* set log type to None to only use
    ariaLogHandler to redirect ARNL log messages to rosconsole by deufault. This can
    be changed in the ARNL parameter file however.
  */
  ArLog::init(ArLog::None, ArLog::Normal); 

  ArLog::setFunctor(new ArGlobalFunctor2<const char *, ArLog::LogLevel>(&ariaLogHandler));

  ros::NodeHandle n(std::string("~"));
  RosArnlNode *node = new RosArnlNode(n);
  if( node->Setup() != 0 )
  {
    ROS_FATAL_NAMED("rosarnl_node", "rosarnl_node: ROS node setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  ROS_INFO_NAMED("rosarnl_node",  "rosarnl_node: Quitting... \n" );
  return 0;
  
}


