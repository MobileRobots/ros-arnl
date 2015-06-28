#ifndef _ARNLSYSTEM_H
#define _ARNLSYSTEM_H

class ArRobot;
class ArPathPlanningTask;
class ArLocalizationTask;
class ArServerModeStop;
class ArRobotPacket;
class ArServerBase;

class ArnlSystem
{
  public:
    ArnlSystem(const char *_logprefix = "");
    virtual ~ArnlSystem();
       
  public:

    typedef enum {
      OK,
      RobotConnectError,
      ParseArgumentsError,
      ConfigError,
      LaserConnectError
    } Error;

    Error setup();

    ArRobot *robot;
    ArPathPlanningTask *pathTask;
    ArLocalizationTask *locTask;
    ArServerModeStop *modeStop;
    ArServerBase *serverBase;

    const char* getServerMode() const ;
    const char* getServerStatus() const ;
    const char* getPathStateName() const ;

  protected:
    const char *logprefix;
    bool handleDebugMessage(ArRobotPacket *pkt);
};

#endif
