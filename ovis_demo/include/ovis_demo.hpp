#ifndef OVIS_DEMO_H
#define OVIS_DEMO_H

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"

// << ---- K I N O V A   D L ---- >>
static int (*MyInitAPI)();
static int (*MyCloseAPI)();
static int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
static int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int& result);
static int (*MyMoveHome)();
static int (*MyInitFingers)();
static int (*MyGetAngularCommand)(AngularPosition&);
static int (*MyEraseAllTrajectories)();
static int (*MyGetSensorsInfo)(SensorsInfo&);
// static int (*MySetActuatorMaxVelocity)(float &);
// static int (*MyGetActuatorsPosition)(float &);
// static int (*MyGetAngularVelocity)(float &);
// static int (*MyGetAngularTorqueCommand)(float[]  );
static int (*MyGetAngularForce)(AngularPosition& Response);

static void* kinovaHandle;

#endif  // OVIS_DEMO_H