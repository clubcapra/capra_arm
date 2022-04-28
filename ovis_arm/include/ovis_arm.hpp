#ifndef OVIS_ARM_H
#define OVIS_ARM_H

#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include "Kinova.API.EthCommandLayerUbuntu.h"
#include "KinovaTypes.h"

int (*ethernet_initAPI)(void);
int (*ethernet_getAPIVersion)(int[API_VERSION_COUNT]);
int (*ethernet_refresDevicesList)(void);
int (*ethernet_getDevices)(KinovaDevice[MAX_KINOVA_DEVICE], int&);
int (*ethernet_setActiveDevice)(KinovaDevice);
int (*ethernet_getGeneralInformations)(GeneralInformations&);
int (*ethernet_setClientConfigurations)(ClientConfigurations);
int (*ethernet_getQuickStatus)(QuickStatus&);

int (*ethernet_getAngularPosition)(AngularPosition&);
int (*ethernet_setAngularControl)();
int (*ethernet_sendAdvanceTrajectory)(TrajectoryPoint);
int (*ethernet_sendBasicTrajectory)(TrajectoryPoint command);
int (*ethernet_setEthernetConfiguration)(EthernetCommConfig & config);


int (*initAPI)(void);
int (*getAPIVersion)(int[API_VERSION_COUNT]);
int (*refresDevicesList)(void);
int (*getDevices)(KinovaDevice[MAX_KINOVA_DEVICE], int&);
int (*setActiveDevice)(KinovaDevice);
int (*getGeneralInformations)(GeneralInformations&);
int (*setClientConfigurations)(ClientConfigurations);
int (*getQuickStatus)(QuickStatus&);

int (*getAngularPosition)(AngularPosition&);
int (*setAngularControl)();
int (*sendAdvanceTrajectory)(TrajectoryPoint);
int (*sendBasicTrajectory)(TrajectoryPoint command);

static void* API_command_lib;
static void* API_comm_lib;
static KinovaDevice devices_list_[MAX_KINOVA_DEVICE];
static int robot_type;

#endif  // OVIS_ARM_H