#ifndef OVIS_DEMO_H
#define OVIS_DEMO_H

#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include "KinovaTypes.h"

int (*initAPI)(void);
int (*getAPIVersion)(int[API_VERSION_COUNT]);
int (*refresDevicesList)(void);
int (*getDevices)(KinovaDevice[MAX_KINOVA_DEVICE], int&);
int (*setActiveDevice)(KinovaDevice);
int (*getGeneralInformations)(GeneralInformations&);
int (*setClientConfigurations)(ClientConfigurations);
int (*getQuickStatus)(QuickStatus&);

static void* API_command_lib;
static void* API_comm_lib;
static KinovaDevice devices_list_[MAX_KINOVA_DEVICE];
static int robot_type;

#endif  // OVIS_DEMO_H