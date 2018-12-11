#ifndef CONFIG_H
#define CONFIG_H
/******************************************************************************/
/*                                    DEFINE                                  */
/******************************************************************************/

#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))
#define PI 3.141592653589793
#define DIAM 5.6
#define RAD 2.8
#define CF_RAMP_UP 2/3
#define CF_RAMP_DW 2/3
#define MAX_SPEED 1050
#define AXE_WHEELS 12
#define ROT_ADJ 3.0/5
#define ROBOT_WIDTH 12
#define ROBOT_LENGTH 30
#define ROBOT_SENSOR_DIST 6
#define FIELD_WIDTH 120 - ROBOT_WIDTH/2
#define FIELD_LENGTH 100 - ROBOT_LENGTH/2
#define START_POS_X 0
#define START_POS_Y 0
#define ANGLE_X FIELD_WIDTH/2-ROBOT_SENSOR_DIST
#define ANGLE_Y FIELD_LENGTH-10-ROBOT_SENSOR_DIST-ROBOT_LENGTH/2

// WIN32 /////////////////////////////////////////
#ifdef __WIN32__
#include <windows.h>
// UNIX //////////////////////////////////////////
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
//////////////////////////////////////////////////
#endif

#endif /* CONFIG_H */
