#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif
const char const *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))
#define PI 3.141592653589793
#define DIAM 5.6
#define RAD 2.8
#define CF_RAMP_UP 2/3
#define CF_RAMP_DW 2/3
#define MAX_SPEED 1050
#define AXE_WHEELS 12
#define ROT_ADJ 3/5


static bool _check_pressed( uint8_t sn )
{
  int val;

  if ( sn == SENSOR__NONE_ ) {
    return ( ev3_read_keys(( uint8_t *) &val ) && ( val & EV3_KEY_UP ));
  }
  return ( get_sensor_value( 0, sn, &val ) && ( val != 0 ));
}


void tacho_wait_term(uint8_t motor) {
	FLAGS_T state;
	do {
		get_tacho_state_flags(motor, &state);
	} while (state);
}

void go_straight_cm(int cm, uint8_t * sn) {
	int max_speed[2];
	// set the relative rad displacement in order to reach the correct displacement in cm
	float deg = 360 * cm / (PI * DIAM);
	
	get_tacho_max_speed(sn[0], &max_speed[0]);
	//get_tacho_max_speed(sn[1], &max_speed[1]);
	// change the braking mode
	multi_set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set the max speed
	multi_set_tacho_speed_sp(sn, MAX_SPEED*2/3);
	// set ramp up & down speed
	multi_set_tacho_ramp_up_sp(sn, MAX_SPEED*CF_RAMP_UP);
	multi_set_tacho_ramp_down_sp(sn, MAX_SPEED*CF_RAMP_DW);
	// set the disp on the motors
	multi_set_tacho_position_sp(sn, deg);
	// initialize the tacho
	multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[0]);
	tacho_wait_term(sn[1]);
}


void go_backwards_cm(int cm, uint8_t * sn) {
	int max_speed[2];
	// set the relative rad displacement in order to reach the correct displacement in cm
	float deg = 360 * cm / (PI * DIAM);
	
	get_tacho_max_speed(sn[0], &max_speed[0]);
	//get_tacho_max_speed(sn[1], max_speed[1]);
	// change the braking mode
	multi_set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set the max speed
	multi_set_tacho_speed_sp(sn, max_speed[0]*2/3);
	// set ramp up & down speed
	multi_set_tacho_ramp_up_sp(sn, max_speed[0]*CF_RAMP_UP);
	multi_set_tacho_ramp_down_sp(sn, max_speed[0]*CF_RAMP_DW);
	// set the disp on the motors
	multi_set_tacho_position_sp(sn, -deg);
	// initialize the tacho
	multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[0]);
	tacho_wait_term(sn[1]);
}


void rotate_dx(int deg, uint8_t * sn) {
	float degree = AXE_WHEELS*(1.0*deg) / DIAM;
	multi_set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set ramp up & down speed at zero
	multi_set_tacho_ramp_up_sp(sn, 0);
	multi_set_tacho_ramp_down_sp(sn, 0);
	multi_set_tacho_speed_sp(sn, MAX_SPEED*ROT_ADJ*4/9);
	// set the disp on the motors
	//set_tacho_position_sp(sn[1], (int)(degree*0.9));
	//set_tacho_position_sp(sn[0], (int)(-degree*0.9));
	set_tacho_position_sp(sn[1], (int)(degree));
	set_tacho_position_sp(sn[0], (int)(-degree));
	
	// initialize the tacho
	multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[1]);
	tacho_wait_term(sn[0]);
}

void rotate_sx(int deg, uint8_t * sn) {
	float degree = AXE_WHEELS*(1.0*deg) / DIAM;
	multi_set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set ramp up & down speed at zero
	multi_set_tacho_ramp_up_sp(sn, 0);
	multi_set_tacho_ramp_down_sp(sn,0);
	multi_set_tacho_speed_sp(sn, MAX_SPEED*ROT_ADJ*4/9);
	
	// set the disp on the motors
	set_tacho_position_sp(sn[0], (int)(degree));
	set_tacho_position_sp(sn[1], (int)(-degree));
	
	// initialize the tacho
	multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[0]);
	tacho_wait_term(sn[1]);
}	

int read_gyro (){
	if ( ev3_sensor[ i ].type_inx == GYRO-ANG ) {
      			printf( "  type = %s\n", ev3_sensor_type( ev3_sensor[ i ].type_inx ));
      			printf( "  port = %s\n", ev3_sensor_port_name( i, s ));
      				if ( get_sensor_mode( i, s, sizeof( s ))) {
        				printf( "  mode = %s\n", s );
      				}
      				if ( get_sensor_num_values( i, &n )) {
        				for ( ii = 0; ii < n; ii++ ) {
          					if ( get_sensor_value( ii, i, &val )) {
            						printf( "  value%d = %d\n", ii, val );
          					}
        				}
      				}
   	 	}


int main( void )
{
  int i;
  uint8_t sn[2];
  FLAGS_T state;
  uint8_t sn_touch;
  uint8_t sn_color;
  uint8_t sn_compass;
  uint8_t sn_sonar;
  uint8_t sn_mag;
  char s[ 256 ];
  int val;
  float value;
  uint32_t n, ii;
#ifndef __ARM_ARCH_4T__
  /* Disable auto-detection of the brick (you have to set the correct address below) */
  //ev3_brick_addr = "192.168.0.204";

#endif
  if ( ev3_init() == -1 ) return ( 1 );

#ifndef __ARM_ARCH_4T__
  //printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );

#else
  
#endif
  while ( ev3_tacho_init() < 1 ) Sleep( 1000 );
  
  printf( "Waiting tacho is plugged...\n" );
  
  ev3_sensor_init();
  
  printf( "Sensor OK...\n" );
  
  ev3_search_tacho_plugged_in(65,0, &sn[0], 0);
  ev3_search_tacho_plugged_in(68,0, &sn[1], 0);
  

  //Run motors in order from port A to D
 	
	//go_backwards_cm(29, sn);
	//go_straight_cm(58, sn);
	for(int j=0; j<2; j++) {
		rotate_dx(180, sn);
		Sleep(500);
		rotate_sx(180, sn);
	}
	
	for ( i = 0; i < DESC_LIMIT; i++ ) {
    		if ( ev3_sensor[ i ].type_inx != SENSOR_TYPE__NONE_ ) {
      			printf( "  type = %s\n", ev3_sensor_type( ev3_sensor[ i ].type_inx ));
      			printf( "  port = %s\n", ev3_sensor_port_name( i, s ));
      				if ( get_sensor_mode( i, s, sizeof( s ))) {
        				printf( "  mode = %s\n", s );
      				}
      				if ( get_sensor_num_values( i, &n )) {
        				for ( ii = 0; ii < n; ii++ ) {
          					if ( get_sensor_value( ii, i, &val )) {
            						printf( "  value%d = %d\n", ii, val );
          					}
        				}
      				}
   	 	}
  	}
	
  //Run all sensors
  
 

  ev3_uninit();
  printf( "*** ( EV3 ) Bye! ***\n" );

  return ( 0 );
}

