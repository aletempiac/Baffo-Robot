#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "lib.h"
#include <pthread.h>
#include "config.h"
/* Function prototypes */

void tacho_wait_term(uint8_t motor) {
	FLAGS_T state;
	do {
		get_tacho_state_flags(motor, &state);
    //printf("State: %d\n", state);
	} while (state);
}

void tacho_wait_ball(uint8_t motor) {
	FLAGS_T state;
	do {
		get_tacho_state_flags(motor, &state);
    //printf("State: %d\n", state);
	} while (state!=2);
}

/* Functions to kill the motors, either single or multiple */
void kill_motor(uint8_t motor) {
	set_tacho_command_inx(motor, TACHO_STOP);
}

void multi_kill_motor(uint8_t *motors) {
	multi_set_tacho_command_inx(motors, TACHO_STOP );
}


void go_straight_cm(int cm, uint8_t *sn) {
	// set the relative rad displacement in order to reach the correct displacement in cm
	float deg = 360 * cm / (PI * DIAM);
	//get_tacho_max_speed(sn[1], &max_speed[1]);
	// change the braking mode
	multi_set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set the max speed
	set_tacho_speed_sp(sn[0], MAX_SPEED/5);
  set_tacho_speed_sp(sn[1], MAX_SPEED/5);
	// set ramp up & down speed
	multi_set_tacho_ramp_up_sp(sn, MAX_SPEED/5*CF_RAMP_UP);
  multi_set_tacho_ramp_down_sp(sn, MAX_SPEED/5*CF_RAMP_DW);
	// set the disp on the motors
	multi_set_tacho_position_sp(sn, deg);
	// initialize the tacho
  multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[0]);
	tacho_wait_term(sn[1]);
  update_position(cm, 0);
}
void rotate(int deg, uint8_t * sn) {
  rotate_action(deg, sn);
  update_position(0, deg);
  return;
}

void rotate_action(int deg, uint8_t * sn) {
  int initial_rot, end_rot;
	float degree = AXE_WHEELS*(1.0*deg) / DIAM;

	multi_set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set ramp up & down speed at zero
	multi_set_tacho_ramp_up_sp(sn, 0);
	multi_set_tacho_ramp_down_sp(sn,0);
	multi_set_tacho_speed_sp(sn, turn_speed(deg));

	// set the disp on the motors
	set_tacho_position_sp(sn[0], (int)(-0.9*degree));
	set_tacho_position_sp(sn[1], (int)(0.9*degree));

	// initialize the tacho
  initial_rot = gyro_dir;
	multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[0]);
	tacho_wait_term(sn[1]);
  Sleep(200);
  pthread_mutex_lock(&sem_gyro);
  end_rot = gyro_dir;
  pthread_mutex_unlock(&sem_gyro);
  //printf("started at: %d  --- ended at: %d  --- rotate deg: %d\n",initial_rot,end_rot,deg);
  //fflush(stdout);
  if ((end_rot > initial_rot && deg>0) || (end_rot < initial_rot && deg<0)){
    rotate((initial_rot-end_rot+deg), sn);
  }
  else if (end_rot < initial_rot  && deg>0){
    rotate((initial_rot-end_rot+deg-360), sn);
  }else if(end_rot > initial_rot && deg<0){
    rotate((initial_rot-end_rot+deg+360), sn);
  }
  return;
}

float turn_speed(int deg){
  if(deg<0) deg=-deg;
  if(deg>90){
    return MAX_SPEED*ROT_ADJ/14;
  }else if(deg>20){
    return MAX_SPEED*ROT_ADJ/14;
  } else {
    return MAX_SPEED*ROT_ADJ/14;
  }
}

// return a float but use an integer to store the value,
// mantissa is always zero
float read_gyro (uint8_t sn_gyro){
	float value;

	if (!get_sensor_value0(sn_gyro, &value)){
		return 0;
	}
  return value;
}


void *gyro_thread(void* arg){
 	//Sleep(10);
 	uint8_t* gyro = (uint8_t*) arg;
 	printf("T1: started gyro thread: %d\n", *gyro);
 	while(flag_kill==0) {
 		pthread_mutex_lock(&sem_gyro);
 		gyro_dir = ((((int)(read_gyro(*gyro)) % 360) + 360) % 360);
 		pthread_mutex_unlock(&sem_gyro);
 	}
 	pthread_exit(0);
}

int read_us(uint8_t sn_us){
  int value;

  if (!get_sensor_value(0, sn_us, &value)){
		return 0;
	}
  return value;
}

void *us_thread(void* arg){
 	uint8_t* us = (uint8_t*) arg;
 	printf("T1: started gyro thread: %d\n", *us);
 	while(flag_kill==0) {
 		pthread_mutex_lock(&sem_us);
 		us_dist = read_us(*us);
 		pthread_mutex_unlock(&sem_us);
 	}
 	pthread_exit(0);
}

void throwball(uint8_t sn, int divisionfactor) {
	int deg = 60;
  int max_speed;
	// change the braking mode
  get_tacho_max_speed(sn, &max_speed);
  set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set the max speed
	set_tacho_speed_sp(sn, max_speed/divisionfactor);
	// set ramp up & down speed
	set_tacho_ramp_up_sp(sn, max_speed/divisionfactor/2*CF_RAMP_UP);
	set_tacho_ramp_down_sp(sn, CF_RAMP_DW);
	// set the disp on the motors
	set_tacho_position_sp(sn, deg);
  set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
  tacho_wait_term(sn);
  //return to initial position
  set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set the max speed
	set_tacho_speed_sp(sn, max_speed);
	// set ramp up & down speed
	set_tacho_ramp_up_sp(sn, max_speed*CF_RAMP_UP);
	set_tacho_ramp_down_sp(sn, 0);
	// set the disp on the motors
	set_tacho_position_sp(sn, -deg);
  set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
  tacho_wait_term(sn);
}

void liftball(uint8_t sn) {
	int deg = -360;
  int max_speed;
	// change the braking mode
  get_tacho_max_speed(sn, &max_speed);
  set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set the max speed
	set_tacho_speed_sp(sn, max_speed);
	// set ramp up & down speed
	set_tacho_ramp_up_sp(sn, max_speed*CF_RAMP_UP);
	set_tacho_ramp_down_sp(sn, 0);
	// set the disp on the motors
	set_tacho_position_sp(sn, deg);
  set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
  tacho_wait_ball(sn);
  //return to abs pos
  set_tacho_position_sp(sn, 0);
  set_tacho_speed_sp(sn, max_speed/6);
  set_tacho_ramp_up_sp(sn, max_speed/6*CF_RAMP_UP);
	set_tacho_ramp_down_sp(sn, max_speed/6*CF_RAMP_UP);
  set_tacho_command_inx(sn, TACHO_RUN_TO_ABS_POS);
}

void sensors_init(){
	printf( "Sensors_init: Waiting tachos are plugged...\n" );
  //Tacho Motors Initialization
  while (ev3_tacho_init() < 4) Sleep(500);
  ev3_search_tacho_plugged_in(65,0, &sn_ball, 0);
  set_tacho_stop_action_inx(sn_ball, TACHO_BRAKE);
	set_tacho_command_inx(sn_ball, TACHO_STOP );

  ev3_search_tacho_plugged_in(66,0, &sn_lift, 0);
  set_tacho_stop_action_inx(sn_lift, TACHO_BRAKE);
	set_tacho_command_inx(sn_lift, TACHO_STOP );

  ev3_search_tacho_plugged_in(67,0, &sn_tacho[0], 0);
  set_tacho_stop_action_inx(sn_tacho[0], TACHO_BRAKE);
	set_tacho_command_inx(sn_tacho[0], TACHO_STOP );

  ev3_search_tacho_plugged_in(68,0, &sn_tacho[1], 0);
  set_tacho_stop_action_inx(sn_tacho[1], TACHO_BRAKE);
	set_tacho_command_inx(sn_tacho[1], TACHO_STOP );


  //Sensors Initialization
  printf( "Sensors_init: Waiting sensors are plugged...\n" );
  while (ev3_sensor_init() < 2) Sleep(500);
	ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0);
  ev3_search_sensor(LEGO_EV3_US, &sn_us,0);
	//printf("Sensors_init: gyro ID %d\n",sn_gyro);
	set_sensor_mode_inx(sn_gyro, GYRO_GYRO_ANG);
  printf( "Sensors init completed!\n" );
}

void update_corner_angles(struct CornerAngles *c_angles, struct Position pos){
  c_angles->bl=-90-180/PI*atan((ANGLE_Y+FIELD_LENGTH+10.0+pos.y)/(ANGLE_X+pos.x));
  c_angles->tl=180/PI*-atan((ANGLE_X+pos.x)/(ANGLE_Y-pos.y));
  c_angles->tr=180/PI*atan((ANGLE_X-pos.x)/(ANGLE_Y-pos.y));
  c_angles->br=90+180/PI*atan((ANGLE_Y+FIELD_LENGTH+10.0+pos.y)/(ANGLE_X-pos.x));
  return;
}

void update_position(int movement, int degree){
  pos.deg=(pos.deg+degree+360)%360;
  pos.x+=movement*sin(PI*pos.deg/180);
  pos.y+=movement*cos(PI*pos.deg/180);
  printf("Position x:%.2f y:%.2f deg:%d deg_abs:%d\n", pos.x, pos.y, pos.deg, pos.start_deg);
}

int simple_search(){
  float x, y, radius, dist;
  int initial_rot;
  float degree = AXE_WHEELS*(-180.0) / DIAM;
  int found=0;
  FLAGS_T state1, state2;
  //supposed to be perpendicular to the basket corner
  Sleep(100);
  pthread_mutex_lock(&sem_us);
  y = us_dist;
  pthread_mutex_unlock(&sem_us);
  rotate(90, sn_tacho);
  Sleep(100);
  pthread_mutex_lock(&sem_us);
  x = read_us(sn_us);
  pthread_mutex_unlock(&sem_us);
  if(x<y) {
    radius=x;
  } else {
    radius=y;
  }

	multi_set_tacho_stop_action_inx(sn_tacho, TACHO_BRAKE);
	// set ramp up & down speed at zero
	multi_set_tacho_ramp_up_sp(sn_tacho, 0);
	multi_set_tacho_ramp_down_sp(sn_tacho,0);
	multi_set_tacho_speed_sp(sn_tacho, turn_speed(-180));

	// set the disp on the motors
	set_tacho_position_sp(sn_tacho[0], (int)(-degree));
	set_tacho_position_sp(sn_tacho[1], (int)(degree));
  printf("The radius is: %.2f\n", radius);
  fflush(stdout);
  pthread_mutex_lock(&sem_gyro);
  initial_rot = gyro_dir;
  pthread_mutex_unlock(&sem_gyro);
	set_tacho_command_inx(sn_tacho[0], TACHO_RUN_TO_REL_POS);
  set_tacho_command_inx(sn_tacho[1], TACHO_RUN_TO_REL_POS);
  //scanning start
	do {
		get_tacho_state_flags(sn_tacho[0], &state1);
    get_tacho_state_flags(sn_tacho[1], &state2);
    pthread_mutex_lock(&sem_us);
    dist = us_dist;
    pthread_mutex_unlock(&sem_us);
    printf("%.2f\n", dist);
    if(dist<(radius-3) && dist!=326 && dist!=321 && dist!=328) {
      found++;
      if(found==4){
        multi_set_tacho_command_inx(sn_tacho, TACHO_STOP);

        //set_tacho_command_inx(sn_tacho[1], TACHO_STOP);
        printf("Distance is: %.2f\n", dist);
        pthread_mutex_lock(&sem_gyro);
        degree = (gyro_dir-initial_rot+90+360)%360;
        printf("stop degree:%.2f\n", degree);
        pthread_mutex_unlock(&sem_gyro);
        update_position(0, degree-pos.deg);
        return dist;
      }
    }
  } while(state1 || state2);
  return 0;
}


/* Signal handler */
void kill_all(int sig_numb){
	if (sig_numb == SIGINT) {
		printf("Handling signal, killing all\n");
		flag_kill=1;
		multi_kill_motor(sn_tacho);
		kill_motor(sn_lift);
		kill_motor(sn_ball);

		/* Destroy mutex & threads */
		pthread_mutex_destroy(&sem_us);
    pthread_mutex_destroy(&sem_gyro);
		pthread_cancel(thread[0]);
		pthread_cancel(thread[1]);
		/* Uninit sensor */
		ev3_uninit();
    kill(getpid(), SIGINT);
		}
	}
