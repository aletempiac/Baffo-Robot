#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include <pthread.h>
#include "config.h"

const char const *color[] = {"?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };

/****************************************************************************************************/
/*                                   GLOBAL VARIABLES                                               */
/****************************************************************************************************/
struct Position {
  float x;
  float y;
  int deg;
  int start_deg;
};

struct CornerAngles {
  int bl; //bottom left
  int tl; //top right
  int tr;
  int br;
};

struct Position pos;
uint8_t sn_ball;		  //tacho to throw the ball
uint8_t sn_lift;      //tacho to lift the ball
uint8_t sn_tacho[3]={DESC_LIMIT, DESC_LIMIT, DESC_LIMIT};  //2 tacho motors, 3rd one for closing the multi
uint8_t sn_gyro;      //gyroscope
uint8_t sn_us;        //ultrasonic distance sensor
uint8_t sn_touch;
uint8_t sn_color;
uint8_t sn_sonar;
uint8_t sn_mag;

FLAGS_T state;
char s[ 256 ];
int val;
float value;

pthread_mutex_t sem_gyro;
pthread_mutex_t sem_us;
pthread_t thread[2];

volatile int gyro_dir = 0.0;
volatile int us_dist = 0.0;
volatile int flag_kill = 0;

/****************************************************************************************************/
/*                                  FUNCTION PROTOTYPES                                             */
/****************************************************************************************************/

void sensors_init(void);
void rotate(int deg, uint8_t * sn);
void set_for_rotate(int deg, uint8_t *sn);
void rotate_action(int deg, uint8_t * sn);
void go_straight_cm(int cm, uint8_t * sn);
void tacho_wait_term(uint8_t motor);
void tacho_wait_ball(uint8_t motor);
float turn_speed(int deg);
float read_gyro(uint8_t sn_gyro);
int read_us(uint8_t sn_us);
void* gyro_thread(void* arg);
void* us_thread(void* arg);
float get_us_value();
void throwball(uint8_t sn, float powerfactor);
void liftball(uint8_t sn);
void update_corner_angles(struct CornerAngles *c_angles, struct Position pos);
void update_position(int movement, int degree_abs);
void alg_flow(uint8_t *sn, uint8_t sn_ball, uint8_t sn_lift, struct Position pos);
int simple_search();
float elliptic_search(uint8_t *sn, struct Position pos);
float elliptic_distance(int deg, float a, float b);
void kill_motor(uint8_t motor);
void multi_kill_motor(uint8_t *motors);
void kill_all(int sig_numb);
float min(float x, float y);
float max(float x, float y);

/****************************************************************************************************/
/****************************************************************************************************/

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
    //printf("State: %d\n", state);
	} while (state);
}

void tacho_wait_ball(uint8_t motor) {
	FLAGS_T state;
	do {
		get_tacho_state_flags(motor, &state);
    //printf("State: %d\n", state);
	} while (state>2);
  printf("out");
}

/* Functions to kill the motors, either single or multiple */
void kill_motor(uint8_t motor) {
	set_tacho_command_inx(motor, TACHO_STOP);
}

void multi_kill_motor(uint8_t *motors) {
	multi_set_tacho_command_inx(motors, TACHO_STOP );
}

float min(float x, float y){
  if(x<y){
    return x;
  } else {
    return y;
  }
}

float max(float x, float y){
  if(x>y){
    return x;
  } else {
    return y;
  }
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
  multi_set_tacho_command_inx(sn, TACHO_STOP);
  update_position(0, deg);
  return;
}

void rotate_action(int deg, uint8_t * sn) {
  int initial_rot, end_rot;
	float degree = AXE_WHEELS*(1.0*deg) / DIAM;

	multi_set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set ramp up & down speed at zero
	multi_set_tacho_ramp_up_sp(sn, MAX_SPEED*ROT_ADJ/7*CF_RAMP_UP);
	multi_set_tacho_ramp_down_sp(sn,MAX_SPEED*ROT_ADJ/7*CF_RAMP_DW);
	multi_set_tacho_speed_sp(sn, turn_speed(deg));

	// set the disp on the motors
	set_tacho_position_sp(sn[0], (int)(-0.9*degree));
	set_tacho_position_sp(sn[1], (int)(0.9*degree));

	// initialize the tacho
  Sleep(200);
  pthread_mutex_lock(&sem_gyro);
  initial_rot = gyro_dir;
  pthread_mutex_unlock(&sem_gyro);
	multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[0]);
	tacho_wait_term(sn[1]);
  Sleep(200);
  pthread_mutex_lock(&sem_gyro);
  end_rot = gyro_dir;
  pthread_mutex_unlock(&sem_gyro);
  printf("started at: %d  --- ended at: %d  --- rotate deg: %d\n",initial_rot,end_rot,deg);
  //fflush(stdout);
  if ((end_rot > initial_rot && deg>0) || (end_rot < initial_rot && deg<0)){
    rotate_action((initial_rot-end_rot+deg), sn);
  }
  else if (end_rot < initial_rot  && deg>0){
    rotate_action((initial_rot-end_rot+deg-360), sn);
  }else if(end_rot > initial_rot && deg<0){
    rotate_action((initial_rot-end_rot+deg+360), sn);
  }
  return;
}

float turn_speed(int deg){
  if(deg<0) deg=-deg;
  if(deg>90){
    return MAX_SPEED*ROT_ADJ/10;
  }else if(deg>20){
    return MAX_SPEED*ROT_ADJ/10;
  } else {
    return MAX_SPEED*ROT_ADJ/10;
  }
}

// return a float but use an integer to store the value,
// mantissa is always zero
float read_gyro (uint8_t sn_gyro){
	float value;

	if (!get_sensor_value0(sn_gyro, &value)){
		return 0;
	}
  return -value;
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

float get_us_value(){
  float dist;
  pthread_mutex_lock(&sem_us);
  dist=(float)us_dist;
  pthread_mutex_unlock(&sem_us);
  return dist;
}

void throwball(uint8_t sn, float powerfactor) {
	int deg = 60;
  int max_speed;
	// change the braking mode
  get_tacho_max_speed(sn, &max_speed);
  set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set the max speed
	set_tacho_speed_sp(sn, max_speed*powerfactor);
	// set ramp up & down speed
	set_tacho_ramp_up_sp(sn, max_speed*powerfactor*CF_RAMP_UP);
	set_tacho_ramp_down_sp(sn, max_speed*CF_RAMP_DW);
	// set the disp on the motors
	set_tacho_position_sp(sn, deg);
  set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
  tacho_wait_ball(sn);
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
  //tacho_wait_ball(sn);
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

  pos.x=START_POS_X;
  pos.y=START_POS_Y;
  pthread_mutex_lock(&sem_gyro);
  pos.start_deg=gyro_dir;                 //Not guaranteed that gyro_dir has the right value
  pthread_mutex_unlock(&sem_gyro);
  pos.deg=0;
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

void look_at_corners(uint8_t *sn, struct CornerAngles c_angles){
  float dist_r, dist_exp;
  FLAGS_T state1, state2;
  int deg_r, deg_init;
  int rot;
  //rotate(c_angles.bl, sn_tacho);
  rot=AXE_WHEELS*(360.0) / DIAM;
  multi_set_tacho_stop_action_inx(sn_tacho, TACHO_BRAKE);
	// set ramp up & down speed at zero
	multi_set_tacho_ramp_up_sp(sn_tacho, 0);
	multi_set_tacho_ramp_down_sp(sn_tacho,0);
	multi_set_tacho_speed_sp(sn_tacho, turn_speed(360));

	// set the disp on the motors
	set_tacho_position_sp(sn_tacho[0], (int)(-rot));
	set_tacho_position_sp(sn_tacho[1], (int)(rot));
  pthread_mutex_lock(&sem_gyro);
  deg_init = gyro_dir;
  pthread_mutex_unlock(&sem_gyro);
  multi_set_tacho_command_inx(sn_tacho, TACHO_RUN_TO_REL_POS);
  do {
		get_tacho_state_flags(sn_tacho[0], &state1);
    get_tacho_state_flags(sn_tacho[1], &state2);
    pthread_mutex_lock(&sem_us);
    dist_r = us_dist;
    pthread_mutex_unlock(&sem_us);
    pthread_mutex_lock(&sem_gyro);
    deg_r = gyro_dir-deg_init;
    pthread_mutex_unlock(&sem_gyro);
    printf("Deg:%d, Dist:%.2f\n", deg_r, dist_r);
  } while(state1 || state2);
}

int simple_search(){
  float x, y, radius, dist;
  int initial_rot;
  float degree = AXE_WHEELS*(-180.0) / DIAM;
  int found=0;
  FLAGS_T state1, state2;
  //supposed to be perpendicular to the basket corner
  Sleep(200);
  pthread_mutex_lock(&sem_us);
  y = us_dist;
  pthread_mutex_unlock(&sem_us);
  rotate(90, sn_tacho);
  Sleep(200);
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
	multi_set_tacho_command_inx(sn_tacho, TACHO_RUN_TO_REL_POS);
  //scanning start
	do {
		get_tacho_state_flags(sn_tacho[0], &state1);
    get_tacho_state_flags(sn_tacho[1], &state2);
    pthread_mutex_lock(&sem_us);
    dist = us_dist;
    pthread_mutex_unlock(&sem_us);
    printf("%.2f\n", dist);
    if(dist<(radius-3) && dist!=321 && dist!=328) {
      found++;
      if(found==6){
        multi_set_tacho_command_inx(sn_tacho, TACHO_STOP);

        //set_tacho_command_inx(sn_tacho[1], TACHO_STOP);
        printf("Distance is: %.2f\n", dist);
        Sleep(200);
        pthread_mutex_lock(&sem_gyro);
        degree = gyro_dir-initial_rot+90;
        printf("stop degree:%.2f\n", degree);
        pthread_mutex_unlock(&sem_gyro);
        update_position(0, degree-pos.deg);
        return dist;
      }
    }
  } while(state1 || state2);
  return 0;
}

void return_to_center(int distance, uint8_t *sn){
  go_straight_cm(-distance, sn_tacho);
  printf("pos.deg: %d\n", pos.deg);
  if(pos.deg>180) {
    update_position(0, +2);
    rotate((360-pos.deg), sn_tacho);
  } else {
    update_position(0, -9);
    rotate(-pos.deg, sn_tacho);
  }
  return;
}

void set_for_rotate(int deg, uint8_t *sn){
  multi_set_tacho_stop_action_inx(sn_tacho, TACHO_BRAKE);
	// set ramp up & down speed at zero
	multi_set_tacho_ramp_up_sp(sn_tacho, turn_speed(deg)*CF_RAMP_UP);
	multi_set_tacho_ramp_down_sp(sn_tacho,turn_speed(-180)*CF_RAMP_DW);
	multi_set_tacho_speed_sp(sn_tacho, turn_speed(deg));
	// set the disp on the motors
	set_tacho_position_sp(sn_tacho[0], (int)(-deg));
	set_tacho_position_sp(sn_tacho[1], (int)(deg));
  return;
}

float elliptic_search(uint8_t *sn, struct Position pos){
  float distance;             //distance of the object found
  float a, b, radius, dist;   //elliptic parameters
  float x, y;                 //distances sampled by us sensor
  int initial_rot;
  int right_left;              //defines if we are in right or left area
  float degree = AXE_WHEELS*(-180.0) / DIAM; //setting range as 180
  int found=0;  //object found

  Sleep(200);
  y=get_us_value(); //value in axe y
  //need to turn of 90 degrees to sample x value;
  rotate(90, sn);
  Sleep(200);
  x=get_us_value(); //value in axe x
  //Need to verify correctness of the value w.r.t current position
  //if(FIELD_WIDTH-pos.x>8 ||
}

float elliptic_distance(int deg, float a, float b){
  float distance;
  //a is the distance that the robot is facing when the robot starts the scan
  distance=(a*b)/(sqrt(a*a*sin(PI/180*deg)*sin(PI/180*deg)+b*b*cos(PI/180*deg)*cos(PI/180*deg)));
  return distance;
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
		pthread_cancel(thread[0]);
		pthread_cancel(thread[1]);
    pthread_mutex_destroy(&sem_us);
    pthread_mutex_destroy(&sem_gyro);
		/* Uninit sensor */
		ev3_uninit();
    kill(getpid(), SIGTERM);
    //EXIT_FAILURE;
	}
}

//this is the function that search a ball and score
void alg_flow(uint8_t *sn_tacho, uint8_t sn_ball, uint8_t sn_lift, struct Position pos){
  int dist;
  int balls;
  //at start robot has too balls
  //go forward to 3 points line and score
  go_straight_cm(25, sn_tacho);
  throwball(sn_ball, 0.5);
  //now lift ready ball
  liftball(sn_lift);
  Sleep(2000);
  throwball(sn_ball, 0.5);
  balls=2;
  //hopefully send 6 points scored message
  //TO BE IMPLEMENTED
  //Scanning phase
  //scan fron area
  dist=simple_search();
  if(dist>0){
    //ball found (can be also a miss in the throw)
    //if distance is > 20 cm go even closer and search again
    if(dist<20){
      go_straight_cm(dist*1.0/10-8, sn_tacho);
      //There is a ball?
      //check with color sensor or distance sensor
      if(get_us_value()<=10){
        //ball near enought
        liftball(sn_lift);
        go_straight_cm(-dist*1.0/10-8, sn_tacho);
        return_to_center(dist, sn_tacho);
        throwball(sn_ball, 0.8);
        balls++;
        //send scored
      }else{
        go_straight_cm(-dist*1.0/10-8, sn_tacho);
        return_to_center(dist, sn_tacho);
        //probably wrong or simple search from that position
        //rotate(-(pos.deg-180));
        //simple_search();
      }
    } else{
      //go near that area and search again
      //go_straight_cm(15, sn_tacho);
      //return_to_center(distance, sn_tacho);
    }
  }
}

/*****************************************MAIN**********************************************/

int main( void ) {
  struct CornerAngles c_angles;
  int t_ret1, t_ret2;
  int i;
  int dist;

#ifndef __ARM_ARCH_4T__
  /* Disable auto-detection of the brick (you have to set the correct address below) */
  //ev3_brick_addr = "192.168.0.204";
#endif

  if ( ev3_init() == -1 ) return 1;

#ifndef __ARM_ARCH_4T__
  //printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );
#else
#endif
  if (signal(SIGINT, kill_all) == SIG_ERR)
      printf("Kill signal handler not set\n");
  //initialize sensors
	sensors_init();
  printf("In main\n");

	//Gyroscope and US Sensor Thread and Mutex creations
  pthread_mutex_init(&sem_gyro, NULL);
  pthread_mutex_init(&sem_us, NULL);

  t_ret1=pthread_create(&thread[0], NULL, &gyro_thread, (void*)(&sn_gyro));
  t_ret2=pthread_create(&thread[1], NULL, &us_thread, (void*)(&sn_us));

  if(t_ret1) {
    fprintf(stderr,"Error - pthread_create() gyro_thread return code: %d\n", t_ret1);
    exit(EXIT_FAILURE);
  }
  if(t_ret2) {
    fprintf(stderr,"Error - pthread_create() us_thread return code: %d\n", t_ret1);
    exit(EXIT_FAILURE);
  }

  //Initial setup
  //Sleep(1000);
/*
  go_straight_cm(10, sn_tacho);
  liftball(sn_lift);
  Sleep(1000);
	throwball(sn_ball, 2);
*/
/*
  printf("Position x:%.2f y:%.2f deg:%d deg_abs:%d\n", pos.x, pos.y, pos.deg, pos.start_deg);
  //update_corner_angles(&c_angles, pos);
  printf("bl:%d\ntl:%d\ntr:%d\nbr:%d\n", c_angles.bl, c_angles.tl, c_angles.tr, c_angles.br);
  //Sleep(3000);
  //rotate(-c_angles.bl, sn_tacho);
  printf("bl:%d\ntl:%d\ntr:%d\nbr:%d\n", c_angles.bl, c_angles.tl, c_angles.tr, c_angles.br);
  printf("Position x:%.2f y:%.2f deg:%d deg_abs:%d\n", pos.x, pos.y, pos.deg, pos.start_deg);
*/
  //go_straight_cm(-40, sn_tacho);
  //look_at_corners(sn_tacho, c_angles);
  //rotate(180, sn_tacho);
  //throwball(sn_ball, 0.8);
  alg_flow(sn_tacho, sn_ball, sn_lift, pos);
  //rotate(-90, sn_tacho);
  //rotate(180, sn_tacho);
/*  dist=simple_search();


  if(dist>0){
    go_straight_cm(dist*1.0/10-8, sn_tacho);
    //simple_search();
    liftball(sn_lift);
    Sleep(2000);
    return_to_center(dist*1.0/10-8, sn_tacho);
    throwball(sn_ball, 1.5);
  }
*/
/*
  for(;;){
    pthread_mutex_lock(&sem_us);
    printf("%d\n", us_dist);
    pthread_mutex_unlock(&sem_us);
    Sleep(100);
  }
*/
  //flag_kill = 1;

  //pthread_join(thread[1],NULL);
  pthread_cancel(thread[0]);
  pthread_cancel(thread[1]);
  pthread_mutex_destroy(&sem_gyro);
  pthread_mutex_destroy(&sem_us);

  ev3_uninit();

  printf( "*** ( EV3 ) Bye! ***\n" );

  return ( 0 );
}
