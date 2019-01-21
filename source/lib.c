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
#include "lib.h"
#include "bt.h"

// If set is present
// Bit 0 = RUNNING
// Bit 1 = RAMPING
// Bit 2 = HOLDING
// Bit 3 = OVERLOADED
// Bit 4 = STALLED

void tacho_wait_term(uint8_t motor) {
	FLAGS_T state;
	do {
		get_tacho_state_flags(motor, &state);
    //printf("State: %x\n", state);
    // masking to se the running bit
	} while (state & 0x1);
}

int tacho_check_overload(uint8_t motor) {
	FLAGS_T state;

	do {
		get_tacho_state_flags(motor, &state);
    //printf("State: %x\n", state);
	} while ( (state & 0x1) && !(state & 0x10) );
	//return different from 0 if the motor is stuck in overload
  return ((int) state & 0x10);
}

/* Functions to kill the motors, either single or multiple */
void kill_motor(uint8_t motor) {
	set_tacho_command_inx(motor, TACHO_STOP);
}

void multi_kill_motor(uint8_t *motors) {
	multi_set_tacho_command_inx(motors, TACHO_STOP );
}

int min(int x, int y){
  if(x<y){
    return x;
  } else {
    return y;
  }
}

int max(int x, int y){
  if(x>y){
    return x;
  } else {
    return y;
  }
}

int go_straight_mm(int mm, uint8_t *sn, int check_area) {
	// set the relative rad displacement in order to reach the correct displacement in cm
	float deg = 36 * mm / (PI * DIAM);
	//get_tacho_max_speed(sn[1], &max_speed[1]);
	// change the braking mode
  if(check_area && check_in_area(mm, pos)){
    return 0; //movement makes the robot go ouitside his area
  }
	multi_set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set the max speed
	set_tacho_speed_sp(sn[0], 398);
  set_tacho_speed_sp(sn[1], 400);
	// set ramp up & down speed
	multi_set_tacho_ramp_up_sp(sn, 10);
  multi_set_tacho_ramp_down_sp(sn, 10);
	// set the disp on the motors
	multi_set_tacho_position_sp(sn, deg);
	// initialize the tacho
  multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[0]);
	tacho_wait_term(sn[1]);
  update_position(mm, 0);
  return 1;
}

int go_straight_fullsped(int mm, uint8_t *sn) {
	// set the relative rad displacement in order to reach the correct displacement in cm
	float deg = 36 * mm / (PI * DIAM);
	//get_tacho_max_speed(sn[1], &max_speed[1]);
	// change the braking mode
	multi_set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set the max speed
	set_tacho_speed_sp(sn[0], MAX_SPEED-50);
  set_tacho_speed_sp(sn[1], MAX_SPEED-50);
	// set ramp up & down speed
	multi_set_tacho_ramp_up_sp(sn, 20);
  multi_set_tacho_ramp_down_sp(sn, 20);
	// set the disp on the motors
	multi_set_tacho_position_sp(sn, deg);
	// initialize the tacho
  multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[0]);
	tacho_wait_term(sn[1]);
  update_position(mm, 0);
  return 1;
}

int check_in_area(int movement, struct Position pos){
  int x=abs(pos.x);
  int y=abs(pos.y);
  int movx=abs((int)movement*sin(PI*(float)pos.deg/180.0));
  int movy=movement*cos(PI*(float)pos.deg/180.0);
  //printf("posx %d posy %d movx %d movy %d\n", x,y,movx,movy );
  if(movx+x>FIELD_WIDTH/2){
    return 1;
  }
  if(movy>0 && movy+pos.y>FIELD_LENGTH_FRONT){
    return 2;
  }
  if(movy<0 && movy+pos.y<-FIELD_LENGTH_BACK){
    return 3;
  }
  return 0;
}

int rotate(int deg, uint8_t *sn){
  //returns number of degrees turned
  int initial_rot, end_rot;
	float degree = AXE_WHEELS*(1.0*deg) / DIAM;

  set_for_rotate(degree, sn);
	// initialize the tacho
  Sleep(200);
  initial_rot = read_gyro(sn_gyro, 5);
	multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[0]);
	tacho_wait_term(sn[1]);
  Sleep(200);
  end_rot = read_gyro(sn_gyro, 5);
  //printf("started at: %d  --- ended at: %d  --- rotate deg: %d\n",initial_rot,end_rot,deg);
  //update_position(0, end_rot-initial_rot);
  return end_rot-initial_rot;
}

void rotate_with_adjustment(int deg, uint8_t * sn) {
  //suggested when number of degrees to turn is high
  static int called = 0;
  called++;

  update_position(0, deg);

  if(called % 3 == 0){
    deg += 4*sign(deg);
  }
  rotate_action(deg, sn);
  multi_set_tacho_command_inx(sn, TACHO_STOP);
  return;
}

void rotate_action(int deg, uint8_t * sn) {
  int initial_rot, end_rot;
  float degree;

  if (deg==0) return;
	degree = AXE_WHEELS*(1.0*deg) / DIAM;

  //if(deg<=1 && deg>=-1) return;
  set_for_rotate(degree, sn);
  Sleep(200);
  initial_rot = read_gyro(sn_gyro, 5);
	multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[0]);
	tacho_wait_term(sn[1]);
  Sleep(200);
  end_rot = read_gyro(sn_gyro, 5);

  //printf("started at: %d  --- ended at: %d  --- rotate deg: %d\n",initial_rot,end_rot,deg);

  if(abs(deg)==1) return;
  //recursive call
  if ((end_rot > initial_rot && deg>0) || (end_rot < initial_rot && deg<0)){
    deg=initial_rot-end_rot+deg;
    if(deg>180) deg-=360;
    rotate_action(deg, sn);
  }
  else if (end_rot < initial_rot  && deg>0){
    deg=initial_rot-end_rot+deg-360;
    if(deg>180) deg-=360;
    rotate_action(deg, sn);
  }else if(end_rot > initial_rot && deg<0){
    deg=initial_rot-end_rot+deg+360;
    if(deg>180) deg-=360;
    rotate_action(deg, sn);
  }
  return;
}

void rotate_with_slowdown(int deg, uint8_t * sn) {
  /* TODO: testing */
	int initial_rot, end_rot;
	deg=((deg + 180) % 360 + 360) % 360 - 180; //from -180 to 179
	float degree = AXE_WHEELS*(1.0*deg) / DIAM;
	float partial_degree=degree-degree/10;

	if (deg>-10 && deg<10) {
		rotate_with_adjustment(deg, sn);
		return;
	}
	set_for_rotate(partial_degree, sn);
	// initialize the tacho
  Sleep(200);
  initial_rot = read_gyro(sn_gyro, 5)-180; //from -180 to 179
	end_rot=((initial_rot+deg)+360)%360-180; //from -180 to 179
	multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn[0]);
	//tacho_wait_term(sn[1]);
	// decellerate
	set_for_rotate(degree/5, sn);
	multi_set_tacho_speed_sp(sn_tacho, MAX_SPEED/15);
	multi_set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	if (deg>0) {
		while((read_gyro(sn_gyro, 5)-180) < end_rot);
	} else {
		while((read_gyro(sn_gyro, 5)-180) > end_rot);
	}
	multi_kill_motor(sn);
  return;
}

int turn_speed(int deg){
  if(abs(deg)>130){
    return 170;
  }else if(abs(deg)>50){
    return 145;
  }else if(abs(deg)>=5){
    return 100;
  }else{
    return 70;
  }
}

// return a float but use an integer to store the value,
// mantissa is always zero
int read_gyro (uint8_t sn_gyro, int n){
	float value;
  float sum = 0;
  for (int i=0; i<n; i++){
	   if (!get_sensor_value0(sn_gyro, &value)){
		   return 0;
	    }
      sum += value;
  }
  return (((((int)sum)/n %360) + 360) % 360);
}

int read_us(uint8_t sn_us, int n){
  float value;
  int sum = 0;
  for (int i=0; i<n; i++){
    if (!get_sensor_value0(sn_us, &value)){
		    return 0;
    }
    sum += (int) value;
  }
  return sum/n;
}

void return_to_zero( uint8_t sn, int max_speed ){
	printf("Return_to_zero\n");
  set_tacho_position_sp(sn, 0);
  // set the max speed
  set_tacho_speed_sp(sn, max_speed >> 4);
  // set ramp up & down speed
  set_tacho_ramp_up_sp(sn, 10);
  set_tacho_ramp_down_sp(sn, 10);
  set_tacho_command_inx(sn, TACHO_RUN_TO_ABS_POS);
  tacho_wait_term(sn);

  return;
}

void start_throwball(uint8_t sn){
	int deg = 100;
  int max_speed;
  int ball_present = 0;
  // change the braking mode

  get_tacho_max_speed(sn, &max_speed);
  set_tacho_stop_action_inx(sn, TACHO_BRAKE);
  // set the max speed
  set_tacho_speed_sp(sn, max_speed);
  // set ramp up & down speed
  set_tacho_ramp_up_sp(sn, 10);
  set_tacho_ramp_down_sp(sn, 10);
  // set the disp on the motors
  set_tacho_position_sp(sn, deg);
  set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
  //Sleep(600);
	tacho_check_overload(sn);
	throwball(sn, 1);
	return;
}

void throwball(uint8_t sn, float powerfactor) {
  int deg = 100;
  int max_speed;
  int ball_present = 0;
  // change the braking mode

  get_tacho_max_speed(sn, &max_speed);

  //return to initial position
  set_tacho_stop_action_inx(sn, TACHO_BRAKE);
  // set the max speed
  //set_tacho_speed_sp(sn, (max_speed*41)/100);
  set_tacho_speed_sp(sn, 1050);
  // set ramp up & down speed
  // between 870 and 920
  set_tacho_ramp_up_sp(sn, 10);
  set_tacho_ramp_down_sp(sn, 10);
  // printf("%f\n", max_speed*powerfactor*CF_RAMP_UP); // it is 700
  // set the disp on the motors
  set_tacho_position_sp(sn, -130);
  set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
  tacho_wait_term(sn);

  return_to_zero(sn, max_speed);
  send_bt(to_bt);
  return;
}

int liftball(uint8_t sn_lift, uint8_t sn_ball) {
  int deg = 430;
  int max_speed;
	int ball_present = 0;
	int motor_pos;

  // change the braking mode
  get_tacho_max_speed(sn_lift, &max_speed);
  set_tacho_stop_action_inx(sn_lift, TACHO_BRAKE);
  // set the max speed
  set_tacho_speed_sp(sn_lift, 120);
  // set ramp up & down speed
  set_tacho_ramp_up_sp(sn_lift, 15);
  set_tacho_ramp_down_sp(sn_lift, 15);
  // set the disp on the motors
  set_tacho_position_sp(sn_lift, 295);
  set_tacho_command_inx(sn_lift, TACHO_RUN_TO_REL_POS);

	Sleep(200);
	if (tacho_check_overload(sn_lift)){
		// lift if stuck against the ball or a barrier
		printf("lift is stuck\n");
		set_tacho_stop_action_inx(sn_lift, TACHO_STOP);
		get_tacho_position(sn_lift, &motor_pos);
		// turn back the motor
		set_tacho_position_sp(sn_lift, -motor_pos);
		set_tacho_command_inx(sn_lift, TACHO_RUN_TO_REL_POS);
		// go back to initial position
		set_tacho_position_sp(sn_lift, 0);
		set_tacho_command_inx(sn_lift, TACHO_RUN_TO_ABS_POS);
		tacho_wait_term(sn_lift);
		return -1;
	}

  tacho_wait_term(sn_lift);

  // change the braking mode
  get_tacho_max_speed(sn_lift, &max_speed);
  set_tacho_stop_action_inx(sn_lift, TACHO_BRAKE);
  // set the max speed
  set_tacho_speed_sp(sn_lift, 720);
  // set ramp up & down speed
  set_tacho_ramp_up_sp(sn_lift, 10);
  set_tacho_ramp_down_sp(sn_lift, 10);
  // set the disp on the motors
  set_tacho_position_sp(sn_lift, 95);
  set_tacho_command_inx(sn_lift, TACHO_RUN_TO_REL_POS);
  tacho_wait_term(sn_lift);
  //Sleep(2000);
  //return to abs pos
  set_tacho_position_sp(sn_lift, 0);
  set_tacho_speed_sp(sn_lift, max_speed >> 2);
  set_tacho_ramp_up_sp(sn_lift, 100);
  set_tacho_ramp_down_sp(sn_lift, 100);
  set_tacho_command_inx(sn_lift, TACHO_RUN_TO_ABS_POS);
	tacho_wait_term(sn_lift);

	//checking the ball is present
	deg = 100;
  // change the braking mode
  get_tacho_max_speed(sn_ball, &max_speed);
  set_tacho_stop_action_inx(sn_ball, TACHO_BRAKE);
  // set the max speed
  set_tacho_speed_sp(sn_ball, max_speed);
  // set ramp up & down speed
  set_tacho_ramp_up_sp(sn_ball, 10);
  set_tacho_ramp_down_sp(sn_ball, 10);
  // set the disp on the motors
  set_tacho_position_sp(sn_ball, deg);
  set_tacho_command_inx(sn_ball, TACHO_RUN_TO_REL_POS);
  //Sleep(600);
  ball_present = tacho_check_overload(sn_ball);
  if (!ball_present){
		//no ball
    return_to_zero(sn_ball, max_speed);
    return 0;
  } else {
		//ball taken
		set_tacho_stop_action_inx(sn_ball, TACHO_STOP);
		return 1;
	}
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
  set_sensor_mode_inx(sn_us, US_US_DC_CM);
	printf("Sensors_init: gyro ID %d, us ID %d\n",sn_gyro, sn_us);
	set_sensor_mode_inx(sn_gyro, GYRO_GYRO_CAL);
  Sleep(2000);
	set_sensor_mode_inx(sn_gyro, GYRO_GYRO_ANG);
  printf( "Sensors init completed!\n" );

  pos.x=START_POS_X;
  pos.y=START_POS_Y;
  pos.start_deg=read_gyro(sn_gyro, 5);              //Not guaranteed that gyro_dir has the right value
  pos.deg=0;
}


void reset_gyro(uint8_t sn_gyro){
  Sleep(1000);
  set_sensor_mode_inx(sn_gyro, GYRO_GYRO_RATE);
  Sleep(1000);
  set_sensor_mode_inx(sn_gyro, GYRO_GYRO_ANG);
  Sleep(2000);
  return;
}


void update_position(int movement, int degree){
  pos.deg=(pos.deg+degree+360)%360;
  pos.x+=movement*sin(PI*(float)pos.deg/180.0);
  pos.y+=movement*cos(PI*(float)pos.deg/180.0);
  printf("Position x:%.2f y:%.2f deg:%d deg_abs:%d\n", pos.x, pos.y, pos.deg, pos.start_deg);
}

int continous_search(struct Search_Areas area){
	struct DistanceReading data[350];
	int value;
	int s_distance = area.radius;
	int deg, deg_err;
	float degree;
	int initial_rot, end_rot;
	int distance, e_distance;
	int i, min, max, init_value, end_value, in_range, found;
	float a,b;
	FLAGS_T state0, state1;

	//go to start position
	rotate_with_adjustment(110, sn_tacho);

	//start scanning
	deg=-220;
	degree = AXE_WHEELS*(1.0*deg) / DIAM;
	set_for_rotate(degree, sn_tacho);
	multi_set_tacho_speed_sp(sn_tacho, 70);
	Sleep(200);
  initial_rot = read_gyro(sn_gyro, 5);
	multi_set_tacho_command_inx(sn_tacho, TACHO_RUN_TO_REL_POS);
  //Sleep(10);
	value=0;
	do {
		//reading values
		data[value].distance=read_us(sn_us, 3);
		data[value].degree=read_gyro(sn_gyro, 1);
		value++;
		if (value >= 350) break;
		get_tacho_state_flags(sn_tacho[0], &state0);
		get_tacho_state_flags(sn_tacho[1], &state1);
	} while ((state0 || state1) & 0x1);

	tacho_wait_term(sn_tacho[0]);
	tacho_wait_term(sn_tacho[1]);
	Sleep(200);
	end_rot = read_gyro(sn_gyro, 5);

	printf("Number of readings: %d\n", value);
	deg_err=360-end_rot+initial_rot-220;
	if(deg_err>180) deg_err=deg_err-360;
	printf("degrees of error: %d\n", deg_err);
	update_position(0, -220-deg_err);

	//search for the min value on the array
	//init_value contains the first index with the min value
	//end_value contains the first index after the min value
	if (area.stype==RADIUS) {

		min=5000;
		init_value=-1;
		end_value=-1;
		in_range=0;
		for (i=0; i<value; i++) {
			printf("Value: %d; dist:%d\tdegr:%d\n", i, data[i].distance, data[i].degree);
			if (data[i].distance<min) {
				min=data[i].distance;
				init_value=i;
				in_range=1;
			} else if (in_range==1 && data[i].distance!=min) {
					end_value=i;
					in_range=0;
			}
		}
		if (in_range) end_value=i;

		//if distance <= 300 rotate towards the ball
		if (min<=s_distance) {
			deg=(data[(end_value+init_value)/2-1].degree-end_rot+360)%360;
			printf("degree to ball: %d, choosen: %d\n", deg, (end_value+init_value)/2);
			rotate_with_adjustment(deg_err+deg, sn_tacho);
			return min;
		} else {
			//rotate_with_adjustment(deg_err+110, sn_tacho);
		}

	} else if (area.stype==ELLIPTIC) {

		a=(float) area.radius;
		b=(float) area.w_dist;
		max=0;
		distance=0;
		init_value=-1;
		end_value=-1;
		in_range=0;
		found=0;
		for (i=0; i<value; i++) {
			e_distance=(int) elliptic_distance((initial_rot-data[i].degree+360)%360-20, a, b);
			printf("Value: %d; dist:%d\tdegr:%d Elliptic distace=%d\n", i, data[i].distance, data[i].degree, e_distance);
			if (e_distance-data[i].distance > max) {
				distance=data[i].distance;
				max=e_distance-data[i].distance;
				init_value=i;
				in_range=1;
				found=1;
			} else if (in_range==1 && (data[i].distance>max+1 || data[i].distance<max-1)) {
					end_value=i;
					in_range=0;
			}
		}
		if (found==1) {
			deg=(data[(end_value+init_value)/2-1].degree-end_rot+360)%360;
			printf("degree to ball: %d, choosen: %d\n", deg, (end_value+init_value)/2);
			rotate_with_adjustment(deg_err+deg, sn_tacho);
			return distance;
		} else {
			//rotate_with_adjustment(deg_err+110, sn_tacho);
		}
	}

	return -1;
}


int closerange_search(){
	struct DistanceReading data[200];
	int value;
	int s_distance = 200;
	int deg, deg_err;
	float degree;
	int initial_rot, end_rot;
	int i, min, init_value, end_value, in_range, found;
	float a,b;
	FLAGS_T state0, state1;

	//go to start position
	rotate_with_adjustment(45, sn_tacho);

	//start scanning
	deg=-90;
	degree = AXE_WHEELS*(1.0*deg) / DIAM;
	set_for_rotate(degree, sn_tacho);
	multi_set_tacho_speed_sp(sn_tacho, 70);
	Sleep(200);
  initial_rot = read_gyro(sn_gyro, 5);
	multi_set_tacho_command_inx(sn_tacho, TACHO_RUN_TO_REL_POS);
  //Sleep(10);
	value=0;
	do {
		//reading values
		data[value].distance=read_us(sn_us, 3);
		data[value].degree=read_gyro(sn_gyro, 1);
		value++;
		if (value >= 350) break;
		get_tacho_state_flags(sn_tacho[0], &state0);
		get_tacho_state_flags(sn_tacho[1], &state1);
	} while ((state0 || state1) & 0x1);

	tacho_wait_term(sn_tacho[0]);
	tacho_wait_term(sn_tacho[1]);
	Sleep(200);
	end_rot = read_gyro(sn_gyro, 5);

	printf("Number of readings: %d\n", value);
	deg_err=360-end_rot+initial_rot-90;
	if(deg_err>180) deg_err=deg_err-360;
	printf("degrees of error: %d\n", deg_err);
	update_position(0, -90-deg_err);

	//search for the min value on the array
	//init_value contains the first index with the min value
	//end_value contains the first index after the min value


		min=5000;
		init_value=-1;
		end_value=-1;
		in_range=0;
		for (i=0; i<value; i++) {
			printf("Value: %d; dist:%d\tdegr:%d\n", i, data[i].distance, data[i].degree);
			if (data[i].distance<min) {
				min=data[i].distance;
				init_value=i;
				in_range=1;
			} else if (in_range==1 && data[i].distance!=min) {
					end_value=i;
					in_range=0;
			}
		}
		if (in_range) end_value=i;

		//if distance <= 300 rotate towards the ball
		if (min<=s_distance) {
			deg=(data[(end_value+init_value)/2].degree-end_rot+360)%360;
			printf("degree to ball: %d, choosen: %d\n", deg, (end_value+init_value)/2);
			rotate_with_adjustment(deg_err+deg, sn_tacho);
			return min;
		} else {
			//rotate_with_adjustment(deg_err+110, sn_tacho);

		}

		return -1;

}


void return_to_center(uint8_t *sn){
  float radius = sqrt(pos.x*pos.x+pos.y*pos.y);
  float gamma = atan(pos.x/pos.y)*180.0/PI;

	if (pos.x == 0 && pos.y == 0 && pos.deg == 0) {
		/* check if we're in the initial position */
		return;
	}

  if (pos.y < 0) {
    gamma += 180;
  } else {
    if (pos.x < 0){
      gamma += 360;
    }
  }
  float beta = (float)pos.deg;
  float alpha = (float) (((int)(beta - gamma + 270) % 360));
  float target_angle;
  if (alpha > 180) {
    target_angle = (float)270 - alpha;
    radius = - radius;
  } else {
    target_angle = 90 - alpha;
  }

  rotate_with_adjustment((int)target_angle, sn_tacho);
  go_straight_mm((int)radius, sn_tacho, 1);
  if(pos.deg>180){
    rotate_with_adjustment(360-pos.deg, sn_tacho);
  } else {
    rotate_with_adjustment(-pos.deg, sn_tacho);
  }
	update_position(0, -pos.deg);
	pos.x=0;
	pos.y=0;
  return;
}

int sign(int x){
  return (x >= 0) - (x < 0);
}

int negative(int x){
  return (x < 0);
}

int min_angle(int delta, int deg){
  if(negative(delta)){
    return (180 - deg);
  }else if(deg > 180){
    return (360 - deg);
  }else {
    return (-deg);
  }
}




void go_to_point90(int pointx, int pointy, uint8_t *sn, enum Dir direction){
  int dx, dy, deg;
  dx=pointx-pos.x;
  dy=pointy-pos.y;//
  if(dy != 0){
    deg=min_angle(dy,pos.deg);
    printf("\n ____ANGLE: %d_____\n",deg);
    rotate_with_adjustment(deg, sn);
    Sleep(200);
    go_straight_mm(abs(dy), sn, 1);
    Sleep(200);
  }
  if(dx != 0 ){
    deg=pos.deg-90;
    deg = min_angle(dx,deg+360*negative(deg));
    printf("\n ____ANGLE: %d_____\n",deg);
    rotate_with_adjustment(deg, sn);
    Sleep(200);
    go_straight_mm(abs(dx),sn, 1);
    Sleep(200);
  }

  switch(direction){
  	case N:
  		deg = min_angle(1, pos.deg);
  		break;

  	case E:
  		deg=pos.deg-90;
    	deg = -min_angle(sign((pos.deg % 180)- 90),deg+360*negative(deg));
  		break;

  	case W:
  		deg=pos.deg-90;
    	deg = -min_angle(-sign((pos.deg % 180)- 90),deg+360*negative(deg));
    	break;

  	case S:
  		deg = min_angle(-1, pos.deg);
  		break;

  }
  rotate_with_adjustment(deg, sn);

  return;
}



void go_to_point(int pointx, int pointy, uint8_t *sn){
  float dx, dy;
  dx=pointx-pos.x;
  dy=pointy-pos.y;
  float radius = sqrt((dx)*(dx)+(dy)*(dy));
  float gamma = atan((dx)/(dy))*180.0/PI;
  printf("gamma: %.2f\n", gamma);
  if ((dy) < 0) {
    gamma += 180;
  } else {
    if (dx < 0){
      gamma += 360;
    }
  }
  int beta = pos.deg;
  int alpha = (((int)(gamma - beta + 360) % 360));
  int target_angle;
  if (alpha > 180) {
    target_angle = alpha-360;//(float)270 - alpha;
  } else {
    //radius = -radius;
    target_angle=alpha;
    //target_angle = 90 - alpha;
  }
  printf("target_angle: %d\n", target_angle);
  rotate_with_adjustment(target_angle, sn_tacho);
  printf("pos.deg: %d\n", pos.deg);
  go_straight_mm(radius, sn_tacho, 0);
  return;
}

void set_for_rotate(int deg, uint8_t *sn){
  int speed=turn_speed(deg);
	//int speed=120;
  //printf("%d\n", speed);

  multi_set_tacho_stop_action_inx(sn, TACHO_BRAKE);
	// set ramp up & down speed at zero
	multi_set_tacho_ramp_up_sp(sn, 10);
	multi_set_tacho_ramp_down_sp(sn, 10);
	set_tacho_speed_sp(sn[0], speed);
  set_tacho_speed_sp(sn[1], speed);
	// set the disp on the motors
	set_tacho_position_sp(sn[0], (int)(-deg));
	set_tacho_position_sp(sn[1], (int)(deg));
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
  y=read_us(sn_us, 5); //value in axe y
  //need to turn of 90 degrees to sample x value;
  rotate(90, sn);
  Sleep(200);
  x=read_us(sn_us, 5);//value in axe x
  //Need to verify correctness of the value w.r.t current position
  //if(FIELD_WIDTH-pos.x>8 ||
}


float elliptic_distance(int deg, float a, float b){
  float distance;
  //a is the distance that the robot is facing when the robot starts the scan
  distance=(a*b)/(sqrt(a*a*sin(PI/180*deg)*sin(PI/180*deg)+b*b*cos(PI/180*deg)*cos(PI/180*deg)));
  return distance;
}

void initialize_areas(struct Search_Areas *areas){
  //function that defines the areas of search by the standard movements
  // first area
	areas[0].posx=0;
  areas[0].posy=0;
	areas[0].radius=400;
  areas[0].w_dist=600-ROBOT_LENGTH/2;
	areas[0].stype=RADIUS;
	areas[0].dir=N;
  //second area
  areas[1].posx=250;
  areas[1].posy=0;
  areas[1].radius=400;
	areas[1].w_dist=300;
	areas[1].stype=ELLIPTIC;
	areas[1].dir=E;
  //third
  areas[2].posx=250;
  areas[2].posy=350;
  areas[2].radius=320;
	areas[2].w_dist=320;
	areas[2].stype=RADIUS;
	areas[2].dir=E;
  //fourth
  areas[3].posx=0;
  areas[3].posy=350;
  areas[3].radius=400;
	areas[3].w_dist=320;
	areas[3].stype=ELLIPTIC;
	areas[3].dir=N;
	//fifth
	areas[4].posx=-250;
  areas[4].posy=350;
  areas[4].radius=320;
	areas[4].w_dist=320;
	areas[4].stype=RADIUS;
	areas[4].dir=W;
	//sixth
	areas[5].posx=-250;
  areas[5].posy=0;
  areas[5].radius=400;
	areas[5].w_dist=300;
	areas[5].stype=ELLIPTIC;
	areas[5].dir=W;
	//seventh
	areas[6].posx=0;
  areas[6].posy=-10;
  areas[6].radius=400;
	areas[6].w_dist=300;
	areas[6].stype=ELLIPTIC;
	areas[6].dir=S;
  return;
}

void sample_w_dist(struct Search_Areas areas[]){
	Sleep(1000);
	printf("Position 0: %d\n", read_us(sn_us, 20));
	fflush(stdout);

	// Second position
	rotate_with_adjustment(90, sn_tacho);
	go_straight_mm(250, sn_tacho, 1);
	Sleep(1000);
	printf("Position 1: %d\n", read_us(sn_us, 20));
	fflush(stdout);

	// Third position
	rotate_with_adjustment(-90, sn_tacho);
	go_straight_mm(400, sn_tacho, 1);
	Sleep(1000);
	printf("Position 2 - front: %d\n", read_us(sn_us, 20));
	fflush(stdout);
	rotate_with_adjustment(90, sn_tacho);
	Sleep(1000);
	printf("Position 2 - lateral: %d\n", read_us(sn_us, 20));
	fflush(stdout);

	// Fourth position
	go_straight_mm(-250, sn_tacho, 1);
	rotate_with_adjustment(-90, sn_tacho);
	Sleep(1000);
	printf("Position 3: %d\n", read_us(sn_us, 20));
	fflush(stdout);

	// Fifth position
	rotate_with_adjustment(-90, sn_tacho);
	go_straight_mm(250, sn_tacho, 1);
	Sleep(1000);
	printf("Position 4 - lateral: %d\n", read_us(sn_us, 20));
	fflush(stdout);

	rotate_with_adjustment(90, sn_tacho);
	Sleep(1000);
	printf("Position 4 - front: %d\n", read_us(sn_us, 20));
	fflush(stdout);

	// Sixth position
	go_straight_mm(-400, sn_tacho, 1);
	rotate_with_adjustment(-90, sn_tacho);
	Sleep(1000);
	printf("Position 5: %d\n", read_us(sn_us, 20));
	fflush(stdout);

	return;
}

/* Signal handler */
void kill_all(int sig_numb){
	if (sig_numb == SIGINT) {
		printf("Handling signal, killing all\n");
		flag_kill=1;
		multi_kill_motor(sn_tacho);
		kill_motor(sn_lift);
		kill_motor(sn_ball);

    close_bt();

		/* Uninit sensor */
		ev3_uninit();
    kill(getpid(), SIGTERM);
    //EXIT_FAILURE;
	}
}

int calibrate(){

	int rotation, rot;
	int dist_lat, dist_front;
	int posx = pos.x;
	int posy = pos.y;
	float deg;

	if (posx >= 0){
		rotation = (90-pos.deg+360)%360;
		rot=-90;
		dist_lat = 630-posx;	//distance between axe wheels and lateral wall
	}
	else {
		rotation = (-90-pos.deg+360)%360;
		rot=90;
		dist_lat = 630+posx;	//distance between axe wheels and lateral wall
	}
	dist_front = 700-posy;	//distance between axe wheel and front wall

	// rotate in order to face closer lateral wall
	rotate_with_adjustment(rotation, sn_tacho);
	// crash into it
	//go_straight_mm(dist_lat+100, sn_tacho, 0);
	deg = 36 * (dist_lat+100) / (PI * DIAM);
	multi_set_tacho_speed_sp(sn_tacho, MAX_SPEED/3);
	multi_set_tacho_ramp_up_sp(sn_tacho, 15);
  multi_set_tacho_ramp_down_sp(sn_tacho, 15);
	multi_set_tacho_position_sp(sn_tacho, deg);
	multi_set_tacho_command_inx(sn_tacho, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn_tacho[0]);
	tacho_wait_term(sn_tacho[1]);



	multi_set_tacho_speed_sp(sn_tacho, MAX_SPEED/5);
	// set ramp up & down speed
	multi_set_tacho_ramp_up_sp(sn_tacho, 15);
  multi_set_tacho_ramp_down_sp(sn_tacho, 15);
	// set the disp on the motors
	multi_set_tacho_time_sp(sn_tacho, 500);
	// initialize the tacho
  set_tacho_command_inx(sn_tacho[0], TACHO_RUN_TIMED);
	tacho_wait_term(sn_tacho[0]);
	set_tacho_command_inx(sn_tacho[1], TACHO_RUN_TIMED);
	tacho_wait_term(sn_tacho[1]);
	multi_set_tacho_command_inx(sn_tacho, TACHO_RUN_TIMED);
	tacho_wait_term(sn_tacho[0]);
	tacho_wait_term(sn_tacho[1]);

	// come back of the wanted distance
	go_straight_mm(-dist_lat+ROBOT_LENGTH/2, sn_tacho, 0);

	// rotate to face front wall
	rotate_with_adjustment(rot, sn_tacho);
	// go crash into it
	go_straight_mm(dist_front+50, sn_tacho, 0);

	// come back of the wanted distance
	go_straight_mm(-dist_front+ROBOT_LENGTH/2-20, sn_tacho, 0);

	// update the position to the initial desired position
	pos.x = posx;
	pos.y = posy;

	return 0;
}

int lateral_calibrate(){

	int rotation, rot;
	int dist_lat, dist_front;
	int posx = pos.x;
	int posy = pos.y;
	float deg;

	if (posx >= 0){
		rotation = (90-pos.deg+360)%360;
		rot=-90;
		dist_lat = 630-posx;	//distance between axe wheels and lateral wall
	}
	else {
		rotation = (-90-pos.deg+360)%360;
		rot=90;
		dist_lat = 630+posx;	//distance between axe wheels and lateral wall
	}
	dist_front = 700-posy;	//distance between axe wheel and front wall

	// rotate in order to face closer lateral wall
	rotate_with_adjustment(rotation, sn_tacho);
	// crash into it
	//go_straight_mm(dist_lat+100, sn_tacho, 0);
	deg = 36 * (dist_lat+100) / (PI * DIAM);
	multi_set_tacho_speed_sp(sn_tacho, MAX_SPEED/3);
	multi_set_tacho_ramp_up_sp(sn_tacho, 15);
  multi_set_tacho_ramp_down_sp(sn_tacho, 15);
	multi_set_tacho_position_sp(sn_tacho, deg);
	multi_set_tacho_command_inx(sn_tacho, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn_tacho[0]);
	tacho_wait_term(sn_tacho[1]);



	multi_set_tacho_speed_sp(sn_tacho, MAX_SPEED/5);
	// set ramp up & down speed
	multi_set_tacho_ramp_up_sp(sn_tacho, 15);
  multi_set_tacho_ramp_down_sp(sn_tacho, 15);
	// set the disp on the motors
	multi_set_tacho_time_sp(sn_tacho, 500);
	// initialize the tacho
  set_tacho_command_inx(sn_tacho[0], TACHO_RUN_TIMED);
	tacho_wait_term(sn_tacho[0]);
	set_tacho_command_inx(sn_tacho[1], TACHO_RUN_TIMED);
	tacho_wait_term(sn_tacho[1]);
	multi_set_tacho_command_inx(sn_tacho, TACHO_RUN_TIMED);
	tacho_wait_term(sn_tacho[0]);
	tacho_wait_term(sn_tacho[1]);

	// come back of the wanted distance
	go_straight_mm(-dist_lat+ROBOT_LENGTH/2, sn_tacho, 0);

	// rotate to face front wall
	rotate_with_adjustment(rot, sn_tacho);

	// update the position to the initial desired position
	pos.x = posx;
	pos.y = posy;

	return 0;
}

void looser(uint8_t sn){
	int deg=30;

	set_tacho_stop_action_inx(sn, TACHO_BRAKE);
  set_tacho_speed_sp(sn, 100);
  set_tacho_ramp_up_sp(sn, 10);
  set_tacho_ramp_down_sp(sn, 10);
	set_tacho_position_sp(sn, -deg/2);
	set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	tacho_wait_term(sn);

	while(1){
		set_tacho_position_sp(sn, deg);
	  set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	  tacho_wait_term(sn);
		set_tacho_position_sp(sn, -deg);
	  set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
	  tacho_wait_term(sn);
	}
}
