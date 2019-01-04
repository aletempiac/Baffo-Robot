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

const char const *color[] = {"?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };

/****************************************************************************************************/
/*                                   GLOBAL VARIABLES                                               */
/****************************************************************************************************/

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
/****************************************************************************************************/

int elaborate_dist(uint8_t *sn_tacho, uint8_t sn_ball, uint8_t sn_lift, struct Position pos, int dist){
  int dist_tmp;
  int balls;

  if(dist>0){
    //ball found
    //if distance is > 300 cm go even closer and search again
    if(dist<300){
      //TBI check factor dist*... is different from zero
      go_straight_mm(dist-100, sn_tacho);
      //There is a ball?
      //check with color sensor or distance sensor
      dist_tmp=get_us_value();
      printf("Distance after moving towards the ball: %d\n", dist_tmp);
      if(get_us_value()<=150){
        //ball near enought
        //liftball(sn_lift);
        //return_to_center(sn_tacho);
        //throwball(sn_ball, 0.8);
        //balls++;
        //send scored
        return BALL_SHOT;
      } else {
        return_to_center(sn_tacho);
        //probably wrong or simple search from that position
        //rotate(-(pos.deg-180));
        //simple_search();
        return NOT_FOUND;
      }
    } else {
      //go near that area and search again
      go_straight_mm(dist-200, sn_tacho);
      return OBJ_IN_AREA;
      //return_to_center(distance, sn_tacho);
    }
  } else if(dist<0){
    //in this case something is found but not really detected
    dist = 130;
    do{
        dist -= 20;

    } while(!go_straight_mm(dist,sn_tacho));
    return AREA_326;
  } else {
    //set the area as free

  }

  return AREA_FREE;

}
//this is the function that search a ball and score
void alg_flow(uint8_t *sn_tacho, uint8_t sn_ball, uint8_t sn_lift, struct Position pos){
  int dist, dist_tmp;
  int balls;
  //at start robot has too balls
  //go forward to 3 points line and score
  //go_straight_mm(25, sn_tacho);
  //throwball(sn_ball, 0.8);
  //now lift ready ball
  //liftball(sn_lift);
  Sleep(2000);
  //throwball(sn_ball, 0.8);
  //scan front area to verify to have scored TO BE IMPLEMENTED
  balls=2;
  //hopefully send 6 points scored message
  //TO BE IMPLEMENTED
  //Scanning phase
  //scan fron area
  dist=simple_search(DEFAULT, 0, 0, 0);

  switch( elaborate_dist(sn_tacho,sn_ball,sn_lift,pos,dist) ){

    case BALL_SHOT:
      //simple_search(CENTERING, 0, 0, 0);
      liftball(sn_lift);
      return_to_center(sn_tacho);
      throwball(sn_ball, 1);
      balls++;
      // to new area
      printf("To new area\n");
      break;

    case NOT_FOUND:
      // free but maybe check again
      printf("To new area, but maybe there is something\n");
      break;

    case OBJ_IN_AREA:
      printf("Something found but distant\n");
      dist = simple_search(CENTERING, 30, -30, 200);
      elaborate_dist(sn_tacho,sn_ball,sn_lift,pos,dist);
      liftball(sn_lift);
      return_to_center(sn_tacho);
      go_straight_mm(200, sn_tacho);
      throwball(sn_ball, 1);
      balls++;
      break;

    case AREA_326:
      printf("Search better, maybe something\n");
      dist = simple_search(SECTOR, 40, -40, 300);
      elaborate_dist(sn_tacho,sn_ball,sn_lift,pos,dist);
      break;

    case AREA_FREE:
      printf("Nothing here\n");
      break;

    default:
      fprintf(stderr,"*************************ERROR IN SEARCH*************************\n");
      break;
  }
  /*if(dist>0){
    //ball found
    //if distance is > 300 cm go even closer and search again
    if(dist<300){
      //TBI check factor dist*... is different from zero
      go_straight_mm(dist-70, sn_tacho);
      //There is a ball?
      //check with color sensor or distance sensor
      dist_tmp=get_us_value();
      printf("Distance after moving towards the ball: %d\n", dist_tmp);
      if(get_us_value()<=120){
        //ball near enought
        liftball(sn_lift);
        return_to_center(sn_tacho);
        throwball(sn_ball, 0.8);
        balls++;
        //send scored
      } else {
        return_to_center(sn_tacho);
        //probably wrong or simple search from that position
        //rotate(-(pos.deg-180));
        //simple_search();
      }
    } else {
      //go near that area and search again
      go_straight_mm(250, sn_tacho);
      dist=simple_search();
      if(dist>0){
        //ball found

      }
      //return_to_center(distance, sn_tacho);
    }
  } else if(dist<0){
    //in this case something is found but not really detected

  } else {
    //set the area as free
  }*/
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
  //throwball(sn_ball, 1);
  //alg_flow(sn_tacho,sn_ball,sn_lift,pos);

  //Initial setup
  // liftball(sn_lift);
  // Sleep(1500);
  // throwball(sn_ball, 1);
  //go_straight_mm(450, sn_tacho);
  //rotate_with_adjustment(-170, sn_tacho);
  //go_straight_mm(-450, sn_tacho);
  //throwball(sn_ball, 1);

  rotate_with_adjustment(-90, sn_tacho);
  //int x = go_straight_mm(350, sn_tacho);
  //printf("Return of go %d\n", x);
  //rotate_with_adjustment(-90, sn_tacho);
  //int j= go_straight_mm(190, sn_tacho);
  //printf("Return of go %d\n", i);
  //rotate_with_adjustment(-135, sn_tacho);
  //go_straight_mm(800, sn_tacho);
  //return_to_center(sn_tacho);
/*
  go_straight_mm(10, sn_tacho);
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
  //go_straight_mm(-40, sn_tacho);
  //look_at_corners(sn_tacho, c_angles);
  //rotate(180, sn_tacho);
  //throwball(sn_ball, 0.8);
  //alg_flow(sn_tacho, sn_ball, sn_lift, pos);
  //rotate(-90, sn_tacho);
  //rotate(180, sn_tacho);
  //dist=simple_search();
/*

  if(dist>0){
    go_straight_mm(dist*1.0/10-8, sn_tacho);
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
