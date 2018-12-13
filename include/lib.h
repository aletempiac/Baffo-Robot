#ifndef LIB_H
#define LIB_H
#include "config.h"


/****************************************************************************************************/
/*                                   GLOBAL VARIABLES                                               */
/****************************************************************************************************/
struct Position {
  float x;  //in mm
  float y;  //in mm
  int deg;
  int start_deg;
};

struct CornerAngles {
  int bl; //bottom left
  int tl; //top right
  int tr;
  int br;
};

extern struct Position pos;
extern uint8_t sn_ball;		  //tacho to throw the ball
extern uint8_t sn_lift;      //tacho to lift the ball
extern uint8_t sn_tacho[3];  //2 tacho motors, 3rd one for closing the multi
extern uint8_t sn_gyro;      //gyroscope
extern uint8_t sn_us;        //ultrasonic distance sensor
extern uint8_t sn_touch;
extern uint8_t sn_color;
extern uint8_t sn_sonar;
extern uint8_t sn_mag;

extern FLAGS_T state;
extern char s[ 256 ];
extern int val;
extern float value;

extern pthread_mutex_t sem_gyro;
extern pthread_mutex_t sem_us;
extern pthread_t thread[2];

extern volatile int gyro_dir;
extern volatile int us_dist;
extern volatile int flag_kill;

/****************************************************************************************************/
/*                                  FUNCTION PROTOTYPES                                             */
/****************************************************************************************************/
/* Sensor initilization */
void sensors_init(void);
/* Function for rotation */
int rotate(int deg, uint8_t * sn);  //returns the degrees rotated
void rotate_with_adjustment(int deg, uint8_t * sn);
void set_for_rotate(int deg, uint8_t *sn);
void rotate_action(int deg, uint8_t * sn);

/* Function for movement */
void go_straight_mm(int mm, uint8_t * sn);
void return_to_center(uint8_t *sn);
int turn_speed(int deg);

/* Blocking function until motors are done */
void tacho_wait_term(uint8_t motor);
void tacho_wait_ball(uint8_t motor);

/* Motor killer */
void kill_motor(uint8_t motor);
void multi_kill_motor(uint8_t *motors);

/* Throw handler */
void throwball(uint8_t sn, float powerfactor);
void liftball(uint8_t sn);

/* Position handling */
void update_corner_angles(struct CornerAngles *c_angles, struct Position pos);
void update_position(int movement, int degree_abs);

/* Function to search the ball */
int simple_search();
float elliptic_search(uint8_t *sn, struct Position pos);
float elliptic_distance(int deg, float a, float b);

/* Sensors manager */
float read_gyro(uint8_t sn_gyro);
int get_gyro_value();
int read_us(uint8_t sn_us);
float get_us_value();

/* Thread routine */
void* gyro_thread(void* arg);
void* us_thread(void* arg);

/* Signal handler for CTRL_C*/
void kill_all(int sig_numb);

/* Function for min and max value */
float min(float x, float y);
float max(float x, float y);

/* Algorithm to describe the whole flow */
void alg_flow(uint8_t *sn_tacho, uint8_t sn_ball, uint8_t sn_lift, struct Position pos);

#endif /* LIB_H */
