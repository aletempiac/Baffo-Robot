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

struct DistanceReading {
  int distance;
  int degree;
};

struct Search_Areas{
  int posx;     //offset from origin of axes
  int posy;     //offset from origin of axes
  int radius;  // radious for the search
  int w_dist;   // distance expected from the wall
  enum Search_Type stype;
  enum Dir dir;
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
void reset_gyro(uint8_t sn_gyro);

/* Function for rotation */
int rotate(int deg, uint8_t * sn);  //returns the degrees rotated
void rotate_with_adjustment(int deg, uint8_t * sn);
void set_for_rotate(int deg, uint8_t *sn);
void rotate_action(int deg, uint8_t * sn);
void rotate_with_slowdown(int deg, uint8_t * sn);

/* Function for movement */
int go_straight_mm(int mm, uint8_t * sn, int check_area); //return if a success or not
int go_straight_fullsped(int mm, uint8_t *sn);
void return_to_center(uint8_t *sn);
void go_to_point(int pointx, int pointy, uint8_t *sn);
void go_to_point90(int pointx, int pointy, uint8_t *sn, enum Dir direction);
int turn_speed(int deg);
int calibrate();
int lateral_calibrate();

/* Blocking function until motors are done */
void tacho_wait_term(uint8_t motor);
void tacho_wait_ball(uint8_t motor);

/* Motor killer */
void kill_motor(uint8_t motor);
void multi_kill_motor(uint8_t *motors);

/* Throw handler */
void start_throwball(uint8_t sn); //function to be called only for the first throw
void throwball(uint8_t sn, float powerfactor);
int liftball(uint8_t sn_lift, uint8_t sn_ball); // returns 1 in case of success, else otherwise
void looser(uint8_t sn);

/* Position handling */
void update_corner_angles(struct CornerAngles *c_angles, struct Position pos);
void update_position(int movement, int degree_abs);
int check_in_area(int movement, struct Position pos);

/* Function to search the ball */
int continous_search(struct Search_Areas area);
int closerange_search();
int simple_search(enum Search_Type OP, int degree_start, int degree_stop, int radious);  //returns 0 when no ball is found, a pos value (distance) when a ball is found, a neg value (degree) when something is found but not detected
float elliptic_search(uint8_t *sn, struct Position pos);
float elliptic_distance(int deg, float a, float b);

/* Sensors manager */
int read_gyro (uint8_t sn_gyro, int n);
int read_us(uint8_t sn_us, int n);

/* Signal handler for CTRL_C*/
void kill_all(int sig_numb);

/* Function for min and max value */
int min(int x, int y);
int max(int x, int y);
int sign(int x);
int negative(int x);
int min_angle(int delta, int deg);

/* Algorithm to describe the whole flow */
void alg_flow(uint8_t *sn_tacho, uint8_t sn_ball, uint8_t sn_lift, struct Position pos, struct Search_Areas *areas, enum Mode mode);

/* Initialize areas of search */
void initialize_areas(struct Search_Areas *areas);
void sample_w_dist(struct Search_Areas areas[]);
#endif /* LIB_H */
