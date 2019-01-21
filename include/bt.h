#ifndef BT_H
#define BT_H

/* Global variables */
//extern int s;
extern uint16_t msgId;
extern int s_bt;
extern pthread_t thread[2];
extern volatile int flag_kill;
extern char to_bt[6];

/* Function prototypes */
int main_bt();
int read_from_server (int sock, char *buffer, size_t maxSize);
int send_to_server (int sock, char *buffer, size_t maxSize);
void send_bt(char * message);
void robot ();

/* Init and close bluetooth */
int initialize_bt();
void close_bt();
/* Thread routine */
void * bt_receiver();
void * bt_sender();




#endif /* BT_H */
