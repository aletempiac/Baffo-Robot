#ifndef BT_H
#define BT_H

/* Global variables */
extern int s;
extern uint16_t msgId;

/* Function prototypes */
int main_bt();
int read_from_server (int sock, char *buffer, size_t maxSize);
void robot ();




#endif /* BT_H */
