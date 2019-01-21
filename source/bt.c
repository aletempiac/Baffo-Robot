#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <math.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include "bt.h"
#include "config.h"
#include <pthread.h>
#include <signal.h>

int read_from_server (int sock, char *buffer, size_t maxSize) {
  int bytes_read = read (sock, buffer, maxSize);

  if (bytes_read <= 0) {
    fprintf (stderr, "Server unexpectedly closed connection...\n");
    close (s_bt);
    exit (EXIT_FAILURE);
  }
  return bytes_read;
}

void send_bt(char * message){
  *((uint16_t *) message) = msgId++;
  send_to_server(s_bt,message,6);
  return;
}

int send_to_server (int sock, char *buffer, size_t maxSize) {
  int bytes_send = write (sock, buffer, maxSize);
  printf("Inside send_to_server %d\n", bytes_send);

  if (bytes_send <= 0) {
    fprintf (stderr, "Server unexpectedly closed connection...\n");
    close (s_bt);
    exit (EXIT_FAILURE);
  }
  return bytes_send;
}

int initialize_bt() {
  struct sockaddr_rc addr = { 0 };
  int status;
  /* allocate a socket */
  s_bt = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

  /* set the connection parameters (who to connect to) */
  addr.rc_family = AF_BLUETOOTH;
  addr.rc_channel = (uint8_t) 1;
  str2ba (SERV_ADDR, &addr.rc_bdaddr);

  /* connect to server */
  status = connect(s_bt, (struct sockaddr *)&addr, sizeof(addr));

  /* if connected */
  if( status == 0 ) {
    char string[58];
    /* Wait for START message */
    read_from_server (s_bt, string, 9);
    if (string[4] == MSG_START) {
      printf("Received start message!\n");
      to_bt[2] = TEAM_ID;
      to_bt[3] = 0xFF;
      to_bt[4] = MSG_SCORE;
      to_bt[5] = 3;
      
      return 1;
    }
    else {
      printf("Illegal start message\n");
      return 0;
    }
  }
  else {
    fprintf (stderr, "Failed to connect to server...\n");
    return -1;
  }
}

void close_bt(){
  close(s_bt);
  /* Close thread related to Bt */
  pthread_cancel(thread[0]);
  pthread_cancel(thread[1]);
}

void * bt_receiver(){
  char buf[40];
  int read_bytes;
  while (flag_kill == 0) {
    //pthread_mutex_lock(&sem_bt);
    read_bytes=read_from_server(s_bt, buf, 40);
    //pthread_mutex_unlock(&sem_bt);
    if (read_bytes <= 0) {
      initialize_bt();
    }
    else {
        if (buf[4] == MSG_STOP || buf[4] == MSG_KICK) {
          flag_kill = 1;
          kill(getpid(), SIGINT);
        }
      }
    }
  pthread_exit(0);
}


void robot () {
  char string[58];
  char type;
  printf ("I'm navigating...\n");

  srand(time(NULL));
  /* Send 3 SCORE messages */
  int i;
  for (i=0; i<30; i++){
    printf("In loop #%d\n", i);
    *((uint16_t *) string) = msgId++;
    string[2] = TEAM_ID;
    string[3] = 0xFF;
    string[4] = MSG_SCORE;
    string[5] = 1;          /* x */
    write(s_bt, string, 6);
    Sleep( 1000 );
  }

  printf("I'm waiting for the stop message");
  while(1){
    //Wait for stop message
    read_from_server (s_bt, string, 58);
    type = string[4];
    if (type ==MSG_STOP){
      return;
    }
  }
}

