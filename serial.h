#ifndef serial_h
#define serial_h

#include <stdint.h>
#include <pthread.h>
#define SWBUFMAX    256
#define SWREADMAX   128
#define SWWRITEMAX  128

#ifdef __cplusplus
extern "C" {
#endif

typedef struct serial {
  char    *port;
  int     fd;
  int8_t  connected;
  int     baudrate;
  int     parity;
 
  /* threaded update */
  pthread_t thread;
  int8_t    alive;

  /* values */
  char    buffer[SWBUFMAX];
  char    readbuf[SWREADMAX];
  int8_t  readAvailable;
  int8_t  bitShift; /* why do we have to do this hack -_- */
  int8_t  checkErr;
} serial_t;

int serial_connect(serial_t *connection, char *port, int baudrate, int parity = 0);
char *serial_read(serial_t *connection);
void serial_write(serial_t *connection, char *message);
void serial_disconnect(serial_t *connection);

#ifdef __cplusplus
}
#endif

#endif
