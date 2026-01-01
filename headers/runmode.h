#ifndef RUNMODE_H
#define RUNMODE_H

#include <stdbool.h>

typedef enum {
    MODE_STANDALONE = 0,
    MODE_SERVER     = 1,
    MODE_CLIENT     = 2
} RunMode;

typedef struct {
    RunMode mode;
    char    server_ip[64];  // used only in client mode
    int     port;           // used in server+client
} RunConfig;

#endif
