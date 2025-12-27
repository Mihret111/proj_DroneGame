// headers/watchdog.h
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <sys/types.h> // pid_t

// PIDs that Watchdog will supervise.
// We send this struct from master to W *once* at startup.
typedef struct {
    pid_t pid_B;   // Server / Blackboard
    pid_t pid_I;   // Keyboard
    pid_t pid_D;   // Dynamics
    pid_t pid_O;   // Obstacles generator
    pid_t pid_T;   // Targets generator
} WatchPids;

// Run watchdog process.
// - cfg_read_fd: W reads WatchPids from here at startup
// - warn_sec: seconds without heartbeat before sending warning to B
// - kill_sec: seconds without heartbeat before terminating everyone
void run_watchdog_process(int cfg_read_fd, int warn_sec, int kill_sec);

#endif
