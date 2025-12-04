// messages.h
// Central definition of the messages sent over pipes between processes.
// This header is included by B, I, and D.
// ===========================================

#ifndef MESSAGES_H
#define MESSAGES_H

// Max numbers (match NUM_OBSTACLES / NUM_TARGETS in util.h)
#define MAX_OBSTACLES 8
#define MAX_TARGETS   8

// Message: Keyboard -> Server (I -> B)
// Contains exactly one key pressed by the user.

typedef struct {
    char key;   // e.g. 'w', 'e', 'd', 'R', 'q', ...
} KeyMsg;


// Message: Server -> Dynamics (B -> D)
// Contains the commanded force and a reset flag.

typedef struct {
    double Fx;   // total commanded force in x
    double Fy;   // total commanded force in y
    int    reset; // 0 = normal, 1 = reset state in D
} ForceStateMsg;


// Message: Dynamics -> Server (D -> B)
// Contains the current drone state.

typedef struct {
    double x, y;    // position
    double vx, vy;  // velocity
} DroneStateMsg;

// Message: Obstacles -> Server (O -> B)
typedef struct {
    double x;
    double y;
    int    life_steps;  // how long this object should live (in B's update steps)
} ObstacleSpec;

typedef struct {
    int    count;       // how many obstacles in this message (≤ MAX_OBSTACLES)
    ObstacleSpec obs[MAX_OBSTACLES];
} ObstacleSetMsg;

// Message: Targets -> Server (T -> B)
typedef struct {
    double x;
    double y;
    int    life_steps;
} TargetSpec;

typedef struct {
    int       count;         // how many targets in this message (≤ MAX_TARGETS)
    TargetSpec tgt[MAX_TARGETS];
} TargetSetMsg;

#endif // MESSAGES_H
