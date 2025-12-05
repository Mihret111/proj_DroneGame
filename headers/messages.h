// messages.h
// Definition of the messages sent over pipes between the generated processes
// This header is included by B, I, and D
// ===========================================

#ifndef MESSAGES_H
#define MESSAGES_H

// Defines max numbers (match NUM_OBSTACLES / NUM_TARGETS in util.h)
#define MAX_OBSTACLES 8
#define MAX_TARGETS   8

// Defines message: Keyboard -> Server (I -> B)
// Contains exactly one key pressed by the user.

typedef struct {
    char key;   // e.g. 'w', 'e', 'd', 'R', 'q', ...
} KeyMsg;


// Defines message: Server -> Dynamics (B -> D)
// Contains the commanded force and a reset flag.

typedef struct {
    double Fx;   // total commanded force in x
    double Fy;   // total commanded force in y
    int    reset; // 0 = normal, 1 = reset state in D
} ForceStateMsg;


// Defines message: Dynamics -> Server (D -> B)
// Contains the current drone state.

typedef struct {
    double x, y;    // position
    double vx, vy;  // velocity
} DroneStateMsg;

// Defines message: Obstacles -> Server (O -> B)
typedef struct {
    double x;
    double y;
    int    life_steps;  // defines how long the obstacle lives (in B's update steps)
} ObstacleSpec;

typedef struct {
    int    count;       // how many obstacles in this message (≤ MAX_OBSTACLES)
    ObstacleSpec obs[MAX_OBSTACLES];
} ObstacleSetMsg;

// Defines message: Targets -> Server (T -> B)
typedef struct {
    double x;
    double y;
    int    life_steps;  // defines how long the target lives (in B's update steps)
} TargetSpec;

typedef struct {
    int       count;         // how many targets in this message (≤ MAX_TARGETS)
    TargetSpec tgt[MAX_TARGETS];
} TargetSetMsg;

#endif // MESSAGES_H
