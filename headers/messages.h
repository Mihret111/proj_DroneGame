// messages.h
// Central definition of the messages sent over pipes between processes.
// This header is included by B, I, and D.
// ===========================================

#ifndef MESSAGES_H
#define MESSAGES_H


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

#endif // MESSAGES_H
