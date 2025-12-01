// util.c
// shared helper functions
// ======================================================================

#include "headers/util.h"

#include <stdio.h>
#include <stdlib.h>

// Return max of two ints.
// ----------------------------------------------------------------------
int imax(int a, int b) {
    return (a > b) ? a : b;
}


// Print error  and terminate program.
// ----------------------------------------------------------------------
void die(const char *msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}

// Convert a key to direction increments in the 9-key cluster.
// The resulting increments are multiplied by force_step to build actual force.
// ----------------------------------------------------------------------
void direction_from_key(char key, double *dFx, double *dFy) {
    *dFx = 0.0;
    *dFy = 0.0;

    switch (key) {
        case 'w': *dFx = -1; *dFy = +1; break; // up-left
        case 'e': *dFx =  0; *dFy = +1; break; // up
        case 'r': *dFx = +1; *dFy = +1; break; // up-right

        case 's': *dFx = -1; *dFy =  0; break; // left
        case 'd': *dFx =  0; *dFy =  0; break; // brake (handled specially)
        case 'f': *dFx = +1; *dFy =  0; break; // right

        case 'x': *dFx = -1; *dFy = -1; break; // down-left
        case 'c': *dFx =  0; *dFy = -1; break; // down
        case 'v': *dFx = +1; *dFy = -1; break; // down-right

        default:
            // any other key results in no directional change
            break;
    }
}
