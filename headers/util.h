// util.h
// Small helper functions shared among modules.
// ======================================================================

#ifndef UTIL_H
#define UTIL_H

// Print error message (with errno) and exit.
void die(const char *msg);

// Integer max (tiny helper for select()).
int  imax(int a, int b);

// Map the keys (w,e,r,s,d,f,x,c,v) to corresponding unit direction increments (dFx, dFy).
void direction_from_key(char key, double *dFx, double *dFy);

#endif // UTIL_H
