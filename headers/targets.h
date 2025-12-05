// targets.h
#ifndef TARGETS_H
#define TARGETS_H
#define _GNU_SOURCE


#define NUM_TARGETS 5  // example number; adjust as required




typedef struct {
    double x;
    double y;
    int    active;      // 1 = visible, 0 = not
    int    life_steps;  // lifetime in steps
} Target;

Target g_targets[NUM_TARGETS];
void run_target_process(int write_fd, SimParams params);
#endif // TARGETS_H