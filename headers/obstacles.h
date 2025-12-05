// obstacles.h
#ifndef OBSTACLES_H
#define OBSTACLES_H

/* typedef struct {
    double x;   // world x coordinate
    double y;   // world y coordinate
} Obstacle; */
#define NUM_OBSTACLES 5

typedef struct {
    double x;
    double y;
    int    active;      // 1 = currently present, 0 = off
    int    life_steps;  // how many state updates left before disappearing
} Obstacle;

static Obstacle g_obstacles[NUM_OBSTACLES];
void run_obstacle_process(int write_fd, SimParams params) ;
static double rand_in_range(double min, double max);
#endif // OBSTACLES_H
