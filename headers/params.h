// params.h
// Simulation parameters (M, K, dt, force step, world size, etc.).
// Loaded once in main() and passed by value into B and D.
// ======================================================================

#ifndef PARAMS_H
#define PARAMS_H

typedef struct {
    double mass;        // M: mass of the drone
    double visc;        // K: viscous friction coefficient
    double dt;          // T: timestep (seconds)
    double force_step;  // increment in force for each directional key
    double world_half;  // world range: x,y ∈ [-world_half, +world_half]

    // These are here for future extensions (walls, obstacles etc.).
    double wall_clearance; // distance from wall where repulsion starts
    double wall_gain;      // strength of repulsive force
} SimParams;

// Fill p with sensible default values.
void init_default_params(SimParams *p);

// Override defaults with values from params.txt, if present.
void load_params_from_file(const char *filename, SimParams *p);

#endif // PARAMS_H
