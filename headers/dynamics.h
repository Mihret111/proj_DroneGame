// dynamics.h
// ======================================================================
// Interface for the dynamics process (D).
// ======================================================================

#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "params.h"

// Run the dynamics process:
//   - Reads ForceStateMsg from force_fd (from B)
//   - Integrates dynamics
//   - Sends DroneStateMsg to state_fd (to B)
void run_dynamics_process(int force_fd, int state_fd, SimParams params);

#endif // DYNAMICS_H
