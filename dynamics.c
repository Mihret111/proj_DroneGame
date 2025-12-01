// dynamics.c
// The dynamics process (D)
// Integrates the drone state forward in time
// ======================================================================

#include "headers/dynamics.h"
#include "headers/messages.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>     // exit, strtod
#include <errno.h>
#include <time.h>
#include <fcntl.h>

// ----------------------------------------------------------------------
// Dynamics process:
//   - Keeps ForceStateMsg f (current force) and DroneStateMsg s (state).
//   - Reads new force commands from B (non-blocking).
//   - If reset flag is set, zero state.
//   - Integrates dynamics at fixed dt and sends s back to B each step.
// ----------------------------------------------------------------------
void run_dynamics_process(int force_fd, int state_fd, SimParams params) {
    setbuf(stdout, NULL);
    fprintf(stderr,
            "[D] Dynamics process started. M=%.3f, K=%.3f, dt=%.3f\n",
            params.mass, params.visc, params.dt);

    double M = params.mass;
    double K = params.visc;
    double T = params.dt;

    // Current force command from B
    ForceStateMsg f;
    f.Fx = 0.0;
    f.Fy = 0.0;
    f.reset = 0;

    // Current drone state
    DroneStateMsg s = (DroneStateMsg){0.0, 0.0, 0.0, 0.0};

    // Make force pipe non-blocking, so read() doesn't block if no new data.
    int flags = fcntl(force_fd, F_GETFL, 0);
    if (flags == -1) flags = 0;
    if (fcntl(force_fd, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("[D] fcntl O_NONBLOCK");
    }

    while (1) {
        // ------------------------------------------------------------------
        // 1) Try to read a new force command from B (non-blocking).
        // ------------------------------------------------------------------
        ForceStateMsg new_f;
        int n = read(force_fd, &new_f, sizeof(new_f));

        if (n == (int)sizeof(new_f)) {
            // Got a full new force command.
            if (new_f.reset != 0) {
                // Reset requested: zero state.
                s.x  = 0.0;
                s.y  = 0.0;
                s.vx = 0.0;
                s.vy = 0.0;
            }
            f = new_f;
            f.reset = 0; // clear locally
        } else if (n == 0) {
            // Pipe closed by B → time to exit.
            fprintf(stderr, "[D] EOF on force pipe, exiting.\n");
            break;
        } else if (n < 0) {
            // EAGAIN/EWOULDBLOCK means "no new data right now"
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("[D] read");
                break;
            }
            // else no new force; keep old f.
        } else {
            // partial read (shouldn't normally happen with our simple usage).
            fprintf(stderr, "[D] Partial read (%d bytes) on force pipe.\n", n);
        }

        // ------------------------------------------------------------------
        // 2) Integrate dynamics:
        //    dv/dt = (F - K v)/M
        //    v += (dv/dt)*T
        //    x += v*T
        // ------------------------------------------------------------------
        double ax = (f.Fx - K * s.vx) / M;
        double ay = (f.Fy - K * s.vy) / M;

        s.vx += ax * T;
        s.vy += ay * T;

        s.x  += s.vx * T;
        s.y  += s.vy * T;

        // ------------------------------------------------------------------
        // 3) Send state back to B.
        // ------------------------------------------------------------------
        if (write(state_fd, &s, sizeof(s)) == -1) {
            perror("[D] write state");
            break;
        }

        // ------------------------------------------------------------------
        // 4) Sleep until next simulation step.
        // ------------------------------------------------------------------
        struct timespec ts;
        ts.tv_sec  = 0;
        ts.tv_nsec = (long)(T * 1e9);
        nanosleep(&ts, NULL);
    }

    close(force_fd);
    close(state_fd);
    exit(EXIT_SUCCESS);
}
