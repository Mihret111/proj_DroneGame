#include "headers/messages.h"
#include "headers/params.h"
#include "headers/targets.h"
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

#define _GNU_SOURCE
#include <math.h>

// #define M_PI 3.14159265358979323846


// Helper: uniform random double in [min, max].
// ---------------------------------------------------------------------
static double rand_in_range(double min, double max) {
    double u = (double)rand() / (double)RAND_MAX;  // in [0,1]
    return min + u * (max - min);                  // linear interpolation
}

void run_target_process(int write_fd, SimParams params) {
    srand((unsigned)time(NULL) ^ (getpid() << 1));

    double world_half = params.world_half;

    // ----  PARAMETERS -----------------------------------------
    // How long each target stays alive in terms of B's "state updates".
    const int life_steps_default = 1000;   // e.g. 1000 physics steps

    // We want targets mostly in the central area, radius < central_factor * world_half.
    const double central_factor  = 0.5;    // e.g. inner 50% radius
    double max_r                 = world_half * central_factor;

    // Minimum spacing between targets in the same batch.
    const double spacing_factor  = 0.12;   // 12% of world_half
    double min_spacing           = world_half * spacing_factor;
    double min_spacing2          = min_spacing * min_spacing;

    // Max attempts per target to find a non-overlapping position.
    // This value should be more than the corresponding amount 
    // for obstacles, since this is the checker for the sage spacing between it and an obstacle neighbour
    const int max_attempts       = 50;  

    // How often we *try* to spawn a new batch of targets (in seconds).
    // B decides whether to overwrite existing targets or ignore, depending on your logic.
    const unsigned spawn_interval_sec = 50;   // TODO tune more
    // -----------------------------------------------------------------
    while (1) {
        TargetSetMsg msg;

        // How many targets per batch? You can use MAX_TARGETS,
        // or choose a smaller number if you want fewer at a time.
        int batch_count = MAX_TARGETS;   // or e.g. 3 or 5
        msg.count = batch_count;

        for (int i = 0; i < batch_count; ++i) {
            int attempts = 0;
            int placed   = 0;

            while (attempts < max_attempts) {
                attempts++;

                // Sample position in a central disk of radius max_r:
                //
                // - theta ∈ [0, 2π)
                // - r ∈ [0, max_r], but to make uniform in area
                //   we sample sqrt(u) * max_r
                double theta = rand_in_range(0.0, 2.0 * M_PI);
                double u     = (double)rand() / (double)RAND_MAX; // [0,1]
                double r     = sqrt(u) * max_r;   // area-uniform disk

                double x = r * cos(theta);
                double y = r * sin(theta);

                // Check spacing with already placed targets in this batch.
                int ok = 1;
                for (int j = 0; j < i; ++j) {
                    double dx = x - msg.tgt[j].x;
                    double dy = y - msg.tgt[j].y;
                    double d2 = dx*dx + dy*dy;
                    if (d2 < min_spacing2) {
                        ok = 0;
                        break;
                    }
                }

                if (ok) {
                    msg.tgt[i].x          = x;
                    msg.tgt[i].y          = y;
                    msg.tgt[i].life_steps = life_steps_default;
                    placed = 1;
                    break;
                }
            }

            if (!placed) {
                // Fallback: just pick some central point without spacing check
                double theta = rand_in_range(0.0, 2.0 * M_PI);
                double u     = (double)rand() / (double)RAND_MAX;
                double r     = sqrt(u) * max_r;

                double x = r * cos(theta);
                double y = r * sin(theta);

                msg.tgt[i].x          = x;
                msg.tgt[i].y          = y;
                msg.tgt[i].life_steps = life_steps_default;
            }
        }

        // Send batch to B.
        if (write(write_fd, &msg, sizeof(msg)) == -1) {
            perror("[T] write to B failed");
            break;
        }

        // Wait before generating the next batch.
        sleep(spawn_interval_sec);
    }

    close(write_fd);
    _exit(EXIT_SUCCESS);
}
