#include "headers/messages.h"
#include "headers/params.h"
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

void run_obstacle_process(int write_fd, SimParams params) {
    srand((unsigned)time(NULL) ^ getpid());

    double world_half = params.world_half;

    while (1) {
        ObstacleSetMsg msg;
        msg.count = MAX_OBSTACLES;

        int life_steps = 1000; // ~300 D-updates lifetime (TODO tune more and decide best)

        for (int i = 0; i < msg.count; ++i) {
            double x = ((double)rand() / RAND_MAX) * 2.0 * world_half - world_half;
            double y = ((double)rand() / RAND_MAX) * 2.0 * world_half - world_half;

            msg.obs[i].x          = x;
            msg.obs[i].y          = y;
            msg.obs[i].life_steps = life_steps;
        }

        if (write(write_fd, &msg, sizeof(msg)) == -1) {
            perror("[O] write to B failed");
            break;
        }

        // Sleep a while before generating the next set.
        // B will kill old obstacles via life_steps in the meantime.
        sleep(40);
    }

    close(write_fd);
    _exit(EXIT_SUCCESS);
}
