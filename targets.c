#include "headers/messages.h"
#include "headers/params.h"
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

void run_target_process(int write_fd, SimParams params) {
    srand((unsigned)time(NULL) ^ (getpid() << 1));

    double world_half = params.world_half;

    while (1) {
        TargetSetMsg msg;
        msg.count = MAX_TARGETS;  // or smaller

        int life_steps = 1000; // lifetime in B's steps

        for (int i = 0; i < msg.count; ++i) {
            double x = ((double)rand() / RAND_MAX) * 2.0 * world_half - world_half;
            double y = ((double)rand() / RAND_MAX) * 2.0 * world_half - world_half;

            msg.tgt[i].x          = x;
            msg.tgt[i].y          = y;
            msg.tgt[i].life_steps = life_steps;
        }

        if (write(write_fd, &msg, sizeof(msg)) == -1) {
            perror("[T] write to B failed");
            break;
        }

        sleep(50);    // tuned for better behavior
    }

    close(write_fd);
    _exit(EXIT_SUCCESS);
}
