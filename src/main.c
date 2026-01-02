// main.c  (FIXED STRUCTURE FOR ASSIGNMENT 3 MODES)

#include "headers/params.h"
#include "headers/util.h"
#include "headers/keyboard.h"
#include "headers/dynamics.h"
#include "headers/server.h"
#include "headers/runmode.h"

#include "headers/obstacles.h"
#include "headers/targets.h"
#include "headers/watchdog.h"

#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>   // memset, strcpy

// ------------------------------
// Step 0: Prompt user BEFORE ncurses
// ------------------------------
static void prompt_run_config(RunConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->mode = MODE_STANDALONE;
    cfg->port = 5001;
    strcpy(cfg->server_ip, "127.0.0.1");

    printf("\n=== Select Mode of Operation ===\n");
    printf("1) standalone\n");
    printf("2) server\n");
    printf("3) client\n");
    char input[32];
    while (1) {
        printf("Select [1, 2 or 3] (q to quit): ");
        fflush(stdout);
        
        if (scanf("%31s", input) != 1) exit(0); // Handle EOF

        if (strcmp(input, "1") == 0) {
            cfg->mode = MODE_STANDALONE;
            break;
        } else if (strcmp(input, "2") == 0) {
            cfg->mode = MODE_SERVER;
            break;
        } else if (strcmp(input, "3") == 0) {
            cfg->mode = MODE_CLIENT;
            break;
        } else if (strcmp(input, "q") == 0) {
            exit(0);
        } else {
            printf("Invalid choice. ");
        }
    }

    if (cfg->mode != MODE_STANDALONE) {
        printf("Port [default 5001]: ");
        fflush(stdout);
        int p;
        if (scanf("%d", &p) == 1 && p > 0 && p < 65536) cfg->port = p;

        if (cfg->mode == MODE_CLIENT) {
            printf("Server IP (e.g. 192.168.1.10): ");  
            fflush(stdout);
            scanf("%63s", cfg->server_ip);
        }
    }

    printf("\nSelected mode: %s\n",
           (cfg->mode==MODE_STANDALONE)?"standalone":
           (cfg->mode==MODE_SERVER)?"server":"client");
    printf("====================================\n\n");
    fflush(stdout);
}

int main(void) {
    // ------------------------------
    // Step 0: choose mode
    // ------------------------------
    RunConfig cfg;
    prompt_run_config(&cfg);

    // Ensures logs/ directory exists
    ensure_logs_dir();

    // ------------------------------
    // Step 1: Load params 
    // ------------------------------
    SimParams params;
    init_default_params(&params);
    load_params_from_file("params.txt", &params);

    // ------------------------------
    // Step 2: Create the ALWAYS-needed pipes (I<->B<->D)
    // ------------------------------
    int pipe_I_to_B[2]; // I writes, B reads
    int pipe_B_to_D[2]; // B writes, D reads
    int pipe_D_to_B[2]; // D writes, B reads

    if (pipe(pipe_I_to_B) == -1) die("pipe I->B");
    if (pipe(pipe_B_to_D) == -1) die("pipe B->D");
    if (pipe(pipe_D_to_B) == -1) die("pipe D->B");

    // ------------------------------
    // Step 3: Fork Keyboard process (I)
    // ------------------------------
    pid_t pid_I = fork();
    if (pid_I == -1) die("fork I");

    if (pid_I == 0) {
        // CHILD I: uses only pipe_I_to_B[1]
        close(pipe_I_to_B[0]);          // close read end
        close(pipe_B_to_D[0]); close(pipe_B_to_D[1]);
        close(pipe_D_to_B[0]); close(pipe_D_to_B[1]);

        run_keyboard_process(pipe_I_to_B[1]);
        // never returns
    }

    // ------------------------------
    // Step 4: Fork Dynamics process (D)
    // ------------------------------
    pid_t pid_D = fork();
    if (pid_D == -1) die("fork D");

    if (pid_D == 0) {
        // CHILD D: reads from pipe_B_to_D[0], writes to pipe_D_to_B[1]
        close(pipe_B_to_D[1]);      // close write end
        close(pipe_D_to_B[0]);      // close read end
        close(pipe_I_to_B[0]); close(pipe_I_to_B[1]);

        run_dynamics_process(pipe_B_to_D[0], pipe_D_to_B[1], params);
        // never returns
    }

    // ------------------------------
    // Step 5: Extras (O, T, W) ONLY in standalone mode
    // ------------------------------
    int   fd_obs_read = -1;
    int   fd_tgt_read = -1;
    pid_t pid_W       = -1;

    pid_t pid_O = -1;
    pid_t pid_T = -1;

    int pipe_O_to_B[2] = {-1, -1};
    int pipe_T_to_B[2] = {-1, -1};
    int pipe_CFG_to_W[2] = {-1, -1};

    if (cfg.mode == MODE_STANDALONE) {
        // Create pipes O->B and T->B
        if (pipe(pipe_O_to_B) == -1) die("pipe O->B");
        if (pipe(pipe_T_to_B) == -1) die("pipe T->B");

        // Create config pipe master->W
        if (pipe(pipe_CFG_to_W) == -1) die("pipe CFG->W");

        // ---- fork O
        pid_O = fork();
        if (pid_O == -1) die("fork O");

        if (pid_O == 0) {
            close(pipe_O_to_B[0]);              // O writes only
            close(pipe_T_to_B[0]); close(pipe_T_to_B[1]);
            close(pipe_CFG_to_W[0]); close(pipe_CFG_to_W[1]);

            close(pipe_I_to_B[0]); close(pipe_I_to_B[1]);
            close(pipe_B_to_D[0]); close(pipe_B_to_D[1]);
            close(pipe_D_to_B[0]); close(pipe_D_to_B[1]);

            run_obstacle_process(pipe_O_to_B[1], params);
        }

        // ---- fork T
        pid_T = fork();
        if (pid_T == -1) die("fork T");

        if (pid_T == 0) {
            close(pipe_T_to_B[0]);              // T writes only
            close(pipe_O_to_B[0]); close(pipe_O_to_B[1]);
            close(pipe_CFG_to_W[0]); close(pipe_CFG_to_W[1]);

            close(pipe_I_to_B[0]); close(pipe_I_to_B[1]);
            close(pipe_B_to_D[0]); close(pipe_B_to_D[1]);
            close(pipe_D_to_B[0]); close(pipe_D_to_B[1]);

            run_target_process(pipe_T_to_B[1], params);
        }

        // ---- fork W
        pid_W = fork();
        if (pid_W == -1) die("fork W");

        if (pid_W == 0) {
            // W reads config from pipe_CFG_to_W[0]
            close(pipe_CFG_to_W[1]);

            // Close everything else
            close(pipe_I_to_B[0]); close(pipe_I_to_B[1]);
            close(pipe_B_to_D[0]); close(pipe_B_to_D[1]);
            close(pipe_D_to_B[0]); close(pipe_D_to_B[1]);
            close(pipe_O_to_B[0]); close(pipe_O_to_B[1]);
            close(pipe_T_to_B[0]); close(pipe_T_to_B[1]);

            run_watchdog_process(pipe_CFG_to_W[0], params.wd_warn_sec, params.wd_kill_sec);
        }

        // Parent ( B) reads from O and T
        close(pipe_O_to_B[1]);   // close write end
        close(pipe_T_to_B[1]);   // close write end
        fd_obs_read = pipe_O_to_B[0];
        fd_tgt_read = pipe_T_to_B[0];

        // Send PIDs to watchdog (master writes one-time config)
        close(pipe_CFG_to_W[0]); // parent writes
        WatchPids wp;
        wp.pid_B = getpid(); // master will become B
        wp.pid_I = pid_I;
        wp.pid_D = pid_D;
        wp.pid_O = pid_O;
        wp.pid_T = pid_T;

        if (write(pipe_CFG_to_W[1], &wp, sizeof(wp)) != (int)sizeof(wp)) {
            perror("[MAIN/B] write WatchPids to W failed");
        }
        close(pipe_CFG_to_W[1]);
    }

    // ------------------------------
    // Step 6: Parent becomes Server (B)
    // ------------------------------

    // Close ends B doesn't use
    close(pipe_I_to_B[1]);   // B reads from I
    close(pipe_B_to_D[0]);   // B writes to D
    close(pipe_D_to_B[1]);   // B reads from D

    // ------------------------------
    // Step 7: Run B (server/client/standalone)
    // In network mode, B will do handshake before ncurses
    // ------------------------------
    run_server_process(pipe_I_to_B[0],
                       pipe_B_to_D[1],
                       pipe_D_to_B[0],
                       fd_obs_read,
                       fd_tgt_read,
                       pid_W,
                       params,
                       cfg);

    // ------------------------------
    // Step 8: Reap children 
    // ------------------------------
    while (wait(NULL) > 0) {}
    return 0;
}
