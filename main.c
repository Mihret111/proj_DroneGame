// main.c 
// Defines the Master process:
//   - Loads parameters
//   - Creates pipes
//   - Forks I and D
//   - Becomes B, the server after bearing its children
// ======================================================================

#include "headers/params.h"
#include "headers/util.h"
#include "headers/keyboard.h"
#include "headers/dynamics.h"
#include "headers/server.h"

#include "headers/obstacles.h"
#include "headers/targets.h"

#include <unistd.h>
#include <sys/wait.h>


int main(void) {
    // 1) Loads parameters BEFORE forking so children inherit the struct.
    SimParams params;
    init_default_params(&params);
    load_params_from_file("params.txt", &params);

    // 2) Creates pipes:
    //    - I -> B
    //    - B -> D
    //    - D -> B
    int pipe_I_to_B[2];
    int pipe_B_to_D[2];
    int pipe_D_to_B[2];
    // Creates pipes from obs and targets
    //    - O -> B
    //    - T -> B
    int pipe_O_to_B[2];
    int pipe_T_to_B[2];

    if (pipe(pipe_I_to_B) == -1) die("pipe I->B");
    if (pipe(pipe_B_to_D) == -1) die("pipe B->D");
    if (pipe(pipe_D_to_B) == -1) die("pipe D->B");
    //
    if (pipe(pipe_O_to_B) == -1) die("pipe O->B");
    if (pipe(pipe_T_to_B) == -1) die("pipe T->B");
    
    // 3) Forks Keyboard process (I)
    pid_t pid_I = fork();
    if (pid_I == -1) die("fork I");

    if (pid_I == 0) {
        // CHILD: I
        close(pipe_I_to_B[0]);   // I only writes to I->B[1]
        close(pipe_B_to_D[0]); close(pipe_B_to_D[1]);
        close(pipe_D_to_B[0]); close(pipe_D_to_B[1]);
        run_keyboard_process(pipe_I_to_B[1]);
    }

    // 4) Forks Dynamics process (D)
    pid_t pid_D = fork();
    if (pid_D == -1) die("fork D");

    if (pid_D == 0) {
        // CHILD: D
        close(pipe_I_to_B[0]); close(pipe_I_to_B[1]);
        close(pipe_B_to_D[1]);   // D reads from B->D[0]
        close(pipe_D_to_B[0]);   // D writes to D->B[1]
        run_dynamics_process(pipe_B_to_D[0], pipe_D_to_B[1], params);
    }

    // 5) Forks Obstacles process (O)
    pid_t pid_O = fork();
    if (pid_O == -1) die("fork O");

    if (pid_O == 0) {
        // CHILD: Obstacle generator
        close(pipe_O_to_B[0]);   // O writes to O->B[1]
        // Closes all unused ends: I, B->D, D->B, T pipes...
        run_obstacle_process(pipe_O_to_B[1], params);
    }

    // 6) Forks Targets process (T)
    pid_t pid_T = fork();
    if (pid_T == -1) die("fork T");

    if (pid_T == 0) {
        // CHILD: Target generator
        close(pipe_T_to_B[0]);   // T writes to T->B[1]
        // Closes all unused ends
        run_target_process(pipe_T_to_B[1], params);
    }

    // 7) PARENT: Becomes Server B
    close(pipe_I_to_B[1]);  // B reads from I->B[0]
    close(pipe_B_to_D[0]);  // B writes to B->D[1]
    close(pipe_D_to_B[1]);  // B reads from D->B[0]

    close(pipe_O_to_B[1]);  // B reads from O->B[0]
    close(pipe_T_to_B[1]);  // B reads from T->B[0]

    run_server_process(pipe_I_to_B[0],
                    pipe_B_to_D[1],
                    pipe_D_to_B[0],
                    pipe_O_to_B[0],
                    pipe_T_to_B[0],
                    params);

    // 8) Waits for children to avoid zombies (good practice)
    wait(NULL);
    wait(NULL);

    return 0;
}
