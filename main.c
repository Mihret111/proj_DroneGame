// main.c 
// The Master process:
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


#include <unistd.h>
#include <sys/wait.h>


int main(void) {
    // 1) Load parameters BEFORE forking so children inherit the struct.
    SimParams params;
    init_default_params(&params);
    load_params_from_file("params.txt", &params);

    // 2) Create pipes:
    //    - I -> B
    //    - B -> D
    //    - D -> B
    int pipe_I_to_B[2];
    int pipe_B_to_D[2];
    int pipe_D_to_B[2];

    if (pipe(pipe_I_to_B) == -1) die("pipe I->B");
    if (pipe(pipe_B_to_D) == -1) die("pipe B->D");
    if (pipe(pipe_D_to_B) == -1) die("pipe D->B");

    // 3) Fork Keyboard process (I)
    pid_t pid_I = fork();
    if (pid_I == -1) die("fork I");

    if (pid_I == 0) {
        // CHILD: I
        close(pipe_I_to_B[0]);   // I only writes to I->B[1]
        close(pipe_B_to_D[0]); close(pipe_B_to_D[1]);
        close(pipe_D_to_B[0]); close(pipe_D_to_B[1]);
        run_keyboard_process(pipe_I_to_B[1]);
    }

    // 4) Fork Dynamics process (D)
    pid_t pid_D = fork();
    if (pid_D == -1) die("fork D");

    if (pid_D == 0) {
        // CHILD: D
        close(pipe_I_to_B[0]); close(pipe_I_to_B[1]);
        close(pipe_B_to_D[1]);   // D reads from B->D[0]
        close(pipe_D_to_B[0]);   // D writes to D->B[1]
        run_dynamics_process(pipe_B_to_D[0], pipe_D_to_B[1], params);
    }

    // 5) PARENT: becomes Server B
    close(pipe_I_to_B[1]);  // B reads from I->B[0]
    close(pipe_B_to_D[0]);  // B writes to B->D[1]
    close(pipe_D_to_B[1]);  // B reads from D->B[0]

    run_server_process(pipe_I_to_B[0], pipe_B_to_D[1], pipe_D_to_B[0], params);

    // 6) Wait for children to avoid zombies (good practice)
    wait(NULL);
    wait(NULL);

    return 0;
}
