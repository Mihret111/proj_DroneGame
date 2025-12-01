// keyboard.c
// Implementation of the keyboard process (I).
// This is the ONLY process that reads from stdin.
// ======================================================================

#include "headers/messages.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

// ----------------------------------------------------------------------
// Keyboard process:
//   - Reads characters from stdin 
//   - Wraps each into KeyMsg and writes to the pipe to B.
//   - Exits on EOF or 'q'.
// ----------------------------------------------------------------------
void run_keyboard_process(int write_fd) {
    // Unbuffer stdout so debug messages appear immediately.
    setbuf(stdout, NULL);

    fprintf(stderr,
        "[I] Keyboard process started.\n"
        "[I] Use w e r / s d f / x c v to command force.\n"
        "[I] 'd' = brake, 'p' = pause, 'R' = reset, 'q' = quit.\n");

    while (1) {
        int c = getchar(); // a blocking read from stdin

        if (c == EOF) {
            fprintf(stderr, "[I] EOF on stdin, exiting keyboard process.\n");
            break;
        }

        KeyMsg km;
        km.key = (char)c;

        // Send key to B through pipe.
        if (write(write_fd, &km, sizeof(km)) == -1) {
            perror("[I] write to B failed");
            break;
        }

        if (km.key == 'q') {
            fprintf(stderr, "[I] 'q' pressed, exiting keyboard process.\n");
            break;
        }
    }

    close(write_fd);
    exit(EXIT_SUCCESS);
}
