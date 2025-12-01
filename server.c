// server.c
// server / blackboard process (B)
//   - Owns global "blackboard" state: force and drone state
//   - Listens to keys from I and states from D (via pipes)
//   - Sends updated forces to D
//   - Draws ncurses User Interface comprising of the drone world and an inspection window
//   - Reacts to the commands pause 'p', reset 'R', brake 'd', quit 'q'
// ======================================================================

#include "headers/server.h"
#include "headers/messages.h"
#include "headers/util.h"

#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>     // exit, strtod
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>

#include <sys/select.h>   // for fd_set, FD_ZERO, FD_SET, select()


void run_server_process(int fd_kb, int fd_to_d, int fd_from_d, SimParams params) {
    // --- Initialize ncurses ---
    initscr();
    cbreak();
    noecho();
    curs_set(0);  // hide cursor

    // --- Open logfile ---
    FILE *logfile = fopen("log.txt", "w");
    if (!logfile) {
        endwin();
        die("[B] cannot open log.txt");
    }

    // --- Blackboard state (shared model of the world) ---
    ForceStateMsg cur_force;
    cur_force.Fx = 0.0;
    cur_force.Fy = 0.0;
    cur_force.reset = 0;

    DroneStateMsg cur_state = (DroneStateMsg){0.0, 0.0, 0.0, 0.0};
    char last_key = '?';
    bool paused = false;

    // Send initial zero-force to D so it starts defined.
    if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
        fclose(logfile);
        endwin();
        die("[B] initial write to D failed");
    }

    int max_y, max_x;

    // --- Main event loop ---
    while (1) {
        // ------------------------------------------------------------------
        // 1) Query current terminal size (for resizing).
        // ------------------------------------------------------------------
        getmaxyx(stdscr, max_y, max_x);

        // Layout planning:
        //   - 2 top lines of info
        //   - horizontal separator
        //   - world area below
        //   - inspection panel on the right
        int content_top    = 1;                 // first row inside border
        int top_lines      = 2;                 // 2 text lines at top
        int top_info_y1    = content_top;
        int top_info_y2    = content_top + 1;
        int sep_y          = content_top + top_lines; // horizontal separator row
        int content_bottom = max_y - 2;         // last row inside bottom border

        if (sep_y >= content_bottom) {
            sep_y = content_top; // in tiny terminals
        }

        // Right inspection panel width
        int insp_width = 35;
        if (max_x < insp_width + 10) {
            insp_width = max_x / 3;
            if (insp_width < 20) insp_width = 20;
        }
        int insp_start_x = max_x - insp_width;
        if (insp_start_x < 1) insp_start_x = 1;

        // World area is below separator.
        int world_top    = sep_y + 1;
        if (world_top > content_bottom) world_top = content_top + 1;
        int world_bottom = content_bottom;
        int world_height = world_bottom - world_top + 1;
        if (world_height < 1) world_height = 1;

        // Left world width.
        int main_width = insp_start_x - 2;
        if (main_width < 10) main_width = 10;

        // ------------------------------------------------------------------
        // 2) Use select() to wait for data from keyboard and dynamics.
        //    Also handle EINTR (e.g., from SIGWINCH on resize).
        // ------------------------------------------------------------------
        fd_set rfds;
        int maxfd = imax(fd_kb, fd_from_d) + 1;

        int sel;
        while (1) {
            FD_ZERO(&rfds);
            FD_SET(fd_kb,     &rfds);
            FD_SET(fd_from_d, &rfds);

            sel = select(maxfd, &rfds, NULL, NULL, NULL);

            if (sel == -1) {
                if (errno == EINTR) {
                    // Interrupted by signal (like resize) → retry
                    continue;
                } else {
                    fclose(logfile);
                    endwin();
                    die("[B] select failed");
                }
            }
            break; // sel >= 0, we have an event
        }

        // ------------------------------------------------------------------
        // 3) Handle keyboard input from I (if available).
        // ------------------------------------------------------------------
        if (FD_ISSET(fd_kb, &rfds)) {
            KeyMsg km;
            int n = read(fd_kb, &km, sizeof(km));
            if (n <= 0) {
                mvprintw(0, 1, "[B] Keyboard process ended (EOF).");
                refresh();
                break;
            }

            last_key = km.key;

            // (a) Global quit
            if (km.key == 'q') {
                fprintf(logfile, "QUIT requested by 'q'\n");
                fflush(logfile);
                break;
            }

            // (b) Pause toggle
            if (km.key == 'p') {
                paused = !paused;

                if (paused) {
                    // When entering pause, we also zero the force.
                    cur_force.Fx = 0.0;
                    cur_force.Fy = 0.0;
                    cur_force.reset = 0;
                    if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
                        fclose(logfile);
                        endwin();
                        die("[B] write to D failed (pause)");
                    }
                    fprintf(logfile, "PAUSE: ON\n");
                } else {
                    fprintf(logfile, "PAUSE: OFF\n");
                }
                fflush(logfile);
            }
            // (c) Reset (uppercase R)
            else if (km.key == 'R') {
                // Reset server-side state
                cur_state.x  = 0.0;
                cur_state.y  = 0.0;
                cur_state.vx = 0.0;
                cur_state.vy = 0.0;

                // Reset forces
                cur_force.Fx = 0.0;
                cur_force.Fy = 0.0;
                cur_force.reset = 1; // signal D to reset its state

                if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
                    fclose(logfile);
                    endwin();
                    die("[B] write to D failed (reset)");
                }

                cur_force.reset = 0; // clear locally
                paused = false;      // also unpause

                fprintf(logfile, "RESET requested (R)\n");
                fflush(logfile);
            }
            // (d) Direction / brake keys
            else {
                double dFx, dFy;
                direction_from_key(km.key, &dFx, &dFy);

                if (!paused) {
                    if (km.key == 'd') {
                        // Brake: zero forces
                        cur_force.Fx = 0.0;
                        cur_force.Fy = 0.0;
                    } else {
                        // Accumulate new force
                        cur_force.Fx += dFx * params.force_step;
                        cur_force.Fy += dFy * params.force_step;
                    }

                    cur_force.reset = 0;

                    if (write(fd_to_d, &cur_force, sizeof(cur_force)) == -1) {
                        fclose(logfile);
                        endwin();
                        die("[B] write to D failed (force)");
                    }

                    fprintf(logfile,
                            "KEY: %c  dFx=%.1f dFy=%.1f -> Fx=%.2f Fy=%.2f\n",
                            km.key, dFx, dFy, cur_force.Fx, cur_force.Fy);
                    fflush(logfile);
                } else {
                    // Paused → ignore directional changes (but still log)
                    fprintf(logfile,
                            "KEY: %c ignored (PAUSED)\n", km.key);
                    fflush(logfile);
                }
            }
        }

        // ------------------------------------------------------------------
        // 4) Handle state updates from D (if available).
        // ------------------------------------------------------------------
        if (FD_ISSET(fd_from_d, &rfds)) {
            DroneStateMsg s;
            int n = read(fd_from_d, &s, sizeof(s));
            if (n <= 0) {
                mvprintw(1, 1, "[B] Dynamics process ended (EOF).");
                refresh();
                break;
            }

            cur_state = s;

            fprintf(logfile,
                    "STATE: x=%.2f y=%.2f vx=%.2f vy=%.2f\n",
                    s.x, s.y, s.vx, s.vy);
            fflush(logfile);
        }

        // ------------------------------------------------------------------
        // 5) Draw UI (world + inspection panel).
        // ------------------------------------------------------------------
        erase();
        box(stdscr, 0, 0);

        // Top info lines
        mvprintw(top_info_y1, 2,
                 "Controls: w e r / s d f / x c v | d=brake, p=pause, R=reset, q=quit");
        mvprintw(top_info_y2, 2,
                 "Paused: %s", paused ? "YES" : "NO");

        // Horizontal separator row (under top info)
        if (sep_y >= 1 && sep_y <= max_y - 2) {
            for (int x = 1; x < max_x - 1; ++x) {
                mvaddch(sep_y, x, '-');
            }
        }

        // Vertical separator between world and inspection
        int sep_x = insp_start_x - 1;
        if (sep_x > 1 && sep_x < max_x - 1) {
            for (int y = world_top; y <= world_bottom; ++y) {
                mvaddch(y, sep_x, '|');
            }
        }

        // WORLD DRAWING (left)
        double world_half = params.world_half;
        double scale_x = main_width  / (2.0 * world_half);
        double scale_y = world_height / (2.0 * world_half);
        if (scale_x <= 0) scale_x = 1.0;
        if (scale_y <= 0) scale_y = 1.0;

        int sx = (int)(cur_state.x * scale_x) + main_width / 2 + 1;
        int sy = (int)(-cur_state.y * scale_y) + world_top + world_height / 2;

        if (sx < 1) sx = 1;
        if (sx > main_width) sx = main_width;
        if (sy < world_top) sy = world_top;
        if (sy > world_bottom) sy = world_bottom;

        mvaddch(sy, sx, '+'); // draw drone

        // INSPECTION panel on the right
        int info_y = world_top;
        int info_x = insp_start_x + 1;

        if (info_x < max_x - 1) {
            mvprintw(info_y,     info_x, "INSPECTION");
            mvprintw(info_y + 2, info_x, "Last key: %c", last_key);
            mvprintw(info_y + 4, info_x, "Fx = %.2f", cur_force.Fx);
            mvprintw(info_y + 5, info_x, "Fy = %.2f", cur_force.Fy);
            mvprintw(info_y + 7, info_x, "x  = %.2f", cur_state.x);
            mvprintw(info_y + 8, info_x, "y  = %.2f", cur_state.y);
            mvprintw(info_y + 9, info_x, "vx = %.2f", cur_state.vx);
            mvprintw(info_y +10, info_x, "vy = %.2f", cur_state.vy);
        }

        refresh();
    }

    // --- Cleanup ---
    fclose(logfile);
    endwin();
    close(fd_kb);
    close(fd_to_d);
    close(fd_from_d);
    exit(EXIT_SUCCESS);
}
