// server.c
// Defines server / blackboard process (B)
//   - Owns global "blackboard" state: force and drone state
//   - Listens to keys from I and states from D (via pipes)
//   - Sends updated forces to D
//   - Monitors obstacles and targets
//   - Draws ncurses User Interface comprising of the drone world and an inspection window
//   - Reacts to the commands pause 'p', reset 'O', brake 'd', quit 'q'
// ======================================================================

#define _POSIX_C_SOURCE 200809L
#include <headers/runmode.h>
#include <headers/net.h>
#include <headers/protocol.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <signal.h>
#include <string.h>
#include <sys/select.h>   // for fd_set, FD_ZERO, FD_SET, select()
#include <sys/types.h>

#include "headers/server.h"
#include "headers/messages.h"
#include "headers/util.h"
#include "headers/obstacles.h"
#include "headers/targets.h"
#include <time.h>   // clock_gettime


#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>     // exit, strtod
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <math.h>  // for sqrt, to be used in key mapping instead of hard code, REP

//#define NUM_OBSTACLES 8
// Defines global / static array of 8 obstacles
//static Obstacle g_obstacles[NUM_OBSTACLES];

// Defines scoring globals
static int g_score             = 0;
static int g_targets_collected = 0;
static int g_last_hit_step     = -1;
static int g_step_counter      = 0;

// blinking warning banner globals
static int  wd_warning_active = 0;   // warning state ON/OFF
static int  wd_blink_phase   = 0;   // 0 or 1 (visible / invisible)
static int  wd_blink_counter = 0;   // counts simulation steps

static int wd_blink_ticks = 0;

// ---- Heartbeat timing ----
static struct timespec g_last_hb_ts;
static int g_have_hb = 0; // becomes 1 after first heartbeat timestamp is recorded

static double monotonic_now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static void set_last_hb_now(void) {
    clock_gettime(CLOCK_MONOTONIC, &g_last_hb_ts);
    g_have_hb = 1;
}

static double hb_age_sec(void) {
    if (!g_have_hb) return 0.0;
    double now = monotonic_now_sec();
    double last = (double)g_last_hb_ts.tv_sec + 1e-9 * (double)g_last_hb_ts.tv_nsec;
    return now - last;
}
// -------------------Networking utilities-------------------
// gets terminal size before ncurses init
static void get_term_size_pre_ncurses(int *W, int *H) {
    struct winsize ws;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0) {  
        *W = (int)ws.ws_col;
        *H = (int)ws.ws_row;
    } else {
        // fallback default values if ioctl fails
        *W = 120;
        *H = 40;
    }
}

// Server protocol state
typedef enum {
    SP_SEND_DRONE_TAG = 0,
    SP_SEND_DRONE_XY,
    SP_WAIT_DOK,
    SP_SEND_OBST_TAG,
    SP_WAIT_OBST_XY,
    SP_SEND_POK
} ServerProtoPhase;

static ServerProtoPhase sp_phase = SP_SEND_DRONE_TAG;
static double sp_last_ox = 0.0, sp_last_oy = 0.0;

//client protocol state
typedef enum {
    CP_WAIT_TAG = 0,
    CP_WAIT_DRONE_XY,
    CP_SEND_OBST_XY,
    CP_WAIT_POK
} ClientProtoPhase;

static ClientProtoPhase cp_phase = CP_WAIT_TAG;
static char cp_last_tag[64];



static int server_proto_tick(int net_fd,
                             const SimParams *params,
                             double myx, double myy,
                             int *have_obst, double *ox, double *oy)
{
    // Returns:
    //  0 : progressed
    //  1 : would block (timeout) -> not an error
    // -1 : hard error (disconnect/protocol error)

    char line[128];
    *have_obst = 0;

    switch (sp_phase) {

    case SP_SEND_DRONE_TAG:
        if (net_send_line(net_fd, "drone") < 0) return -1;
        sp_phase = SP_SEND_DRONE_XY;
        return 0;

    case SP_SEND_DRONE_XY:
        // if (proto_send_xy(net_fd, myx, myy) < 0) return -1;
        double xv, yv;
        local_to_virtual(myx, myy, params, &xv, &yv);       // coordinate frame conversion
        if (proto_send_xy(net_fd, xv, yv) < 0) return -1;

        sp_phase = SP_WAIT_DOK;
        return 0;

    case SP_WAIT_DOK:
        if (net_recv_line(net_fd, line, sizeof(line)) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 1; // timeout: try later
            return -1;
        }
        if (strcmp(line, "dok") != 0) return -1;
        sp_phase = SP_SEND_OBST_TAG;
        return 0;

    case SP_SEND_OBST_TAG:
        if (net_send_line(net_fd, "obst") < 0) return -1;
        sp_phase = SP_WAIT_OBST_XY;
        return 0;

    case SP_WAIT_OBST_XY: {
        double xv, yv;
        if (proto_recv_xy(net_fd, &xv, &yv) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 1;
            return -1;
        }
        double lx, ly;
        virtual_to_local(xv, yv, params, &lx, &ly);
        sp_last_ox = lx; sp_last_oy = ly;
        sp_phase = SP_SEND_POK;
        return 0;
    }

    case SP_SEND_POK:
        if (net_send_line(net_fd, "pok") < 0) return -1;

        *have_obst = 1;
        *ox = sp_last_ox;
        *oy = sp_last_oy;

        sp_phase = SP_SEND_DRONE_TAG;
        return 0;
    }

    return 0;
}


static int client_proto_tick(int net_fd,
                             const SimParams *params, 
                             double myx, double myy,
                             int *have_remote_drone,
                             double *rx, double *ry,
                             int *server_quit)
{
    // returns:
    //  0 : progressed
    //  1 : would block (timeout) -> NOT an error
    // -1 : hard error (disconnect / protocol error)

    *have_remote_drone = 0;
    *server_quit = 0;

    switch (cp_phase) {

    case CP_WAIT_TAG:
        if (net_recv_line(net_fd, cp_last_tag, sizeof(cp_last_tag)) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 1;
            return -1;
        }

        if (strcmp(cp_last_tag, "q") == 0) {
            *server_quit = 1;
            return 0;
        }

        if (strcmp(cp_last_tag, "drone") == 0) {
            cp_phase = CP_WAIT_DRONE_XY;
            return 0;
        }

        if (strcmp(cp_last_tag, "obst") == 0) {
            cp_phase = CP_SEND_OBST_XY;
            return 0;
        }

        // unknown tag
        return -1;

    case CP_WAIT_DRONE_XY:{
        double xv, yv;
        if (proto_recv_xy(net_fd, &xv, &yv) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 1;
            return -1;
        }
        virtual_to_local(xv, yv, params, rx, ry);

        if (net_send_line(net_fd, "dok") < 0) return -1;

        *have_remote_drone = 1;
        cp_phase = CP_WAIT_TAG;
        return 0;
    }

    case CP_SEND_OBST_XY:{
        // if (proto_send_xy(net_fd, myx, myy) < 0) return -1;
        double xv, yv;
        local_to_virtual(myx, myy, params, &xv, &yv);       // coordinate frame conversion
        if (proto_send_xy(net_fd, xv, yv) < 0) return -1;

        cp_phase = CP_WAIT_POK;
        return 0;
    }

    case CP_WAIT_POK: {
        char line[64];
        if (net_recv_line(net_fd, line, sizeof(line)) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 1;
            return -1;
        }
        if (strcmp(line, "pok") != 0) return -1;

        cp_phase = CP_WAIT_TAG;
        return 0;
    }
    }

    return 0;
}


// ---------------- Watchdog signal flags (set by signal handlers) ----------------
static volatile sig_atomic_t g_wd_warning_flag = 0; // set by SIGUSR2 handler
static volatile sig_atomic_t g_wd_stop    = 0;  // set when SIGTERM arrives

static void on_watchdog_warning(int signo) {
    (void)signo;
    g_wd_warning_flag = 1;
}

static void on_watchdog_stop(int signo) {
    (void)signo;
    g_wd_stop = 1;
}

// ---------------- Watchdog banner UI state ----------------
// Show a warning banner for a limited amount of time after SIGUSR2
// We store it as "how many simulation steps remaining" to show the banner.
static char watchdog_banner_msg[] = "WATCHDOG WARNING, system may be unstable"; 

/**
 * @brief Main function for the Server (B) process.
 *
 * @details
 * Acts as the "Blackboard" or central hub of the architecture.
 * - **Responsibility**: Maintains the authoritative state of the world (drone, obstacles, targets).
 * - **IPC Hub**: Multiplexes inputs from Keyboard (I), Dynamics (D), Obstacles (O), and Targets (T).
 * - **Visualization**: Draws the ncurses UI.
 * - **Synchronization**: Sends the official force commands to Dynamics to step the physics.
 * 
 * @param fd_kb      Pipe FD for reading KeyMsg from Keyboard (I).
 * @param fd_to_d    Pipe FD for writing ForceStateMsg to Dynamics (D).
 * @param fd_from_d  Pipe FD for reading DroneStateMsg from Dynamics (D).
 * @param fd_obs     Pipe FD for reading obstacles from Generator (O).
 * @param fd_tgt     Pipe FD for reading targets from Generator (T).
 * @param pid_W      PID of the Watchdog process (for sending heartbeat signals).
 * @param params     Simulation parameters.
 */
void run_server_process(int fd_kb, int fd_to_d, int fd_from_d, int fd_obs, int fd_tgt, pid_t pid_W, SimParams params, RunConfig cfg) 
{
    // --- Opens logfile ---
    // --- Opens logfile ---
    const char *log_name = (cfg.mode == MODE_CLIENT) ? "server_client" : "server_server";
    FILE *logfile = open_process_log(log_name, "B");
    if (!logfile) {
        endwin();
        char err_msg[128];
        snprintf(err_msg, sizeof(err_msg), "[B] cannot open logs/%s.log", log_name);
        die(err_msg);
    }
    // -----------------------------Networking variables------------------------
    int net_fd = -1;     // connection socket (socket fd if connected)
    int listen_fd = -1;  // listening socket (server)
    int peer_W = -1;     // remote screen width  (for scaling later)
    int peer_H = -1;     // remote screen height
    
    //
    int forced_max_x = -1;
    int forced_max_y = -1;

    // Networking state variables
    // typedef enum { NET_SEND_DRONE_TAG, NET_SEND_DRONE_XY, NET_WAIT_DOK,
    //            NET_SEND_OBST_TAG, NET_WAIT_OBST_XY, NET_SEND_POK } NetPhase;

    // static NetPhase phase = NET_SEND_DRONE_TAG;
    // static double pending_ox = 0.0, pending_oy = 0.0;

    //---------------- As seen by server ----------------
    // Remote obstacle position received from client (server interprets as one obstacle)
    // static int remote_obst_valid = 0;
    static double remote_obst_x = 0.0;
    static double remote_obst_y = 0.0;

    // Pick one obstacle slot to represent the remote obstacle in server mode
    // In server/client mode, O generator is OFF, thus we reuse slot 0
    #define REMOTE_OBST_IDX 0

    //----------------- As seen by client ----------------
    // Server drone position as seen by client
    static int remote_drone_valid = 0;
    static double remote_drone_x = 0.0;
    static double remote_drone_y = 0.0;

    // ================= Network handshake (BEFORE ncurses) =================
    if (cfg.mode == MODE_SERVER) {
        // SERVER MODE: We wait for a connection from a Client
        
        fprintf(logfile, "[B] MODE_SERVER: listen on port %d\n", cfg.port);
        fflush(logfile);

        // 1. Create and bind listening socket
        listen_fd = net_server_listen(cfg.port);
        if (listen_fd < 0) {
            fprintf(logfile, "[B] ERROR: net_server_listen failed\n");
            fflush(logfile);
            exit(EXIT_FAILURE);
        }

        fprintf(logfile, "[B] Waiting for 1 client...\n");
        fflush(logfile);

        // 2. Accept exactly one client connection
        // This blocks until a client connects
        net_fd = net_server_accept(listen_fd);

        // Set receive timeout
        net_set_rcv_timeout_ms(net_fd, params.req_timeout_ms);

        // 3. Stop listening after accepting
        // We only want 1 client for this assignment. Closing listen_fd prevents others from connecting
        if (listen_fd >= 0) {
            close(listen_fd);
        }
        listen_fd = -1;
    
        if (net_fd < 0) {
            fprintf(logfile, "[B] ERROR: net_server_accept failed\n");
            fflush(logfile);
            exit(EXIT_FAILURE);
        }

        // ---Server HANDSHAKE PROTOCOL ---
        // 1) Verify connection: Send "ok", expect "ook"
        // This confirms the other side is indeed our client protocol
        if (net_send_line(net_fd, "ok") < 0) die("[B] net_send_line(ok)");
        
        char line[256];
        // Read response (expecting "ook")
        if (net_recv_line(net_fd, line, sizeof(line)) < 0) die("[B] net_recv_line(ook)");
        if (strcmp(line, "ook") != 0) die("[B] Expected 'ook'");

        // 2) Synchronize Window Size: Send "size W H", expect "sok"
        // The server dictates the required terminal size to the client
        int W, H;
        get_term_size_pre_ncurses(&W, &H); // Get own terminal size
        
        // Send our size to client so it can check if it fits
        if (net_sendf(net_fd, "size %d %d", W, H) < 0) die("[B] net_sendf(size)");
        
        // Client confirms receipt with "sok"
        if (net_recv_line(net_fd, line, sizeof(line)) < 0) die("[B] net_recv_line(sok)");
        if (strcmp(line, "sok") != 0) die("[B] Expected 'sok'");

        fprintf(logfile, "[B] Handshake OK (server). Sent size=%dx%d\n", W, H);
        fflush(logfile);

    } else if (cfg.mode == MODE_CLIENT) {
        // CLIENT MODE: Connect to an existing Server

        fprintf(logfile, "[B] MODE_CLIENT: connect to %s:%d\n", cfg.server_ip, cfg.port);
        fflush(logfile);

        // 1. Connect to the server IP/Port.
        net_fd = net_client_connect(cfg.server_ip, cfg.port);
        
        net_set_rcv_timeout_ms(net_fd, params.req_timeout_ms);
        if (net_fd < 0) {
            fprintf(logfile, "[B] ERROR: net_client_connect failed\n");
            fflush(logfile);
            exit(EXIT_FAILURE);
        }

        char line[256];

        // --- Client HANDSHAKE PROTOCOL ---
        // 1) Verify connection: Expect "ok", send "ook"
        // Wait for server to say hello
        if (net_recv_line(net_fd, line, sizeof(line)) < 0) die("[B] net_recv_line(ok)");
        if (strcmp(line, "ok") != 0) die("[B] Expected 'ok'");
        
        // Respond: "I hear you"
        if (net_send_line(net_fd, "ook") < 0) die("[B] net_send_line(ook)");

        // 2) Synchronize Window Size: Expect "size W H", send "sok"
        // Wait for server's required dimensions
        if (net_recv_line(net_fd, line, sizeof(line)) < 0) die("[B] net_recv_line(size)");
        
        // Parse the size command
        if (sscanf(line, "size %d %d", &peer_W, &peer_H) != 2) die("[B] Bad 'size' format");
        
        // Acknowledge size receipt
        if (net_send_line(net_fd, "sok") < 0) die("[B] net_send_line(sok)");

        // 3) Client Sanity Check: ensuring local terminal is large enough
        // We cannot shrink the server's UI, so we must be at least as big as the server
        int W, H;
        get_term_size_pre_ncurses(&W, &H);
        
        if (W < peer_W || H < peer_H) {
            // Log error to stderr (visible to user immediately) and file
            fprintf(stderr, "[CLIENT] Terminal too small. Need at least %dx%d (current %dx%d)\n",
                    peer_W, peer_H, W, H);
            fprintf(logfile, "[B] Terminal too small. Need %dx%d (current %dx%d)\n",
                    peer_W, peer_H, W, H);
            fflush(logfile);
            close(net_fd); // Clean up
            exit(EXIT_FAILURE);
        }
        forced_max_x = peer_W;
        forced_max_y = peer_H;


        fprintf(logfile, "[B] Handshake OK (client). Peer size=%dx%d, local=%dx%d\n",
                peer_W, peer_H, W, H);
        fflush(logfile);
    }
    // ================================================================================ 

    
    // -----------------------------Heartbeat variables------------------------
    // Initialize heartbeat tracking
    set_last_hb_now(); // assume "alive" at start

    // --- Initialize ncurses ---
    initscr();      // Assignment-1 (previously was called inside loop which caused seldom window flickering issues)
    cbreak();
    noecho();
    curs_set(0);  // hide cursor

    // Assignment-1 (previously was defined inside loop casing uneccessary repeated calls)
    // ---- ncurses color init (DO THIS ONCE) ----
    if (has_colors()) {     
        start_color();

        // Only attempt init_color if terminal supports changing colors.
        if (can_change_color()) {
            init_color(COLOR_YELLOW, 1000, 500, 0); // orange-ish
            init_color(COLOR_GREEN,  0, 1000, 0);   // green
        }

        init_pair(1, COLOR_YELLOW, COLOR_BLACK); // obstacles
        init_pair(2, COLOR_GREEN,  COLOR_BLACK); // targets
        init_pair(3, COLOR_RED,    COLOR_BLACK); // watchdog warning
    } else {
        // If cmd doesnot permit colors, then continue without colors.
    }

    // Define the whole window size
    WINDOW *frame = NULL;
    int frame_w = 0, frame_h = 0;
    int frame_x0 = 0, frame_y0 = 0;

    // 
    if (cfg.mode == MODE_CLIENT && forced_max_x > 0 && forced_max_y > 0) {
        frame_w = forced_max_x;
        frame_h = forced_max_y;

        int term_y, term_x;
        getmaxyx(stdscr, term_y, term_x);

        // first try anchoring top-left first to eliminate mess:
        frame_y0 = 0;
        frame_x0 = 0;

        // (Later  try centering safely once everything is stable)
        // frame_y0 = (term_y - frame_h) / 2; if (frame_y0 < 0) frame_y0 = 0;
        // frame_x0 = (term_x - frame_w) / 2; if (frame_x0 < 0) frame_x0 = 0;

        frame = newwin(frame_h, frame_w, frame_y0, frame_x0);
        if (!frame) {
            endwin();
            fprintf(stderr, "[CLIENT] newwin() failed\n");
            goto shutdown_and_exit;
        }
    }



    // ---------------- Install signal handlers for Watchdog ----------------
    struct sigaction sa_warn;
    memset(&sa_warn, 0, sizeof(sa_warn));
    sa_warn.sa_handler = on_watchdog_warning;
    sigemptyset(&sa_warn.sa_mask);
    sa_warn.sa_flags = SA_RESTART;
    if (sigaction(SIGUSR2, &sa_warn, NULL) == -1) {
        fprintf(logfile, "[B] sigaction(SIGUSR2) failed: %s\n", strerror(errno));
        fflush(logfile);
    }

    struct sigaction sa_stop;
    memset(&sa_stop, 0, sizeof(sa_stop));
    sa_stop.sa_handler = on_watchdog_stop;
    sigemptyset(&sa_stop.sa_mask);
    sa_stop.sa_flags = SA_RESTART;
    if (sigaction(SIGTERM, &sa_stop, NULL) == -1) {
        fprintf(logfile, "[B] sigaction(SIGTERM) failed: %s\n", strerror(errno));
        fflush(logfile);
    }


    // --- Defines Blackboard state (model of the world)
    ForceStateMsg cur_force;
    cur_force.Fx = 0.0;
    cur_force.Fy = 0.0;
    cur_force.reset = 0;

    DroneStateMsg cur_state = (DroneStateMsg){0.0, 0.0, 0.0, 0.0};
    char last_key = '?';
    bool paused = false;

    // Sends to helper rather than directly write to D
    // Initial state is zero, so cur_state is still {0,0,0,0}.
    // Sends initial total force (which is just user=0 + obstacles repulsion).
    send_total_force_to_d(&cur_force,
                          &cur_state,
                          &params,
                          g_obstacles,
                          NUM_OBSTACLES,
                          fd_to_d,
                          logfile,
                          "init");


    int max_y, max_x;

    // Needed for handling time_since_last_hit
    double time_since_last_hit = 0; // for tracking time since last hit

    // --- Main event loop ---
    while (1) {

        // Check if client terminal tries to resize too small
        if (cfg.mode == MODE_CLIENT && forced_max_x > 0 && forced_max_y > 0) {
            int real_y, real_x;
            getmaxyx(stdscr, real_y, real_x);
            if (real_x < forced_max_x || real_y < forced_max_y) {
                endwin();
                fprintf(stderr,
                    "[CLIENT] Terminal resized too small. Need %dx%d, now %dx%d.\n"
                    "Resize back and rerun.\n",
                    forced_max_x, forced_max_y, real_x, real_y);
                goto shutdown_and_exit;
            }
        }


        // ---------------- Watchdog notifications ----------------
        // Watchdog warning -> start banner
        // Convert the SIGUSR2 flag into a visible banner for 2 seconds.
        // We do this here (NOT in signal handler) because ncurses is not signal-safe.
        if (g_wd_warning_flag) {
            g_wd_warning_flag = 0;

            // Start blinking warning until SIGTERM arrives.
            wd_warning_active = 1;
            wd_blink_phase    = 1;   // start "visible"
            wd_blink_counter  = 0;
            wd_blink_ticks    = 0;

            fprintf(logfile, "[B] WATCHDOG WARNING: blinking ON\n");
            fflush(logfile);
        }



        if (g_wd_stop) {
            fprintf(logfile, "[B] WATCHDOG STOP: received SIGTERM, exiting.\n");
            fflush(logfile);
            break; // exit from server loop
        }

        // Queries current terminal size (for resizing)
        // getmaxyx(stdscr, max_y, max_x);
        // if (cfg.mode == MODE_CLIENT && forced_max_x > 0 && forced_max_y > 0) {
        //     max_x = forced_max_x;
        //     max_y = forced_max_y;
            
        //     int term_y, term_x;
        //     getmaxyx(stdscr, term_y, term_x);

        //     // Center the frame inside the terminal (or set 0,0 if you prefer top-left)
        //     frame_y0 = (term_y - frame_h) / 2;
        //     frame_x0 = (term_x - frame_w) / 2;
        //     if (frame_y0 < 0) frame_y0 = 0;
        //     if (frame_x0 < 0) frame_x0 = 0;

        //     frame = newwin(frame_h, frame_w, frame_y0, frame_x0);
        //     if (!frame) {
        //         endwin();
        //         fprintf(stderr, "[CLIENT] newwin() failed\n");
        //         goto shutdown_and_exit;
        //     }
        // } else {
        //     // WINDOW *win = (frame ? frame : stdscr);
        //     getmaxyx(stdscr, max_y, max_x);
        // }
        // IWhile in loop, get size from the correct window
        WINDOW *win = (frame ? frame : stdscr);
        getmaxyx(win, max_y, max_x);


        // Plans layout:
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

        // Defines right inspection panel width
        int insp_width = 35;               // was 35
        if (max_x < insp_width + 10) {
            insp_width = max_x / 4;
            if (insp_width < 10) insp_width = 10;
        }
        int insp_start_x = max_x - insp_width;
        if (insp_start_x < 1) insp_start_x = 1;

        // Defines world area below separator.
        int world_top    = sep_y + 1;
        if (world_top > content_bottom) world_top = content_top + 1;
        int world_bottom = content_bottom;
        int world_height = world_bottom - world_top + 1;
        if (world_height < 1) world_height = 1;

        // Defines left world width.
        int main_width = insp_start_x - 2;
        if (main_width < 10) main_width = 10;

        // ---------------- Uses select() to wait for events ----------------        // Uses select() to wait for data from keyboard, dynamics, obstacles, and targets
        // Also handles EINTR (signal generated on resize to permit window resize without exiting the program).
        fd_set rfds;

        /* maxfd is "highest fd + 1" for select() */
        int maxfd = -1;

        if (fd_kb >= 0 && fd_kb > maxfd)       maxfd = fd_kb;
        if (fd_from_d >= 0 && fd_from_d > maxfd) maxfd = fd_from_d;
        if (fd_obs >= 0 && fd_obs > maxfd)     maxfd = fd_obs;
        if (fd_tgt >= 0 && fd_tgt > maxfd)     maxfd = fd_tgt;

        maxfd += 1;   // maxfd is "highest fd + 1" for select()
        int sel;
        while (1) {
            FD_ZERO(&rfds);
            if (fd_kb >= 0)     FD_SET(fd_kb, &rfds);
            if (fd_from_d >= 0) FD_SET(fd_from_d, &rfds);

            // Only add obs/tgt pipes if they exist (standalone mode)
            if (fd_obs >= 0)    FD_SET(fd_obs, &rfds);
            if (fd_tgt >= 0)    FD_SET(fd_tgt, &rfds);

            struct timeval tv;
            tv.tv_sec  = 0;
            // tv.tv_usec = 100000; // 100 ms
            tv.tv_usec = 10000; // 10 ms

            sel = select(maxfd, &rfds, NULL, NULL, &tv);

            if (sel == 0) {     // no pipes ready
                if (wd_warning_active && !paused) {
                    wd_blink_ticks++;

                    // blink every 5 ticks -> 5 * 100ms = 500ms
                    if (wd_blink_ticks >= 5) {
                        wd_blink_ticks = 0;
                        wd_blink_phase = !wd_blink_phase;
                    }
                }
                break;     // break so we still draw the UI this frame
            }

            if (sel == -1) {     // select failed
                if (errno == EINTR) {
                    // Retries if interrupted by signal (like resize)
                    continue;
                } else {
                    fclose(logfile);
                    endwin();
                    die("[B] select failed");
                }
            }
            break; 
        }

        // sel >= 0, we have an event

        // ------------------------------------------------------------------
        // Handles keyboard input from I (if available).
        // ------------------------------------------------------------------
        if (FD_ISSET(fd_kb, &rfds)) {
            KeyMsg km;
            int n = read(fd_kb, &km, sizeof(km));
            if (n <= 0) {
                mvwprintw(win,0, 1, "[B] Keyboard process ended (EOF).");
                refresh();
                break;
            }

            last_key = km.key;

            // Handles Quit request
            if (km.key == 'q') {
                fprintf(logfile, "QUIT requested by 'q'\n");
                fflush(logfile);

                // =================== SERVER QUIT HANDSHAKE ===================
                if (cfg.mode == MODE_SERVER && net_fd >= 0) {
                    fprintf(logfile, "[B] NET: sending quit 'q' to client...\n");
                    fflush(logfile);

                    if (proto_server_send_quit(net_fd) < 0) {
                        fprintf(logfile, "[B] NET: quit handshake failed\n");
                        fflush(logfile);
                    }
                }
                // =============================================================


                break;
            }
            // ------------------------------------------------------------------
            // Handles Pause toggle
            // ------------------------------------------------------------------
            if (km.key == 'p') {
                paused = !paused;

                if (paused) {
                    // Zeroes the force when entering pause.
                    cur_force.Fx = 0.0;
                    cur_force.Fy = 0.0;
                    cur_force.reset = 0;
                    send_total_force_to_d(&cur_force,
                                        &cur_state,
                                        &params,
                                        g_obstacles,
                                        NUM_OBSTACLES,
                                        fd_to_d,
                                        logfile,
                                        "key");
                    fprintf(logfile, "PAUSE: ON\n");
                } else {
                    fprintf(logfile, "PAUSE: OFF\n");
                }
                fflush(logfile);
            }
            // ------------------------------------------------------------------
            // Handles Reset (uppercase O)
            // ------------------------------------------------------------------
            else if (km.key == 'O') {
                // Resets server-side state
                cur_state.x  = 0.0;
                cur_state.y  = 0.0;
                cur_state.vx = 0.0;
                cur_state.vy = 0.0;

                // Resets forces
                cur_force.Fx = 0.0;
                cur_force.Fy = 0.0;
                cur_force.reset = 1; // Signals D to reset its state

                send_total_force_to_d(&cur_force,
                      &cur_state,
                      &params,
                      g_obstacles,
                      NUM_OBSTACLES,
                      fd_to_d,
                      logfile,
                      "key");

                cur_force.reset = 0; // Clears locally
                paused = false;      // Unpauses

                fprintf(logfile, "RESET requested (O)\n");
                fflush(logfile);
            }
            // ------------------------------------------------------------------
            // Handles Directional keys and the break 'd'
            // ------------------------------------------------------------------
            else {
                double dFx, dFy;
                direction_from_key(km.key, &dFx, &dFy);

                if (!paused) {
                    if (km.key == 'd') {
                        // Brake: Zeroes forces
                        cur_force.Fx = 0.0;
                        cur_force.Fy = 0.0;
                    } else {
                        // Accumulates new force
                        cur_force.Fx += dFx * params.force_step;
                        cur_force.Fy += dFy * params.force_step;
                    }

                    cur_force.reset = 0;

                    send_total_force_to_d(&cur_force,
                        &cur_state,
                        &params,
                        g_obstacles,
                        NUM_OBSTACLES,
                        fd_to_d,
                        logfile,
                        "key");

                    fprintf(logfile,
                            "KEY: %c  dFx=%.1f dFy=%.1f -> Fx=%.2f Fy=%.2f\n",
                            km.key, dFx, dFy, cur_force.Fx, cur_force.Fy);
                    fflush(logfile);
                } else {
                    // Paused: Ignores directional changes (but still log)
                    fprintf(logfile,
                            "KEY: %c ignored (PAUSED)\n", km.key);
                    fflush(logfile);
                }
            }
        }

        // ------------------------------------------------------------------
        // 4) Handles state updates from D (if available).
        // ------------------------------------------------------------------
        if (FD_ISSET(fd_from_d, &rfds)) {
            DroneStateMsg s;
            int n = read(fd_from_d, &s, sizeof(s));
            if (n == (int)sizeof(s)) {
                // We received a valid "tick" from dynamics => system is alive
                set_last_hb_now();

                // Send heartbeat to watchdog (as before)
                if (pid_W > 0) kill(pid_W, SIGUSR1);

                // POLISH: if we were blinking due to warning, clear it once activity resumes
                if (wd_warning_active) {
                    wd_warning_active = 0;
                    wd_blink_phase = 0;
                    wd_blink_counter = 0;

                    fprintf(logfile, "[B] Heartbeat resumed -> cleared watchdog warning UI\n");
                    fflush(logfile);
                }
            }
            else if (n <= 0) {
                mvwprintw(win,1, 1, "[B] Dynamics process ended (EOF).");
                refresh();
                break;
            } else {
                // partial read (should not happen with pipes + small struct, but handle anyway)
                fprintf(logfile, "[B] Partial read from D: %d bytes\n", n);
                fflush(logfile);
                continue;
            }

            // Updates current state
            cur_state = s;

            // Increments global step counter (one more state update)
            if (!paused) {
                g_step_counter++;
            }   

            // Logs state
            fprintf(logfile,
                    "STATE: x=%.2f y=%.2f vx=%.2f vy=%.2f\n",
                    s.x, s.y, s.vx, s.vy);
            fflush(logfile);
            // Checks for target hits (only when not paused)
            if (!paused) {
                int hits = check_target_hits(&cur_state,
                                            g_targets,
                                            NUM_TARGETS,
                                            &params,
                                            &g_score,
                                            &g_targets_collected,
                                            &g_last_hit_step,
                                            g_step_counter);
                if (hits > 0) {
                    fprintf(logfile,
                            "[B] Collected %d target(s). SCORE=%d\n",
                            hits, g_score);
                    fflush(logfile);
                }
            }
            // Decrements obstacles and targets lifetimes 
            // Considers each time input is received from D, 1 sim time had elapsed
            // Only age obstacles & targets when simulation is running
            if (!paused){    
                for (int i = 0; i < NUM_OBSTACLES; ++i) {
                    if (g_obstacles[i].active && g_obstacles[i].life_steps > 0) {
                        g_obstacles[i].life_steps--;   // Decreases 1 step from its lifetime
                        if (g_obstacles[i].life_steps == 0) {
                            g_obstacles[i].active = 0;
                        }
                    }
                }
                for (int i = 0; i < NUM_TARGETS; ++i) {
                    if (g_targets[i].active && g_targets[i].life_steps > 0) {
                        g_targets[i].life_steps--;
                        if (g_targets[i].life_steps == 0) {
                            g_targets[i].active = 0;
                        }
                    }
                }
            }
            // Update blinking phase only while running (not paused)
            if (wd_warning_active && !paused) {
                wd_blink_counter++;

                // Blink period in seconds:
                const double BLINK_PERIOD_SEC = 0.5; // 0.5s ON/OFF toggle

                int blink_steps = (int)(BLINK_PERIOD_SEC / params.dt);
                if (blink_steps < 1) blink_steps = 1;

                if (wd_blink_counter >= blink_steps) {
                    wd_blink_counter = 0;
                    wd_blink_phase = !wd_blink_phase; // toggle
                }
            }

            

            // Then, sends updated total force (evenif user doesn't send cmd) (user + obstacles)
            send_total_force_to_d(&cur_force,
                                  &cur_state,
                                  &params,
                                  g_obstacles,
                                  NUM_OBSTACLES,
                                  fd_to_d,
                                  logfile,
                                  "state");
                                  
        }

        // ------------------------------------------------------------------
        // Handles obstacle set messages from O
        // ------------------------------------------------------------------
        if (fd_obs >= 0 && FD_ISSET(fd_obs, &rfds)) {     // so that we never call FD_ISSET on a disabled FD
            ObstacleSetMsg msg;
            int n = read(fd_obs, &msg, sizeof(msg));
            if (n <= 0) {
                // if nth read, O process ended; may log and continue
                mvwprintw(win,0, 1, "[B] Obstacle generator ended.");
                // Optionally: fd_obs = -1 and stop using it
            } else {
                if (paused){
                    // Reads but ignores new obstacles while paused
                    fprintf(logfile,
                            "[B] Received obstacle set but PAUSED -> ignored.\n");
                    fflush(logfile);
                } else {
                    int requested = msg.count;
                    if (requested > NUM_OBSTACLES) requested = NUM_OBSTACLES;

                    // Uses a clearance similar to what we used for targets
                    double tgt_clearance = params.world_half * 0.15;

                    int accepted = 0;

                    for (int i = 0; i < requested; ++i) {
                        double x = msg.obs[i].x;
                        double y = msg.obs[i].y;

                        // Rejects if too close to any active target
                        if (too_close_to_any_pointlike(x, y,
                               (PointLike*)g_obstacles,
                               NUM_OBSTACLES,
                               tgt_clearance)){
                            fprintf(logfile,
                                    "[B] Obstacle (%.2f, %.2f) rejected: too close to target.\n",
                                    x, y);
                            continue;
                        }

                        // Stores it if accepted index is within capacity
                        if (accepted < NUM_OBSTACLES) {
                            g_obstacles[accepted].x          = x;
                            g_obstacles[accepted].y          = y;
                            g_obstacles[accepted].life_steps = msg.obs[i].life_steps;
                            g_obstacles[accepted].active     = 1;
                            accepted++;
                        }
                    }

                    // Deactivates remaining slots
                    for (int i = accepted; i < NUM_OBSTACLES; ++i) {
                        g_obstacles[i].active     = 0;
                        g_obstacles[i].life_steps = 0;
                    }

                    fprintf(logfile,
                            "[B] Accepted %d obstacles (requested %d).\n",
                            accepted, requested);
                    fflush(logfile);
                }

            }
        }

        // ------------------------------------------------------------------
        // Handles target-set messages from T
        // ------------------------------------------------------------------

        if (fd_tgt >= 0 && FD_ISSET(fd_tgt, &rfds)) {
            TargetSetMsg msg;
            int n = read(fd_tgt, &msg, sizeof(msg));
            if (n <= 0) {
                mvwprintw(win,1, 1, "[B] Target generator ended.");
            } else {
                if (paused) {
                    fprintf(logfile,
                            "[B] Received target set but PAUSED -> ignored.\n");
            fflush(logfile);
        } else {
            int requested = msg.count;
            if (requested > NUM_TARGETS) requested = NUM_TARGETS;

            // Tuning for filtering:
            double wall_margin     = params.world_half * 0.20; // keep away from walls
            double obs_clearance   = params.world_half * 0.15; // away from obstacles

            int accepted = 0;

            for (int i = 0; i < requested; ++i) {
                double x = msg.tgt[i].x;
                double y = msg.tgt[i].y;

                // Rejects if too close to walls
                if (target_too_close_to_wall(x, y, &params, wall_margin)) {
                    fprintf(logfile,
                            "[B] Target (%.2f,%.2f) rejected: too close to walls.\n",
                            x, y);
                    continue;
                }

                // Rejects if too close to obstacles
                if (too_close_to_any_pointlike(x, y,
                               (PointLike*)g_targets,
                               NUM_TARGETS,
                               obs_clearance)){
                    fprintf(logfile,
                            "[B] Target (%.2f,%.2f) rejected: too close to obstacles.\n",
                            x, y);
                    continue;
                }

                // Accepts target if it passed the above checks
                if (accepted < NUM_TARGETS) {
                    g_targets[accepted].x          = x;
                    g_targets[accepted].y          = y;
                    g_targets[accepted].life_steps = msg.tgt[i].life_steps;
                    g_targets[accepted].active     = 1;
                    accepted++;
                }
            }

            // Deactivates remaining slots
            for (int i = accepted; i < NUM_TARGETS; ++i) {
                g_targets[i].active     = 0;
                g_targets[i].life_steps = 0;
            }

            fprintf(logfile,
                    "[B] Accepted %d targets (requested %d).\n",
                    accepted, requested);
            fflush(logfile);
        }
    }

}
        // =================== NETWORK STEP (NON-BLOCKING) ===================
        if (cfg.mode == MODE_SERVER && net_fd >= 0 && !paused) {

            int have = 0;
            double ox = 0.0, oy = 0.0;

            // trying a few micro-steps per frame:keeps it responsive even if net is behind
            for (int k = 0; k < 4; k++) {
                int rc = server_proto_tick(net_fd, &params, cur_state.x, cur_state.y, &have, &ox, &oy);
                if (rc == 1) break;        // would block -> stop trying this frame
                if (rc < 0) {              // hard error -> terminate networking
                    fprintf(logfile, "[B] NET: server_proto_tick hard error -> disconnect\n");
                    fflush(logfile);
                    goto shutdown_and_exit; // or set a quit flag and break outer loop
                }
            }

            if (have) {
                // Inject client drone as a single obstacle (repulsion uses your existing obstacle loop)
                g_obstacles[REMOTE_OBST_IDX].active = 1;
                g_obstacles[REMOTE_OBST_IDX].x = ox;
                g_obstacles[REMOTE_OBST_IDX].y = oy;
            }
        }
        // ================================================================================

        // Client mode net step can also live here (same idea: read tag, act)
        // =================== CLIENT NETWORK STEP (NON-BLOCKING) ===================
        if (cfg.mode == MODE_CLIENT && net_fd >= 0 && !paused) {

            int have_remote = 0;
            int server_quit = 0;
            double sx = 0.0, sy = 0.0;

            // advance protocol state machine a few micro-steps per frame
            for (int k = 0; k < 4; k++) {
                int rc = client_proto_tick(net_fd,
                                            &params,
                                            cur_state.x, cur_state.y,
                                            &have_remote,
                                            &sx, &sy,
                                            &server_quit);

                if (rc == 1) break;          // would block -> stop this frame
                if (rc < 0) {
                    fprintf(logfile, "[B] NET: client_proto_tick hard error -> disconnect\n");
                    fflush(logfile);
                    goto shutdown_and_exit;  // or set quit flag
                }
            }

            if (server_quit) {
                fprintf(logfile, "[B] NET: server requested quit\n");
                fflush(logfile);
                net_send_line(net_fd, "qok");
                goto shutdown_and_exit;
            }

            if (have_remote) {
                // store server drone position for rendering
                remote_drone_x = sx;
                remote_drone_y = sy;
                remote_drone_valid = 1;
            }
        }
        // =======================================================================================



        // ------------------------------------------------------------------
        // Draws UI (drone world + inspection panel)
        // ------------------------------------------------------------------
        
        // based on forced_max_x and forced_max_y (retirieved from server for client mode operation)
        
        // if (frame) {    // flickered when i did this (visible on/off)
        //     werase(stdscr);
        //     wrefresh(stdscr);
        // }
        werase(win);
        box(win, 0, 0);


        // Top info lines
        mvwprintw(win, top_info_y1, 2,
                 "Controls: w e r / s d f / x c v | d=brake, p=pause, O=reset, q=quit");
        mvwprintw(win, top_info_y2, 2,
                 "Paused: %s", paused ? "YES" : "NO");
        
        // Watchdog blinking warning: visible only when active AND blink phase is ON
        // --- Watchdog live timing info ---
        double age = hb_age_sec();  // seconds since last valid DroneStateMsg
        double warn_in = (double)params.wd_warn_sec - age;
        double kill_in = (double)params.wd_kill_sec - age;

        if (warn_in < 0) warn_in = 0;
        if (kill_in < 0) kill_in = 0;

        if (wd_warning_active && wd_blink_phase) {
            // If colors exist, use a red-ish pair. Otherwise use reverse + bold.
            if (has_colors()) {
                // Create once somewhere during init if you want:
                // init_pair(3, COLOR_RED, COLOR_BLACK);
                wattron(win, COLOR_PAIR(3) | A_BOLD | A_REVERSE);
                mvwprintw(win,top_info_y2, 18, " %s ", watchdog_banner_msg);
                mvwprintw(win,top_info_y2, 60, "KILL IN: %.2fs", kill_in);
                wattroff(win, COLOR_PAIR(3) | A_BOLD | A_REVERSE);
            } else {
                wattron(win, A_BOLD | A_REVERSE);
                mvwprintw(win,top_info_y2, 18, " %s ", watchdog_banner_msg);
                mvwprintw(win,top_info_y2, 60, "KILL IN: %.2fs", kill_in);
                wattroff(win, A_BOLD | A_REVERSE);
            }
        }

        // Horizontal separator row (under top info)
        if (sep_y >= 1 && sep_y <= max_y - 2) {
            for (int x = 1; x < max_x - 1; ++x) {
                mvwaddch(win,sep_y, x, '-');
            }
        }

        // Vertical separator between world and inspection
        int sep_x = insp_start_x - 1;
        if (sep_x > 1 && sep_x < max_x - 1) {
            for (int y = world_top; y <= world_bottom; ++y) {
                mvwaddch(win,y, sep_x, '|');
            }
        }

        // WORLD DRAWING (left)
        double world_half = params.world_half;
        double scale_x = main_width  / (2.0 * world_half);        // Maps world coordinates to the drone world on the display scree
        double scale_y = world_height / (2.0 * world_half);
        if (scale_x <= 0) scale_x = 1.0;
        if (scale_y <= 0) scale_y = 1.0;

        int sx = (int)(cur_state.x * scale_x) + main_width / 2 + 1;
        int sy = (int)(-cur_state.y * scale_y) + world_top + world_height / 2;

        if (sx < 1) sx = 1;
        if (sx > main_width) sx = main_width;
        if (sy < world_top) sy = world_top;
        if (sy > world_bottom) sy = world_bottom;

        mvwaddch(win,sy, sx, '+'); // Draws drone

        // Draw remote (server) drone on client as 'X' using same world->screen mapping
        if (cfg.mode == MODE_CLIENT && remote_drone_valid) {

            int sx = (int)(remote_drone_x * scale_x) + main_width / 2 + 1;
            int sy = (int)(-remote_drone_y * scale_y) + world_top + world_height / 2;

            // clamp like you do for obstacles
            if (sx < 1) sx = 1;
            if (sx > main_width) sx = main_width;
            if (sy < world_top) sy = world_top;
            if (sy > world_bottom) sy = world_bottom;

            // color it (use defined pair)
            // attron(COLOR_PAIR(2));q
            mvwaddch(win,sy, sx, 'x');
            // attroff(COLOR_PAIR(2));
        }



        // Draws active obstacles as 'o' in the drone world
        for (int k = 0; k < NUM_OBSTACLES; ++k) {
            if (!g_obstacles[k].active) continue;  // Skips inactive 

            int ox = (int)(g_obstacles[k].x * scale_x) + main_width / 2 + 1;
            int oy = (int)(-g_obstacles[k].y * scale_y) + world_top + world_height / 2;

            if (ox < 1) ox = 1;
            if (ox > main_width) ox = main_width;
            if (oy < world_top) oy = world_top;
            if (oy > world_bottom) oy = world_bottom;
            
            wattron(win, COLOR_PAIR(1));
            mvwaddch(win,oy, ox, 'o');  // TODO: Adds color to make them orange
            
            wattroff(win, COLOR_PAIR(1));
        }

        for (int k = 0; k < NUM_TARGETS; ++k) {
            if (!g_targets[k].active) continue;

            int tx = (int)(g_targets[k].x * scale_x) + main_width / 2 + 1;
            int ty = (int)(-g_targets[k].y * scale_y) + world_top + world_height / 2;

            if (tx < 1) tx = 1;
            if (tx > main_width) tx = main_width;
            if (ty < world_top) ty = world_top;
            if (ty > world_bottom) ty = world_bottom;
            
            wattron(win, COLOR_PAIR(2));
            mvwaddch(win,ty, tx, 'T');  // Placeholder, later make them numbered
            wattroff(win, COLOR_PAIR(2));
        }


        // INSPECTION panel on the right
        int info_y = world_top;
        int info_x = insp_start_x + 1;

        
        if (info_x < max_x - 1) {
            mvwprintw(win, info_y,     info_x, "INSPECTION");
            mvwprintw(win,info_y + 2, info_x, "Last key: %c", last_key);
            mvwprintw(win,info_y + 4, info_x, "Fx = %.2f", cur_force.Fx);
            mvwprintw(win,info_y + 5, info_x, "Fy = %.2f", cur_force.Fy);
            mvwprintw(win,info_y + 7, info_x, "x  = %.2f", cur_state.x);
            mvwprintw(win,info_y + 8, info_x, "y  = %.2f", cur_state.y);
            mvwprintw(win,info_y + 9, info_x, "vx = %.2f", cur_state.vx);
            mvwprintw(win,info_y +10, info_x, "vy = %.2f", cur_state.vy);
            
            mvwprintw(win,info_y +12, info_x, "Score: %d", g_score);
            mvwprintw(win,info_y +13, info_x, "Targets collected: %d", g_targets_collected);
            if (g_last_hit_step >= 0 ) {
                time_since_last_hit = (g_step_counter - g_last_hit_step) * params .dt;

                mvwprintw(win,info_y +15, info_x, "Since last hit: %.2f sec", time_since_last_hit);
            }
            else {
                mvwprintw(win,info_y +14, info_x, "Last hit: none");
            }

        }

        wrefresh(win);
    }
 

    shutdown_and_exit:

        // Final cleanup
        if (net_fd >= 0) close(net_fd);
        if (listen_fd >= 0) close(listen_fd);    // also closed earlier after accept()
        
        if (logfile) {
            fprintf(logfile, "[B] Exiting.\n");
            fclose(logfile);
        }
        // Ends ncurses
        if (frame) delwin(frame);
        endwin();
        // Closes pipes
        close(fd_kb);
        close(fd_to_d);
        close(fd_from_d);
        exit(EXIT_SUCCESS);

}

