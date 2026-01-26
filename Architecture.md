# Architectural Documentation

This project implements a simple 2D drone simulator using multiple POSIX processes and IPC primitives. The Master process initializes the simulation, creates communication pipes, and forks five child processes: **Keyboard (I)**, **Dynamics (D)**, **Obstacles (O)**, **Targets (T)**, and **Watchdog (W)**. After forking, the Master process transitions into the **Server (B)** process. The server aggregates user input, environment information and simulation state, computes total forces (including wall and obstacle repulsion), and updates a User-Interface using `ncurses`.

The application now supports ** Networked Modes**, allowing two instances to connect over a TCP network.

# 1- Architecture Sketch

```mermaid
graph TD
    subgraph "Process Architecture (Standalone Mode)"
    I["Keyboard (I)"] -->|"KeyMsg"| B[" Blackboard Server (B)"]
    B -->|"ForceStateMsg"| D["Dynamics (D)"]
    D -->|"DroneStateMsg"| B
    O["Obstacles (O)"] -->|"ObstacleSetMsg"| B
    T["Targets (T)"] -->|"TargetSetMsg"| B
    B -.->|"SIGUSR1 (Heartbeat)"| W["Watchdog (W)"]
    W -.->|"SIGUSR2 (Warn)"| B
    W ==>|"SIGTERM (Kill)"| EXIT{"System Shutdown<br/>(B, I, D, O, T)"}
    end
```

```mermaid
graph TD
    subgraph "Networked Mode Architecture"
    
        subgraph "Instance 1 (Server)"
            I1["Keyboard (I)"] -->|"KeyMsg"| B1["Blackboard Server (B)"]
            B1 -->|"ForceStateMsg"| D1["Dynamics (D)"]
            D1 -->|"DroneStateMsg"| B1
        end

        subgraph "Instance 2 (Client)"
            I2["Keyboard (I)"] -->|"KeyMsg"| B2["Blackboard Server (B)"]
            B2 -->|"ForceStateMsg"| D2["Dynamics (D)"]
            D2 -->|"DroneStateMsg"| B2
        end

        B1 <==>|"TCP/IP Protocol<br/>(Drone Pos <-> Obstacle Pos)"| B2
    end
```

> **Note on Network Mode**: In **Server** and **Client** modes, the **Obstacles (O)**, **Targets (T)**, and **Watchdog (W)** processes are **disabled**. The system relies on the network peer for obstacle data.
# 2. Active Components — Definitions, IPC, and Algorithms

## 2.1 Keyboard Process (I)
- Role: Reads keystrokes from the user and forwards them to the Server.
- IPC: Sends `KeyMsg → B` (pipe)
- Behaviour:
    - Blocking read using `getchar()`
    - Sends every keystroke immediately
    - Supports directional cluster:
                w   e   r
                s   d   f
                x   c   v
    - Special keys:
    - `d` → brake (zero force)
    - `p` → toggle pause
    - `R` → reset drone
    - `q` → quit all processes

## 2.2 Modes of Operation (Assignment 3)

The system prompts for a mode at startup:

1.  **Standalone**: Standard Assignment 2 behavior. All processes (I, D, B, O, T, W) are active.
2.  **Server**:
    *   Acts as the game host.
    *   Listens on a TCP Port.
    *   Disables Obstacles (O), Targets (T), and Watchdog (W).
    *   Receives the Client's drone position and treats it as a simplified obstacle.
    *   Send its drone position to the Client.
3.  **Client**:
    *   Connects to the Server's IP/Port.
    *   Disables O, T, W.
    *   Receives window dimensions from Server.
    *   Send its drone position to the Server.
    *   Receives the Server's drone position and displays it not as a target but as an entity to avoid.

## 2.2 Server / Blackboard Process (B)
- Role: Main coordinator. Manages all IPC, world state, UI, scoring, environment logic.
- IPC:
    - Reads `KeyMsg` from I  
    - Reads `DroneStateMsg` from D  
    - Reads `ObstacleSetMsg` from O  
    - Reads `TargetSetMsg` from T  
    - Writes `ForceStateMsg` to D  
    - Uses `select()` to wait on multiple pipes
- Algorithms / Responsibilities:
    - User Force Handling
        - Updates accumulated user force from key cluster
        - Brake (`d`) resets force to zero
        - Pause freezes the simulation
    - Virtual-Key Obstacle Repulsion  
        - Compute continuous obstacle repulsive vector
        - Project onto 8 key directions
        - Select maximum positive projection
        - Convert magnitude to virtual key impulses
        - Update force accordingly
    - Target & Obstacle Filtering
        B ensures valid spawning:
        **Targets rejected if:**
        - too close to walls
        - too close to active obstacles

        **Obstacles rejected if:**
        - too close to active targets
        - new batch arrives while old ones still active
    - Target Hit Detection / Scoring
        If drone gets within `R_hit` of a target:
        - target deactivates  
        - score increments  
        - last-hit time updated  
    - World Rendering (ncurses)
        - Left pane → world (drone, walls, obstacles, targets)
        - Right pane → telemetry + score
        - Top row → instructions
        - UI updates every cycle
    - Pause / Reset / Quit
        - Pause freezes: obstacles, targets, forces, physics
        - Reset: set drone to origin with zero velocity
        - Quit: clean shutdown of all processes

## 2.3 Dynamics Process (D)
- Role: Simulates drone physics in real time.
- IPC:
    - Reads `ForceStateMsg` from B  
    - Writes `DroneStateMsg` to B  
- Algorithms: Applies 2D dynamics:
    - Adds continuous Khatib wall-repulsion  
    - Handles reset command  
    - Uses `nanosleep(dt)` for real-time pacing

## 2.4 Obstacle Generator Process (O)
- Role: Periodically generates dynamic obstacles.
- IPC: Sends `ObstacleSetMsg → B`
- Algorithms:
    - Samples random positions in an inner safe box  
    - Enforces minimum spacing  
    - Assigns lifetime (`life_steps`)  
    - New waves only accepted when none active  

## 2.5 Target Generator Process (T)
- Role: Generates collectible targets.
- IPC:Sends `TargetSetMsg → B`
- Algorithms:
    - Samples target positions in a central disk  
    - Applies spacing constraints  
    - B further filters targets:
        - too close to walls → reject
        - too close to obstacles → reject  

## 2.6 Parameter Module (`params.c`)
- Loads simulation parameters from `params.txt`:
    - mass, visc, dt  
    - force_step  
    - world_half  
    - spawn timings & clearances  

## 2.7 Utility Module (`util.c`)
- Shared helpers:
    - `check_target_hits()` for scoring  
    - distance filtering functions  
    - random sampling helpers  
    - direction-vector utilities for virtual keys 
    - generic logging handlers for processes


## 2.8 Watchdog Process (W)
- **Role**: System Health Monitor. Ensures the simulation is running responsively.
- **Design ("Chain of Trust")**: 
    - The Server (B) sends a heartbeat `SIGUSR1` to Watchdog (W) **only** after receiving a valid state update from Dynamics (D).
    - This effectively monitors the **Physics Loop**: if D freezes, B stops receiving updates, stops sending heartbeats, and W triggers a reset/warning.
- **IPC**:
    - **Input**: `SIGUSR1` from Server (B) (Heartbeat)
    - **Output**: 
        - `SIGUSR2` to B (Warning)
        - `SIGTERM` to All Processes (System Kill)
- **Algorithms**:
    - Monitors time since last heartbeat.
    - If silence > 2s: Warns B (triggers **blinking UI banner** with a **countdown timer**). The warning is cleared if the system resumes.
    - If silence > 10s: Terminates the entire system.
    - The timeout values are configurable in `params.txt`.

## 2.9 Network Protocol

When operating in **Server** or **Client** mode, the application uses a custom protocol.

### Handshake
1.  **Connection**: Server listens, Client connects.
2.  **Verification**: Server sends `ok`, Client responds `ook`.
3.  **Window Synchronization**:
    *   Server sends `size <Width> <Height>`
    *   Client resizes/verifies and responds `sok` (or `sok ...`).

### Game Loop Protocol
The Server and Client exchange state in a lock-step fashion every simulation cycle:

1.  **Drone State Exchange**:
    *   Sender sends `drone`.
    *   Sender sends `X Y` (Virtual Coordinates).
    *   Receiver responds `dok`.
2.  **Obstacle State Exchange** (The "other" drone is seen as an obstacle):
    *   Sender sends `obst`.
    *   Sender sends `X Y` (Virtual Coordinates).
    *   Receiver responds `pok`.

*Note: Coordinates are normalized to a "Virtual System" to handle different window sizes or offsets.*

## 3 File Organization

### 3.1 File Structure

```text
proj_DroneGame/
│
├── src/          <-- Source files (.c)
│   ├── main.c           # Entry point
│   ├── server.c         # Blackboard server
│   ├── dynamics.c       # Physics simulation
│   ├── keyboard.c       # Input handling
│   ├── obstacles.c      # Obstacle generation
│   ├── targets.c        # Target generation
│   ├── watchdog.c       # System monitor
│   ├── params.c         # Config loader
│   ├── net.c            # Tcp/Ip networking wrapper
│   ├── protocol.c       # Game protocol handling
│   └── util.c           # Utilities
│
├── headers/      <-- Header files (.h)
│   ├── server.h
│   ├── dynamics.h
│   ├── keyboard.h
│   ├── obstacles.h
│   ├── targets.h
│   ├── watchdog.h
│   ├── params.h
│   ├── net.h
│   ├── protocol.h
│   ├── runmode.h
│   ├── util.h
│   └── messages.h
│
├── build/        <-- Compiled object files (.o)
│
├── logs/         <-- Runtime logs
│
├── install/      <-- Installation scripts
│
├── Makefile
├── README.md
└── Architecture.md
```


### 3.2 Source Files
-   `main.c`: Entry point. Handles parameter loading, pipe creation, and process forking.
-   `server.c`: Implementation of the Server (B) process logic and UI.
-   `dynamics.c`: Implementation of the Dynamics (D) process physics loop.
-   `keyboard.c`: Implementation of the Keyboard (I) process.
-   `obstacles.c`: Implementation of the Obstacles (O) generator.
-   `targets.c`: Implementation of the Targets (T) generator.
-   `watchdog.c`: Implementation of the Watchdog (W) process.
-   `params.c`: Helper functions for loading and initializing simulation parameters.
-   `net.c`: socket wrappers and timeout utilities.
-   `protocol.c`: Visualization/Serialization of connection handshake and game state.
-   `util.c`: Shared utility functions (math, logging, helpers).

### 3.3 Headers (`./headers/`)
*   `server.h`: Server definitions.
*   `dynamics.h`: Dynamics definitions.
*   `keyboard.h`: Keyboard definitions.
*   `obstacles.h`: Obstacles definitions.
*   `targets.h`: Targets definitions.
*   `watchdog.h`: Watchdog definitions.
*   `params.h`: Parameter definitions.
*   `net.h`: Networking primitives.
*   `protocol.h`: Protocol handlers.
*   `runmode.h`: Run configuration enumerations.
*   `util.h`: Utility definitions.
*   `messages.h`: IPC message structures.

### 3.4 Configuration
-   `params.txt`: Runtime configuration of drone parameters (can be modified in real-time).
-   `logs/`: Directory housing runtime logs for each process (e.g., `server.log`, `dynamics.log`, `watchdog.log`).

#### 3.5 Build & Documentation
*   `Makefile`: Build configuration.
*   `README.md`: Project overview.
*   `Architecture.md`: System architecture documentation.
