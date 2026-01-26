#define _POSIX_C_SOURCE 200809L
#include "headers/net.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdarg.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// ---------------------------- sockets ----------------------------

// Implement networking utilities

int net_server_listen(int port) {
    // 1. Create a socket file descriptor (IPv4, TCP stream)
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    // If creation fails, return error code -1
    if (fd < 0) return -1;

    // 2. Set socket option SO_REUSEADDR
    // 'yes' is a flag to enable the option
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    // This allows the server to bind to the same port immediately after it restarts,
    // avoiding the "Address already in use" error if the old socket is still in TIME_WAIT state

    // 3. Configure the server address structure
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));           // Zero out the structure to prevent garbage values
    addr.sin_family      = AF_INET;           // Use IPv4
    addr.sin_addr.s_addr = htonl(INADDR_ANY); // Bind to ALL available network interfaces (any IP)
    addr.sin_port        = htons((uint16_t)port); // Set the port number (converted to Network Byte Order)
    // WHY: htonl/htons ensure the numbers are Big Endian, as required by network protocols

    // 4. Bind the socket to the address/port configuration
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(fd); // If bind fails (e.g., port busy), close socket to avoid leaks
        return -1;
    }

    // 5. Start listening for incoming connections
    // The second argument '1' is the backlog (queue size for pending connections)
    if (listen(fd, 1) < 0) {
        close(fd);
        return -1;
    }

    return fd; // Return the successfully listening file descriptor
}

int net_server_accept(int listen_fd) {
    for (;;) { // Infinite loop to retry if interrupted
        // Blocks until a client connects. NULLs mean we don't care about the client's address right now
        int cfd = accept(listen_fd, NULL, NULL);
        if (cfd >= 0) return cfd; // Success! Return new connection SD

        // If the blocking call was interrupted by a signal (like a timer or OS signal), retry
        if (errno == EINTR) continue;
        return -1; // Specific fatal error occurred
    }
}

int net_client_connect(const char *server_ip, int port) {
    // 1. Create a socket (IPv4, TCP)
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        fprintf(stderr, "[CLIENT-case-1] net_client_connect failed\n"); 
        return -1;}

    // 2. Configure the target server address
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons((uint16_t)port);

    // 3. Convert string IP (e.g., "127.0.0.1") to binary format
    if (inet_pton(AF_INET, server_ip, &addr.sin_addr) != 1) {
        close(fd);
        errno = EINVAL; // Invalid argument (bad IP string)
        fprintf(stderr, "[CLIENT-case-2] net_client_connect failed\n"); 
        return -1;
    }

    // 4. Attempt to connect
    for (;;) {
        if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) == 0) return fd; // Success
        if (errno == EINTR) continue; // Retry if interrupted by signal
        
        // Connection failed definitively
        int saved_errno = errno;
        close(fd); 
        errno = saved_errno; // restore errno for the caller or printing

        fprintf(stderr, "[CLIENT-case-3] net_client_connect failed: %s (errno=%d)\n", strerror(errno), errno); 
        return -1;
    }
}

// ---------------------------- line protocol ----------------------------

// write all bytes
// Purpose: A robust wrapper for write() that ensures ALL bytes are sent, even if the OS splits the transmission
static int write_all(int fd, const char *buf, size_t n) {
    size_t off = 0; // Current offset (how many bytes written so far)
    while (off < n) {
        // Try to write the remaining bytes (n - off)
        ssize_t w = write(fd, buf + off, n - off);
        if (w > 0) { 
            off += (size_t)w; // We wrote 'w' bytes, advance offset
            continue; 
        }
        // If interrupted by signal (common in non-blocking or busy systems), just retry
        if (w < 0 && errno == EINTR) continue;
        return -1; // Fatal error
    }
    return 0; // All bytes written successfully
}

int net_send_line(int fd, const char *line) {
    // sends: line + '\n'
    // Purpose: Appends a newline to a string and sends it using write_all
    char tmp[1024]; // Temporary buffer
    size_t len = strlen(line);
    
    // Check if the message + newline fits in the buffer
    if (len + 1 >= sizeof(tmp)) { errno = EMSGSIZE; return -1; }

    memcpy(tmp, line, len); // Copy message
    tmp[len] = '\n';        // Append newline (protocol requirement)
    tmp[len+1] = '\0';      // Null terminate (good practice, though write uses length)

    // Send the exact number of bytes (len + 1 for the newline)
    return write_all(fd, tmp, len + 1);
}

int net_sendf(int fd, const char *fmt, ...) {
    // Purpose: printf-style wrapper for sending formatted strings over the network
    char buf[512]; // Fixed size buffer for the formatted string
    va_list ap;    // Variable argument list handler
    va_start(ap, fmt); // Initialize list with the format string
    
    // Format the string into 'buf' (safe version ensures no overflow of 512 bytes)
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap); // Clean up
    
    // Send it using the standard line sender
    return net_send_line(fd, buf);
}

// reads until '\n' (newline is removed). returns 0 on success, -1 on error/EOF
// Purpose: Reads from the socket byte-by-byte until a newline '\n' is found
// int net_recv_line(int fd, char *buf, size_t maxlen) {
//     if (!buf || maxlen < 2) { errno = EINVAL; return -1; } // Basic validity checks

//     size_t used = 0; // How many bytes we've read into 'buf'
//     while (1) {
//         char c;
//         ssize_t r = read(fd, &c, 1); // READ 1 BYTE at a time
//         // WHY: Reading 1 byte is inefficient but guarantees we don't accidentally 
//         // read into the *next* message waiting in the socket buffer

//         if (r == 1) { // We got a byte
//             if (c == '\n') {
//                 buf[used] = '\0'; // Replace newline with null-terminator for C string compatibility
//                 return 0; // Success!
//             }
//             if (used + 1 >= maxlen) {
//                 // Buffer is full and we haven't found a newline yet
//                 errno = EMSGSIZE;
//                 return -1;  
//             }
//             buf[used++] = c; // Store character and increment index
//             continue;
//         }

//         if (r == 0) {
//             // EOF: The other side closed the connection
//             errno = ECONNRESET;
//             return -1;
//         }

//         // r < 0
//         if (errno == EINTR) continue; // Retry if interrupted
//         return -1; // Fatal error.
//     }
// }


#include <sys/time.h>   // add at top of net.c (for timeout helper)
#include <stdlib.h>

// --- helper: set recv timeout in ms ---
int net_set_rcv_timeout_ms(int fd, int ms) {
    struct timeval tv;
    tv.tv_sec  = ms / 1000;
    tv.tv_usec = (ms % 1000) * 1000;
    return setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}
//****************************************************************/
// ---------------- buffered line receive ----------------
// This preserves partial data across calls (critical for timeouts)
int net_recv_line(int fd, char *out, size_t maxlen) {
    if (!out || maxlen < 2) { errno = EINVAL; return -1; }

    // One slot (one net fd at a time)
    static int    s_fd = -1;
    static char   buf[4096];
    static size_t len = 0;

    if (s_fd != fd) {
        s_fd = fd;
        len = 0;
    }

    for (;;) {
        // 1) Do we already have a full line in the buffer?
        for (size_t i = 0; i < len; i++) {
            if (buf[i] == '\n') {
                size_t line_len = i; // excluding '\n'
                if (line_len + 1 > maxlen) { errno = EMSGSIZE; return -1; }

                memcpy(out, buf, line_len);
                out[line_len] = '\0';

                // Remove consumed line + '\n' from buffer
                size_t remaining = len - (i + 1);
                memmove(buf, buf + i + 1, remaining);
                len = remaining;

                return 0;
            }
        }

        // 2) Need more data. Read into buffer tail.
        if (len >= sizeof(buf)) {
            errno = EMSGSIZE; // line too long
            return -1;
        }

        ssize_t r = recv(fd, buf + len, sizeof(buf) - len, 0);
        if (r > 0) {
            len += (size_t)r;
            continue;
        }

        if (r == 0) {
            errno = ECONNRESET; // peer closed
            return -1;
        }

        // r < 0
        if (errno == EINTR) continue;

        // Timeout / no data available (with SO_RCVTIMEO set)
        // return -1 but KEEP partial bytes in 'buf'
        if (errno == EAGAIN || errno == EWOULDBLOCK) return -1;

        return -1;
    }
}

