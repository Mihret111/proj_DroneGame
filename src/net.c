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
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons((uint16_t)port);

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }

    if (listen(fd, 1) < 0) {
        close(fd);
        return -1;
    }

    return fd;
}

int net_server_accept(int listen_fd) {
    for (;;) {
        int cfd = accept(listen_fd, NULL, NULL);
        if (cfd >= 0) return cfd;
        if (errno == EINTR) continue;
        return -1;
    }
}

int net_client_connect(const char *server_ip, int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons((uint16_t)port);

    if (inet_pton(AF_INET, server_ip, &addr.sin_addr) != 1) {
        close(fd);
        errno = EINVAL;
        return -1;
    }

    for (;;) {
        if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) == 0) return fd;
        if (errno == EINTR) continue;
        close(fd);
        return -1;
    }
}

// ---------------------------- line protocol ----------------------------

// write all bytes
static int write_all(int fd, const char *buf, size_t n) {
    size_t off = 0;
    while (off < n) {
        ssize_t w = write(fd, buf + off, n - off);
        if (w > 0) { off += (size_t)w; continue; }
        if (w < 0 && errno == EINTR) continue;
        return -1;
    }
    return 0;
}

int net_send_line(int fd, const char *line) {
    // sends: line + '\n'
    char tmp[1024];
    size_t len = strlen(line);
    if (len + 1 >= sizeof(tmp)) { errno = EMSGSIZE; return -1; }

    memcpy(tmp, line, len);
    tmp[len] = '\n';
    tmp[len+1] = '\0';

    return write_all(fd, tmp, len + 1);
}

int net_sendf(int fd, const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return net_send_line(fd, buf);
}

// reads until '\n' (newline is removed). returns 0 on success, -1 on error/EOF
int net_recv_line(int fd, char *buf, size_t maxlen) {
    if (!buf || maxlen < 2) { errno = EINVAL; return -1; }

    size_t used = 0;
    while (1) {
        char c;
        ssize_t r = read(fd, &c, 1);

        if (r == 1) {
            if (c == '\n') {
                buf[used] = '\0';
                return 0;
            }
            if (used + 1 >= maxlen) {
                // line too long
                errno = EMSGSIZE;
                return -1;
            }
            buf[used++] = c;
            continue;
        }

        if (r == 0) {
            // peer closed
            errno = ECONNRESET;
            return -1;
        }

        // r < 0
        if (errno == EINTR) continue;
        return -1;
    }
}
