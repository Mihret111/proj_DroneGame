#include "headers/protocol.h"
#include "headers/net.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// network helpers:
int net_send_line(int fd, const char *line);
int net_sendf(int fd, const char *fmt, ...);
int net_recv_line(int fd, char *buf, size_t maxlen);

static int streq(const char *a, const char *b) {
    return strcmp(a, b) == 0;
}

int proto_expect(int fd, const char *expected) {
    char line[256];
    if (net_recv_line(fd, line, sizeof(line)) < 0) return -1;
    if (!streq(line, expected)) return -2;
    return 0;
}

int proto_send_and_expect(int fd, const char *msg, const char *expected_ack) {
    if (net_send_line(fd, msg) < 0) return -1;
    return proto_expect(fd, expected_ack);
}

int proto_recv_tag(int fd, char *buf, size_t buflen) {
    if (!buf || buflen == 0) return -3;
    if (net_recv_line(fd, buf, buflen) < 0) return -1;
    return 0;
}

int proto_send_xy(int fd, double x, double y) {
    // Strings only
    // %.6f would be enough for terminal coordinates
    return net_sendf(fd, "%.6f %.6f", x, y) < 0 ? -1 : 0;
}

int proto_recv_xy(int fd, double *x, double *y) {
    char line[256];
    if (!x || !y) return -3;
    if (net_recv_line(fd, line, sizeof(line)) < 0) return -1;
    if (sscanf(line, "%lf %lf", x, y) != 2) return -2;
    return 0;
}

// ----------------- protocol steps ----------------

int proto_server_send_drone(int fd, double x, double y) {
    if (net_send_line(fd, "drone") < 0) return -1;
    if (proto_send_xy(fd, x, y) < 0) return -1;
    return proto_expect(fd, "dok"); // client ack
}

int proto_client_recv_drone_and_ack(int fd, double *x, double *y) {
    if (proto_recv_xy(fd, x, y) < 0) return -1;
    if (net_send_line(fd, "dok") < 0) return -1;
    return 0;
}

int proto_server_request_obstacle(int fd, double *ox, double *oy) {
    if (net_send_line(fd, "obst") < 0) return -1;
    if (proto_recv_xy(fd, ox, oy) < 0) return -1;   // client sends x y
    if (net_send_line(fd, "pok") < 0) return -1;     // server ack
    return 0;
}

int proto_client_send_obstacle_and_wait_ack(int fd, double x, double y) {
    if (proto_send_xy(fd, x, y) < 0) return -1;
    return proto_expect(fd, "pok");
}

int proto_server_send_quit(int fd) {
    if (net_send_line(fd, "q") < 0) return -1;
    return proto_expect(fd, "qok");
}

int proto_client_ack_quit(int fd) {
    return net_send_line(fd, "qok") < 0 ? -1 : 0;
}
