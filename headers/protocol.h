#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>

// Read one line and require exact match
int proto_expect(int fd, const char *expected);

// Send one line (string) and then expect ack
int proto_send_and_expect(int fd, const char *msg, const char *expected_ack);

// Receive a tag line into buf (e.g., "drone", "obst", "q")
int proto_recv_tag(int fd, char *buf, size_t buflen);

// Send "x y" as doubles
int proto_send_xy(int fd, double x, double y);

// Receive "x y" as doubles
int proto_recv_xy(int fd, double *x, double *y);

// ---- protocol steps ----

// Server: send drone tag + (x,y), wait dok
int proto_server_send_drone(int fd, double x, double y);

// Client: receive drone (x,y) after tag already received, send dok
int proto_client_recv_drone_and_ack(int fd, double *x, double *y);

// Server: send obst tag, receive (x,y) from client, send pok
int proto_server_request_obstacle(int fd, double *ox, double *oy);

// Client: upon receiving obst tag, send (x,y), wait pok
int proto_client_send_obstacle_and_wait_ack(int fd, double x, double y);

// Server: send quit, wait qok
int proto_server_send_quit(int fd);

// Client: upon receiving quit tag, send qok
int proto_client_ack_quit(int fd);

#endif
