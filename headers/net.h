#ifndef NET_H
#define NET_H

#include <stddef.h>

// Define networking utilities
// Server: create listening socket and accept 1 client
int net_server_listen(int port);
int net_server_accept(int listen_fd);

// Client: connect to server
int net_client_connect(const char *server_ip, int port);

// Reliable line protocol 
// to send/receive newline-terminated strings over TCP
int net_send_line(int fd, const char *line);
int net_recv_line(int fd, char *buf, size_t maxlen);

// Send formatted text for convenience
int net_sendf(int fd, const char *fmt, ...);

#endif
