#ifndef _UDPUTIL_H
#define _UDPUTIL_H

#ifdef __cplusplus
extern "C" {
#endif

/** make and bind a udp socket to an ephemeral port. Returns the fd. **/
int udp_socket_create(void);

/** make and bind a udp socket to a specified port. Returns the fd. **/
int udp_socket_listen(int port);

/** return the local port number for a socket. **/
int udp_socket_get_port(int sock);

// convenience method that sends a one-off udp message
// return 0 on success
int udp_send(const char *ipaddr, int port, const void *data, int datalen);

#define udp_send_string(ipaddr, port, string) udp_send(ipaddr, port, string, strlen(string))

#ifdef __cplusplus
}
#endif

#endif
