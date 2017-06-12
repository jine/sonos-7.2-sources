/* socket.h */
#ifndef _SOCKET_H
#define _SOCKET_H

int read_interface(const char *interface, int *ifindex, u_int32_t *addr, unsigned char *arp);
int listen_socket(unsigned int ip, int port, const char *inf);
int raw_socket(int ifindex);

#if defined(__SONOS_LINUX__) && !defined(SONOS_ARCH_ARM)
int bridge_ready(const char *interface);
int ethernet_available();
#endif

#endif
