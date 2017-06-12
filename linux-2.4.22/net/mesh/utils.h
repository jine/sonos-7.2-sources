#ifndef UTILS_H
#define UTILS_H

#include <linux/module.h>
#include <linux/kernel.h>

int seq_greater(u_int32_t seq_one, u_int32_t seq_two);
int seq_less_or_equal(u_int32_t seq_one, u_int32_t seq_two);

int inet_aton(const char *cp, __u32 *addr);
char *inet_ntoa(__u32 ina);

int send_message(u_int32_t dst_ip,u_int8_t ttl, void *data, int datalen);
int send_multicast_message(u_int32_t dst, u_int16_t datalen, void *data,
        u_int8_t ttl);
int send_broadcast(u_int8_t ttl, void *data, int datalen);

u_int64_t get_current_time(void);

int init_sock(struct socket *sock, u_int32_t ip, char *dev_name);
int init_multicast_sock(void);
void close_sock(void);

void init_iw_sock(void);
void close_iw_sock(void);
void setup_spy_interface(void);
void get_wireless_stats(void);
void update_signal_strength_by_hw(char *hw_addr, u_int8_t signal_strength);

#endif 
