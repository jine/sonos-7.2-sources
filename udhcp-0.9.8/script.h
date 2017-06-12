#ifndef _SCRIPT_H
#define _SCRIPT_H

void run_script(struct dhcpMessage *packet, const char *name);
void run_script_zeroconf(const char* ip, 
                         const char* netmask, const char *name);

#endif
