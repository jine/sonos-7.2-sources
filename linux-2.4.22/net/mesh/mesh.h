#ifndef MESH_H
#define MESH_H

#define AODVPORT 654

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#ifndef MAX
#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#endif

/* Protocol constants are defined in draft-ietf-manet-aodv-13.txt */
#define ACTIVE_ROUTE_TIMEOUT   60000
#define ALLOWED_HELLO_LOSS     3
#define DELETE_PERIOD          MAX((ACTIVE_ROUTE_TIMEOUT), \
                                   ((ALLOWED_HELLO_LOSS) * (HELLO_INTERVAL)))
#define HELLO_INTERVAL         1000
#define MY_ROUTE_TIMEOUT       2 * (ACTIVE_ROUTE_TIMEOUT)
#define NET_DIAMETER           35
#define NET_TRAVERSAL_TIME     3 * NODE_TRAVERSAL_TIME * NET_DIAMETER / 2
#define NEXT_HOP_WAIT          (NODE_TRAVERSAL_TIME) + 10
#define NODE_TRAVERSAL_TIME    50
#define PATH_TRAVERSAL_TIME    2 * NET_TRAVERSAL_TIME
#define RREQ_RETRIES           2
#define TTL_START              1
#define TTL_INCREMENT          2
#define TTL_THRESHOLD          7

/* AODV message types */
#define RREQ                   1
#define RREP                   2
#define RERR                   3
#define RREP_ACK               4
#define MACT                   5
#define GRPH                   6

#endif
