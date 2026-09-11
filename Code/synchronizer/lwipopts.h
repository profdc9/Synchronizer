/* lwipopts.h - lwIP configuration for the Synchronizer.

   NO_SYS, raw API only: the project needs UDP and DNS and nothing else. */

#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

#include <stdlib.h>          /* for rand(), which LWIP_RAND needs below */

#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0

#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    16000
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24

/* TCP, for the built-in web interface.  A handful of short-lived
   connections serving a page of a few kilobytes - no more than that. */
#define MEMP_NUM_TCP_PCB            8
#define MEMP_NUM_TCP_PCB_LISTEN     2
#define TCP_MSS                     1460
#define TCP_SND_BUF                 (4 * TCP_MSS)
#define TCP_WND                     (4 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * TCP_SND_BUF) / TCP_MSS)

/* mDNS needs a random source, multicast group membership, one slot of
   per-netif client data, and one more system timeout than it would
   otherwise use.  Miss any of them and mdns.c refuses to compile. */
#define LWIP_RAND()                 ((u32_t)rand())
#define LWIP_IGMP                   1
#define LWIP_NUM_NETIF_CLIENT_DATA  2
/* lwIP sizes this pool from a formula that does not count mDNS at all, and
   mdns.c only asks for "one more".  One is not enough here: the responder
   runs on two interfaces - the station and the setup access point - and its
   probe and announce sequence schedules several timeouts of its own.  Too
   small and lwIP does not degrade, it panics:

     *** PANIC *** sys_timeout: timeout != NULL, pool MEMP_SYS_TIMEOUT is empty

   which halts the firmware outright - no console, no DHCP, just an SSID you
   can associate with.  Eight costs about 128 bytes. */
#define MEMP_NUM_SYS_TIMEOUT        (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 8)
#define LWIP_MDNS_RESPONDER         1
#define MDNS_MAX_SERVICES           1

/* DHCP server, DNS hijack, NTP client and mDNS each want one. */
#define MEMP_NUM_UDP_PCB            8

#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define LWIP_UDP                    1
#define LWIP_TCP                    1
#define LWIP_DHCP                   1
#define LWIP_DNS                    1
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0

#define DNS_TABLE_SIZE              2
#define DNS_MAX_NAME_LENGTH         64

#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETCONN_SEM_PER_THREAD 0
#define LWIP_CHKSUM_ALGORITHM       3

#define LWIP_STATS                  0
#define LWIP_STATS_DISPLAY          0

#ifndef NDEBUG
#define LWIP_DEBUG                  1
#define LWIP_STATS                  0
#endif

#define ETHARP_DEBUG                LWIP_DBG_OFF
#define NETIF_DEBUG                 LWIP_DBG_OFF
#define PBUF_DEBUG                  LWIP_DBG_OFF
#define API_LIB_DEBUG               LWIP_DBG_OFF
#define IP_DEBUG                    LWIP_DBG_OFF
#define UDP_DEBUG                   LWIP_DBG_OFF
#define TCP_DEBUG                   LWIP_DBG_OFF
#define DHCP_DEBUG                  LWIP_DBG_OFF
#define DNS_DEBUG                   LWIP_DBG_OFF

#endif /* _LWIPOPTS_H */
