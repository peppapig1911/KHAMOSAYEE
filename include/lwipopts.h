#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Must be 0 when using FreeRTOS — enables sys_arch (mutexes, semaphores, mailboxes)
#define NO_SYS 0
#define LWIP_SOCKET 1
#define LWIP_NETCONN 1         // sockets layer requires netconn
#define SYS_LIGHTWEIGHT_PROT 1 // thread-safe pbuf/mem operations

#define MEM_LIBC_MALLOC 0
#define MEMP_MEM_MALLOC 0
#define MEM_ALIGNMENT 4
#define MEM_SIZE 4000
#define MEMP_NUM_TCP_SEG 32
#define MEMP_NUM_ARP_QUEUE 10
#define PBUF_POOL_SIZE 24

#define LWIP_ARP 1
#define LWIP_ETHERNET 1
#define LWIP_ICMP 1
#define LWIP_RAW 1
#define LWIP_TCP 1
#define LWIP_UDP 1
#define LWIP_DHCP 1
#define LWIP_IPV4 1
#define LWIP_DNS 1
#define LWIP_TCP_KEEPALIVE 1
#define LWIP_NETIF_TX_SINGLE_PBUF 1
#define DHCP_DOES_ARP_CHECK 0
#define LWIP_DHCP_DOES_ACD_CHECK 0

#define TCP_MSS 1460
#define TCP_WND (8 * TCP_MSS)
#define TCP_SND_BUF (8 * TCP_MSS)
#define TCP_SND_QUEUELEN ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))

#define LWIP_HTTPD_CGI 0
#define LWIP_HTTPD_SSI 0
#define LWIP_HTTPD_SSI_INCLUDE_TAG 0

#define LWIP_ALTCP 1
#define LWIP_ALTCP_TLS 1

#define LWIP_COMPAT_SOCKETS 1
// #define LWIP_POSIX_SOCKETS_IO_NAMES 1

// Needed for DNS resolution (inet_aton, gethostbyname etc.)
#define LWIP_DNS_API_DEFINE_TYPES 1

#define LWIP_TIMEVAL_PRIVATE 0

#endif