#ifndef CT_SWIC_H_
#define CT_SWIC_H_

/* Define the architecture. Currently supported X86 and PPC */ 
/* #define ARCH_X86 */
#define ARCH_PPC

typedef struct Q_Tag {
  u32 tag ;
  u32 a_time ;
} Q_Tag ;

#define HOOK_BUFFER_SIZE  2048  /* must be a power of two */

struct Hook_Buffer {
  int head ;
  volatile int tail ;
  Q_Tag buffer[HOOK_BUFFER_SIZE] ;
} ;

#endif
