#ifndef __DEBUG_H__
#define __DEBUG_H__

//
// Uncomment to add debugging print information
//
#define DEBUG

//
// DBG_VPRINTF is the normal printf
// DBG_CPRINTF is to print constant strings
//
#ifdef DEBUG

#include <assert.h>

#define DBG_PRINTF(fmt,args...) printf(fmt, ##args)
#define DBG_KPRINTF(fmt)        nrk_kprintf(PSTR(fmt)) 
#define DBG_GET_TIME(time)		nrk_time_get(&time)
#define DBG_RESPONSE(start,end)					\
  (((end).secs-(start).secs)*1000+				\
   ((end).nano_secs-(start).nano_secs)/1000000)

#else // DEBUG

//
// No-ops
//

#define assert(x)               ((void)0)

#define DBG_PRINTF(fmt,args...) ((void)0)
#define DBG_KPRINTF(fmt)        ((void)0)
#define DBG_GET_TIME(time)      ((void)0)
#define DBG_RESPONSE(start,end) ((void)0)
#endif // DEBUG

#endif // __DEBUG_H__
