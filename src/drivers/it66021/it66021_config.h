#ifndef _COOLFLY_IT66021_CONFIG_H_
#define _COOLFLY_IT66021_CONFIG_H_

#include "px4_log.h"

#define MS_TimeOut(x) (x+1)

#ifndef TRUE 
	#define TRUE  (1)
#endif

#ifndef FALSE
	#define FALSE (0)
#endif

// #define IT66021DEBUG

#ifdef IT66021DEBUG
#define  IT_INFO(...) PX4_INFO(__VA_ARGS__)
#else
#define  IT_INFO(...)
#endif


#endif