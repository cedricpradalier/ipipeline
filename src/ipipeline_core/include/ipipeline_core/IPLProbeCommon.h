#ifndef IPL_PROBE_COMMON_H
#define IPL_PROBE_COMMON_H


/** Possible request on a probe **/
typedef enum {
	Probe_NOOP=0, 
	Probe_SetIndex=1, 
	Probe_ToNext=2, 
	Probe_ToPrev=3, 
	Probe_Reset=4
} IPLProbeAction;


#endif // IPL_PROBE_COMMON_H
