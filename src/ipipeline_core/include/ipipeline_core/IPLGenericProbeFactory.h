#ifndef IPL_GENERIC_PROBE_FACTORY_H
#define IPL_GENERIC_PROBE_FACTORY_H

#include "ipipeline_core/IPLGenericProbe.h"


/**
 * \class IPLGenericProbeFactory: example of probe factory class
 * Mostly useful for the test programs, given that the GenericProbe is
 * not able to do anything
 * */

class IPLGenericProbeFactory 
{
    public:
        IPLGenericProbeFactory() {}
        ~IPLGenericProbeFactory() {}

        IPLGenericProbe * operator()(unsigned int i) {
            char probename[64];
            sprintf(probename,"Probe%04d",i);
            return new IPLGenericProbe(probename);
        }
};



#endif // IPL_GENERIC_PROBE_FACTORY_H
