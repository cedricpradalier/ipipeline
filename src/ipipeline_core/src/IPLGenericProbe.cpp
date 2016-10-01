
#include "ipipeline_core/IPLGenericProbe.h"

void IPLGenericProbe::processCommand(IPLProbeAction action, unsigned int index)
{
	IPLImageProcessor * op = NULL;
	int loop = 0;
	if (opstore == NULL) return;
	if (opstore->empty()) return;
    if (action == Probe_NOOP) return;

    ROS_DEBUG("Probe %s command %d index %d",this->getName(),action,index); 

    switch (action) {
        case Probe_NOOP: 
            return;
        case Probe_SetIndex: 
            inputIndex = index;
            // processInput();
            return;
        case Probe_ToNext :
            {
                op = *opit;
                op->removeReceiver(this);
                do {
                    ++ opit;
                    if (opit == opstore->end()) {
                        opit = opstore->begin();
                        ++ loop;
                        if (loop > 1) break;
                    }
                    op = *opit;
                } while (op->getOutput().size() == 0);
                op->addReceiver(this);
                // processInput();
                break;
            }
        case Probe_ToPrev :
            {
                op = *opit;
                op->removeReceiver(this);
                do {
                    if (opit == opstore->begin()) {
                        opit = opstore->end();
                        ++ loop;
                        if (loop > 1) break;
                    }
                    -- opit;
                    op = *opit;
#ifdef PROBE_DEBUG
                    printf("SDLProbe : testing '%s'\n",op->getName());
#endif // PROBE_DEBUG
                } while (op->getOutput().size() == 0);
#ifdef PROBE_DEBUG
                printf("SDLProbe : accepted '%s'\n",op->getName());
#endif // PROBE_DEBUG
                op->addReceiver(this);
                // processInput();
                break;
            }
        case Probe_Reset :
            {
                op = *opit;
                op->removeReceiver(this);
                opit = opstore->begin();
                op = *opit;
                op->addReceiver(this);
                // processInput();
                break;
            }

    }
}

