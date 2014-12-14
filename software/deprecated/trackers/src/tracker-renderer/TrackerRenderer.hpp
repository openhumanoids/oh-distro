#ifndef _TrackerRenderer_hpp_
#define _TrackerRenderer_hpp_

// simple c-style method that hides implementation from viewer main
void tracker_renderer_setup(BotViewer* iViewer,
                            const int iPriority,
                            const lcm_t* iLcm,
                            const BotParam* iParam,
                            const BotFrames* iFrames);

#endif
