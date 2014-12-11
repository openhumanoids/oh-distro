#ifndef _atlas_camera_renderer_hpp_
#define _atlas_camera_renderer_hpp_

typedef struct _BotViewer BotViewer;
typedef struct _BotRenderer BotRenderer;

// simple c-style method that hides implementation from viewer main
// alternatively could have class definition here and instantiate in viewer main
void atlas_camera_renderer_setup(BotViewer* iViewer, const int iPriority,
                                 const lcm_t* iLcm, const BotParam* iParam,
                                 const BotFrames* iFrames);

// simple c-style method that attaches another simple renderer to this one
void atlas_camera_renderer_add_overlay(BotRenderer* iCamRenderer,
                                       BotRenderer* iOverlayRenderer);
 
#endif
