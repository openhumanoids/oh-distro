#ifndef _DRAWING_DEFS_HPP
#define _DRAWING_DEFS_HPP

#include <GL/gl.h>

//-------- configuration space properties -------------
// obstacles
#define BT_OBSTACLE_COLOR 0,0,0

// bounding box
#define BT_BB_COLOR BT_OBSTACLE_COLOR
#define BT_BB_LINE_WIDTH 8.0

#define BT_REGION_COLOR 0,.7,0
#define BT_GOAL_COLOR 1,.5,0

//--------------- Belief Tree Drawing Properties -----------------------
#define BT_EDGE_COLOR .5,.5,.5
#define BT_EDGE_WIDTH 3

#define BT_BELIEF_PARENT_COLOR 0,0,1
#define BT_BELIEF_PARENT_WIDTH 1

#define BT_BELIEF_CHILD_COLOR 1, 0, 1
#define BT_BELIEF_CHILD_WIDTH 1

#define BT_BELIEF_NODE_COLOR 1,0,0
#define BT_BELIEF_NODE_WIDTH 2

#define BT_VERTEX_COLOR 0,0,1
#define BT_VERTEX_SIZE 5

#define BT_NEW_SAMPLE_COLOR 0,0,.5
#define BT_NEW_SAMPLE_SIZE 8

#define BT_NEAREST_COLOR .3,.3,1
#define BT_NEAREST_SIZE 8

#define BT_NEAR_COLOR 0,.8,.8
#define BT_NEAR_SIZE 4

#define BT_BALL_COLOR 0,1,0
#define BT_BALL_WIDTH 5

#define BT_OPTIMAL_PATH_COLOR .5,0,0
#define BT_OPTIMAL_PATH_WIDTH 5
#define BT_OPTIMAL_AUG_WIDTH 2

#define BT_STEER_COLOR 0,0,.5
#define BT_STEER_COLOR_COLLISION 1,0,0
#define BT_STEER_WIDTH 4

#define BT_PROPAGATE_COLOR 0,0,1
#define BT_PROPAGATE_COLOR_COLLISION 1,0,0
#define BT_PROPAGATE_WIDTH BT_BELIEF_NODE_WIDTH

#define BT_SIMULATE_COLOR 0,0,1
#define BT_SIMULATE_SIZE 5
#define BT_SIMULATE_WIDTH .5

#define DEPTH_TEST true

#define Z_LINES 0.0
#define Z_REGIONS -1.0
#define Z_REGIONS_CIRCLE -2.0
#define Z_SAMPLE_TRAJ -0.5

#define DRAW_ENTROPY_SCALE 3.0
#define DRAW_PROB_SCALE 15.0

/*
 * setup depth testing for a given lcmgl object (slows down drawing significantly)
 */
static inline void initDepthTest(bot_lcmgl_t * lcmgl)
{
  if (DEPTH_TEST) {
    bot_lcmgl_push_attrib(lcmgl, GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
    bot_lcmgl_enable(lcmgl, GL_DEPTH_TEST);
    bot_lcmgl_depth_func(lcmgl, GL_LESS);
  }
}

/*
 * clear depth testing for a given lcmgl object (should be called after initDepthTest after drawing is done)
 */
static inline void endDepthTest(bot_lcmgl_t * lcmgl)
{
  if (DEPTH_TEST)
    bot_lcmgl_pop_attrib(lcmgl);
}

#endif
