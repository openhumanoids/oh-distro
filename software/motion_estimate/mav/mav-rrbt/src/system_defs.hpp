#ifndef _SYSTEM_DEFS_HPP
#define _SYSTEM_DEFS_HPP
#include <list>

/*
 * Abstract class definition for standard system interface
 */
template<class X, class P>
class SystemInterface {
public:
  /*
   * drives the system from one state to a desired state
   *
   * initial_state: the starting state
   * final_state: the state we're steering to
   * path_data: a list of path elements to be filled out along the path
   * returns true if the path is collision free
   */
  virtual bool steer(const X & initial_state, const X & final_state, std::list<P *> & path_data, bot_lcmgl_t * lcmgl =
      NULL, int animate_usleep = 0)=0;

  /*
   * drives the system from one state towards a desired state, and replaces final state with the actual end of the path
   *
   * initial_state: the starting state
   * final_state: the state we're steering to
   * path_data: a list of path elements to be filled out along the path
   * returns true if the path is collision free
   */
  virtual bool steerApprox(const X & initial_state, X & final_state, std::list<P *> & path_data, bot_lcmgl_t * lcmgl =
      NULL, int animate_usleep = 0)=0;

  /*
   * sample a state element from a uniform distribution in the state space
   *
   * returns a sample
   */
  virtual void sample(X & x_samp) = 0;

  /*
   * compute the distance between two states, used for finding near and nearest
   * state1: a state
   * state2: a state
   * returns distance from state1 to state2
   */
  virtual double distance(const X & state1, const X & state2) = 0;

  /*
   * returns the state dimensionality, used in determining ball radius
   *
   * returns n, the state dimension
   */
  virtual int getStateDimension() = 0;

  /*
   * tests whether a state element is on the goal
   *
   * returns true if state_element in goal
   */
  virtual bool onStateGoal(const X & state_element) = 0;

  virtual X getXInit() = 0;

  virtual bool simulate(const std::list<P *> & path_data, bot_lcmgl_t * lcmgl = NULL) = 0;

  virtual ~SystemInterface()
  {

  }

  /*
   * drawing functions
   */
  //used for drawing dynamic system (obstacles, beacons, start, goal, etc)
  virtual void lcmgl_system(bot_lcmgl_t * lcmgl) = 0;
  virtual void lcmgl_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data) = 0;
  virtual void lcmgl_state(bot_lcmgl_t * lcmgl, const X & x) = 0;

};

/*
 * Abstract interface class for a dynamic system
 */
template<class X, class P, class A>
class AugSystemInterface: public SystemInterface<X, P> {
public:

  /*
   * propagates the augmented state along a path defined by path_data
   *
   * path_data: a list of path elements previously computed by steer
   * aug_init: the initial augmented state (must correspond to the start of path_data
   * aug_final: the final augmented state (must correspond to the end of path_data
   * returns true if the path is collision free
   */
  virtual bool propagate(const std::list<P *> & path_data, const A & aug_init, A & aug_final, double * cost,
      bot_lcmgl_t * lcmgl = NULL, int animate_usleep = 0) = 0;

  /*
   * tests whether an augmented element is on the goal
   *
   * returns true if aug in the augmented goal region
   */
  virtual bool onAugGoal(const X & x, const A & aug) = 0;

  virtual bool compareAugPartial(const A & aug1, const A & aug2, double epsilon) = 0;

  virtual bool compareAugTotal(const A & aug1, const A & aug2) = 0;

  virtual double getAugCost(const A & aug) = 0;

  virtual A getAugInit() = 0;

  /*
   * drawing functions
   */
  virtual void lcmgl_aug(bot_lcmgl_t * lcmgl, const X & x, const A & aug) = 0;
  virtual void lcmgl_aug_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data, const A & aug_init,
      const A & aug_final) = 0;
  virtual void lcmgl_aug_along_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data, const A & aug_init) = 0;

};

#endif
