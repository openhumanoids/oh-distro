#ifndef _BELIEF_TREE_HPP
#define _BELIEF_TREE_HPP

#include <list>
#include <queue>
#include <assert.h>
#include <math.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <bot_core/lcm_util.h>
#include <kdtree/kdtree.h>
#include <Eigen/Dense>
#include <eigen_utils/eigen_utils.hpp>
#include "system_defs.hpp"
#include "drawing_defs.hpp"

template<class X, class P, class A>
class BeliefNodeT;

template<class X, class P, class A>
class StateVertexT;

template<class X, class P, class A>
class EdgeT;

template<class X, class P, class A>
class BeliefTree;

template<class T>
void freeList(std::list<T *> & ptr_list)
{
  typename std::list<T *>::iterator ptr_it = ptr_list.begin();
  while (!ptr_list.empty()) {
    T * cur_ptr = *ptr_it;
    ptr_it = ptr_list.erase(ptr_it);
    delete cur_ptr;
  }
}

/*
 * vertices in the sub state
 */
template<class X, class P, class A>
class StateVertexT {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef BeliefNodeT<X, P, A> BeliefNode;
  typedef StateVertexT<X, P, A> StateVertex;
  typedef EdgeT<X, P, A> Edge;

  X x; //the state element at vertex
  std::list<BeliefNode *> belief_nodes; //the owning list of belief nodes at this vertex
  std::list<Edge *> out_edges; //the owning list of edges eminating from this vertex
  bool on_goal; //is this vertex on the goal?

  /*
   * construct state vertex
   *
   * x: state
   * on_goal ?
   */
  StateVertexT(X & x, bool on_goal) :
      belief_nodes(), out_edges()
  {
    this->x = x;
    this->on_goal = on_goal;
  }

  /*
   * default constructor initializes lists, but nothing else
   */
  StateVertexT() :
      belief_nodes(), out_edges()
  {

  }

  ~StateVertexT()
  {

    freeList(belief_nodes);
    freeList(out_edges);

  }

};

/*
 * sub class forms the belief nodes of augmented state at each sub state
 */
template<class X, class P, class A>
class BeliefNodeT {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef BeliefNodeT<X, P, A> BeliefNode;
  typedef StateVertexT<X, P, A> StateVertex;
  typedef EdgeT<X, P, A> Edge;

  A * aug; //the augmented state element
  double cost; //the cost (this should be handled as part of the augmented state, but is kept for convenience
  BeliefNode * parent_belief; //pointer to parent belief node
  Edge * to_edge; //pointer to the edge that takes you to this belief node
  std::list<BeliefNode *> children_beliefs; //list of pointers to children beliefs
  StateVertex * state_vertex; //pointer to the state vertex that owns this belief
  bool is_constraint; //flag for whether this belief node is a constraint (i.e. just a container for aug checks)
  bool to_delete; //flag for deleting a belief node, that is on the queue (so we have to wait to delete it)
  bool on_queue; //flag to keep track of whether it is on the queue
  Edge * new_edge; //edge to be expanded when the node comes off the queue, if null, then all the edges out of the owning vertex are expanded

  BeliefTree<X, P, A> * belief_tree;

  /*
   * standard constructor
   *
   * aug: the augmented state element
   * cost: the cost at this belief (should also be part of aug, this is just for convenience)
   * parent_belief: the parent of this belief node (i.e. the node just prior to this one in the path)
   * to_edge: the edge that one follows from parent to this belief node
   * state_vertex: the owning state vertex
   */
  BeliefNodeT(A * aug, double cost, BeliefNode * parent_belief, Edge * to_edge, StateVertex * state_vertex, BeliefTree<
      X, P, A> * belief_tree) :
      children_beliefs()
  {
    this->aug = aug;
    this->parent_belief = parent_belief;
    this->to_edge = to_edge;
    this->cost = cost;
    this->state_vertex = state_vertex;
    this->is_constraint = false;
    this->on_queue = false;
    this->to_delete = false;
    this->belief_tree = belief_tree;
    this->belief_tree->num_belief_nodes++;

    if (this->parent_belief != NULL) {
      this->parent_belief->children_beliefs.push_front(this);
    }
  }

  /*
   * copy constructor, only used for copying to a constraint, everything else not done.
   */
  BeliefNodeT(const BeliefNode & bel) :
      children_beliefs()

  {
    this->aug = new A(*bel.aug);
    this->cost = bel.cost;
    this->parent_belief = NULL;
    this->to_edge = NULL;
    this->is_constraint = true;
    this->to_delete = false;
    this->on_queue = false;
    this->state_vertex = NULL;
    this->belief_tree = bel.belief_tree;
  }

  /*
   * destructor sets the parent node for all the children of this node to null
   */
  ~BeliefNodeT()
  {
    if (!this->is_constraint) {

      typename std::list<BeliefNode *>::iterator child_it;
      for (child_it = children_beliefs.begin(); child_it != children_beliefs.end(); child_it++) {
        (*child_it)->parent_belief = NULL;
      }

      this->state_vertex->belief_nodes.remove(this);

      if (this->parent_belief != NULL) {
        this->parent_belief->children_beliefs.remove(this);
      }

      if (this->to_edge != NULL && this->parent_belief != NULL) {
        belief_tree->addEdgeConstraint(this->to_edge, this->parent_belief);
      }

      if (this == belief_tree->min_belief_at_goal) {
        belief_tree->min_belief_at_goal = NULL;
      }

      this->belief_tree->num_belief_nodes--;
    }

    delete this->aug;

  }

  /*
   * gets the set of out edges from the owning state vertex for which this belief does not have a child
   */
  void getOutEdgesForExpansion(std::list<Edge *> & out_edges_for_expansion)
  {
    if (this->new_edge == NULL) { //case where this node was generated by search

      if (true) { //this is the right way to do it which makes the algorithm n*log(n)^2
        typename std::list<Edge *>::iterator edge_it;
        for (edge_it = state_vertex->out_edges.begin(); edge_it != state_vertex->out_edges.end(); edge_it++) {
          out_edges_for_expansion.push_back(*edge_it);
        }
      }
      else { //this wrong, but works if there is always a total ordering beliefs (essentially reverts to rrt*) and makes it n*log(n)
        typename std::list<BeliefNode *>::iterator child_bel_it;
        for (child_bel_it = this->children_beliefs.begin(); child_bel_it != this->children_beliefs.end();
            child_bel_it++) {
          out_edges_for_expansion.push_back((*child_bel_it)->to_edge);
        }
      }
    }
    else { //case where this node was generated by adding an out edge to the owning vertex (if that's the case we only want to go out that vertex)
      out_edges_for_expansion.push_back(this->new_edge);
    }
  }

};

template<class X, class P, class A>
class beliefNodeCompare {
public:
  beliefNodeCompare()
  {
  }
  bool operator()(BeliefNodeT<X, P, A> * bel1, BeliefNodeT<X, P, A> * bel2) const
      {
    return bel1->cost > bel2->cost;
  }
};

template<class X, class P, class A>
class EdgeT {
public:

  typedef BeliefNodeT<X, P, A> BeliefNode;
  typedef StateVertexT<X, P, A> StateVertex;
  typedef EdgeT<X, P, A> Edge;

  StateVertex * to_vertex; //the vertex the edge goes to
  StateVertex * from_vertex; //the vertex the edge comes from
  std::list<BeliefNode *> belief_constraints; //the belief constraints for this edge (owning list)
  std::list<P *> path_data;

  EdgeT(StateVertex * from_vertex, StateVertex * to_vertex) :
      belief_constraints(), path_data()
  {
    this->from_vertex = from_vertex;
    this->to_vertex = to_vertex;
  }

  ~EdgeT()
  {
    freeList(path_data);
    freeList(belief_constraints);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

class BeliefTreeInterface {
public:
  virtual void iterateRRBT() = 0;
  virtual double getMinCostAtGoal() = 0;
  virtual int getNumStateVertices() = 0;
  virtual int getNumBeliefNodes() = 0;
  virtual double simOptPath(int num_sims) = 0;
  virtual void lcmgl_draw() = 0;
  virtual int getNumNearLast() = 0;
  virtual double getBallRadius() = 0;

  virtual ~BeliefTreeInterface()
  {

  }
};

enum lcmgl_name_enum {
  edges_lcmgl,
  belief_children_lcmgl,
  belief_parents_lcmgl,
  belief_nodes_lcmgl,
  state_vertices_lcmgl,
  propagate_steer_lcmgl,
  new_sample_lcmgl,
  nearest_lcmgl,
  near_lcmgl,
  environment_lcmgl,
  optimal_path_lcmgl
};

char lcmgl_names[][50] = { "edges_lcmgl",
    "belief_children",
    "belief_parents",
    "belief_nodes",
    "state_vertices",
    "propagate_steer",
    "new_sample",
    "nearest",
    "near",
    "environment",
    "optimal_path" };

typedef enum {
  bt_no_visualization = 0, bt_draw = 1, bt_animate = 2, bt_draw_at_end = 3
} bt_visualization_t;

typedef enum {
  depth_first, breadth_first, uniform_cost
} search_t;

template<class X, class P, class A>
class BeliefTree: public BeliefTreeInterface {
protected:
  typedef BeliefNodeT<X, P, A> BeliefNode;
  typedef StateVertexT<X, P, A> StateVertex;
  typedef EdgeT<X, P, A> Edge;

  AugSystemInterface<X, P, A> * dynamic_system;
  typename std::list<BeliefNode *> search_list;
  typename std::priority_queue<BeliefNode *, std::vector<BeliefNode *>, beliefNodeCompare<X, P, A> > search_queue;
  std::list<StateVertex *> state_vertices;
  int num_state_vertices;
  int num_belief_nodes;

  int num_near_last;

  //  typename std::priority_queue<BeliefNode *> search_queue;
  BeliefNode * min_belief_at_goal;

  //rrt*/rrg related constants for ball size
  double max_ball_radius;
  double ball_radius_constant;

  //epsilon comparison threshold for comparing beliefs
  double epsilon;

  int draw_mod;
  int animate_usleep;
  bt_visualization_t draw_mode;
  bool RRBT_mode; //flag, false = just RRT, true = RRBT with graph search
  search_t search_order; //flag, 0 - depth, 1 - breadth, 2 - uniform cost

  //the starting state vertex
  StateVertex * root_state_vertex;

  /*
   * inserts a belief node into a list of belief nodes, pruning all the beliefs that the new_belief dominates,
   * if the new belief is dominated by any existing belief, it is not inserted
   * maintains belief_nodes, such that belief_node_i < not greater than belief_node_j for all i,j
   *
   * epsilon is scalar parameter for force new_belief<<(epsilon) belief_nodes_i
   * if new_belief does not fit, it is deleted
   * returns true if fits, false otherwise
   */
  bool beliefInsertPartial(std::list<BeliefNode *> & belief_nodes, BeliefNode * new_belief, double epsilon = 0);

  /*
   * returns ? exist i : new_belief>belief_nodes_i (same operations as beliefInsertPartial, but doesn't insert the new belief)
   * used for constraint checking on edges
   *
   * returns false if exist i : new_belief<belief_nodes_i (if the new belief is dominated by any belief_nodes_i)
   */
  bool beliefCheckPartial(const std::list<BeliefNode *> & belief_nodes, BeliefNode * new_belief, double epsilon = 0);

  /*
   * adds a constraint to an edge - any node dominated by this constraint won't be allowed to propagate
   */
  void addEdgeConstraint(Edge * edge, const BeliefNode * belief_node)
  {
    BeliefNode * new_belief_constraint = new BeliefNode(*belief_node);
    beliefInsertPartial(edge->belief_constraints, new_belief_constraint, 0);
  }

  BeliefNode * popSearchQueue();
  void pushSearchQueue(BeliefNode * belief_node, Edge * new_edge);
  bool queueNotEmpty()
  {
    if (this->search_order == uniform_cost)
      return !this->search_queue.empty();
    else if (this->search_order == depth_first || this->search_order == breadth_first)
      return !this->search_list.empty();
  }

  /*
   * hide in function in case we want to make espilon variable at some point
   */
  double getEpsilon()
  {
    return epsilon;
  }

  /*
   * near function gets the near and nearest using dynamic system distance metric
   */
  virtual StateVertex * near(StateVertex * new_vertex, std::list<StateVertex *> & near_vertices);

  void expandBelief(BeliefNode * belief_node);

  /*
   * add an edge between two states
   * require_belief_connection: if true, the edge will only be added if at least 1 belief can be connected from starting to ending
   */
  bool addEdge(StateVertex * starting_vertex, StateVertex * ending_vertex, bool first_connection = false);

  /*
   * queues the beliefs at starting vertex, and inserts edge into that vertex
   */
  void insertEdge(StateVertex * starting_vertex, Edge * new_edge);

  void reconstructPath(const BeliefNode * end_belief, std::list<P *> & path);

  void setMinBeliefAtGoal();

  virtual void addVertex(StateVertex * new_state_vertex);

public:
  //drawing stuff
  lcm_t * lcm;

  std::vector<bot_lcmgl_t *> lcmgl_objs;

  bot_lcmgl_t * prop_steer_lcmgl;

  bool found_goal;

  friend class BeliefNodeT<X, P, A> ;

  void baseConstructor(AugSystemInterface<X, P, A> * dynamic_system_arg, double ball_radius_constant,
  double max_ball_radius, double epsilon, int draw_mod, bt_visualization_t draw_mode_, int animate_usleep,
  bool RRBT_mode, search_t search_order)
  {
    dynamic_system = dynamic_system_arg;

    this->num_state_vertices = 0;
    this->num_belief_nodes = 0;
    this->num_near_last = 0;

    this->RRBT_mode = RRBT_mode;
    this->search_order = search_order;

    min_belief_at_goal = NULL;
    found_goal = false;

    this->max_ball_radius = max_ball_radius;
    this->ball_radius_constant = ball_radius_constant;
    this->epsilon = epsilon;

    X x_init = dynamic_system->getXInit();
    A aug_init = dynamic_system->getAugInit();
    this->root_state_vertex = new StateVertex(x_init, this->dynamic_system->onStateGoal(x_init));
    this->root_state_vertex->belief_nodes.push_back(new BeliefNode(new A(aug_init), 0, NULL, NULL,
        this->root_state_vertex, this));
    this->addVertex(this->root_state_vertex);

    this->lcm = bot_lcm_get_global(NULL);
    this->draw_mod = draw_mod;
    this->draw_mode = draw_mode_;
    this->animate_usleep = animate_usleep;

    for (int ii = edges_lcmgl; ii <= optimal_path_lcmgl; ii++) {
      this->lcmgl_objs[ii] = bot_lcmgl_init(this->lcm, lcmgl_names[ii]);
      if (this->draw_mode == bt_draw || this->draw_mode == bt_animate)
        bot_lcmgl_switch_buffer(this->lcmgl_objs[ii]);
    }

    //if we're displaying prop and steer, turn this on so it gets passed on those functions
    if (this->draw_mode == bt_animate) {
      prop_steer_lcmgl = this->lcmgl_objs[propagate_steer_lcmgl];
    }
    else {
      prop_steer_lcmgl = NULL;
    }

    if (this->draw_mode == bt_draw || this->draw_mode == bt_animate) {
      this->lcmgl_draw_dynamic_system();
    }
  }

  BeliefTree(AugSystemInterface<X, P, A> * dynamic_system_arg, double ball_radius_constant, double max_ball_radius,
  double epsilon, int draw_mod, bt_visualization_t draw_animate, int animate_usleep, bool RRBT_mode,
  search_t search_order) :
      state_vertices(), search_list(), lcmgl_objs(optimal_path_lcmgl + 1), search_queue()
  {
    this->baseConstructor(dynamic_system_arg, ball_radius_constant, max_ball_radius, epsilon, draw_mod, draw_animate,
        animate_usleep, RRBT_mode, search_order);
  }

  ~BeliefTree()
  {
    for (int ii = edges_lcmgl; ii <= optimal_path_lcmgl; ii++) {
      bot_lcmgl_destroy(this->lcmgl_objs[ii]);
    }

    lcm_destroy(this->lcm);

    freeList(this->state_vertices);

    delete this->dynamic_system;
  }

  double getBallRadius()
  {
    int num_mu_nodes = this->getNumStateVertices();
    double rn = pow(this->ball_radius_constant * log(num_mu_nodes) / num_mu_nodes, 1.0
        / ((double) this->dynamic_system->getStateDimension()));
    if (rn > this->max_ball_radius || num_mu_nodes < 2)
      rn = this->max_ball_radius;
    return rn;
  }

  int getNumNearLast()
  {
    return this->num_near_last;
  }

  /*
   * iterate the RRBT
   */
  void iterateRRBT();

  int getNumStateVertices()
  {
    return this->num_state_vertices;
  }

  int getNumBeliefNodes()
  {
    return this->num_belief_nodes;
  }

  const A * getMinAugAtGoal()
  {
    if (min_belief_at_goal == NULL
      )
      return NULL;
    else
      return min_belief_at_goal->aug;
  }

  double getMinCostAtGoal()
  {
    const A * min_aug_at_goal = getMinAugAtGoal();
    if (min_aug_at_goal == NULL) {
      return -1;
    }
    else {
      return dynamic_system->getAugCost(*min_aug_at_goal);
    }
  }

  bool getOptPath(std::list<P *> & path)
  {
    if (min_belief_at_goal == NULL
      )
      return false;
    else {
      reconstructPath(min_belief_at_goal, path);
      return true;
    }

  }

  double simOptPath(int num_sims)
  {

    std::list<P *> opt_path = std::list<P *>();
    if (this->getOptPath(opt_path)) {
      double num_success = 0;
      for (int ii = 0; ii < num_sims; ii++) {
        if (this->dynamic_system->simulate(opt_path, this->lcmgl_objs[propagate_steer_lcmgl])) { //, rrbt.lcmgl_objs[propagate_steer_lcmgl])
          num_success += 1;
        }
      }
      bot_lcmgl_switch_buffer(this->lcmgl_objs[propagate_steer_lcmgl]);

      return num_success / ((double) num_sims);
    }
    else {
      std::cout << "warning, no optimal path to simulate\n";
      return 0;
    }

  }

  void lcmgl_draw()
  {
    bt_visualization_t bt_vis_temp = this->draw_mode;
    this->draw_mode = bt_draw;
    lcmgl_print(0, false);
    this->draw_mode = bt_vis_temp;
  }

  void lcmgl_draw_dynamic_system()
  {
    bot_lcmgl_switch_buffer(this->lcmgl_objs[environment_lcmgl]);
    this->dynamic_system->lcmgl_system(this->lcmgl_objs[environment_lcmgl]);
    bot_lcmgl_switch_buffer(this->lcmgl_objs[environment_lcmgl]);
  }

  void lcmgl_print(int iteration_number, bool intermediate_draw);

  void lcmgl_drawNearNearestNew(const std::list<StateVertex *> & near_states, const StateVertex * nearest_state,
      const StateVertex * new_state);
};

template<class X, class P, class A>
bool BeliefTree<X, P, A>::beliefInsertPartial(std::list<BeliefNode *> & belief_nodes, BeliefNode * new_belief,
    double tolerance)
{

  //  #define _PRUNE_DEBUG
#ifdef _PRUNE_DEBUG
  std::list<A> pruned_aug = std::list<A>();
#endif

  bool has_pruned = false; //for sanity keep track of whether the new_belief has forced pruning of other beliefs

  //iterate through belief_nodes to perform check/pruning
  typename std::list<BeliefNode *>::iterator belief_it;
  for (belief_it = belief_nodes.begin(); belief_it != belief_nodes.end(); belief_it++) {
    BeliefNode * belief_node = *belief_it;

    //if new_belief is dominated return null
    if (dynamic_system->compareAugPartial(*belief_node->aug, *new_belief->aug, tolerance)) {

#ifdef _PRUNE_DEBUG
      if (has_pruned) {
        std::cout << "\n\nAug: \n" << new_belief->aug << "\nPruned " << pruned_aug.size() << " nodes:\n";
        std::list<A>::iterator pruned_it;
        for (pruned_it = pruned_aug.begin(); pruned_it != pruned_aug.end(); pruned_it++) {
          const A & aug_pruned = *pruned_it;
          std::cout << aug_pruned << "\n";
        }
        std::cout << "and is being pruned by:\n" << belief_node->aug;
        //        assert(!has_pruned); //sanity check that it can't be pruned if it has already pruned other nodes
      }
#endif
      delete new_belief;
      return false;
    }
    else if (dynamic_system->compareAugPartial(*new_belief->aug, *belief_node->aug, 0)) { //check to see if it dominates other nodes

#ifdef _PRUNE_DEBUG
      pruned_aug.push_front(*belief_node->aug);
#endif

      //if its on the queue we mark it for deletion as soon as it's popped
      if (belief_node->on_queue) {
        belief_node->to_delete = true;
      }
      else {
        belief_it = belief_nodes.erase(belief_it);
        delete belief_node;
      }
      has_pruned = true;
    }
  }
  //if we've made it this far, add it to the set of belief nodes
  belief_nodes.push_front(new_belief);
  return true;
}

template<class X, class P, class A>
bool BeliefTree<X, P, A>::beliefCheckPartial(const std::list<BeliefNode *> & belief_nodes, BeliefNode * new_belief,
    double tolerance)
{
  //iterate through belief nodes to perform check
  typename std::list<BeliefNode *>::const_iterator belief_it;
  for (belief_it = belief_nodes.begin(); belief_it != belief_nodes.end(); belief_it++) {
    const BeliefNode * belief_node = *belief_it;

    //check to see if the new belief is dominated by the current belief
    if (dynamic_system->compareAugPartial(*belief_node->aug, *new_belief->aug, tolerance))
      return false;
  }
  return true;
}

template<class X, class P, class A>
void BeliefTree<X, P, A>::expandBelief(BeliefNode * belief_node)
{

  if (belief_node == NULL
    )
    return;

  //get a list of the edges for expansion of this belief
  typename std::list<Edge *> out_edges_for_expansion = std::list<Edge *>();
  belief_node->getOutEdgesForExpansion(out_edges_for_expansion);

  //iterate through the edges for expansion
  typename std::list<Edge*>::iterator out_edge_it;
  for (out_edge_it = out_edges_for_expansion.begin(); out_edge_it != out_edges_for_expansion.end(); out_edge_it++) {
    Edge * out_edge = *out_edge_it;

    //check to see if belief_node can be ruled out by an existing constraint
    if (beliefCheckPartial(out_edge->belief_constraints, belief_node)) {

      //try to propagate
      A * aug_final = new A;
      double new_cost;
      if (dynamic_system->propagate(out_edge->path_data, *belief_node->aug, *aug_final, &new_cost, prop_steer_lcmgl,
          this->animate_usleep)) {

        //if propagation is successful, create a new belief
        BeliefNode * new_belief = new BeliefNode(aug_final, belief_node->cost + new_cost, belief_node, out_edge,
            out_edge->to_vertex, this);

        //insertion is successful, add it to the search queue and check for optimality
        if (beliefInsertPartial(out_edge->to_vertex->belief_nodes, new_belief, getEpsilon())) {
          this->lcmgl_print(this->getNumStateVertices(), true);

          pushSearchQueue(new_belief, NULL);

          //check to see if we should replace the optimal belief with this one
          if (new_belief->state_vertex->on_goal) {
            if (dynamic_system->onAugGoal(new_belief->state_vertex->x, *new_belief->aug)) {
              if (this->min_belief_at_goal == NULL) {
                this->min_belief_at_goal = new_belief;
                found_goal = true;
              }
              else if (dynamic_system->compareAugTotal(*new_belief->aug, *min_belief_at_goal->aug)) {
                min_belief_at_goal = new_belief;
              }
            }
          }

        }
        else {
          addEdgeConstraint(out_edge, belief_node);
        }
      }
      else { //if propagation fails, add this belief as a constraint
        delete aug_final;
        addEdgeConstraint(out_edge, belief_node);
      }
    }
  }
}

template<class X, class P, class A>
bool BeliefTree<X, P, A>::addEdge(StateVertex * starting_vertex, StateVertex * ending_vertex,
    bool first_connection)
{
  //allocate a list to hold the path data
  Edge * new_edge = new Edge(starting_vertex, ending_vertex);

  bool valid_path;
  if (first_connection)
    valid_path = dynamic_system->steerApprox(starting_vertex->x, ending_vertex->x, new_edge->path_data,
        prop_steer_lcmgl,
        this->animate_usleep);
  else
    valid_path = dynamic_system->steer(starting_vertex->x, ending_vertex->x, new_edge->path_data, prop_steer_lcmgl,
        this->animate_usleep);

  if (valid_path) {
    //if a belief connection is required, we need to propagate as well
    if (first_connection) {
      double cost;
      A cov_final;
      typename std::list<BeliefNode *>::iterator belief_it;
      for (belief_it = starting_vertex->belief_nodes.begin(); belief_it != starting_vertex->belief_nodes.end();
          belief_it++) {
        if (dynamic_system->propagate(new_edge->path_data, *(*belief_it)->aug, cov_final, &cost, prop_steer_lcmgl,
            this->animate_usleep)) {
          goto success;
        }
        else {
          addEdgeConstraint(new_edge, *belief_it);
        }
        //TODO: save some computation by adding constraints to the edge for each belief that fails to propagate
      }
      goto failure;
    }
    else {
      goto success;
    }
  }
  else {
    goto failure;
  }

  success: insertEdge(starting_vertex, new_edge);
  this->lcmgl_print(this->getNumStateVertices(), true);
  return true;

  failure: delete new_edge;
  return false;
}

template<class X, class P, class A>
void BeliefTree<X, P, A>::insertEdge(StateVertex * starting_vertex, Edge * new_edge)
{
  starting_vertex->out_edges.push_back(new_edge);

  //iterate through beliefs at starting vertex and queue them
  typename std::list<BeliefNode *>::iterator belief_it;
  for (belief_it = starting_vertex->belief_nodes.begin(); belief_it != starting_vertex->belief_nodes.end();
      belief_it++) {
    pushSearchQueue(*belief_it, new_edge);
  }

}

template<class X, class P, class A>
void BeliefTree<X, P, A>::reconstructPath(const BeliefNode * end_belief, std::list<P *> & constructed_path)
{
  //reverse iterator for copying paths into the constructed path
  typename std::list<P *>::const_reverse_iterator path_it;

  //loop through the belief nodes on the path
  while (end_belief->parent_belief != NULL) {
    //copy the path segment between beliefs into the constructed path
    const std::list<P *> * path_segment = &end_belief->to_edge->path_data;
    for (path_it = path_segment->rbegin(); path_it != path_segment->rend(); path_it++) {
      constructed_path.push_front(*path_it);
    }
    end_belief = end_belief->parent_belief;
  }
}

template<class X, class P, class A>
BeliefNodeT<X, P, A> * BeliefTree<X, P, A>::popSearchQueue()
{
  BeliefNode * front_element;
  do {
    if (this->search_order == depth_first || this->search_order == breadth_first) {
      front_element = this->search_list.front();
      this->search_list.pop_front();
    }
    else if (this->search_order == uniform_cost) {
      front_element = this->search_queue.top();
      this->search_queue.pop();
    }
    front_element->on_queue = false;
    if (front_element->to_delete) {
      delete front_element;
      front_element = NULL;
    }
  } while (front_element == NULL && queueNotEmpty() > 0);
  return front_element;
}

template<class X, class P, class A>
void BeliefTree<X, P, A>::pushSearchQueue(BeliefNode * belief_node, Edge * new_edge)
{
  belief_node->on_queue = true;
  belief_node->new_edge = new_edge;
  if (this->search_order == depth_first)
    this->search_list.push_front(belief_node);
  else if (this->search_order == breadth_first)
    this->search_list.push_back(belief_node);
  else if (this->search_order == uniform_cost)
    this->search_queue.push(belief_node);
}

template<class X, class P, class A>
void BeliefTree<X, P, A>::iterateRRBT()
{
  //start the iteration by sampling
  X new_state;
  this->dynamic_system->sample(new_state);

  StateVertex * new_state_vertex = new StateVertex(new_state, this->dynamic_system->onStateGoal(new_state));

  //get the near and nearest
  std::list<StateVertex *> near_state_vertices = std::list<StateVertex *>();
  StateVertex * nearest_state_vertex = near(new_state_vertex, near_state_vertices);

  this->lcmgl_drawNearNearestNew(near_state_vertices, nearest_state_vertex, new_state_vertex);

  //add an edge from nearest to new, requiring that a belief make the connection
  if (addEdge(nearest_state_vertex, new_state_vertex, true)) {
    //add the sampled state to the owning list
    this->addVertex(new_state_vertex);

    if (RRBT_mode) { //if we're just doing RRT, dont' add any more edges
      //add the rest of the necessary edges
      addEdge(new_state_vertex, nearest_state_vertex);
      typename std::list<StateVertex *>::iterator near_it;
      for (near_it = near_state_vertices.begin(); near_it != near_state_vertices.end(); near_it++) {
        if (*near_it == nearest_state_vertex)
          continue;
        addEdge(*near_it, new_state_vertex);
        addEdge(new_state_vertex, *near_it);
      }
    }

    BeliefNode * popped_belief;
    //exhaust the search queue
    while (queueNotEmpty()) {
      popped_belief = popSearchQueue();
      expandBelief(popped_belief);
      this->lcmgl_print(this->getNumStateVertices(), true);

    }

    this->lcmgl_print(this->getNumStateVertices(), false);

  }
  else { //if the connection to nearest failed, remove the state vertex from the owning list
    delete new_state_vertex;
  }
}

template<class X, class P, class A>
void BeliefTree<X, P, A>::addVertex(StateVertex * new_state_vertex)
{
  this->num_state_vertices++;
  this->state_vertices.push_front(new_state_vertex);
}

template<class X, class P, class A>
StateVertexT<X, P, A> * BeliefTree<X, P, A>::near(StateVertex * new_vertex, std::list<StateVertex *> & near_vertices)
{
  double distance;
  double closest_distance = INFINITY;
  double ball_radius = getBallRadius();
  StateVertex * nearest_vertex_ptr;

  this->num_near_last = 0;

  //iterate through all the state vertices in belief tree
  typename std::list<StateVertex *>::iterator state_vertex_it;
  for (state_vertex_it = this->state_vertices.begin(); state_vertex_it != this->state_vertices.end();
      state_vertex_it++) {
    StateVertex * cur_state = *state_vertex_it;

    //don't compare new_vertex to self
    if (cur_state == new_vertex) {
      continue;
    }

    //compute distance
    distance = this->dynamic_system->distance(new_vertex->x, cur_state->x);

    //near
    if (distance < ball_radius) {
      near_vertices.push_front(cur_state);
      this->num_near_last++;
    }

    //nearest
    if (distance < closest_distance) {
      nearest_vertex_ptr = cur_state;
      closest_distance = distance;
    }

  }
  //  std::cout << "n = " << this->getNumStateVertices() << ", r = " << ball_radius << ", |near| = "
  //      << near_vertices.size() << "\n";
  return nearest_vertex_ptr;
}

template<class X, class P, class A>
void BeliefTree<X, P, A>::setMinBeliefAtGoal()
{
  typename std::list<StateVertex *>::iterator vert_it;
  for (vert_it = state_vertices.begin(); vert_it != state_vertices.end(); vert_it++) {
    StateVertex * vert = *vert_it;
    if (vert->on_goal) {
      typename std::list<BeliefNode *>::iterator bel_it;
      for (bel_it = vert->belief_nodes.begin(); bel_it != vert->belief_nodes.end(); bel_it++) {
        BeliefNode * bel = *bel_it;
        if (this->min_belief_at_goal == NULL) {
          if (this->dynamic_system->onAugGoal(vert->x, *bel->aug)) {
            this->min_belief_at_goal = bel;
            found_goal = true;
          }
        }
        else if (dynamic_system->compareAugTotal(*bel->aug, *min_belief_at_goal->aug, 0)
            && this->dynamic_system->onAugGoal(vert->x, *bel->aug)) {
          this->min_belief_at_goal = bel;
        }
      }
    }
  }
}

template<class X, class P, class A>
void BeliefTree<X, P, A>::lcmgl_print(int iteration_number, bool intermediate_draw)
{
  if (draw_mode == bt_no_visualization || draw_mode == bt_draw_at_end)
    return;

  if ((iteration_number % draw_mod) != 0)
    return;
  if (intermediate_draw && draw_mode == bt_draw)
    return;

  //optimal path
  BeliefNode * cur_opt_bel = this->min_belief_at_goal;
  if (cur_opt_bel != NULL) {
    initDepthTest(this->lcmgl_objs[optimal_path_lcmgl]);
    bot_lcmgl_line_width(this->lcmgl_objs[optimal_path_lcmgl], BT_OPTIMAL_PATH_WIDTH);
    bot_lcmgl_color3f(this->lcmgl_objs[optimal_path_lcmgl], BT_OPTIMAL_PATH_COLOR);

    //    while (cur_opt_bel->parent_belief!=NULL) {
    //      dynamic_system->lcmgl_aug_path(this->lcmgl_objs[optimal_path_lcmgl], cur_opt_bel->to_edge->path_data, *cur_opt_bel->parent_belief->aug, *cur_opt_bel->aug);
    //      cur_opt_bel = cur_opt_bel->parent_belief;
    //    }

    std::list<P *> opt_path;
    this->getOptPath(opt_path);
    bot_lcmgl_line_width(this->lcmgl_objs[optimal_path_lcmgl], BT_OPTIMAL_AUG_WIDTH);
    dynamic_system->lcmgl_aug_along_path(this->lcmgl_objs[optimal_path_lcmgl], opt_path,
        this->dynamic_system->getAugInit());
    endDepthTest(this->lcmgl_objs[optimal_path_lcmgl]);
    bot_lcmgl_switch_buffer(this->lcmgl_objs[optimal_path_lcmgl]);
  }
  else
    bot_lcmgl_switch_buffer(this->lcmgl_objs[optimal_path_lcmgl]);

  typename std::list<StateVertex *>::const_iterator state_it;
  typename std::list<BeliefNode *>::const_iterator bel1_it;
  typename std::list<BeliefNode *>::const_iterator bel2_it;
  typename std::list<Edge *>::const_iterator edge_it;
  const StateVertex * state_vert;
  const BeliefNode * bel1;
  const BeliefNode * bel2;
  const Edge * edge;

  //belief parents
  initDepthTest(this->lcmgl_objs[belief_parents_lcmgl]);
  bot_lcmgl_line_width(this->lcmgl_objs[belief_parents_lcmgl], BT_BELIEF_PARENT_WIDTH);
  bot_lcmgl_color3f(this->lcmgl_objs[belief_parents_lcmgl], BT_BELIEF_PARENT_COLOR);
  for (state_it = state_vertices.begin(); state_it != state_vertices.end(); state_it++) {
    state_vert = *state_it;
    for (bel1_it = state_vert->belief_nodes.begin(); bel1_it != state_vert->belief_nodes.end(); bel1_it++) {
      bel1 = *bel1_it;
      if (bel1->parent_belief == NULL
        )
        continue;

      dynamic_system->lcmgl_aug_path(this->lcmgl_objs[belief_parents_lcmgl], bel1->to_edge->path_data,
          *bel1->parent_belief->aug, *bel1->aug);
    }
  }
  endDepthTest(this->lcmgl_objs[belief_parents_lcmgl]);
  bot_lcmgl_switch_buffer(this->lcmgl_objs[belief_parents_lcmgl]);

  if (this->getNumStateVertices() > 1000) {
    bot_lcmgl_switch_buffer(this->lcmgl_objs[belief_children_lcmgl]);
    bot_lcmgl_switch_buffer(this->lcmgl_objs[edges_lcmgl]);
    bot_lcmgl_switch_buffer(this->lcmgl_objs[belief_nodes_lcmgl]);
    bot_lcmgl_switch_buffer(this->lcmgl_objs[state_vertices_lcmgl]);
    return;
  }

  //state vertices
  initDepthTest(this->lcmgl_objs[state_vertices_lcmgl]);
  bot_lcmgl_point_size(this->lcmgl_objs[state_vertices_lcmgl], BT_VERTEX_SIZE);
  bot_lcmgl_color3f(this->lcmgl_objs[state_vertices_lcmgl], BT_VERTEX_COLOR);
  for (state_it = state_vertices.begin(); state_it != state_vertices.end(); state_it++) {
    state_vert = *state_it;
    dynamic_system->lcmgl_state(this->lcmgl_objs[state_vertices_lcmgl], state_vert->x);
  }
  endDepthTest(this->lcmgl_objs[state_vertices_lcmgl]);
  bot_lcmgl_switch_buffer(this->lcmgl_objs[state_vertices_lcmgl]);

  //belief children
  initDepthTest(this->lcmgl_objs[belief_children_lcmgl]);
  bot_lcmgl_line_width(this->lcmgl_objs[belief_children_lcmgl], BT_BELIEF_CHILD_WIDTH);
  bot_lcmgl_color3f(this->lcmgl_objs[belief_children_lcmgl], BT_BELIEF_CHILD_COLOR);
  //    bot_lcmgl_color4f(this->lcmgl_objs[belief_children_lcmgl], .5,.5,.5,5);
  for (state_it = state_vertices.begin(); state_it != state_vertices.end(); state_it++) {
    state_vert = *state_it;
    for (bel1_it = state_vert->belief_nodes.begin(); bel1_it != state_vert->belief_nodes.end(); bel1_it++) {
      bel1 = *bel1_it;
      for (bel2_it = bel1->children_beliefs.begin(); bel2_it != bel1->children_beliefs.end(); bel2_it++) {
        bel2 = *bel2_it;
        dynamic_system->lcmgl_aug_path(this->lcmgl_objs[belief_children_lcmgl], bel2->to_edge->path_data, *bel1->aug,
            *bel2->aug);

      }
    }
  }
  endDepthTest(this->lcmgl_objs[belief_children_lcmgl]);
  bot_lcmgl_switch_buffer(this->lcmgl_objs[belief_children_lcmgl]);

  //edges
  initDepthTest(this->lcmgl_objs[edges_lcmgl]);
  bot_lcmgl_line_width(this->lcmgl_objs[edges_lcmgl], BT_EDGE_WIDTH);
  bot_lcmgl_color3f(this->lcmgl_objs[edges_lcmgl], BT_EDGE_COLOR);
  for (state_it = state_vertices.begin(); state_it != state_vertices.end(); state_it++) {
    state_vert = *state_it;
    for (edge_it = state_vert->out_edges.begin(); edge_it != state_vert->out_edges.end(); edge_it++) {
      edge = *edge_it;
      dynamic_system->lcmgl_path(this->lcmgl_objs[edges_lcmgl], edge->path_data);
    }
  }
  endDepthTest(this->lcmgl_objs[edges_lcmgl]);
  bot_lcmgl_switch_buffer(this->lcmgl_objs[edges_lcmgl]);

  //belief nodes
  initDepthTest(this->lcmgl_objs[belief_nodes_lcmgl]);
  bot_lcmgl_line_width(this->lcmgl_objs[belief_nodes_lcmgl], BT_BELIEF_NODE_WIDTH);
  bot_lcmgl_color3f(this->lcmgl_objs[belief_nodes_lcmgl], BT_BELIEF_NODE_COLOR);
  for (state_it = state_vertices.begin(); state_it != state_vertices.end(); state_it++) {
    state_vert = *state_it;
    for (bel1_it = state_vert->belief_nodes.begin(); bel1_it != state_vert->belief_nodes.end(); bel1_it++) {
      bel1 = *bel1_it;
      dynamic_system->lcmgl_aug(this->lcmgl_objs[belief_nodes_lcmgl], state_vert->x, *bel1->aug);
    }
  }
  endDepthTest(this->lcmgl_objs[belief_nodes_lcmgl]);
  bot_lcmgl_switch_buffer(this->lcmgl_objs[belief_nodes_lcmgl]);
}

template<class X, class P, class A>
void BeliefTree<X, P, A>::lcmgl_drawNearNearestNew(const std::list<StateVertex *> & near_states,
    const StateVertex * nearest_state, const StateVertex * new_state)
{

  if (draw_mode != bt_animate)
    return;

  typename std::list<StateVertex *>::const_iterator state_it;
  const StateVertex * state_vert;

  bot_lcmgl_color3f(this->lcmgl_objs[near_lcmgl], BT_NEAR_COLOR);
  for (state_it = near_states.begin(); state_it != near_states.end(); state_it++) {
    state_vert = *state_it;
    dynamic_system->lcmgl_state(this->lcmgl_objs[near_lcmgl], state_vert->x);
  }
  bot_lcmgl_switch_buffer(this->lcmgl_objs[near_lcmgl]);

  //nearest vertices
  bot_lcmgl_point_size(this->lcmgl_objs[nearest_lcmgl], BT_NEAREST_SIZE);
  bot_lcmgl_color3f(this->lcmgl_objs[nearest_lcmgl], BT_NEAREST_COLOR);
  dynamic_system->lcmgl_state(this->lcmgl_objs[nearest_lcmgl], nearest_state->x);
  bot_lcmgl_switch_buffer(this->lcmgl_objs[nearest_lcmgl]);

  //new vertex
  bot_lcmgl_point_size(this->lcmgl_objs[new_sample_lcmgl], BT_NEW_SAMPLE_SIZE);
  bot_lcmgl_color3f(this->lcmgl_objs[new_sample_lcmgl], BT_NEW_SAMPLE_COLOR);
  dynamic_system->lcmgl_state(this->lcmgl_objs[new_sample_lcmgl], new_state->x);
  bot_lcmgl_switch_buffer(this->lcmgl_objs[new_sample_lcmgl]);
}

/*
 * uses a kd-tree on the first M dimensions of the state vector for nearest and near queries
 */
template<int N, class P, class A, int M = N>
class BeliefTreeKD: public BeliefTree<Eigen::Matrix<double, N, 1>, P, A> {
public:
  typedef Eigen::Matrix<double, N, 1> X;
  typedef BeliefNodeT<X, P, A> BeliefNode;
  typedef StateVertexT<X, P, A> StateVertex;
  typedef EdgeT<X, P, A> Edge;

protected:
  kdtree_t * kdtree;
  void addVertex(StateVertex * new_state_vertex);
  StateVertex * near(StateVertex * new_vertex, std::list<StateVertex *> & near_vertices);

public:

  BeliefTreeKD(AugSystemInterface<X, P, A> * dynamic_system_arg, double ball_radius_constant, double max_ball_radius,
  double epsilon, int draw_mod, bt_visualization_t draw_animate, int animate_usleep, bool RRBT_mode
      , search_t search_order) :
      BeliefTree<Eigen::Matrix<double, N, 1>, P, A>(dynamic_system_arg, ball_radius_constant, max_ball_radius, epsilon,
          draw_mod, draw_animate, animate_usleep, RRBT_mode, search_order)
  {
    this->kdtree = kd_create(M);
    double pos[N];
    for (int ii = 0; ii < N; ii++) {
      pos[ii] = this->root_state_vertex->x(ii, 0);
    }
    kd_insert(this->kdtree, pos, (void *) this->root_state_vertex);
  }

  ~BeliefTreeKD()
  {
    kd_free(this->kdtree);
  }
};

template<int N, class P, class A, int M>
StateVertexT<Eigen::Matrix<double, N, 1>, P, A> * BeliefTreeKD<N, P, A, M>::near(StateVertex * new_vertex, std::list<
    StateVertex *> & near_vertices)
{
  double pos[N];
  for (int ii = 0; ii < N; ii++) {
    pos[ii] = new_vertex->x(ii, 0);
  }

  double rho = this->getBallRadius();

  kdres_t * near_set = kd_nearest_range(this->kdtree, pos, rho);
  this->num_near_last = kd_res_size(near_set);
  if (kd_res_size(near_set) > 0) {
    while (kd_res_next(near_set)) {
      near_vertices.push_back((StateVertex *) kd_res_item_data(near_set));
    }
  }
  kd_res_free(near_set);

  kdres_t * res_nearest = kd_nearest(this->kdtree, pos);
  assert(kd_res_size(res_nearest) > 0);
  StateVertex * nearest_node = (StateVertex *) kd_res_item_data(res_nearest);
  kd_res_free(res_nearest);
  return nearest_node;
}

template<int N, class P, class A, int M>
void BeliefTreeKD<N, P, A, M>::addVertex(StateVertex * new_state_vertex)
{
  double pos[M];
  for (int ii = 0; ii < M; ii++) {
    pos[ii] = new_state_vertex->x(ii, 0);
  }
  kd_insert(this->kdtree, pos, (void *) new_state_vertex);

  this->num_state_vertices++;
  this->state_vertices.push_front(new_state_vertex);
}

#endif

