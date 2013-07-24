#include "keyboard_signal_utils.hpp"
#include "SelectionManager.hpp"

namespace visualization_utils
{
  typedef boost::shared_ptr<InteractableGlKinematicBody> ItemPtr;
  typedef boost::weak_ptr<InteractableGlKinematicBody> ItemWeakPtr;
  
 // A class than manages a list of weak ptrs to interactable objects in the renderer memory 
 // space. Wraps some common utility methods that 
 // i) returns shortest distance to a set of interactable objects give a mouse ray for use within 
 //    the pick query method in viewer renderers. Pick query method enables auctioning for mouse 
 //    based selection events among various renderers to resolve conflicts.
 // ii) methods that adjust object current or future state on marker motion
 
    class InteractionManager 
    {
      public:
        InteractionManager(KeyboardSignalRef signalRef):_plane_height(10)
        {
           _selectionManager = boost::shared_ptr<SelectionManager>(new SelectionManager(signalRef));
        };
        
        InteractionManager(KeyboardSignalRef signalRef,double plane_height):_plane_height(plane_height)
        {
           _selectionManager = boost::shared_ptr<SelectionManager>(new SelectionManager(signalRef));
        };      
          
        ~InteractionManager(){  };

        
        //access and management methods
        //---------------
        
        void addItem(ItemWeakPtr &item)
        {
          _items.push_back(item);
        };

        void clear()
        {
          _items.clear();
        };

        void autoclean()
        {
          std::vector<ItemWeakPtr>::iterator iter = _items.begin();
          while(iter != _items.end())
          {
             if ((*iter).expired())
             {
                iter = _items.erase(iter);
             }
             else
             {
                //ItemPtr item = (*iter).lock(); // create a shared_ptr from the weak_ptr
                //dosomething();
                ++iter;
             }
          }  
        };

        void update_params(Eigen::Vector3f &ray_start,Eigen::Vector3f &ray_dir);
        
         // utility methods
         //---------------
         // utility method for pick query method in viewer renderers
         // that enables auctioning for mouse based selection events.
         // returns the shortest distance to all active items, their 
         // corresponding body-pose and jointdof markers if they are active
         double get_shortest_distance(Eigen::Vector3f &ray_start,Eigen::Vector3f &ray_dir);
         // calls set_state() of InteractableGlKinematicBody
         void adjust_state_on_marker_motion();
         // calls set_state() of InteractableGlKinematicBody
         void adjust_future_state_on_marker_motion();
        
     
        private:
        //public members
        std::vector<ItemWeakPtr> _items;
        boost::shared_ptr<SelectionManager> _selectionManager;
        
        Eigen::Vector3f _ray_start,_ray_end,_ray_hit,_ray_hit_drag,_prev_ray_hit_drag;
        double _ray_hit_t;
        double _plane_height;
      
       
    };
}; // end namespace
