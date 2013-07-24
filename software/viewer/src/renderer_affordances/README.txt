Some notes on CODE DESIGN STRUCTURE and FUNCTION of the Affordance renderer
---------------------------------------------------------------
FUNCTION
=========
The affordance renderer's primary function is to render otdf object instances and other features that are specified in the object frame.

The renderer currently performances the following functions
 1) Listens to `affordance_state_t' and `affordance_collection_t' msgs, creates object instances and renders them.
 
 2) Can create and manage its own cache of object instances fused with the affordance msgs coming in from outside. The cache is synchronized with the affordance store process. In case the viewer crashes, we won't loose the affordances. 
    
 3) Maintains a local cache of list of sticky hands and feet that are associated with objects. 

STRUCTURE
==========
----------------------------------------------------------------------------------
Host-Parasite model 
----------------------------------------------------------------------------------
Description:
------------
An attempt at managing complexity as we create instances of objects and associate many `stuff/features'(parasites) that are stuck to it. Example `Parasites' could include sticky hands, sticky feet or sticky manipulation plans that are defined in object frame. As an object is tracked in world frame, all the dependent parasites get rendered with it.

Notes:
------
- Parent objects are agnostic about parasites that are stuck to it.
- If a object is deleted, a search through a global parasite list is performed and all parasites that have the deleted object as their parent are erased. This search is inefficient, but it is worth for the clean separation it offers as there is no information about parasites stored in the object struct.
- If a parasite is deleted, then only the parasite list needs to be updated.
- In this structure, the object structure wont grow in complexity. If someone, does not want the sticky something feature. Then it is relatively easy to remove it from the code.
- This also allows for separate affordance store and parasite feature store processes. So if one of them fails, we won't loose everything.


Sisir
Created: 8th January 2013
Last Updated: 14th July 2013
