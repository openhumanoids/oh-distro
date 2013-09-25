#ifndef __SharedPtrMacros_h
#define __SharedPtrMacros_h

#include <boost/shared_ptr.hpp>

#define SharedPtr boost::shared_ptr
#define WeakPtr boost::weak_ptr
#define PtrCast boost::dynamic_pointer_cast

#define ClassPtrMacro(className) \
  typedef SharedPtr<className> Ptr; \
  typedef const SharedPtr<className> ConstPtr;

#endif
