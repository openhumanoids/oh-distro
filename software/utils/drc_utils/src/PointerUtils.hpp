#ifndef _drc_PointerUtils_hpp_
#define _drc_PointerUtils_hpp_

#include <memory>
#include <boost/shared_ptr.hpp>

namespace drc {

struct PointerUtils {

  template<class SharedPointer>
  struct Holder {
    SharedPointer mPtr;

    Holder(const SharedPointer& iPtr) : mPtr(iPtr) {}
    Holder(const Holder& iHolder) : mPtr(iHolder.mPtr) {}
    Holder(Holder&& iHolder) : mPtr(std::move(iHolder.mPtr)) {}

    void operator() (...) const {}
  };

  template<class T>
  static std::shared_ptr<T>
  stdPtr(const boost::shared_ptr<T>& iPtr) {
    typedef Holder<std::shared_ptr<T>> H;
    if(H* h = boost::get_deleter<H,T>(iPtr)) {
      return h->mPtr;
    } else {
      return std::shared_ptr<T>(iPtr.get(), Holder<boost::shared_ptr<T>>(iPtr));
    }
  }

  template<class T>
  static boost::shared_ptr<T>
  boostPtr(const std::shared_ptr<T>& iPtr){
    typedef Holder<boost::shared_ptr<T>> H;
    if(H* h = std::get_deleter<H,T>(iPtr)) {
      return h->mPtr;
    } else {
      return boost::shared_ptr<T>(iPtr.get(), Holder<std::shared_ptr<T>>(iPtr));
    }
  }
};

}

#endif
