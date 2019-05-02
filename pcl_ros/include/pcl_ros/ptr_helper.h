//
//  ptr_helper.h
//  perception_pcl
//
//  Created by Andreas Klintberg on 5/1/19.
//  Copyright © 2019 Andreas Klintberg. All rights reserved.
//

#ifndef ptr_helper_h
#define ptr_helper_h

#include <boost/shared_ptr.hpp>
#include <memory>


/*https://stackoverflow.com/a/12605002/1829511*/

namespace {
  template<class SharedPointer> struct Holder {
    SharedPointer p;
    
    Holder(const SharedPointer &p) : p(p) {}
    Holder(const Holder &other) : p(other.p) {}
    Holder(Holder &&other) : p(std::move(other.p)) {}
    
    void operator () (...) { p.reset(); }
  };
}

template<class T> std::shared_ptr<T> to_std_ptr(const boost::shared_ptr<T> &p) {
  typedef Holder<std::shared_ptr<T>> H;
  if(H *h = boost::get_deleter<H>(p)) {
    return h->p;
  } else {
    return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
  }
}

template<class T> boost::shared_ptr<T> to_boost_ptr(const std::shared_ptr<T> &p){
  typedef Holder<boost::shared_ptr<T>> H;
  if(H * h = std::get_deleter<H>(p)) {
    return h->p;
  } else {
    return boost::shared_ptr<T>(p.get(), Holder<std::shared_ptr<T>>(p));
  }
}

#endif /* ptr_helper_h */
