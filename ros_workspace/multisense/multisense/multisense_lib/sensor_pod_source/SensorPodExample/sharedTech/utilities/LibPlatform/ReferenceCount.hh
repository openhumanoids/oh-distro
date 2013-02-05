/**
 * @file LibPlatform/ReferenceCount.hh
 *
 * Declares a ReferenceCount class for tracking resources.  This is
 * derived from Eric Kratzer's ReferenceCount class, which was
 * developed under project RD1034.
 *
 * Copyright 2011
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-08-14, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_LIBPLATFORM_REFERENCECOUNT_HH
#define CRL_LIBPLATFORM_REFERENCECOUNT_HH

#include <stdio.h>
#include <sys/time.h>
#include <stdint.h>

namespace crl {

  // A simple reference count class

  class ReferenceCount
  {
  public:
    
    bool isShared() {
      if (m_countP && (*m_countP) > 1)
        return true;
      return false;
    }

    void reset() {
      release();
      m_countP = new int32_t(1);
    }

    ReferenceCount() 
      : m_countP(new int32_t(1)) {};

    ReferenceCount(const ReferenceCount& source) 
      : m_countP(source.m_countP) {
      share();
    }

    ~ReferenceCount() {
      release();
    }

    ReferenceCount& operator=(const ReferenceCount& source) {
      if (this != &source) {
        release();
        m_countP = source.m_countP;
        share();
      }
      return *this;
    }                  

  private:

    int32_t *m_countP;
        
    void share() {
      if (m_countP)
        ++(*m_countP);
    }

    void release() {
      if (m_countP) {
        --(*m_countP);
        if ((*m_countP) <= 0)
          delete m_countP;
        m_countP = NULL;
      }
    }
  };

} // namespace crl

#endif /* #ifndef CRL_LIBPLATFORM_REFERENCECOUNT_HH */
