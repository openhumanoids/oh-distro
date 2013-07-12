/**
 * @file LibMultiSense/details/utility/ReferenceCount.hh
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
 * This software is free: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation,
 * version 3 of the License.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Significant history (date, user, job code, action):
 *   2012-08-14, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_MULTISENSE_REFERENCECOUNT_HH
#define CRL_MULTISENSE_REFERENCECOUNT_HH

#include <stdint.h>

namespace crl {
namespace multisense {
namespace details {
namespace utility {

class ReferenceCount
{
public:
    
    bool isShared() const {
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

    volatile int32_t *m_countP;
        
    void share() {
        if (m_countP) 
            __sync_fetch_and_add(m_countP, 1);
    }

    void release() {
        if (m_countP) {
            int32_t count = __sync_sub_and_fetch(m_countP, 1);
            if (count <= 0)
                delete m_countP;
            m_countP = NULL;
        }
    }
};

}}}} // namespaces

#endif /* #ifndef CRL_MULTISENSE_REFERENCECOUNT_HH */
