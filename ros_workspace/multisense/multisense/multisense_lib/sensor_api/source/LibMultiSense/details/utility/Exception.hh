/**
 * @file LibMultiSense/details/utility/Exception.hh
 *
 * This header file is adapted from Eric Kratzer's (and Dan
 * Tascione's?) StandardException.h file, which was developed under
 * project RD1013.
 *
 * Copyright 2012
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
 *   2012-05-07, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_MULTISENSE_EXCEPTION_HH
#define CRL_MULTISENSE_EXCEPTION_HH

#include <stdio.h>
#include <string.h>
#include <exception>
#include <string>

#include "TimeStamp.hh"

#define CRL_FILENAME                            \
    (strrchr(__FILE__,'/')                      \
     ? strrchr(__FILE__,'/')+1                  \
     : __FILE__)

#define CRL_EXCEPTION(fmt, ...)                                         \
    do {                                                                \
        throw crl::multisense::details::utility::Exception("%s(%d): %s: "fmt,CRL_FILENAME,__LINE__, \
                                                           __PRETTY_FUNCTION__,##__VA_ARGS__); \
    } while(0)

#define CRL_DEBUG(fmt, ...)                                             \
    do {                                                                \
        double now = crl::multisense::details::utility::TimeStamp::getCurrentTime(); \
        fprintf(stderr, "[%.3f] %s(%d): %s: "fmt,now,CRL_FILENAME,__LINE__, \
                __PRETTY_FUNCTION__,##__VA_ARGS__);                     \
    } while(0)


namespace crl {
namespace multisense {
namespace details {
namespace utility {

class Exception : public std::exception
{
private:

    std::string reason;

public:
    
    Exception(const char *failureReason, ...);
    Exception(const std::string failureReason);
    ~Exception() throw();
    
    virtual const char* what() const throw();
};

}}}} // namespaces

#endif /* #ifndef CRL_MULTISENSE_EXCEPTION_HH */
