/**
 * @file LibMultiSense/details/utility/Exception.cc
 *
 * This header file is adapted from Eric Kratzer's (and Dan
 * Tascione's?) StandardException.cc file, which was developed under
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

#include "Exception.hh"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

namespace crl {
namespace multisense {
namespace details {
namespace utility {

/**
 * Constructor. Initializes with the reason given.
 *
 * \param failureReason The reason for the exception.
 */
Exception::Exception(const char *failureReason, ...)
{
    char   *stringP=NULL;
    va_list ap;
    int returnValue;

    va_start(ap, failureReason);
    returnValue = vasprintf(&stringP, failureReason, ap);
    va_end(ap);
    
    if ((NULL != stringP) && (returnValue != -1)) {
        reason = std::string(stringP);
        free(stringP);
    }
}

Exception::Exception(const std::string failureReason)
{
    reason = failureReason;
}

/**
 * Destructor. Empty.
 */
Exception::~Exception() throw()
{
    // Empty.
}

/**
 * Returns the reason for the exception.
 */
const char* Exception::what() const throw()
{
    return this->reason.c_str();
}

}}}} // namespaces
