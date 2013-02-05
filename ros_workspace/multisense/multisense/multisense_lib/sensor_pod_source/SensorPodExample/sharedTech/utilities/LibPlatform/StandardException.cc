/**
 * @file LibUtilities/StandardException.cc
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
 * Significant history (date, user, job code, action):
 *   2012-05-07, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#include "StandardException.hh"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

namespace crl {

    /**
     * Constructor. Initializes with the reason given.
     *
     * \param failureReason The reason for the exception.
     */
    StandardException::StandardException(const char *failureReason, ...)
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

    StandardException::StandardException(const std::string failureReason)
    {
        reason = failureReason;
    }

    /**
     * Destructor. Empty.
     */
    StandardException::~StandardException() throw()
    {
        // Empty.
    }

    /**
     * Returns the reason for the exception.
     */
    const char* StandardException::what() const throw()
    {
        return this->reason.c_str();
    }

} // namespace crl
