/**
 * @file LibUtilities/StandardException.hh
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
 * Significant history (date, user, job code, action):
 *   2012-05-07, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_LIBPLATFORM_STANDARDEXCEPTION_HH
#define CRL_LIBPLATFORM_STANDARDEXCEPTION_HH

#include <stdio.h>
#include <string.h>
#include <exception>
#include <string>

#include <LibPlatform/TimeStamp.hh>

#define CRL_FILENAME                            \
    (strrchr(__FILE__,'/')                      \
     ? strrchr(__FILE__,'/')+1                  \
     : __FILE__)

#define CRL_STANDARD_EXCEPTION(fmt, ...)                                      \
    do {                                                                      \
        throw crl::StandardException("%s(%d): %s: "fmt,CRL_FILENAME,__LINE__, \
                                     __PRETTY_FUNCTION__,##__VA_ARGS__);      \
    } while(0)

#define CRL_STANDARD_DEBUG(fmt, ...)                                          \
    do {                                                                      \
    double now = crl::TimeStamp::getCurrentTime();                            \
        fprintf(stderr, "[%.3f] %s(%d): %s: "fmt,now,CRL_FILENAME,__LINE__,   \
                __PRETTY_FUNCTION__,##__VA_ARGS__);                           \
    } while(0)


#ifndef CRL_NO_BACKWARDS_COMPATIBILITY_MACROS
#define STANDARD_EXCEPTION(fmt, ...) CRL_STANDARD_EXCEPTION(fmt, ##__VA_ARGS__)
#define STANDARD_DEBUG(fmt, ...)     CRL_STANDARD_DEBUG    (fmt, ##__VA_ARGS__)
#endif /* #ifndef CRL_NO_BACKWARDS_COMPATIBILITY_MACROS */

namespace crl {

    // The defined exception class.

    class StandardException : public std::exception
    {
    private:

        std::string reason;

    public:

        StandardException(const char *failureReason, ...);
        StandardException(const std::string failureReason);
        ~StandardException() throw();

        virtual const char* what() const throw();
    };

} // namespace crl

#endif /* #ifndef CRL_LIBPLATFORM_STANDARDEXCEPTION_HH */
