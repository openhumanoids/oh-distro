/**
 * @file LibPlatform/Units.hh
 *
 * Functions for converting between physical units.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-08-14, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_LIBPLATFORM_UNITS_HH
#define CRL_LIBPLATFORM_UNITS_HH

#include <LibPlatform/Constants.hh>

namespace crl {

  template <class Type>
  inline Type radiansToDegrees(Type const& value) {
    return ((value) * (180.0 / crl::constants::pi));
  }
  
  template <class Type>
  inline Type degreesToRadians(Type const& value) {
    return ((value) * (crl::constants::pi / 180.0));
  }

} // namespace crl

#endif /* #ifndef CRL_LIBPLATFORM_UNITS_HH */
