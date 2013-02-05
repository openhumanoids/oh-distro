/**
 * @file LibPlatform/Functional.hh
 *
 * Declarations of convenience functions and functors.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-08-14, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_LIBPLATFORM_FUNCTIONAL_HH
#define CRL_LIBPLATFORM_FUNCTIONAL_HH

namespace crl {

  template <class Type>
  inline bool
  approximatelyEqual(Type const& xx, Type const& yy, Type const& epsilon) {
    return (((xx)-(yy) < (epsilon)) && ((xx)-(yy) > -(epsilon)));
  }

  template <class Type>
  inline bool
  approxEqual(Type const& xx, Type const& yy, Type const& epsilon) {
    return approximatelyEqual(xx, yy, epsilon);
  }

  template <class Type>
  inline Type
  boundValue(Type const& value, Type const& minimum, Type const& maximum) {
    return ((value > maximum) ? maximum : (value < minimum) ? minimum : value);
  }

  template <class Type>
  inline Type
  decayedAverage(Type const& previous, Type const& samples, Type const& newest) {
      return (((samples - 1) * previous) + newest) / samples;
  }

} // namespace crl

#endif /* #ifndef CRL_LIBPLATFORM_FUNCTIONAL_HH */
