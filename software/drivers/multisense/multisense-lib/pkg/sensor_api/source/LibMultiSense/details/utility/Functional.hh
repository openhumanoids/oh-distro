/**
 * @file LibMultiSense/details/utility/Functional.hh
 *
 * Declarations of convenience functions and functors.
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
 *   2012-08-14, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_MULTISENSE_FUNCTIONAL_HH
#define CRL_MULTISENSE_FUNCTIONAL_HH

namespace crl {
namespace multisense {
namespace details {
namespace utility {

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

}}}} // namespaces

#endif /* #ifndef CRL_MULTISENSE_FUNCTIONAL_HH */
