/**
 * @file util.h
 * @brief Basic utility functions that are independent of iSAM.
 * @author Michael Kaess
 * @version $Id: util.h 2953 2010-09-07 21:43:43Z hordurj $
 *
 * Copyright (C) 2009-2010 Massachusetts Institute of Technology.
 * Michael Kaess, Hordur Johannsson and John J. Leonard
 *
 * This file is part of iSAM.
 *
 * iSAM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * iSAM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with iSAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <string>

namespace isam {

// some math constants
#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;
const double HALFPI = PI/2.0;

// values up to this constant are considered zero and removed from the matrix
const double NUMERICAL_ZERO = 1e-8;

/**
 * Return current system time in seconds.
 */
double tic();

/**
 * Remember and return system time in seconds.
 * @param id Name of time slot.
 */
double tic(std::string id);

/**
 * Return difference between current system time and t in seconds.
 * @param t0 Start time as returned by tic();
 */
double toc(double t0);

/**
 * Return difference between current system time and remembered time in
 * seconds, and add to accumulated time.
 * @param id Name of time slot.
 */
double toc(std::string id);

/**
 * Print a list of accumulated times and additional statistics
 * for each name used in tic/toc.
 */
void tictoc_print();

/**
 * Return the accumulated time.
 * @param id Name of time slot.
 */
double tictoc(std::string id);


/**
 * Calculate Givens rotation so that a specific entry becomes 0.
 * @param a Diagonal entry from above.
 * @param b Entry to be zeroed out.
 * @param c Resulting cosine part.
 * @param s Resulting sine part.
 */
void givens(const double a, const double b, double& c, double& s);

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

#ifdef NDEBUG
// remove all requirements (only slightly faster, not really worth it)
#define require(req,msg)
#else
// cause a crash to allow backtracing
#define require(req,msg) if (!(req)) {fputs(msg, stderr);fputs("\n\n", stderr); *((int*)NULL) = NULL; exit(1);}
#endif

}
