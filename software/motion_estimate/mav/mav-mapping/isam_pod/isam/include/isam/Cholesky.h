/**
 * @file Cholesky.h
 * @brief Cholesky batch factorization using SuiteSparse by Tim Davis.
 * @author Michael Kaess
 * @version $Id: Cholesky.h 2839 2010-08-20 14:11:11Z kaess $
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

#include "SparseSystem.h"

namespace isam {

class Cholesky {
public:
  virtual ~Cholesky() {}

  virtual void factorize(const SparseSystem& Ab, Vector* delta = NULL) = 0;
  virtual void get_R(SparseSystem& R) = 0;
  virtual int* get_order() = 0;

  static Cholesky* Create();
protected:
  Cholesky() {}
};

}
