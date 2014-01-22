/**
 * @file Node.h
 * @brief Graph node for iSAM.
 * @author Michael Kaess
 * @version $Id: Node.h 2885 2010-08-23 03:53:45Z kaess $
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

#include <list>

#include "Element.h"
#include "Vector.h"

namespace isam {

class Factor; // Factor.h not included here to avoid circular dependency

// Node of the graph also containing measurements (Factor).
class Node : public Element {
  friend std::ostream& operator<<(std::ostream& output, const Node& n) {
    n.write(output);
    return output;
  }

  int _start; // needed for Slam::jacobian after removing nodes
  static int _next_id;
protected:
  std::list<Factor*> _factors; // list of adjacent factors
  bool _initialized;
public:
  Node(const char* name, int dim) : Element(name, dim), _initialized(false) {
    _id = _next_id++;
  }
  virtual bool initialized() {return _initialized;}
  virtual ~Node() {};
  virtual Vector vector() const = 0;
  virtual Vector vector0() const = 0;
  virtual void update(const Vector& v) = 0;
  virtual void update0(const Vector& v) = 0;
  void add_factor(Factor* e) {_factors.push_back(e);}
  void remove_factor(Factor* e) {_factors.remove(e);}
  std::list<Factor*> factors() {return _factors;}

  virtual void write(std::ostream &out) const = 0;

  friend class Slam;
};

// Generic template for easy instantiation of the multiple node types.
template <class T>
class NodeT : public Node {
  T* _value;  // current estimate
  T* _value0; // linearization point
public:
  NodeT() : Node(T::name(), T::dim) {
    _value = NULL;
    _value0 = NULL;
  }
  virtual ~NodeT() {
    delete _value;
    delete _value0;
  }
  void init(const T& t) {_value = new T(t); _value0 = new T(t); _initialized = true;}
  T value() const {return *_value;}
  T value0() const {return *_value0;}
  Vector vector() const {return _value->vector();}
  Vector vector0() const {return _value0->vector();}
  void update(const Vector& v) {_value->set(v);}
  void update0(const Vector& v) {_value0->set(v);}

  void write(std::ostream &out) const {
    out << name() << "_Node " << _id;
    if (_value) {
      out << " " << value();
    }
  }
};

}
