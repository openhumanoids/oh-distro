/**
 * @file Element.h
 * @brief Basic functionality for nodes and factors of a graph.
 * @author Michael Kaess
 * @version $Id: Element.h 5796 2011-12-07 00:39:30Z kaess $
 *
 * [insert iSAM license]
 *
 */

#pragma once

#include <list>
#include <ostream>

namespace isam {

class Element {
  Element(const Element& rhs); // not allowed
  const Element& operator= (const Element& rhs); // not allowed

  const char* _name;

  int _start; // needed for Slam::jacobian

protected:
  int _id;
  int _dim;

public:
  Element(const char* name, int dim) : _name(name), _dim(dim) {}
  virtual ~Element() {};

  virtual int unique_id() {return _id;}
  virtual const char* name() const {return _name;}
  inline int dim() const {return _dim;}
  inline int start() const {return _start;}

  virtual void write(std::ostream &out) const {
    out << name();
  }

  friend class Slam;
  friend class Covariances;
};

}
