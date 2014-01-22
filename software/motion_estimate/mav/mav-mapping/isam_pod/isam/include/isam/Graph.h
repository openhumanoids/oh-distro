/**
 * @file Graph.h
 * @brief Basic graph for iSAM.
 * @author Michael Kaess
 * @version $Id: Graph.h 2736 2010-08-04 20:24:05Z kaess $
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
#include <string>
#include <ostream>

#include "Node.h"
#include "Factor.h"

namespace isam {

class Graph {
  Graph(const Graph& rhs); // not allowed
  const Graph& operator= (const Graph& rhs); // not allowed
  std::list<Node*> _nodes;
  std::list<Factor*> _factors;
public:
  Graph() {}
  virtual ~Graph() {}
  virtual void add_node(Node* node) {
    _nodes.push_back(node);
  }
  virtual void add_factor(Factor* factor) {
    _factors.push_back(factor);
  }
  virtual void remove_node(Node* node) {
    _nodes.remove(node);
  }
  virtual void remove_factor(Factor* factor) {
    _factors.remove(factor);
  }
  const std::list<Node*>& get_nodes() const {return _nodes;}
  const std::list<Factor*>& get_factors() const {return _factors;}
  int num_nodes() const {return _nodes.size();}
  int num_factors() const {return _factors.size();}

  virtual void print_graph() const {
    printf("****GRAPH****:\n");
    printf("**NODES**:\n");
    for(std::list<Node*>::const_iterator it = _nodes.begin(); it!=_nodes.end(); it++) {
      (*it)->write(std::cout);
      printf("  Factors: ");
      std::list<Factor*> neighbors = (*it)->factors();
      for(std::list<Factor*>::iterator ite = neighbors.begin(); ite!=neighbors.end(); ite++) {
        printf("%i ", (*ite)->unique_id());
      }
      printf("\n");
    }
    printf("**FACTORS**:\n");
    for(std::list<Factor*>::const_iterator it = _factors.begin(); it!=_factors.end(); it++) {
      std::cout << (**it);
      printf("  Nodes: ");
      std::vector<Node*> neighbors = (*it)->nodes();
      for(std::vector<Node*>::iterator itn = neighbors.begin(); itn!=neighbors.end(); itn++) {
        printf("%i ", (*itn)->unique_id());
      }
      printf("\n");
    }
    printf("****END OF GRAPH****:\n");
  }

  virtual void write(std::ostream &out) const {
    for(std::list<Factor*>::const_iterator it = _factors.begin(); it!=_factors.end(); it++) {
      Factor& factor = **it;
      out << factor;
      out << "\n";
    }
    for(std::list<Node*>::const_iterator it = _nodes.begin(); it!=_nodes.end(); it++) {
      Node& node = **it;
      out << node;
      out << "\n";
    }
  }
};

}
