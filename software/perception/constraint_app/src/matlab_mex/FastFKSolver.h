#ifndef FASTFKSOLVER_H
#define FASTFKSOLVER_H

#include <iostream>
#include <kdl/frames_io.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/utility.hpp>
#include <boost/property_map/property_map.hpp>

class FastFKSolver {
 public:
  class Data {
  public:
  Data(const KDL::TreeElement& _element) : name(_element.segment.getName()), 
      element(_element), frameValid(false) {}

    std::string name;
    KDL::TreeElement element;

    KDL::Frame frame;
    bool frameValid;
  };

  typedef boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS, Data> SegmentGraph;
  typedef boost::graph_traits<SegmentGraph>::vertex_descriptor SegmentGraphVertex;
  typedef boost::graph_traits<SegmentGraph>::out_edge_iterator SegmentGraphOutEdges;
  typedef std::map<std::string, SegmentGraphVertex> VertexMap;
  
 FastFKSolver(const KDL::Tree& tree) : m_tree(tree), 
    m_log("/tmp/cam2.log", std::ios::trunc|std::ios::binary)  {

    m_graph.clear();
    m_rootVertex = buildGraph(m_tree.getRootSegment());

    printGraph();
  }

  SegmentGraphVertex buildGraph(const KDL::SegmentMap::const_iterator thisSegment) {
    m_log << "buildGraph: adding node " << thisSegment->first 
	  << ", with " << thisSegment->second.children.size() << " children" << std::endl;
    m_log.flush();

    Data d(thisSegment->second);
    SegmentGraphVertex thisVertex = add_vertex(d, m_graph);
    
    m_map.insert(std::pair<std::string, SegmentGraphVertex>(d.name, thisVertex));

    for ( std::vector<KDL::SegmentMap::const_iterator>::const_iterator child = 
	    thisSegment->second.children.begin(); 
	  child != thisSegment->second.children.end(); 
	  ++child ) {
      SegmentGraphVertex childVertex = buildGraph(*child);
      add_edge(thisVertex, childVertex, m_graph);
    }
    
    return thisVertex;
  }

  void printGraph() {

    //loop through the verticies
    SegmentGraph::vertex_iterator iter, end;
    for ( boost::tie(iter, end) = vertices(m_graph); iter != end; ++iter ) { 
      SegmentGraphVertex thisVertex = *iter;
      m_log << m_graph[thisVertex].name << std::endl;

      //loop through all outgoing edges
      SegmentGraph::out_edge_iterator out_begin, out_end;
      for (boost::tie(out_begin, out_end) = out_edges(thisVertex, m_graph); 
	   out_begin != out_end; ++out_begin) {   
	SegmentGraphVertex t = target(*out_begin, m_graph);
	m_log << "   out:" << m_graph[t].name << std::endl;
      }

      //loop through all incoming edges
      SegmentGraph::in_edge_iterator in_begin, in_end;
      for (boost::tie(in_begin, in_end) = in_edges(thisVertex, m_graph); 
	   in_begin != in_end; ++in_begin) { 
	SegmentGraphVertex s = source(*in_begin, m_graph);
	m_log << "    in:" << m_graph[s].name << std::endl;
      }
      m_log << std::endl;
    }
  }

  bool getFrame(const std::string& segmentName, 
		KDL::Frame& frame) {
    VertexMap::const_iterator iter = m_map.find(segmentName);
    if ( iter == m_map.end() ) return false;

    //m_log << "getFrame for " << segmentName << std::endl;

    typedef std::list<SegmentGraphVertex> VertexList;
    VertexList pendingVertices;

    SegmentGraphVertex currentVertex = iter->second;    

    while ( !m_graph[currentVertex].frameValid ) {
      pendingVertices.push_back(currentVertex);

      SegmentGraph::in_edge_iterator in_begin, in_end;
      boost::tie(in_begin, in_end) = in_edges(currentVertex, m_graph);

      if ( in_begin == in_end ) {
	m_log << "ERROR: we reached the root vertex without it being known" << std::endl
	      << m_graph[currentVertex].name << " has no parents, but frameValid=false" << std::endl;
	return false;
      }

      currentVertex = source(*in_begin, m_graph);
    }

    //at this point, we know that m_graph[currentVertex].frame is correctly calculated
    //  and we can follow the kinematic chain starting at the back of pendingVertices
    frame = m_graph[currentVertex].frame;
    for ( VertexList::const_reverse_iterator iter = pendingVertices.rbegin();
	  iter != pendingVertices.rend(); ++iter ) {
      KDL::TreeElement& currentElement(m_graph[*iter].element);
      KDL::Frame currentFrame = currentElement.segment.pose(m_currentJoints(currentElement.q_nr));
      frame = frame * currentFrame;

      //m_log << "  calculating frame " << m_graph[*iter].name << std::endl;

      m_graph[*iter].frameValid = true;
      m_graph[*iter].frame = frame;
    }

    return true;

    /*
    SegmentGraphVertex currentVertex = iter->second;
    KDL::TreeElement currentElement = m_graph[currentVertex].element;
    KDL::Frame currentFrame = currentElement.segment.pose(m_currentJoints(currentElement.q_nr));

    //m_log << "starting with frame: " << m_graph[currentVertex].name << std::endl;
    frame = currentFrame;

    SegmentGraph::in_edge_iterator in_begin, in_end;
    boost::tie(in_begin, in_end) = in_edges(currentVertex, m_graph); 

    // follow the tree back up the incoming edges
    while ( in_begin != in_end ) {
      currentVertex = source(*in_begin, m_graph);
      currentElement = m_graph[currentVertex].element;
      currentFrame = currentElement.segment.pose(m_currentJoints(currentElement.q_nr));

      frame = currentFrame * frame;
      //m_log << "  linked from: " << m_graph[currentVertex].name << std::endl;

      boost::tie(in_begin, in_end) = in_edges(currentVertex, m_graph); 
    }
    
    return true;
    */
  }

  void setJointPositions(const KDL::JntArray& q_in) {
    m_currentJoints = q_in;

    //loop through the verticies and clear all the frames
    SegmentGraph::vertex_iterator iter, end;
    for ( boost::tie(iter, end) = vertices(m_graph); iter != end; ++iter ) { 
      SegmentGraphVertex thisVertex = *iter;
      m_graph[thisVertex].frameValid = false;
    }

    // by construction, every frame links to the rootVertex
    m_graph[m_rootVertex].frameValid = true;
    m_graph[m_rootVertex].frame = KDL::Frame::Identity();
  }

  KDL::JntArray m_currentJoints;
  KDL::Tree m_tree;
  SegmentGraph m_graph;
  VertexMap m_map;
  SegmentGraphVertex m_rootVertex;
  std::ofstream m_log;
};


#endif
