
#if defined(_MSC_VER)
#pragma warning(disable : 4996)
#endif

#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/graphviz.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <boost/config.hpp>
#include <vector>
#include <algorithm>
#include <utility>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graph_traits.hpp>

#define MIN(x,y) (((x) < (y))?(x):(y))
#define MAX(x,y) (((x) > (y))?(x):(y))

#define DATATYPEIN unsigned char
#define MAX_NUM_EDGE 28885000  //2885000, 85000  //why 90000 causes crash?
#define MAXNumBranch 10 

using namespace std;
using namespace boost;

typedef adjacency_list <vecS, vecS, undirectedS, no_property, property <edge_weight_t, float> >  Graph;
typedef graph_traits < Graph >::edge_descriptor  Edge;
typedef graph_traits < Graph >::vertex_descriptor  Vertex;
typedef std::pair<int, int>  E;

typedef graph_traits < Graph >::edge_iterator Edge_iter;  
typedef graph_traits < Graph >::vertex_iterator Vertex_iter;

struct  VoxelPosition
{
	float x;
	float y;
	float z;
};

//void morphGraphPrune(int *degree_nodes, Graph msTree, int num_nodes, struct VoxelPosition *vertexPos);
Graph morphGraphPrune(Graph msTree, int num_nodes, struct VoxelPosition *vertexPos, float length_leaf);
