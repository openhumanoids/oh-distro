// testing_colortree

// test code taken from /home/snobili/main-distro/software/externals/octomap/octomap/src/testing.
// this code ha been modified to test the search function (neighborhood search).
// refer to original code to test colors and probabilities.

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include "testing.h"

using namespace std;
using namespace octomap;


void print_query_info(point3d query, ColorOcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
    cout << "color of node is: " << node->getColor()
         << endl;    
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;  
}


int main(int argc, char** argv) {

  double res = 0.05;  // create empty tree with resolution 0.05 (different from default 0.1 for test)
  ColorOcTree tree (res);
  // insert some measurements of occupied cells
  for (int x=-3; x<4; x++) {
    for (int y=-3; y<4; y++) {
      for (int z=-3; z<4; z++) {
        point3d endpoint ((float) x*0.05f+0.01f, (float) y*0.05f+0.01f, (float) z*0.05f+0.01f);
        ColorOcTreeNode* n = tree.updateNode(endpoint, true); 
        //n->setColor(z*5+100,x*5+100,y*5+100);
        n->setColor(255,0,0); // set color to red
      }
    }
  }
  
  OcTreeKey key = tree.coordToKey(point3d(-0.125,-0.125,-0.125)); 

  OcTreeKey neighborkey = key;
  point3d p = tree.keyToCoord(neighborkey);
  ColorOcTreeNode* n = tree.search(neighborkey);
  n->setColor(255,255,0); // set color to yellow

  int counter = 0;
  /*
  for(ColorOcTree::tree_iterator it=tree.begin_tree(),
      end=tree.end_tree(); it!= end; ++it) {
    if (it.isLeaf()) {
      OcTreeKey k = it.getKey();
      cout << "key: " << k[0] << "," << k[1] << "," << k[2] << endl;
      point3d coord = tree.keyToCoord(k);
      cout << "coord: " << coord << endl;
    }
  }*/

  OcTreeKey tmp_key;
  for (int i = 1; i < 4; i++)
  {
    for (int j = 1; j < 4; j++)
    {
      for (int k = 1; k < 4; k++)
      {
        tmp_key[0] = neighborkey[0] + i; // neighbor in pos. x-direction
        //cout << "neighborkey[0]: " << tmp_key[0] << endl;
        tmp_key[1] = neighborkey[1] + j; // neighbor in pos. y-direction
        //cout << "neighborkey[1]: " << tmp_key[1] << endl;
        tmp_key[2] = neighborkey[2] + k; // neighbor in pos. z-direction
        //cout << "neighborkey[2]: " << tmp_key[2] << endl;
        ColorOcTreeNode* result = tree.search(tmp_key);
        if (result != NULL){
          counter++;
          result->setColor(0,0,255);  // set color to yellow
        }
      }
    }
  }

  cout << "Counter: " << counter << endl;

  // insert some measurements of free cells
  for (int x=-30; x<30; x++) {
    for (int y=-30; y<30; y++) {
      for (int z=-30; z<30; z++) {
        point3d endpoint ((float) x*0.02f+2.0f, (float) y*0.02f+2.0f, (float) z*0.02f+2.0f);
        ColorOcTreeNode* n = tree.updateNode(endpoint, false); 
        n->setColor(255,255,0); // set color to yellow
      }
    }
  }

  // set inner node colors
  tree.updateInnerOccupancy();

  cout << endl;


  std::string filename ("simple_color_tree.ot");
  std::cout << "Writing color tree to " << filename << std::endl;
  // write color tree
  EXPECT_TRUE(tree.write(filename));


  // read tree file
  cout << "Reading color tree from "<< filename <<"\n";
  AbstractOcTree* read_tree = AbstractOcTree::read(filename);
  EXPECT_TRUE(read_tree);
  EXPECT_EQ(read_tree->getTreeType().compare(tree.getTreeType()), 0);
  EXPECT_FLOAT_EQ(read_tree->getResolution(), tree.getResolution());
  EXPECT_EQ(read_tree->size(), tree.size());
  ColorOcTree* read_color_tree = dynamic_cast<ColorOcTree*>(read_tree);
  EXPECT_TRUE(read_color_tree);


  cout << "Performing some queries:" << endl;
  
  {
    point3d query (0., 0., 0.);
    ColorOcTreeNode* result1 = tree.search (query);
    ColorOcTreeNode* result2 = read_color_tree->search (query);
    std::cout << "READ: ";
    print_query_info(query, result1);
    std::cout << "WRITE: ";
    print_query_info(query, result2);
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_EQ(result1->getColor(), result2->getColor());
    EXPECT_EQ(result1->getLogOdds(), result2->getLogOdds());
    
    /*
    query = point3d(-1.,-1.,-1.);
    result1 = tree.search (query);
    result2 = read_color_tree->search (query);
    print_query_info(query, result1);
    std::cout << "READ: ";
    print_query_info(query, result1);
    std::cout << "WRITE: ";
    print_query_info(query, result2);
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_EQ(result1->getColor(), result2->getColor());
    EXPECT_EQ(result1->getLogOdds(), result2->getLogOdds());
    
    query = point3d(1.,1.,1.);
    result1 = tree.search (query);
    result2 = read_color_tree->search (query);
    print_query_info(query, result1);
    std::cout << "READ: ";
    print_query_info(query, result1);
    std::cout << "WRITE: ";
    print_query_info(query, result2);
    EXPECT_FALSE(result1);
    EXPECT_FALSE(result2);
    */
  }

  delete read_tree;

  return 0;
}
