#ifndef NODE_S_H
#define NODE_S_H

#include <vector>

#include <theseus/map_s.h>
#include <theseus/fillet_s.h>

namespace theseus
{
// Class Definition
struct node                    // This is the node struct for each spot on the tree
{
  NED_s p;                     // North, East Down of the node position
  fillet_s fil;                // *fillet information for the PARENT
  std::vector<node*> children; // Vector of all of the children nodes
  node* parent;                // *Pointer to the parent of this node
  float cost;                  // Distance from this node to its parent
  bool dontConnect;            // true means closest nodes generated won't connect to this node
  bool connects2wp;            // true if this node connects to the next waypoint
  void equal(node* n)
  {
    p           = n->p;
    fil         = n->fil;
    children    = n->children;
    parent      = n->parent;
    cost        = n->cost;
    dontConnect = n->dontConnect;
    connects2wp = n->connects2wp;
  }
};
} // end namespace theseus

#endif
