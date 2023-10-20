/***********************************************************
 *
 * @file: nodes.h
 * @breif: Contains common/commonly used nodes data strcutre
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef COMMON_NODES_H
#define COMMON_NODES_H

namespace ipp_tools
{
namespace common
{
/**
 * @brief Basic Node class templated to work with different
 *       configuration spaces.
 * @details      
 *       The Xn represents the configuration type of the node (normaly Affine2D or Affine3D)
 *       CAREFUL: This implementation only works for undirected, acyclic 
 *       and single parent graphs (https://stackoverflow.com/questions/27348396/smart-pointers-for-graph-representation-vertex-neighbors-in-c11)
 *       - In case of more parents, use a vector of shared pointers
 *       - In case of cyclic graphs, use weak pointers to avoid memory leaks
 *       - In case of directed graphs, the implementation should be more customed
 * 
 */
template <typename Xn>
struct Node
{
  /**
   * @brief Constructor for Node class
   * @param _x   configuration x in generic space
   * @param _v   generic value of the node
   * @param _g   g value, cost to get to this node
   * @param _h   h value, heuritic cost of this node
   * @param _id  node's id
   * @param _p   parent's reference
   */
    Node(Xn _x, double _v, double _g, double _h, int _id, const Node<Xn>* _p)
        : x(_x), v(_v), g(_g), h(_h), id(_id), parent(_p) {}

    Xn x;               // configuration x in generic space
    double v;           // generic value of the node
    double g;           // g value, cost to get to this node
    double h;           // h value, heuritic cost of this node
    int id;             // node's id
    const Node<Xn>* parent;   // parent's reference

};

} // namespace common
} // namespace ipp_tools

#endif  // COMMON_NODES_H