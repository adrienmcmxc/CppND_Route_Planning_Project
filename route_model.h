#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

// The RouteModel class inherits from the Model class, which is provided by the IO2D OSM example
class RouteModel : public Model {

  public:
    //Subclass Node which inherits from the Model struct, also called Node 
    //The model struct Node provided X and Y coordinates and we extend that struct by adding:
    class Node : public Model::Node {
      public:
        Node * parent = nullptr; //A pointer to the parent of the node
        float h_value = std::numeric_limits<float>::max(); // H value
        float g_value = 0.0; // G value
        bool visited = false; //A boolean flag called visited, which will be useful for AStarSearch
        std::vector<Node *> neighbors; //A vector of node pointers. Each node has this vector of neighbors, and 
        //that will be populated with the neighbors of the node.  

        void FindNeighbors(); //Method which will populte above vector once it is called.
        
        float distance(Node other) const { //Method to calculate the distance from this node to any other node
            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }

        Node(){} //Default constructor
        //A constructor that accept an index, a route model, and an existing model node.
        //We can use this to construct  a route model node from an existing model node.
        //At the right theres a constructor list to initialize all of these variables 
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

      private:
        int index; //A private variable "index"
        Node * FindNeighbor(std::vector<int> node_indices); //Private function which is used in the FindNeighbors function
        RouteModel * parent_model = nullptr; //A pointer to the route model, to which this node belongs
    };

    RouteModel(const std::vector<std::byte> &xml);
    Node &FindClosestNode(float x, float y);
    auto &SNodes() { return m_Nodes; }
    std::vector<Node> path;
    
  private:
    void CreateNodeToRoadHashmap();
    std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;
    std::vector<Node> m_Nodes;

};

#endif
