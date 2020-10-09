#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"

//RoutePlanner class definition
class RoutePlanner {
  public:
    //RoutePlanner method, which takes as an input the model, and the start and end coordinates
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    //GetDistance method, which returns the distance, stored down below as a private variable in the model
    float GetDistance() const {return distance;} //That distance will be used to store the complete distance of the found path
    //Declaration of the AStarSearch method
    void AStarSearch();

    // The following methods have been made public so we can test them individually.
    //These methods will all be used to conduct the AStar Search
    void AddNeighbors(RouteModel::Node *current_node);
    float CalculateHValue(RouteModel::Node const *node);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    RouteModel::Node *NextNode();

  private:
    // Add private variables or methods declarations here.
    // Private variables for the open list
    std::vector<RouteModel::Node*> open_list; // Vector of node pointers for the open list 
    RouteModel::Node *start_node; // A pointer to the start node
    RouteModel::Node *end_node; // A pointer to the end node

    float distance = 0.0f; 
    RouteModel &m_Model;

};

#endif