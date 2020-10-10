#include "route_planner.h"
#include <algorithm>

// Definition of the class constructor
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node); //Accessing the method 'distance' of the class 'Node' through a pointer
                                     //The argument of the function is the value pointed to by 'end_node'
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    //call FindNeighbors() method on the current_node
    current_node->FindNeighbors(); //this would equal to (*current_node).FindNeighbors()
                                   //Accessing the method 'FindNeighbors()' of the class 'Node' through a pointer
    
    
    //Populate the vector current_node.neighbors with current_node 's neighbors 
    //For each node in current_node.neighbors, set the parent, the h_value, the g_value

    for (auto neighbor : current_node->neighbors){ 
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        //g value is set as the g of the current node plus the distance from the current node to the neighbor
        neighbor->h_value = CalculateHValue(neighbor);
        open_list.push_back(neighbor); //adds the neighbor to the open list
        neighbor->visited = true; //sets the neighbor as visited
    }  

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    //Sort the open_list according to the f value (f = h + g), in descending order
    
    std::sort(open_list.begin(), open_list.end(),
        //lambda expression to define the comparison criteria (comp) 
        [](auto *i, auto *j){
            float f_i = i->h_value + i->g_value;
            float f_j = j->h_value + j->g_value;
            return f_i > f_j;
            }
        );
    //create a pointer to the node in the list with the lowest f value
    RouteModel::Node *next_node = open_list.back();
    open_list.pop_back(); //remove the last element from the list, which is already picked

    //Return the pointer
    return next_node; 
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.


/*This method will take the current (final) node as an argument and iteratively follow the chain of parents
   until the starting node is found*/
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Method variables
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while(current_node->parent != nullptr){ //looping until the parent of the current node reaches the starting point
        //sum the distance between the current node and its parent
        distance += current_node->parent->distance(*current_node); 
        //adding the current node to the end of the end of the path (ascending order)
        path_found.push_back(*current_node); 
        //updating the current node as a parent of itself 
        current_node = current_node->parent;
    }   
    path_found.push_back(*current_node); //The last node (start_node) was skipped in the while loop
                                         //thus it is added to the list.

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    
    //reversing the found path (end-start -> start-end)
    std::reverse(path_found.begin(), path_found.end());

    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr; //Initializing the current node as a null pointer

    start_node->visited = true;
    open_list.push_back(start_node);

    while(open_list.empty() == 0){ //loops until the list of open list is non-empty
            current_node = NextNode(); //Sorting the open list and returning the next node
            if(current_node == end_node){ //Check if the current node is the goal (i.e. end_node)
                m_Model.path = ConstructFinalPath(current_node); 
            }
    
    AddNeighbors(current_node); //Expanding the current node by adding all unvisited neighbors to the open list
    }

}