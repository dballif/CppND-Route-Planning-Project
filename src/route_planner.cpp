#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
  	start_node = &m_Model.FindClosestNode(start_x,start_y);
	end_node = &m_Model.FindClosestNode(end_x,end_y);
}


//Calculate the H value (the distance between the end node and the node passed to this function.
//this: in this case is the end_node of RoutePlanner stored
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*(this->end_node)); 
}


//AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	//Populate the neighbors for the current node
  	current_node->FindNeighbors();
  	//Do the steps for each neighbor
  	for(auto &node:current_node->neighbors){
  		//Set the parent of each neighbor to the current_node
      	node->parent = current_node;
      	//Calculate the h_value for each neighbor
      	node->h_value = CalculateHValue(node);
      	//Calculate the g_value for each neighbor
      	node->g_value = current_node->g_value + node->distance(*current_node);
      	//Set the visited value
      	node->visited=true;
      	//Add each neighbor to the open_list.
      	open_list.push_back(node);
  }
}


//NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
	//Sort the open_list by f_value (h_value + g_value) - Sorted in decreasing order
  	std::sort(open_list.begin(),open_list.end(),[](const auto &first_f, const auto &second_f){
      return(first_f->h_value + first_f->g_value > second_f->h_value + second_f->g_value);
      });
  	//Pointer *low_f points to the lowest f_value
	RouteModel::Node *low_f = open_list.back();

	//Remove the low_f from open_list
  	open_list.erase(open_list.end());
  
  	//Return the pointer
  	return low_f;
}

//Using the end_node as input, ConstructFinalPath from start_node to end_node
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

  	//Initialize paren
  	RouteModel::Node parent;
  
	//Iterate back through the parents
  	while(current_node->parent != nullptr){
      	//Add the current node to the path_found vector
    	path_found.push_back(*current_node);
      	//Define the parent node
      	parent = *(current_node->parent);
      	//Add the distance between the current_node and the parent_node
      	distance += current_node->distance(parent);
      	//Move to the parent node above
      	current_node = current_node->parent;
    }
  	//When the first node is reached, it will exit the while loop so add it hear
  	path_found.push_back(*current_node);
  
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  	//Reverse the vector to to get the correct order (start_node should be first in vector)
  	std::reverse(path_found.begin(),path_found.end());
  
    return path_found;

}

//A*Search
void RoutePlanner::AStarSearch() {
  	//Deal with the start_node
  	start_node->visited=true;
  	open_list.push_back(start_node);
  
    RouteModel::Node *current_node = nullptr;
  
  	//Iterate through all nodes until end_node (i.e. when current_node->distance to *end_node is 0)
  	while(open_list.size()>0){
    	current_node=NextNode();
      	
      	if(current_node->distance(*end_node) == 0){
          	//Store in final path in m_Model.path
        	m_Model.path = ConstructFinalPath(current_node);
          	return;
        }
      	else{
        	//If it's not the end node, keep going.
        	AddNeighbors(current_node);
        }
   }
}