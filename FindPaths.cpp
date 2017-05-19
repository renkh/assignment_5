/******************************************************************************
 * Title          : FindPaths.cpp
 * Author         : Renat Khalikov
 * Created on     : May 18, 2017
 * Description    : main for part 2 of Assignment 5. Use Dijkstra’s Algorithm
 *                  to find the shortest paths from a given starting vertex to
 *                  all vertices in the graph file and print out the paths to
 *                  every destination along with its cost
 * Purpose        : Implement Dijkstra’s Algorithm, using a priority queue
 * Usage          : ./FindPaths Graph2.txt 1
 * Build with     : make all
 */
#include "Graph.h"
#include <iostream>
#include <fstream>
#include <sstream>  // istringstream
#include <cstdlib>  // exit(1) call
using namespace std;

/*
 * InsertVerticesIntoGraph( ) creates adjacency list from
 * given input file and calls FindTheShortestPaths( ) to perform Dijkstra’s
 * Algorithm
 *
 * @param {string} graph_file: input file that contains a graph
 *        to be inserted into adjacency_list
 * @param {int} starting_vertex: user input specifies what vertex to start
 *        Dijkstra’s Algorithm
 */
void InsertVerticesIntoGraph( const string &graph_file, const int &starting_vertex ) {
  ifstream input_filename(graph_file);
  if (input_filename.fail()) {
    cerr << "Could not open <adjecency_list_graph>\n";
    exit(1); // 1 indicates an error occurred
  }
  string line;  // for getline( )
  int total_number_of_vertices;
  int first_vertex;
  int connected_vertex;
  double weight;
  // first line contains total number of vertices in graph
  getline( input_filename, line );
  // convertion from string to int
  istringstream(line) >> total_number_of_vertices;
  // creates a vector of size total_number_of_vertices
  Graph a_graph(total_number_of_vertices+1);
  while( getline( input_filename, line ) ) {
    istringstream ss(line);  // convertion from string to int
    ss >> first_vertex;
    while ( ss >> connected_vertex >> weight ) {
      a_graph.Insert( first_vertex, connected_vertex, weight );
    }
  }
  input_filename.close();
  a_graph.FindTheShortestPaths(starting_vertex, cout);
}

// Sample main for program CreateGraphAndTest
int main(int argc, char **argv) {
  if (argc != 3) {
    cout << "Usage: " << argv[0] << " <GRAPH_FILE> <STARTING_VERTEX>" << endl;
    return 0;
  }
  
  const string graph_file(argv[1]);
  const string starting_vertex(argv[2]);
  // convert string to int
  int int_starting_vertex;
  istringstream(starting_vertex) >> int_starting_vertex;

  InsertVerticesIntoGraph( graph_file, int_starting_vertex );
  
  return 0;
}
