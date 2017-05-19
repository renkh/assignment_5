/******************************************************************************
 * Title          : TopologicalSort.cpp
 * Author         : Renat Khalikov
 * Created on     : May 18, 2017
 * Description    : main for part 3 of Assignment 5. Write a program that
 *                  computes a sequence of vertices that satisfy the
 *                  topological sorting sequence
 * Purpose        : implement the topological sorting algorithm
 * Usage          : ./TopologicalSort Graph3.txt
 * Build with     : make all
 */
#include "Graph.h"
#include <iostream>
#include <fstream>
#include <sstream>  // istringstream
#include <cstdlib>  // exit(1) call
using namespace std;

/*
 * InsertVerticesIntoGraph( ) creates adjacency list from given input file and
 * performs TopologicalSort( )
 *
 * @param {string} graph_file: input file that contains a graph
 *        to be inserted into adjacency_list
 */
void InsertVerticesIntoGraph( const string &graph_file ) {
  ifstream input_filename(graph_file);
  if (input_filename.fail()) {
    cerr << "Could not open <GRAPH_FILE>\n";
    exit(1); // 1 indicates an error occurred
  }
  string line;  // for getline( )
  int total_number_of_vertices;
  int first_vertex;
  int connected_vertex;
  // first line contains total number of vertices in graph
  getline( input_filename, line );
  // convertion from string to int
  istringstream(line) >> total_number_of_vertices;
  // creates a vector of size total_number_of_vertices
  Graph a_graph(total_number_of_vertices+1);
  // insert vertices into vector
  while( getline( input_filename, line ) ) {
    istringstream ss(line);  // convertion from string to int
    while ( ss >> first_vertex >> connected_vertex ) {
      if (connected_vertex != 0)
      {
        a_graph.Insert( first_vertex, connected_vertex );
        connected_vertex = 0;  // input file contains trailing int
                               // if while reaches the end of the line
                               // ignore the trailing int
      }
    }
  }
  input_filename.close();
  a_graph.TopologicalSort();
  a_graph.PrintTopologicalSort(cout);
}

// Sample main for program CreateGraphAndTest
int main(int argc, char **argv) {
  if (argc != 2) {
    cout << "Usage: " << argv[0] << " <GRAPH_FILE>" << endl;
    return 0;
  }
  
  const string graph_file(argv[1]);

  InsertVerticesIntoGraph( graph_file );
  
  return 0;
}
