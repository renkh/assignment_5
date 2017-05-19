/******************************************************************************
 * Title          : CreateGraphAndTest.cpp
 * Author         : Renat Khalikov
 * Created on     : May 18, 2017
 * Description    : main for part 1 of Assignment 5. Create adjacency list by
 *                  reading it from text file Graph1.txt. Open the file
 *                  AdjacenyQueries.txt and for each pair of vertices cout
 *                  whether the vertices are adjacent or not, and if they are,
 *                  cout the weight of the edge that connects them
 * Purpose        : Represent a graph using adjacency list.
 * Usage          : ./CreateGraphAndTest Graph1.txt AdjacencyQueries1.txt
 * Build with     : make all
 */
#include "Graph.h"
#include <iostream>
#include <fstream>
#include <sstream>  // istringstream
#include <cstdlib>  // exit(1) call
using namespace std;

/*
 * SearchAndPrintVertices( ) opens query_filename, for each vertex in file
 * is searched in adjacency_list, if found prints vertex it is connected to
 * and its weight, if not found prints Not Connected
 *
 * @param {Graph} a_graph: contains adjacency_list 
 * @param {string} query_filename: file that contains vertices to search in
 *        adjacency_list
 */
void SearchAndPrintVertices(Graph &a_graph, const string &query_filename) {
  ifstream adjacency_query(query_filename);
  if (adjacency_query.fail()) {
    cerr << "Could not open <adjancency_list_query_file>\n";
    exit(1); // 1 indicates an error occurred
  }
  string line;
  int vertex;
  int connected_vertex;
  double weight;
  while(getline( adjacency_query, line )){
    istringstream(line) >> vertex >> connected_vertex;
    // Search( ) returns the weight between vertex and connected_vertex
    // weight of -1 indicates vertex is not connected to connected_vertex
    weight = a_graph.Search(vertex, connected_vertex);
    if (weight != -1)
      cout << vertex << " " << connected_vertex << ": " << "Connected, weight of edge is " << weight << endl;
    else
      cout << vertex << " " << connected_vertex << ": " << "Not Connected" << endl;
  }
}

/*
 * InsertVerticesIntoAdjacencyListAndSearch( ) creates adjacency list from
 * given input file and calls SearchAndPrintVertices( ) function that prints
 * the output
 *
 * @param {string} adjacency_list_filename: input file that contains a graph
 *        to be inserted into adjacency_list
 * @param {string} query_filename: file that contains vertices to search in
 *        adjacency_list
 */
void InsertVerticesIntoAdjacencyListAndSearch( const string &adjacency_list_filename, const string &query_filename ) {
  ifstream adjacency_filename(adjacency_list_filename);
  if (adjacency_filename.fail()) {
    cerr << "Could not open <adjecency_list_graph>\n";
    exit(1); // 1 indicates an error occurred
  }
  string line;  // for getline( )
  int total_number_of_vertices;
  int first_vertex;
  int connected_vertex;
  double weight;
  // first line contains total number of vertices in graph
  getline( adjacency_filename, line );
  // convertion from string to int
  istringstream(line) >> total_number_of_vertices;
  // creates a vector of size total_number_of_vertices
  Graph a_graph(total_number_of_vertices+1);
  // insert vertices into vector
  while( getline( adjacency_filename, line ) ) {
    istringstream ss(line);  // convertion from string to int
    ss >> first_vertex;
    while ( ss >> connected_vertex >> weight ) {
      a_graph.Insert( first_vertex, connected_vertex, weight );
    }
  }
  adjacency_filename.close();
  SearchAndPrintVertices(a_graph, query_filename);
}

// Sample main for program CreateGraphAndTest
int main(int argc, char **argv) {
  if (argc != 3) {
    cout << "Usage: " << argv[0] << " <adjacency_list_graph.txt> <adjancency_list_query_file.txt>" << endl;
    return 0;
  }
  
  const string adjacency_list_filename(argv[1]);
  const string query_filename(argv[2]);

  InsertVerticesIntoAdjacencyListAndSearch( adjacency_list_filename, query_filename );
  
  return 0;
}
