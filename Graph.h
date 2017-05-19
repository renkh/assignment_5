/******************************************************************************
 * Title          : Graph.h
 * Author         : Renat Khalikov
 * Created on     : May 18, 2017
 * Description    : implementation of adjacency list, Dijkstra, and
 *                  topological sorting
 * Purpose        : 
 * Usage          : 
 * Build with     : 
 */
#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <list>
#include <ostream>  // cout
#include <queue>

/**
 * Graph class
 * 
 * CONSTRUCTION: {int} vector_size: total number of nodes in the graph
 * ******************PUBLIC OPERATIONS*****************************************
 * void Insert( x,y,z )   : Insert vertex, connected vertex, and its weight
 *                          into adjacency list
 * void Insert( x,y )     : Insert vertex and connected vertex into adjacency
 *                          list
 * double Search( x,y )   : Finds connected vertex and returns its weight
 * void FindTheShortestPaths( x,ostream )    : Dijkstra's implementaton
 * void Print( x, ostream )                  : Prints Dijkstra
 * void TopologicalSort( x )                 : Topological sorting sequence
 * void ComputeInDegrees( )                  : Computes in_degree
 * void FindNewVertexOfIndegreeZero( x )     : Populates queue of in_degree 0
 * void PrintTopologicalSort( ostream )      : prints Topological sorting
 * int GetTotalNumberOfVertices( )           : returns total number of vertices
 * ******************ERRORS****************************************************
 * No errors :)
 */
class Graph
{
 public:
  /*
   * Constructor sets vector size
   *
   * @param {int} vector_size: number of vertices in graph
   */ 
  Graph( const int vector_size )
  {
    adjacency_list_.resize(vector_size);
    path_list_.resize(vector_size);
    in_degree_.resize(vector_size, 0);
    distance_.resize(vector_size, INFINITY);
    parent_.resize(vector_size, -1);
    number_of_vertices_ = vector_size;
  }

  ~Graph() = default;

  /*
   * Insert( ) inserts vectors into adjacency list
   *
   * @param {int} a_vertex: starting vector
   * @param {int} connected_vertex: vertex that is connected to a_vertex
   * @param {double} weight: connection weight
   */ 
  void Insert( int & a_vertex, int & connected_vertex, double & weight )
  { 
    adjacency_list_[a_vertex].push_back(std::make_pair(connected_vertex, weight)); 
  }

  /*
   * Insert( ) inserts vectors into adjacency list. This one is used for
   * Topological sorting implementaton
   *
   * @param {int} a_vertex: starting vector
   * @param {int} connected_vertex: vertex that is connected to a_vertex
   */ 
  void Insert( int & a_vertex, int & connected_vertex ) 
  { 
    path_list_[a_vertex].push_back(connected_vertex); 
  }

  /*
   * Search( ) looks for connected_vertex, returns the weight if connection
   * is found, otherwise returns -1
   *
   * @param {int} a_vertex: starting vector
   * @param {int} connected_vertex: vertex that is connected to a_vertex
   */ 
  double Search (const int &vertex, const int &connected_vertex)
  {
    std::list< std::pair<int, double> >::iterator itr = adjacency_list_[vertex].begin();
    while (itr != adjacency_list_[vertex].end()) {
      if (itr->first == connected_vertex)
        return itr->second;
      else
        ++itr;
    }
    return -1;
  }

  /*
   * FindTheShortestPaths( ) implements Dijkstra's algorithm by using priority
   * queue. 
   *
   * @param {int} start_vertex: starting vector
   * @param {ostream} stream: outputs to standard output
   */ 
  void FindTheShortestPaths(int start_vertex, std::ostream& stream)
  {
    // keep all vertices of shortest distance in a queue
    // start vertex of distance 0 is placed on an initially empty queue
    std::priority_queue< vertex_pair_, std::vector <vertex_pair_> , std::greater<vertex_pair_> > priority_q;
    priority_q.push(std::make_pair(0, start_vertex));
    distance_[start_vertex] = 0;

    // While the queue is not empty,
    // a vertex is removed, 
    // and all vertices adjacent to vertex have their distances checked
    while (!priority_q.empty())
    {
      int priority_vertex = priority_q.top().second;
      priority_q.pop();

      // and all vertices adjacent to vertex have their distances checked
      std::list< std::pair<int, double> >::iterator itr;
      for (itr = adjacency_list_[priority_vertex].begin(); itr != adjacency_list_[priority_vertex].end(); ++itr)
      {
        int vertex = itr->first;
        double weight = itr->second;

        // a vertex is put on the queue as soon as its distance is less than
        // current vertex
        // the shortest path is the order in which the vertices dequeue
        // add the dequeued vertex to parent_ vector
        if (distance_[vertex] > distance_[priority_vertex] + weight)
        {
          distance_[vertex] = distance_[priority_vertex] + weight;
          priority_q.push(std::make_pair(distance_[vertex], vertex));
          parent_[vertex] = priority_vertex;
        }
      }
    }

    // print the shortest path
    for (int i = 1; i < number_of_vertices_; i++) {
      stream << i << ": " << start_vertex;
      Print(i, stream);
      stream << " (Cost: " << distance_[i] << ")" << std::endl;
    }
  }

  /*
   * Print( ) helper function to FindTheShortestPaths( ) prints connected
   * vertices, used to print the shortest path.
   *
   * @param {ostream} stream: outputs to standard output
   */ 
  void Print(int vertex, std::ostream& stream)
  {
    if (parent_[vertex]==-1)
      return;
    Print(parent_[vertex], stream);
    stream << ", " << vertex;
  }

  /*
   * TopologicalSort( ) Topological sorting implementaton by using priority
   * queue. The topological ordering is the order in which the vertices dequeue
   */ 
  void TopologicalSort( )
  {
    // compute the indegrees of all vertices in the graph.
    ComputeInDegrees( );

    // keep all (unassigned) vertices of indegree 0 in a queue
    // all vertices of indegree 0 are placed on an initially empty queue
    FindNewVertexOfIndegreeZero( );

    // While the queue is not empty,
    // a vertex is removed, 
    // and all vertices adjacent to vertex have their indegrees decremented
    while (!q_.empty())
    {
      int vertex = q_.front();
      q_.pop();
      // the topological ordering then is the order in which the vertices dequeue.
      topological_.push_back(vertex);
      std::list<int>::iterator itr;
      for (itr = path_list_[vertex].begin(); itr != path_list_[vertex].end(); itr++) {
        // all vertices adjacent to v have their indegrees decremented
        // a vertex is put on the queue as soon as its indegree falls to 0
        if (--in_degree_[*itr] == 0)
            q_.push(*itr);
      }
      topological_counter_++;
    }
  }

  /*
   * ComputeInDegrees( ) Topological sorting implementaton. Computes in_degree
   * of each node and stores it in in_degree_ vector.
   */ 
  void ComputeInDegrees( )
  {
    for (int i = 0; i < number_of_vertices_; i++) {
      std::list<int>::iterator itr;
      for (itr = path_list_[i].begin(); itr != path_list_[i].end(); itr++)
        in_degree_[*itr]++;
    }
  }

  /*
   * FindNewVertexOfIndegreeZero( ) Topological sorting implementaton. Finds
   * 0 indegree vertices and pushes them onto queue.
   */ 
  void FindNewVertexOfIndegreeZero( )
  {
    for (int i = 0; i < number_of_vertices_; i++)
      if (in_degree_[i] == 0)
        q_.push(i);
  }

  /*
   * PrintTopologicalSort( ) prints the result of Topological sorting
   * implementaton.
   *
   * @param {ostream} stream: outputs to standard output
   */ 
  void PrintTopologicalSort( std::ostream& stream )
  {
    // Print the topological ordering
    if (number_of_vertices_ == topological_counter_)
    {
      for (unsigned int i = 1; i < topological_.size(); i++)
        stream << topological_[i] << " ";
      stream << std::endl;
    }
    else
      stream << "Cycle found\n";
  }

  /*
   * GetTotalNumberOfVertices( ) returns total number of vertices
   */ 
  int GetTotalNumberOfVertices( )
  {
    return TotalNumberOfVertices( );
  }

 private:
  std::vector< std::list< std::pair<int, double> > > adjacency_list_;
  std::vector< std::list< int > > path_list_;
  typedef std::pair<int, double> vertex_pair_;
  std::vector<int> parent_;        // stores the shortest path
  std::vector<int> distance_;      // stores the shortest distance 
  std::vector<int> in_degree_;     // stores the in degree 
  std::vector <int> topological_;  // contains vertices in topological order
  std::queue<int> q_;              // stores vertices of in degree 0

  int INFINITY = 1000000000;
  int topological_counter_ = 0;
  int number_of_vertices_;
  
  /*
   * TotalNumberOfVertices( ) internal function returns total number of
   * vertices
   */ 
  int TotalNumberOfVertices( )
  {
    return number_of_vertices_;
  }
};

#endif
