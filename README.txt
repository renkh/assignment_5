Assignment 4
----------------
All parts of the assignment are complete.

----------------
Bugs
----------------
No bugs. 

----------------
Compilation Instructions
----------------
Open a terminal.
Go to the directory containing the source code.
To compile type:
  make all

It is assumed that you are using a Linux machine and a g++ compiler.

----------------
To execute:
----------------
For Part 1:
In source code directory type:
./CreateGraphAndTest <adjacency_list_graph> <adjancency_list_query_file>

<adjacency_list_graph> file containing vertices. Vertices will be inserted into adjacency list.

<adjancency_list_query_file> file containing vertices that you want to search in adjacency list

Note:
Files follow a format. See sample files for more details. 

Example: 

./CreateGraphAndTest Graph1.txt AdjacencyQueries1.txt


For Part 2:
In source code directory type:

./FindPaths <GRAPH_FILE> <STARTING_VERTEX>

<GRAPH_FILE> file containing vertices. Vertices will be inserted into adjacency list.

<STARTING_VERTEX> vertex {int} where Dijkstra’s algorithm starts.

Note:
Files follow a format. See sample files for more details. 

Example:

./FindPaths Graph2.txt 1


For Part 3:
In source code directory type:
./TopologicalSort <GRAPH>

<GRAPH> file containing vertices. Vertices will be inserted into adjacency list.

Note:
Files follow a format. See sample files for more details. 

Example: 

./TopologicalSort Graph3.txt


