#ifndef _GRAPH_HPP
#define _GRAPH_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <map>
#include <vector>

#define MAX_NODES 100
#define MAX_EDGES 1000

class Graph 
{
    int nodes_;
    int edges_;
    bool matrix[MAX_NODES][MAX_NODES] = {0};
    std::map<int, std::list<int> > adjacencyList_;
    std::vector<bool> visited_;
    std::map<int, std::list<int> > connectedElements_;

    void vectorInit();

    void addEdgeToList(int, int);
    void addSingularNodes(std::map<int, std::list<int>>&);

    void displayMatrix(int, bool[][MAX_NODES]);
    void displayAdjacencyList(std::map<int, std::list<int> >);
    void displayConnectedElements(std::map<int, std::list<int>>&);

    void dfsElement(int, int);

public:
    void dfs();
    void readFromFile();
};

#endif  // _GRAPH_HPP