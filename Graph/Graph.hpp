#ifndef _GRAPH_HPP
#define _GRAPH_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <map>
#include <vector>
#include <iomanip>
#include <limits> 
#include<algorithm>

#define MAX_NODES 100
#define MAX_EDGES 1000

class Graph 
{
    unsigned nodes_;
    unsigned edges_;
    uint16_t matrix[MAX_NODES][MAX_NODES] = {0};
    std::map<unsigned, std::list<unsigned> > adjacencyList_;
    std::vector<bool> visited_;
    std::map<unsigned, std::list<unsigned> > connectedElements_;

    std::vector<uint32_t> minDist_;

    void vectorInit();

    void readLoop(std::ifstream&, uint16_t(*)[MAX_NODES][MAX_NODES], const std::string&);

    void addEdgeToList(unsigned, unsigned);
    void addSingularNodes(std::map<unsigned, std::list<unsigned>>&);

    void displayMatrix(unsigned, uint16_t[][MAX_NODES]);
    void displayAdjacencyList(std::map<unsigned, std::list<unsigned> >);
    void displayConnectedElements(std::map<unsigned, std::list<unsigned>>&);

    void dfsElement(unsigned, unsigned);

    unsigned short minNodeDist();
    void dijkInit(unsigned short srcNode);
    void dijkDisplay(unsigned short srcNode);
public:
    void dfs();
    void dijkstra(unsigned short srcNode);
    void readFromFile(const std::string&);
};

#endif  // _GRAPH_HPP