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
#include <algorithm>
#include <cstring>
#include <set>
#include <boost/date_time.hpp>

#define MAX_NODES 100
#define MAX_EDGES 1000

class Graph 
{
    void readLoop(std::ifstream&, uint16_t(*)[MAX_NODES][MAX_NODES], const std::string&);

    void addEdgeToList(unsigned, unsigned);
    void addWeightedEdgeToList(unsigned, unsigned, uint32_t);
    void addSingularNodes(std::map<unsigned, std::list<unsigned>>&);

    void displayMatrix(unsigned, uint16_t[][MAX_NODES]);
    void displayAdjacencyList(std::map<unsigned, std::list<unsigned> >);
    void displayWeightedAdjacencyList(std::map<unsigned, std::list< std::pair<unsigned, uint32_t > > >);
    void displayConnectedElements(std::map<unsigned, std::list<unsigned>>&);

    void dfsElement(unsigned, unsigned);

    unsigned short minNodeDist();
    void dijkDisplay(unsigned short srcNode);

    void prepFWMatrix();
    void printFWMatrix();

    unsigned nodes_;
    unsigned edges_;
    uint16_t matrix[MAX_NODES][MAX_NODES] = {0};
    uint32_t solutionMatrix[MAX_NODES][MAX_NODES];
    std::map<unsigned, std::list<unsigned> > adjacencyList_;
    std::vector<bool> visited_;
    std::map<unsigned, std::list<unsigned> > connectedElements_;
    std::vector<uint32_t> minDist_;
    std::map<unsigned, std::list< std::pair < unsigned, uint32_t > > > adjacencyListWeighted_;
    std::set< std::pair <unsigned, uint32_t> > activeNodes_;
public:
    void dfs();
    void init();
    void dijkstra(unsigned short srcNode);
    void floydWarshall();
    uint32_t fastDisjkstra(unsigned short srcNode, unsigned short targetNode);
    void readFromFile(const std::string&);
};

#endif  // _GRAPH_HPP