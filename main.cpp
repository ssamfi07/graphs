#include <iostream>
#include "Graph/Graph.hpp"

Graph graph = Graph();

int main(int argc, char** argv)
{
    graph.readFromFile("in.txt");
    graph.dfs();

    graph.readFromFile("in2.txt");
    graph.dijkstra(1);
    return 0;
}