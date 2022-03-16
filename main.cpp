#include <iostream>
#include "Graph/Graph.hpp"

Graph graph = Graph();

int main(int argc, char** argv)
{
    graph.init();
    graph.readFromFile("in.txt");
    graph.dfs();

    graph.init();
    graph.readFromFile("in2.txt");
    graph.dijkstra(1);

    graph.floydWarshall();
    return 0;
}