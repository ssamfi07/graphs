#include <iostream>
#include "Graph/Graph.hpp"

Graph graph = Graph();

int main(int argc, char** argv)
{
    graph.readFromFile();
    graph.dfs();

    return 0;
}