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

    // start timer
    std::cout << "======== fast dijkstra executed starts =========" << std::endl;
    boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();

    std::cout << graph.fastDisjkstra(1, 10) << std::endl;

    // stop timer
    boost::posix_time::ptime stop = boost::posix_time::microsec_clock::local_time();
    // calculate difference
    boost::posix_time::time_duration diff = stop - start;
    std::cout << "======== fast dijkstra executed in " << diff.total_nanoseconds() <<" ns=========" << std::endl;
    return 0;
}