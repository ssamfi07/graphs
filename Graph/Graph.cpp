#include "Graph.hpp"

// set visited vector sizes and initial values
// !!!IMPORTANT!!!  Vector size is nodes_ + 1 because we mark the NODES as visited (they start from 1)
void Graph::vectorInit()
{
    visited_.resize(nodes_ + 1, false);
}

void Graph::displayMatrix(int nodes, bool matrix[][MAX_NODES])
{
    std::cout << "========= Display adjacency matrix begins =========" << std::endl;
    for(auto i = 1; i <= nodes; ++i)
    {
        for (auto j = 1; j <= nodes; ++j)
        {
            std:: cout << matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "========= Display adjacency matrix ends =========" << std::endl;
}

void Graph::readFromFile() 
{
    std::string fileName("in.txt");
    std::ifstream fin(fileName);

    if(!fin.is_open())
    {
        std::cout << "Could not open file " << fileName << std::endl;
        return;
    }

    // file is open here

    while(!fin.eof())
    {
        fin >> nodes_ >> edges_;
        if(nodes_ > MAX_NODES || edges_ > MAX_EDGES)
        {
            std::cout << "Invalid number of nodes_ or edges_" << std::endl;
        }
        else
        {
            int source, destination;
            for (auto i = 0; i < edges_; ++i)
            {
                // read source and destination nodes_
                fin >> source >> destination;
                // set node in adjacencyList_(undirected graph)
                addEdgeToList(source, destination);
                addEdgeToList(destination, source);
                // set nodes_ in adiancence matrix
                matrix[source][destination] = matrix[destination][source] = 1;  // simetric matrix
            }
            std::cout << "========= Read successful =========" << std::endl;
        }
        
    }
    displayMatrix(nodes_, matrix);
    displayAdjacencyList(adjacencyList_);
    vectorInit();
    fin.close();
}

void Graph::addEdgeToList(int source, int destination)
{
    adjacencyList_[source].push_back(destination);
}

void Graph::displayAdjacencyList(std::map<int, std::list<int> > adjacencyList)
{
    std::cout << "========= Display adjacency list begins =========" << std::endl;
    for (auto const& mapElem : adjacencyList)
    {
        // display node
        std::cout << mapElem.first << ": ";
        // display node adjacency list
        for (auto const& listElem : mapElem.second)
        {
            std::cout << listElem << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "========= Display adjacency list ends =========" << std::endl;
}

// triggers depth-first traversal for the graph from a starting node
void Graph::dfsElement(int visited, int times)
{
    visited_[visited] = true;

    // add visited node to connectedElements_ map
    connectedElements_[times].push_back(visited);

    // go through the element's adjacency list
    for (auto listIt = adjacencyList_[visited].begin(); listIt != adjacencyList_[visited].end(); ++listIt)
    {
        if(!visited_[*listIt])
        {
            dfsElement(*listIt, times);
        }
    }
}

//triggers depth-first traversal for the graph by triggering dfsElement from each node (if not visited)
void Graph::dfs()
{
    int times = 1;
    for(auto const& listIt : adjacencyList_)
    {
        if(!visited_[listIt.first])
        {
            dfsElement(listIt.first, times);
            ++times;
        }
    }
    std::cout << "========= DFS worked =========" << std::endl;
    displayConnectedElements(connectedElements_);
}

void Graph::addSingularNodes(std::map<int, std::list<int>>& connectedElements)
{
    std::list<int> singularNodes;
    for (auto it = 1; it <= visited_.size(); ++it)
    {
        if(!visited_[it])
        {
            singularNodes.push_back(it);
            connectedElements.insert(std::make_pair(connectedElements_.size() + 1, singularNodes));
            singularNodes.clear();
        }
    }
}

void Graph::displayConnectedElements(std::map<int, std::list<int>>& connectedElements)
{
    addSingularNodes(connectedElements);
    std::cout << "========= Connected elements begin =========" << std::endl;
    for (auto const& mapElem : connectedElements)
    {
        // display node
        std::cout << mapElem.first << ": ";
        // display node adjacency list
        for (auto const& listElem : mapElem.second)
        {
            std::cout << listElem << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "========= Connected elements ends =========" << std::endl;
}