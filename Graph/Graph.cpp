#include "Graph.hpp"

#define INF std::numeric_limits<uint32_t>::max()

// initialize all 
void Graph::init()
{
    std::cout << "======== initialization =========" << std::endl;
    visited_.resize(nodes_ + 1, false);
    std::memset(matrix, 0, sizeof(matrix));
    adjacencyList_.clear();
    std::fill(visited_.begin(), visited_.end(), false);
    minDist_.resize(nodes_ + 1, INF);
}

void Graph::displayMatrix(unsigned nodes, uint16_t matrix[][MAX_NODES])
{
    std::cout << "========= Display adjacency matrix begins =========" << std::endl;
    for(auto i = 1; i <= nodes; ++i)
    {
        for (auto j = 1; j <= nodes; ++j)
        {
            std::cout << std::setw(3) << matrix[i][j];
        }
        std::cout << std::endl;
    }
    std::cout << "========= Display adjacency matrix ends =========" << std::endl;
}

void Graph::readFromFile(const std::string& fileName) 
{
    std::ifstream fin(fileName);

    if(!fin.is_open())
    {
        std::cout << "Could not open file " << fileName << std::endl;
        return;
    }

    // file is open here
    readLoop(fin, &matrix, "non-oriented");
    displayMatrix(nodes_, matrix);
    displayAdjacencyList(adjacencyList_);
}

void Graph::readLoop(std::ifstream& fin, uint16_t (*matrix)[MAX_NODES][MAX_NODES], const std::string& graphType)
{
    while(!fin.eof())
    {
        fin >> nodes_ >> edges_;
        if(nodes_ > MAX_NODES || edges_ > MAX_EDGES)
        {
            std::cout << "Invalid number of nodes_ or edges_" << std::endl;
        }
        else
        {
            unsigned source, destination, weight;
            for (auto i = 0; i < edges_; ++i)
            {
                fin >> source >> destination >> weight;
                if (graphType == "oriented") 
                {
                    (*matrix)[source][destination] = weight;  //asimetric matrix -- oriented graph
                }
                else 
                {
                    (*matrix)[source][destination] = weight;
                    (*matrix)[destination][source] = weight;  // simetric matrix
                    // set node in adjacencyList_(undirected graph)
                    addEdgeToList(source, destination);
                    addEdgeToList(destination, source);
                }
                
            }
            std::cout << "========= Read successful =========" << std::endl;
        }
    }
    fin.close();
}

void Graph::addEdgeToList(unsigned source, unsigned destination)
{
    adjacencyList_[source].push_back(destination);
}

void Graph::displayAdjacencyList(std::map<unsigned, std::list<unsigned> > adjacencyList)
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
void Graph::dfsElement(unsigned visited, unsigned times)
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
    unsigned times = 1;
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

void Graph::addSingularNodes(std::map<unsigned, std::list<unsigned>>& connectedElements)
{
    std::list<unsigned> singularNodes;
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

void Graph::displayConnectedElements(std::map<unsigned, std::list<unsigned>>& connectedElements)
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

// return the index of the vertex not visited
unsigned short Graph::minNodeDist()
{
    // initialization
    unsigned short index;
    uint32_t minimum = INF;

    for (auto i = 0; i < nodes_; ++i)
    {
        // only remember the vertexes that are not visited and the distance is less than the current distance
        // basically we can only check adjacent nodes
        if(minDist_[i] <= minimum && !visited_[i])
        {
            minimum = minDist_[i];
            index = i;
        }
    }
    return index;
}

// display minDist from source to all nodes
void Graph::dijkDisplay(unsigned short srcNode)
{
    std::cout << "======== dijkstra display begins =========" << std::endl;
    unsigned short i = 1;
    std::for_each(minDist_.begin() + 1, minDist_.end(), [&i, srcNode](uint32_t& dist) 
        {
            std::cout << "Distance from " << srcNode << " to " << i << " is " << dist << std::endl;
            ++i;
        });
    std::cout << "======== dijkstra display ends =========" << std::endl;
}

void Graph::dijkstra(unsigned short srcNode)
{
    minDist_[srcNode] = uint32_t(0);

    // start timer
    boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
    for (auto i = 1; i <= nodes_; ++i)
    {
        unsigned short minIndex = minNodeDist();
        visited_[minIndex] = true;
        for (auto j = 1; j <= nodes_; ++j)
        {
            // performing minimum distance update, if possible
            if (!visited_[j]
                && matrix[minIndex][j]
                && minDist_[minIndex] != INF
                && minDist_[minIndex] + matrix[minIndex][j] < minDist_[j])
            {
                minDist_[j] = minDist_[minIndex] + matrix[minIndex][j];
            }
        }
    }
    // stop timer
    boost::posix_time::ptime stop = boost::posix_time::microsec_clock::local_time();
    // calculate difference
    boost::posix_time::time_duration diff = stop - start;
    std::cout << "======== dijkstra executed in " << diff.total_nanoseconds() <<" ns=========" << std::endl;
    dijkDisplay(srcNode);
}

void Graph::prepFWMatrix()
{
    for (auto i = 1; i <= nodes_; ++i)
    {
        for (auto j = 1; j <= nodes_; ++j)
        {
            if (i != j && !matrix[i][j])
            {
                matrix[i][j] = std::numeric_limits<uint16_t>::max();
            }
        }
    }
}

// Implementing floyd warshall algorithm
void Graph::floydWarshall()
{
    prepFWMatrix();
    // start timer
    boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
    for (auto i = 1; i <= nodes_; i++)
        for (auto j = 1; j <= nodes_; j++)
            solutionMatrix[i][j] = matrix[i][j];
    // Adding vertices individually
    for (auto k = 1; k <= nodes_; k++) 
    {
        for (auto i = 1; i <= nodes_; i++) 
        {
            for (auto j = 1; j <= nodes_; j++) 
            {
                if (solutionMatrix[i][k] + solutionMatrix[k][j] < solutionMatrix[i][j])
                    solutionMatrix[i][j] = solutionMatrix[i][k] + solutionMatrix[k][j];
            }
        }
    }
    // stop timer
    boost::posix_time::ptime stop = boost::posix_time::microsec_clock::local_time();
    // calculate difference
    boost::posix_time::time_duration diff = stop - start;
    std::cout << "======== floyd warshall executed in " << diff.total_nanoseconds() <<" ns=========" << std::endl;
    printFWMatrix();
}

void Graph::printFWMatrix()
{
    for (auto i = 1; i <= nodes_; ++i) 
    {
        for (auto j = 1; j <= nodes_; ++j) 
        {
            if (solutionMatrix[i][j] == std::numeric_limits<uint16_t>::max())
                std::cout << std::setw(3) << "INF";
            else
                std::cout << std::setw(3) << solutionMatrix[i][j];
            }
        std::cout << std::endl;
    }
}

