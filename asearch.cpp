#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <iterator>
#include <limits>
#include <cmath>
#include <fstream>
#include <sstream>
#include <tuple>

namespace constants 
{
    const int infinity = std::numeric_limits<int>::max();
    const int negative = -1;
}

namespace helpers
{
    template<typename T, typename V>
    void remove(T & container, const V & v)
    {
        container.erase(std::remove(container.begin(), container.end(), v), container.end());
    }

    template<typename T, typename V>
    bool find(const T & container, const V & v)
    {
        return std::find(container.begin(), container.end(), v) != container.end();
    }

    template<typename T, typename V>
    int position(const T & container, const V & v)
    {
        return std::distance(container.begin(), std::find(container.begin(), container.end(), v));
    }
}

struct ValDefaultInf 
{
    int v = constants::infinity;
};

struct ValDefaultNeg
{
    int v = constants::negative;
};

enum class Heuristic
{
    OutOfPosition,
    SumOfDiff
};

template<typename T>
using Vec = std::vector<T>;

template<typename T>
using Vec2d = std::vector<std::vector<T>>;

template<typename T1, typename T2>
using Map = std::map<T1, T2>;

template<typename T>
std::ostream &operator <<(std::ostream &os, const Vec<T> &v)
{
   //std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
    int i = 1;
    for (const auto & e : v) 
    {
        os << e << " ";
        if (i++ % 3 == 0) os << "\n";
    }
    return os;
}

int heuristicOutOfPosition(const Vec<int> & start, const Vec<int> & goal) 
{
    int numbersOutPosition = 0;
    for (int i = 0; i < 9; ++i) 
    {
        if (start[i] != goal[i]) ++numbersOutPosition;
    }
    
    return numbersOutPosition;
}

int heuristicSumOfDiff(const Vec<int> & start, const Vec<int> & goal) 
{
    int sumDiff = 0;
    for (int i = 0; i < 9; ++i) 
    {
        sumDiff += std::abs(start[i] - goal[i]);
    }
    
    return sumDiff;
}

int heuristic(const Vec<int> & start, const Vec<int> & goal, Heuristic h)
{
    if (start.size() != goal.size() || start.size() != 9 || goal.size() != 9) 
    {
        return -1; // error , throw exception ?   
    }

    switch(h)
    {
        case Heuristic::OutOfPosition:
            return heuristicOutOfPosition(start, goal);
        case Heuristic::SumOfDiff:
            return heuristicSumOfDiff(start, goal);
        default:
            return -1;
    }
}

void printMap(const Map<Vec<int>, Vec<int>> & cameFrom, Vec<int> current) 
{
    for(int l = 1;; ++l) 
    {
        std::cout << l << ":\n" << current;
        auto it = cameFrom.find(current);
        if (it == cameFrom.end()) break; // while there are positions it map
        current = it->second;
    }
}

Vec<int> lowestFScore(const Vec2d<int> & openSet, const Map<Vec<int>, ValDefaultInf> & fScore) 
{
    ValDefaultInf lowestValue;
    Vec<int> lowestNode;

    for (const auto & node : openSet) 
    {
        auto it = fScore.find(node);
        if (it != fScore.end())
        {
            if (it->second.v < lowestValue.v)
            {
                lowestValue = it->second;
                lowestNode = node;
            }
        }
    }

    return lowestNode;
}

Vec2d<int> calculateNeighboors(const Vec<int> & node) 
{
    int positionZero = helpers::position(node, 0);
    if (positionZero == static_cast<int>(node.size())) // if its equal to nodes size, its an error
        return {}; // error, throw exception
    
    Vec2d<int> posChanges = {
        {1, 3}, {0, 2, 4}, {1, 5}, {0, 4, 6}, {1, 3, 5, 7}, {4, 2, 8}, {3, 7}, {6, 4, 8}, {7, 5}
    };

    Vec2d<int> neighboors;
    auto posVec = posChanges[positionZero];
    for (const auto & p : posVec)
    {
        auto newNode{node};
        using std::swap;
        swap(newNode[positionZero], newNode[p]);
        neighboors.push_back(newNode);
    }

    return neighboors;
}

int distanceBetween(const Map<Vec<int>, ValDefaultNeg> & level, const Vec<int> & start, const Vec<int> & end)
{
    auto itStart = level.find(start);
    auto itEnd = level.find(end);
    if (itStart != level.end() && itEnd != level.end())
    {
        return itEnd->second.v - itStart->second.v;
    }
    return constants::infinity;
}

void A_star(const Vec<int> & start, const Vec<int> & goal, Heuristic h) 
{
    // The set of nodes already evaluated.
    Vec2d<int> closedSet;

    // The set of currently discovered nodes that are not evaluated yet.
    // Initially, only the start node is known.
    Vec2d<int> openSet;
    openSet.push_back(start);
    
    // For each node, which node it can most efficiently be reached from.
    // If a node can be reached from many nodes, cameFrom will eventually contain the
    // most efficient previous step.
    Map<Vec<int>, Vec<int>> cameFrom;
    
    // For each node, the cost of getting from the start node to that node.
    Map<Vec<int>, ValDefaultInf> gScore; // map with default value of Infinity

    // The cost of going from start to start is zero.
    gScore[start] = ValDefaultInf{0};

    Map<Vec<int>, ValDefaultNeg> level;
    level[start] = ValDefaultNeg{0}; // start is level 0
        
    // For each node, the total cost of getting from the start node to the goal
    // by passing by that node. That value is partly known, partly heuristic.
    Map<Vec<int>, ValDefaultInf> fScore;

    // For the first node, that value is completely heuristic.
    fScore[start] = ValDefaultInf{heuristic(start, goal, h)};

    while (!openSet.empty()) 
    {
        Vec<int> current = lowestFScore(openSet, fScore); // the node in openSet having the lowest fScore[] value
        
        if (current == goal) 
        {
            printMap(cameFrom, current);
            return;
        }
        
        // remove current node from openSet
        helpers::remove(openSet, current);
        
        closedSet.push_back(current);

        // positions current can move
        Vec2d<int> neighboors = calculateNeighboors(current);

        for (const auto & neighbor : neighboors) 
        {
            if (level.find(neighbor) == level.end()) // if neighbor's level was not setted yet
                level[neighbor] = ValDefaultNeg{level[current].v + 1};
            
            // search if its in closeSet
            if (helpers::find(closedSet, neighbor))
                continue; // Ignore the neighbor which is already evaluated.

            int tentative_gScore = gScore[current].v + distanceBetween(level, current, neighbor);

            // search if its not in openSet
            if (!helpers::find(openSet, neighbor))
            {
                openSet.push_back(neighbor); // find a new node and add to openSet
            }
            else if (tentative_gScore >= gScore[neighbor].v) 
                continue; // pass, not the best way
            
            cameFrom[neighbor] = current;
            gScore[neighbor] = ValDefaultInf{tentative_gScore};
            fScore[neighbor] = ValDefaultInf{gScore[neighbor].v + heuristic(neighbor, goal, h)};
        }
    }
    
    std::cout << "Nao achou a solucao\n";
}

void displayErrorAndExit(std::string && error)
{
    std::cout << error << "\n";
    exit(0);
}

std::tuple<Vec<int>, Vec<int>, Heuristic> readData()
{
    std::ifstream file("estagios.txt", std::ios::binary);
    if (!file) 
    {
        displayErrorAndExit("Erro ao abrir arquivo estagios");
    }

    std::tuple<Vec<int>, Vec<int>, Heuristic> tuple;

    std::string input;
    for (int i = 0; std::getline(file, input); ++i) 
    {
        std::istringstream iss(input);
        if (i == 0 || i == 1) // 2 first lines are stages, start and goal
        {
            std::istream_iterator<int> itstart(iss), itend;
            Vec<int> stage(itstart, itend); // read all elements into vector 
            if (static_cast<int>(stage.size()) != 9) 
            {
                displayErrorAndExit("Erro na leitura do estagio");
            }
            if (i == 0) std::get<0>(tuple) = stage; // set tuple with stage
            else std::get<1>(tuple) = stage; // set tuple with stage
        }
        else  if (i == 2) // 3 line, heuristic
        {
            int heuristic;
            iss >> heuristic;
            if (heuristic != 1 && heuristic != 2)
            {
                displayErrorAndExit("Erro na leitura da heuristica");
            }
            std::get<2>(tuple) = static_cast<Heuristic>(heuristic);
        }
        else break; // no more lines
    }

    return tuple;
}

int main()
{
    std::ios::sync_with_stdio(false);
    // 2 8 3 1 6 4 7 0 5 // 6 passos
    // 8 1 3 2 4 5 0 7 6 // 9 passos

    auto data = readData();
    A_star(std::get<0>(data), std::get<1>(data), std::get<2>(data));

    #ifdef WINDOWS
    std::cin.ignore();
    #endif
}
