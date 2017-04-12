#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <iterator>
#include <limits>
#include <cmath>

namespace constants 
{
    const int infinity = std::numeric_limits<int>::max();
    const int negative = -1;
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
std::ostream &operator <<(std::ostream &os, const std::vector<T> &v)
{
   std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
   return os;
}

int heuristicOutOfPosition(const std::vector<int> & start, const std::vector<int> & goal) 
{
    int numbersOutPosition = 0;
    for (int i = 0; i < 9; ++i) 
    {
        if (start[i] != goal[i]) ++numbersOutPosition;
    }
    
    return numbersOutPosition;
}

int heuristicSumOfDiff(const std::vector<int> & start, const std::vector<int> & goal) 
{
    int sumDiff = 0;
    for (int i = 0; i < 9; ++i) 
    {
        sumDiff += std::abs(start[i] - goal[i]);
    }
    
    return sumDiff;
}

int heuristic(const std::vector<int> & start, const std::vector<int> & goal, Heuristic h)
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

void printMap(const std::map<std::vector<int>, std::vector<int>> & cameFrom, std::vector<int> current) 
{
    std::cout << current << "\n";

    for(;;) 
    {
        auto it = cameFrom.find(current);
        if (it != cameFrom.end())  // enquanto existir posicao no map
        {
            current = it->second;
            std::cout << current << "\n";
        }
        else 
            break;
    }
}

std::vector<int> lowestFScore(const std::vector<std::vector<int>> & openSet, const std::map<std::vector<int>, ValDefaultInf> & fScore) 
{
    ValDefaultInf lowestValue;
    std::vector<int> lowestNode;
    for (const auto & node : openSet) 
    {
        // std::cout << "node " << node << "\n";
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

std::vector<std::vector<int>> calculateNeighboors(const std::vector<int> & node) 
{
    int positionZero = std::distance(node.begin(), std::find(node.begin(), node.end(), 0)); // posicao do zero
    if (positionZero == static_cast<int>(node.size())) // se for igual a 9, deu erro
        return {}; // error, throw exception
    
    std::vector<std::vector<int>> posChanges = {
        {1, 3}, {0, 2, 4}, {1, 5}, {0, 4, 6}, {1, 3, 5, 7}, {4, 2, 8}, {3, 7}, {6, 4, 8}, {7, 5}
    };

    std::vector<std::vector<int>> neighboors;
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

int distanceBetween(const std::map<std::vector<int>, ValDefaultNeg> & level, const std::vector<int> & start, const std::vector<int> & end)
{
    auto itStart = level.find(start);
    auto itEnd = level.find(end);
    if (itStart != level.end() && itEnd != level.end())
    {
        return itEnd->second.v - itStart->second.v;
    }
    return constants::infinity;
}

void A_star(const std::vector<int> & start, const std::vector<int> & goal, Heuristic h) 
{
    // The set of nodes already evaluated.
    std::vector<std::vector<int>> closedSet;

    // The set of currently discovered nodes that are not evaluated yet.
    // Initially, only the start node is known.
    std::vector<std::vector<int>> openSet;
    openSet.push_back(start);
    
    // For each node, which node it can most efficiently be reached from.
    // If a node can be reached from many nodes, cameFrom will eventually contain the
    // most efficient previous step.
    std::map<std::vector<int>, std::vector<int>> cameFrom;
    
    // For each node, the cost of getting from the start node to that node.
    std::map<std::vector<int>, ValDefaultInf> gScore; // map with default value of Infinity

    // The cost of going from start to start is zero.
    gScore[start] = ValDefaultInf{0};

    std::map<std::vector<int>, ValDefaultNeg> level;
    level[start] = ValDefaultNeg{0}; // start is level 0
        
    // For each node, the total cost of getting from the start node to the goal
    // by passing by that node. That value is partly known, partly heuristic.
    std::map<std::vector<int>, ValDefaultInf> fScore;

    // For the first node, that value is completely heuristic.
    fScore[start] = ValDefaultInf{heuristic(start, goal, h)};

    while (!openSet.empty()) 
    {
        std::vector<int> current = lowestFScore(openSet, fScore); // the node in openSet having the lowest fScore[] value
        
        if (current == goal) 
        {
            printMap(cameFrom, current);
            return;
        }
        
         // remover nodo current do openSet
        openSet.erase(std::remove(openSet.begin(), openSet.end(), current), openSet.end());
        
        closedSet.push_back(current);

         // posicoes que ele pode mover 
        std::vector<std::vector<int>> neighboors = calculateNeighboors(current);

        for (const auto & neighbor : neighboors) 
        {
            if (level.find(neighbor) == level.end()) // if neighbor's level was not setted yet
                level[neighbor] = ValDefaultNeg{level[current].v + 1};
            
             // procurar se está no closedSet
            if (std::find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end())
                continue; // Ignore the neighbor which is already evaluated.

            int tentative_gScore = gScore[current].v + distanceBetween(level, current, neighbor);

            // procurar se não está no openSet
            if (std::find(openSet.begin(), openSet.end(), neighbor) == openSet.end())
            {
                openSet.push_back(neighbor); // descobriu um nodo novo e adicionou no openSet
            }
            else if (tentative_gScore >= gScore[neighbor].v) 
                continue; // Não é o melhor caminho
            
            cameFrom[neighbor] = current;
            gScore[neighbor] = ValDefaultInf{tentative_gScore};
            fScore[neighbor] = ValDefaultInf{gScore[neighbor].v + heuristic(neighbor, goal, h)};
        }
    }
    
    std::cout << "Não achou a solucao\n";
}

int main()
{
    std::ios::sync_with_stdio(false);
    A_star(
        {2, 8, 3, 1, 6, 4, 7, 0, 5}, // 6 passos
        //{8, 1, 3, 2, 4, 5, 0, 7, 6}, // 9 passos
        //{0, 1, 3, 4, 2, 5, 7, 8, 6}, // tem que voltar
        // {0, 1, 4, 3, 2, 5, 7, 8, 6},
        // {1, 2, 3, 4, 5, 6, 7, 8, 0}, // nao tem solucao
        {1, 2, 3, 8, 0, 4, 7, 6, 5},
        //{1, 2, 3, 4, 5, 6, 7, 8, 0},
        Heuristic::OutOfPosition
    );
}
