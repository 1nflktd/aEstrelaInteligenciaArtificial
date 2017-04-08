#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <limits>

namespace constants 
{
    const int infinity = std::numeric_limits<int>::max();
};

struct ValueForMyMap 
{
    int v = constants::infinity;
};

// verificar quantidade de números fora de posição
int heuristic_cost_estimate(const std::vector<int> & start, const std::vector<int> & goal) 
{
    if (start.size() != goal.size() || start.size() != 9 || goal.size() != 9) 
    {
        return -1; // error , throw exception ?   
    }
    
    int numbersOutPosition = 0;
    for (int i = 0; i < 9; ++i) 
    {
        if (start[i] != goal[i]) ++numbersOutPosition;
    }
    
    return numbersOutPosition;
}

void printPosition(const std::vector<int> & current) 
{
    for (const auto & c : current) 
        std::cout << c << " ";   
    std::cout << "\n";
}

void printMap(const std::map<std::vector<int>, std::vector<int>> & cameFrom, std::vector<int> current) 
{
    printPosition(current);

    for(;;) 
    {
        auto it = cameFrom.find(current);
        if (it != cameFrom.end())  // enquanto existir posicao no map
        {
            current = it->second;
            printPosition(current);
        }
        else 
            break;
    }
}

std::vector<int> lowestFScore(const std::vector<std::vector<int>> & openSet, const std::map<std::vector<int>, ValueForMyMap> & fScore) 
{
    ValueForMyMap lowestValue;
    std::vector<int> lowestNode;
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

void A_star(const std::vector<int> & start, const std::vector<int> & goal) 
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
    std::map<std::vector<int>, ValueForMyMap> gScore; // map with default value of Infinity

    // The cost of going from start to start is zero.
    gScore[start] = ValueForMyMap{0};
        
    // For each node, the total cost of getting from the start node to the goal
    // by passing by that node. That value is partly known, partly heuristic.
    std::map<std::vector<int>, ValueForMyMap> fScore; // := map with default value of Infinity

    // For the first node, that value is completely heuristic.
    fScore[start] = ValueForMyMap{heuristic_cost_estimate(start, goal)};

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
             // procurar se está no closedSet
            if (std::find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end())
                continue; // Ignore the neighbor which is already evaluated.
            
            int tentative_gScore = gScore[current].v + 1; // dist_between(current, neighbor); verificar essa funcao
            
             // procurar se não está no openSet
            if (std::find(openSet.begin(), openSet.end(), neighbor) == openSet.end())
                openSet.push_back(neighbor); // descobriu um nodo novo e adicionou no openSet
            else if (tentative_gScore >= gScore[neighbor].v) 
                continue; // Não é o melhor caminho
            
            cameFrom[neighbor] = current;
            gScore[neighbor] = ValueForMyMap{tentative_gScore};
            fScore[neighbor] = ValueForMyMap{gScore[neighbor].v + heuristic_cost_estimate(neighbor, goal)};
        }
    }
    
    std::cout << "Não achou a solucao\n";
}

int main()
{
    A_star(
        // {2, 8, 3, 1, 6, 4, 7, 0, 5},
        {8, 1, 3, 2, 4, 5, 0, 7, 6},
        {1, 2, 3, 8, 0, 4, 7, 6, 5}
    );   
}