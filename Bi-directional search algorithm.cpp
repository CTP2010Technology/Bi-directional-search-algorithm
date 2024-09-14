#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <limits>

struct Node {
    int x, y;
    
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template <>
    struct hash<Node> {
        size_t operator()(const Node& node) const {
            return hash<int>()(node.x) ^ (hash<int>()(node.y) << 1);
        }
    };
}

class BidirectionalAStar {
private:
    std::vector<std::vector<int>> grid;
    int rows, cols;

    double heuristic(const Node& a, const Node& b) const {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    std::vector<Node> getNeighbors(const Node& node) const {
        std::vector<Node> neighbors;
        std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        for (const auto& dir : directions) {
            int nx = node.x + dir.first, ny = node.y + dir.second;
            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && grid[nx][ny] == 0) {
                neighbors.push_back({nx, ny});
            }
        }
        return neighbors;
    }

    std::vector<Node> reconstructPath(const std::unordered_map<Node, Node>& cameFrom, Node current) const {
        std::vector<Node> path = {current};
        while (cameFrom.find(current) != cameFrom.end()) {
            current = cameFrom.at(current);
            path.push_back(current);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

public:
    BidirectionalAStar(const std::vector<std::vector<int>>& grid) : grid(grid) {
        rows = grid.size();
        cols = grid[0].size();
    }

    std::vector<Node> findPath(const Node& start, const Node& goal) {
        auto compare = [](const std::pair<double, Node>& a, const std::pair<double, Node>& b) {
            return a.first > b.first;
        };
        std::priority_queue<std::pair<double, Node>, std::vector<std::pair<double, Node>>, decltype(compare)> openSetForward(compare);
        std::priority_queue<std::pair<double, Node>, std::vector<std::pair<double, Node>>, decltype(compare)> openSetBackward(compare);

        std::unordered_map<Node, Node> cameFromForward, cameFromBackward;
        std::unordered_map<Node, double> gScoreForward, gScoreBackward;

        gScoreForward[start] = 0;
        gScoreBackward[goal] = 0;
        openSetForward.push({heuristic(start, goal), start});
        openSetBackward.push({heuristic(goal, start), goal});

        while (!openSetForward.empty() && !openSetBackward.empty()) {
            Node currentForward = openSetForward.top().second;
            Node currentBackward = openSetBackward.top().second;

            if (currentForward == currentBackward || 
                gScoreBackward.find(currentForward) != gScoreBackward.end() ||
                gScoreForward.find(currentBackward) != gScoreForward.end()) {
                // Path found
                Node meetingPoint = (gScoreForward[currentForward] < gScoreBackward[currentBackward]) ? currentForward : currentBackward;
                auto forwardPath = reconstructPath(cameFromForward, meetingPoint);
                auto backwardPath = reconstructPath(cameFromBackward, meetingPoint);
                forwardPath.insert(forwardPath.end(), backwardPath.rbegin() + 1, backwardPath.rend());
                return forwardPath;
            }

            openSetForward.pop();
            for (const Node& neighbor : getNeighbors(currentForward)) {
                double tentativeGScore = gScoreForward[currentForward] + 1;
                if (gScoreForward.find(neighbor) == gScoreForward.end() || tentativeGScore < gScoreForward[neighbor]) {
                    cameFromForward[neighbor] = currentForward;
                    gScoreForward[neighbor] = tentativeGScore;
                    double fScore = tentativeGScore + heuristic(neighbor, goal);
                    openSetForward.push({fScore, neighbor});
                }
            }

            openSetBackward.pop();
            for (const Node& neighbor : getNeighbors(currentBackward)) {
                double tentativeGScore = gScoreBackward[currentBackward] + 1;
                if (gScoreBackward.find(neighbor) == gScoreBackward.end() || tentativeGScore < gScoreBackward[neighbor]) {
                    cameFromBackward[neighbor] = currentBackward;
                    gScoreBackward[neighbor] = tentativeGScore;
                    double fScore = tentativeGScore + heuristic(neighbor, start);
                    openSetBackward.push({fScore, neighbor});
                }
            }
        }

        return {}; // No path found
    }
};

int main() {
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };

    BidirectionalAStar astar(grid);
    Node start = {0, 0};
    Node goal = {4, 4};

    std::vector<Node> path = astar.findPath(start, goal);

    if (!path.empty()) {
        std::cout << "Path found:" << std::endl;
        for (const Node& node : path) {
            std::cout << "(" << node.x << ", " << node.y << ") ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "No path found." << std::endl;
    }

    return 0;
}