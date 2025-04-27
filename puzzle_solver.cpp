#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <unordered_map>
#include <functional>
#include <fstream>

using namespace std;

int counter = 0;

class Node {
public:
    string state;
    Node* parent;
    string action;
    int pathCost, heuristicCost, totalCost;
    bool isReplaced = false;

    struct NodeCostComparator {
        bool operator()(const Node* a, const Node* b) const {
            return a->totalCost > b->totalCost;
        }
    };

    Node(string state, Node* parent, string action, int pathCost, int heuristicCost)
        : state(state), parent(parent), action(action), pathCost(pathCost), heuristicCost(heuristicCost) {
            totalCost = pathCost + heuristicCost;
            counter += 1;
        }

    //Root node constructors:

    Node(string state)
        : state(state), parent(nullptr), action(""), pathCost(0), heuristicCost(0) {
            totalCost = pathCost + heuristicCost;
            counter += 1;
        }
    Node(string state, int heuristicCost)
        : state(state), parent(nullptr), action(""), pathCost(0), heuristicCost(heuristicCost) {
            totalCost = pathCost + heuristicCost;
            counter += 1;
        }
};

class Problem {
public:
    string initialState;
    string goalState;

    Problem(string initialState, string goalState)
        : initialState(initialState), goalState(goalState) {}

    bool isGoalState(const string& state) {
        return state == goalState;
    }

    vector<string> actions(const string& state) {
        vector<string> possibleActions;
        if (state.find('#') != string::npos) {
            size_t blankPos = state.find('#');

            if (blankPos >= 4) {
                possibleActions.push_back("UP");
            }
            if (blankPos < 12) {
                possibleActions.push_back("DOWN");
            }
            if (blankPos % 4 != 0) {
                possibleActions.push_back("LEFT");
            }
            if (blankPos % 4 != 3) {
                possibleActions.push_back("RIGHT");
            }
        }
        return possibleActions;
    }

    string result(const string& state, const string& action) {
        string newState = state;
        size_t blankPos = state.find('#');

        if (action == "UP") {
            swap(newState[blankPos], newState[blankPos - 4]);
        }
        else if (action == "DOWN") {
            swap(newState[blankPos], newState[blankPos + 4]);
        }
        else if (action == "LEFT") {
            swap(newState[blankPos], newState[blankPos - 1]);
        }
        else if (action == "RIGHT") {
            swap(newState[blankPos], newState[blankPos + 1]);
        }

        return newState;
    }

    int getHeuristicCost(const string& state) {
        string goal = "ABCDEFGHIJKLMNO#";

        unordered_map<char, pair<int, int>> goal_positions;

        for (int i = 0; i < goal.size(); i++) {
            goal_positions[goal[i]] = {i / 4, i % 4};
        }

        int distance = 0;
        for (int i = 0; i < state.size(); i++) {
            if (state[i] != '#') {
                int x1 = i / 4, y1 = i % 4;
                int x2 = goal_positions[state[i]].first;
                int y2 = goal_positions[state[i]].second;
                distance += abs(x1 - x2) + abs(y1 - y2);
            }
        }

        return distance;
    }

    Node* childNode(Node* parent, const string& action) {
        string newState = result(parent->state, action);
        int heuristicCost = getHeuristicCost(newState);
        return new Node(newState, parent, action, parent->pathCost+1, heuristicCost);
    }
};

class SearchAlgorithm {
public:
    Problem problem;

    SearchAlgorithm(Problem problem)
        : problem(problem) {}

    vector<string> solution(Node* node) {
        vector<string> path;

        while (node->parent != nullptr) {
            path.push_back(node->action);
            node = node->parent;
        }

        reverse(path.begin(), path.end());
        return path;
    }

    vector<string> breadthFirstSearch() {

        vector<Node*> allCreatedNodes;

        //node <- a node with STATE = problem.INITIAL-STATE, PATH-COST = 0
        Node* node = new Node(problem.initialState);
        allCreatedNodes.push_back(node);

        //if problem.GOAL-TEST(node.STATE) then return SOLUTION(node)
        if (problem.isGoalState(node->state)) {
            vector<string> soln = solution(node);
            for (Node* node : allCreatedNodes) {
                delete node;
            }
            return soln;
        }

        //frontier <- a FIFO queue with node as the only element
        queue<Node*> frontier;
        frontier.push(node);

        //explored <- an empty set
        unordered_set<string> explored;

        //loop do if EMPTY?(frontier ) then return failure
        while (!frontier.empty()) {
            //node <- POP(frontier ) /* chooses the shallowest node in frontier */
            node = frontier.front();
            frontier.pop();

            //add node.STATE to explored
            explored.insert(node->state);

            vector<string> actions = problem.actions(node->state);

            //for each action in problem.ACTIONS(node.STATE) do
            for (const string& action : actions) {
                //child <- CHILD-NODE(problem, node, action)
                Node* child = problem.childNode(node, action);
                allCreatedNodes.push_back(child);

                //if child.STATE is not in explored or frontier then
                if (explored.find(child->state) == explored.end()) {
                    //if problem.GOAL-TEST(child.STATE) then return SOLUTION(child)
                    if (problem.isGoalState(child->state)) {
                        vector<string> sol = solution(child);
                        for (Node* node : allCreatedNodes) {
                            delete node;
                        }
                        return sol;
                    }

                    //frontier <- INSERT(child,frontier )
                    frontier.push(child);
                    explored.insert(child->state);
                }
            }
        }

        for (Node* node : allCreatedNodes) {
            delete node;
        }

        return {};
    }

    vector<string> aStarSearch() {

        vector<Node*> allCreatedNodes;

        //node <- a node with STATE = problem.INITIAL-STATE, TOTAL-COST = 0
        Node* node = new Node(problem.initialState, problem.getHeuristicCost(problem.initialState));
        allCreatedNodes.push_back(node);

        //frontier <- a priority queue ordered by TOTAL-COST, with node as the only element
        priority_queue<Node*, vector<Node*>, Node::NodeCostComparator> frontier;
        frontier.push(node);

        unordered_map<string, Node*> frontierStateToNodeMap;
        frontierStateToNodeMap.insert({node->state, node});

        //explored <- an empty set
        unordered_set<string> explored;

        //loop do if EMPTY?(frontier ) then return failure
        while (!frontier.empty()) {
            //node <- POP(frontier ) /* chooses the lowest-cost node in frontier */
            node = frontier.top();
            frontier.pop();

            if (node->isReplaced == true) {
                continue;
            }

            frontierStateToNodeMap.erase(node->state);

            //if problem.GOAL-TEST(node.STATE) then return SOLUTION(node)
            if (problem.isGoalState(node->state)) {
                vector<string> soln = solution(node);
                for (Node* node : allCreatedNodes) {
                    delete node;
                }
                return soln;
            }

            //add node.STATE to explored
            explored.insert(node->state);

            //for each action in problem.ACTIONS(node.STATE) do
            vector<string> actions = problem.actions(node->state);

            for (string& action : actions) {
                //child <- CHILD-NODE(problem, node, action)
                Node* child = problem.childNode(node, action);
                allCreatedNodes.push_back(child);

                //if child.STATE is not in explored or frontier then
                if (explored.find(child->state) == explored.end() || frontierStateToNodeMap.find(child->state) == frontierStateToNodeMap.end()) {
                    //frontier <- INSERT(child,frontier)
                    frontier.push(child);
                    frontierStateToNodeMap.insert({child->state, child});
                }
                //else if child.STATE is in frontier with higher TOTAL-COST then
                else if (frontierStateToNodeMap.find(child->state) != frontierStateToNodeMap.end()) {
                    Node* oldNode = frontierStateToNodeMap.at(child->state);
                    if (oldNode->totalCost > child->totalCost) {
                        //replace that frontier node with child
                        frontierStateToNodeMap[child->state] = child;
                        frontier.push(child);
                        oldNode->isReplaced = true;
                    }
                }
            }
        }

        for (Node* node : allCreatedNodes) {
            delete node;
        }

        return {};
    }
};

int main() {
    ifstream inputFile("puzzles.txt");
    string line;

    while (getline(inputFile, line)) {
        Problem problem(line, "ABCDEFGHIJKLMNO#");
        SearchAlgorithm searchAlgorithm(problem);

        vector<string> solution = searchAlgorithm.aStarSearch();

        cout << "Solution Cost: " << solution.size() << " Nodes Generated: " << counter << endl;
    }

    inputFile.close();

    return 0;
}
