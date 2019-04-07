#include <iostream>
#include <queue>
#include <unordered_map>
#include <algorithm>

using namespace std;

struct Priority_Queue {
  //  A priority queue achieved by min heap used to hold nodes in exploring queue, which could achieve extracking node 
  //  with least priority in O(log(n)) of time complexity, where n is the number of nodes in queue
	typedef pair<int, pair<int, int>> element;
	priority_queue<element, vector<element>, greater<element>> elements;

	bool empty() {
		return elements.empty();
	}

	void push(int priority, pair<int, int> node) {
		elements.emplace(priority, node);
	}

	pair<int, int> pop() {
		auto minP = elements.top().second;
		elements.pop();
		return minP;
	}

};

struct pair_hash {
  // Method for pair hash
	template <class T1, class T2> 
	size_t operator() (const pair<T1, T2> & p) const {
		size_t h1 = hash<T1>()(p.first);
		size_t h2 = hash<T2>()(p.second);
		return h1 ^ h2;
	}
};

class discrete_planner {
  // Search methods for discrete motion planner, optimal planner achieved by A* algorithm
	int x_d;   // range in x dimension
	int y_d;   // range in y dimension
  public:
  	discrete_planner () {};

  	vector<pair<int, int>> optimal_planner (vector<vector<int>> & world_state, pair<int, int> & robot_pose, 
																						pair<int, int> & goal_pose) { 
      // Optimal planner achieved by A* Algorithm
  		x_d = world_state.size();
  		y_d = world_state[0].size();

      // Final generated path
  		vector<pair<int, int>> path = {};

      // Hold and extract node in exploring queue
  		Priority_Queue frontier;
  		frontier.push(0, robot_pose);

  		// Record nodes and their distances from start pose
      unordered_map<pair<int, int>, int, pair_hash> cost;
  		cost[robot_pose] = 0;

      // Hold nodes that has already been visited and record their parent nodes
      unordered_map<pair<int, int>, pair<int, int>, pair_hash> parent; 

  		while (!frontier.empty()) {
        // Get and visit nodes with least priority
  			auto cur = frontier.pop();

  			// If reach goal pose, track back to get path
        if (cur == goal_pose)
  				path = generate_path(cur, robot_pose, parent);

  			// Find neighbor nodes of current nodes
        vector<pair<int, int>> neighbors = find_neighbor(cur, world_state);
  			for (auto neighbor: neighbors) {
  				int new_cost = cost[cur] + 1;
          // No need to explore node that has been visited or its cost doesn't need to be updated
  				if (parent.find(neighbor) == parent.end() || new_cost < cost[neighbor]) {
  					cost[neighbor] = new_cost;
  					int priority = new_cost + cal_heuristic(cur, goal_pose);
  					frontier.push(priority, neighbor);
  					parent[neighbor] = cur;
  				}
  			}

  		}
  		return path;
  	}

  	vector<pair<int, int>> generate_path(pair<int, int> goal, pair<int, int> start, unordered_map<pair<int, int>, 
																				 pair<int, int>, pair_hash> & parent) {
      // Track back to get path from robot pose to goal pose
  		vector<pair<int, int>> path;
  		path.push_back(goal);
  		auto node = parent[goal];
  		while (node != start) {
  			path.push_back(node);
  			node = parent[node];
  		}
  		path.push_back(start);
  		reverse(path.begin(), path.end());
  		return path;
  	}

  	bool verify_node(pair<int, int> cur, vector<vector<int>> & world_state) {
      // Verify node whether reachable or not, if node is out of range of world_state or in obstacle, it can not be reached
  		if (cur.first >= 0 && cur.first < x_d && cur.second >= 0 && cur.second < y_d && !world_state[cur.first][cur.second])
  			return true;
  		else return false;
  	}

  	vector<pair<int, int>> find_neighbor(pair<int, int> cur, vector<vector<int>> & world_state) {
    // Find possbile neightbor nodes
      vector<pair<int, int>> NS;
  		vector<pair<int, int>> neighbors = {{cur.first+1, cur.second}, {cur.first-1, cur.second}, 
																					{cur.first, cur.second+1}, {cur.first, cur.second-1}};
  		for (auto n: neighbors) {
  			if (verify_node(n, world_state)) NS.push_back(n);
  		}
  		return NS;
  	}

  	int cal_heuristic(pair<int, int> cur, pair<int, int> goal) {
    // Calculate Manhatten distance between node and goal_pose
      return abs(cur.first - goal.first) + abs(cur.second - goal.second);
  	}
};

void generate_obstacle(vector<vector<int>>& world_state, int x_d, int y_d) {
  // generate obstacle in world graph
  for (int i = 0; i < y_d / 5 * 4; i++)
      world_state[x_d / 10 * 4 - 1][i] = 1;

  for (int i = y_d / 5; i < y_d; i++)
      world_state[x_d / 10 * 6 - 1][i] = 1;
}

int main () {
  int x_d = 50, y_d = 50;
  vector<vector<int>> world_state(x_d);
  for (int i = 0; i < x_d ; i++ )
    world_state[i].resize(y_d);
  generate_obstacle(world_state, x_d, y_d);
  pair<int, int> robot_pose = {5, 5};
  pair<int, int> goal_pose = {45, 45};
  discrete_planner DP;
  vector<pair<int, int>> path = DP.optimal_planner(world_state, robot_pose, goal_pose);
  for (auto p: path) {
      cout << p.first << " " << p.second << endl;
  }
  return 0;
}

