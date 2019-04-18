#include <iostream>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <math.h>
using namespace std;

struct Priority_Queue {
	// Priority Queue implemented by pair and priority_queue to hold nodes in exploring queue, which could achieve extracting node 
  	// with least priority and inserting new node in O(log(n)) of time complexity, where n is the number of nodes in queue
  	typedef pair<double, pair<int, int>> node;
  	priority_queue<node, vector<node>, greater<node>> nodeQueue;

  	bool empty() {
    		return nodeQueue.empty();
  	}

  	void push(double priority, pair<int, int> pos) {
     		nodeQueue.emplace(priority, pos);
  	}

  	pair<int, int> pop() {
    		auto minNodePos = nodeQueue.top().second;
    		nodeQueue.pop();
    		return minNodePos;
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

class Search {
  	// Search methods for path planner
    private:
  	int x_range; 
  	int y_range;
  	int robot_size;
    public:
  	vector<pair<int, int>> A_star (vector<vector<int>> & world_state, pair<int, int> & robot_pose, pair<int, int> & goal_pose, vector<vector<int>> & obs_list, int rs) { 
    		// Optimal planner achieved by A* Algorithm
    		x_range = world_state.size();
    		y_range = world_state[0].size();
    		robot_size = rs;

    		// Final generated path
    		vector<pair<int, int>> path = {};

    		// Exploring queue
    		Priority_Queue frontier;
    		frontier.push(0.0, robot_pose);

    		// Record nodes and their costs from start pose
    		unordered_map<pair<int, int>, double, pair_hash> cost;
    		cost[robot_pose] = 0.0;

    		// Record visitted nodes and their parents
    		unordered_map<pair<int, int>, pair<int, int>, pair_hash> parent; 

    		while (!frontier.empty()) {
      			// Extract and visit nodes with least priority
      			auto cur = frontier.pop();

      			// If we reach goal pose, track back to get path
      			if (cur == goal_pose)
        			path = generate_path(cur, robot_pose, parent);

      			// Get possible next step movements of current node
      			vector<pair<int, int>> motions = get_robot_motion(cur, obs_list);
      			for (auto motion: motions) {
        			double new_cost = cost[cur] + cal_dis(cur, motion);
        			// No need to explore node that has been visited or its cost doesn't need to be updated
        			if (parent.find(motion) == parent.end() || new_cost < cost[motion]) {
          				cost[motion] = new_cost;
          				double priority = new_cost + cal_heuristic(motion, goal_pose);
          				frontier.push(priority, motion);
          				parent[motion] = cur;
        			}
      			}
    		}
    		return path;
  	}

  	vector<pair<int, int>> generate_path(pair<int, int> goal, pair<int, int> start, unordered_map<pair<int, int>, pair<int, int>, pair_hash> & parent) {
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

  	vector<pair<int, int>> get_robot_motion(pair<int, int> cur, vector<vector<int>> & obs_list) {
  		// Robot motion model
    		int x;
  		int y;
    		vector<pair<int, int>> next_step;
    		vector<pair<int, int>> robot_motion = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

    		for (auto motion: robot_motion) {
    			x = cur.first + motion.first;
    			y = cur.second + motion.second;
    			if (verify_node(x, y) and ! check_collision(x, y, obs_list))
    				next_step.emplace_back(x, y);
    		}
    		return next_step;
	}

    	bool verify_node(int x, int y) {
    		// Verify whether a node is in the range of world
    		if (x >= 0 && x < x_range && y >= 0 && y < y_range)
    			return true;
    		else 
    			return false;
    	}

    	bool check_collision(int x, int y, vector<vector<int>> & obs_list) {
    		// Check if node get collision with obstacle, take into consideration about robot size
    		for (auto obs: obs_list) {
    			if (x >= obs[0] - robot_size and x < obs[1] + robot_size and y >= obs[2] - robot_size and y < obs[3] + robot_size)
    				return true;
    		}
    		return false;
    	}

    	double cal_dis(pair<int, int> cur, pair<int, int> node) {
    		// Calculate distance between two nodes
    		return sqrt(pow((cur.first - node.first), 2) + pow((cur.second - node.second), 2));
    	}

  	double cal_heuristic(pair<int, int> cur, pair<int, int> goal) {
    		// Calculate distance between node and goal_pose as heuristic
    		return pow((cur.first - goal.first), 2) + pow((cur.second - goal.second), 2);
  	}
};


int main() {
	// Parameter initialization
	int x_range = 50, y_range = 50;
  	int robot_size = 3;

  	// World graph generation
  	vector<vector<int>> world_state(x_range);
  	for (int i = 0; i < x_range ; i++ )
    	world_state[i].resize(y_range);

    	// Obstacles generation
    	vector<vector<int>> obs_list = {{0, x_range, 0, 1}, {0, x_range, y_range - 1, y_range}, {0, 1, 0, y_range}, {x_range - 1, x_range, 0, y_range}};
    	obs_list.push_back({15, 16, 0, 30});
    	obs_list.push_back({35, 36, 20, 50});

    	// Initialize robot pose and goal pose
  	pair<int, int> robot_pose = {5, 5};
  	pair<int, int> goal_pose = {x_range - 5, y_range - 5};

  	// Run optimal planner
  	Search search;
  	vector<pair<int, int>> path = search.A_star(world_state, robot_pose, goal_pose, obs_list, robot_size);
  	if (!path.empty()) 
  		cout << "Optimal search succeed!" << endl;
  	else 
  		cout << "No optimal path is found!" << endl;
  	return 0;
}
