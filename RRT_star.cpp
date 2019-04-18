#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <climits>
#include <algorithm>
#include <vector>
#include <unordered_map>
using namespace std;

class RRTnode {
  public:
  double x;
  double y;
  int parent;
  double cost;
  pair<double, double> tuple;
  
  RRTnode() {}

  RRTnode(double a, double b) {
    x = a;
    y = b;
    tuple = {a, b};
  }
};

class pathPlanner {
  private:
    double x_range;
    double y_range;
    double expand_length;
    int iteration;
    double robot_size;
    RRTnode start;
    RRTnode goal;
    vector<RRTnode> tree;
    unordered_map<int, pair<double, bool>> store;
  public:
    vector<pair<double, double>> RRT (double xr, double yr, pair<double, double> & robot_pose, pair<double, double> & goal_pose, double el, int iter, vector<vector<double>> & obs_list, double rs) {
      x_range = xr;
      y_range = yr;
      expand_length = el;
      iteration = iter;
      robot_size = rs;
      start = RRTnode(robot_pose.first, robot_pose.second);
      start.cost = 0.0; 
      tree.push_back(start);
      goal = RRTnode(goal_pose.first, goal_pose.second);

      vector<pair<double, double>> path = {};

      for (int i = 0; i < iteration; i++) {
        RRTnode n_rand = random_sample();
        int nearest_index = nearest(n_rand);
        RRTnode n_nearest = tree[nearest_index];
        RRTnode n_new = steer(nearest_index, n_nearest, n_rand);

        if (!check_collision(n_new, obs_list))
          tree.push_back(n_new);
        else 
          continue;

        if (reach_goal(n_new)) {
          path = generate_path(n_new);
          break;
        }
      }
      return path;
    }

    RRTnode random_sample () {
      double x = x_range * (double) rand() / (double) RAND_MAX;
      double y = y_range * (double) rand() / (double) RAND_MAX;
      return RRTnode(x, y);
    }

    double cal_dis(RRTnode & a, RRTnode & b) {
      return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    int nearest(RRTnode & n_rand) {
      int min_index;
      double min_dis = (double) INT_MAX;
      int n = tree.size();
      for (int i = 0; i < n; i++) {
        double dis = cal_dis(n_rand, tree[i]);
        if (dis < min_dis) {
          min_index = i;
          min_dis = dis;
        }
      }
      return min_index;
    }

    RRTnode steer(int nearest_index, RRTnode & n_nearest, RRTnode & n_rand) {
      double theta = atan2 (n_rand.y - n_nearest.y, n_rand.x - n_nearest.x);
      double x = n_nearest.x + expand_length * cos(theta);
      double y = n_nearest.y + expand_length * sin(theta);
      RRTnode n_new(x, y);
      n_new.parent = nearest_index;
      return n_new;
    }

    bool check_collision(RRTnode & n_new, vector<vector<double>> & obs_list) {
      double x = n_new.x;
      double y = n_new.y;
      for (auto obs: obs_list) {
        if (x >= obs[0] - robot_size and x < obs[1] + robot_size and y >= obs[2] - robot_size and y < obs[3] + robot_size)
          return true;
      }
      return false;
    }

    bool reach_goal(RRTnode & n_new) {
      if (cal_dis(n_new, goal) <= expand_length)
        return true;
      else
        return false;
    }

    vector<pair<double, double>> generate_path(RRTnode & n_new) {
      vector<pair<double, double>> path;
      path.push_back(goal.tuple);
      path.push_back(n_new.tuple);
      RRTnode temp = tree[n_new.parent];
      while (temp.tuple != start.tuple) {
        path.push_back(temp.tuple);
        temp = tree[temp.parent];
      }
      path.push_back(start.tuple);
      reverse(path.begin(), path.end());
      return path;
    }

    vector<pair<double, double>> RRT_star (double xr, double yr, pair<double, double> & robot_pose, pair<double, double> & goal_pose, double el, int iter, vector<vector<double>> & obs_list, double rs) {
      x_range = xr;
      y_range = yr;
      expand_length = el;
      iteration = iter;
      robot_size = rs;
      start = RRTnode(robot_pose.first, robot_pose.second);
      start.cost = 0.0; 
      tree.push_back(start);
      goal = RRTnode(goal_pose.first, goal_pose.second);

      vector<pair<double, double>> path = {};

      for (int i = 0; i < iteration; i++) {
        RRTnode n_rand = random_sample();
        int nearest_index = nearest(n_rand);
        RRTnode n_nearest = tree[nearest_index];
        RRTnode n_new = steer(nearest_index, n_nearest, n_rand);

        if (!check_collision(n_new, obs_list)) {
          vector<int> nears_index = find_nears(n_new);
          if (nears_index.empty())
            continue;
          choose_best_parent(n_new, nears_index, obs_list);
          tree.push_back(n_new);
          rewire(n_new, nears_index);
        }
      }

      int best_last_index = get_best_index();
        if (best_last_index >= 0)
            path = generate_path(tree[best_last_index]);

      return path;
    }

    vector<int> find_nears(RRTnode & n_new) {
      vector<int> nears_index;
      int n = tree.size();
      double r = 50.0 * sqrt(log(n + 1) / n);
      for (int i = 0; i < n; i++) {
        double dis = cal_dis(n_new, tree[i]);
        if (dis <= r) 
          nears_index.push_back(i);
      }
      return nears_index;
    }

    void choose_best_parent(RRTnode & n_new, vector<int> & nears_index, vector<vector<double>> & obs_list) {
      int min_index;
      double min_cost = (double) INT_MAX;
      for (int i: nears_index) {
        double dis = cal_dis(n_new, tree[i]);
        bool collision = check_collision_extend(n_new, tree[i], dis, obs_list);
        store[i] = make_pair(dis, collision);
        double cost = dis + tree[i].cost;
        if (!collision && cost < min_cost) {
          min_cost = cost;
          min_index = i;
        }
      }
      n_new.cost = min_cost;
      n_new.parent = min_index;
    }

    bool check_collision_extend(RRTnode & n_new, RRTnode & node, double dis, vector<vector<double>> & obs_list) {
      double theta = atan2 (node.y - n_new.y, node.x - n_new.x);
      for (int i = 1; i < int (dis / expand_length); i++) {
        double x = n_new.x + cos(theta) * i * expand_length;
        double y = n_new.y + sin(theta) * i * expand_length;
        RRTnode temp = RRTnode(x, y);
        if (check_collision(temp, obs_list))
          return true;
      }
      return false;
    }

    void rewire(RRTnode & n_new, vector<int> & nears_index) {
      int n = tree.size();
      for (int i: nears_index) {
        double new_cost = store[i].first + n_new.cost;
        if (i != n_new.parent && !store[i].second && new_cost < tree[i].cost) {
          tree[i].cost = new_cost;
          tree[i].parent = n - 1;
        }
      }
    }

    int get_best_index() {
      int n = tree.size();
      double best_dis = (double) INT_MAX ;
      double best_index;
      for (int i = 0; i < n; i++) {
        double dis = cal_dis(tree[i], goal);
        if (dis < best_dis) {
          best_dis = dis;
          best_index = i;
        }

      }
      if (best_dis <= expand_length)
        return best_index;
      else
        return -1;
    }
};

int main() {
  // Parameter initialization
  double x_range = 50, y_range = 50;
  double expand_length = 0.5;
  int iteration = 5000;
    double robot_size = 3.0;

    // Obstacles generation
    vector<vector<double>> obs_list = {{0.0, x_range, 0.0, 1.0}, {0.0, x_range, y_range - 1.0, y_range}, {0.0, 1.0, 0.0, y_range}, {x_range - 1.0, x_range, 0.0, y_range}};
    obs_list.push_back({15.0, 16.0, 0.0, 30.0});
    obs_list.push_back({35.0, 36.0, 20.0, 50.0});

    // Initialize robot pose and goal pose
    pair<double, double> robot_pose = {5.0, 5.0};
    pair<double, double> goal_pose = {x_range - 5.0, y_range - 5.0};

    // Run optimal planner
    pathPlanner op;
    vector<pair<double, double>> op_path = op.RRT(x_range, y_range, robot_pose, goal_pose, expand_length, iteration, obs_list, robot_size);
    if (!op_path.empty()) 
      cout << "Optimal search succeed!" << endl;
    else 
      cout << "No optimal path is found!" << endl;

    // Run sub-optimal planner
    pathPlanner sp;
    vector<pair<double, double>> sub_path = sp.RRT_star(x_range, y_range, robot_pose, goal_pose, expand_length, iteration, obs_list, robot_size);
    if (!sub_path.empty()) 
      cout << "Sub-optimal search succeed!" << endl;
    else 
      cout << "No sub-optimal path is found!" << endl;

  return 0;
}
