#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "hybrid_breadth_first.h"

using namespace std;

HBF::HBF()
{
}

HBF::~HBF() {}

bool HBF::compare_maze_s(const HBF::maze_s &lhs, const HBF::maze_s &rhs)
{
  return lhs.f < rhs.f;
}

// 距离目标点的距离作为启发式的cost
double HBF::heuristic(double x, double y, vector<double> goal)
{
  double x_diff = abs(x - goal[0]);
  double y_diff = abs(y - goal[1]);
  double index_diff = hypot(x_diff, y_diff);
  // return fabs(y - goal[0]) + fabs(x - goal[1]); //return grid distance to goal
  return index_diff;
}

int HBF::theta_to_stack_number(double theta)
{
  /*
  Takes an angle (in radians) and returns which "stack" in the 3D configuration space
  this angle corresponds to. Angles near 0 go in the lower stacks while angles near 
  2 * pi go in the higher stacks.
  */
  double new_theta = fmod((theta + 2 * M_PI), (2 * M_PI));
  int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2 * M_PI))) % NUM_THETA_CELLS;
  return stack_number;
}

int HBF::idx(double float_num)
{
  /*
  Returns the index into the grid for continuous position. So if x is 3.621, then this
  would return 3 to indicate that 3.621 corresponds to array index 3.
  */
  return int(floor(float_num));
}

// 根据自行车的运动学模型,进行扩展
vector<HBF::maze_s> HBF::expand(HBF::maze_s state, vector<double> goal)
{
  double g = state.g;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;

  double g2 = g + 1;
  vector<HBF::maze_s> next_states;
  // 角度的限制
  for (double delta_i = -60; delta_i <= 60; delta_i += 2)
  {

    double delta = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta);
    double theta2 = theta + omega;
    if (theta2 < 0)
    {
      theta2 += 2 * M_PI;
    }
    double x2 = x + SPEED * cos(theta2);
    double y2 = y + SPEED * sin(theta2);
    HBF::maze_s state2;

    state2.f = g2 + heuristic(x2, y2, goal);
    state2.g = g2;
    state2.x = x2;
    state2.y = y2;
    state2.theta = theta2;
    next_states.push_back(state2);
  }
  // for (int i = -1; i <= 1; i += 1)
  // {
  //   for (int j = -1; j <= 1; j += 1)
  //   {

  //     double x2 = x + i;
  //     double y2 = y + j;
  //     HBF::maze_s state2;
  //     state2.f = g2 + heuristic(x2, y2, goal);
  //     state2.g = g2;
  //     state2.x = x2;
  //     state2.y = y2;
  //     state2.theta = 0;
  //     next_states.push_back(state2);
  //   }
  // }
  return next_states;
}

vector<HBF::maze_s> HBF::reconstruct_path(vector<vector<vector<HBF::maze_s>>> came_from, vector<double> start, HBF::maze_s final)
{

  vector<maze_s> path = {final};

  int stack = theta_to_stack_number(final.theta);

  maze_s current = came_from[stack][idx(final.x)][idx(final.y)];

  stack = theta_to_stack_number(current.theta);

  double x = current.x;
  double y = current.y;
  
  while (x != start[0] || y != start[1])
  {
    path.push_back(current);
    current = came_from[stack][idx(x)][idx(y)];
    x = current.x;
    y = current.y;
    stack = theta_to_stack_number(current.theta);
  }

  return path;
}



HBF::maze_path HBF::search(vector<vector<int>> grid, vector<double> start, vector<double> goal)
{
  /*
  Working Implementation of breadth first search. Does NOT use a heuristic
  and as a result this is pretty inefficient. Try modifying this algorithm 
  into hybrid A* by adding heuristics appropriately.
  */
  int num = max(grid[0].size(), grid.size());
  vector<vector<vector<int>>> closed(NUM_THETA_CELLS, vector<vector<int>>(num, vector<int>(num)));
  // theta,x,y 三个状态量
  // vector<vector<vector<int>>> closed(NUM_THETA_CELLS, vector<vector<int>>(grid[0].size(), vector<int>(grid.size())));
  vector<vector<vector<maze_s>>> came_from(NUM_THETA_CELLS, vector<vector<maze_s>>(grid[0].size(), vector<maze_s>(grid.size())));
  double theta = start[2];
  int stack = theta_to_stack_number(theta);
  double g = 0;

  maze_s state;
  state.g = g;
  state.x = start[0];
  state.y = start[1];
  // heuristic() return grid distance to goal
  state.f = g + heuristic(state.x, state.y, goal);
  state.theta = theta;

  closed[stack][idx(state.x)][idx(state.y)] = 1;
  came_from[stack][idx(state.x)][idx(state.y)] = state;

  int total_closed = 1;
  vector<maze_s> opened = {state};
  bool finished = false;

  while (!opened.empty())
  {
    // 按照cost最小排序
    sort(opened.begin(), opened.end(), compare_maze_s);
    //grab first elment  // cost最小
    maze_s current = opened[0];
    opened.erase(opened.begin()); //pop first element

    double x = current.x;
    double y = current.y;

    // if ( idx(x) == goal[0] && idx(y) == goal[1])
    int x_diff = abs(idx(x) - goal[0]);
    int y_diff = abs(idx(y) - goal[1]);
    float index_diff = hypot(x_diff, y_diff);
    // printf("index_diff   %f ", index_diff);
    if (index_diff < 5)
    {
      cout << "found path to goal in " << total_closed << " expansions" << endl;
      maze_path path;
      path.came_from = came_from;
      path.closed = closed;
      path.final = current;
      return path;
    }

    // 扩展当前节点的周围节点,运动学方程
    vector<maze_s> next_state = expand(current, goal);

    for (int i = 0; i < next_state.size(); i++)
    {
      int g2 = next_state[i].g;
      double x2 = next_state[i].x;
      double y2 = next_state[i].y;
      double theta2 = next_state[i].theta;

      if ((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size()))
      {
        //invalid cell
        continue;
      }
      int stack2 = theta_to_stack_number(theta2);

      if (closed[stack2][idx(x2)][idx(y2)] == 0 && grid[idx(x2)][idx(y2)] == 0)
      {

        opened.push_back(next_state[i]);
        closed[stack2][idx(x2)][idx(y2)] = 1;
        came_from[stack2][idx(x2)][idx(y2)] = current;
        total_closed += 1;
      }
    }
    // printf("Hybrid A* searching   ");
  }
  cout << "no valid path." << endl;
  HBF::maze_path path;
  path.came_from = came_from;
  path.closed = closed;
  path.final = state;
  return path;
}
