#ifndef HYBRID_BREADTH_FIRST_H_
#define HYBRID_BREADTH_FIRST_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class HBF {
public:

	int NUM_THETA_CELLS = 11;
	double SPEED = 2;
	double LENGTH = 1;

	struct maze_s {
	
		double g;	// iteration
		double f;
		double x;
		double y;
		double theta;
	};

	struct maze_path {
	
		vector< vector< vector<int> > > closed;
		vector< vector< vector<maze_s> > > came_from;
		maze_s final;

	};


	/**
  	* Constructor
  	*/
 	HBF();

	/**
 	* Destructor
 	*/
 	virtual ~HBF();
 	
 	static bool compare_maze_s(const HBF::maze_s & lhs, const HBF::maze_s & rhs);
    
    double heuristic(double x, double y, vector<double> goal);
     	
 	int theta_to_stack_number(double theta);

  	int idx(double float_num);

  	vector<maze_s> expand(maze_s state, vector<double> goal);

  	maze_path search(vector< vector<int> > grid, vector<double> start, vector<double> goal);

  	vector<maze_s> reconstruct_path(vector< vector< vector<maze_s> > > came_from, vector<double> start, HBF::maze_s final);
  	

};

#endif

