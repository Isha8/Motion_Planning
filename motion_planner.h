#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include <set>
#include <unordered_set>
#include <boost/circular_buffer.hpp>
#include <cstdlib>
#include <ctime>


using namespace std;

class motion_planner{
	
	public:
	
	int max_step_number;														//max steps after which random planner should stop
	typedef boost::circular_buffer<pair<int,int> > circular_buffer;
	
	/* 
	 * Class for optimal planner (A* search). Each postion in the grid is a object of this class
	 * */
	class Node
	{
	  public:
		int g= 0, h=0;
		char val;                                                                //Char value in the grid
		pair<int,int> pos;
		Node *parent = 0; 
			 
		Node(pair<int,int>nodePos,int value)
		{
			pos=nodePos;
			val=value;
		}
		//Node() : parent( make_unique<Node>()) {}
		int move_cost(Node other)
		{
			return 1;
		}
	 
		pair<int,int> get_pos() const
		{
			return pos;
		}	 
	};
	
	/* 
	 * Custom Hasher for Class Node, to be used for Unordered_set. 
	 * (Can even use priority_queues for storing the obejcts of class Node instead of hashset)
	 * */
	struct NodeHasher
	{
	  //template <typename T, typename U>
	  size_t
	  operator()( const Node &obj) const
	  {
		pair<int,int> position;
		position = obj.get_pos();
		return 3* std::hash<int>()(position.first) + std::hash<int>()(position.second) ; //custom hasher key when we using pair
	  }
	};

	/*
	 *  Custom Comparator for Class Node, to be used for Unordered_set
	 * */
	struct NodeComparator
	{
	  bool
	  operator()(const Node  &obj1, const  Node  &obj2) const
	  {
		if (obj1.get_pos() == obj2.get_pos())
		  return true;
		return false;
	  }
	};

	/*
	 *  Comparing the objects based on the f = h+g cost
	 * */
	struct MinFValue
	{
	  bool
	  operator() (const Node &obj1, const Node &obj2) const
	  {
		  return obj1.h+obj1.g < obj2.h+obj2.g;
	  }
	}minObj;
	
	/*
	 * Return type for the random planner (not using it currently)
	 * */
	struct returnType
	{
		bool t;
		vector<pair<int,int> > randPath;
	};

	vector<bool> foundInVisited (pair<int,int> nod, circular_buffer cb);
	vector<pair<int,int> > adjRandom(pair<int,int> node, vector<vector<int > > &world, circular_buffer visited);
	vector<pair<int,int> > randPlan(vector<vector<int> > &grid, pair<int,int> start, pair<int,int> goal);
	
	int heuris(pair<int,int> goal, pair<int,int> node);
	vector<pair<int,int> > adjAstar(pair<int,int> node, vector<vector<Node> > &world);
	vector<Node> aStar(vector<vector<Node> > &grid, pair<int,int> start, pair<int,int> goal);
	vector<pair<int,int> > optPlan(vector<vector<int> > &world, pair<int,int> start, pair<int,int> goal);
};

#endif
