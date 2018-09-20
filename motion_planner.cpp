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
#include <motion_planner.h>


using namespace std;

/* ***************************************************************************************************************************
 * Function to check which of the 4 adjacent cells of the current cell are in the visited list
 * 
 * @output: returns a boolean vector of size 4
 * @input: current cell, buffer of visited nodes( size- square root of max_step_number) 
 * ***************************************************************************************************************************/
vector<bool> motion_planner::foundInVisited (pair<int,int> nod, circular_buffer cb)
{
	vector<bool> boolAdj {true, true, true, true};
	int x = nod.first;
	int y = nod.second;
 
	for (int i = 0 ; i< cb.size(); i++)
	{
		//int r = cb[i].first(), c = cb[i].second();
		if(make_pair(x+1,y) == cb[i])
		{
			boolAdj[0] = false ;
		}
		if(make_pair(x,y+1) == cb[i])
		{
			boolAdj[1] = false;
		}
		if(make_pair(x-1,y) == cb[i])
		{
			boolAdj[2] = false;
		}
		if(make_pair(x,y-1) == cb[i])
		{
			boolAdj[3] = false;
		}
		
	}
	return boolAdj;
}

/* ***************************************************************************************************************************
 * Function to find which adjacent nodes of the current nodes are passable
 * 
 * @output: vector of passable nodes
 * @input: current node, the world state, buffer of visited nodes( size- square root of max_step_number) 
 * ***************************************************************************************************************************/
vector<pair<int,int> > motion_planner::adjRandom(pair<int,int> node, vector<vector<int > > &world, circular_buffer visited)
{
	vector<pair<int,int> > resNodes;
	//fromVisited = foundInVisited (node, visited);
	int x = node.first, y = node.second;
	bool t1 = false,t2=false,t3=false,t4 = false;
	vector<bool> resAdj = foundInVisited(node, visited);
	if ( (x+1) <= world.size()-1 && world[x+1][y] != 1 )
	{
		t1 = true;
		if(resAdj[0])
			resNodes.push_back(make_pair(x+1,y));
	}
	if ( (y+1) <= world[0].size()-1 && world[x][y+1] != 1 )
	{
		t2 = true;
		if(resAdj[1])
			resNodes.push_back(make_pair(x,y+1));
	}
	if ( (x-1) >= 0 && world[x-1][y] != 1 )
	{
		t3 = true;
		if(resAdj[2])
			resNodes.push_back(make_pair(x-1,y));
	}
	if ( (y-1) >= 0 && world[x][y-1] != 1 )
	{
		t4 = true;
		if(resAdj[3])
			resNodes.push_back(make_pair(x,y-1));
	}
	if(t1 == false && t2 ==t1 && t3 == t2 && t4 ==t1)
	{
		return resNodes;
	}
	else if (resNodes.size()==0)
	{
		vector<pair<int,int> > ts;
		if(t1 == true)  ts.push_back(make_pair(x+1,y));
		if(t2 == true)  ts.push_back(make_pair(x,y+1));
		if(t3 == true)  ts.push_back(make_pair(x-1,y));
		if(t4 == true)  ts.push_back(make_pair(x,y-1));
		random_shuffle(ts.begin(),ts.end());
		resNodes.push_back(*ts.begin());
		return resNodes;
	}
	else
		return resNodes;
 
}


/* *************************************************************************************************************************
 * Function implements random planner logic, uses a circular buffer to store the last visited nodes, random_shuffle to select next node from the vector of adjacent passable nodes, random)shuffle has no bias as opposed to rand, and size of this vector is always less than or equal to 4, time efficient. For a bigger vector, a random generator should be created to get a random element.
 * 
 * @output: vector of nodes traversed (size max_step_number or less)
 * @input: world state, start state, goal state
 * ************************************************************************************************************************/
vector<pair<int,int> > motion_planner::randPlan(vector<vector<int> > &grid, pair<int,int> start, pair<int,int> goal)
{
	vector<pair<int,int> > path;
	unsigned int squarert = abs(sqrt(max_step_number));
	circular_buffer visited(squarert);
	int sx=start.first, sy=start.second, gx=goal.first, gy=goal.second;
	int minF = 10000;
	pair<int,int> current = start;
	//path.push_back(current);
	visited.push_back(current);
	vector<pair<int,int> > childNodes;
	srand ( unsigned ( std::time(0) ) );
	if (current == goal)
	{
		return {current};
	}
	while(max_step_number-- > 0)
	{
		if (current == goal)
		{	
			cout<< "\n Path found with random planner!!!\n"<<endl;
			return path;
		}
		childNodes = adjRandom(current,grid,visited);
		random_shuffle(childNodes.begin(),childNodes.end()); 
		current = *childNodes.begin();
		visited.push_back(current);
		path.push_back(current);
	} 
	if(path[path.size()-1]!=goal) cout<<"\nGoal not reached using Random Planner(Max steps reached)!!!\n"<<endl;
	return path;
}


/* *************************************************************************************************************************
 * Function calculates heuristic of each node. Euclidean distance is used to calcualte the cost from current node to goal
 * 
 * @output: integer value heuristic
 * @input: goal state, current state
 * ************************************************************************************************************************/

int motion_planner::heuris(pair<int,int> goal, pair<int,int> node)
{
	int h = abs(goal.first-node.first) + abs(goal.second-node.second);
	return h;
}


/* *************************************************************************************************************************
 * Function returns the adjacent node positions of the current node that are passable for the A* search 
 * 
 * @output: vector of pairs(positions) of adjacent passable nodes 
 * @input: current position, world state
 * ************************************************************************************************************************/

vector<pair<int,int> > motion_planner::adjAstar(pair<int,int> node, vector<vector<motion_planner::Node> > &world)
{
	vector<pair<int,int> > adjNodes;
	int x = node.first, y = node.second;
	if ( (x+1) <= world.size()-1 && world[x+1][y].val != 1)
	{
 
		adjNodes.push_back(make_pair(x+1,y));
 
	}
	if ( (y+1) <= world[0].size()-1 && world[x][y+1].val != 1)
	{
		adjNodes.push_back(make_pair(x,y+1));
	}
	if ( (x-1) >= 0 && world[x-1][y].val != 1 )
	{

		adjNodes.push_back(make_pair(x-1,y));
	}
	if ( (y-1) >= 0 && world[x][y-1].val != 1 )
	{
		adjNodes.push_back(make_pair(x,y-1));
	}
	return adjNodes;
}


/* *************************************************************************************************************************
 * Function implements A* search logic, returns a vector of Node objects that are traversed.
 * 
 * @output: vector of nodes traversed
 * @input: 2D list of Node objects, start state, goal state
 * ************************************************************************************************************************/
vector<motion_planner::Node> motion_planner::aStar(vector<vector<motion_planner::Node> > &grid, pair<int,int> start, pair<int,int> goal)
{
	int sx=start.first, sy=start.second, gx=goal.first, gy=goal.second;
	int minF = 10000;
	
	unordered_set<motion_planner::Node,motion_planner::NodeHasher,motion_planner::NodeComparator> openList, closedList;
	//cout<< "grid size"<< grid[0].size()<<endl;
	openList.insert(grid[sx][sy]);
	//cout<<gx<<" "<<gy<<endl;
	
	vector<motion_planner::Node > path;
	int hihi=0;
	int i =0;
	while (!openList.empty())
	{
		i++;
		//cout<<"its here"<<endl;
		Node current = *min_element(openList.begin(), openList.end(), minObj);
		/*cout<<i<<endl;
		for (Node x : openList)
		{
			cout<<"h="<< x.h<< "g=" <<x.g << " "<< x.get_pos().first << "," << x.get_pos().second << "\t";
	  
		}
		cout<< endl;
		cout<< current.get_pos().first<<" "<<current.get_pos().second<<endl; */
		int currX=current.get_pos().first, currY = current.get_pos().second;
		if (current.get_pos() == goal)
		{
			//cout<<"here";
			while(current.parent != NULL )
			{
			   
				//cout<< current.get_pos().first<<" "<<current.get_pos().second<<endl;
				path.push_back(current);
				current = *current.parent;				
			}
			path.push_back(current);
			return path;
		}
		openList.erase(current);
		closedList.insert(current);
		//openList.erase(grid[currX][currY]);

		for (pair<int,int> nod : adjAstar(current.pos,grid))
		{
			hihi++;
			int r=nod.first, c = nod.second;
			//auto searchCL = find(closedList.begin(),closedList.end(),nod);
			auto searchCL = closedList.find(grid[r][c]);
			if(searchCL!=closedList.end())
				continue;
  
			auto searchOL = openList.find(grid[r][c]);
			if(searchOL!=openList.end())
			{
				//cout<<"hoorah"<<endl;
				int n = current.g+current.move_cost(grid[r][c]);
				if(grid[r][c].g>n)
				{
					grid[r][c].g=n;
					int a = current.get_pos().first; int b = current.get_pos().second;
					grid[r][c].parent = &grid[a][b];
				}
			}

			else
			{		  
				grid[r][c].g = current.g + current.move_cost(grid[r][c]);
				grid[r][c].h = heuris(goal,grid[r][c].pos);
				//cout<<&current<<endl;
				int a = current.get_pos().first; int b = current.get_pos().second;
				//cout<<"a "<<grid[r][c].get_pos().first<<" b "<<grid[r][c].get_pos().second<<endl;
				//cout<<"position of parent "<<grid[a][b].get_pos().first<<","<<grid[a][b].get_pos().second<<endl;
				grid[r][c].parent = &grid[a][b];
				openList.insert(grid[r][c]);		  
			}
  
		 }
  
	 }
	//cout<<hihi;
	return path;
 
}


/* *************************************************************************************************************************
 * Function to get the positions of the nodes returned by the function aStar.
 * 
 * @output: vector of positions(pairs) of nodes traversed 
 * @input: world state, start state, goal state
 * ************************************************************************************************************************/
vector<pair<int,int> > motion_planner::optPlan(vector<vector<int> > &world, pair<int,int> start, pair<int,int> goal)
{
	vector<vector<motion_planner::Node> > grid(world.size());
	vector<pair<int,int> > res;
	vector<motion_planner::Node > path;
	for (int i = 0; i<world.size() ; i++)
    {
        for(int j =0; j<world[0].size(); j++)
        { 
            //Node* a = new Node(make_pair(i,j), world[i][j]);
            motion_planner::Node a = motion_planner::Node(make_pair(i,j),world[i][j]);
            grid[i].push_back(a);
        }
    }
    
    path = motion_planner::aStar(grid,start,goal);
    for(int i= path.size()-1; i>=0; i--)
    {
        res.push_back(path[i].get_pos());
    } 
	return res;
 
}

