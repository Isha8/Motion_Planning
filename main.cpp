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

//extern int max_step_number;

/* **********************************************************
 * Test Input
 * Enter max_step_number for the random planner: 25 
 * Enter start positon: e.g: 2 0
 * 2 0 
 * Enter goal positon: e.g: 5 5
 * 5 5 
 * Enter world size: e.g: 6 6
 * 6 6
 *  0 0 1 0 0 0
	0 0 1 0 0 0
	0 0 0 0 1 0
	0 0 0 0 1 0
	1 0 1 1 1 0
	0 0 0 0 0 0
 * *************************************************************/

int main(void)
{
    motion_planner mp;
    int x,y,gx,gy,r,c;
    int val;
    //string vals;
    cout<<"Enter max_step_number for the random planner: ";
    cin>> mp.max_step_number;
    cout<<"\nEnter start positon: e.g: 2 1 \n";
    cin>>x>>y;
    cout<<"\nEnter goal positon: e.g: 5 4 \n";
    cin>>gx>>gy;
    cout<<"\nEnter world size: e.g: 6 6 \n";
    cin>>r>>c;
    pair<int,int> start = make_pair(x,y);
    pair<int,int> goal = make_pair(gx,gy);
    vector<vector<int> > world(r);
    //vector<vector<Node> > grid(r);
    for (int i = 0;i<r;i++)
    {
        for(int j = 0; j<c; j++)
        {
            cin>>val;
            world[i].push_back(val);
        }
    }    
    //cout<< world[4][0]<<"haaaaa"<<endl;

    //cout<< "Grid size"<< world.size()<<endl;
    
    vector<pair<int,int> >resRan = mp.randPlan(world,start,goal);
    //cout<<res.size()-1<<endl;
 
    pair<int,int> out;
    cout<< "Random Planner Path Traversed: (Showing position after start) "<<endl;
    for(int i= 0; i< resRan.size(); i++)
    {
        pair<int,int> out = resRan[i];
        cout<<out.first<<","<<out.second<<endl;
    }
   
    cout<< "\nOptimal Planner Path Traversed: (Showing position from start)"<<endl; 
    vector<pair<int,int> >resOpt = mp.optPlan(world,start,goal);
    //pair<int,int> out;
    for(int i=0; i < resOpt.size(); i++)
    {
        pair<int,int> out = resOpt[i];
        cout<<out.first<<","<<out.second<<endl;
    }
    //cout<<world[4][5]<<endl;
    //cout<<world.size();
    return 0;
}
