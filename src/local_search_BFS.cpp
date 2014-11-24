#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include <queue>
#include <stack>

#include <ctime>

#define QUEUE_SIZE 1000

#define FREE 0
#define OCCUPIED 100
#define UNVISITED false
#define VISITED true

#define HORIZONTAL_SIZE 1000
#define VERTICAL_SIZE 1000

class Local_search
{
public:

    Local_search(const ros::NodeHandle& n);

    void get_map();

    void load_search_data ( int startpos_index, int goalpos_index );

    void run();

private:

    struct node {
      int this_place;
      int came_from;
    } ;

    std::queue<node> openlist;
    std::stack<node> closedlist;

    int startpos_index, goalpos_index;

    ros::NodeHandle n_;

    ros::Subscriber occgrid_sub_;

    // called until it gets the map
    void occgridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    // map and corresponding map which follows which cells are already visited
    int map[ HORIZONTAL_SIZE * VERTICAL_SIZE ];
    bool visited_map[ HORIZONTAL_SIZE * VERTICAL_SIZE ];

    // flag which is used to show that map is loaded
    bool got_map;

    // backtracks through closed list to get the path
    void get_path();

    // appends upper cell, lower, right and left if it is possible (exists, is not visited and is not blocked) to open list
    void expand_node( node active_node );
};

Local_search::Local_search(const ros::NodeHandle &n)
    : n_(n)
{
    occgrid_sub_ = n_.subscribe("/map/occ_grid", QUEUE_SIZE,  &Local_search::occgridCallback, this);
}

void Local_search::occgridCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if( !got_map )
    {
        for( int i = 0; i < HORIZONTAL_SIZE * VERTICAL_SIZE ; i++ )
        {
            map[ i ] = msg->data[ i ];
        }
        got_map = true;
    }
}

void Local_search::get_map()
{
    // getting the map
    got_map = false;
    while( !got_map )
    {
        ros::spinOnce();
    }
    // at this point map is loaded
}

int main (int argc, char* argv[])
{


    // ** Init node
    ros::init(argc, argv, "Local_search");
    ros::NodeHandle n;

    // ** Create object
    Local_search ls(n);

    // loads the map into the map variable;
    ls.get_map();



    // using 1D array values - start position is n-th element and goal is m-th.
    ls.load_search_data(999, 5015); // 11, 88

    clock_t t1;

    // perform search operation
    ls.run();

    float diff = (double)(clock() - t1)/CLOCKS_PER_SEC ;
    std::cout << std::endl << "The time taken for Breadth first search: "<< diff << std::endl;
}

void Local_search::load_search_data( int startpos_index, int goalpos_index )
{
    this->startpos_index = startpos_index;
    this->goalpos_index = goalpos_index;
}

void Local_search::run()
{
    // create visited/unvisited array
    for( int i = 0; i < HORIZONTAL_SIZE * VERTICAL_SIZE; i++ )
    {
        visited_map[i] = false;
    }

    std::cout << "start pos: " << startpos_index << " goal: " << goalpos_index << std::endl;
    std::cout << "horizontal size: " << HORIZONTAL_SIZE << " vertical size: " << VERTICAL_SIZE << std::endl;

    // start search
    // push in openlist startpos node and when move to openlist - mark as visited
    openlist.push ( (node) {startpos_index, startpos_index} );
    visited_map[ startpos_index ] = VISITED;

    while( !openlist.empty() )
    {
        node active_node = openlist.front();
        openlist.pop();
        closedlist.push ( active_node );

        // check if not goal
        if( active_node.this_place == goalpos_index )
        {
            std::cout << "Path found!!!" << std::endl;
            get_path();
            break;
        }
        expand_node( active_node );
    }

    std::cout << "Exiting...\n";
}

void Local_search::expand_node( node active_node )
{
    // check right cell
    int new_place = active_node.this_place + 1;
    if( (new_place % HORIZONTAL_SIZE != 0) && (map[ new_place ] == FREE) && visited_map[ new_place ] == UNVISITED )
    {
        openlist.push ( (node) {new_place, active_node.this_place} );
        visited_map[ new_place ] = VISITED;
    }

    new_place = active_node.this_place - HORIZONTAL_SIZE;
    // check upper cell
    if( new_place >= 0 && map[ new_place ] == FREE && visited_map[ new_place ] == UNVISITED )
    {
        openlist.push ( (node) {new_place, active_node.this_place} );
        visited_map[ new_place ] = VISITED;
    }

    new_place = active_node.this_place - 1;
    // check left cell
    if( active_node.this_place % HORIZONTAL_SIZE != 0 && map[ new_place ] == FREE && visited_map[ new_place ] == UNVISITED )
    {
        openlist.push ( (node) {new_place, active_node.this_place} );
        visited_map[ new_place ] = VISITED;
    }

    new_place = active_node.this_place + HORIZONTAL_SIZE;
    // check lower cell
    if( new_place < HORIZONTAL_SIZE * VERTICAL_SIZE && map[ new_place ] == FREE && visited_map[ new_place ] == UNVISITED )
    {
        openlist.push ( (node) {new_place, active_node.this_place} );
        visited_map[ new_place ] = VISITED;
    }
}

void Local_search::get_path()
{
    int find_pos = goalpos_index;

    std::cout << "Path from goal to the start position: ";

    while( !closedlist.empty() )
    {
        node active_node = closedlist.top();
        closedlist.pop();
        if( active_node.this_place == find_pos )
        {
            find_pos = active_node.came_from;
            std::cout << active_node.this_place << " ";
        }
    }
}
