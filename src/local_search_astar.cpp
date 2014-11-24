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

    void load_search_data ( int startpos_index, int goalpos_index );

    void run();

    void get_map();

private:

    struct node {
      int this_place;
      int came_from;
      int dist_from_start;
      int priority_score;
    };

    struct openlistsort{
        bool operator()(const node& a, const node& b){
            return a.priority_score>b.priority_score;
        }
    };

    std::priority_queue<node, std::vector<node>, openlistsort> openlist;
    std::stack<node> closedlist;

    int get_distance(int position_index);

    int startpos_index, goalpos_index;

    ros::NodeHandle n_;

    ros::Subscriber occgrid_sub_;

    // called until it gets the map
    void occgridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    int map[ HORIZONTAL_SIZE * VERTICAL_SIZE ];
    bool visited_map[ HORIZONTAL_SIZE * VERTICAL_SIZE ];

    bool got_map;

    void get_path();

    void start_search();

    void expand_node( node active_node );

};


int Local_search::get_distance(int position_index)
{
    // distance from position to goal (not taking into accout obstacles)
    // Manhattan distance
    int xdiff, ydiff;

    int x1dist = position_index % HORIZONTAL_SIZE;
    int y1dist = position_index / HORIZONTAL_SIZE;

    int x2dist = goalpos_index % HORIZONTAL_SIZE;
    int y2dist = goalpos_index / HORIZONTAL_SIZE;

    xdiff = abs( x1dist - x2dist );
    ydiff = abs( y1dist - y2dist );

    return xdiff + ydiff;
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

Local_search::Local_search(const ros::NodeHandle &n)
    : n_(n)
{

    occgrid_sub_ = n_.subscribe("/map/occ_grid", QUEUE_SIZE,  &Local_search::occgridCallback, this);

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

    // using 1D array values - start position is 11th element and goal is 88th.
    // start position (2,2), goal position (9,9) (2nd row, 2nd column and 9th row, 9th column)
    ls.load_search_data(999,5015); // 11, 88

    clock_t t1;

    // performs search operation
    ls.run();

    float diff = (double)(clock() - t1)/CLOCKS_PER_SEC ;
    std::cout << std::endl << "The time taken for Astar search: "<< diff << std::endl;
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

    // start search
    start_search();

    std::cout << "Exiting...\n";
}

void Local_search::start_search()
{
    int g = 0;
    int h = get_distance(startpos_index);
    openlist.push ( (node) {startpos_index, -1, g, g+h} );
    visited_map[ startpos_index ] = VISITED;

    while( !openlist.empty() )
    {
        node active_node = openlist.top();
        openlist.pop();
        closedlist.push ( active_node );


        // check if not goal
        if( active_node.this_place == goalpos_index )
        {
            std::cout << "Goal found!!!" << std::endl;
            get_path();
            break;
        }

        expand_node( active_node );
    }
}

void Local_search::expand_node( node active_node )
{
    int new_g;
    int new_h;
    // check right cell
    // check if position exists
    int new_place = active_node.this_place + 1;

    if( (new_place % HORIZONTAL_SIZE != 0) && (map[ new_place ] == FREE) && visited_map[ new_place ] == UNVISITED )
    {
        new_g = active_node.dist_from_start+1;
        new_h = get_distance(new_place);
        // push in openlist
        openlist.push ( (node) {new_place, active_node.this_place, new_g, new_g+new_h} );
        // mark as visited
        visited_map[ new_place ] = VISITED;
    }



    new_place = active_node.this_place - HORIZONTAL_SIZE;
    // check upper cell
    if( new_place >= 0 && map[ new_place ] == FREE && visited_map[ new_place ] == UNVISITED )
    {
        new_g = active_node.dist_from_start+1;
        new_h = get_distance(new_place);
        // push in openlist
        openlist.push ( (node) {new_place, active_node.this_place, new_g, new_g+new_h} );
        // mark as visited
        visited_map[ new_place ] = VISITED;
    }


    new_place = active_node.this_place - 1;
    // check left cell
    // if map index doesn't go to previous row, position exists
    if( active_node.this_place % HORIZONTAL_SIZE != 0 && map[ new_place ] == FREE && visited_map[ new_place ] == UNVISITED )
    {
        new_g = active_node.dist_from_start+1;
        new_h = get_distance(new_place);
        // push in openlist
        openlist.push ( (node) {new_place, active_node.this_place, new_g, new_g+new_h} );
        // mark as visited
        visited_map[ new_place ] = VISITED;
    }


    new_place = active_node.this_place + HORIZONTAL_SIZE;
    // check lower cell
    if( new_place < HORIZONTAL_SIZE * VERTICAL_SIZE && map[ new_place ] == FREE && visited_map[ new_place ] == UNVISITED )
    {
        new_g = active_node.dist_from_start+1;
        new_h = get_distance(new_place);
        // push in openlist
        openlist.push ( (node) {new_place, active_node.this_place, new_g, new_g+new_h} );
        // mark as visited
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
    std::cout << std::endl;
}
