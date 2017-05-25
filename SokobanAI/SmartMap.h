#ifndef SMARTMAP_H
#define SMARTMAP_H

#include "Grid.h"

bool check_map_closure(bool**, int, int, Grid);

/***
 * get connectivity areas of map
 * connected grids share a same flag, disconnected grids have different flags
 * connectivity flags: '1', '2',...
 * wall flag: '#'
***/
void get_connectivity_map(char**, bool**, int, int);

/* get area connected with a specified grid */
void get_reachability_map(bool**, bool**, int, int, Grid);

/***
 * overload get_reachability_map
 * get minimum steps for a man starting from each grid of the map to reach a specified grid
 * -1 represents unreachable
***/
void get_reachability_map(int**, bool**, int, int, Grid);

/***
 * get minimum steps for a box starting from each grid of the map to reach a specified goal
 * LARGE represents unreachable
***/
void get_goal_h_map(unsigned char**, bool**, int, int, Grid);

#endif