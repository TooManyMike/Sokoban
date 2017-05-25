#ifndef SOKOBAN_H
#define SOKOBAN_H

/* xsb format definitions */
#define MAN					'@'
#define MAN_ON_GOAL			'+'
#define BOX					'$'
#define BOX_ON_GOAL			'*'
#define GOAL				'.'
#define FLOOR				'-'
#define WALL				'#'

/* lurd format definitions */
#define LEFT				'l'
#define UP					'u'
#define RIGHT				'r'
#define DOWN				'd'
#define LEFT_MOVE_BOX		'L'
#define UP_MOVE_BOX			'U'
#define RIGHT_MOVE_BOX		'R'
#define DOWN_MOVE_BOX		'D'

/* return value of SokobanSolve */
#define MAP_ERROR			-1
#define NO_SOLUTION			-2
#define CANT_FIND_SOLUTION	-3

#endif