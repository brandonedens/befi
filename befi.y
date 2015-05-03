/* Brandon Edens Flight Interpreter Parser.
 * 2015-04-22
 * Copyright 2015 by Brandon Edens. All Rights Reserved
 */
%{

/*******************************************************************************
 * Include Files
 */

#include "drone.h"

/*******************************************************************************
 * Local Variables
 */

/*******************************************************************************
 * Local Functions
 */

/******************************************************************************/


%}

%token TOKEN_ALT TOKEN_COORD TOKEN_EXEC TOKEN_FLYTO TOKEN_LAND TOKEN_LOITER TOKEN_REMOVE TOKEN_ROUTE TOKEN_SET TOKEN_TAKEOFF TOKEN_WAYPOINT

%union {
	int    number;
	float  decimal;
	char  *string;
}

%token <decimal> DECIMAL
%token <number> NUMBER
%token <string> IDENTIFIER

%%

statements:
		  | statements statement
		  ;

statement: route
		 | exec
		 ;

route: TOKEN_ROUTE TOKEN_SET IDENTIFIER { drone_route_add(drone_default(), $3); } '=' '{' waypoints '}'
	 | TOKEN_ROUTE TOKEN_REMOVE IDENTIFIER ';' { drone_route_remove(drone_default(), $3); }
	 ;

exec: TOKEN_EXEC TOKEN_ROUTE IDENTIFIER ';' { drone_exec_route(drone_default(), $3); }
    | TOKEN_EXEC TOKEN_FLYTO TOKEN_ALT DECIMAL ';'
    | TOKEN_EXEC TOKEN_FLYTO TOKEN_COORD DECIMAL DECIMAL ';'
    | TOKEN_EXEC TOKEN_FLYTO TOKEN_COORD DECIMAL DECIMAL DECIMAL ';'
    ;

waypoints:
		 | waypoints waypoint
		 ;

waypoint: TOKEN_WAYPOINT TOKEN_FLYTO DECIMAL DECIMAL DECIMAL ';' { drone_route_waypoint_add_flyto(drone_default(), $3, $4, $5); }
		| TOKEN_WAYPOINT TOKEN_LAND ';' { drone_route_waypoint_add_land(drone_default()); }
		| TOKEN_WAYPOINT TOKEN_LOITER DECIMAL ';' { drone_route_waypoint_add_loiter(drone_default(), $3); }
		| TOKEN_WAYPOINT TOKEN_TAKEOFF ';' { drone_route_waypoint_add_takeoff(drone_default()); }
		;

