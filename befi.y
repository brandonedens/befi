/* Brandon Edens Flight Interpreter Parser.
 * 2015-04-22
 * Copyright 2015 by Brandon Edens. All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
%{

/*******************************************************************************
 * Include Files
 */
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>

#include "drone.h"

/*******************************************************************************
 * Global Variables
 */

extern bool shutdown;

/*******************************************************************************
 * Local Functions
 */

/******************************************************************************/


%}

%token TOKEN_ALT TOKEN_COORD TOKEN_EXEC TOKEN_EXIT TOKEN_FLYTO TOKEN_LAND TOKEN_LOITER TOKEN_REMOVE TOKEN_ROUTE TOKEN_SET TOKEN_SLEEP TOKEN_TAKEOFF TOKEN_WAYPOINT

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
		 | exit
		 | sleep
		 ;

exec: TOKEN_EXEC TOKEN_ROUTE IDENTIFIER ';' { drone_exec_route(drone_default(), $3); }
    | TOKEN_EXEC TOKEN_FLYTO TOKEN_ALT DECIMAL ';'
    | TOKEN_EXEC TOKEN_FLYTO TOKEN_COORD DECIMAL DECIMAL ';'
    | TOKEN_EXEC TOKEN_FLYTO TOKEN_COORD DECIMAL DECIMAL DECIMAL ';'
    ;

exit: TOKEN_EXIT ';' { exit(0); /* XXX This is not graceful. */ }

route: TOKEN_ROUTE TOKEN_SET IDENTIFIER { drone_route_add(drone_default(), $3); } '=' '{' waypoints '}'
	 | TOKEN_ROUTE TOKEN_REMOVE IDENTIFIER ';' { drone_route_remove(drone_default(), $3); }
	 ;

sleep: TOKEN_SLEEP NUMBER ';' { sleep($2); }

waypoints:
		 | waypoints waypoint
		 ;

waypoint: TOKEN_WAYPOINT TOKEN_FLYTO DECIMAL DECIMAL DECIMAL ';' { drone_route_waypoint_add_flyto(drone_default(), $3, $4, $5); }
		| TOKEN_WAYPOINT TOKEN_LAND ';' { drone_route_waypoint_add_land(drone_default()); }
		| TOKEN_WAYPOINT TOKEN_LOITER DECIMAL ';' { drone_route_waypoint_add_loiter(drone_default(), $3); }
		| TOKEN_WAYPOINT TOKEN_TAKEOFF ';' { drone_route_waypoint_add_takeoff(drone_default()); }
		;

