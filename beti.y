/* Brandon Edens Tiny Flight Interpreter Parser.
 * 2015-04-22
 * Copyright 2015 by Brandon Edens. All Rights Reserved
 */
%{
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct waypoint {
	float lat;
	float lon;
	float alt;
	struct waypoint *next;
};

struct route {
	char *name;
	struct waypoint *waypoints;
	struct route *next;
};

struct route *routes;

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

/* Define a new route */
route: TOKEN_ROUTE TOKEN_SET IDENTIFIER '=' '{' waypoints '}'
	 | TOKEN_ROUTE TOKEN_REMOVE IDENTIFIER ';'
	 ;

/* fly route home-loop */
exec: TOKEN_EXEC TOKEN_ROUTE IDENTIFIER ';'
   | TOKEN_EXEC TOKEN_FLYTO TOKEN_ALT DECIMAL ';'
   | TOKEN_EXEC TOKEN_FLYTO TOKEN_COORD DECIMAL DECIMAL ';'
   | TOKEN_EXEC TOKEN_FLYTO TOKEN_COORD DECIMAL DECIMAL DECIMAL ';'
   ;

waypoints:
		 | waypoints waypoint
		 ;

/* waypoint lat lon alt */
waypoint: TOKEN_WAYPOINT TOKEN_FLYTO DECIMAL DECIMAL DECIMAL ';'
		| TOKEN_WAYPOINT TOKEN_LAND ';' 
		| TOKEN_WAYPOINT TOKEN_LOITER DECIMAL ';' 
		| TOKEN_WAYPOINT TOKEN_TAKEOFF ';' 
		;

