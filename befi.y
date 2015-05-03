/* Brandon Edens Flight Interpreter Parser.
 * 2015-04-22
 * Copyright 2015 by Brandon Edens. All Rights Reserved
 */
%{

/*******************************************************************************
 * Include Files
 */
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 * Local Types
 */

enum waypoint_type {
	WAYPOINT_TYPE_FLYTO = 0,
	WAYPOINT_TYPE_LAND,
	WAYPOINT_TYPE_LOITER,
	WAYPOINT_TYPE_TAKEOFF,
};

struct waypoint {
	enum waypoint_type type;

	union {
		struct flyto {
			float lat;
			float lon;
			float alt;
		} flyto;

		struct loiter {
			float duration;
		} loiter;
	};

	struct waypoint *next;
};

struct route {
	char name[128];
	struct waypoint *waypoints;
	struct waypoint *waypoints_tail;
	struct route *prev;
	struct route *next;
};

/*******************************************************************************
 * Local Variables
 */

static struct route *routes_head;
static struct route *cur_route;

/*******************************************************************************
 * Local Functions
 */

static void exec_route(struct route *r);
static struct route *route_add(struct route **routes, char const *name);
static struct route *route_find(struct route const *routes, char const *name);
static void route_remove(struct route **routes, struct route *r);
static void route_waypoint_add_flyto(struct route *r, float lat, float lon, float alt);

/******************************************************************************/

/** Execute the given route. */
static void exec_route(struct route *r)
{
	printf("Flying route: %s\n", r->name);
	for (struct waypoint *w = r->waypoints; NULL != w; w = w->next) {
		switch (w->type) {
		case WAYPOINT_TYPE_FLYTO:
			printf("\tFlying to waypoint: lat: %f lon: %f alt: %f\n",
				w->flyto.lat, w->flyto.lon, w->flyto.alt);
			break;
		case WAYPOINT_TYPE_LAND:
			printf("\tLanding\n");
			break;
		case WAYPOINT_TYPE_LOITER:
			printf("\tLoitering for %f secs\n", w->loiter.duration);
			break;
		case WAYPOINT_TYPE_TAKEOFF:
			printf("\tTaking off\n");
			break;
		default:
			// Route type not defined.
			assert(false);
		}
	}
}

/** Add a route to the system. */
static struct route *route_add(struct route **routes, char const *name)
{
	struct route *r = route_find(*routes, name);
	if (NULL != r) {
		return r;
	}
	r = calloc(1, sizeof(struct route));
	if (NULL != *routes) {
		(*routes)->prev = r;
	}
	strncpy(r->name, name, sizeof(r->name) - 1);
	r->next = *routes;
	*routes = r;
	return r;
}

/** Find a route in the list of routes. */
static struct route *route_find(struct route const *routes, char const *name)
{
	for (struct route const *r = routes; NULL != r; r = r->next) {
		if (0 == strcmp(name, r->name)) {
			return (struct route *)r;
		}
	}
	return NULL;
}

/** Remove a route from the list of routes. */
static void route_remove(struct route **routes, struct route *r)
{
	if (NULL == r) {
		return;
	}
	if (NULL == r->prev) {
		if (NULL != r->next) {
			r->next->prev = NULL;
		}
		*routes = r->next;
	} else {
		r->prev->next = r->next;
		r->next->prev = r->prev;
	}
	// Remove all waypoints.
	struct waypoint *w = r->waypoints;
	while (NULL != w)
	{
		struct waypoint *w_next = w->next;
		free(w);
		w = w_next;
	}
	free(r);
}

/** Route add waypoint element to the route. */
static void route_add_waypoint_elem(struct route *r, struct waypoint *w)
{
	if (NULL == r->waypoints) {
		r->waypoints = w;
		r->waypoints_tail = w;
		return;
	}

	r->waypoints_tail->next = w;
	r->waypoints_tail = w;
}

/** Add waypoint instructing drone to fly to a coordinate and altitude. */
static void route_waypoint_add_flyto(struct route *r, float lat, float lon, float alt)
{
	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_FLYTO;
	w->flyto = (struct flyto) {
		.lat = lat,
		.lon = lon,
		.alt = alt,
	};

	route_add_waypoint_elem(r, w);
}

/** Add waypoint instructing drone to land. */
static void route_waypoint_add_land(struct route *r)
{
	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_LAND;
	route_add_waypoint_elem(r, w);
}

/** Add waypoint instructing drone loiter in a region. */
static void route_waypoint_add_loiter(struct route *r, float duration)
{
	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_LOITER;
	w->loiter.duration = duration;
	route_add_waypoint_elem(r, w);
}

/** Add waypoint instructing drone to take off. */
static void route_waypoint_add_takeoff(struct route *r)
{
	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_TAKEOFF;
	route_add_waypoint_elem(r, w);
}

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

route: TOKEN_ROUTE TOKEN_SET IDENTIFIER { struct route *r = route_add(&routes_head, $3); cur_route = r; } '=' '{' waypoints '}'
	 | TOKEN_ROUTE TOKEN_REMOVE IDENTIFIER ';'
	 {
		struct route *r = route_find(routes_head, $3);
		route_remove(&routes_head, r);
	 }
	 ;

exec: TOKEN_EXEC TOKEN_ROUTE IDENTIFIER ';'
	{
		struct route *r = route_find(routes_head, $3);
		if (NULL == r) {
			printf("ERROR: Unable to fly to route: %s\n", $3);
		} else {
			exec_route(r);
		}
	}
    | TOKEN_EXEC TOKEN_FLYTO TOKEN_ALT DECIMAL ';'
    | TOKEN_EXEC TOKEN_FLYTO TOKEN_COORD DECIMAL DECIMAL ';'
    | TOKEN_EXEC TOKEN_FLYTO TOKEN_COORD DECIMAL DECIMAL DECIMAL ';'
    ;

waypoints:
		 | waypoints waypoint
		 ;

waypoint: TOKEN_WAYPOINT TOKEN_FLYTO DECIMAL DECIMAL DECIMAL ';' { route_waypoint_add_flyto(cur_route, $3, $4, $5); }
		| TOKEN_WAYPOINT TOKEN_LAND ';' { route_waypoint_add_land(cur_route); }
		| TOKEN_WAYPOINT TOKEN_LOITER DECIMAL ';' { route_waypoint_add_loiter(cur_route, $3); }
		| TOKEN_WAYPOINT TOKEN_TAKEOFF ';' {route_waypoint_add_takeoff(cur_route); }
		;

