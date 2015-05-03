/** @file
 * Implementation of drone simulation.
 *
 *==============================================================================
 * Copyright 2015 by Brandon Edens. All Rights Reserved
 *==============================================================================
 *
 * @author  Brandon Edens
 * @date    2015-05-03
 * @details
 *
 */

/*******************************************************************************
 * Include Files
 */

#include "drone.h"

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 * Constants
 */

/*******************************************************************************
 * Local Types
 */

enum waypoint_type {
	WAYPOINT_TYPE_FLYTO = 0,
	WAYPOINT_TYPE_LAND,
	WAYPOINT_TYPE_LOITER,
	WAYPOINT_TYPE_TAKEOFF,
};

struct route {
	char name[128];
	struct waypoint *waypoints;
	struct waypoint *waypoints_tail;
	struct route *prev;
	struct route *next;
};

struct coord {
	float lat;
	float lon;
	float alt;
};

struct waypoint {
	enum waypoint_type type;

	union {
		struct coord flyto;

		struct loiter {
			float duration;
		} loiter;
	};

	struct waypoint *next;
};

struct drone {
	struct coord coord;
	float battery;
	struct route *routes;
	struct route *cur_route;
};

/*******************************************************************************
 * Macros
 */

/*******************************************************************************
 * Local Variables
 */

/*******************************************************************************
 * Global Variable Definitions
 */

/*******************************************************************************
 * Local Functions
 */

static struct drone *drone_alloc(void);
#if 0
static void drone_free(struct drone *d);
#endif
static void drone_init(struct drone *d);
static void route_add_waypoint_elem(struct route *r, struct waypoint *w);
static struct route *route_find(struct route const *routes, char const *name);

/******************************************************************************/

/** Allocate memory for a drone. */
static struct drone *drone_alloc(void)
{
	struct drone *d = calloc(1, sizeof(*d));
	assert(NULL != d);
	return d;
}

/** Return a handle to the default drone. */
struct drone *drone_default(void)
{
	static struct drone *d;
	if (NULL == d) {
		d = drone_alloc();
		drone_init(d);
	}
	return d;
}

/** Instruct drone to execute route. */
int drone_exec_route(struct drone *d, char const *name)
{
	printf("Flying route: %s\n", name);
	struct route const *r = route_find(d->routes, name);
	if (NULL == r) {
		fprintf(stderr, "Failure to execute route named: %s.\n", name);
		return EINVAL;
	}

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

	return 0;
}

#if 0
/** Free memory associated with a drone. */
static void drone_free(struct drone *d)
{
	free(d);
}
#endif

static void drone_init(struct drone *d)
{
	// By default our drone has a 3500 mAh battery.
	d->battery = 3500;
}

/** Add a route to the drone and set that route as currently selected. */
struct route *drone_route_add(struct drone *d, char const *name)
{
	struct route *r = route_find(d->routes, name);
	if (NULL != r) {
		// Route already exists.
		return r;
	}

	// Creating a new route.
	r = calloc(1, sizeof(struct route));
	if (NULL != d->routes) {
		d->routes->prev = r;
	}
	strncpy(r->name, name, sizeof(r->name) - 1);
	r->next = d->routes;
	d->routes = r;
	d->cur_route = r;

	return r;
}

/** Remove a route from the list of routes. */
int drone_route_remove(struct drone *d, char const *name)
{
	struct route *r = route_find(d->routes, name);
	if (NULL == r) {
		fprintf(stderr, "No route with the name: %s exists.", name);
		return EINVAL;
	}

	if (NULL == r->prev) {
		if (NULL != r->next) {
			r->next->prev = NULL;
		}
		d->routes = r->next;
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

	return 0;
}

/** Select the given route as the active route. */
int drone_route_select(struct drone *d, struct route *r)
{
	// Confirm that the given route exists.
	struct route *existing_route = route_find(d->routes, r->name);
	if (NULL == existing_route) {
		// No such route exists.
		return EINVAL;
	}

	d->cur_route = existing_route;
	return 0;
}

/** Add waypoint instructing drone to fly to a coordinate and altitude for
 * currently selected route.
 */
int drone_route_waypoint_add_flyto(struct drone *d, float lat, float lon,
                                    float alt)
{
	struct route * const r = d->cur_route;
	if (NULL == r) {
		// No current route selected; so unable to add a waypoint.
		fprintf(stderr,
		        "Cannot add flyto waypoint when no selected route.\n");
		return ENOENT;
	}
	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_FLYTO;
	w->flyto = (struct coord) {
		.lat = lat,
		.lon = lon,
		.alt = alt,
	};

	route_add_waypoint_elem(r, w);
	return 0;
}

/** Add waypoint instructing drone to land. */
int drone_route_waypoint_add_land(struct drone *d)
{
	if (NULL == d->cur_route) {
		// No current route selected; so unable to add a waypoint.
		fprintf(stderr,
		        "Cannot add land waypoint when no selected route.\n");
		return ENOENT;
	}
	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_LAND;
	route_add_waypoint_elem(d->cur_route, w);
	return 0;
}

/** Add waypoint instructing drone loiter in a region. */
int drone_route_waypoint_add_loiter(struct drone *d, float duration)
{
	if (NULL == d->cur_route) {
		// No current route selected; so unable to add a waypoint.
		fprintf(stderr,
		        "Cannot add loiter waypoint when no selected route.\n");
		return ENOENT;
	}
	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_LOITER;
	w->loiter.duration = duration;
	route_add_waypoint_elem(d->cur_route, w);
	return 0;
}

/** Add waypoint instructing drone to take off. */
int drone_route_waypoint_add_takeoff(struct drone *d)
{
	if (NULL == d->cur_route) {
		// No current route selected; so unable to add a waypoint.
		fprintf(stderr,
		        "Cannot add takeoff waypoint when no selected route.\n");
		return ENOENT;
	}
	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_TAKEOFF;
	route_add_waypoint_elem(d->cur_route, w);
	return 0;
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
