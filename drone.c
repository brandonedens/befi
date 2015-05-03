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
#include <math.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/*******************************************************************************
 * Constants
 */

/** The Earth radius in kilometers. */
#define EARTH_RADIUS 6371

/** The maximum velocity of the craft in meters per a second. */
#define MAX_VELOCITY 10

/** Number of meters per a kilometer. */
#define METER_PER_KM 1000

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
	/** Flag that marks the route in use. */
	pthread_mutex_t in_use;
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
	struct coord pos;
	float battery;
	float velocity;
	float time;
	struct route *routes;
	struct route *cur_route;

	/** Thread that represents drone simulation execution. */
	pthread_t exec;
	pthread_mutex_t exec_lock;
	/** The route the drone is currently flying. */
	struct route *flying_route;
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

static int dist_to_time(double dist);
static struct drone *drone_alloc(void);
#if 0
static void drone_free(struct drone *d);
#endif
static void drone_init(struct drone *d);
static double haversine_dist(struct coord a, struct coord b);
static void route_add_waypoint_elem(struct route *r, struct waypoint *w);
static struct route *route_find(struct route const *routes, char const *name);
static int route_waypoint_sanity_check(struct route *r);
static void *sim_drone_flyto(void *data);
static double to_radian(double deg);

/******************************************************************************/

/** Convert the given distance to seconds given the maximum velocity of the
 * drone. Note that this is extremely poor and doesn't take into account
 * acceleration.
 */
static int dist_to_time(double dist)
{
	return dist / MAX_VELOCITY;
}

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
	if (0 != pthread_mutex_trylock(&d->exec_lock)) {
		// Already executing a route. Cannot start another.
		fprintf(stderr, "Drone already executing a route.\n");
		return EBUSY;
	}

	printf("Flying route: %s\n", name);
	struct route *r = route_find(d->routes, name);
	if (NULL == r) {
		fprintf(stderr, "Failure to execute route named: %s.\n", name);
		return EINVAL;
	}

	// Start a thread that attempts to simulating flying through the route.
	d->flying_route = r;
	pthread_create(&d->exec, NULL, sim_drone_flyto, d);

	pthread_mutex_unlock(&d->exec_lock);
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
	// Random default coordinates set to the Berkeley marina.
	d->pos = (struct coord) {
		.lat = 37.873760,
		.lon = -122.320580,
		.alt = 0,
	};
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
		fprintf(stderr, "No route with the name: %s exists.\n", name);
		return EINVAL;
	}
	if (0 != pthread_mutex_trylock(&r->in_use)) {
		fprintf(stderr, "Cannot remove in use route %s.\n", name);
		return EBUSY;
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

	pthread_mutex_unlock(&r->in_use);
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
	int err = route_waypoint_sanity_check(r);
	if (0 != err) {
		return err;
	}

	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_FLYTO;
	w->flyto = (struct coord) {
		.lat = lat,
		.lon = lon,
		.alt = alt,
	};

	route_add_waypoint_elem(r, w);
	pthread_mutex_unlock(&r->in_use);
	return 0;
}

/** Add waypoint instructing drone to land. */
int drone_route_waypoint_add_land(struct drone *d)
{
	struct route * const r = d->cur_route;
	int err = route_waypoint_sanity_check(r);
	if (0 != err) {
		return err;
	}

	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_LAND;
	route_add_waypoint_elem(d->cur_route, w);
	pthread_mutex_unlock(&r->in_use);
	return 0;
}

/** Add waypoint instructing drone loiter in a region. */
int drone_route_waypoint_add_loiter(struct drone *d, float duration)
{
	struct route * const r = d->cur_route;
	int err = route_waypoint_sanity_check(r);
	if (0 != err) {
		return err;
	}

	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_LOITER;
	w->loiter.duration = duration;
	route_add_waypoint_elem(d->cur_route, w);
	pthread_mutex_unlock(&r->in_use);
	return 0;
}

/** Add waypoint instructing drone to take off. */
int drone_route_waypoint_add_takeoff(struct drone *d)
{
	struct route * const r = d->cur_route;
	int err = route_waypoint_sanity_check(r);
	if (0 != err) {
		return err;
	}

	struct waypoint *w = calloc(1, sizeof(struct waypoint));
	w->type = WAYPOINT_TYPE_TAKEOFF;
	route_add_waypoint_elem(d->cur_route, w);
	pthread_mutex_unlock(&r->in_use);
	return 0;
}

/** Compute the haversine distance between two coordinates in km.
 * Note that this doesn't take into account altitude differences.
 * See: https://en.wikipedia.org/wiki/Haversine_formula
 */
static double haversine_dist(struct coord x, struct coord y)
{
	double delta_lat = to_radian(y.lat - x.lat);
	double delta_lon = to_radian(y.lon - x.lon);

	double a = sin(delta_lat / 2) * sin(delta_lat / 2) +
		cos(to_radian(x.lat)) * cos(to_radian(y.lat)) *
		sin(delta_lon / 2) * sin(delta_lon / 2);
	double c = 2 * asin(fmin(1, sqrt(a)));
	double d = EARTH_RADIUS * c;
	return d;
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

/** Perform some initial checks to ensure that this route can be modified.
 * Note that the successful execution of this function will result in the
 * route being locked in_use.
 */
static int route_waypoint_sanity_check(struct route *r)
{
	if (NULL == r) {
		// No current route selected; so unable to add a waypoint.
		fprintf(stderr,
		        "Cannot add waypoint when no selected route.\n");
		return ENOENT;
	}
	if (0 != pthread_mutex_trylock(&r->in_use)) {
		fprintf(stderr, "Cannot modify in use route %s.\n", r->name);
		return EBUSY;
	}

	return 0;
}


/** Simulate a drone flying. */
static void *sim_drone_flyto(void *data)
{
	struct drone *d = (struct drone *)data;
	pthread_mutex_lock(&d->exec_lock);

	struct route *r = d->flying_route;
	pthread_mutex_lock(&r->in_use);

	for (struct waypoint *w = r->waypoints; NULL != w; w = w->next) {
		switch (w->type) {
		case WAYPOINT_TYPE_FLYTO: {
			double dist = haversine_dist(d->pos, w->flyto) *
				METER_PER_KM;
			// FIXME we compute the difference in altitude and
			// presume that the total distance traveled is either
			// the maximum lat/lon change or altitude change.
			double dist_alt = fabs(d->pos.alt - w->flyto.alt);
			dist = fmax(dist, dist_alt);
			printf("\tBegin flying to waypoint: lat: %f lon: %f alt: %f dist: %f\n",
				w->flyto.lat, w->flyto.lon, w->flyto.alt, dist);
			sleep(dist_to_time(dist));
			d->pos = w->flyto;
			printf("\tEnd flying.\n");
		}
			break;
		case WAYPOINT_TYPE_LAND:
			printf("\tBegin landing.\n");
			// Landing requires that we bring altitude back to 0.
			sleep(dist_to_time(d->pos.alt));
			d->pos.alt = 0;
			printf("\tEnd landing.\n");
			break;
		case WAYPOINT_TYPE_LOITER:
			printf("\tBegin loitering for %f secs\n", w->loiter.duration);
			sleep(w->loiter.duration);
			printf("\tEnd loitering.\n");
			break;
		case WAYPOINT_TYPE_TAKEOFF:
			printf("\tBegin taking off.\n");
			// Takeoff raises drone to 3 meters altitude.
			sleep(dist_to_time(3));
			d->pos.alt = 3;
			printf("\tEnd taking off.\n");
			break;
		default:
			// Route type not defined.
			assert(false);
		}
	}

	printf("Route %s completed.\n", r->name);
	pthread_mutex_unlock(&r->in_use);
	pthread_mutex_unlock(&d->exec_lock);
	return NULL;
}

/** Convert degrees to radians. */
static double to_radian(double deg)
{
	return (deg * M_PI) / 180;
}
