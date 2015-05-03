/** @file
 * Interface to drone simulation.
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
#ifndef DRONE_H_
#define DRONE_H_

/*******************************************************************************
 * Include Files
 */

/*******************************************************************************
 * Constants
 */

/*******************************************************************************
 * Global Types
 */

struct drone;
struct route;

/*******************************************************************************
 * Global Variables
 */

/*******************************************************************************
 * Macros
 */

/*******************************************************************************
 * Global Functions
 */

struct drone *drone_default(void);
int drone_exec_route(struct drone *d, char const *name);
struct route *drone_route_add(struct drone *d, char const *name);
int drone_route_remove(struct drone *d, char const *name);
int drone_route_select(struct drone *d, struct route *r);
int drone_route_waypoint_add_flyto(struct drone *d, float lat, float lon,
                                   float alt);
int drone_route_waypoint_add_land(struct drone *d);
int drone_route_waypoint_add_loiter(struct drone *d, float duration);
int drone_route_waypoint_add_takeoff(struct drone *d);

#endif  // DRONE_H_
