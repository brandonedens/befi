/** @file
 * Interface to drone simulation.
 *
 *==============================================================================
 * Copyright 2015 by Brandon Edens. All Rights Reserved
 *==============================================================================
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
