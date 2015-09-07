/*
 * airport.h
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2014 Timothy D. Walker and others.
 *
 * All rights reserved. This program and the accompanying materials are made
 * available under the terms of the GNU General Public License (GPL) version 2
 * which accompanies this distribution (LICENSE file), and is also available at
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * Contributors:
 *     Timothy D. Walker
 */

#ifndef NDT_AIRPORT_H
#define NDT_AIRPORT_H

#include "common/common.h"
#include "common/list.h"

#include "waypoint.h"

typedef struct ndt_airport
{
    ndt_info      info;        // identification information
    ndt_position  coordinates; // coordinates of airport
    ndt_distance  tr_altitude; // transition altitude
    ndt_distance  trans_level; // transition level
    ndt_distance  rwy_longest; // longest runway
    ndt_waypoint *waypoint;    // associated waypoint
    ndt_list     *runways;     // available runways (ndt_runway)
} ndt_airport;

ndt_airport* ndt_airport_init (                      );
void         ndt_airport_close(ndt_airport **_airport);

typedef struct ndt_runway
{
    ndt_info      info;       // identification information
    ndt_distance  width;      // width
    ndt_distance  length;     // length
    int           heading;    // heading
    ndt_distance  overfly;    // threshold overflying height
    ndt_position  threshold;  // threshold coordinates
    ndt_waypoint *waypoint;   // associated waypoint

    struct
    {
        int           avail;  // availability
        int           course; // runway heading
        double        slope;  // glideslope angle
        ndt_info      info;   // ILS identifier
        ndt_frequency freq;   // ILS frequency
    } ils;

    enum
    {
        NDT_RWYUSE_CLOSED,    // closed (unused)
        NDT_RWYUSE_LDGTOF,    // landing and takeoff
        NDT_RWYUSE_LANDNG,    // landing only
        NDT_RWYUSE_TAKEOF,    // takeoff only
    } status;

    enum
    {
        NDT_RWYSURF_OTHER,    // other/unknown
        NDT_RWYSURF_ASPHT,    // asphalt, bitumen
        NDT_RWYSURF_CONCR,    // concrete
        NDT_RWYSURF_GRASS,    // grass
        NDT_RWYSURF_GRAVL,    // gravel, coral, ice
        NDT_RWYSURF_WATER,    // water
    } surface;

    void *exits; // reserved for future use
} ndt_runway;

ndt_runway* ndt_runway_init (                                         );
void        ndt_runway_close(ndt_runway  **_runway                    );
ndt_runway* ndt_runway_get  (ndt_airport  *airport, const char *runway);

#endif /* NDT_AIRPORT_H */
