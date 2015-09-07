/*
 * waypoint.h
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

#ifndef NDT_WAYPOINT_H
#define NDT_WAYPOINT_H

#include "common/common.h"

typedef enum ndt_restrict
{
    NDT_RESTRICT_AB, // at or above
    NDT_RESTRICT_AT, // at restriction
    NDT_RESTRICT_BL, // at or below
    NDT_RESTRICT_NO, // no restriction
} ndt_restrict;

typedef struct ndt_restriction
{
    struct
    {
        ndt_restrict typ;
        ndt_distance alt;
    } altitude;

    struct
    {
        ndt_restrict typ;
        ndt_airspeed spd;
    } speed;

    enum
    {
        NDT_WPTCONST_NO,
        NDT_WPTCONST_FOV,
    } waypoint;

} ndt_restriction;

typedef struct ndt_waypoint
{
    ndt_info      info;      // identification information
    char          region[3]; // two-letter region/country code
    ndt_position  position;  // longitude, latitude and altitude
    ndt_frequency frequency; // associated navaid's frequency   (if applicable)
    ndt_distance  range;     // associated navaid's range       (if applicable)
    int           dme;       // associated navaid has a DME component

    enum
    {
        NDT_WPTYPE_APT, // airport (ICAO code or equivalent)
        NDT_WPTYPE_DME, // standalone DME
        NDT_WPTYPE_FIX, // RNAV fix
        NDT_WPTYPE_LLC, // latitude/longitude coordinates
        NDT_WPTYPE_LOC, // ILS localizer
        NDT_WPTYPE_NDB, // NDB radio beacon
        NDT_WPTYPE_RWY, // runway threshold
        NDT_WPTYPE_TOC, // top of climb   (pseudo-waypoint)
        NDT_WPTYPE_TOD, // top of descent (pseudo-waypoint)
        NDT_WPTYPE_VOR, // VOR radio beacon
    } type;
} ndt_waypoint;

ndt_waypoint* ndt_waypoint_init (                        );
void          ndt_waypoint_close(ndt_waypoint **_waypoint);
ndt_waypoint* ndt_waypoint_llc  (const char    *format   );
ndt_waypoint* ndt_waypoint_pbd  (ndt_waypoint  *place, double magbearing, ndt_distance distance, ndt_date date, void *wmm);

#endif /* NDT_WAYPOINT_H */
