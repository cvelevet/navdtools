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

typedef enum ndt_acftype
{
    NDT_ACFTYPE_ALL, // unknown/all
    NDT_ACFTYPE_JET, // jet engine
    NDT_ACFTYPE_NON, // none
    NDT_ACFTYPE_OTH, // other
    NDT_ACFTYPE_TBP, // turboprop
} ndt_acftype;

typedef enum ndt_restrict
{
    NDT_RESTRICT_AB, // at or above min
    NDT_RESTRICT_AT, // at w/min == max
    NDT_RESTRICT_BL, // at or below max
    NDT_RESTRICT_BT, // between min-max
    NDT_RESTRICT_NO, // no restriction
} ndt_restrict;

typedef struct ndt_restriction
{
    struct
    {
        ndt_restrict typ;
        ndt_distance min;
        ndt_distance max;
    } altitude;

    struct
    {
        ndt_restrict typ;
        ndt_acftype  acf;
        ndt_airspeed min;
        ndt_airspeed max;
    } airspeed;

    enum
    {
        NDT_WPTCONST_FAF,
        NDT_WPTCONST_FOV,
        NDT_WPTCONST_IAF,
        NDT_WPTCONST_MAP,
        NDT_WPTCONST_NO,
    } waypoint;

    enum
    {
        NDT_TURN_LEFT,
        NDT_TURN_RIGHT,
        NDT_TURN_SHORT,
    } turn;

} ndt_restriction;

typedef struct ndt_waypoint
{
    ndt_info      info;      // identification information
    char          region[3]; // two-letter region/country code
    ndt_position  position;  // longitude, latitude and altitude
    ndt_frequency frequency; // associated navaid's frequency   (if applicable)
    ndt_distance  range;     // associated navaid's range       (if applicable)
    int           dme;       // associated navaid has a DME component

    union
    {
        struct
        {
            struct ndt_waypoint *place;
            double             bearing;
            ndt_distance      distance;
        } pbd;

        struct
        {
            struct ndt_waypoint *wpt1;
            double               brg1;
            struct ndt_waypoint *wpt2;
            double               brg2;
        } pbpb;

        struct
        {
            struct ndt_waypoint  *place;
            double              bearing;
            struct ndt_waypoint *navaid;
            ndt_distance       distance;
        } pbpd;
    };

    // sorted as per X-Plane earth_nav.dat format
    enum
    {
        NDT_WPTYPE_APT =  1, // airport (ICAO code or equivalent)
        NDT_WPTYPE_NDB =  2, // NDB radio beacon
        NDT_WPTYPE_VOR =  3, // VOR radio beacon
//      NDT_WPTYPE_ILS =  4, // localizer (ILS) (use the next one for all LOCs)
        NDT_WPTYPE_LOC =  5, // localizer (standalone)
        NDT_WPTYPE_FIX = 11, // RNAV fix
        NDT_WPTYPE_DME = 13, // standalone DME
        NDT_WPTYPE_LLC = 28, // latitude/longitude coordinates
        NDT_WPTYPE_RWY = 31, // runway threshold                    (not XP type)
        NDT_WPTYPE_PBD = 41, // place/bearing/distance              (not XP type)
        NDT_WPTYPE_PPB = 42, // place/bearing, place/bearing        (not XP type)
        NDT_WPTYPE_PPD = 43, // place/bearing, place/distance       (not XP type)
        NDT_WPTYPE_TOC = 51, // top of climb   (pseudo-waypoint)    (not XP type)
        NDT_WPTYPE_TOD = 52, // top of descent (pseudo-waypoint)    (not XP type)
    } type;
} ndt_waypoint;

ndt_waypoint* ndt_waypoint_init (                        );
void          ndt_waypoint_close(ndt_waypoint **_waypoint);
ndt_waypoint* ndt_waypoint_llc  (const char    *format   );
ndt_waypoint* ndt_waypoint_pbd  (ndt_waypoint  *place, double magbearing, ndt_distance distance,                    ndt_date date, void *wmm);
ndt_waypoint* ndt_waypoint_pbpb (ndt_waypoint  *plce1, double magneticb1, ndt_waypoint   *plce2, double magneticb2, ndt_date date, void *wmm);
ndt_waypoint* ndt_waypoint_pbpd (ndt_waypoint  *plce1, double magbearing, ndt_waypoint   *plce2, ndt_distance dist, ndt_date date, void *wmm);

#endif /* NDT_WAYPOINT_H */
