/*
 * flightplan.h
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

#ifndef NDT_FLIGHTPLAN_H
#define NDT_FLIGHTPLAN_H

#include "common/common.h"
#include "common/list.h"

#include "airport.h"
#include "airway.h"
#include "navdata.h"
#include "waypoint.h"

typedef enum ndt_fltplanformat
{
    NDT_FLTPFMT_OTHER, // unknown or unsupported format
    NDT_FLTPFMT_AIBXT, // Airbus X Extended format
    NDT_FLTPFMT_DCDED, // decoded route legs, same line
    NDT_FLTPFMT_ICAOR, // ICAO route with departure/arrival airports, no SID/STAR
    NDT_FLTPFMT_IRECP, // route recap, multiple lines per leg
    NDT_FLTPFMT_SBRIF, // ICAO route only, with FlightAware latitude/longitude
    NDT_FLTPFMT_XPFMS, // X-Plane default FMS format
    NDT_FLTPFMT_XPCVA, // optimized for use with CIVA
    NDT_FLTPFMT_XPHLP, // optimized for use with X-Plane FMS
    NDT_FLTPFMT_XPCDU, // optimized for use with QPAC's MCDU
} ndt_fltplanformat;

typedef struct ndt_flightplan
{
    ndt_info     info;         // identification information
    ndt_list    *cws;          // custom waypoints (struct ndt_waypoint)
    ndt_distance crz_altitude; // cruise altitude

    struct
    {
        ndt_airport *apt;      // departure airport
        ndt_runway  *rwy;      // departure runway
    } dep;

    struct
    {
        ndt_airport *apt;      // arrival airport
        ndt_runway  *rwy;      // arrival runway
    } arr;

    ndt_list *rte;             // list of segments (struct ndt_route_segment)
    ndt_list *legs;            // decoded route    (struct ndt_route_leg)

    /* TODO: everything else */
} ndt_flightplan;

ndt_flightplan* ndt_flightplan_init         (                                                                                                       );
void            ndt_flightplan_close        (ndt_flightplan **_flightplan                                                                           );
int             ndt_flightplan_set_departure(ndt_flightplan   *flightplan, ndt_navdatabase *navdatabase, const char *icao,  const char       *runway);
int             ndt_flightplan_set_arrival  (ndt_flightplan   *flightplan, ndt_navdatabase *navdatabase, const char *icao,  const char       *runway);
int             ndt_flightplan_set_route    (ndt_flightplan   *flightplan, ndt_navdatabase *navdatabase, const char *route, ndt_fltplanformat format);
int             ndt_flightplan_write        (ndt_flightplan   *flightplan, FILE *file,                                      ndt_fltplanformat format);

typedef struct ndt_route_segment
{
    ndt_info      info;        // identification information
    ndt_waypoint *src;         // segment's entry point (NULL for discontinuities)
    ndt_waypoint *dst;         // segment's exit  point (always set)

    enum
    {
        NDT_RSTYPE_AWY,        // airway
        NDT_RSTYPE_DCT,        // direct to
        NDT_RSTYPE_DSC,        // route discontinuity
        NDT_RSTYPE_PRC,        // procedure
    } type;

    union
    {
        // associated airway info
        struct
        {
            ndt_airway     *awy;
            ndt_airway_leg *src;
            ndt_airway_leg *dst;
        }
        awy;
    };

    ndt_list *legs;
} ndt_route_segment;

ndt_route_segment* ndt_route_segment_init  (                                                                                                                                                 );
void               ndt_route_segment_close (ndt_route_segment **_segment                                                                                                                     );
ndt_route_segment* ndt_route_segment_airway(ndt_waypoint        *src_wpt,  ndt_waypoint *dst_wpt, ndt_airway *airway, ndt_airway_leg *inleg, ndt_airway_leg *outleg, ndt_navdatabase *navdata);
ndt_route_segment* ndt_route_segment_direct(ndt_waypoint        *src_wpt,  ndt_waypoint *dst_wpt,                                                                    ndt_navdatabase *navdata);
ndt_route_segment* ndt_route_segment_discon(                               ndt_waypoint *dst_wpt                                                                                             );

typedef struct ndt_route_leg
{
    ndt_waypoint      *src;    // leg's start point                       (may be NULL)
    ndt_waypoint      *dst;    // leg's stop  point                       (NULL: manual termination)
    ndt_distance       dis;    // distance covered by leg                 (unset if src or dst NULL)
    double             trb;    // bearing (unit: deg), true               (unset if src or dst NULL)
    double             imb;    // bearing (unit: deg), magnetic,  inbound (unset if src or dst NULL)
    double             omb;    // bearing (unit: deg), magnetic, outbound (unset if src or dst NULL)
    ndt_route_segment *rsg;    // leg's parent route segment

    enum
    {
        // ARINC 424
        NDT_LEGTYPE_AF =  1,   // constant DME arc to a fix
        NDT_LEGTYPE_CA =  2,   // course to an altitude
        NDT_LEGTYPE_CD =  3,   // course to a DME distance
        NDT_LEGTYPE_CI =  4,   // course to next leg
        NDT_LEGTYPE_CF =  5,   // course to a fix
        NDT_LEGTYPE_CR =  6,   // course to a radial termination
        NDT_LEGTYPE_DF =  7,   // computed track direct to a fix
        NDT_LEGTYPE_FA =  8,   // course from a fix to an altitude
        NDT_LEGTYPE_FC =  9,   // course from a fix to a distance
        NDT_LEGTYPE_FD = 10,   // course from a fix to a DME distance
        NDT_LEGTYPE_FM = 11,   // course from a fix to a manual termination
        NDT_LEGTYPE_IF = 12,   // initial fix
        NDT_LEGTYPE_PI = 13,   // procedure turn followed by course to a fix
        NDT_LEGTYPE_RF = 14,   // constant radius to a fix
        NDT_LEGTYPE_TF = 15,   // track between two fixes (great circle)
        NDT_LEGTYPE_VA = 16,   // heading to an altitude
        NDT_LEGTYPE_VD = 17,   // heading to a DME distance
        NDT_LEGTYPE_VI = 18,   // heading to next leg
        NDT_LEGTYPE_VM = 19,   // heading to manual termination
        NDT_LEGTYPE_VR = 20,   // heading to radial
        NDT_LEGTYPE_HF = 21,   // hold at a fix after one full circuit
        NDT_LEGTYPE_HA = 22,   // hold at a fix after reaching an altitude
        NDT_LEGTYPE_HM = 23,   // hold manually

        // custom
        NDT_LEGTYPE_ZZ = 99,   // discontinuity
    } type;

    union
    {
        // associated airway info
        struct
        {
            ndt_airway_leg *leg;
        }
        awy;
    };

    ndt_restriction constraints;
} ndt_route_leg;

ndt_route_leg* ndt_route_leg_init    (                    );
void           ndt_route_leg_close   (ndt_route_leg **_leg);
int            ndt_route_leg_restrict(ndt_route_leg   *leg, ndt_restriction constraints);

#endif /* NDT_FLIGHTPLAN_H */
