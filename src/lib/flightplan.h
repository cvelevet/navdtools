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
    NDT_FLTPFMT_DTSTG, // decoded route legs, same line, lat/lon only
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
    ndt_info             info; // identification information
    ndt_list             *cws; // custom waypoints (struct ndt_waypoint)
    ndt_navdatabase      *ndb; // associated navigation database
    ndt_distance crz_altitude; // cruise altitude

    struct
    {
        ndt_airport *apt;                           // departure airport
        ndt_runway  *rwy;                           // departure runway
        struct
        {
            struct ndt_procedure     *proc;         // standard departure (template)
            struct ndt_route_segment *rsgt;         // standard departure (editable)
            struct
            {
                struct ndt_procedure     *proc;     // enroute transition (template)
                struct ndt_route_segment *rsgt;     // enroute transition (editable)
            } enroute;
        } sid;
    } dep;

    struct
    {
        ndt_airport *apt;                           // arrival airport
        ndt_runway  *rwy;                           // arrival runway
        struct
        {
            struct ndt_procedure     *proc;         // standard arrival (template)
            struct ndt_route_segment *rsgt;         // standard arrival (editable)
            struct
            {
                struct ndt_procedure     *proc;     // enroute transition (template)
                struct ndt_route_segment *rsgt;     // enroute transition (editable)
            } enroute;
        } star;
        struct
        {
            struct ndt_procedure     *proc;         // final approach (template)
            struct ndt_route_segment *rsgt;         // final approach (editable)
            struct
            {
                struct ndt_procedure     *proc;     // approach transition (template)
                struct ndt_route_segment *rsgt;     // approach transition (editable)
            } transition;
        } apch;
        struct
        {
            struct ndt_route_segment *rsgt;         // dummy leg to runway threshold
            struct ndt_route_leg     *rleg;         // dummy leg to runway threshold
        } last;
    } arr;

    ndt_list *rte;             // list of segments (struct ndt_route_segment)
    ndt_list *legs;            // decoded route    (struct ndt_route_leg)
} ndt_flightplan;

ndt_flightplan* ndt_flightplan_init         (ndt_navdatabase *navdatabase                                             );
void            ndt_flightplan_close        (ndt_flightplan **_flightplan                                             );
int             ndt_flightplan_set_departure(ndt_flightplan   *flightplan, const char *icao,  const char       *runway);
int             ndt_flightplan_set_departsid(ndt_flightplan   *flightplan, const char *name,  const char   *transition);
int             ndt_flightplan_set_arrival  (ndt_flightplan   *flightplan, const char *icao,  const char       *runway);
int             ndt_flightplan_set_arrivstar(ndt_flightplan   *flightplan, const char *name,  const char   *transition);
int             ndt_flightplan_set_arrivapch(ndt_flightplan   *flightplan, const char *name,  const char   *transition);
int             ndt_flightplan_set_route    (ndt_flightplan   *flightplan, const char *route, ndt_fltplanformat format);
int             ndt_flightplan_write        (ndt_flightplan   *flightplan, FILE *file,        ndt_fltplanformat format);

typedef struct ndt_procedure
{
    ndt_info      info;                 // identification information
    ndt_list *proclegs;                 // legs (main procedure)
    ndt_list *mapplegs;                 // legs (missed approach)
    ndt_list *custwpts;                 // procedure-specific custom waypoints
    ndt_list  *runways;                 // applicable runways for this procedure (NULL for SID/STAR enroute transitions)

    struct
    {
        // transition lists             // (NULL if N/A)
        ndt_list *approach;             // applicable to: final
        ndt_list *enroute;              // applicable to: STAR, SID

        // transition points            // (NULL if N/A)
        union
        {
            struct ndt_waypoint   *wpt; // applicable to: transitions (approach, enroute)
            struct ndt_procedure  *sid; // applicable to: transitions (runway  ->    SID)
            struct ndt_procedure *star; // applicable to: transitions (STAR    -> runway)
        };
    } transition;

    struct
    {
        enum
        {
            NDT_APPRTYPE_CTL,           // circle-to-land (a.k.a. letdown)
            NDT_APPRTYPE_GLS,           // GLS
            NDT_APPRTYPE_IGS,           // IGS
            NDT_APPRTYPE_ILS,           // ILS
            NDT_APPRTYPE_LDA,           // LDA
            NDT_APPRTYPE_LOC,           // LOC
            NDT_APPRTYPE_LOCB,          // LOC back course
            NDT_APPRTYPE_NDB,           // NDB
            NDT_APPRTYPE_NDBD,          // NDB + DME
            NDT_APPRTYPE_RNAV,          // RNAV
            NDT_APPRTYPE_VOR,           // VOR
            NDT_APPRTYPE_VORD,          // VOR + DME
            NDT_APPRTYPE_VORT,          // VOR + TACAN
            NDT_APPRTYPE_UNK,           // unknown
        } type;

        char short_name[9];             // for space-constrainted listing
    } approach;

    enum ndt_procedure_type
    {
        NDT_PROCTYPE_SID_1 = 10,        // transition: runway   ->  SID
        NDT_PROCTYPE_SID_2 = 11,        // SID
        NDT_PROCTYPE_SID_3 = 12,        // transition: SID      ->  enroute
        NDT_PROCTYPE_SID_4 = 20,        // transition: runway   ->  SID      (RNAV)
        NDT_PROCTYPE_SID_5 = 21,        // SID                               (RNAV)
        NDT_PROCTYPE_SID_6 = 22,        // transition: SID      ->  enroute  (RNAV)
        NDT_PROCTYPE_STAR1 = 30,        // transition: enroute  ->  STAR
        NDT_PROCTYPE_STAR2 = 31,        // STAR
        NDT_PROCTYPE_STAR3 = 32,        // transition: STAR     ->  runway
        NDT_PROCTYPE_STAR4 = 40,        // transition: enroute  ->  STAR     (RNAV)
        NDT_PROCTYPE_STAR5 = 41,        // STAR                              (RNAV)
        NDT_PROCTYPE_STAR6 = 42,        // transition: STAR     ->  runway   (RNAV)
        NDT_PROCTYPE_STAR7 = 50,        // transition: enroute  ->  profile descent
        NDT_PROCTYPE_STAR8 = 51,        // profile descent
        NDT_PROCTYPE_STAR9 = 52,        // transition: descent  ->  runway
        NDT_PROCTYPE_APPTR = 60,        // transition: STAR     ->  final    (from STAR/transition endpoint)
        NDT_PROCTYPE_FINAL = 61,        // final approach
    } type;
} ndt_procedure;

ndt_procedure* ndt_procedure_init (                              enum ndt_procedure_type  type);
void           ndt_procedure_close(                                   ndt_procedure **_pointer);
ndt_procedure* ndt_procedure_get  (ndt_list *procedures,  const char *name, ndt_runway *runway);
ndt_procedure* ndt_procedure_gettr(ndt_list *transitions, const char *name                    );
void           ndt_procedure_names(ndt_list *procedures,  ndt_list *output                    );
void           ndt_procedure_trans(ndt_list *transitions, ndt_list *output                    );

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
        NDT_RSTYPE_MAP,        // procedure (missed approach)
        NDT_RSTYPE_PRC,        // procedure (everything else)
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

        // associated procedure
        ndt_procedure *prc;
    };

    ndt_list *legs;
} ndt_route_segment;

ndt_route_segment* ndt_route_segment_init  (                                                                                                                                                 );
void               ndt_route_segment_close (ndt_route_segment **_segment                                                                                                                     );
ndt_route_segment* ndt_route_segment_airway(ndt_waypoint        *src_wpt,  ndt_waypoint *dst_wpt, ndt_airway *airway, ndt_airway_leg *inleg, ndt_airway_leg *outleg, ndt_navdatabase *navdata);
ndt_route_segment* ndt_route_segment_direct(ndt_waypoint        *src_wpt,  ndt_waypoint *dst_wpt                                                                                             );
ndt_route_segment* ndt_route_segment_discon(                                                                                                                                                 );
ndt_route_segment* ndt_route_segment_proced(ndt_waypoint        *src_wpt,  ndt_restriction              *constraints, ndt_procedure                      *procedure, ndt_navdatabase *navdata);

typedef struct ndt_route_leg
{
    ndt_info          info;    // identification information
    ndt_waypoint      *src;    // leg's start point                       (may be NULL)
    ndt_waypoint      *dst;    // leg's stop  point                       (may be NULL)
    ndt_distance       dis;    // distance covered by leg                 (unset if src or dst NULL)
    double             trb;    // bearing (unit: deg), true               (unset if src or dst NULL)
    double             imb;    // bearing (unit: deg), magnetic,  inbound (unset if src or dst NULL)
    double             omb;    // bearing (unit: deg), magnetic, outbound (unset if src or dst NULL)
    ndt_route_segment *rsg;    // leg's parent route segment
    ndt_list        *xpfms;    // list of fake waypoints for XPFMS based formats

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
        NDT_LEGTYPE_HF = 21,   // hold at a fix
        NDT_LEGTYPE_HA = 22,   // hold at a fix to altitude
        NDT_LEGTYPE_HM = 23,   // hold manually

        // custom
        NDT_LEGTYPE_ZZ = 99,   // discontinuity
    } type;

    union
    {
        // airway leg
        ndt_airway_leg *awyleg;

        // course to fix, radial, altitude, DME distance or intercept
        struct
        {
            ndt_waypoint *navaid;       // source navaid
            double      magnetic;       // magnetic course
            union
            {
                double         radial;  // navaid radial
                ndt_distance altitude;  // target altitude
                ndt_distance distance;  // DME or leg distance
            };
        } course;

        // heading to fix, radial, altitude, DME distance or intercept
        struct
        {
            ndt_waypoint *navaid;       // source navaid
            double       degrees;       // heading
            union
            {
                double         radial;  // navaid radial
                ndt_distance altitude;  // target altitude
                ndt_distance distance;  // DME or leg distance
            };
        } heading;

        // procedure turn
        struct
        {
            ndt_waypoint *waypoint;     // associated waypoint
            ndt_waypoint   *navaid;     // associated navaid
            ndt_distance    limdis;     // turn distance limit (from: ????????)
            ndt_distance    outdis;     // outbound distance   (from: waypoint)
            double          outbrg;     // outbound bearing    (from: waypoint)
            double          tangle;     // turn angle from outbound bearing
        } turn;

        // arc to fix
        struct
        {
            double          start;      // start radial
            double           stop;      // stop  radial
            ndt_waypoint  *center;      // arc   center
            ndt_distance distance;      // DME distance
        } arc;

        // radius to fix
        struct
        {
            double          start;      // mangetic bearing (src to center)
            ndt_waypoint  *center;      // arc center
            ndt_distance distance;      // radius
        } radius;

        // fix to altitude, DME/track distance or manual termination
        struct
        {
            ndt_waypoint    *src;       // source waypoint (fix)
            ndt_waypoint *navaid;       // source navaid
            double        course;       // magnetic course
            union
            {
                ndt_distance altitude;  // target altitude
                ndt_distance distance;  // DME or along track distance
            };
        } fix;

        // hold at fix/to altitude/manual termination
        struct
        {
            enum
            {
                NDT_HOLD_SECONDS,
                NDT_HOLD_DISTANCE,
            } type;

            union
            {
                int          duration;
                ndt_distance distance;
            };

            ndt_waypoint *waypoint;
            ndt_distance  altitude;
            double  inbound_course;
        } hold;
    };

    ndt_restriction constraints;
} ndt_route_leg;

ndt_route_leg* ndt_route_leg_init    (                                                 );
void           ndt_route_leg_close   (ndt_route_leg **_leg                             );
int            ndt_route_leg_restrict(ndt_route_leg   *leg, ndt_restriction constraints);

#endif /* NDT_FLIGHTPLAN_H */
