/*
 * ndb_xpgns.c
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2014-2016 Timothy D. Walker and others.
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

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "common/common.h"

#include "compat/compat.h"

#include "airport.h"
#include "airway.h"
#include "flightplan.h"
#include "navdata.h"
#include "ndb_xpgns.h"
#include "waypoint.h"

// check whether first decimal digit is odd
#define NDT_ODD_DEC1(F) (((int)(10 * F)) % 2)

static int parse_airac     (char *src, ndt_navdatabase *ndb                  );
static int parse_airports  (char *src, ndt_navdatabase *ndb                  );
static int parse_airways   (char *src, ndt_navdatabase *ndb                  );
static int parse_navaids   (char *src, ndt_navdatabase *ndb                  );
static int parse_waypoints (char *src, ndt_navdatabase *ndb                  );
static int parse_procedures(char *src, ndt_navdatabase *ndb, ndt_airport *apt);
static int place_procedures(           ndt_navdatabase *ndb, ndt_airport *apt);
static int rename_finalappr(                                 ndt_airport *apt);
static int open_a_procedure(           ndt_navdatabase *ndb, ndt_procedure *p);

int ndt_ndb_xpgns_navdatabase_init(ndt_navdatabase *ndb)
{
    /*
     * The root of the navdata is a folder containing the following subfolders:
     *
     * - airspaces (optional)
     * - navdata   (mandatory for our use, including all elements below)
     *    - Airports.txt   // airports
     *    - ATS.txt        // airways
     *    - cycle_info.txt // AIRAC information
     *    - Navaids.txt    // radio navigation aids
     *    - Proc
     *       - <*>.txt     // procedures (per-airport)
     *    - Waypoints.txt  // RNAV fixes
     */
    char  *airac        = NULL, *airports  = NULL, *airways   = NULL;
    char  *navaids      = NULL, *waypoints = NULL;
    DIR   *procedures   = NULL;
    char  *path         = NULL;
    int    pathlen, ret = 0;

    if ((ret = ndt_file_getpath(ndb->root, "/cycle_info.txt", &path, &pathlen)))
    {
        goto end;
    }
    airac = ndt_file_slurp(path, &ret);

    if (ret || (ret = parse_airac(airac, ndb)))
    {
        goto end;
    }

    if ((ret = ndt_file_getpath(ndb->root, "/Airports.txt", &path, &pathlen)))
    {
        goto end;
    }
    airports = ndt_file_slurp(path, &ret);

    if (ret || (ret = parse_airports(airports, ndb)))
    {
        goto end;
    }

    if ((ret = ndt_file_getpath(ndb->root, "/ATS.txt", &path, &pathlen)))
    {
        goto end;
    }
    airways = ndt_file_slurp(path, &ret);

    if (ret || (ret = parse_airways(airways, ndb)))
    {
        goto end;
    }

    if ((ret = ndt_file_getpath(ndb->root, "/Navaids.txt", &path, &pathlen)))
    {
        goto end;
    }
    navaids = ndt_file_slurp(path, &ret);

    if (ret || (ret = parse_navaids(navaids, ndb)))
    {
        goto end;
    }

    if ((ret = ndt_file_getpath(ndb->root, "/Waypoints.txt", &path, &pathlen)))
    {
        goto end;
    }
    waypoints = ndt_file_slurp(path, &ret);

    if (ret || (ret = parse_waypoints(waypoints, ndb)))
    {
        goto end;
    }

    if ((ret = ndt_file_getpath(ndb->root, "/Proc", &path, &pathlen)))
    {
        goto end;
    }

    if (!(procedures = opendir(path)))
    {
        ret = errno;
        goto end;
    }

end:
    free(airac);
    free(airports);
    free(airways);
    free(navaids);
    free(path);
    if  (procedures) closedir(procedures);
    free(waypoints);
    return ret;
}

static int parse_airac(char *src, ndt_navdatabase *ndb)
{
    char *vlist[] = { "Aerosoft NavDataPro", "Navigraph", NULL };
    int   vendor  = -1;
    int   version = -1;
    char *pos     = src;
    char *line    = NULL;
    int   linecap, ret = 0;

    while ((ret = ndt_file_getline(&line, &linecap, &pos)) > 0)
    {
        if (*line == '\r' || *line == '\n')
        {
            continue; // skip blank lines
        }

        int  ivalue;
        char buffer[2][12];

        if (sscanf(line, "AIRAC cycle    : %d", &ivalue) == 1)
        {
            snprintf(ndb->info.idnt, sizeof(ndb->info.idnt), "AIRAC%d", ivalue);
            continue;
        }

        if (sscanf(line, "Valid (from/to): %11s - %11s",
                   buffer[0], buffer[1]) == 2)
        {
            snprintf(ndb->info.misc, sizeof(ndb->info.misc), "%s - %s",
                     buffer[0], buffer[1]);
            continue;
        }

        if (sscanf(line, "Version        : %d", &ivalue) == 1 ||
            sscanf(line, "Revision       : %d", &ivalue) == 1)
        {
            version = ivalue;
            continue;
        }

        for (ivalue = 0; vlist[ivalue]; ivalue++)
        {
            if (strcasestr(line, vlist[ivalue]))
            {
                vendor = ivalue;
                break;
            }
        }
    }

    if (ret < 0)
    {
        ret = EIO;
        goto end;
    }

    if (!strnlen(ndb->info.idnt, 1) ||
        !strnlen(ndb->info.misc, 1) || vendor < 0 || version < 0)
    {
        ret = EINVAL;
        goto end;
    }

    snprintf(ndb->info.desc, sizeof(ndb->info.desc),
             "%s, format: X-Plane 10 (GNS430), %s v%d, valid: %s",
             vlist[vendor], ndb->info.idnt, version, ndb->info.misc);

end:
    free(line);
    return ret;
}

static int parse_airports(char *src, ndt_navdatabase *ndb)
{
    char         *pos  = src;
    ndt_airport  *apt  = NULL;
    char         *line = NULL;
    int           linecap, ret = 0;

    while ((ret = ndt_file_getline(&line, &linecap, &pos)) > 0)
    {
        if (*line == '\r' || *line == '\n')
        {
            continue; // skip blank lines
        }

        if (!strncmp(line, "A,", 2))
        {
            double latitude, longitude;
            int    elevation, transalt, translvl, longestr;

            apt = ndt_airport_init();
            if (!apt)
            {
                ret = ENOMEM;
                goto end;
            }
            apt->waypoint = ndt_waypoint_init();
            if (!apt->waypoint)
            {
                ndt_airport_close(&apt);
                ret = ENOMEM;
                goto end;
            }

            /*
             * Format: CSV
             * - 'A'
             * - ICAO code or equivalent
             * - name
             * - latitude
             * - longitude
             * - elevation           (unit: ft)
             * - transition altitude (unit: ft)
             * - transition level    (unit: ft)
             * - longest runway      (unit: ft)
             */
            if (sscanf(line, "A,%4s,%127[^,],%lf,%lf,%d,%d,%d,%d",
                       apt->info.idnt,
                       apt->info.misc,
                       &latitude,
                       &longitude,
                       &elevation,
                       &transalt,
                       &translvl,
                       &longestr) != 8)
            {
                ndt_waypoint_close(&apt->waypoint);
                ndt_airport_close (&apt);
                ret = EINVAL;
                goto end;
            }

            if (translvl > 0 && transalt > 0)
            {
                snprintf(apt->info.desc, sizeof(apt->info.desc),
                         "Airport %s (%s), elevation %d, TRL/TA %d/%d",
                         apt->info.idnt,
                         apt->info.misc, elevation, translvl, transalt);
            }
            else if (translvl > 0)
            {
                snprintf(apt->info.desc, sizeof(apt->info.desc),
                         "Airport %s (%s), elevation %d, TRL/TA %d/ATC",
                         apt->info.idnt,
                         apt->info.misc, elevation, translvl);
            }
            else if (transalt > 0)
            {
                snprintf(apt->info.desc, sizeof(apt->info.desc),
                         "Airport %s (%s), elevation %d, TRL/TA ATC/%d",
                         apt->info.idnt,
                         apt->info.misc, elevation, transalt);
            }
            else
            {
                snprintf(apt->info.desc, sizeof(apt->info.desc),
                         "Airport %s (%s), elevation %d, TRL/TA ATC",
                         apt->info.idnt,
                         apt->info.misc, elevation);
            }

            strncpy(apt->waypoint->region,    "",             sizeof(apt->waypoint->region));
            strncpy(apt->waypoint->info.idnt, apt->info.idnt, sizeof(apt->waypoint->info.idnt));
            strncpy(apt->waypoint->info.misc, apt->info.misc, sizeof(apt->waypoint->info.misc));
            strncpy(apt->waypoint->info.desc, apt->info.desc, sizeof(apt->waypoint->info.desc));

            ndt_distance airprt_alt = ndt_distance_init(elevation, NDT_ALTUNIT_FT);
            apt->tr_altitude        = ndt_distance_init(transalt,  NDT_ALTUNIT_FT);
            apt->trans_level        = ndt_distance_init(translvl,  NDT_ALTUNIT_FT);
            apt->rwy_longest        = ndt_distance_init(longestr,  NDT_ALTUNIT_FT);
            apt->coordinates        =
            apt->waypoint->position = ndt_position_init(latitude, longitude, airprt_alt);
            apt->waypoint->type     = NDT_WPTYPE_APT;

            ndt_list_add(ndb->airports,  apt);
            ndt_list_add(ndb->waypoints, apt->waypoint);
            continue;
        }

        if (!strncmp(line, "R,", 2))
        {
            const char *surfname, *usgename;
            double frequency, latitude, longitude;
            int    length, width, elevation, overfly, surface, usage;

            if (!apt)
            {
                ret = EINVAL;
                goto end;
            }

            ndt_runway *rwy = ndt_runway_init();
            if (!rwy)
            {
                ret = ENOMEM;
                goto end;
            }
            rwy->waypoint = ndt_waypoint_init();
            if (!rwy->waypoint)
            {
                ndt_runway_close(&rwy);
                ret = ENOMEM;
                goto end;
            }

            /*
             * Format: CSV
             * - 'R'
             * - identifier
             * - heading              (unit: deg)
             * - length               (unit:  ft)
             * - width                (unit:  ft)
             * - ILS: availability
             * - ILS: frequency       (unit: kHz)
             * - ILS: course          (unit: deg)
             * - threshold: latitude
             * - threshold: longitude
             * - threshold: elevation (unit:  ft)
             * - glideslope: angle    (unit: deg)
             * - thres. ovfly. height (unit:  ft)
             * - surface type
             * - usage
             */
            if (sscanf(line,
                       "R,%4[^,],%d,%d,%d,%d,%lf,%d,%lf,%lf,%d,%lf,%d,%d,%d", rwy->info.idnt,
                       &rwy->heading,
                       &length,
                       &width,
                       &rwy->ils.avail,
                       &frequency,
                       &rwy->ils.course,
                       &latitude,
                       &longitude,
                       &elevation,
                       &rwy->ils.slope,
                       &overfly,
                       &surface,
                       &usage) != 14)
            {
                ndt_waypoint_close(&rwy->waypoint);
                ndt_runway_close  (&rwy);
                ret = EINVAL;
                goto end;
            }

            if (usage == 3)
            {
                //runway closed
                ndt_waypoint_close(&rwy->waypoint);
                ndt_runway_close  (&rwy);
                continue;
            }

            switch (usage)
            {
                case 1:
                    rwy->status = NDT_RWYUSE_TAKEOF;
                    usgename    = "takeoff only";
                    break;
                case 2:
                    rwy->status = NDT_RWYUSE_LANDNG;
                    usgename    = "landing only";
                    break;
                case 0:
                default:
                    rwy->status = NDT_RWYUSE_LDGTOF;
                    usgename    = NULL;
                    break;
            }

            switch (surface)
            {
                case 0:
                    rwy->surface = NDT_RWYSURF_CONCR;
                    surfname     = "concrete";
                    break;
                case 1:
                    rwy->surface = NDT_RWYSURF_ASPHT;
                    surfname     = "asphalt/bitumen";
                    break;
                case 2:
                    rwy->surface = NDT_RWYSURF_GRAVL;
                    surfname     = "gravel/coral/ice";
                    break;
                case 3:
                default:
                    rwy->surface = NDT_RWYSURF_OTHER;
                    surfname     = NULL;
                    break;
            }

            snprintf(rwy->info.desc, sizeof(rwy->info.desc),
                     "%s runway %s, heading %03d°, length %d ft, width %d ft",
                     apt->info.idnt, rwy->info.idnt, rwy->heading, length, width);

            if ((rwy->ils.avail = !!rwy->ils.avail))
            {
                rwy->waypoint->range     = ndt_distance_init(18, NDT_ALTUNIT_NM);
                rwy->waypoint->frequency = rwy->ils.freq = ndt_frequency_init(frequency);
                snprintf(rwy->info.desc         + strlen(rwy->info.desc),
                         sizeof(rwy->info.desc) - strlen(rwy->info.desc),
                         ", ILS %.3lf course %03d° slope %.1lf",
                         ndt_frequency_get(rwy->ils.freq), rwy->ils.course, rwy->ils.slope);
            }
            else if (rwy->ils.slope > 0.1)
            {
                rwy->ils.avail = -1;
            }

            if (surfname || usgename)
            {
                snprintf(rwy->info.desc         + strlen(rwy->info.desc),
                         sizeof(rwy->info.desc) - strlen(rwy->info.desc),
                         " (%s%s%s)",
                         surfname != NULL ? surfname : "",
                         surfname && usgename ? ", " : "",
                         usgename != NULL ? usgename : "");
            }

            strncpy (rwy->waypoint->region, "", sizeof(rwy->waypoint->region));
            snprintf(rwy->waypoint->info.idnt,  sizeof(rwy->waypoint->info.idnt),
                     "%s%s",                    apt->info.idnt, rwy->info.idnt);
            snprintf(rwy->waypoint->info.desc,  sizeof(rwy->waypoint->info.desc),
                     "Runway %s, airport: %s",  rwy->info.idnt, apt->info.idnt);

            ndt_distance thresh_alt = ndt_distance_init(elevation, NDT_ALTUNIT_FT);
            rwy->length             = ndt_distance_init(length,    NDT_ALTUNIT_FT);
            rwy->width              = ndt_distance_init(width,     NDT_ALTUNIT_FT);
            rwy->overfly            =  ndt_distance_init(overfly,  NDT_ALTUNIT_FT);
            rwy->threshold          =
            rwy->waypoint->position = ndt_position_init(latitude, longitude, thresh_alt);
            rwy->waypoint->type     = NDT_WPTYPE_RWY;
            ndt_list_add(apt->runways,   rwy);
            ndt_list_add(ndb->waypoints, rwy->waypoint);
            continue;
        }

        if (!strncmp(line, "X,", 2))
        {
            continue; // AIRAC cycle and dates
        }

        // unsupported input
        ret = EINVAL;
        goto end;
    }

    if (ret < 0)
    {
        ret = EIO;
        goto end;
    }

end:
    if (ret == EINVAL)
    {
        ndt_log("[ndb_xpgns] parse_airports: failed to parse \"");
        for (int i = 0; line[i] != '\r' && line[i] != '\n'; i++)
        {
            ndt_log("%c", line[i]);
        }
        ndt_log("\"\n");
    }
    free(line);
    return ret;
}

static int parse_airways(char *src, ndt_navdatabase *ndb)
{
    char           *pos  = src;
    ndt_airway     *awy  = NULL;
    ndt_airway_leg *leg  = NULL;
    char           *line = NULL;
    int             linecap, count_in, count_out, ret = 0;

    while ((ret = ndt_file_getline(&line, &linecap, &pos)) > 0)
    {
        if (*line == '\r' || *line == '\n')
        {
            continue; // skip blank lines
        }

        if (!strncmp(line, "A,", 2))
        {
            if (awy)
            {
                // previous airway wasn't properly finalized
                ret = EINVAL;
                goto end;
            }

            awy = ndt_airway_init();
            if (!awy)
            {
                ret = ENOMEM;
                goto end;
            }

            /*
             * Format: CSV
             * - 'A'
             * - identifier
             * - waypoint count
             */
            if (sscanf(line, "A,%6[^,],%d", awy->info.idnt, &count_in) != 2)
            {
                ret = EINVAL;
                goto end;
            }

            ndt_list_add(ndb->airways, awy);
            count_out = 0;
            continue;
        }

        if (!strncmp(line, "S,", 2))
        {
            double latitude[2], longitude[2], distance;
            size_t start = 0;

            if (!awy)
            {
                ret = EINVAL;
                goto end;
            }

            ndt_airway_leg *next = calloc(1, sizeof(ndt_airway_leg));
            if (!next)
            {
                ret = ENOMEM;
                goto end;
            }

            /*
             * Format: CSV
             * - 'S'
             * - identifier (entry point)
             * - latitude   (entry point)
             * - longitude  (entry point)
             * - identifier (exit  point)
             * - latitude   (exit  point)
             * - longitude  (exit  point)
             * - course     (inbound)
             * - course     (outbound)
             * - distance   (unit: nmi)
             */
            if (sscanf(line,
                       "S,%5[^,],%lf,%lf,%5[^,],%lf,%lf,%d,%d,%lf",
                       next->in. info.idnt, &latitude[0], &longitude[0],
                       next->out.info.idnt, &latitude[1], &longitude[1],
                      &next->course.inbound, &next->course.outbound, &distance) != 9)
            {
                free(next);
                ret = EINVAL;
                goto end;
            }

            next->in. position = ndt_position_init(latitude[0], longitude[0], ndt_distance_init(0, NDT_ALTUNIT_NA));
            next->out.position = ndt_position_init(latitude[1], longitude[1], ndt_distance_init(0, NDT_ALTUNIT_NA));
            next->length       = ndt_distance_init((int)(distance * 1852.), NDT_ALTUNIT_ME);
            next->awy          = awy;

            if (!leg)
            {
                awy->leg = leg = next;
            }
            else
            {
                leg->next = next;
                leg       = next;
            }

            if (++count_out == count_in)
            {
                // that was the last leg, finalize airway
                snprintf(awy->info.desc, sizeof(awy->info.desc),
                         "Airway: %5s, %2d legs, %-5s -> %s",  awy->info.idnt,
                         count_in, awy->leg->in.info.idnt, leg->out.info.idnt);

                awy = NULL;
                leg = NULL;
            }
            continue;
        }

        // unsupported input
        ret = EINVAL;
        goto end;
    }

    if (ret < 0)
    {
        ret = EIO;
        goto end;
    }

end:
    if (ret == EINVAL)
    {
        ndt_log("[ndb_xpgns] parse_airways: failed to parse \"");
        for (int i = 0; line[i] != '\r' && line[i] != '\n'; i++)
        {
            ndt_log("%c", line[i]);
        }
        ndt_log("\"\n");
    }
    free(line);
    return ret;
}

static int parse_navaids(char *src, ndt_navdatabase *ndb)
{
    char *pos  = src;
    char *line = NULL;
    int   linecap, ret = 0;

    while ((ret = ndt_file_getline(&line, &linecap, &pos)) > 0)
    {
        if (*line == '\r' || *line == '\n')
        {
            continue; // skip blank lines
        }

        int    elevation, range, vor;
        double frequency, latitude, longitude;

        ndt_waypoint *wpt = ndt_waypoint_init();
        if (!wpt)
        {
            ret = ENOMEM;
            goto end;
        }

        /*
         * Format: CSV
         * - identifier
         * - name
         * - frequency              (unit: variable)
         * - navaid has a VOR component
         * - navaid has a DME component
         * - range                  (unit: nmi)
         * - latitude               (unit: deg)
         * - longitude              (unit: deg)
         * - elevation              (unit:  ft)
         * - region/country code
         * - exclude from auto-tune (1: exclude, 0: include) (unused)
         */
        if (sscanf(line, "%4[^,],%127[^,],%lf,%d,%d,%d,%lf,%lf,%d,%2[^,],%*d", wpt->info.idnt,
                   wpt->info.misc, &frequency, &vor, &wpt->dme, &range, &latitude, &longitude, &elevation,
                   wpt->region) != 10)
        {
            ndt_waypoint_close(&wpt);
            ret = EINVAL;
            goto end;
        }

        if (vor)
        {
            wpt->type = NDT_WPTYPE_VOR;
        }
        else if (frequency >= 108. && frequency <= 111.95 && NDT_ODD_DEC1(frequency))
        {
            wpt->type = NDT_WPTYPE_LOC;
        }
        else if (wpt->dme)
        {
            wpt->type = NDT_WPTYPE_DME;
        }
        else if (frequency >= 175. && frequency <= 1750.)
        {
            /*
             * NDB frequency range: 190-1750 kHz; counter-examples:
             *
             * - "BS (PETROPAVLOVSK-KAMCHATSKY), 180 kHz (Aerosoft  1405)
             * - "LJ (SANJIAZI QIQIHAR),         177 kHz (Navigraph 1411)
             */
            wpt->type = NDT_WPTYPE_NDB;
        }
        else if (!frequency)
        {
            // unsure what those navaids are, TBH
            wpt->type = NDT_WPTYPE_LLC;
        }
        else
        {
            ndt_waypoint_close(&wpt);
            ret = EINVAL;
            goto end;
        }

        snprintf(wpt->info.misc         + strlen(wpt->info.misc),
                 sizeof(wpt->info.misc) - strlen(wpt->info.misc), " %s%s",
                 wpt->type == NDT_WPTYPE_VOR ? "VOR" :
                 wpt->type == NDT_WPTYPE_LOC ? "LOC" :
                 wpt->type == NDT_WPTYPE_NDB ? "NDB" :
                 wpt->type == NDT_WPTYPE_DME ? "DME" : "",
                 wpt->type != NDT_WPTYPE_DME && wpt->dme ? "/DME" : "");
        snprintf(wpt->info.desc, sizeof(wpt->info.desc),
                 "%3s%4s %-4s -- frequency: %8.3lf, region: %.2s",
                 wpt->type == NDT_WPTYPE_VOR ? "VOR" :
                 wpt->type == NDT_WPTYPE_LOC ? "LOC" :
                 wpt->type == NDT_WPTYPE_NDB ? "NDB" :
                 wpt->type == NDT_WPTYPE_DME ? "DME" : "",
                 wpt->type != NDT_WPTYPE_DME && wpt->dme ? "/DME" : "",
                 wpt->info.idnt, frequency, wpt->region);

        wpt->frequency = ndt_frequency_init(frequency);
        wpt->range     = ndt_distance_init (range, NDT_ALTUNIT_NM);
        wpt->position  = ndt_position_init (latitude, longitude, ndt_distance_init(elevation, NDT_ALTUNIT_FT));
        ndt_list_add(ndb->waypoints, wpt);
        continue;
    }

    if (ret < 0)
    {
        ret = EIO;
        goto end;
    }

end:
    if (ret == EINVAL)
    {
        ndt_log("[ndb_xpgns] parse_navaids: failed to parse \"");
        for (int i = 0; line[i] != '\r' && line[i] != '\n'; i++)
        {
            ndt_log("%c", line[i]);
        }
        ndt_log("\"\n");
    }
    free(line);
    return ret;
}

static int parse_waypoints(char *src, ndt_navdatabase *ndb)
{
    char *pos  = src;
    char *line = NULL;
    int   linecap, ret = 0;

    while ((ret = ndt_file_getline(&line, &linecap, &pos)) > 0)
    {
        if (*line == '\r' || *line == '\n')
        {
            continue; // skip blank lines
        }

        double latitude, longitude;
        char   letter[1];
        int    digit [1];

        ndt_waypoint *wpt = ndt_waypoint_init();
        if (!wpt)
        {
            ret = ENOMEM;
            goto end;
        }

        /*
         * Format: CSV
         * - identifier
         * - latitude
         * - longitude
         * - region/country code (may be empty)
         */
        if (sscanf(line, "%5[^,],%lf,%lf,%2[^,]", wpt->info.idnt, &latitude, &longitude, wpt->region) != 4)
        {
            ndt_waypoint_close(&wpt);
            ret = EINVAL;
            goto end;
        }

        if (*wpt->region == ' ' || *wpt->region == '\r' || *wpt->region == '\n')
        {
            *wpt->region = '\0';
        }

        snprintf(wpt->info.desc, sizeof(wpt->info.desc), "Fix %5s, region: %.3s",
                 wpt->info.idnt, *wpt->region ? wpt->region : "N/A");

        wpt->position = ndt_position_init(latitude, longitude, ndt_distance_init(0, NDT_ALTUNIT_NA));
        wpt->type     = NDT_WPTYPE_FIX;
        ndt_list_add(ndb->waypoints, wpt);
        continue;
    }

    if (ret < 0)
    {
        ret = EIO;
        goto end;
    }

end:
    if (ret == EINVAL)
    {
        ndt_log("[ndb_xpgns] parse_waypoints: failed to parse \"");
        for (int i = 0; line[i] != '\r' && line[i] != '\n'; i++)
        {
            ndt_log("%c", line[i]);
        }
        ndt_log("\"\n");
    }
    free(line);
    return ret;
}

ndt_airport* ndt_ndb_xpgns_navdata_init_airport(ndt_navdatabase *ndb, ndt_airport *apt)
{
    ndt_airport *ret = NULL;
    char       *path = NULL, *procedures = NULL, suffix[15];
    int err, pathlen = 0;

    if (apt->allprocs)
    {
        // success (already parsed)
        err = 000;
        ret = apt;
        goto  end;
    }

    apt->allprocs = ndt_list_init();
    if (!apt->allprocs)
    {
        goto end;
    }

    // get path to the procedures file
    err = snprintf(suffix, sizeof(suffix), "/Proc/%s.txt", apt->info.idnt);
    if (err <= 10 || err >= 15)
    {
        goto end; // airport ID must be 1-4 characters
    }
    if (ndt_file_getpath(ndb->root, suffix, &path, &pathlen))
    {
        goto end;
    }

    // then read it
    procedures = ndt_file_slurp(path, &err);
    if (err)
    {
        if (err == ENOENT)
        {
            err = 0; ret = apt; goto end; // doesn't exist: non-issue
        }
        goto end;
    }

    // and parse it
    if ((err = parse_procedures(procedures, ndb, apt)))
    {
        goto end;
    }

    // place them in various lists for correct access
    if ((err = place_procedures(ndb, apt)))
    {
        goto end;
    }

    // finally, do any desired cleanup, renaming etc.
    if ((err = rename_finalappr(apt)))
    {
        goto end;
    }

    // success
    err = 000;
    ret = apt;

end:
    if (!ret && apt->allprocs)
    {
        ndt_list_close(&apt->allprocs);
    }
    if (err)
    {
        char errbuf[64]; strerror_r(err, errbuf, sizeof(errbuf));
        ndt_log("[ndb_xpgns] navdata_init_airport: failed (%s)\n", errbuf);
    }
    free(procedures);
    free(path);
    return ret;
}

ndt_procedure* ndt_ndb_xpgns_navdata_open_procdre(ndt_navdatabase *ndb, ndt_procedure *proc)
{
    if (proc == NULL)
    {
        return  NULL;
    }
    if (proc->opened)
    {
        return proc;
    }
    if (open_a_procedure(ndb, proc))
    {
        return NULL;
    }
    if (proc->raw_data)
    {
        free(proc->raw_data);
        proc->raw_data = NULL;
    }
    proc->opened = 1;
    return proc;
}

static void handle_altitude_constraints(ndt_restriction *constraints, int altcst, int altone, int alttwo)
{
    if (constraints)
    {
        switch (altcst)
        {
            case 1:
                constraints->altitude.min =
                constraints->altitude.max = ndt_distance_init(altone, NDT_ALTUNIT_FT);
                constraints->altitude.typ = NDT_RESTRICT_AT;
                break;
            case 2:
                constraints->altitude.min = ndt_distance_init(altone, NDT_ALTUNIT_FT);
                constraints->altitude.typ = NDT_RESTRICT_AB;
                break;
            case 3:
                constraints->altitude.max = ndt_distance_init(altone, NDT_ALTUNIT_FT);
                constraints->altitude.typ = NDT_RESTRICT_BL;
                break;
            case 4:
                if (alttwo < altone) // Aerosoft and Navigraph apparently code this differently
                {
                    int tm = altone;
                    altone = alttwo;
                    alttwo = tm;
                }
                constraints->altitude.min = ndt_distance_init(altone, NDT_ALTUNIT_FT);
                constraints->altitude.max = ndt_distance_init(alttwo, NDT_ALTUNIT_FT);
                constraints->altitude.typ = NDT_RESTRICT_BT;
                break;
            case 0:
            default:
                constraints->altitude.typ = NDT_RESTRICT_NO;
                break;
        }
    }
}

static void handle_airspeed_constraints(ndt_restriction *constraints, int spdcst, int spdone, int spdtwo)
{
    if (constraints)
    {
        if (spdcst && (spdone > 0 || spdtwo > 0))
        {
            if (spdone > 0 && spdtwo > 0)
            {
                constraints->airspeed.min = ndt_airspeed_init(spdone, NDT_SPDUNIT_KTS);
                constraints->airspeed.max = ndt_airspeed_init(spdtwo, NDT_SPDUNIT_KTS);
                constraints->airspeed.typ = spdone == spdtwo ? NDT_RESTRICT_AT : NDT_RESTRICT_BT;
            }
            else if (spdone > 0)
            {
                constraints->airspeed.max = ndt_airspeed_init(spdone, NDT_SPDUNIT_KTS);
                constraints->airspeed.typ = NDT_RESTRICT_BL;
            }
            else // (spdtwo > 0)
            {
                constraints->airspeed.min = ndt_airspeed_init(spdone, NDT_SPDUNIT_KTS);
                constraints->airspeed.typ = NDT_RESTRICT_AB;
            }
        }
        else
        {
            constraints->airspeed.typ     = NDT_RESTRICT_NO;
        }
        switch (spdcst)
        {
            case 1:
                constraints->airspeed.acf = NDT_ACFTYPE_ALL;
                break;
            case 2:
                constraints->airspeed.acf = NDT_ACFTYPE_JET;
                break;
            case 3:
                constraints->airspeed.acf = NDT_ACFTYPE_TBP;
                break;
            case 4:
                constraints->airspeed.acf = NDT_ACFTYPE_OTH;
                break;
            case 0:
            default:
                constraints->airspeed.acf = NDT_ACFTYPE_NON;
                constraints->airspeed.typ = NDT_RESTRICT_NO;
                break;
        }
    }
}

static void handle_waypoint_constraints(ndt_restriction *constraints, int spcial, int ovrfly)
{
    if (constraints)
    {
        switch (spcial)
        {
            case 1:
                constraints->waypoint = NDT_WPTCONST_IAF;
                break;
            case 2:
                constraints->waypoint = NDT_WPTCONST_FAF;
                break;
            case 3:
                constraints->waypoint = NDT_WPTCONST_MAP;
                break;
            case 0:
            default:
                constraints->waypoint = (ovrfly == 1) ? NDT_WPTCONST_FOV : NDT_WPTCONST_NO;
                break;
        }
    }
}

static void handle_turndrct_constraints(ndt_restriction *constraints, int turndrct)
{
    if (constraints)
    {
        switch (turndrct)
        {
            case 1:
                constraints->turn = NDT_TURN_LEFT;
                break;
            case 2:
                constraints->turn = NDT_TURN_RIGHT;
                break;
            case 0:
            default:
                constraints->turn = NDT_TURN_SHORT;
                break;
        }
    }
}

static ndt_waypoint* procedure_waypoint(const char *idt, double lat, double lon)
{
    ndt_waypoint *wpt = NULL;

    if (round(fabs(lat)) > 90. || round(fabs(lon)) > 180.)
    {
        ndt_log("procedure_waypoint: invalid coordinates  %lf  %lf\n", lat, lon);
        goto end;
    }
    if (!(wpt = ndt_waypoint_init()))
    {
        goto end;
    }
    snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "%s", idt);
    wpt->position = ndt_position_init(lat, lon, NDT_DISTANCE_ZERO);
    wpt->type     = NDT_WPTYPE_LLC;

end:
    return wpt;
}

static double ndt_distance_nm(ndt_distance distance)
{
       return ndt_distance_get(distance, NDT_ALTUNIT_ME) / 1852.;
}

static int procedure_appendleg(ndt_procedure *proc, ndt_route_leg *leg1, int spcial, const char *line)
{
    if (!proc)
    {
        ndt_log("procedure_appendleg: no procedure for \"%s\"\n", line);
        ndt_route_leg_close(&leg1);
        return EINVAL;
    }

    switch (leg1->type)
    {
        case NDT_LEGTYPE_FA:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc), "TRK %03.0lf %s", leg1->fix.course, leg1->fix.src->info.idnt);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),    "(%"PRId64")", ndt_distance_get(leg1->constraints.altitude.min, NDT_ALTUNIT_FT));
            break;
        case NDT_LEGTYPE_CA:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),    "TRK %03.0lf", leg1->course.magnetic);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),    "(%"PRId64")", ndt_distance_get(leg1->constraints.altitude.min, NDT_ALTUNIT_FT));
            break;
        case NDT_LEGTYPE_VA:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),    "HDG %03.0lf", leg1->heading.degrees);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),    "(%"PRId64")", ndt_distance_get(leg1->constraints.altitude.min, NDT_ALTUNIT_FT));
            break;
        case NDT_LEGTYPE_FC:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc), "TRK %03.0lf %s", leg1->fix.course, leg1->fix.src->info.idnt);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),    "(%s D%.1lf)", leg1->fix.src->   info.idnt, ndt_distance_nm(leg1->fix.distance));
            break;
        case NDT_LEGTYPE_FD:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc), "TRK %03.0lf %s", leg1->fix.course, leg1->fix.src->info.idnt);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),    "(%s D%.1lf)", leg1->fix.navaid->info.idnt, ndt_distance_nm(leg1->fix.distance));
            break;
        case NDT_LEGTYPE_CD:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),    "TRK %03.0lf", leg1->course.magnetic);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),    "(%s D%.1lf)", leg1->course.navaid->info.idnt, ndt_distance_nm(leg1-> course.distance));
            break;
        case NDT_LEGTYPE_VD:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),    "HDG %03.0lf", leg1->heading.degrees);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),    "(%s D%.1lf)", leg1->heading.navaid->info.idnt, ndt_distance_nm(leg1->heading.distance));
            break;
        case NDT_LEGTYPE_CR:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),    "TRK %03.0lf", leg1->course.magnetic);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),  "(%s R%03.0lf)", leg1->course.navaid->info.idnt, leg1->course.radial);
            break;
        case NDT_LEGTYPE_VR:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),    "HDG %03.0lf", leg1->heading.degrees);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),  "(%s R%03.0lf)", leg1->heading.navaid->info.idnt, leg1->heading.radial);
            break;
        case NDT_LEGTYPE_CF:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),    "TRK %03.0lf", leg1->course.magnetic);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),             "%s", leg1->dst->info.idnt);
            break;
        case NDT_LEGTYPE_DF:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),  "TURN %sDIRECT", leg1->constraints.turn == NDT_TURN_SHORT ?       "" :
                                                                                 leg1->constraints.turn == NDT_TURN_RIGHT ? "RIGHT " : "LEFT ");
        case NDT_LEGTYPE_IF:
        case NDT_LEGTYPE_TF:
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),             "%s", leg1->dst->info.idnt);
            break;
        case NDT_LEGTYPE_AF:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),"ARC %s%s D%.1lf", leg1->constraints.turn == NDT_TURN_SHORT ?       "" :
                                                                                 leg1->constraints.turn == NDT_TURN_RIGHT ? "RIGHT " : "LEFT ",
                                                                                 leg1->arc.center->info.idnt, ndt_distance_nm(leg1->arc.distance));
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),             "%s", leg1->dst->info.idnt);
            break;
        case NDT_LEGTYPE_RF:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),      "%s D%.1lf", leg1->radius.center->info.idnt, ndt_distance_nm(leg1->radius.distance));
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),             "%s", leg1->dst->info.idnt);
            break;
        case NDT_LEGTYPE_HA:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),
                                                       "HOLD %s to (%"PRId64")", leg1->constraints.turn == NDT_TURN_RIGHT ? "RIGHT" : "LEFT",
                                                                                 ndt_distance_get(leg1->hold.altitude, NDT_ALTUNIT_FT));
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),             "%s", leg1->hold.waypoint->info.idnt);
            break;
        case NDT_LEGTYPE_HF:
        case NDT_LEGTYPE_HM:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),        "HOLD %s", leg1->constraints.turn == NDT_TURN_RIGHT ? "RIGHT" : "LEFT");
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),             "%s", leg1->hold.waypoint->info.idnt);
            break;
        case NDT_LEGTYPE_PI:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),      "P-TURN %s", leg1->turn.waypoint->info.idnt);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),           "(%s)", "INTC");
            break;
        case NDT_LEGTYPE_CI:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),    "TRK %03.0lf", leg1->course.magnetic);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),           "(%s)", "INTC");
            break;
        case NDT_LEGTYPE_VI:
            snprintf(leg1->info.misc, sizeof(leg1->info.misc),    "HDG %03.0lf", leg1->heading.degrees);
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),           "(%s)", "INTC");
            break;
        case NDT_LEGTYPE_FM:
        case NDT_LEGTYPE_VM:
            snprintf(leg1->info.idnt, sizeof(leg1->info.idnt),           "(%s)", "VECTOR");
            break;
        default:
            ndt_log("procedure_appendleg: unknown leg \"%s\" of type %d\n", line, leg1->type);
            ndt_route_leg_close(&leg1);
            return EINVAL;
    }
    snprintf(leg1->info.desc, sizeof(leg1->info.desc), "%s", line);

    // check for missed approach legs
    if (spcial == 3 || ndt_list_count(proc->mapplegs))
    {
        if (!proc->mapplegs)
        {
            ndt_log("procedure_appendleg: procedure %s is not FINAL\n", proc->info.idnt);
            ndt_route_leg_close(&leg1);
            return EINVAL;
        }
        ndt_list_add(proc->mapplegs, leg1);
        return 0;
    }

    // if it's not part of the missed approach, it's a standard leg
    ndt_list_add(proc->proclegs, leg1);
    return 0;
}

static int parse_procedures(char *src, ndt_navdatabase *ndb, ndt_airport *apt)
{
    char *pos = src;
    char *line = NULL;
    int i, linecap, ret = 0;
    ndt_procedure *proc = NULL;
    size_t raw_len, src_len = strlen(src);

    while ((ret = ndt_file_getline(&line, &linecap, &pos)) > 0)
    {
        if (*line == '\r' || *line == '\n')
        {
            continue; // skip blank lines
        }
        for (i = 0; line[i] != '\r' && line[i] != '\n'; i++)
        {
            continue;
        }
        line[i] = '\0'; // make line printable as a string

        int  anythg, segtyp;
        char procid[11], rwy_id[6], wpt_id[6], apptyp[2];

        if (!strncmp(line, "SID,", 4))
        {
            if (proc)
            {
                if (proc->raw_data)
                {
                    proc->raw_data = realloc(proc->raw_data, strlen(proc->raw_data) + 1);
                }
                proc = NULL;
            }

            /*
             * Format: CSV
             * - 'SID'
             * - identifier: SID
             * - identifier: runway or transition
             * - segment (1-6)
             */
            if (sscanf(line, "SID,%10[^,],%5[^,],%d",
                       procid, rwy_id, &segtyp) != 3)
            {
                ret = EINVAL;
                goto end;
            }

            /*
             * Recode some SID types, for example (Navigraph, AIRAC 1510):
             *
             * - LSGG SIDs of type 2, 5 are runway-specific (should be 1, 4)
             */
            if ((segtyp == 2 || segtyp == 5) && strcmp(rwy_id, "ALL") != 0)
            {
                segtyp--; // SID is runway-specific, thus becomes runway->SID
            }
            switch (segtyp)
            {
                case 1:
                    proc = ndt_procedure_init(NDT_PROCTYPE_SID_1);
                    break;
                case 2:
                    proc = ndt_procedure_init(NDT_PROCTYPE_SID_2);
                    break;
                case 3:
                    proc = ndt_procedure_init(NDT_PROCTYPE_SID_3);
                    break;
                case 4:
                    proc = ndt_procedure_init(NDT_PROCTYPE_SID_4);
                    break;
                case 5:
                    proc = ndt_procedure_init(NDT_PROCTYPE_SID_5);
                    break;
                case 6:
                    proc = ndt_procedure_init(NDT_PROCTYPE_SID_6);
                    break;
                default:
                    ret = EINVAL;
                    goto end;
            }
            if (!proc)
            {
                ret = ENOMEM;
                goto end;
            }

            snprintf(proc->info.idnt, sizeof(proc->info.idnt), "%s", procid);
            snprintf(proc->info.misc, sizeof(proc->info.misc), "%s", rwy_id);
            snprintf(proc->info.desc, sizeof(proc->info.desc), "%s", line);
            ndt_list_add(apt->allprocs, proc); proc->apt = apt;
            continue; // procedure ready
        }

        if (!strncmp(line, "STAR,", 5))
        {
            if (proc)
            {
                if (proc->raw_data)
                {
                    proc->raw_data = realloc(proc->raw_data, strlen(proc->raw_data) + 1);
                }
                proc = NULL;
            }

            /*
             * Format: CSV
             * - 'STAR'
             * - identifier: STAR
             * - identifier: runway or transition
             * - segment (1-9)
             */
            if (sscanf(line, "STAR,%10[^,],%5[^,],%d",
                       procid, rwy_id, &segtyp) != 3)
            {
                ret = EINVAL;
                goto end;
            }

            /*
             * Recode some STAR types, for example (Navigraph, AIRAC 1510):
             *
             * - LSGG STARs of type 2, 5 are runway-specific (should be 3, 6)
             */
            if ((segtyp == 2 || segtyp == 5 || segtyp == 8) && strcmp(rwy_id, "ALL") != 0)
            {
                segtyp++; // STAR is runway-specific, thus becomes STAR->runway
            }
            switch (segtyp)
            {
                case 1:
                    proc = ndt_procedure_init(NDT_PROCTYPE_STAR1);
                    break;
                case 2:
                    proc = ndt_procedure_init(NDT_PROCTYPE_STAR2);
                    break;
                case 3:
                    proc = ndt_procedure_init(NDT_PROCTYPE_STAR3);
                    break;
                case 4:
                    proc = ndt_procedure_init(NDT_PROCTYPE_STAR4);
                    break;
                case 5:
                    proc = ndt_procedure_init(NDT_PROCTYPE_STAR5);
                    break;
                case 6:
                    proc = ndt_procedure_init(NDT_PROCTYPE_STAR6);
                    break;
                case 7:
                    proc = ndt_procedure_init(NDT_PROCTYPE_STAR7);
                    break;
                case 8:
                    proc = ndt_procedure_init(NDT_PROCTYPE_STAR8);
                    break;
                case 9:
                    proc = ndt_procedure_init(NDT_PROCTYPE_STAR9);
                    break;
                default:
                    ret = EINVAL;
                    goto end;
            }
            if (!proc)
            {
                ret = ENOMEM;
                goto end;
            }

            snprintf(proc->info.idnt, sizeof(proc->info.idnt), "%s", procid);
            snprintf(proc->info.misc, sizeof(proc->info.misc), "%s", rwy_id);
            snprintf(proc->info.desc, sizeof(proc->info.desc), "%s", line);
            ndt_list_add(apt->allprocs, proc); proc->apt = apt;
            continue; // procedure ready
        }

        if (!strncmp(line, "APPTR,", 6))
        {
            if (proc)
            {
                if (proc->raw_data)
                {
                    proc->raw_data = realloc(proc->raw_data, strlen(proc->raw_data) + 1);
                }
                proc = NULL;
            }

            /*
             * Format: CSV
             * - 'APPTR'
             * - identifier: approach
             * - identifier: runway
             * - transition name
             */
            if (sscanf(line, "APPTR,%10[^,],%4[^,],%5[^,]",
                       procid, rwy_id, wpt_id) != 3)
            {
                ret = EINVAL;
                goto end;
            }

            proc = ndt_procedure_init(NDT_PROCTYPE_APPTR);
            if (!proc)
            {
                ret = ENOMEM;
                goto end;
            }

            snprintf(proc->info.idnt, sizeof(proc->info.idnt), "%s", procid);
            snprintf(proc->info.misc, sizeof(proc->info.misc), "%s", wpt_id);
            snprintf(proc->info.desc, sizeof(proc->info.desc), "%s", line);
            ndt_list_add(apt->allprocs, proc); proc->apt = apt;
            continue; // procedure ready
        }

        if (!strncmp(line, "FINAL,", 6))
        {
            if (proc)
            {
                if (proc->raw_data)
                {
                    proc->raw_data = realloc(proc->raw_data, strlen(proc->raw_data) + 1);
                }
                proc = NULL;
            }

            /*
             * Format: CSV
             * - 'FINAL'
             * - identifier: approach
             * - identifier: runway
             * - approach type
             */
            if (sscanf(line, "FINAL,%10[^,],%4[^,],%1[^,],%d", procid, rwy_id, apptyp, &anythg) != 4 &&
                sscanf(line, "FINAL,%10[^,],%4[^,],%1[^,]",    procid, rwy_id, apptyp)          != 3)
            {
                ret = EINVAL;
                goto end;
            }

            proc = ndt_procedure_init(NDT_PROCTYPE_FINAL);
            if (!proc)
            {
                ret = ENOMEM;
                goto end;
            }

            proc->approach.type = NDT_APPRTYPE_UNK;
            switch (*procid)
            {
                // VOR-based
                case 'D':
                    proc->approach.type = NDT_APPRTYPE_VDM;
                    break;
                case 'S':
                case 'V':
                    proc->approach.type = NDT_APPRTYPE_VOR;
                    break;
                case 'T':
                    proc->approach.type = NDT_APPRTYPE_TAC;
                    break;
                // NDB-based
                case 'N':
                    proc->approach.type = NDT_APPRTYPE_NDB;
                    break;
                case 'Q':
                    proc->approach.type = NDT_APPRTYPE_NDM;
                    break;
                // LOC-based
                case 'B':
                    proc->approach.type = NDT_APPRTYPE_LBC;
                    break;
                case 'G':
                    proc->approach.type = NDT_APPRTYPE_IGS;
                    break;
                case 'I':
                    proc->approach.type = NDT_APPRTYPE_ILS;
                    break;
                case 'L':
                    proc->approach.type = NDT_APPRTYPE_LOC;
                    break;
                case 'X':
                    proc->approach.type = NDT_APPRTYPE_LDA;
                    break;
                // GPS-based
                case 'H':
                    proc->approach.type = NDT_APPRTYPE_RNP;
                    break;
                case 'J':
                    proc->approach.type = NDT_APPRTYPE_GLS;
                    break;
                case 'P':
                    proc->approach.type = NDT_APPRTYPE_GPS;
                    break;
                case 'R':
                    proc->approach.type = NDT_APPRTYPE_RNV;
                    break;
                // other
                case 'F':
                    proc->approach.type = NDT_APPRTYPE_FMS;
                    break;
                case 'M':
                case 'W':
                case 'Y':
                    proc->approach.type = NDT_APPRTYPE_MLS;
                    break;
                case 'U':
                    proc->approach.type = NDT_APPRTYPE_SDF;
                    break;
                default:
                    break;
            }
            snprintf(proc->approach.short_name, sizeof(proc->approach.short_name), "%s", procid);
            snprintf(proc->          info.idnt, sizeof(proc->          info.idnt), "%s", procid);
            snprintf(proc->          info.misc, sizeof(proc->          info.misc), "%s", rwy_id);
            snprintf(proc->          info.desc, sizeof(proc->          info.desc), "%s", line);
            ndt_list_add(apt->allprocs, proc); proc->apt = apt;
            continue; // procedure ready
        }

        if (proc == NULL)
        {
            ret = EINVAL;
            goto end;
        }

        // add line to procedure's raw data for later parsing
        if (proc->raw_data == NULL)
        {
            if ((proc->raw_data = malloc(src_len)) == NULL) // shrink it later
            {
                ret = ENOMEM;
                goto end;
            }
            proc->raw_data[0] = 0; // stringify
        }
        raw_len = strlen(proc->raw_data);
        snprintf(proc->raw_data + raw_len, src_len - raw_len, "%s\n", line);
    }

end:
    if (ret)
    {
        char *errstr = NULL;
        char  errbuf[64];
#ifdef _GNU_SOURCE
        // GNU-specific strerror_r() variant
        errstr = strerror_r(ret, errbuf, sizeof(errbuf));
#else
        int errcode = strerror_r(ret, errbuf, sizeof(errbuf));
        if (errcode != 0 && errcode != EINVAL)
        {
            goto linefree;
        }
#endif
        switch (ret)
        {
            case EINVAL:
                ndt_log("[ndb_xpgns] parse_procedures: failed to parse \"%s\"\n", line);
                break;

            default:
                ndt_log("[ndb_xpgns] parse_procedures: '%.63s'\n", errstr ? errstr : errbuf);
                break;
        }
    }
linefree:
    free(line);
    return ret;
}

static int check_unprocessed(ndt_list *list, ndt_procedure *proc)
{
    for (size_t i = 0; i < ndt_list_count(list); i++)
    {
        ndt_procedure *item = ndt_list_item(list, i);
        if (item == proc)
        {
            return EINVAL;
        }
    }
    return 0;
}

static int place_procedures(ndt_navdatabase *ndb, ndt_airport *apt)
{
    ndt_procedure *proc1, *proc2;
    ndt_runway *rwy;
    int ret = 0;

    ndt_list *allprocs = ndt_list_init();
    if (!allprocs)
    {
        ret = ENOMEM;
        goto end;
    }

    /*
     * Store unprocessed procedures in a new
     * list so we can check for orphans later.
     */
    for (size_t i = 0; i < ndt_list_count(apt->allprocs); i++)
    {
        if (!(proc1 = ndt_list_item(apt->allprocs, i)))
        {
            ret = ENOMEM;
            goto end;
        }
        if (proc1->type != NDT_PROCTYPE_STAR7 &&
            proc1->type != NDT_PROCTYPE_STAR8 &&
            proc1->type != NDT_PROCTYPE_STAR9)
        {
            ndt_list_add(allprocs, proc1); // supported procedure
        }
        else
        {
            ndt_log("[ndb_xpgns] place_procedures: %s: unsupported procedure \"%s\"\n",
                    apt->info.idnt, proc1->info.desc); // warning
        }
    }

    /*
     * Runway specific SID segments plug into the corresponding runway's list.
     */
    for (size_t i = 0; i < ndt_list_count(allprocs);)
    {
        if (!(proc1 = ndt_list_item(allprocs, i)))
        {
            ret = ENOMEM;
            goto end;
        }
        if (proc1->type != NDT_PROCTYPE_SID_1 && proc1->type != NDT_PROCTYPE_SID_4)
        {
            i++; continue; // not runway-specific SID segment
        }
        if (!(rwy = ndt_runway_get(apt->runways, proc1->info.misc)))
        {
            // Aerosoft/1405/OIFP: runway 08/26 only, but procedures coded for 08L/26R
            ndt_list_rem(allprocs, proc1); continue; // skip
        }
        ndt_list_add(proc1->runways, rwy);
        ndt_list_add(rwy->sids,    proc1);
        ndt_list_add(apt->sids,    proc1);
        ndt_list_rem(allprocs,     proc1);
    }

    /*
     * Runway-agnostic SID segments plug into the corresponding runway-specific
     * segment's transition (if applicable), else plug into all runways' lists.
     */
    for (size_t i = 0; i < ndt_list_count(allprocs);)
    {
        if (!(proc1 = ndt_list_item(allprocs, i)))
        {
            ret = ENOMEM;
            goto end;
        }
        if (proc1->type != NDT_PROCTYPE_SID_2 && proc1->type != NDT_PROCTYPE_SID_5)
        {
            i++; continue; // not runway-agnostic SID segment
        }
        if ((proc2 = ndt_procedure_get(apt->sids, proc1->info.idnt, NULL)))
        {
            // this SID segment continues one or more runway-specific segments
            for (size_t j = 0; j < ndt_list_count(apt->runways); j++)
            {
                if (!(rwy = ndt_list_item(apt->runways, j)))
                {
                    ret = ENOMEM;
                    goto end;
                }
                if ((proc2 = ndt_procedure_get(rwy->sids, proc1->info.idnt, rwy)))
                {
                    (proc2->transition.sid) = proc1;
                }
            }
            ndt_list_rem(allprocs, proc1);
        }
        else
        {
            // this SID segment is standalone and thus applies to all runways
            for (size_t j = 0; j < ndt_list_count(apt->runways); j++)
            {
                if (!(rwy = ndt_list_item(apt->runways, j)))
                {
                    ret = ENOMEM;
                    goto end;
                }
                ndt_list_add(proc1->runways, rwy);
                ndt_list_add(rwy->sids,    proc1);
            }
            // add once to airport's list
            ndt_list_add(apt->sids, proc1);
            ndt_list_rem(allprocs,  proc1);
        }
    }

    /*
     * SID enroute transitions plug into all matching SID segments.
     */
    for (size_t i = 0; i < ndt_list_count(allprocs);)
    {
        if (!(proc1 = ndt_list_item(allprocs, i)))
        {
            ret = ENOMEM;
            goto end;
        }
        if (proc1->type != NDT_PROCTYPE_SID_3 && proc1->type != NDT_PROCTYPE_SID_6)
        {
            i++; continue; // not SID enroute transition
        }
        for (size_t j = 0; j < ndt_list_count(apt->runways); j++)
        {
            if (!(rwy = ndt_list_item(apt->runways, j)))
            {
                ret = ENOMEM;
                goto end;
            }
            if ((proc2 = ndt_procedure_get(rwy->sids, proc1->info.idnt, rwy)))
            {
                // note: the same procedure may be stored in multiple runways'
                //       lists, so we have to make sure we don't add it twice
                if (!ndt_procedure_gettr(proc2->transition.enroute,
                                         proc1->info.misc))
                {
                    ndt_list_add(proc2->transition.enroute, proc1);
                }
                if ((proc2 = proc2->transition.sid))
                {
                    // add it to runway-agnostic segment too (ease of access)
                    if (!ndt_procedure_gettr(proc2->transition.enroute,
                                             proc1->info.misc))
                    {
                        ndt_list_add(proc2->transition.enroute, proc1);
                    }
                }
                // Aerosoft/1405/KRNO: PVINE1 (SID) transitions duplicated
                // just drop the duplicate set of transitions to the floor
                ndt_list_rem(allprocs, proc1);
            }
        }
        if ((ret = check_unprocessed(allprocs, proc1)))
        {
            goto unprocessed;
        }
    }

    /*
     * Runway specific STAR segments plug into the corresponding runway's list.
     */
    for (size_t i = 0; i < ndt_list_count(allprocs);)
    {
        if (!(proc1 = ndt_list_item(allprocs, i)))
        {
            ret = ENOMEM;
            goto end;
        }
        if (proc1->type != NDT_PROCTYPE_STAR3 && proc1->type != NDT_PROCTYPE_STAR6)
        {
            i++; continue; // not runway-specific STAR segment
        }
        if (!(rwy = ndt_runway_get(apt->runways, proc1->info.misc)))
        {
            // Navigraph/1510/WSSS: runway 02R/20L closed, but procedures coded
            ndt_list_rem(allprocs, proc1); continue; // skip
        }
        ndt_list_add(proc1->runways, rwy);
        ndt_list_add(rwy->stars,   proc1);
        ndt_list_add(apt->stars,   proc1);
        ndt_list_rem(allprocs,     proc1);
    }

    /*
     * Runway-agnostic STAR segment plugs into the corresponding runway-specific
     * segment's transition (if applicable), else plugs into all runways' lists.
     */
    for (size_t i = 0; i < ndt_list_count(allprocs);)
    {
        if (!(proc1 = ndt_list_item(allprocs, i)))
        {
            ret = ENOMEM;
            goto end;
        }
        if (proc1->type != NDT_PROCTYPE_STAR2 && proc1->type != NDT_PROCTYPE_STAR5)
        {
            i++; continue; // not runway-agnostic STAR segment
        }
        if ((proc2 = ndt_procedure_get(apt->stars, proc1->info.idnt, NULL)))
        {
            // this STAR segment continues one or more runway-specific segments
            for (size_t j = 0; j < ndt_list_count(apt->runways); j++)
            {
                if (!(rwy = ndt_list_item(apt->runways, j)))
                {
                    ret = ENOMEM;
                    goto end;
                }
                if ((proc2 = ndt_procedure_get(rwy->stars, proc1->info.idnt, rwy)))
                {
                    (proc2->transition.star) = proc1;
                }
            }
            ndt_list_rem(allprocs, proc1);
        }
        else
        {
            // this STAR segment is standalone and thus applies to all runways
            for (size_t j = 0; j < ndt_list_count(apt->runways); j++)
            {
                if (!(rwy = ndt_list_item(apt->runways, j)))
                {
                    ret = ENOMEM;
                    goto end;
                }
                ndt_list_add(proc1->runways, rwy);
                ndt_list_add(rwy->stars,   proc1);
            }
            // add once to airport's list
            ndt_list_add(apt->stars, proc1);
            ndt_list_rem(allprocs,   proc1);
        }
    }

    /*
     * STAR enroute transitions plug into all matching STAR segments.
     */
    for (size_t i = 0; i < ndt_list_count(allprocs);)
    {
        if (!(proc1 = ndt_list_item(allprocs, i)))
        {
            ret = ENOMEM;
            goto end;
        }
        if (proc1->type != NDT_PROCTYPE_STAR1 && proc1->type != NDT_PROCTYPE_STAR4)
        {
            i++; continue; // not STAR enroute transition
        }
        for (size_t j = 0; j < ndt_list_count(apt->runways); j++)
        {
            if (!(rwy = ndt_list_item(apt->runways, j)))
            {
                ret = ENOMEM;
                goto end;
            }
            if ((proc2 = ndt_procedure_get(rwy->stars, proc1->info.idnt, rwy)))
            {
                // note: the same procedure may be stored in multiple runways'
                //       lists, so we have to make sure we don't add it twice
                if (!ndt_procedure_gettr(proc2->transition.enroute,
                                         proc1->info.misc))
                {
                    ndt_list_add(proc2->transition.enroute, proc1);
                }
                if ((proc2 = proc2->transition.star))
                {
                    // add it to runway-agnostic segment too (ease of access)
                    if (!ndt_procedure_gettr(proc2->transition.enroute,
                                             proc1->info.misc))
                    {
                        ndt_list_add(proc2->transition.enroute, proc1);
                    }
                }
                // Aerosoft/1405/KRNO: PVINE1 (SID) transitions duplicated
                // just drop the duplicate set of transitions to the floor
                ndt_list_rem(allprocs, proc1);
            }
        }
        if ((ret = check_unprocessed(allprocs, proc1)))
        {
            goto unprocessed;
        }
    }

    /*
     * Final approach procedures plug into the corresponding runway's list.
     */
    for (size_t i = 0; i < ndt_list_count(allprocs);)
    {
        if (!(proc1 = ndt_list_item(allprocs, i)))
        {
            ret = ENOMEM;
            goto end;
        }
        if (proc1->type != NDT_PROCTYPE_FINAL)
        {
            i++; continue; // not final approach
        }
        /* Mismatch between runway IDs in Airports.txt
         * and approach records in Proc/ICAO.txt files;
         * this doesn't seem to affect SID/STAR records.
         *
         * We don't do this earlier during parsing, as
         * it will interfere w/the procedure name code.
         */
        char rwy_id[sizeof(proc1->info.misc)];
        strncpy(rwy_id, proc1->info.misc, sizeof(rwy_id));
        size_t last = strnlen(rwy_id, sizeof(rwy_id)) - 1;
        if (rwy_id[last] == 'T')
        {
            rwy_id[last] = '\0';
        }
        if (!(rwy = ndt_runway_get(apt->runways, rwy_id)))
        {
            // Navigraph/1510/LOAV: runway 28 non-existent, but approaches coded
            ndt_list_rem(allprocs, proc1); continue; // skip
        }
        ndt_list_add(proc1->runways,    rwy);
        ndt_list_add(rwy->approaches, proc1);
        ndt_list_rem(allprocs,        proc1);
    }

    /*
     * Approach transitions plug into the corresponding final approach's list.
     */
    for (size_t i = 0; i < ndt_list_count(allprocs);)
    {
        if (!(proc1 = ndt_list_item(allprocs, i)))
        {
            ret = ENOMEM;
            goto end;
        }
        if (proc1->type != NDT_PROCTYPE_APPTR)
        {
            i++; continue; // not approach transition
        }
        for (size_t j = 0; j < ndt_list_count(apt->runways); j++)
        {
            if (!(rwy = ndt_list_item(apt->runways, j)))
            {
                ret = ENOMEM;
                goto end;
            }
            if ((proc2 = ndt_procedure_get(rwy->approaches, proc1->info.idnt, rwy)))
            {
                // the transition's approach type is the same as its parent
                proc1->approach.type   =   proc2->approach.type;
                ndt_list_add(proc1->runways,               rwy);
                ndt_list_add(proc2->transition.approach, proc1);
                ndt_list_rem(allprocs,                   proc1);
            }
        }
        if ((ret = check_unprocessed(allprocs, proc1)))
        {
            goto unprocessed;
        }
    }

    /*
     * All procedures should be assigned now, check for orphans;
     * this can be our bug or a navdata bug (never seen so far).
     */
    for (size_t i = 0; i < ndt_list_count(allprocs); i++)
    {
        if (!(proc1 = ndt_list_item(allprocs, i)))
        {
            ret = ENOMEM;
            goto end;
        }
        ret = EINVAL;
        goto unprocessed;
    }

end:
    ndt_list_close(&allprocs);
    return ret;

unprocessed:
    ndt_log("[ndb_xpgns] place_procedures: %s: unprocessed procedure \"%s\"\n",
            apt->info.idnt, proc1->info.desc);
    goto end;
}

static int rename_finalappr(ndt_airport *apt)
{
    ndt_runway    *rwy;
    ndt_procedure *final, *apptr;
    char temp[sizeof(final->info.idnt)];
    // now that all procedures are sorted, we can update final
    // approach procedure names to something more descriptive
    for (size_t i = 0; i < ndt_list_count(apt->runways); i++)
    {
        if ((rwy = ndt_list_item(apt->runways, i)))
        {
            for (size_t j = 0; j < ndt_list_count(rwy->approaches); j++)
            {
                if ((final = ndt_list_item(rwy->approaches, j)) &&
                    (final->type == NDT_PROCTYPE_FINAL))
                {
                    const char *prefix = NULL;
                    switch (final->approach.type) // acronym source: IXEG B733CL
                    {
                        // VOR-based
                        case NDT_APPRTYPE_VDM:
                            prefix = "VDM";
                            break;
                        case NDT_APPRTYPE_VOR:
                            prefix = "VOR";
                            break;
                        case NDT_APPRTYPE_TAC:
                            prefix = "TAC";
                            break;
                        // NDB-based
                        case NDT_APPRTYPE_NDB:
                            prefix = "NDB";
                            break;
                        case NDT_APPRTYPE_NDM:
                            prefix = "NDM";
                            break;
                        // LOC-based
                        case NDT_APPRTYPE_LBC:
                            prefix = "LBC";
                            break;
                        case NDT_APPRTYPE_LDA:
                            prefix = "LDA";
                            break;
                        case NDT_APPRTYPE_LOC:
                            prefix = "LOC";
                            break;
                        case NDT_APPRTYPE_IGS:
                            prefix = "IGS";
                            break;
                        case NDT_APPRTYPE_ILS:
                            prefix = "ILS";
                            break;
                        // GPS-based
                        case NDT_APPRTYPE_GLS:
                            prefix = "GLS";
                            break;
                        case NDT_APPRTYPE_GPS:
                            prefix = "GPS";
                            break;
                        case NDT_APPRTYPE_RNP:
                            prefix = "RNP";
                            break;
                        case NDT_APPRTYPE_RNV:
                            prefix = "RNV";
                            break;
                        // other
                        case NDT_APPRTYPE_FMS: // never found in any database(s)
                            prefix = "FMS";
                            break;
                        case NDT_APPRTYPE_MLS: // never found in any database(s)
                            prefix = "MLS";
                            break;
                        case NDT_APPRTYPE_SDF:
                            prefix = "SDF";
                            break;
                        default:
                            break;
                    }
                    if ((strlen(final->info.idnt) - strlen(final->info.misc)) > 1)
                    {
                        final->approach.suffix[0] = final->info.idnt[strlen(final->info.idnt) - 1];
                        final->approach.suffix[1] = '\0';
                    }
                    else
                    {
                        final->approach.suffix[0] = '\0';
                    }
                    if (prefix && strlen(final->info.idnt) > strlen(final->info.misc))
                    {
                        /*
                         * Max. 5 characters:
                         * D28R      VDM
                         * D28-A     VDM-A
                         * D28RY     VDM-Y
                         * D28R-Y    VDM-Y
                         */
                        snprintf(final->approach.short_name,
                                 sizeof(final->approach.short_name), "%s%s%s", prefix,
                                 *final->approach.suffix ? "-" : "", final->approach.suffix);
                        /*
                         * Max. 7 characters:
                         * D28R      VDM28R
                         * D28-A     VDM28-A
                         * D28RY     VDM28RY
                         * D28R-Y    VDM28RY
                         */
                        if (strnlen(final->info.misc, 3) < 3)
                        {
                            snprintf(temp, sizeof(temp), "%s%s%s%s", prefix, final->info.misc,
                                     *final->approach.suffix ? "-" : "", final->approach.suffix);
                        }
                        else
                        {
                            snprintf(temp, sizeof(temp), "%s%s%s", prefix, final->info.misc, final->approach.suffix);
                        }
                        /*
                         * Update IDs for final approach and its transitions.
                         */
                        for (size_t k = 0; k < ndt_list_count(final->transition.approach); k++)
                        {
                            if ((apptr = ndt_list_item(final->transition.approach, k)) &&
                                (apptr->type == NDT_PROCTYPE_APPTR))
                            {
                                snprintf(apptr->info.idnt, sizeof(apptr->info.idnt), "%s", temp);
                            }
                        }
                        snprintf(final->info.idnt, sizeof(final->info.idnt), "%s", temp);
                    }
                }
            }
        }
    }
    return 0;
}

static int open_a_procedure(ndt_navdatabase *ndb, ndt_procedure *proc)
{
    if (!ndb || !proc)
    {
        return -1;
    }
    if (proc->opened)
    {
        return 0;
    }
    if (proc->raw_data == NULL)
    {
        return -1;
    }

    char *line = NULL;
    ndt_position posn;
    ndt_distance dist;
    ndt_route_leg *leg1;
    int linecap, i, ret = 0;
    char *pos  = proc->raw_data;
    ndt_restriction constraints;
    ndt_waypoint  *wpt1 = NULL, *wpt2, *wpt3;

    while ((ret = ndt_file_getline(&line, &linecap, &pos)) > 0)
    {
        if (*line == '\r' || *line == '\n')
        {
            continue; // skip blank lines
        }
        for (i = 0; line[i] != '\r' && line[i] != '\n'; i++)
        {
            continue;
        }
        line[i] = '\0'; // make line printable as a string

        int    altcst, altone, alttwo, spdcst, spdone, spdtwo;
        int    anythg, segtyp, turndr, spcial, ovrfly, mapfix;
        double wptlat, wptlon, navbrg, navdis, navrad, magcrs;
        double legdis, dmedis, stradl, radius, intcrs;
        char   wpt_id[6], nav_id[6];

        if (!strncmp(line, "AF,", 3))
        {
            /*
             * Format: CSV
             * - 'AF'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - navaid identifier
             * - sweep angle                (unit: deg)
             * - DME distance               (unit: nmi)
             * - start radial               (unit: deg)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "AF,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navrad, &dmedis, &stradl,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 16)
            {
                ret = EINVAL;
                goto end;
            }

            dist = ndt_distance_init(dmedis * 1852. / .3048, NDT_ALTUNIT_FT);
            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt2 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt2 && !(wpt2 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(wpt3 = ndt_navdata_get_wptnear2(ndb, nav_id, NULL, wpt2->position)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->arc.start    = stradl;
            leg1->arc.stop     = navrad;
            leg1->arc.center   = wpt3;
            leg1->arc.distance = dist;
            leg1->src          = wpt1;
            leg1->dst          = wpt1 = wpt2;
            leg1->type         = NDT_LEGTYPE_AF;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "CA,", 3))
        {
            /*
             * Format: CSV
             * - 'CA'
             * - turn direction
             * - magnetic course            (unit: deg)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "CA,%d,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &turndr, &magcrs,
                       &altcst, &altone, &alttwo,
                       &spdcst, &spdone, &spdtwo,
                       &spcial, &ovrfly) != 10)
            {
                ret = EINVAL;
                goto end;
            }

            dist = ndt_distance_init(altone, NDT_ALTUNIT_FT);
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints,      2, altone,      0);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->course.magnetic = magcrs;
            leg1->course.altitude = dist;
            leg1->src             = wpt1;
            leg1->dst             = wpt1 = NULL;
            leg1->type            = NDT_LEGTYPE_CA;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "CD,", 3))
        {
            /*
             * Format: CSV
             * - 'CD'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - navaid identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - magnetic course            (unit: deg)
             * - DME distance               (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "CD,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis, &magcrs, &dmedis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 17)
            {
                ret = EINVAL;
                goto end;
            }

            dist = ndt_distance_init(dmedis * 1852. / .3048, NDT_ALTUNIT_FT);
            if (!(wpt3 = ndt_navdata_get_wptnear2(ndb, nav_id, NULL, wpt1 ? wpt1->position : proc->apt->coordinates)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->course.magnetic = magcrs;
            leg1->course.distance = dist;
            leg1->course.navaid   = wpt3;
            leg1->src             = wpt1;
            leg1->dst             = wpt1 = NULL;
            leg1->type            = NDT_LEGTYPE_CD;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "CI,", 3))
        {
            /*
             * Format: CSV
             * - 'CI'
             * - turn direction
             * - navaid identifier
             * - intercept course           (unit: deg)
             * - magnetic course            (unit: deg)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "CI,%d,%5[^,],%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &turndr,
                       &nav_id[0], &intcrs, &magcrs,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 12)
            {
                ret = EINVAL;
                goto end;
            }

            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->course.magnetic = magcrs;
            leg1->src             = wpt1;
            leg1->dst             = wpt1 = NULL;
            leg1->type            = NDT_LEGTYPE_CI;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "CF,", 3))
        {
            /*
             * Format: CSV
             * - 'CF'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - navaid identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - magnetic course            (unit: deg)
             * - leg distance               (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "CF,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis, &magcrs, &legdis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 17)
            {
                ret = EINVAL;
                goto end;
            }

            dist = ndt_distance_init(legdis * 1852. / .3048, NDT_ALTUNIT_FT);
            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt2 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt2 && !(wpt2 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->course.magnetic = magcrs;
            leg1->course.distance = dist;
            leg1->src             = wpt1;
            leg1->dst             = wpt1 = wpt2;
            leg1->type            = NDT_LEGTYPE_CF;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "CR,", 3))
        {
            /*
             * Format: CSV
             * - 'CR'
             * - turn direction
             * - navaid identifier
             * - navaid radial              (unit: deg)
             * - magnetic course            (unit: deg)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "CR,%d,%5[^,],%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &turndr,
                       &nav_id[0], &navrad, &magcrs,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 12)
            {
                ret = EINVAL;
                goto end;
            }

            if (!(wpt3 = ndt_navdata_get_wptnear2(ndb, nav_id, NULL, wpt1 ? wpt1->position : proc->apt->coordinates)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->course.magnetic = magcrs;
            leg1->course.radial   = navrad;
            leg1->course.navaid   = wpt3;
            leg1->src             = wpt1;
            leg1->dst             = wpt1 = NULL;
            leg1->type            = NDT_LEGTYPE_CR;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "DF,", 3))
        {
            /*
             * Format: CSV
             * - 'DF'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - navaid identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "DF,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 15)
            {
                ret = EINVAL;
                goto end;
            }

            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt2 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt2 && !(wpt2 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->src  = wpt1;
            leg1->dst  = wpt1 = wpt2;
            leg1->type = NDT_LEGTYPE_DF;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "FA,", 3))
        {
            /*
             * Format: CSV
             * - 'FA'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - navaid identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - magnetic course            (unit: deg)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "FA,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis, &magcrs,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 16)
            {
                ret = EINVAL;
                goto end;
            }

            dist = ndt_distance_init(altone, NDT_ALTUNIT_FT);
            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt1 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt1 && !(wpt1 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints,      2, altone,      0);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->fix.course   = magcrs;
            leg1->fix.altitude = dist;
            leg1->fix.src      = wpt1;
            leg1->src          = wpt1;
            leg1->dst          = wpt1 = NULL;
            leg1->type         = NDT_LEGTYPE_FA;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "FC,", 3))
        {
            /*
             * Format: CSV
             * - 'FC'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - navaid identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - mangetic course            (unit: deg)
             * - along track distance       (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "FC,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis, &magcrs, &dmedis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 17)
            {
                ret = EINVAL;
                goto end;
            }

            dist = ndt_distance_init(dmedis * 1852. / .3048, NDT_ALTUNIT_FT);
            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt1 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt1 && !(wpt1 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->fix.course   = magcrs;
            leg1->fix.distance = dist;
            leg1->fix.src      = wpt1;
            leg1->src          = wpt1;
            leg1->dst          = wpt1 = NULL;
            leg1->type         = NDT_LEGTYPE_FC;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "FD,", 3))
        {
            /*
             * Format: CSV
             * - 'FD'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - DME identifier
             * - waypoint bearing           (unit: deg)
             * - DME distance               (unit: nmi)
             * - magnetic course            (unit: deg)
             * - leg distance               (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "FD,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &dmedis, &magcrs, &legdis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 17)
            {
                ret = EINVAL;
                goto end;
            }

            dist = ndt_distance_init(dmedis * 1852. / .3048, NDT_ALTUNIT_FT);
            wpt1 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt1 && !(wpt1 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(wpt3 = ndt_navdata_get_wptnear2(ndb, nav_id, NULL, wpt1->position)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->fix.course   = magcrs;
            leg1->fix.distance = dist;
            leg1->fix.navaid   = wpt3;
            leg1->fix.src      = wpt1;
            leg1->src          = wpt1;
            leg1->dst          = wpt1 = NULL;
            leg1->type         = NDT_LEGTYPE_FD;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "FM,", 3))
        {
            /*
             * Format: CSV
             * - 'FM'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - navaid identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - mangetic course            (unit: deg)
             * - along track distance       (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "FM,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis, &magcrs,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 16)
            {
                ret = EINVAL;
                goto end;
            }

            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt1 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt1 && !(wpt1 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->fix.course = magcrs;
            leg1->fix.src    = wpt1;
            leg1->src        = wpt1;
            leg1->dst        = wpt1 = NULL;
            leg1->type       = NDT_LEGTYPE_FM;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "IF,", 3))
        {
            /*
             * Format: CSV
             * - 'IF'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - navaid identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "IF,%5[^,],%lf,%lf,%5[^,],%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon,
                       &nav_id[0], &navbrg, &navdis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 14)
            {
                ret = EINVAL;
                goto end;
            }

            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt2 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt2 && !(wpt2 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->src  = wpt1;
            leg1->dst  = wpt1 = wpt2;
            leg1->type = NDT_LEGTYPE_IF;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "PI,", 3))
        {
            /*
             * Format: CSV
             * - 'PI'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - fix identifier
             * - first turn                 (unit: deg)
             * - turn limit to fix          (unit: nmi)
             * - outbound from fix          (unit: deg)
             * - outbound from fix          (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "PI,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis, &magcrs, &dmedis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 17)
            {
                ret = EINVAL;
                goto end;
            }

            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt1 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt1 && !(wpt1 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(wpt3 = ndt_navdata_get_wptnear2(ndb, nav_id, NULL, wpt1->position)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->turn.limdis   = ndt_distance_init(navdis * 1852. / .3048, NDT_ALTUNIT_FT);
            leg1->turn.outdis   = ndt_distance_init(dmedis * 1852. / .3048, NDT_ALTUNIT_FT);
            leg1->turn.tangle   = ndt_position_bearing_angle(magcrs, navbrg);
            leg1->turn.outbrg   = magcrs;
            leg1->turn.navaid   = wpt3;
            leg1->turn.waypoint = wpt1;
            leg1->src           = wpt1;
            leg1->dst           = wpt1 = NULL;
            leg1->type          = NDT_LEGTYPE_PI;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "RF,", 3))
        {
            /*
             * Format: CSV
             * - 'RF'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - arc center identifier
             * - sweep angle                (unit: deg)
             * - radius                     (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "RF,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navrad, &radius,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 15)
            {
                ret = EINVAL;
                goto end;
            }

            dist = ndt_distance_init(radius * 1852. / .3048, NDT_ALTUNIT_FT);
            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt2 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt2 && !(wpt2 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(wpt3 = ndt_navdata_get_wptnear2(ndb, nav_id, NULL, wpt2->position)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->radius.start    = navrad;
            leg1->radius.distance = dist;
            leg1->radius.center   = wpt3;
            leg1->src             = wpt1;
            leg1->dst             = wpt1 = wpt2;
            leg1->type            = NDT_LEGTYPE_RF;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "TF,", 3))
        {
            /*
             * Format: CSV
             * - 'TF'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - navaid identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - magnetic course            (unit: deg)
             * - leg distance               (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "TF,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis, &magcrs, &legdis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 17)
            {
                ret = EINVAL;
                goto end;
            }

            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt2 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt2 && !(wpt2 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->src  = wpt1;
            leg1->dst  = wpt1 = wpt2;
            leg1->type = NDT_LEGTYPE_TF;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "VA,", 3))
        {
            /*
             * Format: CSV
             * - 'VA'
             * - turn direction
             * - heading                    (unit: deg)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "VA,%d,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &turndr, &magcrs,
                       &altcst, &altone, &alttwo,
                       &spdcst, &spdone, &spdtwo,
                       &spcial, &ovrfly) != 10)
            {
                ret = EINVAL;
                goto end;
            }

            dist = ndt_distance_init(altone, NDT_ALTUNIT_FT);
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints,      2, altone,      0);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->heading.degrees  = magcrs;
            leg1->heading.altitude = dist;
            leg1->src              = wpt1;
            leg1->dst              = wpt1 = NULL;
            leg1->type             = NDT_LEGTYPE_VA;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "VD,", 3))
        {
            /*
             * Format: CSV
             * - 'VD'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - DME identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - heading                    (unit: deg)
             * - DME distance               (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "VD,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis, &magcrs, &dmedis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 17)
            {
                ret = EINVAL;
                goto end;
            }

            dist = ndt_distance_init(dmedis * 1852. / .3048, NDT_ALTUNIT_FT);
            if (!(wpt3 = ndt_navdata_get_wptnear2(ndb, nav_id, NULL, wpt1 ? wpt1->position : proc->apt->coordinates)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->heading.degrees  = magcrs;
            leg1->heading.distance = dist;
            leg1->heading.navaid   = wpt3;
            leg1->src              = wpt1;
            leg1->dst              = wpt1 = NULL;
            leg1->type             = NDT_LEGTYPE_VD;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "VI,", 3))
        {
            /*
             * Format: CSV
             * - 'VI'
             * - turn direction
             * - navaid identifier
             * - intercept course           (unit: deg)
             * - heading                    (unit: deg)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "VI,%d,%5[^,],%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &turndr,
                       &nav_id[0], &intcrs, &magcrs,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 12)
            {
                ret = EINVAL;
                goto end;
            }

            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->heading.degrees = magcrs;
            leg1->src             = wpt1;
            leg1->dst             = wpt1 = NULL;
            leg1->type            = NDT_LEGTYPE_VI;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "VM,", 3))
        {
            /*
             * Format: CSV
             * - 'VM'
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - heading                    (unit: deg)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "VM,%lf,%lf,%d,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wptlat, &wptlon, &turndr, &magcrs,
                       &altcst, &altone, &alttwo,
                       &spdcst, &spdone, &spdtwo,
                       &spcial, &ovrfly) != 12)
            {
                ret = EINVAL;
                goto end;
            }

            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->heading.degrees = magcrs;
            leg1->src             = wpt1;
            leg1->dst             = wpt1 = NULL;
            leg1->type            = NDT_LEGTYPE_VM;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "VR,", 3))
        {
            /*
             * Format: CSV
             * - 'VR'
             * - turn direction
             * - navaid identifier
             * - navaid radial              (unit: deg)
             * - heading                    (unit: deg)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             */
            if (sscanf(line, "VR,%d,%5[^,],%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d",
                       &turndr,
                       &nav_id[0], &navrad, &magcrs,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly) != 12)
            {
                ret = EINVAL;
                goto end;
            }

            if (!(wpt3 = ndt_navdata_get_wptnear2(ndb, nav_id, NULL, wpt1 ? wpt1->position : proc->apt->coordinates)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            leg1->heading.degrees = magcrs;
            leg1->heading.radial  = navrad;
            leg1->heading.navaid  = wpt3;
            leg1->src             = wpt1;
            leg1->dst             = wpt1 = NULL;
            leg1->type            = NDT_LEGTYPE_VR;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "HF,", 3))
        {
            /*
             * Format: CSV
             * - 'HF'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - navaid identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - inbound course             (unit: deg)
             * - leg distance               (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             * - hold distance type
             */
            if (sscanf(line,
                       "HF,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis, &magcrs, &legdis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly, &segtyp) != 18)
            {
                ret = EINVAL;
                goto end;
            }

            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt2 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt2 && !(wpt2 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            switch (segtyp)
            {
                case 1:
                    leg1->hold.type     = NDT_HOLD_SECONDS;
                    leg1->hold.duration = legdis;
                    break;
                case 0:
                default:
                    leg1->hold.type     = NDT_HOLD_DISTANCE;
                    leg1->hold.distance = ndt_distance_init(legdis * 1852. / .3048, NDT_ALTUNIT_FT);
                    break;
            }
            leg1->hold.inbound_course = magcrs;
            leg1->hold.waypoint       = wpt2;
            leg1->src                 = wpt2;
            leg1->dst                 = wpt1 = wpt2;
            leg1->type                = NDT_LEGTYPE_HF;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "HA,", 3))
        {
            /*
             * Format: CSV
             * - 'HA'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - navaid identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - inbound course             (unit: deg)
             * - leg distance               (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             * - hold distance type
             */
            if (sscanf(line,
                       "HA,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis, &magcrs, &legdis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly, &segtyp) != 18)
            {
                ret = EINVAL;
                goto end;
            }

            dist = ndt_distance_init(altone, NDT_ALTUNIT_FT);
            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt2 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt2 && !(wpt2 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints,      2, altone,      0);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            switch (segtyp)
            {
                case 1:
                    leg1->hold.type     = NDT_HOLD_SECONDS;
                    leg1->hold.duration = legdis;
                    break;
                case 0:
                default:
                    leg1->hold.type     = NDT_HOLD_DISTANCE;
                    leg1->hold.distance = ndt_distance_init(legdis * 1852. / .3048, NDT_ALTUNIT_FT);
                    break;
            }
            leg1->hold.inbound_course = magcrs;
            leg1->hold.altitude       = dist;
            leg1->hold.waypoint       = wpt2;
            leg1->src                 = wpt2;
            leg1->dst                 = wpt1 = wpt2;
            leg1->type                = NDT_LEGTYPE_HA;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        if (!strncmp(line, "HM,", 3))
        {
            /*
             * Format: CSV
             * - 'HM'
             * - waypoint identifier
             * - waypoint latitude          (unit: deg)
             * - waypoint longitude         (unit: deg)
             * - turn direction
             * - navaid identifier
             * - waypoint bearing           (unit: deg)
             * - waypoint distance          (unit: nmi)
             * - inbound course             (unit: deg)
             * - leg distance               (unit: nmi)
             * - constraints: altitude
             * - constraints: altitude 1    (unit:  ft)
             * - constraints: altitude 2    (unit:  ft)
             * - constraints: speed
             * - constraints: speed 1       (unit: kts)
             * - constraints: speed 2       (unit: kts)
             * - waypoint is special
             * - waypoint is overfly
             * - hold distance type
             */
            if (sscanf(line,
                       "HM,%5[^,],%lf,%lf,%d,%5[^,],%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                       &wpt_id[0], &wptlat, &wptlon, &turndr,
                       &nav_id[0], &navbrg, &navdis, &magcrs, &legdis,
                       &altcst,    &altone, &alttwo,
                       &spdcst,    &spdone, &spdtwo,
                       &spcial,    &ovrfly, &segtyp) != 18)
            {
                ret = EINVAL;
                goto end;
            }

            posn = ndt_position_init(wptlat, wptlon, NDT_DISTANCE_ZERO);
            wpt2 = ndt_navdata_get_wpt4pos(ndb, wpt_id, NULL, posn);
            if (!wpt2 && !(wpt2 = procedure_waypoint(wpt_id, wptlat, wptlon)))
            {
                ret = ENOMEM;
                goto end;
            }
            if (!(leg1 = ndt_route_leg_init()))
            {
                ret = ENOMEM;
                goto end;
            }

            handle_altitude_constraints(&constraints, altcst, altone, alttwo);
            handle_airspeed_constraints(&constraints, spdcst, spdone, spdtwo);
            handle_waypoint_constraints(&constraints, spcial, ovrfly);
            handle_turndrct_constraints(&constraints, turndr);
            ret = ndt_route_leg_restrict(leg1, constraints);
            if (ret)
            {
                ndt_route_leg_close(&leg1);
                goto end;
            }

            switch (segtyp)
            {
                case 1:
                    leg1->hold.type     = NDT_HOLD_SECONDS;
                    leg1->hold.duration = legdis;
                    break;
                case 0:
                default:
                    leg1->hold.type     = NDT_HOLD_DISTANCE;
                    leg1->hold.distance = ndt_distance_init(legdis * 1852. / .3048, NDT_ALTUNIT_FT);
                    break;
            }
            leg1->hold.inbound_course = magcrs;
            leg1->hold.waypoint       = wpt2;
            leg1->src                 = wpt2;
            leg1->dst                 = wpt1 = NULL;
            leg1->type                = NDT_LEGTYPE_HM;

            if ((ret = procedure_appendleg(proc, leg1, spcial, line)))
            {
                goto end;
            }
            continue; // leg ready
        }

        // unknown procedure or leg type
        ret = EINVAL;
        goto end;
    }

    // now that we have all legs, set transition waypoint if applicable
    if (proc->type == NDT_PROCTYPE_SID_3 || proc->type == NDT_PROCTYPE_SID_6)
    {
        ndt_route_leg *leg = ndt_list_item(proc->proclegs, -1);
        if (!leg)
        {
            ndt_log("[ndb_xpgns] open_a_procedure: %s: no leg(s) for %s.%s\n",
                    proc->apt->info.idnt, proc->info.idnt, proc->info.misc);
            ret = ENOMEM;
            goto end;
        }
        if (!leg->dst)
        {
            ndt_log("[ndb_xpgns] open_a_procedure: %s: no endpoint for %s.%s\n",
                    proc->apt->info.idnt, proc->info.idnt, proc->info.misc);
            ret = EINVAL;
            goto end;
        }
        proc->transition.wpt = leg->dst;
    }
    if (proc->type == NDT_PROCTYPE_STAR1 || proc->type == NDT_PROCTYPE_STAR4)
    {
        ndt_route_leg *leg = ndt_list_item(proc->proclegs, 0);
        if (!leg)
        {
            ndt_log("[ndb_xpgns] open_a_procedure: %s: no leg(s) for %s.%s\n",
                    proc->apt->info.idnt, proc->info.idnt, proc->info.misc);
            ret = ENOMEM;
            goto end;
        }
        if (leg->type == NDT_LEGTYPE_IF)
        {
            proc->transition.wpt = leg->dst;
        }
        else
        {
            if (!leg->src)
            {
                ndt_log("[ndb_xpgns] open_a_procedure: %s: no startpoint for %s.%s\n",
                        proc->apt->info.idnt, proc->info.idnt, proc->info.misc);
                ret = EINVAL;
                goto end;
            }
            proc->transition.wpt = leg->src;
        }
    }
    if (proc->type == NDT_PROCTYPE_APPTR)
    {
        ndt_route_leg *leg = ndt_list_item(proc->proclegs, 0);
        if (!leg)
        {
            ndt_log("[ndb_xpgns] open_a_procedure: %s: no leg(s) for %s.%s\n",
                    proc->apt->info.idnt, proc->info.idnt, proc->info.misc);
            ret = ENOMEM;
            goto end;
        }
        if (leg->type == NDT_LEGTYPE_IF)
        {
            proc->transition.wpt = leg->dst;
        }
        else
        {
            if (!leg->src)
            {
                ndt_log("[ndb_xpgns] open_a_procedure: %s: no startpoint for %s.%s\n",
                        proc->apt->info.idnt, proc->info.idnt, proc->info.misc);
                ret = EINVAL;
                goto end;
            }
            proc->transition.wpt = leg->src;
        }
    }

    // we're done here
    if (proc->raw_data)
    {
        free(proc->raw_data);
        proc->raw_data = NULL;
    }
    proc->opened = 1;

end:
    if (ret)
    {
        char *errstr = NULL;
        char  errbuf[64];
#ifdef _GNU_SOURCE
        // GNU-specific strerror_r() variant
        errstr = strerror_r(ret, errbuf, sizeof(errbuf));
#else
        int errcode = strerror_r(ret, errbuf, sizeof(errbuf));
        if (errcode != 0 && errcode != EINVAL)
        {
            goto linefree;
        }
#endif
        switch (ret)
        {
            case EINVAL:
                ndt_log("[ndb_xpgns] open_a_procedure: failed to parse \"%s\"\n", line);
                break;

            default:
                ndt_log("[ndb_xpgns] open_a_procedure: '%.63s'\n", errstr ? errstr : errbuf);
                break;
        }
    }
linefree:
    free(line);
    return ret;
}
