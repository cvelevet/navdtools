/*
 * ndb_xpgns.c
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

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "common/common.h"

#include "compat/compat.h"

#include "airport.h"
#include "airway.h"
#include "navdata.h"
#include "waypoint.h"

// check whether first decimal digit is odd
#define ODD_DEC1(F) (((int)(10 * F)) % 2)

static int parse_airac    (char *src, ndt_navdatabase *ndb);
static int parse_airports (char *src, ndt_navdatabase *ndb);
static int parse_airways  (char *src, ndt_navdatabase *ndb);
static int parse_navaids  (char *src, ndt_navdatabase *ndb);
static int parse_waypoints(char *src, ndt_navdatabase *ndb);

int ndt_ndb_xpgns_navdatabase_init(ndt_navdatabase *ndb, const char *ndr)
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
    char  *navaids      = NULL, *procedure = NULL, *waypoints = NULL;
    DIR   *procedures   = NULL;
    char  *path         = NULL;
    int    pathlen, ret = 0;

    if ((ret = ndt_file_getpath(ndr, "/cycle_info.txt", &path, &pathlen)))
    {
        goto end;
    }
    airac = ndt_file_slurp(path, &ret);

    if (ret || (ret = parse_airac(airac, ndb)))
    {
        goto end;
    }

    if ((ret = ndt_file_getpath(ndr, "/Airports.txt", &path, &pathlen)))
    {
        goto end;
    }
    airports = ndt_file_slurp(path, &ret);

    if (ret || (ret = parse_airports(airports, ndb)))
    {
        goto end;
    }

    if ((ret = ndt_file_getpath(ndr, "/ATS.txt", &path, &pathlen)))
    {
        goto end;
    }
    airways = ndt_file_slurp(path, &ret);

    if (ret || (ret = parse_airways(airways, ndb)))
    {
        goto end;
    }

    if ((ret = ndt_file_getpath(ndr, "/Navaids.txt", &path, &pathlen)))
    {
        goto end;
    }
    navaids = ndt_file_slurp(path, &ret);

    if (ret || (ret = parse_navaids(navaids, ndb)))
    {
        goto end;
    }

    if ((ret = ndt_file_getpath(ndr, "/Waypoints.txt", &path, &pathlen)))
    {
        goto end;
    }
    waypoints = ndt_file_slurp(path, &ret);

    if (ret || (ret = parse_waypoints(waypoints, ndb)))
    {
        goto end;
    }

    if ((ret = ndt_file_getpath(ndr, "/Proc", &path, &pathlen)))
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
    free(procedure);
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
    ndt_waypoint *wpt = NULL;
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
            int    elevation, transition;
            double latitude,  longitude;

            wpt = ndt_waypoint_init();
            if (!wpt)
            {
                ret = ENOMEM;
                goto end;
            }

            apt = ndt_airport_init();
            if (!apt)
            {
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
             * - UNKNOWN
             * - longest runway      (unit: ft)
             */
            if (sscanf(line, "%*c,%4s,%127[^,],%lf,%lf,%d,%d,%*d,%*d",
                       apt->info.idnt,
                       apt->info.misc,
                       &latitude,
                       &longitude,
                       &elevation,
                       &transition) != 6)
            {
                ndt_log("[ndb_xpgns] parse_airports: failed to parse \"");
                for (int i = 0; line[i] != '\r' && line[i] != '\n'; i++)
                {
                    ndt_log("%c", line[i]);
                }
                ndt_log("\"\n");
                ndt_airport_close(&apt);
                ret = EINVAL;
                goto end;
            }

            snprintf(apt->info.desc, sizeof(apt->info.desc),
                     "Airport %s (%s), elevation %d ft, t/altitude %d ft",
                     apt->info.idnt, apt->info.misc, elevation, transition);

            strncpy(wpt->region,    "",             sizeof(wpt->region));
            strncpy(wpt->info.idnt, apt->info.idnt, sizeof(wpt->info.idnt));
            strncpy(wpt->info.desc, apt->info.desc, sizeof(wpt->info.desc));

            wpt->type        = NDT_WPTYPE_APT;
            wpt->position    =
            apt->coordinates = ndt_position_init(latitude, longitude, ndt_distance_init(elevation, NDT_ALTUNIT_FT));
            apt->tr_altitude = ndt_distance_init(transition, NDT_ALTUNIT_FT);

            ndt_list_add(ndb->airports,  apt);
            ndt_list_add(ndb->waypoints, wpt);
            wpt = NULL;
            continue;
        }

        if (!strncmp(line, "R,", 2))
        {
            double      frequency, latitude, longitude;
            int         length, width, altitude;

            if (!apt)
            {
                ret = EINVAL;
                goto end;
            }

            wpt = ndt_waypoint_init();
            if (!wpt)
            {
                ret = ENOMEM;
                goto end;
            }

            ndt_runway *rwy = ndt_runway_init();
            if (!rwy)
            {
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
             * - UNKNOWN
             * - UNKNOWN
             * - UNKNOWN
             */
            if (sscanf(line,
                       "%*c,%4[^,],%d,%d,%d,%d,%lf,%d,%lf,%lf,%d", rwy->info.idnt,
                       &rwy->heading,
                       &length,
                       &width,
                       &rwy->ils.avail,
                       &frequency,
                       &rwy->ils.course,
                       &latitude,
                       &longitude,
                       &altitude) != 10)
            {
                ndt_runway_close(&rwy);
                ret = EINVAL;
                goto end;
            }

            snprintf(rwy->info.desc, sizeof(rwy->info.desc),
                     "%4s runway %4s, heading %03d°, length %5d ft, width %3d ft",
                     apt->info.idnt, rwy->info.idnt, rwy->heading, length, width);

            if (rwy->ils.avail)
            {
                size_t len     = strlen(rwy->info.desc);
                wpt->frequency = rwy->ils.freq = ndt_frequency_init(frequency);
                snprintf(rwy->info.desc + len, sizeof(rwy->info.desc) - len,
                         ", ILS %.3lf course %03d°",
                         ndt_frequency_get(rwy->ils.freq), rwy->ils.course);
            }

            strncpy (wpt->region, "", sizeof(wpt->region));
            snprintf(wpt->info.idnt,  sizeof(wpt->info.idnt), "%s%s",                   apt->info.idnt, rwy->info.idnt);
            snprintf(wpt->info.desc,  sizeof(wpt->info.desc), "Runway %s, airport: %s", rwy->info.idnt, apt->info.idnt);

            wpt->type      = NDT_WPTYPE_RWY;
            wpt->position  =
            rwy->threshold = ndt_position_init(latitude, longitude, ndt_distance_init(altitude, NDT_ALTUNIT_FT));
            rwy->length    = ndt_distance_init(length, NDT_ALTUNIT_FT);
            rwy->width     = ndt_distance_init(width,  NDT_ALTUNIT_FT);
            ndt_list_add(apt->runways,   rwy);
            ndt_list_add(ndb->waypoints, wpt);
            wpt = NULL;
            continue;
        }

        if (!strncmp(line, "X,", 2))
        {
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
        ndt_log("[ndb_xpgns] parse_airports: failed to parse \"");
        for (int i = 0; line[i] != '\r' && line[i] != '\n'; i++)
        {
            ndt_log("%c", line[i]);
        }
        ndt_log("\"\n");
    }
    if (wpt)
    {
        // note: we don't need to clean apt as it's guaranteed to be in the
        // list or already closed; thus global cleanup will take care of it
        ndt_waypoint_close(&wpt);
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
            if (sscanf(line, "%*c,%5[^,],%d", awy->info.idnt, &count_in) != 2)
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
             * - identifier
             * - latitude
             * - longitude
             * - identifier (next waypoint)
             * - latitude   (next waypoint)
             * - longitude  (next waypoint)
             * - UNKNOWN    (related to course between waypoints)
             * - UNKNOWN    (related to course between waypoints)
             * - distance between waypoint and next (unit: NM)
             */
            if (sscanf(line,
                       "%*c,%5[^,],%lf,%lf,%5[^,],%lf,%lf,%*d,%*d,%lf",
                       next->in. info.idnt, &latitude[0], &longitude[0],
                       next->out.info.idnt, &latitude[1], &longitude[1], &distance) != 7)
            {
                free(next);
                ret = EINVAL;
                goto end;
            }

            next->in. position = ndt_position_init(latitude[0], longitude[0], ndt_distance_init(0, NDT_ALTUNIT_NA));
            next->out.position = ndt_position_init(latitude[1], longitude[1], ndt_distance_init(0, NDT_ALTUNIT_NA));

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

        int    elevation, vor;
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
         * - frequency (unit: kHz)
         * - navaid has a VOR component
         * - navaid has a DME component
         * - UNKNOWN (either 195 or 110)
         * - latitude
         * - longitude
         * - elevation (unit:  ft)
         * - region/country code
         * - UNKNOWN (always zero)
         */
        if (sscanf(line, "%4[^,],%127[^,],%lf,%d,%d,%*d,%lf,%lf,%d,%2[^,],%*d", wpt->info.idnt,
                   wpt->info.misc, &frequency, &vor, &wpt->dme, &latitude, &longitude, &elevation,
                   wpt->region) != 9)
        {
            ndt_waypoint_close(&wpt);
            ret = EINVAL;
            goto end;
        }

        if (vor)
        {
            wpt->type = NDT_WPTYPE_VOR;
        }
        else if (frequency >= 108. && frequency <= 111.95 && ODD_DEC1(frequency))
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

        snprintf(wpt->info.desc, sizeof(wpt->info.desc),
                 "%3s%4s %-4s -- frequency: %8.3lf, region: %.2s",
                 wpt->type == NDT_WPTYPE_VOR ? "VOR" :
                 wpt->type == NDT_WPTYPE_LOC ? "LOC" :
                 wpt->type == NDT_WPTYPE_NDB ? "NDB" :
                 wpt->type == NDT_WPTYPE_DME ? "DME" : "",
                 wpt->type != NDT_WPTYPE_DME && wpt->dme ? "/DME" : "",
                 wpt->info.idnt, frequency, wpt->region);

        wpt->frequency = ndt_frequency_init(frequency);
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

        /*
         * In addition to official fixes, the database may contain generic
         * coordinates, e.g. "5040N" or "06S10". Since the list is incomplete
         * and the format is easily handled at runtime, they are redundant.
         *
         * They can also give us a false match, e.g. '4600N' for '4600N/05000W'.
         */
        if (sscanf(wpt->info.idnt, "%*1d%*1d%*1d%1d%1[NEWS]", digit, letter) == 2 ||
            sscanf(wpt->info.idnt, "%*1d%*1d%1[NEWS]%*1d%1d", letter, digit) == 2)
        {
            ndt_waypoint_close(&wpt);
            continue;
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

#undef ODD_DEC1
