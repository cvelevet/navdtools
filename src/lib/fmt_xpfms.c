/*
 * fmt_xpfms.c
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

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/common.h"
#include "common/list.h"

#include "compat/compat.h"

#include "airway.h"
#include "flightplan.h"
#include "fmt_xpfms.h"
#include "waypoint.h"

static int print_line    (FILE *fd, const char   *idt, int alt, ndt_position pos, int row);
static int print_waypoint(FILE *fd, ndt_waypoint *wpt, int alt                           );
static int print_airport (FILE *fd, ndt_airport  *apt                                    );

int ndt_fmt_xpfms_flightplan_set_route(ndt_flightplan *flp, ndt_navdatabase *ndb, const char *rte)
{
    ndt_waypoint *src = NULL;
    char         *start, *pos;
    char         *line = NULL, *last_apt = NULL, buf[64];
    int           linecap, header = 0, err = 0;
    int           alt, typ, nwaypoints, discontinuity = 0;
    double        lat, lon;

    if (!flp || !ndb || !rte)
    {
        err = ENOMEM;
        goto end;
    }

    start = pos = strdup(rte);
    if (!start)
    {
        err = ENOMEM;
        goto end;
    }

    while ((err = ndt_file_getline(&line, &linecap, &pos)) > 0)
    {
        if (*line == '\r' || *line == '\n')
        {
            continue; // skip blank lines
        }

        if (header < 1)
        {
            if (*line != 'I' && *line != 'A' && *line != 'L')
            {
                ndt_log("[fmt_xpfms]: invalid header (line 1)\n");
                err = EINVAL;
                goto end;
            }
            header++;
            continue;
        }
        else if (header < 2)
        {
            if (strncmp(line, "3 version", 9))
            {
                ndt_log("[fmt_xpfms]: invalid header (line 2)\n");
                err = EINVAL;
                goto end;
            }
            header++;
            continue;
        }
        else if (header < 3)
        {
            // skip next line (don't know the meaning of the field)
            header++;
            continue;
        }
        else if (header < 4)
        {
            if (sscanf(line, "%d", &nwaypoints) != 1)
            {
                ndt_log("[fmt_xpfms]: invalid header (line 4)\n");
                err = EINVAL;
                goto end;
            }
            if (nwaypoints <= 1)
            {
                nwaypoints = -1; // don't trust it, read the whole file
            }
            header++;
            continue;
        }

        if (sscanf(line, "%d %63s %d %lf %lf", &typ, buf, &alt, &lat, &lon) != 5)
        {
            err = EINVAL;
            goto end;
        }

        if (typ == 0)
        {
            discontinuity = 1;
            continue;
        }

        /* Departure */
        if (!src)
        {
            if (!flp->dep.apt)
            {
                if (typ != 1)
                {
                    ndt_log("[fmt_xpfms]: departure airport not set\n");
                    err = EINVAL;
                    goto end;
                }
                if ((err = ndt_flightplan_set_departure(flp, ndb, buf, NULL)))
                {
                    goto end;
                }
            }
            for (size_t i = 0; (src = ndt_navdata_get_waypoint(ndb, flp->dep.apt->info.idnt, &i));  i++)
            {
                if (src->type == NDT_WPTYPE_APT)
                {
                    break;
                }
            }
            if (!src)
            {
                ndt_log("[fmt_xpfms]: invalid departure airport '%s'\n",
                        flp->dep.apt->info.idnt);
                err = EINVAL;
                goto end;
            }

            /* Don't store the departure airport in the route */
            if (typ == 1 && !strcasecmp(buf, flp->dep.apt->info.idnt))
            {
                continue;
            }
        }

        ndt_route_segment *rsg = NULL;
        ndt_waypoint      *dst = NULL;

        if (typ != 13 && typ != 28)
        {
            for (size_t dstidx = 0; (dst = ndt_navdata_get_waypoint(ndb, buf, &dstidx)); dstidx++)
            {
                switch (dst->type)
                {
                    case NDT_WPTYPE_APT:
                        if (typ !=  1) dst = NULL;
                        break;

                    case NDT_WPTYPE_NDB:
                        if (typ !=  2) dst = NULL;
                        break;

                    case NDT_WPTYPE_VOR:
                        if (typ !=  3) dst = NULL;
                        break;

                    case NDT_WPTYPE_FIX:
                        if (typ != 11) dst = NULL;
                        break;

                    default:
                        dst = NULL;
                        break;
                }
                if (dst)
                {
                    if (ndt_distance_get(ndt_position_calcdistance(dst->position,
                                                                   ndt_position_init(lat,
                                                                                     lon,
                                                                                     ndt_distance_init(0, NDT_ALTUNIT_NA))), NDT_ALTUNIT_NM) > 1)
                    {
                        dst = NULL; // too far
                    }
                    else
                    {
                        break; // we have our waypoint
                    }
                }
            }
        }

        if (!dst) // use coordinates
        {
            if (round(fabs(lat)) > 90. || round(fabs(lon)) > 180.)
            {
                ndt_log("[fmt_xpfms]: invalid coordinates (%lf, %lf)\n",
                        lat, lon);
                err = EINVAL;
                goto end;
            }
            else
            {
                // we're done with the identifier, we can use buf
                snprintf(buf, sizeof(buf), "%+010.6lf/%+011.6lf", lat, lon);
            }
            if (!(dst = ndt_waypoint_llc(buf)))
            {
                err = ENOMEM;
                goto end;
            }
            ndt_list_add(flp->cws, dst);
        }

        if (discontinuity)
        {
            discontinuity = 0;
            rsg = ndt_route_segment_discon(dst);
        }
        else
        {
            rsg = ndt_route_segment_direct(src, dst);
        }

        if (!rsg)
        {
            err = ENOMEM;
            goto end;
        }

        /*
         * Altitude coding; last digit decides:
         *
         *  - 0: regular waypoint (no overfly, if altitude != 0, @constraint)
         *  - 1: altitude constraint is at or above
         *  - 2: altitude constraint is at or above + overfly
         *  - 3: overfly + if altitude != 0, @constraint
         *  - 8: altitude constraint is at or below + overfly | if followed by waypoints with 9 all the way to airport: FAF for RNP approach
         *  - 9: altitude constraint is at or below           | if directly before airport: RNP Approach
         */
        if (alt)
        {
            switch (alt % 10)
            {
                case 0:
                    rsg->constraints.altitude.typ = NDT_RESTRICT_AT;
                    break;

                case 3:
                    rsg->constraints.altitude.typ = NDT_RESTRICT_AT;
                    rsg->constraints.waypoint     = NDT_WPTCONST_FOV;
                    break;

                case 1:
                    rsg->constraints.altitude.typ = NDT_RESTRICT_AB;
                    break;

                case 2:
                    rsg->constraints.altitude.typ = NDT_RESTRICT_AB;
                    rsg->constraints.waypoint     = NDT_WPTCONST_FOV;
                    break;

                case 9:
                    rsg->constraints.altitude.typ = NDT_RESTRICT_BL;
                    break;

                case 8:
                    rsg->constraints.altitude.typ = NDT_RESTRICT_BL;
                    rsg->constraints.waypoint     = NDT_WPTCONST_FOV;
                    break;

                default:
                    rsg->constraints.altitude.typ = NDT_RESTRICT_NO;
                    break;
            }
            rsg->constraints.altitude.alt = ndt_distance_init(alt - alt % 10, NDT_ALTUNIT_FT);
        }

        /* We have a leg, our last endpoint becomes our new startpoint */
        src = rsg->dst;

        /* Let's not forget to add our new segment to the route */
        ndt_list_add(flp->rte, rsg);

        if (nwaypoints != -1)
        {
            nwaypoints--;
        }
        if (nwaypoints == 0)
        {
            break; // don't read any further waypoints
        }
    }
    if (err < 0)
    {
        err = EIO;
        goto end;
    }

    /* Arrival */
    ndt_route_segment *rsg = ndt_list_item(flp->rte, ndt_list_count(flp->rte) - 1);
    if (!rsg)
    {
        err = ENOMEM;
        goto end;
    }
    else
    {
        err = 0;
    }
    if (!flp->arr.apt)
    {
        if (rsg->dst->type != NDT_WPTYPE_APT)
        {
            ndt_log("[fmt_xpfms]: arrival airport not set\n");
            err = EINVAL;
            goto end;
        }
        if ((err = ndt_flightplan_set_arrival(flp, ndb, rsg->dst->info.idnt, NULL)))
        {
            ndt_log("[fmt_xpfms]: invalid arrival airport '%s'\n",
                    flp->arr.apt->info.idnt);
            err = EINVAL;
            goto end;
        }
    }
    if (rsg->dst->type == NDT_WPTYPE_APT &&
        strcasecmp(rsg->dst->info.idnt, flp->arr.apt->info.idnt) == 0)
    {
        /* Don't store the arrival airport in the route */
        ndt_list_rem  (flp->rte, rsg);
        ndt_route_segment_close(&rsg);
    }

end:
    if (err == EINVAL)
    {
        ndt_log("[fmt_xpfms]: failed to parse \"");
        for (int i = 0; line[i] != '\r' && line[i] != '\n'; i++)
        {
            ndt_log("%c", line[i]);
        }
        ndt_log("\"\n");
    }
    free(start);
    free(line);
    return err;
}

int ndt_fmt_xpfms_flightplan_write(ndt_flightplan *flp, FILE *fd)
{
    int ret = 0, count = 1, altitude;

    if (!flp || !fd)
    {
        ret = ENOMEM;
        goto end;
    }

    if (!flp->dep.apt || !flp->arr.apt)
    {
        ndt_log("[fmt_xpfms]: departure or arrival airport not set\n");
        ret = EINVAL;
        goto end;
    }

    // header
    for (size_t i = 0; i < ndt_list_count(flp->legs); i++)
    {
        ndt_route_leg *leg = ndt_list_item(flp->legs, i);
        if (!leg)
        {
            ret = ENOMEM;
            goto end;
        }

        switch (leg->type)
        {
            case NDT_LEGTYPE_DISC:
                count += 2;
                continue;

            default:
                count++;
                break;
        }
    }
    ret = fprintf(fd, "I\n3 version\n1\n%d\n", count);
    if (ret < 0)
    {
        goto end;
    }

    // departure airport
    if ((ret = print_airport(fd, flp->dep.apt)))
    {
        goto end;
    }

    for (size_t i = 0; i < ndt_list_count(flp->legs); i++)
    {
        ndt_route_leg *leg = ndt_list_item(flp->legs, i);
        if (!leg)
        {
            ret = ENOMEM;
            goto end;
        }

        /*
         * Altitude coding; last digit decides:
         *
         *  - 0: regular waypoint (no overfly, if altitude != 0, @constraint)
         *  - 1: altitude constraint is at or above
         *  - 2: altitude constraint is at or above + overfly
         *  - 3: overfly + if altitude != 0, @constraint
         *  - 8: altitude constraint is at or below + overfly | if followed by waypoints with 9 all the way to airport: FAF for RNP approach
         *  - 9: altitude constraint is at or below           | if directly before airport: RNP Approach
         */
        switch (leg->type)
        {
            case NDT_LEGTYPE_DISC:
                ret = print_line(fd, "-------", 0, ndt_position_init(0., 0., ndt_distance_init(0, NDT_ALTUNIT_NA)), 0);
                if (ret)
                {
                    goto end;
                } // don't lose leg->dst, just pass through
            case NDT_LEGTYPE_DCTO:
                if (leg->constraints.waypoint == NDT_WPTCONST_FOV)
                {
                    switch (leg->constraints.altitude.typ)
                    {
                        case NDT_RESTRICT_AT:
                            altitude = round(ndt_distance_get(leg->constraints.altitude.alt, NDT_ALTUNIT_FT) / 10.) * 10 + 3;
                            break;
                        case NDT_RESTRICT_AB:
                            altitude = round(ndt_distance_get(leg->constraints.altitude.alt, NDT_ALTUNIT_FT) / 10.) * 10 + 2;
                            break;
                        case NDT_RESTRICT_BL:
                            altitude = round(ndt_distance_get(leg->constraints.altitude.alt, NDT_ALTUNIT_FT) / 10.) * 10 - 2;
                            break;
                        default:
                            altitude = 3;
                            break;
                    }
                }
                else
                {
                    switch (leg->constraints.altitude.typ)
                    {
                        case NDT_RESTRICT_AT:
                            altitude = round(ndt_distance_get(leg->constraints.altitude.alt, NDT_ALTUNIT_FT) / 10.) * 10 + 0;
                            break;
                        case NDT_RESTRICT_AB:
                            altitude = round(ndt_distance_get(leg->constraints.altitude.alt, NDT_ALTUNIT_FT) / 10.) * 10 + 1;
                            break;
                        case NDT_RESTRICT_BL:
                            altitude = round(ndt_distance_get(leg->constraints.altitude.alt, NDT_ALTUNIT_FT) / 10.) * 10 - 1;
                            break;
                        default:
                            altitude = 0;
                            break;
                    }
                }
                ret = print_waypoint(fd, leg->dst, altitude);
                break;

            default:
                break; // skip unknown leg types for now
        }

        if (ret)
        {
            goto end;
        }
    }

    // arrival airport
    if ((ret = print_airport(fd, flp->arr.apt)))
    {
        goto end;
    }

    // end of file
    ret = print_line(fd, "-------", 0, ndt_position_init(0., 0., ndt_distance_init(0, NDT_ALTUNIT_NA)), 0);

end:
    return ret;
}

static int print_line(FILE *fd, const char *idt, int alt, ndt_position pos, int row)
{
    if (fd && idt)
    {
        int ret = fprintf(fd, "%-2d  %-7s  %05d  %+010.6lf  %+011.6lf\n",
                          row, idt, alt,
                          ndt_position_getlatitude (pos, NDT_ANGUNIT_DEG),
                          ndt_position_getlongitude(pos, NDT_ANGUNIT_DEG));

        return ret > 0 ? 0 : ret ? ret : -1;
    }

    return -1;
}

static int print_waypoint(FILE *fd, ndt_waypoint *wpt, int alt)
{
    if (fd && wpt)
    {
        switch (wpt->type)
        {
            case NDT_WPTYPE_APT:
                return print_line(fd, wpt->info.idnt, alt, wpt->position,  1);

            case NDT_WPTYPE_NDB:
                return print_line(fd, wpt->info.idnt, alt, wpt->position,  2);

            case NDT_WPTYPE_VOR:
                return print_line(fd, wpt->info.idnt, alt, wpt->position,  3);

            case NDT_WPTYPE_FIX:
                return print_line(fd, wpt->info.idnt, alt, wpt->position, 11);

            default: // latitude/longitude or other unsupported type
            {
                // X-Plane 10.30 doesn't care much about the name, keep it short
                char   lat_lon_string[17];
                double lat = ndt_position_getlatitude (wpt->position, NDT_ANGUNIT_DEG);
                double lon = ndt_position_getlongitude(wpt->position, NDT_ANGUNIT_DEG);

                snprintf(lat_lon_string, sizeof(lat_lon_string), "%c%02d%c%03d",
                         lat >= 0. ? 'N' : 'S', (int)round(fabs(lat)),
                         lon >= 0. ? 'E' : 'W', (int)round(fabs(lon)));

                return print_line(fd, lat_lon_string, alt, wpt->position, 28);
            }
        }
    }

    return -1;
}

static int print_airport(FILE *fd, ndt_airport *apt)
{
    if (fd && apt)
    {
        ndt_distance distance = ndt_position_getaltitude(apt->coordinates);
        int          altitude = ndt_distance_get(distance, NDT_ALTUNIT_FT);
        return print_line(fd, apt->info.idnt, altitude, apt->coordinates, 1);
    }

    return -1;
}
