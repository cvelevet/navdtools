/*
 * fmt_xpfms.c
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

int ndt_fmt_xpfms_flightplan_set_route(ndt_flightplan *flp, const char *rte)
{
    ndt_waypoint *src = NULL;
    char         *start, *pos;
    char         *line = NULL, *last_apt = NULL, buf[64];
    int           linecap, header = 0, err = 0, init = 0;
    int           aft, typ, n_waypnts, discontinuity = 0;
    double        alt, lat, lon, spd; int after_fafx = 0;
    ndt_position  llc;
    ndt_airspeed  kts;

    if (!flp || !rte)
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
            if (sscanf(line, "%d", &n_waypnts) != 1)
            {
                ndt_log("[fmt_xpfms]: invalid header (line 4)\n");
                err = EINVAL;
                goto end;
            }
            if (n_waypnts <= 1)
            {
                n_waypnts = -1; // don't trust it, read the whole file
            }
            else
            {
                n_waypnts++;   // format doesn't account for waypoint 0
            }
            header++;
            continue;
        }

        if (sscanf(line, "%d %63s %lf %lf %lf %lf", &typ, buf, &alt, &lat, &lon, &spd) != 6)
        {
            if (sscanf(line, "%d %63s %lf %lf %lf", &typ, buf, &alt, &lat, &lon)       != 5)
            {
                err = EINVAL;
                goto end;
            }
            // no speed constraint on this line, reset
            spd = 0.;
        }

        llc = ndt_position_init(lat, lon, ndt_distance_init(0, NDT_ALTUNIT_NA));
        kts = ndt_airspeed_init(spd, NDT_SPDUNIT_KTS);
        aft = round(10. * round(alt / 10.));

        if (typ == 0)
        {
            discontinuity = 1;
            continue;
        }

        /* Departure */
        if (!init)
        {
            if (!flp->dep.apt)
            {
                if (typ != 1)
                {
                    ndt_log("[fmt_xpfms]: departure airport not set\n");
                    err = EINVAL;
                    goto end;
                }
                if ((err = ndt_flightplan_set_departure(flp, buf, NULL)))
                {
                    goto end;
                }
            }

            /* Set the initial source waypoint (SID, runway or airport). */
            src = (flp->dep.sid.enroute.rsgt ? flp->dep.sid.enroute.rsgt->dst :
                   flp->dep.sid.        rsgt ? flp->dep.sid.        rsgt->dst :
                   flp->dep.rwy              ? flp->dep.rwy->waypoint         : flp->dep.apt->waypoint);

            init = 1;//done
        }

        ndt_route_segment *rsg = NULL;
        ndt_waypoint      *dst = NULL;
        ndt_restriction   constraints;

        if (typ != 13 && typ != 28)
        {
            for (size_t dstidx = 0; (dst = ndt_navdata_get_waypoint(flp->ndb, buf, &dstidx)); dstidx++)
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
                    ndt_position a = dst->position;
                    ndt_position b = llc;

                    if (ndt_distance_get(ndt_position_calcdistance(a, b), NDT_ALTUNIT_NM) <= 1)
                    {
                        break; // we have our waypoint
                    }
                    dst = NULL; // too far
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
            ndt_route_segment *dsc = ndt_route_segment_discon();
            if (!dsc)
            {
                err = ENOMEM;
                goto end;
            }
            discontinuity = 0;
            ndt_list_add(flp->rte, dsc);
            rsg = ndt_route_segment_direct(NULL, dst);
        }
        else
        {
            rsg = ndt_route_segment_direct( src, dst);
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
        if ((int)alt)
        {
            switch ((int)alt % 10)
            {
                case 0:
                    constraints.altitude.typ = NDT_RESTRICT_AT;
                    break;

                case 3:
                    constraints.altitude.typ = NDT_RESTRICT_AT;
                    constraints.waypoint     = NDT_WPTCONST_FOV;
                    break;

                case 1:
                    constraints.altitude.typ = NDT_RESTRICT_AB;
                    break;

                case 2:
                    constraints.altitude.typ = NDT_RESTRICT_AB;
                    constraints.waypoint     = NDT_WPTCONST_FOV;
                    break;

                case 9:
                    if (after_fafx)
                    {
                        constraints.altitude.typ = NDT_RESTRICT_AT;
                        constraints.waypoint     = NDT_WPTCONST_FOV;
                        break;
                    }
                    constraints.altitude.typ = NDT_RESTRICT_BL;
                    break;

                case 8:
                    constraints.altitude.typ = NDT_RESTRICT_AT;
                    constraints.waypoint     = NDT_WPTCONST_FAF;
                    after_fafx               = 1;
                    break;

                default:
                    constraints.altitude.typ = NDT_RESTRICT_NO;
                    break;
            }
            if (((int)alt % 10) == 3 && ((int)alt - (int)alt % 10) == 0)
            {
                // overfly but no altitude constraint
                constraints.altitude.typ = NDT_RESTRICT_NO;
            }
            if (constraints.altitude.typ == NDT_RESTRICT_AT ||
                constraints.altitude.typ == NDT_RESTRICT_AB)
            {
                constraints.altitude.min = ndt_distance_init(aft, NDT_ALTUNIT_FT);
            }
            if (constraints.altitude.typ == NDT_RESTRICT_AT ||
                constraints.altitude.typ == NDT_RESTRICT_BL)
            {
                constraints.altitude.max = ndt_distance_init(aft, NDT_ALTUNIT_FT);
            }
        }
        else
        {
            constraints.altitude.typ = NDT_RESTRICT_NO;
            constraints.waypoint     = NDT_WPTCONST_NO;
        }

        /*
         * Speed restriction; if non-zero, fly at or below speed.
         */
        if (ndt_airspeed_get(kts, NDT_SPDUNIT_KTS, NDT_MACH_DEFAULT) > 0LL)
        {
            constraints.airspeed.typ = NDT_RESTRICT_BL;
            constraints.airspeed.acf = NDT_ACFTYPE_ALL;
            constraints.airspeed.max = kts;
        }
        else
        {
            constraints.airspeed.typ = NDT_RESTRICT_NO;
        }

        /*
         * Apply restrictions to the route segment's only leg.
         */
        ndt_route_leg_restrict(ndt_list_item(rsg->legs, 0), constraints);

        /* We have a leg, our last endpoint becomes our new startpoint */
        src = rsg->dst;

        /* Let's not forget to add our new segment to the route */
        ndt_list_add(flp->rte, rsg);

        if (n_waypnts != -1)
        {
            n_waypnts--;
        }
        if (n_waypnts == 0)
        {
            break; // don't read any further waypoints
        }
    }
    if (err < 0)
    {
        err = EIO;
        goto end;
    }
    else
    {
        err = 0; // we may break out with err > 0 (e.g. when n_waypnts reaches 0)
    }

    /* Arrival */
    if (!flp->arr.apt)
    {
        ndt_route_segment *rsg = ndt_list_item(flp->rte, -1);
        if (!rsg)
        {
            err = ENOMEM;
            goto end;
        }
        if (rsg->dst->type != NDT_WPTYPE_APT)
        {
            ndt_log("[fmt_xpfms]: arrival airport not set\n");
            err = EINVAL;
            goto end;
        }
        if ((err = ndt_flightplan_set_arrival(flp, rsg->dst->info.idnt, NULL)))
        {
            ndt_log("[fmt_xpfms]: invalid arrival airport '%s'\n",
                    rsg->dst->info.idnt);
            err = EINVAL;
            goto end;
        }
        ndt_list_rem  (flp->rte, rsg);
        ndt_route_segment_close(&rsg);
    }

    /*
     * Remove unwanted segments.
     */
    ndt_route_segment *fst = ndt_list_item(flp->rte,  0);
    ndt_route_segment *ult = ndt_list_item(flp->rte, -1);
    if (fst && fst->dst == flp->dep.apt->waypoint)
    {
        // check for duplicates
        if (fst == ult)
        {
            ult = NULL;
        }

        // don't include the departure airport as the first leg
        ndt_list_rem  (flp->rte, fst);
        ndt_route_segment_close(&fst);
    }
    if (ult && ult->dst == flp->arr.apt->waypoint)
    {
        // don't include the arrival airport as the last leg
        ndt_list_rem  (flp->rte, ult);
        ndt_route_segment_close(&ult);
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

static int update__row(FILE *fd, int row)
{
    if (row <= 0 || row >= 9)
    {
        // new page
        if (ndt_fprintf(fd, "%s", "\n"))
        {
            return -1;
        }
        return 1;
    }
    return row + 1;
}

static int helpr_waypoint_write(FILE *fd, ndt_waypoint *wpt, int row, ndt_fltplanformat fmt, ndt_restriction *constraints)
{
    char buf[25];
    int  ret;

    if (fmt == NDT_FLTPFMT_XPHLP)
    {
        if (wpt == NULL)
        {
            return ndt_fprintf(fd, "-------  %-19s  -------------------------\n", "F-PLN DISCONTINUITY");
        }
        ret = ndt_fprintf(fd, "%2d  %s  %-19s  %2d  %+07.3lf  %+08.3lf  %2d", row,
                          wpt->type == NDT_WPTYPE_APT ? "APT" :
                          wpt->type == NDT_WPTYPE_FIX ? "fix" :
                          wpt->type == NDT_WPTYPE_NDB ? "NDB" :
                          wpt->type == NDT_WPTYPE_VOR ? "VOR" : "l/l",
                          wpt->info.idnt, row,
                          ndt_position_getlatitude (wpt->position, NDT_ANGUNIT_DEG),
                          ndt_position_getlongitude(wpt->position, NDT_ANGUNIT_DEG), row);
    }
    else if (fmt == NDT_FLTPFMT_XPCDU)
    {
        if (wpt == NULL)
        {
            return ndt_fprintf(fd, "--  %-19s  ------------------------\n", "F-PLN DISCONTINUITY");
        }
        if (ndt_position_sprintllc(wpt->position, NDT_LLCFMT_AIBUS,
                                   buf, sizeof(buf)) < 0)
        {
            return EIO;
        }
        ret = ndt_fprintf(fd, "%2d  %-19s  %2d  %s  %2d", row, wpt->info.idnt, row, buf, row);
    }
    else if (fmt == NDT_FLTPFMT_XPCVA)
    {
        if (ndt_position_sprintllc(wpt->position, NDT_LLCFMT_CEEVA,
                                   buf, sizeof(buf)) < 0)
        {
            return EIO;
        }
        ret = ndt_fprintf(fd, "%d  %-19s  %d  %s  %d", row, wpt->info.idnt, row, buf, row);
    }
    if (ret)
    {
        return ret;
    }

    if (constraints)
    {
        int altmin = ndt_distance_get(constraints->altitude.min, NDT_ALTUNIT_FT);
        int altmax = ndt_distance_get(constraints->altitude.max, NDT_ALTUNIT_FT);
        int spdmin = ndt_airspeed_get(constraints->airspeed.min, NDT_SPDUNIT_KTS, NDT_MACH_DEFAULT);
        int spdmax = ndt_airspeed_get(constraints->airspeed.max, NDT_SPDUNIT_KTS, NDT_MACH_DEFAULT);

        if (fmt == NDT_FLTPFMT_XPCDU)
        {
            switch (constraints->waypoint)
            {
                case NDT_WPTCONST_FAF:
                    ret = ndt_fprintf(fd, "  %s", "f");
                    break;
                case NDT_WPTCONST_FOV:
                    ret = ndt_fprintf(fd, "  %s", "o");
                    break;
                case NDT_WPTCONST_IAF:
                    ret = ndt_fprintf(fd, "  %s", "i");
                    break;
                case NDT_WPTCONST_MAP:
                    ret = ndt_fprintf(fd, "  %s", "m");
                    break;
                default:
                    ret = ndt_fprintf(fd, "  %s", " ");
                    break;
            }
            if (ret)
            {
                return ret;
            }
        }

        switch (constraints->altitude.typ)
        {
            case NDT_RESTRICT_AB:
                ret = ndt_fprintf(fd, "  ALT above %5d", altmin);
                break;
            case NDT_RESTRICT_AT:
                ret = ndt_fprintf(fd, "  ALT    at %5d", altmax);
                break;
            case NDT_RESTRICT_BL:
                ret = ndt_fprintf(fd, "  ALT below %5d", altmax);
                break;
            case NDT_RESTRICT_BT:
                ret = ndt_fprintf(fd, "  ALT %5d %5d",   altmin, altmax);
                break;
            default:
                ret = ndt_fprintf(fd, "%17s", " ");
                break;
        }
        if (ret)
        {
            return ret;
        }

        if (constraints->airspeed.acf == NDT_ACFTYPE_ALL ||
            constraints->airspeed.acf == NDT_ACFTYPE_JET)
        {
            switch (constraints->airspeed.typ)
            {
                case NDT_RESTRICT_AB:
                    ret = ndt_fprintf(fd, "  SPD min %3d", spdmin);
                    break;
                case NDT_RESTRICT_AT:
                    ret = ndt_fprintf(fd, "  SPD  at %3d", spdmax);
                    break;
                case NDT_RESTRICT_BL:
                    ret = ndt_fprintf(fd, "  SPD max %3d", spdmax);
                    break;
                case NDT_RESTRICT_BT:
                    ret = ndt_fprintf(fd, "  SPD %3d %3d", spdmin, spdmax);
                    break;
                default:
                    break;
            }
            if (ret)
            {
                return ret;
            }
        }
    }

    return ndt_fprintf(fd, "%s", "\n");
}

static int ceeva_flightplan_write(ndt_flightplan *flp, FILE *fd, ndt_fltplanformat fmt)
{
    int ret = 0, row = 0;

    // departure airport and runway
    ret = helpr_waypoint_write(fd, flp->dep.apt->waypoint, 0, fmt, NULL);
    if (ret)
    {
        goto fail;
    }
    if (flp->dep.rwy)
    {
        ret = helpr_waypoint_write(fd, flp->dep.rwy->waypoint, 0, fmt, NULL);
        if (ret)
        {
            goto fail;
        }
    }

    // all flightplan legs
    for (size_t i = 0; i < ndt_list_count(flp->legs); i++)
    {
        ndt_route_leg *leg = ndt_list_item(flp->legs, i);
        if (!leg)
        {
            ret = ENOMEM;
            goto fail;
        }
        switch (leg->type)
        {
            case NDT_LEGTYPE_HF: // skipped
            case NDT_LEGTYPE_HA: // skipped
            case NDT_LEGTYPE_HM: // skipped
            case NDT_LEGTYPE_ZZ: // skipped
                break;

            case NDT_LEGTYPE_DF:
            {
                if (leg->src != leg->dst || ndt_list_count(leg->xpfms))
                {
                    for (size_t j = 0; j < ndt_list_count(leg->xpfms); j++)
                    {
                        row = update__row(fd, row);
                        ret = helpr_waypoint_write(fd, ndt_list_item(leg->xpfms, j), row, fmt, NULL);
                        if (ret)
                        {
                            goto fail;
                        }
                    }
                    row = update__row(fd, row);
                    ret = helpr_waypoint_write(fd, leg->dst, row, fmt, &leg->constraints);
                    if (ret)
                    {
                        goto fail;
                    }
                }
                break;
            }

            default:
            {
                for (size_t j = 0; j < ndt_list_count(leg->xpfms); j++)
                {
                    row = update__row(fd, row);
                    ret = helpr_waypoint_write(fd, ndt_list_item(leg->xpfms, j), row, fmt, leg->dst ? NULL : &leg->constraints);
                    if (ret)
                    {
                        goto fail;
                    }
                }
                if (leg->dst)
                {
                    row = update__row(fd, row);
                    ret = helpr_waypoint_write(fd, leg->dst, row, fmt, &leg->constraints);
                    if (ret)
                    {
                        goto fail;
                    }
                }
                break;
            }
        }
    }

    // arrival runway and airport
    if (flp->arr.rwy)
    {
        row = update__row(fd, row);
        ret = helpr_waypoint_write(fd, flp->arr.rwy->waypoint, row, fmt, NULL);
        if (ret)
        {
            goto fail;
        }
    }
    row = update__row(fd, row);
    ret = helpr_waypoint_write(fd, flp->arr.apt->waypoint, row, fmt, NULL);
    if (ret)
    {
        goto fail;
    }

    if ((ret = ndt_fprintf(fd, "%s", "\n")))
    {
        goto fail;
    }
    return ndt_flightplan_write(flp, fd, NDT_FLTPFMT_IRECP);

fail:
    return ret;
}

static int helpr_rtesegment_write(FILE *fd, ndt_route_segment *rsg, int *row, ndt_fltplanformat fmt)
{
    int ret = 0;
    if (!fd || !rsg || !row)
    {
        ret = ENOMEM;
        goto end;
    }
    for (size_t i = 0; i < ndt_list_count(rsg->legs); i++)
    {
        ndt_route_leg *leg = ndt_list_item(rsg->legs, i);
        if (!leg)
        {
            ret = ENOMEM;
            goto end;
        }
        switch (leg->type)
        {
            case NDT_LEGTYPE_HF: // skipped
            case NDT_LEGTYPE_HA: // skipped
            case NDT_LEGTYPE_HM: // skipped
                break;

            case NDT_LEGTYPE_ZZ:
                if ((ret = helpr_waypoint_write(fd, NULL, 0, fmt, NULL)))
                {
                    goto end;
                }
                break;

            case NDT_LEGTYPE_DF:
            {
                if (leg->src == leg->dst && !ndt_list_count(leg->xpfms))
                {
                    if ((ret = helpr_waypoint_write(fd, NULL, 0, fmt, NULL)))
                    {
                        goto end;
                    }
                }
                for (size_t j = 0; j < ndt_list_count(leg->xpfms); j++)
                {
                    if ((ret = helpr_waypoint_write(fd, ndt_list_item(leg->xpfms, j), *row, fmt, NULL)))
                    {
                        goto end;
                    }
                    *row += 1;
                }
                if ((ret = helpr_waypoint_write(fd, leg->dst, *row, fmt, &leg->constraints)))
                {
                    goto end;
                }
                *row += 1;
                break;
            }

            default:
            {
                for (size_t j = 0; j < ndt_list_count(leg->xpfms); j++)
                {
                    if ((ret = helpr_waypoint_write(fd, ndt_list_item(leg->xpfms, j), *row, fmt, leg->dst ? NULL : &leg->constraints)))
                    {
                        goto end;
                    }
                    *row += 1;
                }
                if (leg->dst)
                {
                    if ((ret = helpr_waypoint_write(fd, leg->dst, *row, fmt, &leg->constraints)))
                    {
                        goto end;
                    }
                    *row += 1;
                }
                break;
            }
        }
    }

end:
    return ret;
}

static int helpr_flightplan_write(ndt_flightplan *flp, FILE *fd, ndt_fltplanformat fmt)
{
    int ret = 0, row = 1;

    // departure airport and runway
    if ((ret = ndt_fprintf(fd, "%s:\n", "Departure")))
    {
        goto fail;
    }
    if ((ret = helpr_waypoint_write(fd, flp->dep.apt->waypoint, 0, fmt, NULL)))
    {
        goto fail;
    }
    if (flp->dep.rwy)
    {
        if ((ret = helpr_waypoint_write(fd, flp->dep.rwy->waypoint, 0, fmt, NULL)))
        {
            goto fail;
        }
    }

    // SID, enroute transition
    if (flp->dep.sid.rsgt)
    {
        if ((ret = ndt_fprintf(fd, "\n%s:\n", flp->dep.sid.rsgt->info.idnt)))
        {
            goto fail;
        }
        if ((ret = helpr_rtesegment_write(fd, flp->dep.sid.rsgt, &row, fmt)))
        {
            goto fail;
        }
    }
    if (flp->dep.sid.enroute.rsgt)
    {
        if ((ret = ndt_fprintf(fd, "\n%s:\n", flp->dep.sid.enroute.rsgt->info.idnt)))
        {
            goto fail;
        }
        if ((ret = helpr_rtesegment_write(fd, flp->dep.sid.enroute.rsgt, &row, fmt)))
        {
            goto fail;
        }
    }

    // decoded route
    if (ndt_list_count(flp->rte))
    {
        if ((ret = ndt_fprintf(fd, "\n%s:\n", "Enroute")))
        {
            goto fail;
        }
        for (size_t i = 0; i < ndt_list_count(flp->rte); i++)
        {
            ndt_route_segment *rsg = ndt_list_item(flp->rte, i);
            if (!rsg)
            {
                ret = ENOMEM;
                goto fail;
            }
            if ((ret = helpr_rtesegment_write(fd, rsg, &row, fmt)))
            {
                goto fail;
            }
        }
    }

    // enroute transition, STAR
    if (flp->arr.star.enroute.rsgt)
    {
        if ((ret = ndt_fprintf(fd, "\n%s:\n", flp->arr.star.enroute.rsgt->info.idnt)))
        {
            goto fail;
        }
        if ((ret = helpr_rtesegment_write(fd, flp->arr.star.enroute.rsgt, &row, fmt)))
        {
            goto fail;
        }
    }
    if (flp->arr.star.rsgt)
    {
        if ((ret = ndt_fprintf(fd, "\n%s:\n", flp->arr.star.rsgt->info.idnt)))
        {
            goto fail;
        }
        if ((ret = helpr_rtesegment_write(fd, flp->arr.star.rsgt, &row, fmt)))
        {
            goto fail;
        }
    }

    // approach transition, final
    if (flp->arr.apch.transition.rsgt)
    {
        if ((ret = ndt_fprintf(fd, "\n%s:\n", flp->arr.apch.transition.rsgt->info.idnt)))
        {
            goto fail;
        }
        if ((ret = helpr_rtesegment_write(fd, flp->arr.apch.transition.rsgt, &row, fmt)))
        {
            goto fail;
        }
    }
    if (flp->arr.apch.rsgt)
    {
        if ((ret = ndt_fprintf(fd, "\n%s:\n", flp->arr.apch.rsgt->info.idnt)))
        {
            goto fail;
        }
        if ((ret = helpr_rtesegment_write(fd, flp->arr.apch.rsgt, &row, fmt)))
        {
            goto fail;
        }
    }

    // arrival runway and airport
    if ((ret = ndt_fprintf(fd, "\n%s:\n", "Arrival")))
    {
        goto fail;
    }
    if (flp->arr.rwy)
    {
        if ((ret = helpr_waypoint_write(fd, flp->arr.rwy->waypoint, row++, fmt, NULL)))
        {
            goto fail;
        }
    }
    if ((ret = helpr_waypoint_write(fd, flp->arr.apt->waypoint, row++, fmt, NULL)))
    {
        goto fail;
    }

    if ((ret = ndt_fprintf(fd, "%s", "\n")))
    {
        goto fail;
    }
    return ndt_flightplan_write(flp, fd, NDT_FLTPFMT_IRECP);

fail:
    return ret;
}

static int print_line(FILE *fd, const char *idt, int alt, int spd, ndt_position pos, int row)
{
    if (fd && idt)
    {
        if (row == 0 || row == 1)
        {
            // don't append speed for discontinuities and airports
            return ndt_fprintf(fd, "%-2d  %-7s  %05d  %+010.6lf  %+011.6lf\n",
                               row, idt, alt,
                               ndt_position_getlatitude (pos, NDT_ANGUNIT_DEG),
                               ndt_position_getlongitude(pos, NDT_ANGUNIT_DEG));
        }
        return ndt_fprintf(fd, "%-2d  %-7s  %05d  %+010.6lf  %+011.6lf  %010.6lf\n",
                           row, idt, alt,
                           ndt_position_getlatitude (pos, NDT_ANGUNIT_DEG),
                           ndt_position_getlongitude(pos, NDT_ANGUNIT_DEG), (double)spd);
    }
    return -1;
}

static int print_waypoint(FILE *fd, ndt_waypoint *wpt, int alt, int spd)
{
    if (fd && wpt)
    {
        // X-Plane 10.36 doesn't check the ID for lat/lon waypoints, keep it short
        char llc[8];
        ndt_position_sprintllc(wpt->position, NDT_LLCFMT_DEFLT, llc, sizeof(llc));

        switch (wpt->type)
        {
            case NDT_WPTYPE_APT:
                return print_line(fd, wpt->info.idnt, alt, spd, wpt->position,  1);

            case NDT_WPTYPE_NDB:
                return print_line(fd, wpt->info.idnt, alt, spd, wpt->position,  2);

            case NDT_WPTYPE_VOR:
                return print_line(fd, wpt->info.idnt, alt, spd, wpt->position,  3);

            case NDT_WPTYPE_FIX:
                return print_line(fd, wpt->info.idnt, alt, spd, wpt->position, 11);

            default: // latitude/longitude or other unsupported type
                return print_line(fd, llc,            alt, spd, wpt->position, 28);
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
        return print_line(fd, apt->info.idnt, altitude, 0, apt->coordinates, 1);
    }

    return -1;
}

static int xpfms_write_header(FILE *fd, int count)
{
    return ndt_fprintf(fd, "I\n3 version\n1\n%d\n", count - 1);
}

static int xpfms_write_footer(FILE *fd)
{
    return print_line(fd, "-------", 0, 0, NDT_POSITION_NULL, 0);
}

static int xpfms_count_legs(ndt_list *legs)
{
    int count = 0;
    ndt_route_leg *leg;
    for (size_t i = 0; i < ndt_list_count(legs); i++)
    {
        if ((leg = ndt_list_item(legs, i)))
        {
            switch (leg->type)
            {
                case NDT_LEGTYPE_HF: // skipped
                case NDT_LEGTYPE_HA: // skipped
                case NDT_LEGTYPE_HM: // skipped
                case NDT_LEGTYPE_ZZ: // skipped
                    break;
                case NDT_LEGTYPE_FM: // discont
                case NDT_LEGTYPE_VM: // discont
                    count += 1;
                    break;
                case NDT_LEGTYPE_DF:
                    if (leg->src == leg->dst && !ndt_list_count(leg->xpfms))
                    {
                        count += 1;  // discont
                    }
                default:
                    count += !!leg->dst + ndt_list_count(leg->xpfms);
                    break;
            }
        }
    }
    return count;
}

static int xpfms_skip4dist(ndt_waypoint *src, ndt_waypoint *leg_wpt, ndt_waypoint *leg_dst, ndt_route_leg *leg_nxt)
{
    /*
     * Calculate distance from src to the next waypoint:
     *     - leg_wpt: src's current leg, dummy waypoint (may be NULL)
     *     - leg_dst: src's current leg, final waypoint (may be NULL)
     *     - leg_nxt: the route's next leg, may also be NULL
     *
     * src should be skipped if the distance is below a preset threshold.
     */
    ndt_waypoint *dst = leg_wpt ? leg_wpt : leg_dst;
    if (dst == NULL && leg_nxt)
    {
        dst = ndt_list_item(leg_nxt->xpfms, 0);
        dst = dst ? dst : leg_nxt->dst;
    }
    if (dst && src)
    {
        // ensure at least 1 nautical mile between 2 consecutive waypoints
        // avoids weird flight path drawing on the QPAC ND (KEWR: EWR2.22L)
        return (ndt_distance_get(ndt_position_calcdistance(src->position,
                                                           dst->position),
                                 NDT_ALTUNIT_ME) < INT64_C(1852));
    }
    return 0;
}

static int xpfms_write_legs(FILE *fd, ndt_list *legs, ndt_runway *arr_rwy)
{
    ndt_waypoint *fapchfix = NULL, *lst = NULL;
    ndt_distance rwthralt, fapchalt;
    ndt_position rwthrpos;
    ndt_route_leg *leg;
    int ret = 0;

    for (size_t i = 0; i < ndt_list_count(legs); i++)
    {
        if (!(leg = ndt_list_item(legs, i)))
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
         *  - 9: altitude constraint is at or below           | if after "FAF" waypoint: NPA waypoint for RNP Approach
         *
         * RNAV approach examples:
         * - navdconv-function nzch n nzqn 05 rnav05-f ibabu dct xplane
         * - navdconv-function loww n lowi 26 rnav26   wi001 dct xplane
         */
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlogical-not-parentheses"
#endif
        int speed, altitude = 0;
        int overfly = leg->constraints.waypoint == NDT_WPTCONST_FOV;
        switch (leg->constraints.altitude.typ)
        {
            case NDT_RESTRICT_BL:
                altitude = round(ndt_distance_get(leg->constraints.altitude.max, NDT_ALTUNIT_FT) / 10.) * 10;
                altitude = altitude + 10 * (altitude == 0);
                altitude = altitude - 01 - (!overfly == 0);
                break;
            case NDT_RESTRICT_AB:
            case NDT_RESTRICT_BT: // the minimum altitude is the one we care about (for ground clearance)
                altitude = round(ndt_distance_get(leg->constraints.altitude.min, NDT_ALTUNIT_FT) / 10.) * 10;
                altitude = altitude + 10 * (altitude == 0);
                altitude = altitude + 01 + (!overfly == 0);
                break;
            case NDT_RESTRICT_AT:
                altitude = round(ndt_distance_get(leg->constraints.altitude.max, NDT_ALTUNIT_FT) / 10.) * 10;
                altitude = altitude + 10 * (altitude == 0);
            default:
                altitude = altitude + 00 + (!overfly == 0) * 3;
                break;
        }
#ifdef __clang__
#pragma clang diagnostic pop
#endif
        if (fapchfix)
        {
            if (!leg->dst)
            {
                ndt_log("[fmt_xpfms]: post-FAF leg '%s' type %d without fix!\n",
                        leg->info.idnt, leg->type);
                ret = EINVAL;
                goto end;
            }
            if (leg->constraints.altitude.typ != NDT_RESTRICT_AT)
            {
                // bogus altitude restriction, we need to make a valid one so
                // this approach can be flown as RNP by the QPAC's FBW plugin
                double d1tratio; int64_t alt_diff; ndt_distance d1, d2, dt, da;
                d1 = ndt_position_calcdistance      (fapchfix->position, leg->dst->position);
                d2 = ndt_position_calcdistance      (leg->dst->position,           rwthrpos);
                da = ndt_distance_rem               (fapchalt,                     rwthralt);
                dt = ndt_distance_add               (d1,                                 d2);
                d1tratio = ((double)ndt_distance_get(d1, NDT_ALTUNIT_FT) /
                            (double)ndt_distance_get(dt, NDT_ALTUNIT_FT));
                alt_diff = ((double)ndt_distance_get(da, NDT_ALTUNIT_FT) * d1tratio);
                altitude = ndt_distance_get   (fapchalt, NDT_ALTUNIT_FT) - alt_diff - 1;
            }
            altitude = round(altitude / 10.) * 10;
            altitude = altitude + 10 * (altitude == 0) - 1;
            // use this waypoint as the new reference for next waypoint
            fapchfix = leg->dst; fapchalt = ndt_distance_init(round(altitude / 10.) * 10, NDT_ALTUNIT_FT);
        }
        else if (arr_rwy && (leg->constraints.waypoint == NDT_WPTCONST_FAF) &&
                 leg->rsg      && (leg->rsg->     type == NDT_RSTYPE_PRC)   &&
                 leg->rsg->prc && (leg->rsg->prc->type == NDT_PROCTYPE_FINAL ||
                                   leg->rsg->prc->type == NDT_PROCTYPE_APPTR))
        {
            // only set FAF for RNAV approaches, else the
            // QPAC plugin will disable ILS functionality
            if (leg->rsg->prc->approach.type == NDT_APPRTYPE_GLS ||
                leg->rsg->prc->approach.type == NDT_APPRTYPE_RNAV)
            {
                if (!arr_rwy->waypoint)
                {
                    ndt_log("[fmt_xpfms]: FAF leg, runway '%s' has no waypoint!\n",
                            arr_rwy->info.idnt);
                    ret = EINVAL;
                    goto end;
                }
                if (!leg->dst)
                {
                    ndt_log("[fmt_xpfms]: FAF leg '%s' type %d without fix!\n",
                            leg->info.idnt, leg->type);
                    ret = EINVAL;
                    goto end;
                }
                if (leg->constraints.altitude.typ == NDT_RESTRICT_NO)
                {
                    ndt_log("[fmt_xpfms]: FAF '%s' without altitude constraint!\n",
                            leg->dst->info.idnt);
                    ret = EINVAL;
                    goto end;
                }
                altitude = round(altitude / 10.) * 10;
                altitude = altitude + 10 * (altitude == 0) - 2;
                rwthralt = arr_rwy->threshold.altitude; rwthrpos = arr_rwy->waypoint->position;
                fapchfix = leg->dst; fapchalt = ndt_distance_init(round(altitude / 10.) * 10, NDT_ALTUNIT_FT);
            }
        }
        switch (leg->constraints.airspeed.typ)
        {
            case NDT_RESTRICT_AT:
            case NDT_RESTRICT_BL:
            case NDT_RESTRICT_BT:
                speed = ndt_airspeed_get(leg->constraints.airspeed.max, NDT_SPDUNIT_KTS, NDT_MACH_DEFAULT);
                break;
            case NDT_RESTRICT_AB:
            default:
                speed = 0;
                break;
        }
        switch (leg->constraints.airspeed.acf)
        {
            case NDT_ACFTYPE_ALL:
            case NDT_ACFTYPE_JET:
                break;
            default: // not applicable
                speed = 0;
                break;
        }
        switch (leg->type)
        {
            case NDT_LEGTYPE_HF:
            case NDT_LEGTYPE_HA:
            case NDT_LEGTYPE_HM:
            case NDT_LEGTYPE_ZZ:
                break;

            case NDT_LEGTYPE_FM:
            case NDT_LEGTYPE_VM:
                ret = print_line(fd, "-------", 0, 0, NDT_POSITION_NULL, 0);
                break;

            case NDT_LEGTYPE_DF:
            {
                if (leg->dst == leg->src && !ndt_list_count(leg->xpfms))
                {
                    if ((ret = print_line(fd, "-------", 0, 0, NDT_POSITION_NULL, 0)))
                    {
                        goto end;
                    }
                }
                for (size_t j = 0; j < ndt_list_count(leg->xpfms) && !fapchfix; j++)
                {
                    if ((ret = print_waypoint(fd, ndt_list_item(leg->xpfms, j), 0, speed)))
                    {
                        goto end;
                    }
                }
                ret = print_waypoint(fd, leg->dst, altitude, speed);
                break;
            }

            default:
            {
                for (size_t j = 0; j < ndt_list_count(leg->xpfms) && !fapchfix; j++)
                {
                    if (xpfms_skip4dist(ndt_list_item(leg->xpfms, j),
                                        ndt_list_item(leg->xpfms, j + 1),
                                        leg->dst, ndt_list_item(legs, i + 1)))
                    {
                        continue; // skip this dummy (too close to next waypoint)
                    }
                    if (xpfms_skip4dist(ndt_list_item(leg->xpfms, j), lst, NULL, NULL))
                    {
                        continue; // skip this dummy (too close to last fix)
                    }
                    if ((ret = print_waypoint(fd, ndt_list_item(leg->xpfms, j), leg->dst ? 0 : altitude, speed)))
                    {
                        goto end;
                    }
                }
                if (leg->dst)
                {
                    ret = print_waypoint(fd, leg->dst, altitude, speed);
                }
                break;
            }
        }
        if (ret)
        {
            goto end;
        }
        lst = leg->dst;
    }

end:
    return ret;
}

static int xpfms_flightplan_write(ndt_flightplan *flp, FILE *fd)
{
    /*
     * QPAC-specific hacks, part 1.
     *
     * Include depart. runway if we also have a SID  (for improved QPAC support).
     * Include arrival runway if we have an approach (for improved QPAC support).
     */
    int dep_rwy = (flp->dep.rwy && flp->dep.sid. proc);
    int arr_rwy = (flp->arr.rwy && flp->arr.apch.proc);
    /*
     * QPAC-specific hacks, part 2.
     *
     * Skip depart. airport if and only if:
     * - departure airport == arrival airport
     * - enroute portion of the plan is empty
     * - we have a STAR/approach selected too
     *
     * Skip arrival airport if and only if:
     * - departure airport == arrival airport
     * - enroute portion of the plan is empty
     * - we have a SID procedure selected too
     *
     * If we don't include the arrival airport (it's a SID) and we're writing
     * the departure runway as well, then also skip the departure airport.
     */
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlogical-not-parentheses"
#endif
    int dep_apt = !(flp->dep.apt == flp->arr.apt && !ndt_list_count(flp->rte) && (flp->arr.star.proc || flp->arr.apch.proc));
    int arr_apt = !(flp->dep.apt == flp->arr.apt && !ndt_list_count(flp->rte) && (flp->dep.sid.proc));
    if (arr_apt == 0 && !dep_rwy == 0)
    {
        dep_apt =  0;
    }
#ifdef __clang__
#pragma clang diagnostic pop
#endif
    /*
     * Count the legs based on the above and write our header.
     */
    int  ret, altitude;
    int  cnt = xpfms_count_legs(flp->legs) + dep_apt + dep_rwy + arr_rwy + arr_apt;
    if ((ret = xpfms_write_header(fd, cnt)))
    {
        goto end;
    }
    /*
     * And now the rest.
     */
    if (dep_apt)
    {
        if ((ret = print_airport(fd, flp->dep.apt)))
        {
            goto end;
        }
    }
    if (dep_rwy)
    {
        altitude = ndt_distance_get(flp->dep.rwy->threshold.altitude, NDT_ALTUNIT_FT);
        altitude = round(altitude / 10.) * 10;
        altitude = altitude + 10 * (altitude == 0); // regular waypoint
        if ((ret = print_waypoint(fd, flp->dep.rwy->waypoint, altitude, 0)))
        {
            goto end;
        }
    }
    if ((ret = xpfms_write_legs(fd, flp->legs, flp->arr.rwy)))
    {
        goto end;
    }
    if (arr_rwy)
    {
        /*
         * For GPS approaches, our runway threshold must be an NPA waypoint.
         *
         * For other approaches, just make it a regular waypoint, as overfly
         * waypoints may sometimes throw the QPAC plugin off, especially when
         * loading the final approach mid-flight. Example of this would be the
         * ILS approach for runway 19 at DTTA (Aerosoft 1511, with or without
         * the TUC approach transition). Not marking the arrival rwy threshold
         * as overfly may not fix the above case, but it's worth a try anyway.
         */
        switch (flp->arr.apch.proc->approach.type)
        {
            case NDT_APPRTYPE_GLS:
            case NDT_APPRTYPE_RNAV:
                altitude = ndt_distance_get(flp->arr.rwy->threshold.altitude, NDT_ALTUNIT_FT);
                altitude = round(altitude / 10.) * 10;
                altitude = altitude + 10 * (altitude == 0) - 1; // NPA waypoint
                break;
            default:
                altitude = ndt_distance_get(flp->arr.rwy->threshold.altitude, NDT_ALTUNIT_FT);
                altitude = round(altitude / 10.) * 10;
                altitude = altitude + 10 * (altitude == 0); // regular waypoint
                break;
        }
        if ((ret = print_waypoint(fd, flp->arr.rwy->waypoint, altitude, 0)))
        {
            goto end;
        }
    }
    if (arr_apt)
    {
        if ((ret = print_airport(fd, flp->arr.apt)))
        {
            goto end;
        }
    }
    /*
     * QPAC planes seem to write a double footer, let's do it here too.
     */
    if ((ret = xpfms_write_footer(fd)))
    {
        goto end;
    }

end:
    return ret ? ret : xpfms_write_footer(fd);
}

int ndt_fmt_xpfms_flightplan_write(ndt_flightplan *flp, FILE *fd, ndt_fltplanformat fmt)
{
    if (!flp || !fd)
    {
        return ENOMEM;
    }
    if (!flp->dep.apt || !flp->arr.apt)
    {
        ndt_log("[fmt_xpfms]: departure or arrival airport not set\n");
        return EINVAL;
    }
    switch (fmt)
    {
        case NDT_FLTPFMT_XPCDU:
            return helpr_flightplan_write(flp, fd, fmt);

        case NDT_FLTPFMT_XPCVA:
            return ceeva_flightplan_write(flp, fd, fmt);

        case NDT_FLTPFMT_XPHLP:
            return helpr_flightplan_write(flp, fd, fmt);

        case NDT_FLTPFMT_XPFMS:
            return xpfms_flightplan_write(flp, fd);

        default:
            ndt_log("[fmt_xpfms]: unsupported flight plan format '%d'\n", fmt);
            return EINVAL;
    }
}
