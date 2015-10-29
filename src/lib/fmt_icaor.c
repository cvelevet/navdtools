/*
 * fmt_icaor.c
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
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "common/common.h"
#include "common/list.h"

#include "compat/compat.h"

#include "wmm/wmm.h"

#include "airway.h"
#include "flightplan.h"
#include "fmt_icaor.h"
#include "waypoint.h"

static int icao_printrt(FILE *fd, ndt_list *rte, ndt_fltplanformat fmt);
static int icao_printlg(FILE *fd, ndt_list *lgs, ndt_fltplanformat fmt);

int ndt_fmt_icaor_flightplan_set_route(ndt_flightplan *flp, const char *rte)
{
    int           err      = 0;
    ndt_airport  *lastapt  = NULL;
    ndt_runway   *lastrwy  = NULL;
    ndt_waypoint *src      = NULL, *lastpl = NULL;
    char         *awy1id   = NULL, *awy2id = NULL;
    char         *rtestart = NULL, *prefix = NULL, *rtenext, *elem;

    if (!flp || !rte)
    {
        err = ENOMEM;
        goto end;
    }

    rtestart = rtenext = strdup(rte);
    if (!rtestart)
    {
        err = ENOMEM;
        goto end;
    }

    while ((elem = strsep(&rtenext, " \r\n\t")))
    {
        if (strnlen   (elem,      1) &&
            strcasecmp(elem,  "DCT") &&
            strcasecmp(elem,  "SID") &&
            strcasecmp(elem, "STAR"))
        {
            char         *dstidt = NULL;
            ndt_waypoint *cuswpt = NULL;

            /*
             * Split each element in two substrings.
             *
             * The second substring contains additional info
             * (step climbs?) which we ignore for now.
             */
            char *suffix = strdup(elem);
            if (prefix)
            {
                free(prefix);
            }
            prefix = strsep(&suffix, "/");

            /* New element, reset last airport & runway. */
            lastapt = NULL;
            lastrwy = NULL;

            if (!src)
            {
                /*
                 * If the first waypoint is an airport, save the matching
                 * airport (and runway, if applicable) for later use.
                 */
                ndt_airport *firstapt = ndt_navdata_get_airport(flp->ndb, prefix);
                ndt_runway  *firstrwy;

                if (firstapt && suffix)
                {
                    firstrwy = ndt_runway_get(firstapt, suffix);
                }
                else
                {
                    firstrwy = NULL;
                }

                /* Set the departure airport and runway if required. */
                if (!flp->dep.apt || (!flp->dep.rwy && flp->dep.apt == firstapt))
                {
                    err = ndt_flightplan_set_departure(flp,
                                                       firstapt ? firstapt->info.idnt : NULL,
                                                       firstrwy ? firstrwy->info.idnt : NULL);
                    if (err)
                    {
                        ndt_log("[fmt_icaor]: invalid departure '%s%s'\n",
                                firstapt ? firstapt->info.idnt : NULL,
                                firstrwy ? firstrwy->info.idnt : "");
                        goto end;
                    }
                }

                /*
                 * The first source is the departure airport or runway.
                 */
                src = flp->dep.rwy ? flp->dep.rwy->waypoint : flp->dep.apt->waypoint;
            }

            /*
             * Place-bearing-distance; supported formats:
             *
             *     PlaceBearingDistance   (bearing and distance must be 3-digit)
             *     PlaceBearing/Distance
             *     Place/Bearing/Distance
             *
             * Place-Bearing/Place-Bearing: not yet implemented.
             *
             * Note: trailing character specifier avoids false matches.
             */
            ndt_distance ndstce;
            char         place[8];
            double       bearing, distance;
            int          bgbuf[3], dibuf[3];
            if (sscanf(prefix, "%5[^0-9]%1d%1d%1d%1d%1d%1d%c", place,
                       &bgbuf[0],
                       &bgbuf[1],
                       &bgbuf[2],
                       &dibuf[0],
                       &dibuf[1],
                       &dibuf[2], place) == 7)
            {
                bearing  = bgbuf[0] * 100. + bgbuf[1] * 10. + bgbuf[2] * 1.;
                distance = dibuf[0] * 100. + dibuf[1] * 10. + dibuf[2] * 1.;
            }
            else if (sscanf(elem, "%7[^/]/%lf/%lf%c",  place, &bearing, &distance, place) != 3 &&
                     sscanf(elem, "%7[^0-9]%lf/%lf%c", place, &bearing, &distance, place) != 3)
            {
                // no match
                bearing = distance = -1.;
            }

            if (distance > 0. &&
                bearing >= 0. && bearing <= 360. &&
                ndt_navdata_get_waypoint(flp->ndb, place, NULL))
            {
                /*
                 * Handled first because we have a specific, reliable match.
                 *
                 * Check if we already have a named waypoint matching the place
                 * in our flightplan and use it, else select the closest one.
                 *
                 * This avoids the following hypothetical situation:
                 *
                 * "[...] place foo place/bearing/distance [...]"
                 *
                 * ...where a second, distinct "place" waypoint exists and is
                 * closer to "foo" than the first "place" waypoint. Assume the
                 * intention is for "place/bearing/distance" to refer to the
                 * same waypoint as the first "place" waypoint.
                 */
                if (lastpl == NULL || strcasecmp(lastpl->info.idnt, place))
                {
                    lastpl  = ndt_navdata_get_wptnear2(flp->ndb, place, NULL, src->position);
                }
                if (lastpl == NULL)
                {
                    err = ENOMEM; // should never happen
                    goto end;
                }

                // convert nautical miles to meters for distance
                ndstce = ndt_distance_init((int64_t)(distance * 1852.), NDT_ALTUNIT_ME);
                cuswpt = ndt_waypoint_pbd(lastpl, bearing, ndstce,
                                          ndt_date_now(), flp->ndb->wmm);
            }
            else if (0)
            {
                /* TODO: handle or skip SID and STAR procedures */
            }
            else if (strlen(prefix) == 4 && !strncasecmp(prefix, "NAT", 3))
            {
                /*
                 * North Atlantic Track; we can't decode it,
                 * but skip it rather than bailing out.
                 */
                continue;
            }
            else if (ndt_navdata_get_airway(flp->ndb, elem, NULL))
            {
                ndt_airway_leg *in;
                ndt_airway     *awy1, *awy2;
                ndt_waypoint   *dst;

                if (awy1id)
                {
                    /*
                     * Two consecutive airways. Ensure they do intersect; else
                     * check it's not an endpoint instead (e.g. 'T103' is both a
                     * waypoint and an airway in AIRAC 1405). If both fail, set
                     * awy2id so we can print an airway-specific error later.
                     */
                    for (size_t awy1idx = 0; (awy1 = ndt_navdata_get_airway(flp->ndb, awy1id, &awy1idx)); awy1idx++)
                    {
                        if ((in = ndt_airway_startpoint(awy1, src->info.idnt, src->position)))
                        {
                            for (size_t awy2idx = 0; (awy2 = ndt_navdata_get_airway(flp->ndb, elem, &awy2idx)); awy2idx++)
                            {
                                if (ndt_airway_intersect(in, awy2))
                                {
                                    awy2id = strdup(elem);
                                    break;
                                }
                            }
                            if (awy2id)
                            {
                                break;
                            }
                            for (size_t dstidx = 0; (dst = ndt_navdata_get_waypoint(flp->ndb, prefix, &dstidx)); dstidx++)
                            {
                                if (ndt_airway_endpoint(in, dst->info.idnt, dst->position))
                                {
                                    dstidt = prefix;
                                    break;
                                }
                            }
                            if (dstidt)
                            {
                                break;
                            }
                        }
                    }
                    if (!awy2id && !dstidt)
                    {
                         awy2id = strdup(elem);
                    }
                }
                else
                {
                    /*
                     * Make sure src is a valid startpoint for the airway; else
                     * check it's not a waypoint instead (e.g. 'T103' is both a
                     * waypoint and an airway in AIRAC 1405). If both fail, set
                     * awy1id so we can print an airway-specific error later.
                     */
                    for (size_t awy1idx = 0; (awy1 = ndt_navdata_get_airway(flp->ndb, elem, &awy1idx)); awy1idx++)
                    {
                        if (ndt_airway_startpoint(awy1, src->info.idnt, src->position))
                        {
                            awy1id = strdup(elem);
                            break;
                        }
                    }
                    if (!awy1id)
                    {
                        ndt_route_segment *rsg1 = ndt_list_item(flp->rte, ndt_list_count(flp->rte) - 1);
                        /*
                         * If and only if the previous segment is a direct, then
                         * src was chosen somewhat randomly (based on distance).
                         * There may actually be a waypoint with an identical
                         * name that's a valid startpoint for this airway.
                         *
                         * Check all valid startpoints matching src->info.idnt
                         * (which is also rsg1->dst->info.idnt), and if we find
                         * one, overwrite src (which is also rsg1->dst).
                         *
                         * Else resume normal behavior (try it as waypoint, or
                         * set awy1id so we can print an airway-specific error).
                         */
                        if (rsg1 && rsg1->type == NDT_RSTYPE_DCT)
                        {
                            for (size_t dstidx = 0; (dst = ndt_navdata_get_waypoint(flp->ndb, rsg1->dst->info.idnt, &dstidx)); dstidx++)
                            {
                                for (size_t awy1idx = 0; (awy1 = ndt_navdata_get_airway(flp->ndb, elem, &awy1idx)); awy1idx++)
                                {
                                    if (ndt_airway_startpoint(awy1, dst->info.idnt, dst->position))
                                    {
                                        src    = rsg1->dst = dst;
                                        awy1id = strdup(elem);
                                        break;
                                    }
                                }
                                if (awy1id)
                                {
                                    break;
                                }
                            }
                        }
                    }
                    if (!awy1id)
                    {
                        if (ndt_navdata_get_waypoint(flp->ndb, prefix, NULL))
                        {
                            dstidt = prefix;
                        }
                        else
                        {
                            awy1id = strdup(elem);
                        }
                    }
                }
            }
            else if (ndt_navdata_get_waypoint(flp->ndb, prefix, NULL))
            {
                dstidt = prefix;
            }
            else if (cuswpt == NULL && ((cuswpt = ndt_waypoint_llc(elem)) ||
                                        (cuswpt = ndt_waypoint_llc(prefix))))
            {
                /*
                 * Valid latitude and longitude coordinates.
                 *
                 * Note: always check the full element first to avoid
                 * a false match, e.g. e.g. '4600N' for '4600N/05000W'.
                 */
                ndt_list_add(flp->cws, cuswpt);
            }
            else
            {
                ndt_log("[fmt_icaor]: invalid element '%s'\n", elem);
                err = EINVAL;
                goto end;
            }

            /*
             * If we have an endpoint (waypoint or consecutive airway), we can
             * create a new route segment and add it to the flight plan.
             */
            if (awy2id || dstidt || cuswpt)
            {
                ndt_route_segment *rsg = NULL;
                ndt_waypoint      *dst = NULL;
                ndt_airway        *awy = NULL;
                ndt_airway_leg    *out = NULL, *in = NULL;

                if      (awy2id)
                {
                    dst = ndt_navdata_get_wpt4aws(flp->ndb, src, awy2id, awy1id, &awy, &in, &out);
                    free(awy1id);
                    awy1id = awy2id;
                    awy2id = NULL;
                }
                else if (awy1id)
                {
                    dst = ndt_navdata_get_wpt4awy(flp->ndb, src, dstidt, awy1id, &awy, &in, &out);
                    free(awy1id);
                    awy1id = NULL;
                }
                else if (dstidt)
                {
                    dst = ndt_navdata_get_wptnear2(flp->ndb, dstidt, NULL, src->position);

                    /*
                     * If the last waypoint is an airport, save the matching
                     * airport (and runway, if applicable) for later use.
                     */
                    if (dst && dst->type == NDT_WPTYPE_APT)
                    {
                        lastapt = ndt_navdata_get_airport(flp->ndb, dst->info.idnt);
                        if (lastapt && suffix)
                        {
                            lastrwy = ndt_runway_get(lastapt, suffix);
                        }
                        else
                        {
                            lastrwy = NULL;
                        }
                    }
                }
                else if (cuswpt)
                {
                    dst = cuswpt;
                }

                if (!dst)
                {
                    err = EINVAL;
                    goto end;
                }

                if (awy && in && out)
                {
                    rsg = ndt_route_segment_airway(src, dst, awy, in, out, flp->ndb);
                }
                else
                {
                    rsg = ndt_route_segment_direct(src, dst, flp->ndb);
                }

                if (!rsg)
                {
                    err = ENOMEM;
                    goto end;
                }

                /* Remove pointless legs */
                if (rsg->type == NDT_RSTYPE_DCT)
                {
                    if (rsg->src == rsg->dst)
                    {
                        ndt_route_segment_close(&rsg);
                        continue;
                    }
                    else
                    {
                        ndt_position a = rsg->src->position;
                        ndt_position b = rsg->dst->position;
                        ndt_distance d = ndt_position_calcdistance(a, b);
                        if (ndt_distance_get(d, NDT_ALTUNIT_NA) == 0)
                        {
                            ndt_route_segment_close(&rsg);
                            continue;
                        }
                    }
                }

                /* We have a leg, our last endpoint becomes our new startpoint */
                if (rsg->dst->type != NDT_WPTYPE_LLC)
                {
                    lastpl = rsg->dst;
                }
                src = rsg->dst;

                /* Let's not forget to add our new segment to the route */
                ndt_list_add(flp->rte, rsg);
                continue;
            }
        }
    }

    /* Set the arrival airport and runway if required. */
    if (!flp->arr.apt || (!flp->arr.rwy && flp->arr.apt == lastapt))
    {
        err = ndt_flightplan_set_arrival(flp,
                                         lastapt ? lastapt->info.idnt : NULL,
                                         lastrwy ? lastrwy->info.idnt : NULL);
        if (err)
        {
            ndt_log("[fmt_icaor]: invalid arrival '%s%s'\n",
                    lastapt ? lastapt->info.idnt : NULL,
                    lastrwy ? lastrwy->info.idnt : "");
            goto end;
        }
    }

    /*
     * Remove unwanted segments.
     */
    ndt_route_segment *fst = ndt_list_item(flp->rte,  0);
    ndt_route_segment *scd = ndt_list_item(flp->rte,  1);
    ndt_route_segment *pen = ndt_list_item(flp->rte, -2);
    ndt_route_segment *ult = ndt_list_item(flp->rte, -1);
    if (fst && fst->type == NDT_RSTYPE_DCT)
    {
        if (flp->dep.apt->waypoint == fst->dst)
        {
            // check for duplicates
            if (fst == scd)
            {
                scd = NULL;
            }
            if (fst == pen)
            {
                pen = NULL;
            }
            if (fst == ult)
            {
                ult = NULL;
            }

            // don't include the departure airport as the first leg
            ndt_list_rem  (flp->rte, fst);
            ndt_route_segment_close(&fst);

            // if we have an airport, the next leg may be a runway
            if (scd != NULL  && scd->type == NDT_RSTYPE_DCT &&
                flp->dep.rwy && flp->dep.rwy->waypoint == scd->dst)
            {
                // check for duplicates
                if (scd == pen)
                {
                    pen = NULL;
                }
                if (scd == ult)
                {
                    ult = NULL;
                }

                // don't include the departure runway as the second leg
                ndt_list_rem  (flp->rte, scd);
                ndt_route_segment_close(&scd);
            }
        }
        else if (flp->dep.rwy && flp->dep.rwy->waypoint == fst->dst)
        {
            // check for duplicates
            if (fst == scd)
            {
                scd = NULL;
            }
            if (fst == pen)
            {
                pen = NULL;
            }
            if (fst == ult)
            {
                ult = NULL;
            }

            // don't include the departure runway as the first leg
            ndt_list_rem  (flp->rte, fst);
            ndt_route_segment_close(&fst);
        }
    }
    if (ult && ult->type == NDT_RSTYPE_DCT)
    {
        if (flp->arr.apt->waypoint == ult->dst)
        {
            // check for duplicates
            if (ult == pen)
            {
                pen = NULL;
            }

            // don't include the arrival airport as the last leg
            ndt_list_rem  (flp->rte, ult);
            ndt_route_segment_close(&ult);

            // if we have an airport, the previous leg may be a runway
            if (pen != NULL  && pen->type == NDT_RSTYPE_DCT &&
                flp->arr.rwy && flp->arr.rwy->waypoint == pen->dst)
            {
                // don't include the arrival runway as the penultimate leg
                ndt_list_rem  (flp->rte, pen);
                ndt_route_segment_close(&pen);
            }
        }
        else if (flp->arr.rwy && flp->arr.rwy->waypoint == ult->dst)
        {
            // don't include the arrival runway as the last leg
            ndt_list_rem  (flp->rte, ult);
            ndt_route_segment_close(&ult);
        }
    }

end:
    free(rtestart);
    free(awy1id);
    free(awy2id);
    free(prefix);
    return err;
}

int ndt_fmt_dcded_flightplan_write(ndt_flightplan *flp, FILE *fd)
{
    int ret = 0;

    if (!flp || !fd)
    {
        ret = ENOMEM;
        goto end;
    }

    if (!flp->dep.apt || !flp->arr.apt)
    {
        ndt_log("[fmt_dcded]: departure or arrival airport not set\n");
        ret = EINVAL;
        goto end;
    }

    // decoded route only
    ret = icao_printlg(fd, flp->legs, NDT_FLTPFMT_DCDED);
    if (ret)
    {
        goto end;
    }
    ret = ndt_fprintf(fd, "%s", "\n");
    if (ret)
    {
        goto end;
    }

end:
    return ret;
}

int ndt_fmt_icaor_flightplan_write(ndt_flightplan *flp, FILE *fd)
{
    int ret = 0;

    if (!flp || !fd)
    {
        ret = ENOMEM;
        goto end;
    }

    if (!flp->dep.apt || !flp->arr.apt)
    {
        ndt_log("[fmt_icaor]: departure or arrival airport not set\n");
        ret = EINVAL;
        goto end;
    }

    // departure airport
    ret = ndt_fprintf(fd, "%s SID%s", flp->dep.apt->info.idnt,
                      ndt_list_count(flp->rte) ? " " : "");
    if (ret)
    {
        goto end;
    }

    // encoded route
    ret = icao_printrt(fd, flp->rte, NDT_FLTPFMT_ICAOR);
    if (ret)
    {
        goto end;
    }

    // arrival airport
    ret = ndt_fprintf(fd, " STAR %s\n", flp->arr.apt->info.idnt);
    if (ret)
    {
        goto end;
    }

end:
    return ret;
}

int ndt_fmt_irecp_flightplan_write(ndt_flightplan *flp, FILE *fd)
{
    char sbrif[13], recap[24];
    int ret = 0, d01, d02;
    const char *surface;
    double disnmile;

    if (!flp || !fd)
    {
        ret = ENOMEM;
        goto end;
    }

    if (!flp->dep.apt || !flp->arr.apt)
    {
        ndt_log("[fmt_irecp]: departure or arrival airport not set\n");
        ret = EINVAL;
        goto end;
    }

    // departure airport and runway
    d01 = ndt_distance_get(flp->dep.apt->coordinates.altitude, NDT_ALTUNIT_FT);
    d02 = ndt_distance_get(flp->dep.apt->tr_altitude,          NDT_ALTUNIT_FT);
    if (d02)
    {
        ret = ndt_fprintf(fd, "Departure: %s (%s), elevation %d ft, transition altitude %d ft\n",
                          flp->dep.apt->info.idnt,
                          flp->dep.apt->info.misc, d01, d02);
    }
    else
    {
        ret = ndt_fprintf(fd, "Departure: %s (%s), elevation %d ft, transition altitude ATC\n",
                          flp->dep.apt->info.idnt,
                          flp->dep.apt->info.misc, d01);
    }
    if (ret)
    {
        goto end;
    }
    if (flp->dep.rwy)
    {
        d01 = ndt_distance_get(flp->dep.rwy->length, NDT_ALTUNIT_FT);
        d02 = ndt_distance_get(flp->dep.rwy->width,  NDT_ALTUNIT_FT);
        switch (flp->dep.rwy->surface)
        {
            case NDT_RWYSURF_ASPHT:
                surface = "asphalt";
                break;
            case NDT_RWYSURF_CONCR:
                surface = "concrete";
                break;
            case NDT_RWYSURF_GRASS:
                surface = "grass";
                break;
            case NDT_RWYSURF_GRAVL:
                surface = "gravel";
                break;
            case NDT_RWYSURF_WATER:
                surface = "water";
                break;
            default:
                surface = "unknown";
                break;
        }
        if (flp->dep.rwy->ils.avail)
        {
            ret = ndt_fprintf(fd, "Runway:    %s (%d°), %d (%d) ft, surface: %s, ILS: %.2lf (%d°, %.1lf°)\n",
                              flp->dep.rwy->info.idnt,
                              flp->dep.rwy->heading, d01, d02, surface,
                              ndt_frequency_get(flp->dep.rwy->ils.freq),
                              flp->dep.rwy->ils.course,
                              flp->dep.rwy->ils.slope);
        }
        else
        {
            ret = ndt_fprintf(fd, "Runway:    %s, %3d°, %d (%d) ft, surface: %s\n",
                              flp->dep.rwy->info.idnt,
                              flp->dep.rwy->heading, d01, d02, surface);
        }
        if (ret)
        {
            goto end;
        }
    }
    ret = ndt_fprintf(fd, "%s", "\n");
    if (ret)
    {
        goto end;
    }

    // TODO: SID and transition(s)

    // flight route
    if (ndt_list_count(flp->rte) == 0)
    {
        ret = ndt_fprintf(fd, "%s", "Flight route: DIRECT\n");
    }
    else
    {
        ret = ndt_fprintf(fd, "%s", "Flight route:\n");
    }
    if (ret)
    {
        goto end;
    }
    for (size_t i = 0; i < ndt_list_count(flp->rte); i++)
    {
        ndt_route_segment *rsg = ndt_list_item(flp->rte, i);
        if (!rsg)
        {
            ret = ENOMEM;
            goto end;
        }

        switch (rsg->type)
        {
            case NDT_RSTYPE_AWY:
            {
                for (size_t j = 0; j < ndt_list_count(rsg->legs); j++)
                {
                    ndt_route_leg *leg = ndt_list_item(rsg->legs, j);
                    if (!leg)
                    {
                        ret = ENOMEM;
                        break;
                    }

                    switch (leg->type)
                    {
                        case NDT_LEGTYPE_TF:
                        {
                            disnmile = ndt_distance_get(leg->dis, NDT_ALTUNIT_ME) / 1852.;
                            if ((ret = ndt_fprintf(fd,
                                                   "\n%-14s  %05.1lf° (%05.1lf°T) %5.1lf nm\n",
                                                   leg->awy. leg->awy->info.idnt,
                                                   leg->omb, leg->trb, disnmile)))
                            {
                                break;
                            }
                            if (leg->dst->info.misc[0])
                            {
                                if ((ret = ndt_fprintf(fd, "%s\n", leg->dst->info.misc)))
                                {
                                    break;
                                }
                            }
                            if ((ret = ndt_fprintf(fd, "%-19s  %s\n", leg->dst->info.idnt, recap)))
                            {
                                break;
                            }
                            break;
                        }

                        case NDT_LEGTYPE_ZZ:
                            ret = ndt_fprintf(fd, "\n%s\n", "       ----F-PLN DISCONTINUITY----");
                            break;

                        default:
                            ndt_log("[fmt_irecp]: unknown leg type '%d'\n", leg->type);
                            ret = EINVAL;
                            break;
                    }
                    if (ret)
                    {
                        break;
                    }
                }
                break;
            }

            case NDT_RSTYPE_DCT:
            {
                ndt_route_leg *leg = ndt_list_item(rsg->legs, 0);
                if (!leg)
                {
                    ret = ENOMEM;
                    break;
                }

                // pre-print latitude and longitude coordinates
                if (ndt_position_sprintllc(leg->dst->position, NDT_LLCFMT_SBRIF,
                                           sbrif, sizeof(sbrif)) < 0)
                {
                    ret = EIO;
                    break;
                }
                if (ndt_position_sprintllc(leg->dst->position, NDT_LLCFMT_RECAP,
                                           recap, sizeof(recap)) < 0)
                {
                    ret = EIO;
                    break;
                }

                switch (leg->type)
                {
                    case NDT_LEGTYPE_TF:
                    {
                        if (leg->src)
                        {
                            disnmile = ndt_distance_get(leg->dis, NDT_ALTUNIT_ME) / 1852.;
                            if ((ret = ndt_fprintf(fd,
                                                   "\n%-14s  %05.1lf° (%05.1lf°T) %5.1lf nm\n",
                                                   "DCT", leg->omb, leg->trb, disnmile)))
                            {
                                break;
                            }
                        }
                        else
                        {
                            if ((ret = ndt_fprintf(fd, "\n%s\n", "DCT")))
                            {
                                break;
                            }
                        }
                        if (leg->dst->type != NDT_WPTYPE_LLC &&
                            leg->dst->info.misc[0])
                        {
                            if ((ret = ndt_fprintf(fd, "%s\n", leg->dst->info.misc)))
                            {
                                break;
                            }
                        }
                        if (leg->dst->type == NDT_WPTYPE_LLC)
                        {
                            if ((ret = ndt_fprintf(fd, "%-19s  %s\n", sbrif, recap)))
                            {
                                break;
                            }
                        }
                        else
                        {
                            if ((ret = ndt_fprintf(fd, "%-19s  %s\n", leg->dst->info.idnt, recap)))
                            {
                                break;
                            }
                        }
                        break;
                    }

                    case NDT_LEGTYPE_ZZ:
                        ret = ndt_fprintf(fd, "\n%s\n", "       ----F-PLN DISCONTINUITY----");
                        break;

                    default:
                        ndt_log("[fmt_irecp]: unknown leg type '%d'\n", leg->type);
                        ret = EINVAL;
                        break;
                }
                break;
            }

            case NDT_RSTYPE_DSC:
                ret = ndt_fprintf(fd, "\n%s\n", "       ----F-PLN DISCONTINUITY----");
                break;

            default:
                ndt_log("[fmt_irecp]: unknown segment type '%d'\n", rsg->type);
                ret = EINVAL;
                break;
        }
        if (ret)
        {
            goto end;
        }
    }

    // TODO: STAR and transition(s)

    // TODO: approach (maybe not?)

    // last leg (TODO: check if final approach selected)
    if (ndt_list_count(flp->rte))
    {
        // route not direct, and no final approach, add
        // a dummy leg to the arrival airport or runway
        ndt_waypoint  *dst = flp->arr.rwy ? flp->arr.rwy->waypoint : flp->arr.apt->waypoint;

        // pre-print latitude and longitude coordinates
        if (ndt_position_sprintllc(dst->position, NDT_LLCFMT_RECAP,
                                   recap, sizeof(recap)) < 0)
        {
            ret = EIO;
            goto  end;
        }

        // the source is the last leg's dst, if it exists
        ndt_route_leg *leg = ndt_list_item(flp->legs, -1);
        if (leg && leg->dst)
        {
            ndt_distance d = ndt_position_calcdistance(leg->dst->position,  dst->position);
            double tru_brg = ndt_position_calcbearing (leg->dst->position,  dst->position);
            double mag_brg = ndt_wmm_getbearing_mag(flp->ndb->wmm, tru_brg, leg->dst->position, ndt_date_now());
            double dis_mil = ndt_distance_get(d, NDT_ALTUNIT_ME) / 1852.;
            if ((ret = ndt_fprintf(fd,
                                   "\n%-14s  %05.1lf° (%05.1lf°T) %5.1lf nm\n",
                                   "DCT", mag_brg, tru_brg, dis_mil)))
            {
                goto end;
            }
            if ((ret = ndt_fprintf(fd, "%-19s  %s\n", dst->info.idnt, recap)))
            {
                goto end;
            }
        }
    }
    ret = ndt_fprintf(fd, "%s", "\n");
    if (ret)
    {
        goto end;
    }

    // arrival airport and runway
    d01 = ndt_distance_get(flp->arr.apt->coordinates.altitude, NDT_ALTUNIT_FT);
    d02 = ndt_distance_get(flp->arr.apt->trans_level,          NDT_ALTUNIT_FT);
    if (d02)
    {
        ret = ndt_fprintf(fd, "Arrival: %s (%s), elevation %d ft, transition level FL%d\n",
                          flp->arr.apt->info.idnt,
                          flp->arr.apt->info.misc, d01, d02 / 100);
    }
    else
    {
        ret = ndt_fprintf(fd, "Arrival: %s (%s), elevation %d ft, transition level ATC\n",
                          flp->arr.apt->info.idnt,
                          flp->arr.apt->info.misc, d01);
    }
    if (ret)
    {
        goto end;
    }
    if (flp->arr.rwy)
    {
        d01 = ndt_distance_get(flp->arr.rwy->length, NDT_ALTUNIT_FT);
        d02 = ndt_distance_get(flp->arr.rwy->width,  NDT_ALTUNIT_FT);
        switch (flp->arr.rwy->surface)
        {
            case NDT_RWYSURF_ASPHT:
                surface = "asphalt";
                break;
            case NDT_RWYSURF_CONCR:
                surface = "concrete";
                break;
            case NDT_RWYSURF_GRASS:
                surface = "grass";
                break;
            case NDT_RWYSURF_GRAVL:
                surface = "gravel";
                break;
            case NDT_RWYSURF_WATER:
                surface = "water";
                break;
            default:
                surface = "unknown";
                break;
        }
        if (flp->arr.rwy->ils.avail)
        {
            ret = ndt_fprintf(fd, "Runway:  %s (%d°), %d (%d) ft, surface: %s, ILS: %.2lf (%d°, %.1lf°)\n",
                              flp->arr.rwy->info.idnt,
                              flp->arr.rwy->heading, d01, d02, surface,
                              ndt_frequency_get(flp->arr.rwy->ils.freq),
                              flp->arr.rwy->ils.course,
                              flp->arr.rwy->ils.slope);
        }
        else
        {
            ret = ndt_fprintf(fd, "Runway:  %s, %3d°, %d (%d) ft, surface: %s\n",
                              flp->arr.rwy->info.idnt,
                              flp->arr.rwy->heading, d01, d02, surface);
        }
        if (ret)
        {
            goto end;
        }
    }

end:
    return ret;
}

int ndt_fmt_sbrif_flightplan_write(ndt_flightplan *flp, FILE *fd)
{
    int ret = 0;

    if (!flp || !fd)
    {
        ret = ENOMEM;
        goto end;
    }

    if (!flp->dep.apt || !flp->arr.apt)
    {
        ndt_log("[fmt_sbrif]: departure or arrival airport not set\n");
        ret = EINVAL;
        goto end;
    }

    // encoded route only
    ret = icao_printrt(fd, flp->rte, NDT_FLTPFMT_SBRIF);
    if (ret)
    {
        goto end;
    }
    ret = ndt_fprintf(fd, "%s", "\n");
    if (ret)
    {
        goto end;
    }

end:
    return ret;
}

static int icao_printrt(FILE *fd, ndt_list *rte, ndt_fltplanformat fmt)
{
    ndt_llcfmt llcfmt;
    int ret = 0;

    if (!fd || !rte)
    {
        ret = ENOMEM;
        goto end;
    }

    switch (fmt)
    {
        case NDT_FLTPFMT_ICAOR:
            llcfmt = NDT_LLCFMT_ICAOR;
            break;

        case NDT_FLTPFMT_SBRIF:
            llcfmt = NDT_LLCFMT_SBRIF;
            break;

        default:
            ndt_log("[icao_printrt]: unsupported flight plan format '%d'\n", fmt);
            ret = EINVAL;
            goto end;
    }

    for (size_t i = 0; i < ndt_list_count(rte); i++)
    {
        ndt_route_segment *rsg = ndt_list_item(rte, i);
        if (!rsg)
        {
            ret = ENOMEM;
            goto end;
        }

        switch (rsg->type)
        {
            case NDT_RSTYPE_AWY:
                ret = ndt_fprintf(fd, "%s %s", rsg->awy.awy->info.idnt, rsg->dst->info.idnt);
                break;

            case NDT_RSTYPE_DCT:
                {
                    switch (rsg->dst->type)
                    {
                        case NDT_WPTYPE_APT:
                        case NDT_WPTYPE_DME:
                        case NDT_WPTYPE_FIX:
                        case NDT_WPTYPE_NDB:
                        case NDT_WPTYPE_VOR: // use the identifier
                            ret = ndt_fprintf(fd, "%s", rsg->dst->info.idnt);
                            break;

                        case NDT_WPTYPE_PBD:
                            if ((fmt == NDT_FLTPFMT_ICAOR || fmt == NDT_FLTPFMT_SBRIF) &&
                                (rsg->dst->pbd.place->type == NDT_WPTYPE_FIX ||
                                 rsg->dst->pbd.place->type == NDT_WPTYPE_NDB ||
                                 rsg->dst->pbd.place->type == NDT_WPTYPE_VOR))
                            {
                                double   nm = ndt_distance_get(rsg->dst->pbd.distance, NDT_ALTUNIT_ME) / 1852.;
                                if (fabs(nm - round(nm)) < .05) // distance in nm is basically an integer, yay!
                                {
                                    ret = ndt_fprintf(fd, "%5s%03.0lf%03.0lf",
                                                      rsg->dst->pbd.place->info.idnt,
                                                      rsg->dst->pbd.bearing, round(nm));
                                    break;
                                }
                            }
                        default: // use latitude/longitude coordinates
                            ret = ndt_position_fprintllc(rsg->dst->position, llcfmt, fd);
                            break;
                    }
                    break;
                }

            case NDT_RSTYPE_DSC: // skip discontinuities
                break;

            default:
                ndt_log("[icao_printrt]: unknown segment type '%d'\n", rsg->type);
                ret = EINVAL;
                break;
        }
        if (ret)
        {
            goto end;
        }
        if (i + 1 < ndt_list_count(rte))
        {
            ret = ndt_fprintf(fd, "%s", " ");
            if (ret)
            {
                goto end;
            }
        }
    }

end:
    return ret;
}

static int icao_printlg(FILE *fd, ndt_list *lgs, ndt_fltplanformat fmt)
{
    ndt_llcfmt llcfmt;
    int ret = 0;

    if (!fd || !lgs)
    {
        ret = ENOMEM;
        goto end;
    }

    switch (fmt)
    {
        case NDT_FLTPFMT_DCDED:
            llcfmt = NDT_LLCFMT_SVECT;
            break;

        default:
            ndt_log("[icao_printlg]: unsupported flight plan format '%d'\n", fmt);
            ret = EINVAL;
            goto end;
    }

    for (size_t i = 0; i < ndt_list_count(lgs); i++)
    {
        ndt_route_leg *leg = ndt_list_item(lgs, i);
        if (!leg)
        {
            ret = ENOMEM;
            goto end;
        }

        switch (leg->type)
        {
            case NDT_LEGTYPE_TF:
                {
                    switch (leg->dst->type)
                    {
                        case NDT_WPTYPE_APT:
                        case NDT_WPTYPE_DME:
                        case NDT_WPTYPE_FIX:
                        case NDT_WPTYPE_NDB:
                        case NDT_WPTYPE_VOR: // use the identifier
                            ret = ndt_fprintf(fd, "%s", leg->dst->info.idnt);
                            break;

                        case NDT_WPTYPE_PBD:
                            if ((fmt == NDT_FLTPFMT_DCDED) &&
                                (leg->dst->pbd.place->type == NDT_WPTYPE_FIX ||
                                 leg->dst->pbd.place->type == NDT_WPTYPE_NDB ||
                                 leg->dst->pbd.place->type == NDT_WPTYPE_VOR))
                            {
                                double   nm = ndt_distance_get(leg->dst->pbd.distance, NDT_ALTUNIT_ME) / 1852.;
                                if (fabs(nm - round(nm)) < .05) // distance in nm is basically an integer, yay!
                                {
                                    ret = ndt_fprintf(fd, "%5s%03.0lf%03.0lf",
                                                      leg->dst->pbd.place->info.idnt,
                                                      leg->dst->pbd.bearing, round(nm));
                                    break;
                                }
                            }
                        default: // use latitude/longitude coordinates
                            ret = ndt_position_fprintllc(leg->dst->position, llcfmt, fd);
                            break;
                    }
                    break;
                }

            case NDT_LEGTYPE_ZZ: // skip discontinuities
                break;

            default:
                ndt_log("[icao_printlg]: unknown leg type '%d'\n", leg->type);
                ret = EINVAL;
                break;
        }
        if (ret)
        {
            goto end;
        }
        if (i + 1 < ndt_list_count(lgs))
        {
            ret = ndt_fprintf(fd, "%s", " ");
            if (ret)
            {
                goto end;
            }
        }
    }

end:
    return ret;
}
