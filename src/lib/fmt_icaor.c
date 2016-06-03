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

static int icao_printrt(FILE *fd, ndt_list     *rte,                    ndt_fltplanformat fmt);
static int icao_printwp(FILE *fd, ndt_waypoint *dst, ndt_llcfmt llcfmt, ndt_fltplanformat fmt);
static int icao_printlg(FILE *fd, ndt_list     *lgs,                    ndt_fltplanformat fmt);

int ndt_fmt_icaor_flightplan_set_route(ndt_flightplan *flp, const char *rte)
{
    int err = 0, init = 0, fv = 0;
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
        if (strnlen(elem,      1) &&
            strcmp (elem,  "DCT") &&
            strcmp (elem,  "SID") &&
            strcmp (elem, "STAR"))
        {
            char         *dstidt = NULL;
            ndt_waypoint *cuswpt = NULL;
            ndt_position lastpos;

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
            /*
             * AIRPORT/RUNWAY           KLAX/06L
             * WAYPOINT/STEPCLIMB       BASIK/N0412F330
             * SID.TRANS, STAR.TRANS    CASTA4.AVE
             */
            prefix = strsep(&suffix, "/.");

            /* New element, reset last airport & runway. */
            lastapt = NULL;
            lastrwy = NULL;

            if (!init)
            {
                /*
                 * If the first waypoint is an airport, save the matching
                 * airport (and runway, if applicable) for later use.
                 */
                ndt_airport *firstapt = ndt_navdata_get_airport(flp->ndb, prefix);
                ndt_runway  *firstrwy;

                if (firstapt && suffix)
                {
                    firstrwy = ndt_runway_get(firstapt->runways, suffix);
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

                /* Set the initial source waypoint (SID, runway or airport). */
                src = (flp->dep.sid.enroute.rsgt ? flp->dep.sid.enroute.rsgt->dst :
                       flp->dep.sid.        rsgt ? flp->dep.sid.        rsgt->dst :
                       flp->dep.rwy              ? flp->dep.rwy->waypoint         : flp->dep.apt->waypoint);

                init = 1;//done
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
                if (lastpl == NULL || strcmp(lastpl->info.idnt, place))
                {
                    lastpos = src ? src->position : lastpl ? lastpl->position : flp->dep.apt->coordinates;
                    lastpl  = ndt_navdata_get_wptnear2(flp->ndb, place, NULL, lastpos);
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
            else if (flp->dep.apt && ndt_procedure_get(flp->dep.apt->sids, prefix, NULL))
            {
                if (flp->dep.sid.proc)
                {
                    ndt_log("[fmt_icaor]: warning: ignoring SID '%s'\n", elem);
                }
                else
                {
                    if ((err = ndt_flightplan_set_departsid(flp, prefix, suffix)))
                    {
                        goto end;
                    }
                    src = flp->dep.sid.enroute.rsgt ? flp->dep.sid.enroute.rsgt->dst : flp->dep.sid.rsgt->dst;
                }
                continue;
            }
            else if (flp->dep.apt && ndt_procedure_get(flp->dep.apt->sids, suffix, NULL))
            {
                if (flp->dep.sid.proc)
                {
                    ndt_log("[fmt_icaor]: warning: ignoring SID '%s'\n", elem);
                }
                else
                {
                    if ((err = ndt_flightplan_set_departsid(flp, suffix, prefix)))
                    {
                        goto end;
                    }
                    src = flp->dep.sid.enroute.rsgt ? flp->dep.sid.enroute.rsgt->dst : flp->dep.sid.rsgt->dst;
                }
                continue;
            }
            else if (flp->arr.apt && ndt_procedure_get(flp->arr.apt->stars, prefix, NULL))
            {
                if ((err = ndt_flightplan_set_arrivstar(flp, prefix, suffix)))
                {
                    goto end;
                }
                continue;
            }
            else if (flp->arr.apt && ndt_procedure_get(flp->arr.apt->stars, suffix, NULL))
            {
                if ((err = ndt_flightplan_set_arrivstar(flp, suffix, prefix)))
                {
                    goto end;
                }
                continue;
            }
            else if (strlen(prefix) == 4 && !strncmp(prefix, "NAT", 3))
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

                if (!src)
                {
                    /*
                     * No startpoint, this can't be an airway; check
                     * whether it's a waypoint before erroring out.
                     */
                    if (!ndt_navdata_get_waypoint(flp->ndb, prefix, NULL))
                    {
                        ndt_log("[fmt_icaor]: no startpoint for airway '%s'\n", elem);
                        err = EINVAL;
                        goto end;
                    }
                    dstidt = prefix;
                }
                else if (awy1id)
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
                    lastpos = src ? src->position : lastpl ? lastpl->position : flp->dep.apt->coordinates;
                    dst     = ndt_navdata_get_wptnear2(flp->ndb, dstidt, NULL, lastpos);

                    /*
                     * If the last waypoint is an airport, save the matching
                     * airport (and runway, if applicable) for later use.
                     */
                    if (dst && dst->type == NDT_WPTYPE_APT)
                    {
                        lastapt = ndt_navdata_get_airport(flp->ndb, dst->info.idnt);
                        if (lastapt && suffix)
                        {
                            lastrwy = ndt_runway_get(lastapt->runways, suffix);
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

                if (flp->arr.star.proc && dst != flp->arr.apt->waypoint)
                {
                    ndt_log("[fmt_icaor]: unexpected waypoint '%s' after STAR\n", dst->info.idnt);
                    err = EINVAL;
                    goto end;
                }

                if (awy && in && out)
                {
                    rsg = ndt_route_segment_airway(src, dst, awy, in, out, flp->ndb);
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

                /* Remove pointless legs */
                if (rsg->type == NDT_RSTYPE_DCT)
                {
                    if (!fv && ((flp->dep.apt && flp->dep.apt->waypoint == rsg->dst) ||
                                (flp->dep.rwy && flp->dep.rwy->waypoint == rsg->dst)))
                    {
                        ndt_route_segment_close(&rsg);
                        continue;
                    }
                    else if (rsg->src == rsg->dst)
                    {
                        ndt_route_segment_close(&rsg);
                        continue;
                    }
                    else if (rsg->src)
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
                fv = 1; // valid waypoint, don't skip additional airports/runways

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
    ndt_route_segment *pen = ndt_list_item(flp->rte, -2);
    ndt_route_segment *ult = ndt_list_item(flp->rte, -1);
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

    // all flightplan legs
    if ((ret = icao_printlg(fd, flp->legs, NDT_FLTPFMT_DCDED)))
    {
        goto end;
    }

end:
    if (ret)
    {
        return ret;
    }
    return ndt_fprintf(fd, "%s", "\n");
}

int ndt_fmt_dtest_flightplan_write(ndt_flightplan *flp, FILE *fd)
{
    int ret = 0;

    if (!flp || !fd)
    {
        ret = ENOMEM;
        goto end;
    }

    if (!flp->dep.apt || !flp->arr.apt)
    {
        ndt_log("[fmt_dtest]: departure or arrival airport not set\n");
        ret = EINVAL;
        goto end;
    }

    if (flp->dep.rwy)
    {
        if ((ret = icao_printwp(fd, flp->dep.rwy->waypoint, NDT_LLCFMT_SVECT, NDT_FLTPFMT_DTEST)) ||
            (ret = ndt_fprintf (fd, "%s", " ")))
        {
            goto end;
        }
    }

    // all flightplan legs
    if ((ret = icao_printlg(fd, flp->legs, NDT_FLTPFMT_DTEST)))
    {
        goto end;
    }

    if (flp->arr.rwy)
    {
        if ((ret = ndt_fprintf (fd, "%s", " ")) ||
            (ret = icao_printwp(fd, flp->arr.rwy->waypoint, NDT_LLCFMT_SVECT, NDT_FLTPFMT_DTEST)))
        {
            goto end;
        }
    }

end:
    if (ret)
    {
        return ret;
    }
    return ndt_fprintf(fd, "%s", "\n");
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
    if ((ret = ndt_fprintf(fd, "%s SID", flp->dep.apt->info.idnt)))
    {
        goto end;
    }

    // SID endpoint, if it's a fix
    if (flp->dep.sid.enroute.rsgt)
    {
        if (flp->dep.sid.enroute.rsgt->dst)
        {
            if ((ret = ndt_fprintf(fd, " %s", flp->dep.sid.enroute.rsgt->dst->info.idnt)))
            {
                goto end;
            }
        }
    }
    else if (flp->dep.sid.rsgt)
    {
        if (flp->dep.sid.rsgt->dst)
        {
            if ((ret = ndt_fprintf(fd, " %s", flp->dep.sid.rsgt->dst->info.idnt)))
            {
                goto end;
            }
        }
    }

    // encoded route
    if ((ret = ndt_fprintf(fd, "%s", " ")))
    {
        goto end;
    }
    if (ndt_list_count(flp->rte))
    {
        if ((ret = icao_printrt(fd, flp->rte, NDT_FLTPFMT_ICAOR)))
        {
            goto end;
        }
    }
    else
    {
        if ((ret = ndt_fprintf(fd, "%s", "DCT")))
        {
            goto end;
        }
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

int ndt_fmt_icaox_flightplan_write(ndt_flightplan *flp, FILE *fd)
{
    int ret = 0;

    if (!flp || !fd)
    {
        ret = ENOMEM;
        goto end;
    }

    if (!flp->dep.apt || !flp->arr.apt)
    {
        ndt_log("[fmt_icaox]: departure or arrival airport not set\n");
        ret = EINVAL;
        goto end;
    }

    // departure airport
    if ((ret = ndt_fprintf(fd, "%s SID", flp->dep.apt->info.idnt)))
    {
        goto end;
    }

    // applicable route elements
    size_t num_route_legs = ndt_list_count(flp->rte);
    ndt_waypoint *sid_dst = NULL;
    if (flp->dep.sid.rsgt)
    {
        sid_dst = flp->dep.sid.rsgt->dst;
    }
    if (flp->dep.sid.enroute.rsgt)
    {
        sid_dst = flp->dep.sid.enroute.rsgt->dst;
    }
    ndt_waypoint *arr_src = NULL;
    if (flp->arr.apch.transition.rsgt)
    {
        // final appraoches use "custom" fixes whose names aren't portable
        // across navigation data providers (Aerosoft vs. Navigraph); however,
        // approach transitions normally start at a regular/official named fix.
        ndt_route_leg *leg = ndt_list_item(flp->arr.apch.transition.rsgt->legs, 0);
        if (leg)
        {
            arr_src = leg->dst;
        }
    }
    if (flp->arr.star.rsgt)
    {
        ndt_route_leg *leg = ndt_list_item(flp->arr.star.rsgt->legs, 0);
        if (leg)
        {
            arr_src = leg->dst;
        }
    }
    if (flp->arr.star.enroute.rsgt)
    {
        ndt_route_leg *leg = ndt_list_item(flp->arr.star.enroute.rsgt->legs, 0);
        if (leg)
        {
            arr_src = leg->dst;
        }
    }

    // validate and write the flightplan
    if (num_route_legs == 0 && sid_dst == NULL && arr_src == NULL)
    {
        ndt_log("[fmt_icaox]: empty flightplan (unsupported)\n");
        ret = EINVAL;
        goto end;
    }
    if (sid_dst)
    {
        // SID endpoint, if it's a fix
        if ((ret = ndt_fprintf(fd, " %s", sid_dst->info.idnt)))
        {
            goto end;
        }
    }
    if (num_route_legs)
    {
        // mandatory "via" field except for first waypoint
        if (sid_dst && (ret = ndt_fprintf(fd, "%s", " DCT")))
        {
            goto end;
        }
        // encoded route
        if ((ret = ndt_fprintf(fd, "%s", " ")))
        {
            goto end;
        }
        if ((ret = icao_printrt(fd, flp->rte, NDT_FLTPFMT_ICAOX)))
        {
            goto end;
        }
    }
    else if (arr_src)
    {
        // mandatory "via" field except for first waypoint
        if ((sid_dst || num_route_legs) && (ret = ndt_fprintf(fd, "%s", " DCT")))
        {
            goto end;
        }
        // STAR or approach entry point, if it's an applicable fix
        if ((ret = ndt_fprintf(fd, " %s", arr_src->info.idnt)))
        {
            goto end;
        }
    }

    // arrival airport
    if ((ret = ndt_fprintf(fd, " STAR %s\n", flp->arr.apt->info.idnt)))
    {
        goto end;
    }

end:
    return ret;
}

static int fmt_irecp_print_leg(FILE *fd, ndt_route_leg *leg)
{
    if (!fd || !leg)
    {
        return ENOMEM;
    }
    switch (leg->type)
    {
        case NDT_LEGTYPE_ZZ:
            return ndt_fprintf(fd, "\n%s\n", "        ----F-PLN DISCONTINUITY----");

        default:
        {
            int rvalue;
            char *idt1, *idt2, sbrif[13];
            switch (leg->rsg->type)
            {
                case NDT_RSTYPE_DCT:
                {
                    if (leg->dst->type == NDT_WPTYPE_LLC)
                    {
                        if (ndt_position_sprintllc(leg->dst->position, NDT_LLCFMT_SBRIF,
                                                   sbrif, sizeof(sbrif)) < 0)
                        {
                            return EIO;
                        }
                        idt2 = sbrif;
                    }
                    else
                    {
                        idt2 = leg->dst->info.idnt;
                    }
                    idt1 = "DCT";
                    break;
                }
                case NDT_RSTYPE_AWY:
                    idt1 = leg->awyleg->awy->info.idnt;
                    idt2 = leg->dst->        info.idnt;
                    break;
                default:
                    idt1 = leg->rsg->info.idnt;
                    idt2 = leg->     info.idnt;
                    break;
            }
            if (leg->src && leg->dst && leg->src != leg->dst && !ndt_list_count(leg->xpfms))
            {
                double dist = ndt_distance_get(leg->dis, NDT_ALTUNIT_ME) / 1852.;
                if ((rvalue = ndt_fprintf(fd, "\n%-16s  %05.1lf° (%05.1lf°T) %5.1lf nm\n",
                                          idt1, leg->omb, leg->trb, dist)))
                {
                    return rvalue;
                }
            }
            else if ((rvalue = ndt_fprintf(fd, "\n%s\n", idt1)))
            {
                // future: print magnetic course if we have it
                return rvalue;
            }
            if ((*leg->info.misc) &&
                (rvalue = ndt_fprintf(fd, "%s\n", leg->info.misc)))
            {
                return rvalue;
            }
            if (leg->dst)
            {
                char recap[24];
                if (ndt_position_sprintllc(leg->dst->position, NDT_LLCFMT_RECAP,
                                           recap, sizeof(recap)) < 0)
                {
                    return EIO;
                }
                if ((*leg->dst->info.misc) &&
                    (rvalue = ndt_fprintf(fd, "%s\n", leg->dst->info.misc)))
                {
                    return rvalue;
                }
                if ((rvalue = ndt_fprintf(fd, "%-21s  %s\n", idt2, recap)))
                {
                    return rvalue;
                }
            }
            else if ((rvalue = ndt_fprintf(fd, "%s\n", idt2)))
            {
                return rvalue;
            }
            return 0;
        }
    }
}

static const char* surfacetype_name(ndt_runway *rwy)
{
    if (rwy)
    {
        switch (rwy->surface)
        {
            case NDT_RWYSURF_ASPHT:
                return   "asphalt";
            case NDT_RWYSURF_CONCR:
                return  "concrete";
            case NDT_RWYSURF_GRASS:
                return     "grass";
            case NDT_RWYSURF_GRAVL:
                return    "gravel";
            case NDT_RWYSURF_WATER:
                return     "water";
            case NDT_RWYSURF_OTHER:
            default:
                return   "unknown";
        }
    }
    return NULL;
}

static const char* navaid_type_name(ndt_runway *rwy)
{
    if (rwy)
    {
        // KLAX, KJFK: ILS but slight mismatch (up to 3°)
        if ((rwy->ils.course >= rwy->heading && rwy->ils.course - rwy->heading <= 3) ||
            (rwy->heading >= rwy->ils.course && rwy->heading - rwy->ils.course <= 3))
        {
            return rwy->ils.slope > 0. ? "ILS" : "LOC";
        }
        else
        {
            return rwy->ils.slope > 0. ? "IGS" : "LDA";
        }
    }
    return NULL;
}

int ndt_fmt_irecp_flightplan_write(ndt_flightplan *flp, FILE *fd)
{
    char sbrif[13], recap[24];
    int ret = 0, d01, d02;
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

    // departure airport/runway, SID, enroute, STAR, approach, arrival airport/runway
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
        if (flp->dep.rwy->ils.avail)
        {
            ret = ndt_fprintf(fd, "Runway:    %s (%d°), %d (%d) ft, surface: %s, %s: %.2lf (%d°, %.1lf°)\n",
                              flp->dep.rwy->info.idnt,
                              flp->dep.rwy->heading, d01, d02,
                              surfacetype_name (flp->dep.rwy),
                              navaid_type_name (flp->dep.rwy),
                              ndt_frequency_get(flp->dep.rwy->ils.freq),
                              flp->dep.rwy->ils.course,
                              flp->dep.rwy->ils.slope);
        }
        else
        {
            ret = ndt_fprintf(fd, "Runway:    %s (%d°), %d (%d) ft, surface: %s\n",
                              flp->dep.rwy->info.idnt,
                              flp->dep.rwy->heading, d01, d02,
                              surfacetype_name(flp->dep.rwy));
        }
        if (ret)
        {
            goto end;
        }
    }
    if (flp->dep.sid.proc)
    {
        if (flp->dep.sid.enroute.proc)
        {
            ret = ndt_fprintf(fd, "SID:       %s with transition: %s\n",
                              flp->dep.sid.        proc->info.idnt,
                              flp->dep.sid.enroute.proc->info.misc);
        }
        else
        {
            ret = ndt_fprintf(fd, "SID:       %s\n",
                              flp->dep.sid.proc->info.idnt);
        }
        if (ret)
        {
            goto end;
        }
    }
    if (ndt_list_count(flp->rte))
    {
        if ((ret = ndt_fprintf(fd,   "%s", "Enroute:   ")))
        {
            goto end;
        }
        if (flp->dep.sid.enroute.rsgt)
        {
            if ((flp->dep.sid.enroute.rsgt->dst) &&
                (ret = ndt_fprintf(fd, "%s ", flp->dep.sid.enroute.rsgt->dst->info.idnt)))
            {
                goto end;
            }
        }
        else if (flp->dep.sid.rsgt)
        {
            if ((flp->dep.sid.rsgt->dst) &&
                (ret = ndt_fprintf(fd, "%s ", flp->dep.sid.rsgt->dst->info.idnt)))
            {
                goto end;
            }
        }
        if ((ret = icao_printrt(fd, flp->rte, NDT_FLTPFMT_ICAOR)))
        {
            goto end;
        }
    }
    else
    {
        if ((ret = ndt_fprintf(fd, "%s", "Enroute:   DCT")))
        {
            goto end;
        }
    }
    if ((ret = ndt_fprintf(fd, "%s", "\n")))
    {
        goto end;
    }
    if (flp->arr.star.proc)
    {
        if (flp->arr.star.enroute.proc)
        {
            ret = ndt_fprintf(fd, "STAR:      %s with transition: %s\n",
                              flp->arr.star.        proc->info.idnt,
                              flp->arr.star.enroute.proc->info.misc);
        }
        else
        {
            ret = ndt_fprintf(fd, "STAR:      %s\n",
                              flp->arr.star.proc->info.idnt);
        }
        if (ret)
        {
            goto end;
        }
    }
    d01 = ndt_distance_get(flp->arr.apt->coordinates.altitude, NDT_ALTUNIT_FT);
    d02 = ndt_distance_get(flp->arr.apt->trans_level,          NDT_ALTUNIT_FT);
    if (d02)
    {
        ret = ndt_fprintf(fd, "Arrival:   %s (%s), elevation %d ft, transition level FL%d\n",
                          flp->arr.apt->info.idnt,
                          flp->arr.apt->info.misc, d01, d02 / 100);
    }
    else
    {
        ret = ndt_fprintf(fd, "Arrival:   %s (%s), elevation %d ft, transition level ATC\n",
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
        if (flp->arr.rwy->ils.avail)
        {
            ret = ndt_fprintf(fd, "Runway:    %s (%d°), %d (%d) ft, surface: %s, %s: %.2lf (%d°, %.1lf°)\n",
                              flp->arr.rwy->info.idnt,
                              flp->arr.rwy->heading, d01, d02,
                              surfacetype_name (flp->arr.rwy),
                              navaid_type_name (flp->arr.rwy),
                              ndt_frequency_get(flp->arr.rwy->ils.freq),
                              flp->arr.rwy->ils.course,
                              flp->arr.rwy->ils.slope);
        }
        else
        {
            ret = ndt_fprintf(fd, "Runway:    %s (%d°), %d (%d) ft, surface: %s\n",
                              flp->arr.rwy->info.idnt,
                              flp->arr.rwy->heading, d01, d02,
                              surfacetype_name(flp->arr.rwy));
        }
        if (ret)
        {
            goto end;
        }
    }
    if (flp->arr.apch.proc)
    {
        if (flp->arr.apch.transition.proc)
        {
            ret = ndt_fprintf(fd, "Approach:  %s with transition: %s\n",
                              flp->arr.apch.           proc->info.idnt,
                              flp->arr.apch.transition.proc->info.misc);
        }
        else
        {
            ret = ndt_fprintf(fd, "Approach:  %s\n",
                              flp->arr.apch.proc->info.idnt);
        }
        if (ret)
        {
            goto end;
        }
    }

    // all flightplan legs
    if (ndt_list_count(flp->legs) == 0)
    {
        ret = ndt_fprintf(fd, "\n%s", "Flight route: DIRECT\n");
    }
    else
    {
        ret = ndt_fprintf(fd, "\n%s", "Flight route:\n");
    }
    if (ret)
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
        if ((ret = fmt_irecp_print_leg(fd, leg)))
        {
            goto end;
        }
    }

    // dummy final leg to runway threshold
    if ((ret = fmt_irecp_print_leg(fd, flp->arr.last.rleg)))
    {
        goto end;
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

    // SID and transition, if present
    if (flp->dep.sid.proc)
    {
        if ((ret = ndt_fprintf(fd, "%s", flp->dep.sid.proc->info.idnt)))
        {
            goto end;
        }
        if (flp->dep.sid.enroute.proc)
        {
            if ((ret = ndt_fprintf(fd, ".%s", flp->dep.sid.enroute.proc->info.misc)))
            {
                goto end;
            }
        }
    }

    // SID endpoint, if it's a fix
    if (flp->dep.sid.enroute.rsgt)
    {
        if ((flp->dep.sid.enroute.rsgt->dst) &&
            (ret = ndt_fprintf(fd, " %s", flp->dep.sid.enroute.rsgt->dst->info.idnt)))
        {
            goto end;
        }
    }
    else if (flp->dep.sid.rsgt)
    {
        if ((flp->dep.sid.rsgt->dst) &&
            (ret = ndt_fprintf(fd, " %s", flp->dep.sid.rsgt->dst->info.idnt)))
        {
            goto end;
        }
    }

    // encoded route
    if (flp->dep.sid.enroute.proc || flp->dep.sid.proc)
    {
        if ((ret = ndt_fprintf(fd, "%s", " ")))
        {
            goto end;
        }
    }
    if (ndt_list_count(flp->rte))
    {
        if ((ret = icao_printrt(fd, flp->rte, NDT_FLTPFMT_SBRIF)))
        {
            goto end;
        }
    }
    else
    {
        if ((ret = ndt_fprintf(fd, "%s", "DCT")))
        {
            goto end;
        }
    }

    // STAR and transition, if present
    if (flp->arr.star.proc)
    {
        if ((ret = ndt_fprintf(fd, "%s", " ")))
        {
            goto end;
        }
        if (flp->arr.star.enroute.proc)
        {
            if ((ret = ndt_fprintf(fd, "%s.", flp->arr.star.enroute.proc->info.misc)))
            {
                goto end;
            }
        }
        if ((ret = ndt_fprintf(fd, "%s", flp->arr.star.proc->info.idnt)))
        {
            goto end;
        }
    }

    // we're done!
    if ((ret = ndt_fprintf(fd, "%s", "\n")))
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

        // some formats don't support latitude/longitude custom waypoints
        // some databases, however, have named fixes for the 5-letter form
        case NDT_FLTPFMT_ICAOX:
            llcfmt = NDT_LLCFMT_DEFS5;
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

        if (i && (ret = ndt_fprintf(fd, "%s", " ")))
        {
            goto end;
        }
        switch (rsg->type)
        {
            case NDT_RSTYPE_AWY:
                ret = ndt_fprintf(fd, "%s %s", rsg->awy.awy->info.idnt, rsg->dst->info.idnt);
                break;

            case NDT_RSTYPE_DCT:
                if (i && fmt == NDT_FLTPFMT_ICAOX)
                {   // mandatory "via" field except for first waypoint
                    if ((ret = ndt_fprintf(fd, "%s", "DCT ")))
                    {
                        goto end;
                    }
                }
                ret = icao_printwp(fd, rsg->dst, llcfmt, fmt);
                break;

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
    }

end:
    return ret;
}

static int icao_printwp(FILE *fd, ndt_waypoint *dst, ndt_llcfmt llcfmt, ndt_fltplanformat fmt)
{
    if (!fd || !dst)
    {
        return ENOMEM;
    }
    if (fmt == NDT_FLTPFMT_DTEST)
    {
        return ndt_position_fprintllc(dst->position, llcfmt, fd);
    }
    switch (dst->type)
    {
        case NDT_WPTYPE_APT:
        case NDT_WPTYPE_DME:
        case NDT_WPTYPE_FIX:
        case NDT_WPTYPE_NDB:
        case NDT_WPTYPE_VOR: // use the identifier
            return ndt_fprintf(fd, "%s", dst->info.idnt);

        case NDT_WPTYPE_PBD:
            if ((fmt == NDT_FLTPFMT_DCDED ||
                 fmt == NDT_FLTPFMT_ICAOR ||
                 fmt == NDT_FLTPFMT_SBRIF) &&
                (dst->pbd.place->type == NDT_WPTYPE_FIX ||
                 dst->pbd.place->type == NDT_WPTYPE_NDB ||
                 dst->pbd.place->type == NDT_WPTYPE_VOR))
            {
                double   nm = ndt_distance_get(dst->pbd.distance, NDT_ALTUNIT_ME) / 1852.;
                if (fabs(nm - round(nm)) < .05) // nm distance basically an integer, yay!
                {
                    return ndt_fprintf(fd, "%5s%03.0lf%03.0lf",
                                       dst->pbd.place->info.idnt,
                                       dst->pbd.bearing, round(nm));
                }
            }
        default: // use latitude/longitude coordinates
            return ndt_position_fprintllc(dst->position, llcfmt, fd);
    }
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
        case NDT_FLTPFMT_DTEST:
            llcfmt = NDT_LLCFMT_SVECT;
            break;

        default:
            ndt_log("[icao_printlg]: unsupported flight plan format '%d'\n", fmt);
            ret = EINVAL;
            goto end;
    }

    for (size_t i = 0, need_space = 0; i < ndt_list_count(lgs); i++)
    {
        ndt_route_leg *leg = ndt_list_item(lgs, i);
        if (!leg)
        {
            ret = ENOMEM;
            goto end;
        }

        if (need_space && (ret = ndt_fprintf(fd, "%s", " ")))
        {
            goto end;
        }
        else
        {
            need_space = 0;
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
                if (leg->dst && (leg->dst != leg->src || ndt_list_count(leg->xpfms)))
                {
                    for (size_t j = 0; j < ndt_list_count(leg->xpfms); j++)
                    {
                        if (j && (ret = ndt_fprintf(fd, "%s", " ")))
                        {
                            goto end;
                        }
                        if ((ret = icao_printwp(fd, ndt_list_item(leg->xpfms, j), llcfmt, fmt)))
                        {
                            goto end;
                        }
                    }
                    if (ndt_list_count(leg->xpfms) && (ret = ndt_fprintf(fd, "%s", " ")))
                    {
                        goto end;
                    }
                    if ((ret = icao_printwp(fd, leg->dst, llcfmt, fmt)))
                    {
                        goto end;
                    }
                    need_space = 1;
                }
                break;
            }

            default:
            {
                for (size_t j = 0; j < ndt_list_count(leg->xpfms); j++)
                {
                    if (j && (ret = ndt_fprintf(fd, "%s", " ")))
                    {
                        goto end;
                    }
                    if ((ret = icao_printwp(fd, ndt_list_item(leg->xpfms, j), llcfmt, fmt)))
                    {
                        goto end;
                    }
                    need_space = 1;
                }
                if (leg->dst)
                {
                    if (ndt_list_count(leg->xpfms) && (ret = ndt_fprintf(fd, "%s", " ")))
                    {
                        goto end;
                    }
                    if ((ret = icao_printwp(fd, leg->dst, llcfmt, fmt)))
                    {
                        goto end;
                    }
                    need_space = 1;
                }
                break;
            }
        }
    }

end:
    return ret;
}

int ndt_fmt_icaor_print_airportnfo(ndt_navdatabase *ndb, const char *icao)
{
    ndt_procedure *pr1, *pr2;
    ndt_route_leg       *leg;
    ndt_airport         *apt;
    ndt_runway          *rwy;
    ndt_list   *names = NULL;
    ndt_list   *trans = NULL;
    int     i, j, k, ret = 0;

    if (!(ndt_navdata_init_airport(ndb, (apt = ndt_navdata_get_airport(ndb, icao)))))
    {
        fprintf(stderr, "Airport %s not found\n", icao);
        ret = EINVAL;
        goto end;
    }

    i = ndt_distance_get(apt->coordinates.altitude, NDT_ALTUNIT_FT);
    j = ndt_distance_get(apt->tr_altitude,          NDT_ALTUNIT_FT);
    k = ndt_distance_get(apt->trans_level,          NDT_ALTUNIT_FT);
    if (j && k)
    {
        fprintf(stdout,
                "Airport: %s (%s), elevation: %d, transition: %d/FL%d\n",
                apt->info.idnt, apt->info.misc, i, j, k / 100);
    }
    else if (j)
    {
        fprintf(stdout,
                "Airport: %s (%s), elevation: %d, transition: %d/ATC\n",
                apt->info.idnt, apt->info.misc, i, j);
    }
    else if (k)
    {
        fprintf(stdout,
                "Airport: %s (%s), elevation: %d, transition: ATC/FL%d\n",
                apt->info.idnt, apt->info.misc, i, k / 100);
    }
    else
    {
        fprintf(stdout,
                "Airport: %s (%s), elevation: %d, transition: ATC\n",
                apt->info.idnt, apt->info.misc, i);
    }

    fprintf(stdout, "Runways:%s", "\n\n");
    for (i = 0; i < ndt_list_count(apt->runways); i++)
    {
        if ((rwy = ndt_list_item(apt->runways, i)))
        {
            j = ndt_distance_get(rwy->length, NDT_ALTUNIT_FT);
            k = ndt_distance_get(rwy->width,  NDT_ALTUNIT_FT);
            if (rwy->ils.avail)
            {
                fprintf(stdout, "    %-3s %03d°   Length: %5d, width: %3d, surface: %9s, %s: %.2lf (%03d°, %.1lf°)\n",
                        rwy->info.idnt,   rwy->heading, j, k,
                        surfacetype_name (rwy),
                        navaid_type_name (rwy),
                        ndt_frequency_get(rwy->ils.freq),
                        rwy->ils.course,  rwy->ils.slope);
            }
            else
            {
                fprintf(stdout, "    %-3s %03d°   Length: %5d, width: %3d, surface: %9s\n",
                        rwy->info.idnt, rwy->heading, j, k,
                        surfacetype_name(rwy));
            }
            for (j = 0; j < ndt_list_count(rwy->approaches); j++)
            {
                if ((pr1 = ndt_list_item(rwy->approaches, j)))
                {
                    leg = ndt_list_item(pr1->proclegs, 0);
                    fprintf(stdout, "        approach: %-11s from: %-5s",
                            pr1->info.idnt,
                            leg && leg->type == NDT_LEGTYPE_IF ? leg->dst->info.idnt :
                            leg && leg->src  != NULL           ? leg->src->info.idnt : "");
                    if (ndt_list_count(pr1->transition.approach))
                    {
                        fprintf(stdout, "%s", " with transition(s):");
                    }
                    for (k = 0; k < ndt_list_count(pr1->transition.approach); k++)
                    {
                        if ((pr2 = ndt_list_item(pr1->transition.approach, k)))
                        {
                            fprintf(stdout, " %s", pr2->info.misc);
                        }
                    }
                    fprintf(stdout, "%s", "\n");
                }
            }
            fprintf(stdout, "%s", "\n");
        }
    }

    if (!(names = ndt_list_init()) || !(trans = ndt_list_init()))
    {
        ret = ENOMEM;
        goto end;
    }
    ndt_procedure_names(apt->sids, names);
    if (ndt_list_count(names))
    {
        fprintf(stdout, "Standard Instrument Departures:%s", "\n\n");
        for (i = 0; i < ndt_list_count(names); i++)
        {
            fprintf(stdout, "    %s\n", (char*)ndt_list_item(names, i));
            fprintf(stdout, "    %s", "applicable to runways:");
            for (j = 0; j < ndt_list_count(apt->runways); j++)
            {
                if ((rwy = ndt_list_item(apt->runways, j)) &&
                    (ndt_procedure_get(apt->sids, ndt_list_item(names, i), rwy)))
                {
                    fprintf(stdout, " %s", rwy->info.idnt);
                }
            }
            fprintf(stdout, "%s", "\n");
            pr1 = ndt_procedure_get(apt->sids, ndt_list_item(names, i), NULL);
            if (pr1)
            {
                pr2 = pr1->transition.sid ? pr1->transition.sid : pr1;
                if ((leg = ndt_list_item(pr2->proclegs, -1)) && leg->dst)
                {
                    fprintf(stdout, "    procedure's final fix: %s\n", leg->dst->info.idnt);
                }
            }
            ndt_procedure_trans(pr1 ? pr1->transition.enroute : NULL, trans);
            if (ndt_list_count(trans))
            {
                fprintf(stdout, "    %s", "enroute transition(s):");
                for (j = 0; j < ndt_list_count(trans); j++)
                {
                    fprintf(stdout, " %s", (char*)ndt_list_item(trans, j));
                }
                fprintf(stdout, "%s", "\n");
            }
            fprintf(stdout, "%s", "\n");
        }
    }

    ndt_procedure_names(apt->stars, names);
    if (ndt_list_count(names))
    {
        fprintf(stdout, "Standard Terminal Arrival Routes:%s", "\n\n");
        for (i = 0; i < ndt_list_count(names); i++)
        {
            fprintf(stdout, "    %s\n", (char*)ndt_list_item(names, i));
            pr1 = ndt_procedure_get(apt->stars, ndt_list_item(names, i), NULL);
            ndt_procedure_trans(pr1 ? pr1->transition.enroute : NULL, trans);
            if (ndt_list_count(trans))
            {
                fprintf(stdout, "    %s", "enroute transition(s):");
                for (j = 0; j < ndt_list_count(trans); j++)
                {
                    fprintf(stdout, " %s", (char*)ndt_list_item(trans, j));
                }
                fprintf(stdout, "%s", "\n");
            }
            if (pr1)
            {
                pr2 = pr1->transition.star ? pr1->transition.star : pr1;
                if ((leg = ndt_list_item(pr2->proclegs, 0)) &&
                    (leg->src || leg->type == NDT_LEGTYPE_IF))
                {
                    fprintf(stdout, "    initial procedure fix: %s\n",
                            leg->src ? leg->src->info.idnt : leg->dst->info.idnt);
                }
            }
            fprintf(stdout, "    %s", "applicable to runways:");
            for (j = 0; j < ndt_list_count(apt->runways); j++)
            {
                if ((rwy = ndt_list_item(apt->runways, j)) &&
                    (ndt_procedure_get(apt->stars, ndt_list_item(names, i), rwy)))
                {
                    fprintf(stdout, " %s", rwy->info.idnt);
                }
            }
            fprintf(stdout, "%s", "\n\n");
        }
    }

end:
    ndt_list_close(&names);
    ndt_list_close(&trans);
    return ret;
}
