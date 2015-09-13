/*
 * flightplan.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/common.h"
#include "common/list.h"

#include "compat/compat.h"

#include "airway.h"
#include "flightplan.h"
#include "fmt_aibxt.h"
#include "fmt_icaor.h"
#include "fmt_xpfms.h"
#include "waypoint.h"

static int route_leg_update(ndt_flightplan *flp, ndt_navdatabase *ndb                                  );
static int route_leg_airway(ndt_flightplan *flp, ndt_navdatabase *ndb, ndt_route_segment *rsg          );
static int route_leg_oftype(ndt_flightplan *flp,                       ndt_route_segment *rsg, int type);

ndt_flightplan* ndt_flightplan_init()
{
    ndt_flightplan *flp = calloc(1, sizeof(ndt_flightplan));
    if (!flp)
    {
        goto end;
    }

    flp->cws = ndt_list_init();
    if (!flp->cws)
    {
        ndt_flightplan_close(&flp);
        goto end;
    }

    flp->rte = ndt_list_init();
    if (!flp->rte)
    {
        ndt_flightplan_close(&flp);
        goto end;
    }

    flp->legs = ndt_list_init();
    if (!flp->legs)
    {
        ndt_flightplan_close(&flp);
        goto end;
    }

    flp->crz_altitude = ndt_distance_init(0, NDT_ALTUNIT_NA);

end:
    return flp;
}

void ndt_flightplan_close(ndt_flightplan **_flp)
{
    if (_flp && *_flp)
    {
        size_t i;
        ndt_flightplan *flp = *_flp;

        if (flp->cws)
        {
            while ((i = ndt_list_count(flp->cws)))
            {
                ndt_waypoint *wpt = ndt_list_item(flp->cws, i-1);
                ndt_list_rem                     (flp->cws, wpt);
                ndt_waypoint_close               (         &wpt);
            }
            ndt_list_close(&flp->cws);
        }

        if (flp->rte)
        {
            while ((i = ndt_list_count(flp->rte)))
            {
                ndt_route_segment *rsg = ndt_list_item(flp->rte, i-1);
                ndt_list_rem                          (flp->rte, rsg);
                ndt_route_segment_close               (         &rsg);
            }
            ndt_list_close(&flp->rte);
        }

        if (flp->legs)
        {
            while ((i = ndt_list_count(flp->legs)))
            {
                ndt_route_leg *leg = ndt_list_item(flp->legs, i-1);
                ndt_list_rem                      (flp->legs, leg);
                ndt_route_leg_close               (          &leg);
            }
            ndt_list_close(&flp->legs);
        }

        free(flp);

        *_flp = NULL;
    }
}

int ndt_flightplan_set_departure(ndt_flightplan *flp, ndt_navdatabase *ndb, const char *icao, const char *rwid)
{
    char         errbuf[64];
    int          err = 0;
    ndt_airport *apt;

    if (!flp || !ndb || !icao)
    {
        err = ENOMEM;
        goto end;
    }

    if (!(apt = ndt_navdata_get_airport(ndb, icao)))
    {
        err = EINVAL;
        goto end;
    }

    flp->dep.apt = apt;

    if (rwid)
    {
        ndt_runway *rwy = ndt_runway_get(apt, rwid);

        if (!rwy)
        {
            err = EINVAL;
            goto end;
        }

        flp->dep.rwy = rwy;
    }

end:
    if (err)
    {
        strerror_r(err, errbuf, sizeof(errbuf));
        ndt_log("flightplan: failed to set departure [%s%s%s] (%s)\n",
                icao ? icao : "",
                rwid ? ", " : "",
                rwid ? rwid : "", errbuf);
    }
    return err;
}

int ndt_flightplan_set_arrival(ndt_flightplan *flp, ndt_navdatabase *ndb, const char *icao, const char *rwid)
{
    char         errbuf[64];
    int          err = 0;
    ndt_airport *apt;

    if (!flp || !ndb || !icao)
    {
        err = ENOMEM;
        goto end;
    }

    if (!(apt = ndt_navdata_get_airport(ndb, icao)))
    {
        err = EINVAL;
        goto end;
    }

    flp->arr.apt = apt;

    if (rwid)
    {
        ndt_runway *rwy = ndt_runway_get(apt, rwid);

        if (!rwy)
        {
            err = EINVAL;
            goto end;
        }

        flp->arr.rwy = rwy;
    }

end:
    if (err)
    {
        strerror_r(err, errbuf, sizeof(errbuf));
        ndt_log("flightplan: failed to set arrival [%s%s%s] (%s)\n",
                icao ? icao : "",
                rwid ? ", " : "",
                rwid ? rwid : "", errbuf);
    }
    return err;
}

int ndt_flightplan_set_route(ndt_flightplan *flp, ndt_navdatabase *ndb, const char *rte, ndt_fltplanformat fmt)
{
    int  err = 0;
    char errbuf[64];

    if (!flp || !ndb || !rte || !(*rte))
    {
        err = ENOMEM;
        goto end;
    }

    switch (fmt)
    {
        case NDT_FLTPFMT_AIBXT:
            err = ndt_fmt_aibxt_flightplan_set_route(flp, ndb, rte);
            break;

        case NDT_FLTPFMT_ICAOR:
            err = ndt_fmt_icaor_flightplan_set_route(flp, ndb, rte);
            break;

        case NDT_FLTPFMT_XPFMS:
            err = ndt_fmt_xpfms_flightplan_set_route(flp, ndb, rte);
            break;

        default:
            err = -1;
            break;
    }

end:
    if (err)
    {
        strerror_r(err, errbuf, sizeof(errbuf));
        ndt_log("flightplan: failed to parse route (%s)\n", errbuf);
        return err;
    }
    return route_leg_update(flp, ndb);
}

int ndt_flightplan_write(ndt_flightplan *flp, FILE *file, ndt_fltplanformat fmt)
{
    int  err = 0;
    char errbuf[64];

    if (!flp || !file)
    {
        err = ENOMEM;
        goto end;
    }

    switch (fmt)
    {
        case NDT_FLTPFMT_AIBXT:
            err = ndt_fmt_aibxt_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_DCDED:
            err = ndt_fmt_dcded_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_ICAOR:
            err = ndt_fmt_icaor_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_CEEVA:
            err = ndt_fmt_ceeva_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_SBRIF:
            err = ndt_fmt_sbrif_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_XPFMS:
            err = ndt_fmt_xpfms_flightplan_write(flp, file);
            break;

        default:
            err = -1;
            break;
    }

end:
    if (err)
    {
        strerror_r(err, errbuf, sizeof(errbuf));
        ndt_log("flightplan: failed to write file (%s)\n", errbuf);
    }
    return err;
}

ndt_route_leg* ndt_route_leg_init()
{
    ndt_route_leg *leg = calloc(1, sizeof(ndt_route_leg));
    if (!leg)
    {
        goto end;
    }

    leg->constraints.altitude.typ = NDT_RESTRICT_NO;
    leg->constraints.altitude.alt = ndt_distance_init(0, NDT_ALTUNIT_NA);
    leg->constraints.speed.   typ = NDT_RESTRICT_NO;
    leg->constraints.speed.   spd = ndt_airspeed_init(0, NDT_SPDUNIT_NAT);
    leg->constraints.    waypoint = NDT_WPTCONST_NO;
    leg->type                     = NDT_LEGTYPE_ZZ;

end:
    return leg;
}

void ndt_route_leg_close(ndt_route_leg **_leg)
{
    if (_leg && *_leg)
    {
        ndt_route_leg *leg = *_leg;

        free(leg);

        *_leg = NULL;
    }
}

ndt_route_segment* ndt_route_segment_init()
{
    ndt_route_segment *rsg = calloc(1, sizeof(ndt_route_segment));
    if (!rsg)
    {
        goto end;
    }

    rsg->constraints.altitude.typ = NDT_RESTRICT_NO;
    rsg->constraints.altitude.alt = ndt_distance_init(0, NDT_ALTUNIT_NA);
    rsg->constraints.speed.   typ = NDT_RESTRICT_NO;
    rsg->constraints.speed.   spd = ndt_airspeed_init(0, NDT_SPDUNIT_NAT);
    rsg->constraints.    waypoint = NDT_WPTCONST_NO;
    rsg->type                     = NDT_RSTYPE_DSC;

end:
    return rsg;
}

void ndt_route_segment_close(ndt_route_segment **_rsg)
{
    if (_rsg && *_rsg)
    {
        ndt_route_segment *rsg = *_rsg;

        free(rsg);

        *_rsg = NULL;
    }
}

ndt_route_segment* ndt_route_segment_airway(ndt_waypoint *src, ndt_waypoint *dst, ndt_airway *awy, ndt_airway_leg *in, ndt_airway_leg *out)
{
    ndt_route_segment *rsg = ndt_route_segment_init();
    if (!rsg)
    {
        goto end;
    }

    if (!src || !dst || !awy || !in || !out)
    {
        ndt_route_segment_close(&rsg);
        goto end;
    }

    snprintf(rsg->info.idnt, sizeof(rsg->info.idnt), "%s %s", awy->info.idnt, dst->info.idnt);
    rsg->type    = NDT_RSTYPE_AWY;
    rsg->src     = src;
    rsg->dst     = dst;
    rsg->data[0] = awy;
    rsg->data[1] =  in;
    rsg->data[2] = out;

end:
    return rsg;
}

ndt_route_segment* ndt_route_segment_direct(ndt_waypoint *src, ndt_waypoint *dst)
{
    ndt_route_segment *rsg = ndt_route_segment_init();
    if (!rsg)
    {
        goto end;
    }

    if (!src || !dst)
    {
        ndt_route_segment_close(&rsg);
        goto end;
    }

    snprintf(rsg->info.idnt, sizeof(rsg->info.idnt), "DCT %s", dst->info.idnt);
    rsg->type = NDT_RSTYPE_DCT;
    rsg->src  = src;
    rsg->dst  = dst;

end:
    return rsg;
}

ndt_route_segment* ndt_route_segment_discon(ndt_waypoint *dst)
{
    ndt_route_segment *rsg = ndt_route_segment_init();
    if (!rsg)
    {
        goto end;
    }

    if (!dst)
    {
        ndt_route_segment_close(&rsg);
        goto end;
    }

    snprintf(rsg->info.idnt, sizeof(rsg->info.idnt), "-| %s", dst->info.idnt);
    rsg->type = NDT_RSTYPE_DSC;
    rsg->src  = NULL;
    rsg->dst  = dst;

end:
    return rsg;
}

static int route_leg_update(ndt_flightplan *flp, ndt_navdatabase *ndb)
{
    int err = 0;

    if (!flp || !ndb)
    {
        err = ENOMEM;
        goto end;
    }

    for (size_t i = 0; i < ndt_list_count(flp->rte); i++)
    {
        ndt_route_segment *rsg = ndt_list_item(flp->rte, i);
        if (!rsg)
        {
            err = ENOMEM;
            goto end;
        }

        switch (rsg->type)
        {
            case NDT_RSTYPE_AWY:
                err = route_leg_airway(flp, ndb, rsg);
                break;

            case NDT_RSTYPE_DCT:
                err = route_leg_oftype(flp, rsg, NDT_LEGTYPE_TF);
                break;

            case NDT_RSTYPE_DSC:
                err = route_leg_oftype(flp, rsg, NDT_LEGTYPE_ZZ);
                break;

            default:
                err = EINVAL;
                goto end;
        }
        if (err)
        {
            goto end;
        }
    }

end:
    return err;
}

static int route_leg_airway(ndt_flightplan *flp, ndt_navdatabase *ndb, ndt_route_segment *rsg)
{
    int err = 0;

    if (!flp || !ndb || !rsg)
    {
        err = ENOMEM;
        goto end;
    }

    ndt_waypoint   *src = rsg->src;
    ndt_airway     *awy = rsg->data[0];
    ndt_airway_leg *in  = rsg->data[1];
    ndt_airway_leg *out = rsg->data[2];

    if (!src || !awy || !in || !out)
    {
        err = EINVAL;
        goto end;
    }

    while (in)
    {
        ndt_waypoint *dst = ndt_navdata_get_wpt4pos(ndb, in->out.info.idnt, NULL, in->out.position);
        if (!dst)
        {
            // navdata bug
            ndt_log("Waypoint '%s' @ '%+010.6lf/%+011.6lf' not found for airway '%s'\n",
                    in->out.info.idnt,
                    ndt_position_getlatitude (in->out.position, NDT_ANGUNIT_DEG),
                    ndt_position_getlongitude(in->out.position, NDT_ANGUNIT_DEG),
                    awy->info.idnt);
            err = -1;
            goto end;
        }

        ndt_route_leg *leg = ndt_route_leg_init();
        if (!leg)
        {
            err = ENOMEM;
            goto end;
        }

        leg->brg         = ndt_position_calcbearing(src->position, dst->position);
        leg->type        = NDT_LEGTYPE_ZA;
        leg->awy         = awy;
        leg->src         = src;
        leg->dst         = dst;
        ndt_list_add(flp->legs, leg);

        if (in == out)
        {
            break;
        }
        else
        {
            src = dst;
            in  = in->next;
        }
    }

end:
    return err;
}

static int route_leg_oftype(ndt_flightplan *flp, ndt_route_segment *rsg, int type)
{
    int err = 0;

    if (!flp || !rsg)
    {
        err = ENOMEM;
        goto end;
    }

    ndt_route_leg *leg = ndt_route_leg_init();
    if (!leg)
    {
        err = ENOMEM;
        goto end;
    }

    if (rsg->src && rsg->dst)
    {
        leg->brg = ndt_position_calcbearing(rsg->src->position, rsg->dst->position);
    }
    leg->constraints = rsg->constraints;
    leg->src         = rsg->src;
    leg->dst         = rsg->dst;
    leg->type        = type;
    ndt_list_add(flp->legs, leg);

end:
    return err;
}
