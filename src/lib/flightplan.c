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

#include "wmm/wmm.h"

#include "airway.h"
#include "flightplan.h"
#include "fmt_aibxt.h"
#include "fmt_icaor.h"
#include "fmt_xpfms.h"
#include "waypoint.h"

static int route_leg_update(ndt_flightplan *flp                                                        );
static int route_leg_airway(ndt_flightplan *flp, ndt_navdatabase *ndb, ndt_route_segment *rsg          );
static int route_leg_oftype(ndt_flightplan *flp,                       ndt_route_segment *rsg, int type);

ndt_flightplan* ndt_flightplan_init(ndt_navdatabase *ndb)
{
    if (!ndb)
    {
        return NULL;
    }

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

    flp->ndb = ndb;

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
            ndt_list_close(&flp->legs);
        }

        free(flp);

        *_flp = NULL;
    }
}

int ndt_flightplan_set_departure(ndt_flightplan *flp, const char *icao, const char *rwid)
{
    char         errbuf[64];
    int          err = 0;
    ndt_airport *apt;

    if (!flp || !icao)
    {
        err = ENOMEM;
        goto end;
    }

    if (!(apt = ndt_navdata_get_airport(flp->ndb, icao)))
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

int ndt_flightplan_set_arrival(ndt_flightplan *flp, const char *icao, const char *rwid)
{
    char         errbuf[64];
    int          err = 0;
    ndt_airport *apt;

    if (!flp || !icao)
    {
        err = ENOMEM;
        goto end;
    }

    if (!(apt = ndt_navdata_get_airport(flp->ndb, icao)))
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

int ndt_flightplan_set_route(ndt_flightplan *flp, const char *rte, ndt_fltplanformat fmt)
{
    int  err = 0;
    char errbuf[64];

    if (!flp || !rte || !(*rte))
    {
        err = ENOMEM;
        goto end;
    }

    switch (fmt)
    {
        case NDT_FLTPFMT_AIBXT:
            err = ndt_fmt_aibxt_flightplan_set_route(flp, rte);
            break;

        case NDT_FLTPFMT_ICAOR:
            err = ndt_fmt_icaor_flightplan_set_route(flp, rte);
            break;

        case NDT_FLTPFMT_XPFMS:
            err = ndt_fmt_xpfms_flightplan_set_route(flp, rte);
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
    return route_leg_update(flp);
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

        case NDT_FLTPFMT_IRECP:
            err = ndt_fmt_irecp_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_SBRIF:
            err = ndt_fmt_sbrif_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_XPCDU:
            err = ndt_fmt_xpfms_flightplan_write(flp, file, fmt);
            break;

        case NDT_FLTPFMT_XPCVA:
            err = ndt_fmt_xpfms_flightplan_write(flp, file, fmt);
            break;

        case NDT_FLTPFMT_XPHLP:
            err = ndt_fmt_xpfms_flightplan_write(flp, file, fmt);
            break;

        case NDT_FLTPFMT_XPFMS:
            err = ndt_fmt_xpfms_flightplan_write(flp, file, fmt);
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

ndt_route_segment* ndt_route_segment_init()
{
    ndt_route_segment *rsg = calloc(1, sizeof(ndt_route_segment));
    if (!rsg)
    {
        goto end;
    }

    rsg->legs = ndt_list_init();
    if (!rsg->legs)
    {
        goto end;
    }

    rsg->type = NDT_RSTYPE_DSC;

end:
    return rsg;
}

void ndt_route_segment_close(ndt_route_segment **_rsg)
{
    if (_rsg && *_rsg)
    {
        ndt_route_segment *rsg = *_rsg;
        size_t i;

        if (rsg->legs)
        {
            while ((i = ndt_list_count(rsg->legs)))
            {
                ndt_route_leg *leg = ndt_list_item(rsg->legs, i-1);
                ndt_list_rem                      (rsg->legs, leg);
                ndt_route_leg_close               (          &leg);
            }
            ndt_list_close(&rsg->legs);
        }

        free(rsg);

        *_rsg = NULL;
    }
}

ndt_route_segment* ndt_route_segment_airway(ndt_waypoint *src, ndt_waypoint *dst, ndt_airway *awy, ndt_airway_leg *in, ndt_airway_leg *out, ndt_navdatabase *ndb)
{
    ndt_route_segment *rsg = ndt_route_segment_init();
    if (!rsg)
    {
        goto fail;
    }

    if (!src || !dst || !awy || !in || !out || !ndb)
    {
        goto fail;
    }

    snprintf(rsg->info.idnt, sizeof(rsg->info.idnt), "%s %s", awy->info.idnt, dst->info.idnt);
    rsg->type    = NDT_RSTYPE_AWY;
    rsg->src     = src;
    rsg->dst     = dst;
    rsg->awy.awy = awy;
    rsg->awy.src =  in;
    rsg->awy.dst = out;

    while (in)
    {
        dst = ndt_navdata_get_wpt4pos(ndb, in->out.info.idnt, NULL, in->out.position);
        if (!dst)
        {
            // navdata bug
            ndt_log("ndt_route_segment_airway:"
                    " waypoint '%s/%+010.6lf/%+011.6lf' not found for airway '%s'\n",
                    in->out.info.idnt,
                    ndt_position_getlatitude (in->out.position, NDT_ANGUNIT_DEG),
                    ndt_position_getlongitude(in->out.position, NDT_ANGUNIT_DEG),
                    awy->info.idnt);
            goto fail;
        }

        ndt_route_leg *leg = ndt_route_leg_init();
        if (!leg)
        {
            goto fail;
        }


        ndt_date now = ndt_date_now();
        leg->dis     = ndt_position_calcdistance(src->position,   dst->position);
        leg->trb     = ndt_position_calcbearing (src->position,   dst->position);
        leg->imb     = ndt_wmm_getbearing_mag(ndb->wmm, leg->trb, dst->position, now);
        leg->omb     = ndt_wmm_getbearing_mag(ndb->wmm, leg->trb, src->position, now);
        leg->type    = NDT_LEGTYPE_TF;
        leg->src     = src;
        leg->dst     = dst;
        leg->rsg     = rsg;
        leg->awy.leg = in;
        ndt_list_add(rsg->legs, leg);

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

    return rsg;

fail:
    ndt_route_segment_close(&rsg);
    return NULL;
}

ndt_route_segment* ndt_route_segment_direct(ndt_waypoint *src, ndt_waypoint *dst, ndt_navdatabase *ndb)
{
    if (!src || !dst)
    {
        goto fail;
    }

    ndt_route_segment *rsg = ndt_route_segment_init();
    if (!rsg)
    {
        goto fail;
    }

    snprintf(rsg->info.idnt, sizeof(rsg->info.idnt), "DCT %s", dst->info.idnt);
    rsg->type = NDT_RSTYPE_DCT;
    rsg->src  = src;
    rsg->dst  = dst;

    ndt_route_leg *leg = ndt_route_leg_init();
    if (!leg)
    {
        goto fail;
    }

    if (src)
    {
        ndt_date now = ndt_date_now();
        leg->dis = ndt_position_calcdistance(src->position,   dst->position);
        leg->trb = ndt_position_calcbearing (src->position,   dst->position);
        leg->imb = ndt_wmm_getbearing_mag(ndb->wmm, leg->trb, dst->position, now);
        leg->omb = ndt_wmm_getbearing_mag(ndb->wmm, leg->trb, src->position, now);
    }
    leg->type = NDT_LEGTYPE_TF;
    leg->src  = src;
    leg->dst  = dst;
    leg->rsg  = rsg;
    ndt_list_add(rsg->legs, leg);

    return rsg;

fail:
    ndt_route_segment_close(&rsg);
    return NULL;
}

ndt_route_segment* ndt_route_segment_discon(ndt_waypoint *dst)
{
    if (!dst)
    {
        goto fail;
    }

    ndt_route_segment *rsg = ndt_route_segment_init();
    if (!rsg)
    {
        goto fail;
    }

    snprintf(rsg->info.idnt, sizeof(rsg->info.idnt), "-| %s", dst->info.idnt);
    rsg->type = NDT_RSTYPE_DSC;
    rsg->src  = NULL;
    rsg->dst  = dst;

    ndt_route_leg *leg = ndt_route_leg_init();
    if (!leg)
    {
        goto fail;
    }

    leg->type = NDT_LEGTYPE_ZZ;
    leg->dst  = dst;
    leg->rsg  = rsg;
    ndt_list_add(rsg->legs, leg);

    return rsg;

fail:
    ndt_route_segment_close(&rsg);
    return NULL;
}

ndt_route_leg* ndt_route_leg_init()
{
    ndt_route_leg *leg = calloc(1, sizeof(ndt_route_leg));
    if (!leg)
    {
        goto end;
    }

    leg->constraints.altitude.altmin =
    leg->constraints.altitude.altmax = ndt_distance_init(0, NDT_ALTUNIT_NA);
    leg->constraints.altitude.  type = NDT_RESTRICT_NO;
    leg->constraints.speed.   spdmin =
    leg->constraints.speed.   spdmax = ndt_airspeed_init(0, NDT_SPDUNIT_NAT);
    leg->constraints.speed.     type = NDT_RESTRICT_NO;
    leg->constraints.       waypoint = NDT_WPTCONST_NO;
    leg->type                        = NDT_LEGTYPE_ZZ;

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

int ndt_route_leg_restrict(ndt_route_leg *leg, ndt_restriction constraints)
{
    if (!leg)
    {
        return ENOMEM;
    }

    switch (constraints.altitude.type)
    {
        case NDT_RESTRICT_AB:
        case NDT_RESTRICT_AT:
        case NDT_RESTRICT_BL:
        case NDT_RESTRICT_BT:
        case NDT_RESTRICT_NO:
            leg->constraints.altitude.type = constraints.altitude.type;
            break;
        default:
            return EINVAL;
    }

    if (constraints.altitude.type == NDT_RESTRICT_AT ||
        constraints.altitude.type == NDT_RESTRICT_AB ||
        constraints.altitude.type == NDT_RESTRICT_BT)
    {
        if (ndt_distance_get(constraints.altitude.altmin, NDT_ALTUNIT_NA) <= 0LL)
        {
            return EINVAL;
        }
        leg->constraints.altitude.altmin = constraints.altitude.altmin;
    }

    if (constraints.altitude.type == NDT_RESTRICT_AT ||
        constraints.altitude.type == NDT_RESTRICT_BL ||
        constraints.altitude.type == NDT_RESTRICT_BT)
    {
        if (ndt_distance_get(constraints.altitude.altmax, NDT_ALTUNIT_NA) <= 0LL)
        {
            return EINVAL;
        }
        leg->constraints.altitude.altmax = constraints.altitude.altmax;
    }

    switch (constraints.speed.type)
    {
        case NDT_RESTRICT_AB:
        case NDT_RESTRICT_AT:
        case NDT_RESTRICT_BL:
        case NDT_RESTRICT_BT:
        case NDT_RESTRICT_NO:
            leg->constraints.speed.type = constraints.speed.type;
            break;
        default:
            return EINVAL;
    }

    if (constraints.speed.type == NDT_RESTRICT_AT ||
        constraints.speed.type == NDT_RESTRICT_AB ||
        constraints.speed.type == NDT_RESTRICT_BT)
    {
        if (ndt_airspeed_get(constraints.speed.spdmin, NDT_SPDUNIT_NAT, NDT_MACH_DEFAULT) <= 0LL)
        {
            return EINVAL;
        }
        leg->constraints.speed.spdmin = constraints.speed.spdmin;
    }

    if (constraints.speed.type == NDT_RESTRICT_AT ||
        constraints.speed.type == NDT_RESTRICT_BL ||
        constraints.speed.type == NDT_RESTRICT_BT)
    {
        if (ndt_airspeed_get(constraints.speed.spdmax, NDT_SPDUNIT_NAT, NDT_MACH_DEFAULT) <= 0LL)
        {
            return EINVAL;
        }
        leg->constraints.speed.spdmax = constraints.speed.spdmax;
    }

    switch (constraints.waypoint)
    {
        case NDT_WPTCONST_NO:
        case NDT_WPTCONST_FOV:
            leg->constraints.waypoint = constraints.waypoint;
            break;
        default:
            return EINVAL;
    }

    return 0;
}

static int route_leg_update(ndt_flightplan *flp)
{
    int err = 0;

    if (!flp)
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

        for (size_t j = 0; j < ndt_list_count(rsg->legs); j++)
        {
            ndt_route_leg *leg = ndt_list_item(rsg->legs, j);
            if (!leg)
            {
                err = ENOMEM;
                goto end;
            }

            ndt_list_add(flp->legs, leg);
        }
    }

end:
    return err;
}
