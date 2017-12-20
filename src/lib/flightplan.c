/*
 * flightplan.c
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

#include "wmm/wmm.h"

#include "airway.h"
#include "flightplan.h"
#include "fmt_aibxt.h"
#include "fmt_icaor.h"
#include "fmt_xpfms.h"
#include "ndb_xpgns.h"
#include "waypoint.h"

static int route_leg_update(ndt_flightplan *flp                                              );
static int route_leg_airway(ndt_flightplan *flp, ndt_navdatabase *ndb, ndt_route_segment *rsg);

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

    // default cruising altitude to remain compatible
    // with most procedures' restrictions if possible
    flp->crz_altitude = ndt_distance_init(33000, NDT_ALTUNIT_FT);

end:
    return flp;
}

static void reset_sid(ndt_flightplan *flp)
{
    if (flp)
    {
        if (flp->dep.sid.rsgt)
        {
            ndt_route_segment_close(&flp->dep.sid.rsgt);
        }
        if (flp->dep.sid.enroute.rsgt)
        {
            ndt_route_segment_close(&flp->dep.sid.enroute.rsgt);
        }
        flp->dep.sid.        proc = NULL;
        flp->dep.sid.enroute.proc = NULL;
    }
}

static void reset_star(ndt_flightplan *flp)
{
    if (flp)
    {
        if (flp->arr.star.rsgt)
        {
            ndt_route_segment_close(&flp->arr.star.rsgt);
        }
        if (flp->arr.star.enroute.rsgt)
        {
            ndt_route_segment_close(&flp->arr.star.enroute.rsgt);
        }
        flp->arr.star.        proc = NULL;
        flp->arr.star.enroute.proc = NULL;
    }
}

static void reset_apch(ndt_flightplan *flp)
{
    if (flp)
    {
        if (flp->arr.apch.rsgt)
        {
            ndt_route_segment_close(&flp->arr.apch.rsgt);
        }
        if (flp->arr.apch.transition.rsgt)
        {
            ndt_route_segment_close(&flp->arr.apch.transition.rsgt);
        }
        flp->arr.apch.           proc = NULL;
        flp->arr.apch.transition.proc = NULL;
    }
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
        if (flp->arr.last.rsgt)
        {
            ndt_route_segment_close(&flp->arr.last.rsgt);
        }
        reset_sid (flp);
        reset_star(flp);
        reset_apch(flp);

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

    if (!ndt_navdata_init_airport(flp->ndb, apt))
    {
        err = EINVAL;
        goto end;
    }

    if (flp->dep.apt && flp->dep.apt != apt)
    {
        flp->dep.rwy = NULL;
        reset_sid(flp);
    }
    flp->dep.apt = apt;

    if (rwid)
    {
        ndt_runway *rwy = ndt_runway_get(apt->runways, rwid);

        if (!rwy)
        {
            err = EINVAL;
            goto end;
        }

        if (flp->dep.rwy && flp->dep.rwy != rwy)
        {
            reset_sid(flp);
        }
        flp->dep.rwy = rwy;
    }

    // transition altitude
    int64_t tr_altitude = (ndt_distance_get(apt->tr_altitude, NDT_ALTUNIT_FT));
    int64_t trans_level = (ndt_distance_get(apt->trans_level, NDT_ALTUNIT_FT));
    flp->tra_altitude   = (tr_altitude ? apt->tr_altitude :
                           trans_level ? apt->trans_level :
                           ndt_distance_init(10000, NDT_ALTUNIT_FT));

end:
    if (err)
    {
        strerror_r(err, errbuf, sizeof(errbuf));
        ndt_log("flightplan: failed to set departure [%s%s%s] (%s)\n",
                icao ? icao : "",
                rwid ? ", " : "",
                rwid ? rwid : "", errbuf);
        return err;
    }
    return route_leg_update(flp);
}

int ndt_flightplan_set_departsid(ndt_flightplan *flp, const char *name, const char *trans)
{
    ndt_procedure *proc, *nrte = NULL;
    char errbuf[64];
    int  err = 0;

    if (!flp || !name)
    {
        err = ENOMEM;
        goto end;
    }
    if (!flp->dep.apt)
    {
        ndt_log("flightplan: departure airport not set!\n");
        err = EINVAL;
        goto end;
    }

    if (!(proc = ndt_procedure_get(flp->dep.apt->sids, name, flp->dep.rwy)))
    {
        if (flp->dep.rwy && ndt_procedure_get(flp->dep.apt->sids, name, NULL))
        {
            ndt_log("flightplan: %s: invalid SID '%s' for runway %s\n",
                    flp->dep.apt->info.idnt, name, flp->dep.rwy->info.idnt);
        }
        else
        {
            ndt_log("flightplan: %s: invalid SID '%s'\n",
                    flp->dep.apt->info.idnt, name);
        }
        err = EINVAL;
        goto end;
    }
    if (!flp->dep.rwy && (proc->type == NDT_PROCTYPE_SID_1 ||
                          proc->type == NDT_PROCTYPE_SID_4))
    {
        // most SIDs require a runway, except vector-based
        // ones such as KABQ's LARGO2 procedure (AIRAC 1511)
        // said SIDs are coded with type 2 or 5 with "ALL" transition,
        // and aren't auto-converted to type 1/4 by our navdata parser
        // so we allow them w/out runway, and error out for all other SIDs
        ndt_log("flightplan: %s: no runway set for SID '%s' (required)\n",
                flp->dep.apt->info.idnt, name);
        err = EINVAL;
        goto end;
    }
    if (trans && !(nrte = ndt_procedure_gettr(proc->transition.enroute, trans)))
    {
        ndt_log("flightplan: %s: invalid transition '%s' for %s (SID)\n",
                flp->dep.apt->info.idnt, trans, name);
        err = EINVAL;
        goto end;
    }

    // we only open a procedure right before we need it, for performance reasons
    if (!proc->opened && !ndt_procedure_open(flp->ndb, proc))
    {
        ndt_log("flightplan: %s: failed to open SID '%s'\n",
                flp->dep.apt->info.idnt, name);
        err = EINVAL;
        goto end;
    }
    if (nrte && !nrte->opened && !ndt_procedure_open(flp->ndb, nrte))
    {
        ndt_log("flightplan: %s: failed to open transition '%s' for SID '%s'\n",
                flp->dep.apt->info.idnt, trans, name);
        err = EINVAL;
        goto end;
    }

    // if the departure runway is not set, we set src to NULL, so
    // ndt_route_segment_proced will insert a discontinuity for us
    ndt_waypoint *src = flp->dep.rwy ? flp->dep.rwy->waypoint : NULL;
    flp->dep.sid.rsgt = ndt_route_segment_proced(src, NULL, proc, flp->ndb);
    flp->dep.sid.proc = proc;
    if (!flp->dep.sid.rsgt)
    {
        err = ENOMEM;
        goto end;
    }
    if (nrte)
    {
        ndt_route_leg *leg = ndt_list_item(flp->dep.sid.rsgt->legs, -1);
        ndt_restriction *c = leg ? &leg->constraints : NULL;
        flp->dep.sid.enroute.rsgt = ndt_route_segment_proced(flp->dep.sid.rsgt->dst, c, nrte, flp->ndb);
        flp->dep.sid.enroute.proc = nrte;
        if (!flp->dep.sid.enroute.rsgt)
        {
            err = ENOMEM;
            goto end;
        }
        if (leg && c && (err = ndt_route_leg_restrict(leg, *c)))
        {
            goto end;
        }
    }

end:
    if (err)
    {
        strerror_r(err, errbuf, sizeof(errbuf));
        ndt_log("flightplan: failed to set SID [%s%s%s] (%s)\n",
                name  ?  name : "",
                trans ?   "." : "",
                trans ? trans : "", errbuf);
        return err;
    }
    return route_leg_update(flp);
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

    if (!ndt_navdata_init_airport(flp->ndb, apt))
    {
        err = EINVAL;
        goto end;
    }

    if (flp->arr.apt && flp->arr.apt != apt)
    {
        flp->arr.rwy = NULL;
        reset_star(flp);
        reset_apch(flp);
    }
    flp->arr.apt = apt;

    if (rwid)
    {
        ndt_runway *rwy = ndt_runway_get(apt->runways, rwid);

        if (!rwy)
        {
            err = EINVAL;
            goto end;
        }

        if (flp->arr.rwy && flp->arr.rwy != rwy)
        {
            reset_star(flp);
            reset_apch(flp);
        }
        flp->arr.rwy = rwy;
    }

    // transition level
    int64_t trans_level = (ndt_distance_get(apt->trans_level, NDT_ALTUNIT_FT));
    int64_t tr_altitude = (ndt_distance_get(apt->tr_altitude, NDT_ALTUNIT_FT));
    flp->trl_altitude   = (trans_level ? apt->trans_level :
                           tr_altitude ? apt->tr_altitude :
                           ndt_distance_init(10000, NDT_ALTUNIT_FT));

end:
    if (err)
    {
        strerror_r(err, errbuf, sizeof(errbuf));
        ndt_log("flightplan: failed to set arrival [%s%s%s] (%s)\n",
                icao ? icao : "",
                rwid ? ", " : "",
                rwid ? rwid : "", errbuf);
        return err;
    }
    return route_leg_update(flp);
}

int ndt_flightplan_set_arrivstar(ndt_flightplan *flp, const char *name, const char *trans)
{
    ndt_procedure *proc, *nrte = NULL;
    char errbuf[64];
    int  err = 0;

    if (!flp || !name)
    {
        err = ENOMEM;
        goto end;
    }
    if (!flp->arr.apt)
    {
        ndt_log("flightplan: arrival airport not set!\n");
        err = EINVAL;
        goto end;
    }

    if (!(proc = ndt_procedure_get(flp->arr.apt->stars, name, flp->arr.rwy)))
    {
        if (flp->arr.rwy && ndt_procedure_get(flp->arr.apt->stars, name, NULL))
        {
            ndt_log("flightplan: %s: invalid STAR '%s' for runway %s\n",
                    flp->arr.apt->info.idnt, name, flp->arr.rwy->info.idnt);
        }
        else
        {
            ndt_log("flightplan: %s: invalid STAR '%s'\n",
                    flp->arr.apt->info.idnt, name);
        }
        err = EINVAL;
        goto end;
    }
    if (!flp->arr.rwy && (proc->type == NDT_PROCTYPE_STAR3 ||
                          proc->type == NDT_PROCTYPE_STAR6 ||
                          proc->type == NDT_PROCTYPE_STAR9))
    {
        if (!proc->transition.star)
        {
            // runway-specific STAR without a runway-agnostic segment
            ndt_log("flightplan: %s: no runway set for STAR '%s' (required)\n",
                    flp->arr.apt->info.idnt, name);
            err = EINVAL;
            goto end;
        }
        proc = proc->transition.star; // fly the runway-agnostic segment only
    }
    if (trans && !(nrte = ndt_procedure_gettr(proc->transition.enroute, trans)))
    {
        ndt_log("flightplan: %s: invalid transition '%s' for %s (STAR)\n",
                flp->arr.apt->info.idnt, trans, name);
        err = EINVAL;
        goto end;
    }

    // we only open a procedure right before we need it, for performance reasons
    if (!proc->opened && !ndt_procedure_open(flp->ndb, proc))
    {
        ndt_log("flightplan: %s: failed to open STAR '%s'\n",
                flp->arr.apt->info.idnt, name);
        err = EINVAL;
        goto end;
    }
    if (nrte && !nrte->opened && !ndt_procedure_open(flp->ndb, nrte))
    {
        ndt_log("flightplan: %s: failed to open transition '%s' for STAR '%s'\n",
                flp->arr.apt->info.idnt, trans, name);
        err = EINVAL;
        goto end;
    }

    // src is the last flightplan leg's waypoint, the runway threshold or departure airport
    ndt_route_leg *leg = flp->dep.sid.rsgt ? ndt_list_item(flp->dep.sid.rsgt->legs, -1) : NULL;
    if (flp->dep.sid.enroute.rsgt)
    {
        leg = ndt_list_item(flp->dep.sid.enroute.rsgt->legs, -1);
    }
    if (ndt_list_count(flp->rte))
    {
        ndt_route_segment *rsg = ndt_list_item(flp->rte, -1);
        if (!rsg)
        {
            err = ENOMEM;
            goto end;
        }
        leg = ndt_list_item(rsg->legs, -1);
    }
    ndt_waypoint    *src = leg ?  leg->dst : flp->dep.rwy ? flp->dep.rwy->waypoint  : flp->dep.apt->waypoint;
    ndt_restriction *cst = leg ? &leg->constraints : NULL;
    if (nrte)
    {
        flp->arr.star.enroute.rsgt = ndt_route_segment_proced(src, cst, nrte, flp->ndb);
        flp->arr.star.enroute.proc = nrte;
        if (!flp->arr.star.enroute.rsgt)
        {
            err = ENOMEM;
            goto end;
        }
        if (leg && cst && (err = ndt_route_leg_restrict(leg, *cst)))
        {
            goto end;
        }
        src = flp->arr.star.enroute.rsgt->dst;
        leg = ndt_list_item(flp->arr.star.enroute.rsgt->legs, -1);
        cst = leg ? &leg->constraints : NULL;
    }
    flp->arr.star.rsgt = ndt_route_segment_proced(src, cst, proc, flp->ndb);
    flp->arr.star.proc = proc;
    if (!flp->arr.star.rsgt)
    {
        err = ENOMEM;
        goto end;
    }
    if (leg && cst && (err = ndt_route_leg_restrict(leg, *cst)))
    {
        goto end;
    }

end:
    if (err)
    {
        strerror_r(err, errbuf, sizeof(errbuf));
        ndt_log("flightplan: failed to set STAR [%s%s%s] (%s)\n",
                trans ? trans : "",
                trans ?   "." : "",
                name  ?  name : "", errbuf);
        return err;
    }
    return route_leg_update(flp);
}

int ndt_flightplan_set_arrivapch(ndt_flightplan *flp, const char *name, const char *trans)
{
    ndt_procedure *proc, *aptr = NULL;
    char errbuf[64];
    int  err = 0;

    if (!flp || !name)
    {
        err = ENOMEM;
        goto end;
    }
    if (!flp->arr.apt)
    {
        ndt_log("flightplan: arrival airport not set!\n");
        err = EINVAL;
        goto end;
    }
    if (!flp->arr.rwy)
    {
        ndt_log("flightplan: arrival runway not set!\n");
        err = EINVAL;
        goto end;
    }

    if (!(proc = ndt_procedure_get(flp->arr.rwy->approaches, name, NULL)))
    {
        ndt_log("flightplan: %s: invalid approach '%s' for runway %s\n",
                flp->arr.apt->info.idnt, name, flp->arr.rwy->info.idnt);
        err = EINVAL;
        goto end;
    }
    if (trans && !(aptr = ndt_procedure_gettr(proc->transition.approach, trans)))
    {
        ndt_log("flightplan: %s: invalid transition '%s' for %s (approach)\n",
                flp->arr.apt->info.idnt, trans, name);
        err = EINVAL;
        goto end;
    }

    // we only open a procedure right before we need it, for performance reasons
    if (!proc->opened && !ndt_procedure_open(flp->ndb, proc))
    {
        ndt_log("flightplan: %s: failed to open approach '%s'\n",
                flp->arr.apt->info.idnt, name);
        err = EINVAL;
        goto end;
    }
    if (aptr && !aptr->opened && !ndt_procedure_open(flp->ndb, aptr))
    {
        ndt_log("flightplan: %s: failed to open transition '%s' for approach '%s'\n",
                flp->arr.apt->info.idnt, trans, name);
        err = EINVAL;
        goto end;
    }

    // src is the last flightplan leg's waypoint, the runway threshold or departure airport
    ndt_route_leg *leg = flp->dep.sid.rsgt ? ndt_list_item(flp->dep.sid.rsgt->legs, -1) : NULL;
    if (flp->dep.sid.enroute.rsgt)
    {
        leg = ndt_list_item(flp->dep.sid.enroute.rsgt->legs, -1);
    }
    if (ndt_list_count(flp->rte))
    {
        ndt_route_segment *rsg = ndt_list_item(flp->rte, -1);
        if (!rsg)
        {
            err = ENOMEM;
            goto end;
        }
        leg = ndt_list_item(rsg->legs, -1);
    }
    if (flp->arr.star.enroute.rsgt)
    {
        leg = ndt_list_item(flp->arr.star.enroute.rsgt->legs, -1);
    }
    if (flp->arr.star.rsgt)
    {
        leg = ndt_list_item(flp->arr.star.rsgt->legs, -1);
    }
    ndt_waypoint    *src = leg ?  leg->dst : flp->dep.rwy ? flp->dep.rwy->waypoint  : flp->dep.apt->waypoint;
    ndt_restriction *cst = leg ? &leg->constraints : NULL;
    if (aptr)
    {
        flp->arr.apch.transition.rsgt = ndt_route_segment_proced(src, cst, aptr, flp->ndb);
        flp->arr.apch.transition.proc = aptr;
        if (!flp->arr.apch.transition.rsgt)
        {
            err = ENOMEM;
            goto end;
        }
        if (leg && cst && (err = ndt_route_leg_restrict(leg, *cst)))
        {
            goto end;
        }
        src = flp->arr.apch.transition.rsgt->dst;
        leg = ndt_list_item(flp->arr.apch.transition.rsgt->legs, -1);
        cst = leg ? &leg->constraints : NULL;
    }
    flp->arr.apch.rsgt = ndt_route_segment_proced(src, cst, proc, flp->ndb);
    flp->arr.apch.proc = proc;
    if (!flp->arr.apch.rsgt)
    {
        err = ENOMEM;
        goto end;
    }
    if (leg && cst && (err = ndt_route_leg_restrict(leg, *cst)))
    {
        goto end;
    }

end:
    if (err)
    {
        strerror_r(err, errbuf, sizeof(errbuf));
        ndt_log("flightplan: failed to set approach [%s%s%s] (%s)\n",
                trans ? trans : "",
                trans ?   "." : "",
                name  ?  name : "", errbuf);
        return err;
    }
    return route_leg_update(flp);
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

        case NDT_FLTPFMT_DTEST:
            err = ndt_fmt_dtest_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_ICAOR:
            err = ndt_fmt_icaor_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_ICAOX:
            err = ndt_fmt_icaox_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_IRECP:
            err = ndt_fmt_irecp_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_SBRIF:
            err = ndt_fmt_sbrif_flightplan_write(flp, file);
            break;

        case NDT_FLTPFMT_XPCDU:
        case NDT_FLTPFMT_XPCVA:
        case NDT_FLTPFMT_XPHLP:
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

static ndt_route_leg* route_leg_direct(ndt_waypoint *src, ndt_waypoint *dst)
{
    ndt_route_leg *leg = ndt_route_leg_init();
    if (leg && dst)
    {
        leg->type = NDT_LEGTYPE_TF;
        leg->src  = src;
        leg->dst  = dst;
    }
    return leg;
}

/*
 * "Airbus-style" (FPL page): split airway route segments so that we
 *                            have one route segment per airway leg.
 */
static int split_airways(ndt_flightplan *flp)
{
    ndt_route_segment *rsg, *new_rsg; ndt_route_leg *leg;
    for (int i = ndt_list_count(flp->rte) - 1; i >= 0; i--)
    {
        if ((rsg = ndt_list_item(flp->rte, i)) &&
            (rsg->type == NDT_RSTYPE_AWY && ndt_list_count(rsg->legs) > 1))
        {
            for (int j = ndt_list_count(rsg->legs) - 1; j >= 0; j--)
            {
                if ((leg = ndt_list_item(rsg->legs, j)))
                {
                    if ((new_rsg = ndt_route_segment_init()) == NULL)
                    {
                        return ENOMEM;
                    }
                    else
                    {
                        // note: faster than using ndt_route_segment_airway()
                        // also, we MUST preserve leg, otherwise we'll break
                        // on-the-fly insertion, removal of flight plan legs
                        snprintf(new_rsg->info.idnt, sizeof(new_rsg->info.idnt), "%s %s", leg->awyleg->awy->info.idnt, leg->dst->info.idnt);
                        new_rsg->awy.awy = leg->awyleg->awy;new_rsg->type = NDT_RSTYPE_AWY;ndt_list_add(new_rsg->legs, leg);leg->rsg = new_rsg;
                        new_rsg->awy.src = leg->awyleg;
                        new_rsg->awy.dst = leg->awyleg;
                        new_rsg->src     = leg->src;
                        new_rsg->dst     = leg->dst;
                    }
                    ndt_list_rem   (rsg->legs,       leg);
                    ndt_list_insert(flp->rte, new_rsg, i);
                }
            }
            if (ndt_list_count(rsg->legs))
            {
                ndt_log("navP [error]: split_airways, rsg has legs (BUG)!\n");
                return ENOMEM;
            }
            ndt_list_rem  (flp->rte, rsg);
            ndt_route_segment_close(&rsg);
        }
    }
    return 0;
}

/*
 * "Boeing-style" (RTE page): consolidate consecutive airway legs
 *                            into a single airway route segment.
 */
static int consolidate_airways(ndt_flightplan *flp)
{
    // note: beware of any airway segments where leg->src doesn't
    //       match the previous leg's dst: they've become directs
    return ENOSYS; // TODO: implement
}

void* ndt_flightplan_insert_airway(ndt_flightplan *flp, ndt_waypoint  *src, ndt_waypoint   *dst,
                                   ndt_airway     *awy, ndt_airway_leg *in, ndt_airway_leg *out,
                                   void           *from)
{
    int err; ndt_route_leg *ret, *leg = from;
    if (flp == NULL || src == NULL || dst == NULL ||
        awy == NULL || in  == NULL || out == NULL || from == NULL)
    {
        err = ENOMEM; goto end;
    }

    /*
     * First: split airway segments to facilitate insertion of legs mid-airway.
     */
    if ((err = split_airways(flp)))
    {
        goto end;
    }

    ndt_route_segment *rsg, *tmp;
    if ((rsg = leg->rsg) == NULL)
    {
        err = EINVAL; goto end;
    }

    for (int i = 0, ii = ndt_list_count(flp->rte); i < ii; i++)
    {
        if ((tmp = ndt_list_item(flp->rte, i)))
        {
            if (tmp == rsg)
            {
                if ((rsg = ndt_route_segment_airway(src, dst, awy, in, out, flp->ndb)) == NULL)
                {
                    err = ENOMEM; goto end;
                }
                if ((ret = ndt_list_item(rsg->legs, -1)) == NULL)
                {
                    ndt_route_segment_close(&rsg); err = ENOMEM; goto end;
                }
                else
                {
                    ndt_list_insert(flp->rte, rsg, i + 1);
                }
                if ((err = route_leg_update(flp)))
                {
                    while (ndt_list_count(rsg->legs))
                    {
                        if ((ret = ndt_list_item(rsg->legs, 0)))
                        {
                            ndt_list_rem(rsg->legs, ret);
                            ndt_route_leg_close   (&ret);
                        }
                    }
                    ndt_list_rem  (flp->rte, rsg);
                    ndt_route_segment_close(&rsg);
                    route_leg_update        (flp); goto end;
                }
                goto end;
            }
        }
    }
    err = ENOENT; goto end;

end:
    if (err)
    {
        char error[64];
        strerror_r(err, error, sizeof(error));
        ndt_log("ndt_flightplan_insert_airway: failure (%s)\n", error);
        return NULL;
    }
    return ret;
}

void* ndt_flightplan_insert_direct(ndt_flightplan *flp, ndt_waypoint *wpt, void *_leg, int insert_after)
{
    /*
     * Note: next_ fields currently unused, but may be usedful for inserting
     *       discontinuities down the road, so compute and save them anyway.
     */
    int insert_at_indx =-1, err = 0;
    ndt_waypoint      *prev_dst = NULL, *next_src = NULL;
    ndt_route_leg     *curr_leg = _leg; ndt_route_segment *prc[6];
    ndt_route_segment *prev_rsg = NULL, *next_rsg = NULL, *new_rsg = NULL, *rsg;
    ndt_route_leg     *prev_leg = NULL, *next_leg = NULL, *new_leg = NULL, *leg;

    /*
     * First: split airway segments to facilitate insertion of legs mid-airway.
     */
    if ((err = split_airways(flp)))
    {
        goto end;
    }

    /*
     * We must initialize this after splitting: curr_leg->rsg may have changed.
     */
    ndt_route_segment *curr_rsg = curr_leg ? curr_leg->rsg : NULL;

    /*
     * Check and save whether any given procedure segment is present:
     *
     * [0] runway   -> SID
     * [1] SID      -> enroute
     * [2] enroute  -> STAR
     * [3] STAR
     * [4] STAR     -> approach
     * [5] approach -> runway
     */
    prc[0] = flp->dep.sid. rsgt;
    prc[1] = flp->dep.sid. rsgt ? flp->dep.sid.    enroute.rsgt : NULL;
    prc[2] = flp->arr.star.rsgt ? flp->arr.star.   enroute.rsgt : NULL;
    prc[3] = flp->arr.star.rsgt;
    prc[4] = flp->arr.apch.rsgt ? flp->arr.apch.transition.rsgt : NULL;
    prc[5] = flp->arr.apch.rsgt;

    /*
     * No leg provided: insert_after has no meaning; append direct to main route
     */
    if (curr_leg == NULL)
    {
        if ((prev_rsg = ndt_list_item(flp->rte, -1)))
        {
            if ((prev_leg = ndt_list_item(prev_rsg->legs, -1)))
            {
                (prev_dst = prev_leg->dst);
            }
        }
        if ((new_rsg = ndt_route_segment_direct(prev_dst, wpt)) == NULL)
        {
            err = ENOMEM;
            goto end;
        }
        else if ((new_leg = ndt_list_item(new_rsg->legs, 0)) == NULL)
        {
            err = EINVAL;
            goto end;
        }
        ndt_list_add(flp->rte, new_rsg);
        goto end;
    }

    /*
     * Future: insert discontinuity if/where required when leg non-NULL
     */

    /*
     * Leg is to destination, append new leg to the last segment found
     */
    if (curr_leg == flp->arr.last.rleg)
    {
        if (insert_after) // future: prepend to the missed approach's leglist
        {
            err = ENOSYS; goto end;
        }
        // if we have a segment coming after main route, append new leg to it
        for (int i = 5; i >= 2; i--)
        {
            if ((prev_rsg = prc[i]))
            {
                if ((prev_leg = ndt_list_item(prev_rsg->legs, -1)))
                {
                    (prev_dst = prev_leg->dst);
                }
                if ((new_leg = route_leg_direct(prev_dst, wpt)) == NULL)
                {
                    err = ENOMEM;
                    goto end;
                }
                ndt_list_add(prev_rsg->legs, new_leg);
                goto end;
            }
        }
        // else, append new segment to main route, even if currently empty
        if ((prev_rsg = ndt_list_item(flp->rte, -1)))
        {
            if ((prev_leg = ndt_list_item(prev_rsg->legs, -1)))
            {
                (prev_dst = prev_leg->dst);
            }
        }
        if ((new_rsg = ndt_route_segment_direct(prev_dst, wpt)) == NULL)
        {
            err = ENOMEM;
            goto end;
        }
        else if ((new_leg = ndt_list_item(new_rsg->legs, 0)) == NULL)
        {
            err = EINVAL;
            goto end;
        }
        ndt_list_add(flp->rte, new_rsg);
        goto end;
    }

    /*
     * Locate the current segment and leg in their respective lists.
     * Determine the previous/next segment and leg based on findings.
     */
    if (curr_rsg == NULL)
    {
        err = EINVAL; goto end; // if curr_leg, curr_rsg should be non-NULL
    }
    if (curr_rsg->type == NDT_RSTYPE_MAP) // future
    {
        err = ENOSYS; goto end;
    }
    if (curr_rsg->type == NDT_RSTYPE_PRC)
    {
        for (int i = 0; i < 6; i++)
        {
            if (prc[i])
            {
                if (curr_rsg == prc[i])
                {
                    if (insert_after)
                    {
                        if (i >= 2 && ndt_list_count(flp->rte)) // arr. segment - previous one is enroute
                        {
                            if ((prev_rsg = ndt_list_item(flp->rte, -1)))
                            {
                                if ((prev_leg = ndt_list_item(prev_rsg->legs, -1)))
                                {
                                    (prev_dst = prev_leg->dst);
                                }
                            }
                        }
                    }
                    else
                    {
                        if (i <= 1 && ndt_list_count(flp->rte)) // dep. segment - the next one is enroute
                        {
                            if ((next_rsg = ndt_list_item(flp->rte, 0)))
                            {
                                if ((next_leg = ndt_list_item(next_rsg->legs, 0)))
                                {
                                    (next_src = next_leg->src);
                                }
                            }
                        }
                        if (next_leg == NULL) // next segment can be other procedure
                        {
                            for (int k = i + 1; k < 6; k++)
                            {
                                if ((next_rsg = prc[k]))
                                {
                                    if ((next_leg = ndt_list_item(next_rsg->legs, 0)))
                                    {
                                        (next_src = next_leg->src); break;
                                    }
                                }
                            }
                        }
                        if (next_leg == NULL) // next segment must be to destination
                        {
                            if ((next_rsg = flp->arr.last.rsgt))
                            {
                                if ((next_leg = flp->arr.last.rleg))
                                {
                                    (next_src = next_leg->src);
                                }
                            }
                        }
                    }
                    break;
                }
                if (insert_after == 0)
                {
                    if ((prev_rsg = prc[i])) // update previous segment on each turn
                    {
                        if ((prev_leg = ndt_list_item(prev_rsg->legs, -1)))
                        {
                            (prev_dst = prev_leg->dst);
                        }
                    }
                }
            }
        }
        for (int i = 0, j = ndt_list_count(curr_rsg->legs); i < j; i++)
        {
            if ((leg = ndt_list_item(curr_rsg->legs, i)))
            {
                if (curr_leg == leg)
                {
                    if (insert_after)
                    {
                        if ((i + 1) < j) // next leg is found in current segment
                        {
                            if ((next_leg = ndt_list_item(curr_rsg->legs, i + 1)))
                            {
                                (next_src = next_leg->src);
                            }
                        }
                    }
                    insert_at_indx = i + !!insert_after; break;
                }
                if (insert_after == 0)
                {
                    if ((prev_leg = leg)) // pr. leg is found in current segment
                    {
                        (prev_dst = prev_leg->dst);
                    }
                }
            }
        }
        if (insert_after)
        {
            if ((new_leg = route_leg_direct(curr_leg->dst, wpt)) == NULL)
            {
                err = ENOMEM;
                goto end;
            }
            ndt_list_insert(curr_rsg->legs, new_leg, insert_at_indx);
            goto end;
        }
        if ((new_leg = route_leg_direct(prev_dst, wpt)) == NULL)
        {
            err = ENOMEM;
            goto end;
        }
        ndt_list_insert(curr_rsg->legs, new_leg, insert_at_indx);
        goto end;
    }

    /* segment and leg not in a procedure: must be in main route */
    for (int i = 0, j = ndt_list_count(flp->rte); i < j; i++)
    {
        if ((rsg = ndt_list_item(flp->rte, i)))
        {
            if (curr_rsg == rsg)
            {
                if (insert_after)
                {
                    if ((i + 1) < j) // next segment still in main route
                    {
                        if ((next_rsg = ndt_list_item(flp->rte, i + 1)))
                        {
                            if ((next_leg = ndt_list_item(next_rsg->legs, 0)))
                            {
                                (next_src = next_leg->src);
                            }
                        }
                    }
                }
                insert_at_indx = i + !!insert_after; break;
            }
            if (insert_after == 0)
            {
                if ((prev_rsg = ndt_list_item(flp->rte, i))) // update previous segment on each turn
                {
                    if ((prev_leg = ndt_list_item(prev_rsg->legs, -1)))
                    {
                        (prev_dst = prev_leg->dst);
                    }
                }
            }
        }
    }
    if (prev_rsg == NULL)
    {
        for (int i = 1; i >= 0; i--) // previous segment is part of a SID
        {
            if ((prev_rsg = prc[i]))
            {
                if ((prev_leg = ndt_list_item(prev_rsg->legs, -1)))
                {
                    (prev_dst = prev_leg->dst); break;
                }
            }
        }
    }
    if (next_rsg == NULL)
    {
        for (int i = 2; i < 6; i++) // next segment is part of a STAR/approach
        {
            if ((next_rsg = prc[i]))
            {
                if ((next_leg = ndt_list_item(next_rsg->legs, 0)))
                {
                    (next_src = next_leg->src); break;
                }
            }
        }
    }
    if (insert_at_indx < 0)
    {
        ndt_log("navP [error]: ndt_flightplan_insert_direct has a BUG!");
        err = EINVAL;
        goto end;
    }
    if ((new_rsg = ndt_route_segment_direct(prev_dst, wpt)) == NULL)
    {
        err = ENOMEM;
        goto end;
    }
    else if ((new_leg = ndt_list_item(new_rsg->legs, 0)) == NULL)
    {
        err = EINVAL;
        goto end;
    }
    ndt_list_insert(flp->rte, new_rsg, insert_at_indx);
    goto end;

    /*
     * future: consolidate airways (one r.s. for all consecutive airway legs)
     * note:   required for Boeing-style RTE, not required at all for Airbus
     */

end:
    if (new_leg)
    {
        err = route_leg_update(flp);
    }
    else
    {
        err = ENOMEM;
    }
    if (err)
    {
        char error[64];
        strerror_r(err, error, sizeof(error));
        ndt_log("ndt_flightplan_insert_direct: failure (%s)\n", error);
        return NULL;
    }
    return new_leg;
}

int ndt_flightplan_remove_leg(ndt_flightplan *flp, void *_leg)
{
    int err = 0;
    if (flp == NULL || _leg == NULL)
    {
        err = ENOMEM; goto end;
    }

    /*
     * Comment of the month (May 2017): declare some variables.
     */
    ndt_route_leg *leg = _leg, *item, *last = NULL;
    ndt_route_segment *rsg, *dct, *tail;

    /*
     * First: split airway segments to facilitate removal of legs mid-airway.
     */
    if ((err = split_airways(flp)))
    {
        goto end;
    }

    /*
     * We must initialize this after splitting: leg->rsg may have changed.
     */
    if ((rsg = leg->rsg) == NULL)
    {
        err = EINVAL; goto end;
    }
    for (int i = 0, ii = ndt_list_count(rsg->legs); i < ii; i++)
    {
        if ((item = ndt_list_item(rsg->legs, i)))
        {
            if (leg == item)
            {
                // future: insert discontinuity
                // currnt: route_leg_update should be able to sanitize correctly
                ndt_list_rem(rsg->legs, leg);
                leg->rsg = NULL;
                goto cleanup;
            }
        }
        if (i == ii - 1) // last iteration, leg not found
        {
            err = EINVAL; goto end;
        }
    }

cleanup: // remove empty route segments
    for (int i = 0, ii = ndt_list_count(flp->rte); i < ii; i++)
    {
        if ((rsg = ndt_list_item(flp->rte, i)))
        {
            if (ndt_list_count(rsg->legs) == 0)
            {
                if (rsg->type == NDT_RSTYPE_PRC)
                {
                    if (rsg == flp->dep.sid.rsgt)
                    {
                        ndt_route_segment_close(&flp->dep.sid.rsgt);            goto end;
                    }
                    if (rsg == flp->dep.sid.enroute.rsgt)
                    {
                        ndt_route_segment_close(&flp->dep.sid.enroute.rsgt);    goto end;
                    }
                    if (rsg == flp->arr.star.enroute.rsgt)
                    {
                        ndt_route_segment_close(&flp->arr.star.enroute.rsgt);   goto end;
                    }
                    if (rsg == flp->arr.star.rsgt)
                    {
                        ndt_route_segment_close(&flp->arr.star.rsgt);           goto end;
                    }
                    if (rsg == flp->arr.apch.transition.rsgt)
                    {
                        ndt_route_segment_close(&flp->arr.apch.transition.rsgt);goto end;
                    }
                    if (rsg == flp->arr.apch.rsgt)
                    {
                        ndt_route_segment_close(&flp->arr.apch.rsgt);           goto end;
                    }
                    err = EINVAL; goto end;
                }
                {
                    // TODO: missed approach?
                }
                ndt_list_rem  (flp->rte, rsg);
                ndt_route_segment_close(&rsg);
                goto end;
            }
        }
    }

end:
    if (err)
    {
        char error[64];
        strerror_r(err, error, sizeof(error));
        ndt_log("ndt_flightplan_remove_leg: failure (%s)\n", error);
        ndt_log("FUCK MY LIFE\n");//debug
        return err;
    }
    ndt_log("WE ARE HERE\n");//debug
    return route_leg_update(flp);
}

#define NDTPROCINITLIST(l) { l = ndt_list_init(); if (!l) goto fail; }

ndt_procedure* ndt_procedure_init(enum ndt_procedure_type typ)
{
    ndt_procedure *proc = calloc(1, sizeof(ndt_procedure));
    if (!proc)
    {
        goto fail;
    }
    proc->type = typ;

    switch (typ)
    {
        case NDT_PROCTYPE_SID_1: // runway->SID
        case NDT_PROCTYPE_SID_4: // runway->SID
        case NDT_PROCTYPE_SID_2: // SID
        case NDT_PROCTYPE_SID_5: // SID
        case NDT_PROCTYPE_STAR2: // STAR
        case NDT_PROCTYPE_STAR5: // STAR
        case NDT_PROCTYPE_STAR8: // STAR
        case NDT_PROCTYPE_STAR3: // STAR->runway
        case NDT_PROCTYPE_STAR6: // STAR->runway
        case NDT_PROCTYPE_STAR9: // STAR->runway
            NDTPROCINITLIST(proc->runways);
            NDTPROCINITLIST(proc->transition.enroute);
            break;

        case NDT_PROCTYPE_SID_3: // SID->enroute
        case NDT_PROCTYPE_SID_6: // SID->enroute
            break;

        case NDT_PROCTYPE_STAR1: // enroute->STAR
        case NDT_PROCTYPE_STAR4: // enroute->STAR
        case NDT_PROCTYPE_STAR7: // enroute->STAR
            break;

        case NDT_PROCTYPE_APPTR:
            NDTPROCINITLIST(proc->runways);
            break;

        case NDT_PROCTYPE_FINAL:
            NDTPROCINITLIST(proc->runways);
            NDTPROCINITLIST(proc->mapplegs);
            NDTPROCINITLIST(proc->transition.approach);
            break;

        default:
            ndt_log("ndt_procedure_init: unknown type '%d'\n", typ);
            goto fail;
    }
    NDTPROCINITLIST(proc->proclegs);
    NDTPROCINITLIST(proc->custwpts);

    return proc;

fail:
    ndt_procedure_close(&proc);
    return NULL;
}

ndt_procedure* ndt_procedure_open(ndt_navdatabase *ndb, ndt_procedure *proc)
{
    if (proc == NULL || !ndb)
    {
        return  NULL;
    }
    if (proc->opened)
    {
        return proc;
    }
    switch (ndb->fmt)
    {
        case NDT_NAVDFMT_XPGNS:
            return ndt_ndb_xpgns_navdata_open_procdre(ndb, proc);

        case NDT_NAVDFMT_OTHER:
        default:
            return NULL;
    }
}

void ndt_procedure_close(ndt_procedure **ptr)
{
    ndt_procedure *proc = *ptr;
    size_t i;

    if (proc)
    {
        if (proc->proclegs)
        {
            while ((i = ndt_list_count(proc->proclegs)))
            {
                ndt_route_leg *leg = ndt_list_item(proc->proclegs,  -1);
                ndt_list_rem                      (proc->proclegs, leg);
                ndt_route_leg_close               (               &leg);
            }
            ndt_list_close(&proc->proclegs);
        }
        if (proc->mapplegs)
        {
            while ((i = ndt_list_count(proc->mapplegs)))
            {
                ndt_route_leg *leg = ndt_list_item(proc->mapplegs,  -1);
                ndt_list_rem                      (proc->mapplegs, leg);
                ndt_route_leg_close               (               &leg);
            }
            ndt_list_close(&proc->mapplegs);
        }
        if (proc->runways)
        {
            ndt_list_close(&proc->runways);
        }
        if (proc->custwpts)
        {
            while ((i = ndt_list_count(proc->custwpts)))
            {
                ndt_waypoint *wpt = ndt_list_item(proc->custwpts,  -1);
                ndt_list_rem                     (proc->custwpts, wpt);
                ndt_waypoint_close               (               &wpt);
            }
            ndt_list_close(&proc->custwpts);
        }
        if (proc->transition.approach)
        {
            ndt_list_close(&proc->transition.approach);
        }
        if (proc->transition.enroute)
        {
            ndt_list_close(&proc->transition.enroute);
        }
        free(proc);
    }

    *ptr = NULL;
}

ndt_procedure* ndt_procedure_get(ndt_list *procedures,  const char *name, ndt_runway *runway)
{
    if (procedures && name)
    {
        for (size_t i = 0; i < ndt_list_count(procedures); i++)
        {
            ndt_procedure *proc = ndt_list_item(procedures, i);
            if ((proc && strcmp(proc->info.idnt, name) == 0) &&
                (runway == NULL || runway == ndt_runway_get(proc->runways, runway->info.idnt)))
            {
                return proc;
            }
        }
    }
    return NULL;
}

ndt_procedure* ndt_procedure_gettr(ndt_list *transitions, const char *name)
{
    if (transitions && name)
    {
        for (size_t i = 0; i < ndt_list_count(transitions); i++)
        {
            ndt_procedure *proc = ndt_list_item(transitions, i);
            if (proc && strcmp(proc->info.misc, name) == 0)
            {
                return proc;
            }
        }
    }
    return NULL;
}

void ndt_procedure_names(ndt_list *procedures, ndt_list *output)
{
    ndt_procedure *p1, *p2;
    size_t i, j, index = 0;
    if (!procedures || !output)
    {
        return;
    }
    if (ndt_list_count(output))
    {
        ndt_list_empty(output);
    }
    for (i = 0; i < ndt_list_count(procedures); i++)
    {
        if ((p1 = ndt_list_item(procedures, i)))
        {
            for (j = 0; j < ndt_list_count(output); j++)
            {
                if ((p2 = ndt_list_item(output, j)))
                {
                    if (!strcmp(p2->info.idnt, p1->info.idnt))
                    {
                        j = -1; break; // already in list, mark it
                    }
                }
            }
            if (j != -1)
            {
                ndt_list_add(output, p1->info.idnt);
            }
        }
    }
}

void ndt_procedure_trans(ndt_list *transitions, ndt_list *output)
{
    ndt_procedure *p1, *p2;
    size_t i, j, index = 0;
    if (!transitions || !output)
    {
        return;
    }
    if (ndt_list_count(output))
    {
        ndt_list_empty(output);
    }
    for (i = 0; i < ndt_list_count(transitions); i++)
    {
        if ((p1 = ndt_list_item(transitions,  i)))
        {
            for (j = 0; j < ndt_list_count(output); j++)
            {
                if ((p2 = ndt_list_item(output, j)))
                {
                    if (!strcmp(p2->info.misc, p1->info.misc))
                    {
                        j = -1; break; // already in list, mark it
                    }
                }
            }
            if (j != -1)
            {
                ndt_list_add(output, p1->info.misc);
            }
        }
    }
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

    // note: we must update split_airways too should we update this function
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
        // note: we must update split_airways too should we update this function
        ndt_date now = ndt_date_now();
        leg->dis     = ndt_position_calcdistance(src->position,   dst->position);
        leg->trb     = ndt_position_calcbearing (src->position,   dst->position);
        leg->imb     = ndt_wmm_getbearing_mag(ndb->wmm, leg->trb, dst->position, now);
        leg->omb     = ndt_wmm_getbearing_mag(ndb->wmm, leg->trb, src->position, now);
        leg->type    = NDT_LEGTYPE_TF;
        leg->src     = src;
        leg->dst     = dst;
        leg->rsg     = rsg;
        leg->awyleg  = in;
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

ndt_route_segment* ndt_route_segment_direct(ndt_waypoint *src, ndt_waypoint *dst)
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

    snprintf(rsg->info.idnt, sizeof(rsg->info.idnt), "DCT %s", dst->info.idnt);
    rsg->type = NDT_RSTYPE_DCT;
    rsg->src  = src;
    rsg->dst  = dst;

    /*
     * TODO: discard pointless legs, for example:
     *
     * if (rsg->src == rsg->dst)
     * {
     *      ndt_route_segment_close(&rsg);
     *      continue;
     *  }
     *  else if (rsg->src)
     *  {
     *      ndt_position a = rsg->src->position;
     *      ndt_position b = rsg->dst->position;
     *      ndt_distance d = ndt_position_calcdistance(a, b);
     *      if (ndt_distance_get(d, NDT_ALTUNIT_NA) == 0) // please improve this
     *      {
     *          ndt_route_segment_close(&rsg);
     *          continue;
     *      }
     *  }
     */

    ndt_route_leg *leg = route_leg_direct(src, dst);
    if (!leg)
    {
        goto fail;
    }
    leg->rsg = rsg;
    ndt_list_add(rsg->legs, leg);

    return rsg;

fail:
    ndt_route_segment_close(&rsg);
    return NULL;
}

static ndt_route_leg* route_leg_discon(void)
{
    ndt_route_leg *leg = ndt_route_leg_init();
    if (leg)
    {
        leg->type = NDT_LEGTYPE_ZZ;
        leg->src  = NULL;
        leg->dst  = NULL;
    }
    return leg;
}

ndt_route_segment* ndt_route_segment_discon(void)
{
    ndt_route_segment *rsg = ndt_route_segment_init();
    if (!rsg)
    {
        goto fail;
    }

    snprintf(rsg->info.idnt, sizeof(rsg->info.idnt), "%s", "----F-PLN DISCONTINUITY---");
    rsg->type = NDT_RSTYPE_DSC;
    rsg->src  = NULL;
    rsg->dst  = NULL;

    ndt_route_leg *leg = route_leg_discon();
    if (!leg)
    {
        goto fail;
    }
    leg->rsg  = rsg;
    ndt_list_add(rsg->legs, leg);

    return rsg;

fail:
    ndt_route_segment_close(&rsg);
    return NULL;
}

static ndt_route_leg* route_leg4procedure(ndt_route_leg *leg, ndt_waypoint *src)
{
    ndt_route_leg *copy = ndt_route_leg_init();
    if (!copy || !leg)
    {
        goto fail;
    }

    memcpy(copy, leg, sizeof(ndt_route_leg));

    /* Holds can be returned "as is" */
    if (copy->type == NDT_LEGTYPE_HF ||
        copy->type == NDT_LEGTYPE_HA || copy->type == NDT_LEGTYPE_HM)
    {
        goto end;
    }

    /* Some leg types as basically "direct to" */
    if (copy->type == NDT_LEGTYPE_IF || copy->type == NDT_LEGTYPE_TF)
    {
        copy->src = src;
        goto end;
    }

    /* Other leg types may require dummy waypoints for navigation */
    if (!(copy->xpfms = ndt_list_init()))
    {
        goto fail;
    }
    if (copy->src == NULL && src)
    {
        copy->src = src;
    }

end:
    return copy;

fail:
    ndt_route_leg_close(&copy);
    return NULL;
}

ndt_route_segment* ndt_route_segment_proced(ndt_waypoint *src, ndt_restriction *constraints, ndt_procedure *proc, ndt_navdatabase *ndb)
{
    size_t firstwpplus1 = 0;
    ndt_route_leg *rleg, *disc, *copy;
    ndt_restriction *skippedcstrs = NULL;
    ndt_list *proclegs = ndt_list_init();
    ndt_route_segment *rsgt = ndt_route_segment_init();
    if (!proc || !ndb || !proclegs || !rsgt)
    {
        goto fail;
    }

    switch (proc->type)
    {
        case NDT_PROCTYPE_SID_3:
        case NDT_PROCTYPE_SID_6:
        case NDT_PROCTYPE_STAR3:
        case NDT_PROCTYPE_STAR6:
        case NDT_PROCTYPE_STAR9:
        case NDT_PROCTYPE_SID_1:
        case NDT_PROCTYPE_SID_4:
        case NDT_PROCTYPE_STAR1:
        case NDT_PROCTYPE_STAR4:
        case NDT_PROCTYPE_STAR7:
        case NDT_PROCTYPE_APPTR:
            snprintf(rsgt->info.idnt, sizeof(rsgt->info.idnt), "%s.%s", proc->info.idnt, proc->info.misc);
            break;
        default:
            snprintf(rsgt->info.idnt, sizeof(rsgt->info.idnt),    "%s", proc->info.idnt);
            break;
    }
    rsgt->type = NDT_RSTYPE_PRC;
    rsgt->prc  = proc;
    rsgt->src  = src;

    /* Include any built-in transition's legs in the new leg list */
    if ((proc->type == NDT_PROCTYPE_STAR3 ||
         proc->type == NDT_PROCTYPE_STAR6 ||
         proc->type == NDT_PROCTYPE_STAR9) && proc->transition.star)
    {
        for (size_t i = 0; i < ndt_list_count(proc->transition.star->proclegs); i++)
        {
            ndt_list_add(proclegs, ndt_list_item(proc->transition.star->proclegs, i));
        }
    }
    for (size_t i = 0; i < ndt_list_count(proc->proclegs); i++)
    {
        ndt_list_add(proclegs, ndt_list_item(proc->proclegs, i));
    }
    if ((proc->type == NDT_PROCTYPE_SID_1 ||
         proc->type == NDT_PROCTYPE_SID_4) && proc->transition.sid)
    {
        for (size_t i = 0; i < ndt_list_count(proc->transition.sid->proclegs); i++)
        {
            ndt_list_add(proclegs, ndt_list_item(proc->transition.sid->proclegs, i));
        }
    }

    if (proc->type == NDT_PROCTYPE_SID_2 || proc->type == NDT_PROCTYPE_SID_5)
    {
        /*
         * Runway-agnostic SID portion, does not start from runway threshold.
         *
         * Note: this can be a procedure starting with vectors (e.g. AIRAC 1510,
         *       KABQ's LARGO 2), else an incomplete procedure (e.g. inserted by
         *       ICAO route parser).
         */
        if (!(disc = route_leg_discon()))
        {
            goto fail;
        }
        firstwpplus1 = 1;
        disc->rsg = rsgt;
        rsgt->src = src = NULL;
        ndt_list_add(rsgt->legs, disc);
    }
    else if (proc->type == NDT_PROCTYPE_SID_1 || proc->type == NDT_PROCTYPE_SID_4)
    {
        /* Runway-specific SID portion, starts from runway threshold. */
        if (!src)
        {
            /* Departure runway not set, threshold waypoint unavailable */
            if (!(disc = route_leg_discon()))
            {
                goto fail;
            }
            disc->rsg = rsgt;
            ndt_list_add(rsgt->legs, disc);
        }
        firstwpplus1 = 1;
    }
    else /* Any other procedure type */
    {
        if (src) /* Check src against the procedure's startpoint(s) */
        {
            /* First leg is always an entry point */
            if (!(rleg = ndt_list_item(proclegs, 0)))
            {
                goto fail;
            }
            if (rleg->src)
            {
                if (rleg->src == src)
                {
                    firstwpplus1 = 1; /* We're good, no discontinuity */
                }
            }
            else
            {
                if ((rleg->dst  == src) &&
                    (rleg->type == NDT_LEGTYPE_IF ||
                     rleg->type == NDT_LEGTYPE_CF ||
                     rleg->type == NDT_LEGTYPE_DF ||
                     rleg->type == NDT_LEGTYPE_TF))
                {
                    firstwpplus1 = 2; /* Basically direct to src, skip it */
                    skippedcstrs = &rleg->constraints;
                }
            }

            /* There can be additional entry points (IF or IAF legs only) */
            if (firstwpplus1 == 0)
            {
                for (size_t i = 1; i < ndt_list_count(proclegs); i++)
                {
                    if (!(rleg = ndt_list_item(proclegs, i)))
                    {
                        goto fail;
                    }
                    if ((rleg->dst == src) &&
                        (rleg->                type == NDT_LEGTYPE_IF ||
                         rleg->constraints.waypoint == NDT_WPTCONST_IAF))
                    {
                        firstwpplus1 = i + 2; /* Basically direct to src, skip it */
                        skippedcstrs = &rleg->constraints;
                        break;
                    }
                }
            }
        }

        if (firstwpplus1 == 0) /* src doesn't match any startpoint */
        {
            /* We need a discontinuity, unless we already have one */
            if (src)
            {
                if (!(disc = route_leg_discon()))
                {
                    goto fail;
                }
                disc->rsg = rsgt;
                rsgt->src = src = NULL;
                ndt_list_add(rsgt->legs, disc);
            }

            /* We will also need a way to reach rleg->src, if it's a fix */
            if (!(rleg = ndt_list_item(proclegs, 0)))
            {
                goto fail;
            }
            if (rleg->src)
            {
                if (!(copy = route_leg_direct(NULL, rleg->src)))
                {
                    goto fail;
                }
                copy->rsg = rsgt;
                rsgt->src = copy->dst;
                ndt_list_add(rsgt->legs, copy);
            }

            /* Obvious :P */
            firstwpplus1 = 1;
        }
    }
    rsgt->dst = rsgt->src; /* Just in case */

    /* Copy all selected legs to the route segment */
    for (size_t i = firstwpplus1 - 1; i < ndt_list_count(proclegs); i++)
    {
        if (!(rleg = ndt_list_item(proclegs, i)))
        {
            goto fail;
        }
        if (rleg->type == NDT_LEGTYPE_IF && rleg->dst == src)
        {
            /*
             * Navigraph 1510, KBOS:
             *
             * STAR,ROBUC1,ALL,5
             * […]
             * TF,JOODY,41.785381,-71.287564,0, ,0.0,0.0,0.0,0.0,0,0,0,0,0,0,0,0
             *
             * STAR,ROBUC1,04L,6
             * IF,JOODY,41.785381,-71.287564, ,0.0,0.0,0,0,0,0,0,0,0,0
             * […]
             *
             * Perfectly valid, but not handled by our leg skipping code above.
             */
            continue; // skip leg
        }

        /* Copy leg from procedure to segment and adjust as required */
        if (!(copy = route_leg4procedure(rleg, src)))
        {
            goto fail;
        }
        copy->rsg = rsgt;
        rsgt->dst = src = copy->dst;
        ndt_list_add(rsgt->legs, copy);

        /* Manual termination (expect VECTORS); append discontinuity */
        if (copy->type == NDT_LEGTYPE_FM || copy->type == NDT_LEGTYPE_VM)
        {
            if (!(disc = route_leg_discon()))
            {
                goto fail;
            }
            disc->rsg = rsgt;
            rsgt->dst = src = disc->dst;
            ndt_list_add(rsgt->legs, disc);
        }
    }

    /* If we skipped one of the procedure's legs, copy its constraints */
    if  (constraints && skippedcstrs)
    {
        *constraints = *skippedcstrs;
    }
    return rsgt;

fail:
    ndt_route_segment_close(&rsgt);
    return NULL;
}

ndt_restriction ndt_leg_const_init(void)
{
    ndt_restriction constraints;
    constraints.altitude.min =
    constraints.altitude.max = NDT_DISTANCE_ZERO;
    constraints.altitude.typ = NDT_RESTRICT_NO;
    constraints.airspeed.min =
    constraints.airspeed.max = ndt_airspeed_init(0, NDT_SPDUNIT_NAT);
    constraints.airspeed.acf = NDT_ACFTYPE_NON;
    constraints.airspeed.typ = NDT_RESTRICT_NO;
    constraints.    waypoint = NDT_WPTCONST_NO;
    constraints.        turn = NDT_TURN_SHORT;
    return constraints;
}

ndt_route_leg* ndt_route_leg_init()
{
    ndt_route_leg *leg = calloc(1, sizeof(ndt_route_leg));
    if (!leg)
    {
        goto end;
    }

    leg->constraints = ndt_leg_const_init();
    leg->type        = NDT_LEGTYPE_ZZ;

end:
    return leg;
}

void ndt_route_leg_close(ndt_route_leg **_leg)
{
    if (_leg && *_leg)
    {
        ndt_route_leg *leg = *_leg;

        if (leg->xpfms)
        {
            ndt_list_close(&leg->xpfms);
        }

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

    switch (constraints.altitude.typ)
    {
        case NDT_RESTRICT_AB:
        case NDT_RESTRICT_AT:
        case NDT_RESTRICT_BL:
        case NDT_RESTRICT_BT:
        case NDT_RESTRICT_NO:
            leg->constraints.altitude.typ = constraints.altitude.typ;
            break;
        default:
            return EINVAL;
    }

    if (constraints.altitude.typ == NDT_RESTRICT_AT ||
        constraints.altitude.typ == NDT_RESTRICT_AB ||
        constraints.altitude.typ == NDT_RESTRICT_BT)
    {
        if (ndt_distance_get(constraints.altitude.min, NDT_ALTUNIT_NA) <= 0LL)
        {
            return EINVAL;
        }
        leg->constraints.altitude.min = constraints.altitude.min;
    }

    if (constraints.altitude.typ == NDT_RESTRICT_AT ||
        constraints.altitude.typ == NDT_RESTRICT_BL ||
        constraints.altitude.typ == NDT_RESTRICT_BT)
    {
        if (ndt_distance_get(constraints.altitude.max, NDT_ALTUNIT_NA) <= 0LL)
        {
            return EINVAL;
        }
        leg->constraints.altitude.max = constraints.altitude.max;
    }

    switch (constraints.airspeed.typ)
    {
        case NDT_RESTRICT_AB:
        case NDT_RESTRICT_AT:
        case NDT_RESTRICT_BL:
        case NDT_RESTRICT_BT:
        case NDT_RESTRICT_NO:
            leg->constraints.airspeed.typ = constraints.airspeed.typ;
            break;
        default:
            return EINVAL;
    }

    switch (constraints.airspeed.acf)
    {
        case NDT_ACFTYPE_ALL:
        case NDT_ACFTYPE_JET:
        case NDT_ACFTYPE_OTH:
        case NDT_ACFTYPE_TBP:
            leg->constraints.airspeed.acf = constraints.airspeed.acf;
            break;
        case NDT_ACFTYPE_NON:
            leg->constraints.airspeed.acf = NDT_ACFTYPE_NON;
            leg->constraints.airspeed.typ = NDT_RESTRICT_NO;
            break;
        default:
            return EINVAL;
    }

    if (constraints.airspeed.typ == NDT_RESTRICT_AT ||
        constraints.airspeed.typ == NDT_RESTRICT_AB ||
        constraints.airspeed.typ == NDT_RESTRICT_BT)
    {
        if (ndt_airspeed_get(constraints.airspeed.min, NDT_SPDUNIT_NAT, NDT_MACH_DEFAULT) <= 0LL)
        {
            return EINVAL;
        }
        leg->constraints.airspeed.min = constraints.airspeed.min;
    }

    if (constraints.airspeed.typ == NDT_RESTRICT_AT ||
        constraints.airspeed.typ == NDT_RESTRICT_BL ||
        constraints.airspeed.typ == NDT_RESTRICT_BT)
    {
        if (ndt_airspeed_get(constraints.airspeed.max, NDT_SPDUNIT_NAT, NDT_MACH_DEFAULT) <= 0LL)
        {
            return EINVAL;
        }
        leg->constraints.airspeed.max = constraints.airspeed.max;
    }

    switch (constraints.waypoint)
    {
        case NDT_WPTCONST_FAF:
        case NDT_WPTCONST_FOV:
        case NDT_WPTCONST_IAF:
        case NDT_WPTCONST_MAP:
        case NDT_WPTCONST_NO:
            leg->constraints.waypoint = constraints.waypoint;
            break;
        default:
            return EINVAL;
    }

    switch (constraints.turn)
    {
        case NDT_TURN_LEFT:
        case NDT_TURN_RIGHT:
        case NDT_TURN_SHORT:
            leg->constraints.turn = constraints.turn;
            break;
        default:
            return EINVAL;
    }

    return 0;
}

static int endpoint_intcpt(ndt_list *xpfms,
                           ndt_list *cwlst,
                           void *wmm, ndt_date now,
                           ndt_waypoint *src1, double brg1,
                           ndt_waypoint *src2, double brg2, double intc_crs)
{
    if (!xpfms || !cwlst || !wmm)
    {
        return ENOMEM;
    }
    if (!src1 || !src2)
    {
        return EINVAL;
    }
    ndt_waypoint *wpt;
    ndt_position posn;
    double trb1, trb2;
    int          pbpb;

    /*
     * Navigraph 1511 examples of "impossible" intercepts:
     *
     * KSFO, 28L, OFFSH9
     * CF,SENZY,37.666500,-122.485167,0,SFO,281.0,6.0,281.0,6.0,2,2500,0,0,0,0,0,1
     * VI,0, ,0.0,203.0,0,0,0,0,0,0,0,0
     * CF,WAMMY,37.541064,-122.724056,0,PYE,151.0,33.0,151.0,3.0,0,0,0,0,0,0,0,0
     *
     * LSZH, 28, DEGE2W
     * CF,ZH552,47.428889,8.391667,0,KLO,252.9,6.5,253.0,4.0,0,0,0,1,210,0,0,1
     * CF,KLO,47.457139,8.545583,1,ZUE,231.9,13.7,52.0,7.0,0,0,0,0,0,0,0,0
     *
     * LSZH, 28, GERS2W
     * CF,KLO02,47.462342,8.489597,0,KLO,275.8,2.3,274.0,2.0,0,0,0,0,0,0,0,1
     * CF,BREGO,47.389722,8.346111,0,KLO,241.6,9.1,233.0,7.0,2,5000,0,0,0,0,0,1
     *
     * The procedures rely on a combination of overfly and a very generous turn
     * radius to make these intercepts actually work, but computing the correct
     * radius and the associated src1 waypoint is highly impractical. Instead,
     * the calculations below compute a course that lets us intercept the next
     * leg's course (intc_crs) at a 90-degree angle, giving a waypoint that
     * appears to work well enough for the above examples and should be OK for
     * any other procedures: considering that the dummy we insert will not be
     * overfly, both the QPAC plugin and default X-Plane 10 navigation should
     * happily "soft skip" it while using it to compute the next leg's course,
     * resulting in a suboptimal flight path, that is nevertheless acceptable.
     *
     * KSFO, 01L, GAPP6
     * CA,0,14.0,2,420,0,0,0,0,0,0
     * FM,SFO,37.619483,-122.373895,0,SFO,0.0,0.0,350.0,0,0,0,0,0,0,0,0
     *
     * Different issue than the above 3, but the code below also works
     * to force-find a suitable incercept course and associated waypoint.
     */
    trb1 = ndt_wmm_getbearing_tru   (  wmm, brg1, src1->position, now);
    trb2 = ndt_wmm_getbearing_tru   (  wmm, brg2, src2->position, now);
    pbpb = ndt_position_calcpos4pbpb(&posn, src1->position, trb1, src2->position, trb2);
    if (pbpb)
    {
        goto force_intercept;
    }
    /*
     * Aerosoft 1405 example of valid but wrong intercept:
     *
     * LSZH, 28, DEGE2W
     * CF,ZH552,47.42888889,8.39166667,0,KLO,253.9,6.5,255,4.00,0,0,0,0,0,0,0,1
     * CF,R234,47.43513056,8.49935556,1,KLO,233.9,2.3,54,4.50,0,0,0,1,210,0,0,0
     *
     * ZH552 (255) (054) R234
     *
     * There is a valid intercept (oops), but it's located thousands of miles
     * away, so we have to force a more reasonable one. So many workarounds :(
     *
     * Note: the issue described above is masked by the fact that we pre-compute
     * the 90-degree angle when the previous leg isn't intercept-only (e.g. CF).
     */
    if (ndt_distance_get(ndt_position_calcdistance(src1->position, posn), NDT_ALTUNIT_NM) > INT64_C(99))
    {
        goto force_intercept; // quite a bit too far, must do something about it
    }
    goto endpoint;

    /*
     * Implement the workaround described above.
     */
force_intercept:
    if (fabs(ndt_position_bearing_angle(brg2, intc_crs)) > 1.)
    {
        /*
         * intc_crs != brg2: next leg is CF. Assume we're flying direct to
         * src2 and update brg1 accordingly; compute turn direction and the
         * associated "90-degree angle" intercept course using the new brg1.
         */
        trb1 = ndt_position_calcbearing(src1->position, src2->position);
        brg1 = ndt_wmm_getbearing_mag  (wmm, trb1, src1->position, now);
    }
    brg1 = (ndt_position_bearing_angle(brg1, brg2) < 0. ?
            ndt_mod(intc_crs - 90., 360.) :
            ndt_mod(intc_crs + 90., 360.));
    trb1 = ndt_wmm_getbearing_tru   ( wmm, brg1, src1->position, now);
    trb2 = ndt_wmm_getbearing_tru   ( wmm, brg2, src2->position, now);
    pbpb = ndt_position_calcpos4pbpb(NULL, src1->position, trb1, src2->position, trb2);
    if (pbpb)
    {
        return pbpb;
    }
    goto endpoint;

endpoint:
    if (!(wpt = ndt_waypoint_pbpb(src1, brg1, src2, brg2, now, wmm)))
    {
        return ENOMEM;
    }
    wpt->type = NDT_WPTYPE_LLC;
    snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "(INTC-%03.0lf)", round(intc_crs));
    ndt_list_add(xpfms, wpt);
    ndt_list_add(cwlst, wpt);
    return 0;
}

static int endpoint_radial(ndt_list *xpfms, ndt_list *cwlst, void *wmm, ndt_date now,
                           ndt_waypoint *src,    double bearing,
                           ndt_waypoint *navaid, double radial)
{
    if (!xpfms || !cwlst || !wmm)
    {
        return ENOMEM;
    }
    if (!src || !navaid)
    {
        return EINVAL;
    }

    /*
     * Aerosoft, 1405, LSZH:
     * SID,ALBI1R,16,2
     * CF,D1KLO,47.44305556,8.55861111,0,KLO,147.0,1.0,154,1.50,0,0,0,0,0,0,0,1
     * FA,D1KLO,47.44305556,8.55861111,0,KLO,147.0,1.0,154,2,2400,0,0,0,0,0,0
     * CR,1,KLO,0.0,255,2,4000,0,1,210,0,0,0
     *
     * (2400) 255.0, KLO 000.0: intersection(s) ambiguous
     *
     * We're instructed to intercept a northbound radial from KLO by heading
     * SSW, like that's actually going to happen. Instead, simply go direct to
     * the navaid (intercepting every radial). Shitty coding, shitty workaround.
     * Note: as per the chart's text, we're supposed to turn left to intercept
     *       KLO R-253, which is equally impossible from present heading; we'd
     *       actually be intercepting R-253 from whatever heading we're flying
     *       at the end of our left turn, but that can only be realistically
     *       determined by a human (IMHO), and I don't care to play this game.
     *
     * Future: consider using a similar forced intercept as endpoint_intcpt()
     */
    double trb1 = ndt_wmm_getbearing_tru(wmm, bearing,   src->position, now);
    double trb2 = ndt_wmm_getbearing_tru(wmm, radial, navaid->position, now);
    if (ndt_position_calcpos4pbpb(NULL, src->position, trb1, navaid->position, trb2))
    {
        ndt_list_add(xpfms, navaid);
        return 0;
    }
    ndt_waypoint *wpt = ndt_waypoint_pbpb(src, bearing, navaid, radial, now, wmm);
    if (!wpt)
    {
        return ENOMEM;
    }
    wpt->type = NDT_WPTYPE_LLC;
    snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "(%s-R%03.0lf)", navaid->info.idnt, round(radial));
    ndt_list_add(xpfms, wpt);
    ndt_list_add(cwlst, wpt);
    return 0;
}

static int endpoint_dmedis(ndt_list *xpfms, ndt_list *cwlst, void *wmm, ndt_date now, ndt_waypoint *src,
                           ndt_waypoint *dmenav, double bearing, ndt_distance dmedis)
{
    if (!xpfms || !cwlst || !wmm)
    {
        return ENOMEM;
    }
    if (!src || !dmenav)
    {
        return EINVAL;
    }
    ndt_waypoint *wpt = ndt_waypoint_pbpd(src, bearing, dmenav, dmedis, now, wmm);
    if (!wpt)
    {
        return ENOMEM;
    }
    wpt->type = NDT_WPTYPE_LLC;
    snprintf(   wpt->info.idnt, sizeof(wpt->info.idnt), "(%s-D%.1lf)",
             dmenav->info.idnt, ndt_distance_get(dmedis, NDT_ALTUNIT_ME) / 1852.);
    ndt_list_add(xpfms, wpt);
    ndt_list_add(cwlst, wpt);
    return 0;
}

static int endpoint_altitd(ndt_list *xpfms, ndt_list *cwlst, void *wmm, ndt_date now, ndt_waypoint *src,
                           double bearing, ndt_distance altfrom, ndt_distance altstop, ndt_distance rwyl)
{
    if (!xpfms || !cwlst || !wmm)
    {
        return ENOMEM;
    }
    if (!src)
    {
        return EINVAL;
    }
    ndt_waypoint *wpt;
    ndt_distance dist;
    ndt_distance diff = ndt_distance_rem(altstop, altfrom);
    if (ndt_distance_get(diff, NDT_ALTUNIT_NA) > 0LL)
    {
        // climbing 4.1° (1 vertical feet per 11 horizontal feet)
        dist = ndt_distance_init(ndt_distance_get(diff, NDT_ALTUNIT_NA) *  11LL, NDT_ALTUNIT_NA);
        dist = ndt_distance_add (dist, rwyl);
    }
    else
    {
        // descending 3.0° (1 vertical feet per 15 horizontal feet)
        dist = ndt_distance_init(ndt_distance_get(diff, NDT_ALTUNIT_NA) * -15LL, NDT_ALTUNIT_NA);
    }
    if (!(wpt = ndt_waypoint_pbd(src, bearing, dist, now, wmm)))
    {
        return ENOMEM;
    }
    wpt->type = NDT_WPTYPE_LLC;
    snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "(%"PRId64")", ndt_distance_get(altstop, NDT_ALTUNIT_FT));
    ndt_list_add(xpfms, wpt);
    ndt_list_add(cwlst, wpt);
    return 0;
}

/*
 * Notes.
 *
 * - fix to <*> legs:   sometimes src != leg->fix.src (e.g. MMSD, MMTO); the leg
 *                      doesn't necessarily start at src, but the dummy is always
 *                      computed from leg->fix.src regardless. However, we should
 *                      not insert a dummy waypoint for leg->fix.src in that case
 *
 * - FA legs:           sometimes the first leg of a runway->SID procedure, but
 *                      the leg's source waypoint is always the runway threshold,
 *                      except for LGPZ and SCBA (Navigraph 1510), but it's still
 *                      close enough to be equivalent; so always use leg->fix.src
 *
 * - <*> to alt. legs:  climb rate calibrated w/QPAC A320 2.0.2 at maximum gross
 *                      weight, FLEX69, takeoff from LSGG23 (1365ft), alt. 7000ft
 *                      reached 4.9nm after Passeiry VOR (direct course from rwy)
 *
 * - crs to <*> legs: future: when legsrc is overfly, the course should not start
 *                            from legsrc exactly, otherwise the endpoint will be
 *                            slightly incorrect; e.g. Navigraph 1510, LSZH, SID,
 *                            ALBI1D: CF(253, ZH553, ofly), CD(149, HOC D31) etc.
 */
static ndt_waypoint* route_leg_xpfms(ndt_flightplan *flp,    ndt_date       now,
                                     ndt_waypoint   *legsrc, ndt_route_leg *leg,
                                     ndt_route_leg   *nxt,   ndt_distance *_alt)
{
    ndt_runway    *rwy = NULL;
    int            err = 0;
    ndt_waypoint  *wpt;
    void          *wmm;
    double pi_finalbrg;

    if (!flp || !legsrc || !leg || !_alt || !(wmm = flp->ndb->wmm))
    {
        err = ENOMEM;
        goto end;
    }

    if ((leg->rsg->     type == NDT_RSTYPE_PRC) &&
        (leg->rsg->prc->type == NDT_PROCTYPE_SID_1 ||
         leg->rsg->prc->type == NDT_PROCTYPE_SID_4))
    {
        if ((ndt_list_count(leg->rsg->prc->runways) > 1) ||
            (rwy = ndt_list_item(leg->rsg->prc->runways, 0)) == NULL)
        {
            err = EINVAL;
            goto end; // neither should ever happen (bug)
        }
        if (leg == ndt_list_item(leg->rsg->legs, 0) || legsrc == rwy->waypoint)
        {
            /*
             * Procedure's first leg, or second leg (when first leg is a
             * discontinuity). Since the procedure's type is runway->SID,
             * legsrc is the procedure runway's threshold.
             */
            *_alt  = rwy->threshold.altitude;
            legsrc = rwy->waypoint;
        }
        else
        {
            rwy = NULL; // no takeoff roll, no runway
        }
    }

    /*
     * Note: if we want to recalculate some legs every time,
     * then empty their xpfms list here and goto as required.
     */
    switch (leg->type)
    {
        case NDT_LEGTYPE_AF:
        case NDT_LEGTYPE_CA:
        case NDT_LEGTYPE_CD:
        case NDT_LEGTYPE_CR:
        case NDT_LEGTYPE_FA:
        case NDT_LEGTYPE_FC:
        case NDT_LEGTYPE_FD:
        case NDT_LEGTYPE_PI: // additional dummies before intercept
        case NDT_LEGTYPE_RF:
        case NDT_LEGTYPE_VA:
        case NDT_LEGTYPE_VD:
        case NDT_LEGTYPE_VR:
            goto dummies;
        case NDT_LEGTYPE_CF: // intc_drect if nxt->type == DF
        case NDT_LEGTYPE_DF: // intc poss. if nxt->type == CF
        case NDT_LEGTYPE_IF: // intc poss. if nxt->type == CF
        case NDT_LEGTYPE_TF: // intc poss. if nxt->type == CF
        case NDT_LEGTYPE_CI: // intc mandatory
        case NDT_LEGTYPE_VI: // intc mandatory
            goto intc;
        case NDT_LEGTYPE_FM:
        case NDT_LEGTYPE_VM:
        case NDT_LEGTYPE_ZZ:
            goto end;
        case NDT_LEGTYPE_HA:
        case NDT_LEGTYPE_HF:
        case NDT_LEGTYPE_HM:
        default:
            goto altitude;
    }

    /*
     * Calculate dummy waypoints for complex legs.
     */
dummies:
    if (ndt_list_count(leg->xpfms))
    {
        goto altitude;
    }
    if (!leg->xpfms)
    {
        err = EINVAL;
        goto end;
    }
    if (leg->type == NDT_LEGTYPE_CA ||
        leg->type == NDT_LEGTYPE_FA ||
        leg->type == NDT_LEGTYPE_VA)
    {
        ndt_distance  alt;
        double        brg;
        switch (leg->type)
        {
            case NDT_LEGTYPE_CA:
                alt = leg->course.altitude;
                brg = leg->course.magnetic;
                break;
            case NDT_LEGTYPE_FA:
                alt = leg->fix.altitude;
                brg = leg->fix.course;
                break;
            case NDT_LEGTYPE_VA:
                alt = leg->heading.altitude;
                brg = leg->heading.degrees;
                break;
            default:
                break; // compiler warning…
        }
        if ((err = endpoint_altitd(leg->xpfms, flp->cws, wmm, now, legsrc, brg,
                                   *_alt, alt, rwy ? rwy->length : NDT_DISTANCE_ZERO)))
        {
            goto end;
        }
        goto intc;
    }
    if (leg->type == NDT_LEGTYPE_CD ||
        leg->type == NDT_LEGTYPE_FD ||
        leg->type == NDT_LEGTYPE_VD)
    {
        ndt_waypoint *nav;
        ndt_waypoint *src;
        ndt_distance  dme;
        double        brg;
        switch (leg->type)
        {
            case NDT_LEGTYPE_CD:
                dme = leg->course.distance;
                brg = leg->course.magnetic;
                nav = leg->course.navaid;
                src = legsrc;
                break;
            case NDT_LEGTYPE_FD:
                dme = leg->fix.distance;
                brg = leg->fix.course;
                nav = leg->fix.navaid;
                src = leg->fix.src;
                break;
            case NDT_LEGTYPE_VD:
                dme = leg->heading.distance;
                brg = leg->heading.degrees;
                nav = leg->heading.navaid;
                src = legsrc;
                break;
            default:
                break; // compiler warning…
        }
        if (!src || !nav)
        {
            err = EINVAL;
            goto end;
        }
        if ((err = endpoint_dmedis(leg->xpfms, flp->cws, wmm, now,
                                   src, nav, brg, dme)))
        {
            goto end;
        }
        goto intc;
    }
    if (leg->type == NDT_LEGTYPE_CR ||
        leg->type == NDT_LEGTYPE_VR)
    {
        ndt_waypoint *nav;
        double        brg;
        double        rad;
        switch (leg->type)
        {
            case NDT_LEGTYPE_CR:
                brg = leg->course.magnetic;
                rad = leg->course.radial;
                nav = leg->course.navaid;
                break;
            case NDT_LEGTYPE_VR:
                brg = leg->heading.degrees;
                rad = leg->heading.radial;
                nav = leg->heading.navaid;
                break;
            default:
                break; // compiler warning…
        }
        if (!nav)
        {
            err = EINVAL;
            goto end;
        }
        if ((err = endpoint_radial(leg->xpfms, flp->cws, wmm, now,
                                   legsrc, brg, nav, rad)))
        {
            goto end;
        }
        goto intc;
    }
    if (leg->type == NDT_LEGTYPE_FC)
    {
        if (!(wpt = ndt_waypoint_pbd(legsrc,
                                     leg->fix.course,
                                     leg->fix.distance, now, wmm)))
        {
            err = ENOMEM;
            goto end;
        }
        wpt->type = NDT_WPTYPE_LLC;
        ndt_list_add(leg->xpfms, wpt);
        ndt_list_add(flp->cws,   wpt);
        goto intc;
    }
    if (leg->type == NDT_LEGTYPE_AF)
    {
        /*
         * Calibrated at LFMN:
         *
         * NERAS.ILS04L:
         * - 79 degrees at DME27: 8 dummies; 79/8 ~= 9.8 degrees per dummy
         * MERL6C:
         * - 22 degrees at DME40: 3 dummies; 22/3 ~= 7.3 degrees per dummy
         * KERI6C:
         * - 11 degrees at DME47: 3 dummies; 11/2 ~= 3.7 degrees per dummy
         *
         * Minimum of 5 degrees per dummy at DME30, 10 degrees at DME15, etc.
         * Minimum of 3 dummies.
         */
        double ang = ndt_position_bearing_angle(leg->arc.start, leg->arc.stop); // angle for shortest turn
        if ((leg->constraints.turn == NDT_TURN_LEFT  && ang > 0.) ||
            (leg->constraints.turn == NDT_TURN_RIGHT && ang < 0.))
        {
            ang = ndt_position_angle_reverse(ang);
        }
        double dme = ndt_distance_get(leg->arc.distance, NDT_ALTUNIT_ME) / 1852.;
        int  count = 2 + (fabs(ang) * dme / 300.); count += count < 3;
        double stp = ang / (double)count;
        for (int i = 0; i < count; i++)
        {
            double radial = ndt_mod(leg->arc.start + ((double)i * stp), 360.);
            if (!(wpt = ndt_waypoint_pbd(leg->arc.center, radial,
                                         leg->arc.distance, now, wmm)))
            {
                err = ENOMEM;
                goto end;
            }
            if (i + 1 < count &&
                !ndt_distance_get(ndt_position_calcdistance(legsrc->position,
                                                            wpt->position), NDT_ALTUNIT_NM))
            {
                // wpt too close to legsrc and we'll have another, skip this one
                ndt_waypoint_close(&wpt);
                continue;
            }
            wpt->type = NDT_WPTYPE_LLC;
            snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "%s/D%.0lf/R%03.0lf",
                     leg->arc.center->info.idnt,
                     round(ndt_distance_get(leg->arc.distance, NDT_ALTUNIT_ME) / 1852.),
                     round(radial));
            ndt_list_add(leg->xpfms, wpt);
            ndt_list_add(flp->cws,   wpt);
        }
        goto intc;
    }
    if (leg->type == NDT_LEGTYPE_RF)
    {
        /*
         * leg->radius.distance is the distance from src to radius.center
         * leg->radius.start    is the bearing  from src to radius.center
         *             start    is the bearing  from radius.center to src
         *             rstop    is the bearing  from radius.center to dst
         *             angle    is the angle covered by the radius segment
         *
         * Minimums are the same as for AF except the
         * first dummy == leg->src, so we don't write it.
         */
        double start = ndt_mod(leg->radius.start + 180., 360.);
        double trueb = ndt_position_calcbearing(leg->radius.center->position, leg->dst->position);
        double rstop = ndt_wmm_getbearing_mag  (wmm, trueb,    leg->radius.center->position, now);
        double angle = ndt_position_bearing_angle(start, rstop); // angle for shortest turn
        if ((leg->constraints.turn == NDT_TURN_LEFT  && angle > 0.) ||
            (leg->constraints.turn == NDT_TURN_RIGHT && angle < 0.))
        {
            angle = ndt_position_angle_reverse(angle);
        }
        double rdist = ndt_distance_get(leg->radius.distance, NDT_ALTUNIT_ME) / 1852.;
        int    count = 2 + (fabs(angle) * rdist / 300.); count += count < 3;
        double rstep = angle / (double)count;
        for (int   i = 1; i < count; i++)
        {
            double bearing = ndt_mod(start + ((double)i * rstep), 360.);
            if (!(wpt = ndt_waypoint_pbd(leg->radius.center, bearing,
                                         leg->radius.distance, now, wmm)))
            {
                err = ENOMEM;
                goto end;
            }
            wpt->type = NDT_WPTYPE_LLC;
            snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "%s/D%.1lf/%03.0lf",
                     leg->radius.center->info.idnt,
                     ndt_distance_get(leg->radius.distance, NDT_ALTUNIT_ME) / 1852.,
                     round(bearing));
            ndt_list_add(leg->xpfms, wpt);
            ndt_list_add(flp->cws,   wpt);
        }
        goto intc;
    }
    if (leg->type == NDT_LEGTYPE_PI)
    {
        /*
         * First segment, from initial turn fix to a distance along a course.
         */
        ndt_waypoint *wpt1, *wpt2, *wpt3;
        ndt_distance  dist = leg->turn.outdis;
        double        brng = leg->turn.outbrg;
        if (!(wpt1 = ndt_waypoint_pbd(leg->turn.waypoint, brng, dist, now, wmm)))
        {
            err = ENOMEM;
            goto end;
        }
        wpt1->type = NDT_WPTYPE_LLC;
        snprintf(wpt1->info.idnt, sizeof(wpt1->info.idnt), "(TURN %03.0lf)",
                 brng + leg->turn.tangle);
        ndt_list_add(leg->xpfms, wpt1);
        ndt_list_add(flp->cws,   wpt1);
        /*
         * Second segment, turn away from outbound course for a short distance.
         * Not sure how to use limdis, so let's just make is short as possible.
         * Calibrated for nicest short turn using FlightFactor/QPAC A350 v1.20.
         */
        dist = ndt_distance_init(INT64_C(5000), NDT_ALTUNIT_ME); // 2.70nm
        brng = brng + leg->turn.tangle;
        if (!(wpt2 = ndt_waypoint_pbd(wpt1, brng, dist, now, wmm)))
        {
            err = ENOMEM;
            goto end;
        }
        wpt2->type = NDT_WPTYPE_LLC;
        snprintf(wpt2->info.idnt, sizeof(wpt2->info.idnt), "(TURN %s)",
                 leg->turn.tangle > 0. ? "LEFT" : "RIGHT");
        ndt_list_add(leg->xpfms, wpt2);
        ndt_list_add(flp->cws,   wpt2);
        /*
         * Third segment, start turning back towards original outbound course.
         */
        dist = ndt_distance_init(INT64_C(5000), NDT_ALTUNIT_ME); // 2.70nm
        brng = brng - leg->turn.tangle / 2.;
        if (!(wpt3 = ndt_waypoint_pbd(wpt1, brng, dist, now, wmm)))
        {
            err = ENOMEM;
            goto end;
        }
        wpt3->type = NDT_WPTYPE_LLC;
        snprintf(wpt3->info.idnt, sizeof(wpt3->info.idnt), "%s", "(TURN BACK)");
        ndt_list_add(leg->xpfms, wpt3);
        ndt_list_add(flp->cws,   wpt3);
        /*
         * Store the final true course in order to compute a final intercept.
         */
        double tbrg = ndt_position_calcbearing(wpt2->position, wpt3->position);
        pi_finalbrg = ndt_wmm_getbearing_mag  (wmm, tbrg, wpt2->position, now);
        goto intc;
    }

    /*
     * Compute explicit or implicit intercept course
     * to the next leg, and add waypoint if required.
     *
     * nxt must be course-defined and have a fix as either its start or endpoint.
     * Note: nxt w/type DF is implicitly course-defined if leg is course-defined.
     */
intc:
    if (!nxt)
    {
        goto altitude;
    }
    ndt_waypoint *src1, *src2;
    double  intc, brg1,  brg2;
    ndt_list            *xpfm;
    switch (leg->type)
    {
        case NDT_LEGTYPE_CA:
        case NDT_LEGTYPE_CD:
        case NDT_LEGTYPE_CR:
            brg1 = leg->course.magnetic;
            src1 = ndt_list_item(leg->xpfms, -1);
            break;
        case NDT_LEGTYPE_CI:
            brg1 = leg->course.magnetic;
            src1 = legsrc;
            break;
        case NDT_LEGTYPE_FA:
        case NDT_LEGTYPE_FD:
            brg1 = leg->fix.course;
            src1 = ndt_list_item(leg->xpfms, -1);
            break;
        case NDT_LEGTYPE_PI:
            brg1 = pi_finalbrg;
            src1 = ndt_list_item(leg->xpfms, -1);
            break;
        case NDT_LEGTYPE_VA:
        case NDT_LEGTYPE_VD:
        case NDT_LEGTYPE_VR:
            brg1 = leg->heading.degrees;
            src1 = ndt_list_item(leg->xpfms, -1);
            break;
        case NDT_LEGTYPE_VI:
            brg1 = leg->heading.degrees;
            src1 = legsrc;
            break;
        case NDT_LEGTYPE_CF:
            if (nxt->type == NDT_LEGTYPE_DF)
            {
                brg1 = leg->course.magnetic;
                src1 = leg->dst;
                goto intc_drect;
            }
        default:
            if (leg->dst && nxt->dst && nxt->type == NDT_LEGTYPE_CF)
            {
                double trb1;
                src1 = leg->dst;
                src2 = nxt->dst;
                intc = nxt->course.magnetic;
                brg2 = ndt_mod(intc + 180., 360.);
                trb1 = ndt_position_calcbearing(src1->position, src2->position);
                brg1 = ndt_wmm_getbearing_mag  (wmm, trb1, src1->position, now);
                if (fabs(ndt_position_bearing_angle (brg1, intc)) < 6.)
                {
                    goto altitude; // almost no turn, pointless intercept
                }
                /*
                 * Pre-compute same 90-degree angle as endpoint_intcpt() would.
                 */
                brg1 = (ndt_position_bearing_angle(brg1, brg2) < 0. ?
                        ndt_mod(intc - 90., 360.) : ndt_mod(intc + 90., 360.));
                break;
            }
            goto altitude;
    }
    switch (nxt->type)
    {
        case NDT_LEGTYPE_FA:
        case NDT_LEGTYPE_FC:
        case NDT_LEGTYPE_FD:
        case NDT_LEGTYPE_FM:
            xpfm = leg->xpfms;
            src2 = nxt->fix.src;
            intc = nxt->fix.course;
            brg2 = nxt->fix.course;
            break;
        case NDT_LEGTYPE_CF:
            src2 = nxt->dst;
            intc = nxt->course.magnetic;
            brg2 = ndt_mod(intc + 180., 360.);
            if (leg->type == NDT_LEGTYPE_CI ||
                leg->type == NDT_LEGTYPE_PI ||
                leg->type == NDT_LEGTYPE_VI)
            {
                xpfm = leg->xpfms;
                break; // mandatory intercept
            }
            if (src1 && src2)
            {
                if (ndt_distance_get(ndt_position_calcdistance(src1->position,
                                                               src2->position), NDT_ALTUNIT_NM) < INT64_C(3))
                {
                    goto altitude; // too short, pointless intercept
                }
            }
            xpfm = nxt->xpfms;
            break;
        case NDT_LEGTYPE_DF:
            goto intc_drect;
        default:
            goto altitude;
    }
    if (!src1 || !src2 || !xpfm)
    {
        err = EINVAL;
        goto end;
    }
    if (ndt_list_count(xpfm))
    {
        if (leg->type == NDT_LEGTYPE_CI || leg->type == NDT_LEGTYPE_VI)
        {
            goto altitude; // xpfm == leg->xpfms (checked earlier except CI/VI)
        }
        if (xpfm == nxt->xpfms)
        {
            goto altitude; // xpfm == nxt->xpfms (checked here only)
        }
    }
    if (leg->type != NDT_LEGTYPE_CI && // mandatory intercept
        leg->type != NDT_LEGTYPE_PI && // mandatory intercept
        leg->type != NDT_LEGTYPE_VI && // mandatory intercept
        fabs(ndt_position_bearing_angle(brg1, intc)) < 6.)
    {
        goto altitude; // almost no turn, pointless intercept
    }
    /*
     * Future: turn direction constraints for intercepts too.
     *         Aerosoft NavDataPro, AIRAC 1405, LSZH, ALBI1D:
     *         SID,ALBI1D,10,2
     *         CF,21KLO,47.45485833,8.59706111,0,KLO,92.7,2.1,95,2.50,0,0,0,0,0,0,0,1
     *         FA,21KLO,47.45485833,8.59706111,0,KLO,92.7,2.1,95,2,2500,0,0,0,0,0,0
     *         CF,KLO,47.45713889,8.54558333,1,KLO,0.0,0.0,255,4.00,2,4000,0,1,210,0,0,0
     *         // turn left to KLO, but QPAC plugin defaults right (A350 v1.2.1)
     */
    if ((err = endpoint_intcpt(xpfm, flp->cws, wmm, now,
                               src1, brg1, src2, brg2, intc)))
    {
        /*
         * Aerosoft 1511, DTTA, ILS 01 approach, TUC transition:
         * FC,TUC05,36.77061667,10.20523333,0,TUC,192.9,5.0,238,4.30,2,1900,0,0,0,0,0,0
         * CI,1, ,0,58,0,0,0,0,0,0,0,0
         * CF,R191H,36.72159444,10.19271667,0,TBL,191.0,9.1,11,2.00,0,0,0,0,0,0,0,0
         *
         * As depicted (LIDO chart), procedure appears to involve an overfly of
         * the waypoint represented by FC/TUC05/238/4.3, followed by a turn to
         * heading 058 from which we can intercept track 011 to R191H fix; since
         * we don't account for the overfly, course 058 from said waypoint can
         * only intercept track 011 after fix R919H, resulting in an impossible
         * intercept. Our workaround then tries to fly heading 101 instead (to
         * intercept track 011 with a 90-degree angle), but the direct course
         * from the waypoint to R191H is approximately 102 degrees, resulting in
         * yet another case of "intercept track to fix after fix, so impossible"
         * issue. We can basically fly the procedure correctly by skipping said
         * mandatory intercept and flying directly to fix R191H, so let's do so.
         *
         * More generally, whenever an intercept fails, if we have a fix we can
         * fly direct to, then ignore the error; otherwise print and error out.
         */
        if (nxt->type == NDT_LEGTYPE_CF)
        {
            err = 0; goto altitude; // we have a fix we can go to, ignore error
        }
        switch (err)
        {
            case EDOM:
                ndt_log("%s %05.1lf, %s %05.1lf: infinity of intersections\n", src1->info.idnt, brg1, src2->info.idnt, brg2);
                break;
            case ERANGE:
                ndt_log("%s %05.1lf, %s %05.1lf: intersection(s) ambiguous\n", src1->info.idnt, brg1, src2->info.idnt, brg2);
                break;
            default:
                break;
        }
        goto end; // intercept failed without obvious fallback: let's error out
    }
    goto altitude;

    /*
     * Directs are tricky: src and dst may be the same,
     * and we may also have a turn direction to respect.
     */
intc_drect:
    if (ndt_list_count(nxt->xpfms))
    {
        goto altitude;
    }
    if (!nxt->xpfms)
    {
        err = EINVAL;
        goto end;
    }
    /*
     * Direct from a fix and back.
     * Calibrated for nicest turn using FlightFactor/QPAC A350 1.20:
     * LIPE 30 LSGG none none none "BOA7P FRZ"
     *
     * Note: we previously relied on distance-based checks only (1nm), but it
     *       gave false positives, e.g. KABQ, JEMEZ3.08, (5860) -> TYILR: ~417m
     */
    ndt_distance dctd = ndt_position_calcdistance(src1->position, nxt->dst->position);
    if ((src1 == nxt->dst) ||
        (src1 == leg->dst &&
         ndt_distance_get(dctd, NDT_ALTUNIT_FT) < INT64_C(660))) // 1 furlong
    {
        dctd = ndt_distance_init(INT64_C(3000), NDT_ALTUNIT_ME);
        if (!(wpt = ndt_waypoint_pbd(nxt->dst, brg1, dctd, now, wmm)))
        {
            err = ENOMEM;
            goto end;
        }
        wpt->type = NDT_WPTYPE_LLC;
        snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "(DCT %s D1.6)", nxt->dst->info.idnt);
        ndt_list_add(nxt->xpfms, wpt);
        ndt_list_add(flp->cws,   wpt);
        switch (nxt->constraints.turn)
        {
            case NDT_TURN_RIGHT:
                brg1 = ndt_mod(brg1 + 60., 360.);
                break;
            case NDT_TURN_LEFT:
                brg1 = ndt_mod(brg1 - 60., 360.);
                break;
            default:
                goto altitude;
        }
        if (!(wpt = ndt_waypoint_pbd(nxt->dst, brg1, dctd, now, wmm)))
        {
            err = ENOMEM;
            goto end;
        }
        wpt->type = NDT_WPTYPE_LLC;
        snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "(TURN %s)",
                 nxt->constraints.turn == NDT_TURN_LEFT ? "LEFT" : "RIGHT");
        ndt_list_add(nxt->xpfms, wpt);
        ndt_list_add(flp->cws,   wpt);
        goto altitude;
    }
    /*
     * Direct from somewhere to a fix. Avoid sharp turns by adding turn
     * helper points, according to how large the turn angle actually is.
     */
    if (!ndt_distance_get(dctd, NDT_ALTUNIT_NM))
    {
        goto altitude; // waypoints too close, so angle unreliable: skip helpers
    }
    double dtrb = ndt_position_calcbearing  (src1->position, nxt->dst->position);
    double dctb = ndt_wmm_getbearing_mag    (wmm,  dtrb,    src1->position, now);
    double angl = ndt_position_bearing_angle(brg1, dctb);
    if ((nxt->constraints.turn == NDT_TURN_LEFT  && angl > 0.) ||
        (nxt->constraints.turn == NDT_TURN_RIGHT && angl < 0.))
    {
        angl = ndt_position_angle_reverse(angl);
    }
    if (fabs(angl) > 120. && leg->constraints.waypoint != NDT_WPTCONST_FOV)
    {
        dctd = ndt_distance_init(INT64_C(3000), NDT_ALTUNIT_ME);
        if (angl < 0.)
        {
            wpt = ndt_waypoint_pbd(src1, ndt_mod(brg1 - 090., 360.), dctd, now, wmm);
        }
        else
        {
            wpt = ndt_waypoint_pbd(src1, ndt_mod(brg1 + 090., 360.), dctd, now, wmm);
        }
        if (!wpt)
        {
            err = ENOMEM;
            goto end;
        }
        wpt->type = NDT_WPTYPE_LLC;
        snprintf(wpt->info.idnt, sizeof(wpt->info.idnt),
                 "(TURN %s)", angl < 0. ? "LEFT" : "RIGHT");
        ndt_list_add(nxt->xpfms, wpt);
        ndt_list_add(flp->cws,   wpt);
    }
    if (fabs(angl) > 180.)
    {
        dctd = ndt_distance_init(INT64_C(4242), NDT_ALTUNIT_ME); // sqrt(2) factor
        if (angl < 0.)
        {
            wpt = ndt_waypoint_pbd(src1, ndt_mod(brg1 - 135., 360.), dctd, now, wmm);
        }
        else
        {
            wpt = ndt_waypoint_pbd(src1, ndt_mod(brg1 + 135., 360.), dctd, now, wmm);
        }
        if (!wpt)
        {
            err = ENOMEM;
            goto end;
        }
        wpt->type = NDT_WPTYPE_LLC;
        snprintf(wpt->info.idnt, sizeof(wpt->info.idnt),
                 "(TURN %s)", angl < 0. ? "LEFT" : "RIGHT");
        ndt_list_add(nxt->xpfms, wpt);
        ndt_list_add(flp->cws,   wpt);
    }
    if (fabs(angl) > 270.)
    {
        dctd = ndt_distance_init(INT64_C(3000), NDT_ALTUNIT_ME);
        if (angl < 0.)
        {
            wpt = ndt_waypoint_pbd(src1, ndt_mod(brg1 - 180., 360.), dctd, now, wmm);
        }
        else
        {
            wpt = ndt_waypoint_pbd(src1, ndt_mod(brg1 + 180., 360.), dctd, now, wmm);
        }
        if (!wpt)
        {
            err = ENOMEM;
            goto end;
        }
        wpt->type = NDT_WPTYPE_LLC;
        snprintf(wpt->info.idnt, sizeof(wpt->info.idnt),
                 "(TURN %s)", angl < 0. ? "LEFT" : "RIGHT");
        ndt_list_add(nxt->xpfms, wpt);
        ndt_list_add(flp->cws,   wpt);
    }
    goto altitude;

// TODO: XP overfly dummy waypoint here (last, but before altitude computation) (for e.g. YFMS, maybe even QPAC down the road)

    /*
     * Update altitude based on distance flown for the previous leg.
     * Not extremely accurate but better than nothing: we climb until reaching
     * our cruise altitude, and start descending when reaching "top of descent".
     *
     * Since we don't compute top of descent yet, just assume it is the
     * STAR's first waypoint, or that of its enroute transition (if any).
     */
altitude:
    {
        // some compilers dislike variable declarations after a label
    }
    int ptype = leg->rsg->prc ? leg->rsg->prc->type : NDT_PROCTYPE_ENRTE;
    int64_t horiz, alt_prev = ndt_distance_get(*_alt, NDT_ALTUNIT_FT);
    ndt_distance climb, desct, interdis, totaldis = NDT_DISTANCE_ZERO;
    ndt_waypoint *dst_wpt, *src_wpt = legsrc;
    for (size_t i = 0; i < ndt_list_count(leg->xpfms); i++)
    {
        if (!(dst_wpt = ndt_list_item(leg->xpfms, i)))
        {
            err = ENOMEM;
            goto end;
        }
        interdis = ndt_position_calcdistance(src_wpt->position, dst_wpt ->position);
        totaldis = ndt_distance_add         (totaldis,                    interdis);
        src_wpt  = dst_wpt;
    }
    if (leg->dst)
    {
        interdis = ndt_position_calcdistance(src_wpt->position, leg->dst->position);
        totaldis = ndt_distance_add         (totaldis,                    interdis);
    }
    if (rwy) // no climb during takeoff roll
    {
        totaldis = ndt_distance_rem(totaldis, rwy->length);
    }
    {
        // make horizontal distance usable (native units for maximum precision)
        horiz = ndt_distance_get(totaldis, NDT_ALTUNIT_NA);
    }
    if (flp->arr.star.enroute.rsgt)
    {
        if (leg->rsg == flp->arr.star.enroute.rsgt &&
            leg      == ndt_list_item(leg->rsg->legs, 0))
        {
            *_alt = ndt_distance_max(*_alt, flp->crz_altitude); // dummy top of descent
        }
    }
    else if (flp->arr.star.rsgt)
    {
        if (leg->rsg == flp->arr.star.rsgt &&
            leg      == ndt_list_item(leg->rsg->legs, 0))
        {
            *_alt = ndt_distance_max(*_alt, flp->crz_altitude); // dummy top of descent
        }
    }
#define ALT1 (INT64_C(10000)) // speed transition
#define ALT2 (INT64_C(20000)) // +10,000
#define ALT3 (INT64_C(30000)) // +10,000
#define ALT4 (INT64_C(40000)) // +10,000
#define ALT5 (INT64_C(50000)) // +10,000
#define ALTM (INT64_C(60000)) // +10,000
#define CLB1 (INT64_C(10))  // <= 10,000, climb 4.5° (1:10 ratio)
#define CLB2 (INT64_C(15))  // <= 20,000, climb 3.0° (1:15 ratio)
#define CLB3 (INT64_C(30))  // <= 30,000, climb 1.5° (1:30 ratio)
#define CLB4 (INT64_C(45))  // <= 40,000, climb 1.0° (1:45 ratio)
#define CLB5 (INT64_C(60))  // <= 50,000, climb .75° (1:60 ratio)
#define CLBM (INT64_C(75))  // <= 60,000, climb 0.6° (1:75 ratio)
    switch (ptype)
    {
        // TODO: start descending after TOD, segment type is irrelevant
        case NDT_PROCTYPE_FINAL:
            if (leg->rsg->type == NDT_RSTYPE_MAP)
            {
                climb = ndt_distance_init(horiz / CLB1, NDT_ALTUNIT_NA);
                *_alt = ndt_distance_add(*_alt, climb); goto altitude_constraints;
            }
            // fall through
        case NDT_PROCTYPE_APPTR:
        case NDT_PROCTYPE_STAR1:
        case NDT_PROCTYPE_STAR2:
        case NDT_PROCTYPE_STAR3:
        case NDT_PROCTYPE_STAR4:
        case NDT_PROCTYPE_STAR5:
        case NDT_PROCTYPE_STAR6:
        case NDT_PROCTYPE_STAR7:
        case NDT_PROCTYPE_STAR8:
        case NDT_PROCTYPE_STAR9:
            {   // minimal 1-degree predicted altitude, used to determine descent rate
                alt_prev -= ndt_distance_get(ndt_distance_init(horiz / INT64_C(45), NDT_ALTUNIT_NA), NDT_ALTUNIT_FT);
            }
            if (alt_prev >= INT64_C(10000)) // descend 3.0° (1:15 ratio)
            {
                desct = ndt_distance_init(horiz / INT64_C(15), NDT_ALTUNIT_NA);
                *_alt = ndt_distance_rem(*_alt, desct); goto altitude_constraints;
            }
            // descend. 2.5° (1:18 ratio)
            desct = ndt_distance_init(horiz / INT64_C(18), NDT_ALTUNIT_NA);
            *_alt = ndt_distance_rem(*_alt, desct); goto altitude_constraints;

        /*
         * TODO: more realistic climb profile
         *
         * Calibr.: LSGG/23 GVA PAS GG602 MOLUS UN871 BERSU LOWI/08
         * S/Brief: A320-IAE, perf: M078, pax: 150, altitude: FL330
         *
         *     GVA       SimBrief FL029     navdconv     FL015
         *     PAS       SimBrief FL102     navdconv     FL062
         *     GG602     SimBrief FL129     navdconv     FL086
         *     TINAM     SimBrief FL223     navdconv     FL184
         *     MOLUS     SimBrief FL243     navdconv     FL216
         *     SOSAL     SimBrief FL266     navdconv     FL238
         *     TELNO     SimBrief FL298     navdconv     FL280
         *     KORED     SimBrief FL309     navdconv     FL295
         *     KONOL     SimBrief FL323     navdconv     FL314
         *     BERSU     SimBrief FL330     navdconv     FL330
         */
        default:
            if (alt_prev >= ndt_distance_get(flp->crz_altitude, NDT_ALTUNIT_FT))
            {
                goto altitude_constraints;  // already reached top of climb
            }
            {   // minimal 1-degree predicted altitude, used to determine climb rate
                alt_prev += ndt_distance_get(ndt_distance_init(horiz / INT64_C(45), NDT_ALTUNIT_NA), NDT_ALTUNIT_FT);
            }
            if (alt_prev <= ALT1)
            {
                climb = ndt_distance_init(horiz / CLB1, NDT_ALTUNIT_NA);
                *_alt = ndt_distance_add(*_alt, climb); goto altitude_climb;
            }
            if (alt_prev <= ALT2)
            {
                climb = ndt_distance_init(horiz / CLB2, NDT_ALTUNIT_NA);
                *_alt = ndt_distance_add(*_alt, climb); goto altitude_climb;
            }
            if (alt_prev <= ALT3)
            {
                climb = ndt_distance_init(horiz / CLB3, NDT_ALTUNIT_NA);
                *_alt = ndt_distance_add(*_alt, climb); goto altitude_climb;
            }
            if (alt_prev <= ALT4)
            {
                climb = ndt_distance_init(horiz / CLB4, NDT_ALTUNIT_NA);
                *_alt = ndt_distance_add(*_alt, climb); goto altitude_climb;
            }
            if (alt_prev <= ALT5)
            {
                climb = ndt_distance_init(horiz / CLB5, NDT_ALTUNIT_NA);
                *_alt = ndt_distance_add(*_alt, climb); goto altitude_climb;
            }
            climb = ndt_distance_init(horiz / CLBM, NDT_ALTUNIT_NA);
            *_alt = ndt_distance_add(*_alt, climb); goto altitude_climb;
    }

    /*
     * During climb, never exceed the specified cruise altitude (obviously).
     */
altitude_climb:
    *_alt = ndt_distance_min(*_alt, flp->crz_altitude);

    /*
     * Respect any and all altitude constraints specified by a procedure.
     */
altitude_constraints:
    switch (leg->type) // TODO: check if we're descending during hold
    {
        case NDT_LEGTYPE_CA:
            *_alt = ndt_distance_max(*_alt, leg->course. altitude);
            goto end;
        case NDT_LEGTYPE_FA:
            *_alt = ndt_distance_max(*_alt, leg->fix.    altitude);
            goto end;
        case NDT_LEGTYPE_HA:
            *_alt = ndt_distance_max(*_alt, leg->hold.   altitude);
            goto end;
        case NDT_LEGTYPE_VA:
            *_alt = ndt_distance_max(*_alt, leg->heading.altitude);
            goto end;
        default:
            break;
    }
    switch (leg->constraints.altitude.typ)
    {
        case NDT_RESTRICT_AT:
            *_alt =                         leg->constraints.altitude.min;
            break;
        case NDT_RESTRICT_AB:
            *_alt = ndt_distance_max(*_alt, leg->constraints.altitude.min);
            break;
        case NDT_RESTRICT_BT:
            *_alt = ndt_distance_max(*_alt, leg->constraints.altitude.min);
            // fall through
        case NDT_RESTRICT_BL:
            *_alt = ndt_distance_min(*_alt, leg->constraints.altitude.max);
            break;
        default:
            break;
    }

end:
    if (err)
    {
        char error[64];
        strerror_r(err, error, sizeof(error));
        ndt_log("route_leg_xpfms: failed to handle \"%s\" type '%d' from '%s' (%s)\n",
                leg    ? leg->info.desc    : NULL,
                leg    ? leg->type         : -1,
                legsrc ? legsrc->info.idnt : NULL, error);
        return NULL;
    }
    {
        leg->altitude = *_alt; // store final altitude in leg before returning
    }
    if (leg->dst)
    {
        return leg->dst;
    }
    if (leg->type == NDT_LEGTYPE_FM ||
        leg->type == NDT_LEGTYPE_HM ||
        leg->type == NDT_LEGTYPE_VM ||
        leg->type == NDT_LEGTYPE_ZZ)
    {
        return legsrc;
    }
    if ((leg->type == NDT_LEGTYPE_CI ||
         leg->type == NDT_LEGTYPE_VI) && !ndt_list_count(leg->xpfms))
    {
        // Navigraph 1511: some apptrs. end with CI leg (have nothing to intcpt)
        // intcpt. failure followed by CF (we ignore error but have no waypoint)
        return legsrc;
    }
    return ndt_list_item(leg->xpfms, -1);
}

static int list_add_legs(ndt_list *list, ndt_list *legs)
{
    if (!list || !legs)
    {
        return ENOMEM;
    }
    for (size_t i = 0; i < ndt_list_count(legs); i++)
    {
        ndt_route_leg *leg = ndt_list_item(legs, i);
        if (!leg)
        {
            return ENOMEM;
        }
        ndt_list_add(list, leg);
    }
    return 0;
}

static int route_leg_update(ndt_flightplan *flp)
{
    int  err = 0;
    if (!flp)
    {
        err = ENOMEM;
        goto end;
    }

    /* Flightplan's leg list doesn't own the legs, so cleanup is easy */
    ndt_list_empty(flp->legs);
    ndt_log("ARE WE SAFE???\n");//debug

    /*
     * Add all flightplan legs in order:
     *
     * - route segment:  SID (contains runway->SID and SID)
     * - route segment:  SID enroute transition
     * - route segments: enroute (airways, directs)
     * - route segment:  STAR enroute transition
     * - route segment:  STAR (contains STAR and STAR->runway)
     * - route segment:  approach transition
     * - route segment:  final approach procedure
     */
    if ((flp->dep.sid.rsgt) &&
        (err = list_add_legs(flp->legs, flp->dep.sid.rsgt->legs)))
    {
        goto end;
    }
    if ((flp->dep.sid.enroute.rsgt) &&
        (err = list_add_legs(flp->legs, flp->dep.sid.enroute.rsgt->legs)))
    {
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
        if ((err = list_add_legs(flp->legs, rsg->legs)))
        {
            goto end;
        }
    }
    if ((flp->arr.star.enroute.rsgt) &&
        (err = list_add_legs(flp->legs, flp->arr.star.enroute.rsgt->legs)))
    {
        goto end;
    }
    if ((flp->arr.star.rsgt) &&
        (err = list_add_legs(flp->legs, flp->arr.star.rsgt->legs)))
    {
        goto end;
    }
    if ((flp->arr.apch.transition.rsgt) &&
        (err = list_add_legs(flp->legs, flp->arr.apch.transition.rsgt->legs)))
    {
        goto end;
    }
    if ((flp->arr.apch.rsgt) &&
        (err = list_add_legs(flp->legs, flp->arr.apch.rsgt->legs)))
    {
        goto end;
    }

    /*
     * Resolve some overlaps; e.g. at KLAX:
     * - SADDE6 STAR: SADDE BAYST SMO JAVSI
     * - ILS 24L, SMO transition: SMO SAPPI JULLI
     * We try to merge this to: SADDE BAYST SMO SAPPI JULLI
     *
     * Future: in the above case, we remove legs from the STAR; if we change
     *         the approach/trans., we must remember to reload the STAR too.
     *
     * Note: size_t is unsigned and ndt_list_count() may return 0, so compare
     *       i + 1 to ndt_list_count() rather than i to ndt_list_count() - 1.
     */
    ndt_route_leg *leg;
    for (size_t i = 0; i + 1 < ndt_list_count(flp->legs); i++)
    {
        if (!(leg = ndt_list_item(flp->legs, i)))
        {
            err = ENOMEM;
            goto end;
        }
        if (!leg->dst)
        {
            continue; // leg without fix-based termination
        }
        for (size_t j = i + 1; j < ndt_list_count(flp->legs); j++)
        {
            ndt_route_leg *fleg = ndt_list_item(flp->legs, j);
            if (!fleg)
            {
                err = ENOMEM;
                goto end;
            }
            if (fleg->rsg && fleg->rsg->type == NDT_RSTYPE_DSC)
            {
                break; // don't handle overlaps when a manual disc. is present
            }
            if (!fleg->dst)
            {
                continue; // leg without fix-based termination
            }
            if ((fleg->dst == leg->dst) &&
                (fleg->                type == NDT_LEGTYPE_IF ||
                 fleg->constraints.waypoint == NDT_WPTCONST_IAF))
            {
                // we have an overlap and the incriminating fix is marked as
                // IAF in the future leg closing the overlap, remove any leg
                // between leg (included) and fleg (not included)
                // also remember k unsigned (same issue as above)
                for (size_t k = j; k > i; k--)
                {
                    ndt_route_leg *skip = ndt_list_item(flp->legs, k - 1);
                    if (!skip || !skip->rsg)
                    {
                        err = ENOMEM;
                        goto end;
                    }
                    ndt_list_rem(skip->rsg->legs, skip);
                    ndt_list_rem(flp->      legs, skip);
                    ndt_route_leg_close         (&skip);
                }
                i--; break; // fleg (next in the i loop) ends up at index ((i--)++)
            }
        }
        // we're here after the break (or if no overlap found)
        continue;
    }

    /*
     * Ensure endpoint consistency (if e.g. legs were added or removed);
     * the initial waypoint is always the departure airport or runway.
     */
    void         *wmm = flp->ndb->wmm;
    ndt_date      now = ndt_date_now();
    ndt_waypoint *src = flp->dep.rwy ? flp->dep.rwy->waypoint : flp->dep.apt->waypoint;
    for (size_t i = 0; i < ndt_list_count(flp->legs); i++)
    {
        ndt_route_leg *nxt = ndt_list_item(flp->legs, i + 1);
        if (!(leg = ndt_list_item(flp->legs, i)))
        {
            err = ENOMEM;
            goto end;
        }
        if ((nxt && nxt->rsg &&
             nxt->type == NDT_LEGTYPE_IF) &&
            (leg->type == NDT_LEGTYPE_CI ||
             leg->type == NDT_LEGTYPE_PI ||
             leg->type == NDT_LEGTYPE_VI))
        {
            /*
             * Navigraph 1511, LSZH, I16 approach, RILAX transition:
             *
             * CF,TRA,47.689500,8.436972,0,TRA,0.0,0.0,189.0,4.4,2,5000,0,0,0,0,0,0
             * CI,0,IZH,0.0,189.0,2,4000,0,0,0,0,0,0
             * IF,CI16,47.676192,8.397186,IZH,333.0,15.5,1,4000,0,0,0,0,0,0
             * CF,ENUSO,47.596417,8.452556,0,IZH,333.0,10.2,153.0,5.3,1,4000,0,0,0,0,2,0
             *
             * Ambiguous intercept:
             *
             * - IF has no course information, we can't intercept it
             * - as per chart, CI16 is NOT part of the transition+approach,
             *   it goes direct from TRA to C153° ENUSO
             * - other occurrences of transition that end with CI/PI/VI and
             *   final approach that starts with IF, just assume it is same
             *   situation as LSZH, should work fine for us (hopefully).
             */
            ndt_list_rem(nxt->rsg->legs, nxt);
            ndt_list_rem(flp->     legs, nxt);
            ndt_route_leg_close        (&nxt);
            continue;
        }
        if (leg->rsg && leg->rsg->type == NDT_RSTYPE_AWY)
        {
            /*
             * check for airway legs where leg->src doesn't match src;
             * this could happen, for example, when inserting a direct
             * or discontinuity during on-the-fly flight plan editing.
             *
             * if the route segment only had a single leg, then it
             * can easily be converted to a direct: problem solved!
             */
            if (leg->src != src && ndt_list_count(leg->rsg->legs) == 1)
            {
                leg->rsg->src     = src;
                leg->rsg->awy.awy = NULL;
                leg->rsg->awy.src = NULL;
                leg->rsg->awy.dst = NULL;
                leg->awyleg       = NULL;
                leg->rsg->type    = NDT_RSTYPE_DCT;
            }
        }
        if (leg->type != NDT_LEGTYPE_ZZ)
        {
            leg->src = src;
        }
        if (leg->src && leg->dst && !ndt_list_count(leg->xpfms))
        {
            // TODO: set distance even with xpfms dummies
            leg->dis = ndt_position_calcdistance(leg->src->position, leg->dst->position);
            leg->trb = ndt_position_calcbearing (leg->src->position, leg->dst->position);
            leg->imb = ndt_wmm_getbearing_mag   (wmm, leg->trb, leg->dst->position, now);
            leg->omb = ndt_wmm_getbearing_mag   (wmm, leg->trb, leg->src->position, now);
        }
        src = leg->dst;
    }

    /*
     * Future: consolidate airways here as necessary.
     *
     * Note: we can probably set a variable in the flight plan to enable/disable
     *       consolidation, as well as maybe a variable in the segment struct to
     *       determine if the airway was part of a segment that got split by our
     *       code vs. originally started out as actual single-leg airway segment
     *       (i.e. single-leg segment specified in coroute or manually by user).
     */

    /*
     * Dummy last leg from last leg's endpoint (if it exists) to the arrival
     * runway threshold (if set, else use arrival airport coordinates).
     */
    ndt_waypoint *dst = flp->arr.rwy ? flp->arr.rwy->waypoint : flp->arr.apt ? flp->arr.apt->waypoint : NULL;
    if (dst)
    {
        if (!flp->arr.last.rsgt)
        {
            if (!(flp->arr.last.rsgt = ndt_route_segment_direct(src, dst)))
            {
                err = ENOMEM;
                goto end;
            }
            if (!(flp->arr.last.rleg = ndt_list_item(flp->arr.last.rsgt->legs, 0)))
            {
                err = ENOMEM;
                goto end;
            }
        }
        flp->arr.last.rleg->src = src;
        flp->arr.last.rleg->dst = dst;
        if (src && dst)
        {
            leg      = flp->arr.last.rleg;
            leg->dis = ndt_position_calcdistance(leg->src->position, leg->dst->position);
            leg->trb = ndt_position_calcbearing (leg->src->position, leg->dst->position);
            leg->imb = ndt_wmm_getbearing_mag   (wmm, leg->trb, leg->dst->position, now);
            leg->omb = ndt_wmm_getbearing_mag   (wmm, leg->trb, leg->src->position, now);
        }
    }
    else if (flp->arr.last.rsgt)
    {
        ndt_route_segment_close(&flp->arr.last.rsgt);
        flp->arr.last.rleg = NULL;
    }

    /*
     * Set dummy xpfms waypoints.
     */
    ndt_waypoint *legsrc = flp->dep.rwy ? flp->dep.rwy->waypoint           : flp->dep.apt->waypoint;
    ndt_distance altitud = flp->dep.rwy ? flp->dep.rwy->threshold.altitude : flp->dep.apt->coordinates.altitude;
    for (size_t i = 0; i < ndt_list_count(flp->legs); i++)
    {
        ndt_route_leg *nxt = ndt_list_item(flp->legs, i + 1);
        if (!(leg = ndt_list_item(flp->legs, i)))
        {
            err = ENOMEM;
            goto end;
        }
        if (!(legsrc = route_leg_xpfms(flp, now, legsrc, leg, nxt, &altitud)))
        {
            err = EINVAL;
            goto end;
        }
    }

end:
    if (err)
    {
        char error[64];
        strerror_r(err, error, sizeof(error));
        ndt_log("flightplan: route_leg_update failed (%s)\n", error);
        return err;
    }
    ndt_log("SHIT GOT UPDATED\n");//debug
    return err;
}
