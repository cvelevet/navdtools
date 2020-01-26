/*
 * navdata.c
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
#include <strings.h>

#include "common/common.h"
#include "common/list.h"

#include "compat/compat.h"

#include "wmm/wmm.h"

#include "airport.h"
#include "airway.h"
#include "navdata.h"
#include "ndb_xpgns.h"
#include "waypoint.h"

static int compare_apt(const void *apt1, const void *apt2);
static int compare_awy(const void *awy1, const void *awy2);
static int compare_wpt(const void *wpt1, const void *wpt2);

ndt_navdatabase* ndt_navdatabase_init(const char *ndr, ndt_navdataformat fmt, ndt_date date)
{
    int  err = 0;
    char errbuf[64];

    ndt_navdatabase *ndb = calloc(1, sizeof(ndt_navdatabase));
    if (!ndb)
    {
        err = ENOMEM;
        goto end;
    }

    ndb->fmt       = fmt;
    ndb->root      = strdup(ndr);
    ndb->airports  = ndt_list_init();
    ndb->airways   = ndt_list_init();
    ndb->waypoints = ndt_list_init();

    if (!ndb->airports || !ndb->airways || !ndb->waypoints || !ndb->root)
    {
        err = ENOMEM;
        goto end;
    }

    if ((ndb->wmm = ndt_wmm_init(date)) == NULL)
    {
        ndt_log("navdata: failed to open World Magnetic Model\n");
        err = -1;
        goto end;
    }

    switch (fmt)
    {
        case NDT_NAVDFMT_XPGNS:
            if ((err = ndt_ndb_xpgns_navdatabase_init(ndb)))
            {
                goto end;
            }
            break;

        case NDT_NAVDFMT_OTHER:
        default:
            err = -1;
            goto end;
    }

    /*
     * While the navdata is usually already sorted, we don't know how, and some
     * lists are compiled from several source files, so we need to re-sort all
     * lists using a known method which can then be used for retrieving items.
     */
    ndt_list_sort(ndb->airports,  sizeof(ndt_airport*),  &compare_apt);
    ndt_list_sort(ndb->airways,   sizeof(ndt_airway*),   &compare_awy);
    ndt_list_sort(ndb->waypoints, sizeof(ndt_waypoint*), &compare_wpt);

#if 0
    /* Database is complete, test parsing of all procedures (slow) */
    for (size_t i = 0; i < ndt_list_count(ndb->airports); i++)
    {
        if (!ndt_navdata_init_airport(ndb, ndt_list_item(ndb->airports, i)))
        {
            err = EINVAL;
            goto end;
        }
    }
#endif

end:
    if (err)
    {
        ndt_navdatabase_close(&ndb);
        strerror_r(err, errbuf, sizeof(errbuf));
        ndt_log("navdata: failed to open database (%s)\n", errbuf);
    }
    return ndb;
}

void ndt_navdatabase_close(ndt_navdatabase **_ndb)
{
    if (_ndb && *_ndb)
    {
        size_t i;
        ndt_navdatabase *ndb = *_ndb;

        if (ndb->airports)
        {
            while ((i = ndt_list_count(ndb->airports)))
            {
                ndt_airport *apt = ndt_list_item(ndb->airports, i-1);
                ndt_list_rem                    (ndb->airports, apt);
                ndt_airport_close               (              &apt);
            }
            ndt_list_close(&ndb->airports);
        }

        if (ndb->airways)
        {
            while ((i = ndt_list_count(ndb->airways)))
            {
                ndt_airway* awy = ndt_list_item(ndb->airways, i-1);
                ndt_list_rem                   (ndb->airways, awy);
                ndt_airway_close               (             &awy);
            }
            ndt_list_close(&ndb->airways);
        }

        if (ndb->waypoints)
        {
            while ((i = ndt_list_count(ndb->waypoints)))
            {
                ndt_waypoint *wpt = ndt_list_item(ndb->waypoints, i-1);
                ndt_list_rem                     (ndb->waypoints, wpt);
                ndt_waypoint_close               (               &wpt);
            }
            ndt_list_close(&ndb->waypoints);
        }

        if (ndb->root)
        {
            free(ndb->root);
        }

        if (ndb->wmm)
        {
            ndt_wmm_close(&ndb->wmm);
        }

        free(ndb);

        *_ndb = NULL;
    }
}

void ndt_navdata_add_waypoint(ndt_navdatabase *ndb, ndt_waypoint *wpt)
{
    if (ndb && wpt)
    {
        ndt_waypoint *wp; int ii;
        ndt_list *l = ndb->waypoints;
        size_t jj = ndt_list_count(l);
        for (ii = 0; ii < jj; ii++)
        {
            if ((wp = ndt_list_item(l, ii)) && (compare_wpt(&wpt, &wp) < 0))
            {
                break;
            }
        }
        ndt_list_insert(l, wpt, ii);
    }
}

void ndt_navdata_rem_waypoint(ndt_navdatabase *ndb, ndt_waypoint *wpt)
{
    if (ndb && wpt)
    {
        ndt_list_rem(ndb->waypoints, wpt);
    }
}

int ndt_navdata_user_airport(ndt_navdatabase *ndb, const char *idnt, const char *misc, ndt_position coordinates)
{
    if (ndb && misc && idnt && *idnt)
    {
        if (ndt_navdata_get_airport(ndb, idnt))
        {
            ndt_log("navdata: can't add user airport \"%s\" (ICAO in use)\n", idnt);
            return EINVAL;
        }
        ndt_airport *apt = ndt_airport_init();
        if (!apt)
        {
            return -1; // ndt_airport_init will log the error
        }
        else
        {
            apt->coordinates = coordinates;
            apt->tr_altitude = NDT_DISTANCE_ZERO;
            apt->trans_level = NDT_DISTANCE_ZERO;
            apt->rwy_longest = NDT_DISTANCE_ZERO;
            snprintf(apt->info.idnt, sizeof(apt->info.idnt), "%s", idnt);
            snprintf(apt->info.misc, sizeof(apt->info.misc), "%s", misc);
            int feet = ndt_distance_get(ndt_position_getaltitude(coordinates), NDT_ALTUNIT_FT);
            snprintf(apt->info.desc, sizeof(apt->info.desc), "Airport %s (%s), elevation %d", idnt, misc, feet);
            {
                ndt_airport *ap; int ii;
                ndt_list *l = ndb->airports;
                size_t jj = ndt_list_count(l);
                for (ii = 0; ii < jj; ii++)
                {
                    if ((ap = ndt_list_item(l, ii)) && (compare_apt(&apt, &ap) < 0))
                    {
                        break;
                    }
                }
                ndt_list_insert(l, apt, ii);
            }
        }
        ndt_waypoint *wpt = ndt_navdata_get_wptnear2(ndb, idnt, NULL, coordinates);
        if (!wpt || wpt->type != NDT_WPTYPE_XPA)
        {
            if ((wpt = ndt_waypoint_init()) == NULL)
            {
                return -1; // ndt_waypoint_init will log the error
            }
            // set sorting-relevant information prior to adding
            strncpy(wpt->info.idnt, idnt, sizeof(wpt->info.idnt));
            wpt->type = NDT_WPTYPE_XPA; ndt_navdata_add_waypoint(ndb, wpt);
        }
        // update waypoint information to match corresponding airport
        strncpy(wpt->info.desc, apt->info.desc, sizeof(wpt->info.desc));
        strncpy(wpt->info.misc, apt->info.misc, sizeof(wpt->info.misc));
        wpt->position = apt->coordinates;
        apt->waypoint = wpt;
        return 0;
    }
    return ENOMEM;
}

ndt_airport* ndt_navdata_get_airport(ndt_navdatabase *ndb, const char *idt)
{
    if (idt)
    {
        for (size_t i = 0; i < ndt_list_count(ndb->airports); i++)
        {
            ndt_airport *apt = ndt_list_item (ndb->airports,  i);
            if (apt)
            {
                int cmp  = strcmp(idt, apt->info.idnt);
                if (cmp <= 0)
                {
                    if (!cmp)
                    {
                        return apt;
                    }
                    break;
                }
            }
        }
    }

    return NULL;
}

ndt_airport* ndt_navdata_init_airport(ndt_navdatabase *ndb, ndt_airport *apt)
{
    if (!ndb || !apt)
    {
        return NULL;
    }

    switch (ndb->fmt)
    {
        case NDT_NAVDFMT_XPGNS:
            if (NULL == (apt = ndt_ndb_xpgns_navdata_init_airport(ndb, apt)))
            {
                return NULL;
            }
            break;

        case NDT_NAVDFMT_OTHER:
        default:
            return NULL;
    }

    /*
     * Recompute all runway headings from coordinates. This allows using:
     * - old navigation databases with newer magnetic models (e.g. navdconv)
     * - new navigation databases with older magnetic models (e.g. X-Plane 10)
     */
    ndt_list *tmp = ndt_list_init(); ndt_runway *rwy, *couples[10][2];
    if (tmp == NULL)
    {
        return NULL;
    }
    for (size_t i = 0; i < 10; i++)
    {
        couples[i][0] = NULL;
        couples[i][1] = NULL;
    }
    for (size_t i = 0; i < ndt_list_count(apt->runways); i++)
    {
        if ((rwy = ndt_list_item(apt->runways, i)))
        {
            ndt_list_add(tmp, rwy);
        }
    }
    for (size_t i = 0; i < 10; i++)
    {
        char rwy_recipr[4], rwy_suffix, scrap; int rwy_number;
        if ((ndt_list_count(tmp) > 0) && (rwy = ndt_list_item(tmp, 0)))
        {
            switch (sscanf(rwy->info.idnt, "%2d%c%c", &rwy_number, &rwy_suffix, &scrap))
            {
                case 2:
                    break;
                case 1:
                    rwy_suffix = 0;
                    break;
                default:
                    ndt_log("ERROR: ndt_navdata_init_airport: unsupported runway identifier \"%s\"\n", rwy->info.idnt);
                    return NULL;
            }
            if (rwy_suffix)
            {
                switch (rwy_suffix)
                {
                    case 'L':
                        rwy_suffix = 'R';
                        break;
                    case 'R':
                        rwy_suffix = 'L';
                        break;
                    case 'C':
                    case 'T':
                        break;
                    default:
                        ndt_log("ERROR: ndt_navdata_init_airport: unsupported runway identifier \"%s\"\n", rwy->info.idnt);
                        return NULL;
                }
            }
            if (rwy_number < 1 || rwy_number > 36)
            {
                ndt_log("ERROR: ndt_navdata_init_airport: unsupported runway identifier \"%s\"\n", rwy->info.idnt);
                return NULL;
            }
            else if (rwy_number <= 18)
            {
                rwy_number += 18;
            }
            else
            {
                rwy_number -= 18;
            }
            if (sizeof(rwy_recipr) <= snprintf(rwy_recipr, sizeof(rwy_recipr), "%02d%c", rwy_number, rwy_suffix))
            {
                ndt_log("ERROR: ndt_navdata_init_airport: unsupported reciprocal runway \"%s\" for \"%s\"\n", rwy_recipr, rwy->info.idnt);
                return NULL;
            }
            else
            {
                couples[i][0] = rwy;
                couples[i][1] = ndt_runway_get(tmp, rwy_recipr);
            }
            if (couples[i][0]) ndt_list_rem(tmp, couples[i][0]);
            if (couples[i][1]) ndt_list_rem(tmp, couples[i][1]);
        }
    }
    for (size_t i = 0; i < 10; i++)
    {
        if (couples[i][0])
        {
            if (couples[i][1])
            {
                couples[i][0]->tru_heading = ndt_position_calcbearing(couples[i][0]->threshold, couples[i][1]->threshold);
                couples[i][0]->mag_heading = ndt_wmm_getbearing_mag(ndb->wmm, couples[i][0]->tru_heading, couples[i][0]->threshold);
                couples[i][1]->tru_heading = ndt_position_calcbearing(couples[i][1]->threshold, couples[i][0]->threshold);
                couples[i][1]->mag_heading = ndt_wmm_getbearing_mag(ndb->wmm, couples[i][1]->tru_heading, couples[i][1]->threshold);
            }
            else
            {
                couples[i][0]->tru_heading = ndt_wmm_getbearing_tru(ndb->wmm, couples[i][0]->ndb_heading, couples[i][0]->threshold);
                couples[i][0]->mag_heading = couples[i][0]->ndb_heading;
            }
        }
    }
    ndt_list_empty(tmp);
    ndt_list_close(&tmp);
    return apt;
}

ndt_airway* ndt_navdata_get_airway(ndt_navdatabase *ndb, const char *idt, size_t *idx)
{
    if (idt)
    {
        for (size_t i = idx ? *idx : 0; i < ndt_list_count(ndb->airways); i++)
        {
            ndt_airway *awy = ndt_list_item(ndb->airways, i);
            if (awy)
            {
                int cmp  = strcmp(idt, awy->info.idnt);
                if (cmp <= 0)
                {
                    if (!cmp)
                    {
                        if (idx) *idx = i;
                        return awy;
                    }
                    break;
                }
            }
        }
    }

    return NULL;
}

ndt_waypoint* ndt_navdata_get_waypoint(ndt_navdatabase *ndb, const char *idt, size_t *idx)
{
    if (idt)
    {
        for (size_t i = idx ? *idx : 0; i < ndt_list_count(ndb->waypoints); i++)
        {
            ndt_waypoint *wpt = ndt_list_item(ndb->waypoints, i);
            if (wpt)
            {
                int cmp  = strcmp(idt, wpt->info.idnt);
                if (cmp <= 0)
                {
                    if (!cmp)
                    {
                        if (idx) *idx = i;
                        return wpt;
                    }
                    break;
                }
            }
        }
    }

    return NULL;
}

ndt_waypoint* ndt_navdata_get_wptnear2(ndt_navdatabase *ndb, const char *idt, size_t *idx, ndt_position pos)
{
    if (idt)
    {
        ndt_waypoint *wpt = NULL, *next;
        int64_t       min = INT64_MAX;

        for (size_t i = idx ? *idx : 0; (next = ndt_navdata_get_waypoint(ndb, idt, &i)); i++)
        {
            int64_t dist = ndt_distance_get(ndt_position_calcdistance(pos, next->position), NDT_ALTUNIT_NA);

            if (dist < min)
            {
                if (idx)
                {
                    *idx = i;
                }
                min = dist;
                wpt = next;
            }
        }

        return wpt;
    }

    return NULL;
}

ndt_waypoint* ndt_navdata_get_wpt4pos(ndt_navdatabase *ndb, const char *idt, size_t *idx, ndt_position pos)
{
    if (idt)
    {
        ndt_waypoint *wpt;

        for (size_t i = idx ? *idx : 0; (wpt = ndt_navdata_get_waypoint(ndb, idt, &i)); i++)
        {
            if (!ndt_distance_get(ndt_position_calcdistance(wpt->position, pos), NDT_ALTUNIT_NA))
            {
                if (idx) *idx = i;
                return wpt;
            }
        }
    }

    return NULL;
}

ndt_waypoint* ndt_navdata_get_wpt4aws(ndt_navdatabase *ndb, ndt_waypoint *src, const char *awy2id, const char *awyidt, ndt_airway **_awy, ndt_airway_leg **_in, ndt_airway_leg **_out)
{
    ndt_airway_leg *out,  *in, *last_in = NULL;
    ndt_airway     *awy1, *awy2;
    ndt_waypoint   *dst;

    if (!ndb || !awy2id || !awyidt)
    {
        goto end;
    }

    for (size_t awy1idx = 0; (awy1 = ndt_navdata_get_airway(ndb, awyidt, &awy1idx)); awy1idx++)
    {
        if ((in = ndt_airway_startpoint(awy1, src->info.idnt, src->position)))
        {
            for (size_t awy2idx = 0; (awy2 = ndt_navdata_get_airway(ndb, awy2id, &awy2idx)); awy2idx++)
            {
                if ((out = ndt_airway_intersect(in, awy2)))
                {
                    if ((dst = ndt_navdata_get_wpt4pos(ndb, out->out.info.idnt, NULL, out->out.position)))
                    {
                        if (_awy) *_awy = awy1;
                        if (_in)  *_in  =   in;
                        if (_out) *_out =  out;
                        return dst;
                    }
                }
            }
            last_in = in;
        }
    }

    if (last_in)
    {
        ndt_log("ndt_navdata_get_wpt4aws: airways '%s', '%s' have no intersection\n",
                awyidt, awy2id);
    }
    else
    {
        ndt_log("ndt_navdata_get_wpt4aws: invalid startpoint '%s' for airway '%s'\n",
                src->info.idnt, awyidt);
    }

end:
    return NULL;
}

ndt_waypoint* ndt_navdata_get_wpt4awy(ndt_navdatabase *ndb, ndt_waypoint *src, const char *dstidt, const char *awyidt, ndt_airway **_awy, ndt_airway_leg **_in, ndt_airway_leg **_out)
{
    ndt_airway_leg *out, *in, *last_in = NULL;
    ndt_airway     *awy;
    ndt_waypoint   *dst;

    if (!ndb || !src || !dstidt || !awyidt)
    {
        goto end;
    }

    for (size_t awyidx = 0; (awy = ndt_navdata_get_airway(ndb, awyidt, &awyidx)); awyidx++)
    {
        if ((in = ndt_airway_startpoint(awy, src->info.idnt, src->position)))
        {
            for (size_t dstidx = 0; (dst = ndt_navdata_get_waypoint(ndb, dstidt, &dstidx)); dstidx++)
            {
                if ((out = ndt_airway_endpoint(in, dst->info.idnt, dst->position)))
                {
                    if (_awy) *_awy = awy;
                    if (_in)  *_in  =  in;
                    if (_out) *_out = out;
                    return dst;
                }
            }
            last_in = in;
        }
    }

    if (last_in)
    {
        ndt_log("ndt_navdata_get_wpt4awy: invalid endpoint '%s' for airway '%s'\n",
                dstidt, awyidt);
    }
    else
    {
        ndt_log("ndt_navdata_get_wpt4awy: invalid startpoint '%s' for airway '%s'\n",
                src->info.idnt, awyidt);
    }

end:
    return NULL;
}

static int compare_apt(const void *p1, const void *p2)
{
    ndt_airport *apt1 = *(ndt_airport**)p1;
    ndt_airport *apt2 = *(ndt_airport**)p2;

    // there shouldn't be any duplicates, but use latitude for determinism
    int cmp = strcmp(apt1->info.idnt, apt2->info.idnt);
    if (cmp)
    {
        return cmp;
    }
    return (fabs(ndt_position_getlatitude(apt1->coordinates, NDT_ANGUNIT_DEG)) <
            fabs(ndt_position_getlatitude(apt2->coordinates, NDT_ANGUNIT_DEG)));
}

static int compare_awy(const void *p1, const void *p2)
{
    ndt_airway *awy1 = *(ndt_airway**)p1;
    ndt_airway *awy2 = *(ndt_airway**)p2;

    // duplicates handled by the getter, no need for determinism
    return strcmp(awy1->info.idnt, awy2->info.idnt);
}

static int compare_wpt(const void *p1, const void *p2)
{
    ndt_waypoint *wpt1 = *(ndt_waypoint**)p1;
    ndt_waypoint *wpt2 = *(ndt_waypoint**)p2;

    // sort by name
    int cmp = strcmp(wpt1->info.idnt, wpt2->info.idnt);
    if (cmp)
    {
        return cmp;
    }

    // then by type, from highest to lowest priority
    switch (wpt1->type)
    {
        case NDT_WPTYPE_FIX:
            if (wpt2->type == NDT_WPTYPE_FIX)
            {
                goto latitude;
            }
            return -1;

        case NDT_WPTYPE_APT:
        case NDT_WPTYPE_XPA:
            if (wpt2->type == NDT_WPTYPE_APT)
            {
                goto latitude;
            }
            return -1;

        case NDT_WPTYPE_VOR:
            if (wpt2->type == NDT_WPTYPE_VOR)
            {
                goto latitude;
            }
            return -1;

        case NDT_WPTYPE_NDB:
            if (wpt2->type == NDT_WPTYPE_NDB)
            {
                goto latitude;
            }
            return -1;

        case NDT_WPTYPE_DME:
            if (wpt2->type == NDT_WPTYPE_DME)
            {
                goto latitude;
            }
            return -1;

        default: // anything else
            if (wpt2->type == wpt1->type)
            {
                goto latitude;
            }
            if (wpt2->type < wpt1->type)
            {
                return 1;
            }
            return -1;
    }

latitude:
    // same name and type: use latitude for determinism
    return (fabs(ndt_position_getlatitude(wpt1->position, NDT_ANGUNIT_DEG)) <
            fabs(ndt_position_getlatitude(wpt2->position, NDT_ANGUNIT_DEG)));
}
