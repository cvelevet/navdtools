/*
 * YFSfpln.c
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2016 Timothy D. Walker and others.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/common.h"
#include "lib/airport.h"
#include "lib/flightplan.h"
#include "lib/navdata.h"

#include "YFSfpln.h"
#include "YFSmain.h"
#include "YFSspad.h"

static void yfs_lsk_callback_fpln(yfms_context *yfms, int key[2],                intptr_t refcon);
static void fpl_spc_callback_lnup(yfms_context *yfms                                            );
static void fpl_spc_callback_lndn(yfms_context *yfms                                            );
static void fpl_print_leg_generic(yfms_context *yfms, int row,                ndt_route_leg *leg);
static void fpl_print_airport_rwy(yfms_context *yfms, int row, ndt_airport *apt, ndt_runway *rwy);
static int  fpl_getindex_for_line(yfms_context *yfms, int line                                  );

static void xplm_flpn_sync(yfms_context *yfms)
{
    /*
     * There may be more than 100 waypoints in our plan, we need to determine
     * the first and last legs so our waypoint count is limited to 99 or less.
     */
    int total_wpnts_count = 0, first_leg_idx = -1, last_leg_idx = 0;
    ndt_route_leg *leg; ndt_waypoint *wpt;yfms->data.fpln.xplm_last = -1;
    for (int i = 0, j = ndt_list_count(yfms->data.fpln.legs); i < j; i++)
    {
        if (i < yfms->data.fpln.lg_idx)
        {
            continue;
        }
        if ((leg = ndt_list_item(yfms->data.fpln.legs, i)))
        {
            if (99 <= total_wpnts_count + 1)
            {
                break;
            }
            total_wpnts_count += ndt_list_count(leg->xpfms) + !!leg->dst + !!leg->xpovf;
            last_leg_idx = i;
        }
    }
    for (int i = last_leg_idx, total_wpnts_count = 0; i >= 0; i--)
    {
        if ((leg = ndt_list_item(yfms->data.fpln.legs, i)))
        {
            if (99 <= total_wpnts_count + 1)
            {
                break;
            }
            total_wpnts_count += ndt_list_count(leg->xpfms) + !!leg->dst + !!leg->xpovf;
            first_leg_idx = i - 1;
        }
    }
    if (first_leg_idx > yfms->data.fpln.lg_idx - 1)
    {
        first_leg_idx = yfms->data.fpln.lg_idx - 1;
    }
    if (last_leg_idx  < yfms->data.fpln.lg_idx + 1)
    {
        last_leg_idx  = yfms->data.fpln.lg_idx + 1;
    }

    /* Now we must fill our internal backend with the selected legs */
    for (int i = first_leg_idx; i <= last_leg_idx; i++)
    {
        if (i < 0)
        {
            /* Store this leg twice - XPLM list 0-based but doesn't display/track item 0 */
            if (yfms->ndt.flp.rte->dep.apt && (wpt = yfms->ndt.flp.rte->dep.apt->waypoint))
            {
                if (yfms->data.fpln.xplm_last >= 99)
                {
                    break;
                }
                yfms->data.fpln.xplm_last++;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud = (float)ndt_position_getlongitude(wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude = (float)ndt_position_getlatitude (wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = XPLM_NAV_NOT_FOUND;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].cst = ndt_leg_const_init();
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].legindex = i;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].altitude = 0;
                if (wpt->xplm.refSet)
                {
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = wpt->xplm.navRef;
                }
                else
                {
                    XPLMNavRef waypnt;
                    if (XPLM_NAV_NOT_FOUND != (waypnt = XPLMFindNavAid(NULL, wpt->info.idnt,
                                                                       &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude,
                                                                       &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud, NULL, xplm_Nav_Airport)))
                    {
                        float la, lo; XPLMGetNavAidInfo(waypnt, NULL, &la, &lo, NULL, NULL, NULL, NULL, NULL, NULL);
                        ndt_distance dist = ndt_position_calcdistance(wpt->position, ndt_position_init((double)la, (double)lo, NDT_DISTANCE_ZERO));
                        if (INT64_C(3) > ndt_distance_get(dist, NDT_ALTUNIT_NM))
                        {
                            yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = waypnt; // a furlong (660ft) or less from waypoint
                        }
                    }
                    wpt->xplm.navRef = yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint;
                    wpt->xplm.refSet = 1;
                }
            }
            if ((yfms->ndt.flp.rte->dep.rwy && (wpt = yfms->ndt.flp.rte->dep.rwy->waypoint)) ||
                (yfms->ndt.flp.rte->dep.apt && (wpt = yfms->ndt.flp.rte->dep.apt->waypoint)))
            {
                if (yfms->data.fpln.xplm_last >= 99)
                {
                    break;
                }
                yfms->data.fpln.xplm_last++;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud = (float)ndt_position_getlongitude(wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude = (float)ndt_position_getlatitude (wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = wpt->xplm.refSet ? wpt->xplm.navRef : XPLM_NAV_NOT_FOUND;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].cst = ndt_leg_const_init();
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].legindex = i;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].altitude = 0;
            }
        }
        else if ((leg = ndt_list_item(yfms->data.fpln.legs, i)))
        {
            for (int j = 0, k = ndt_list_count(leg->xpfms); j < k; j++)
            {
                if ((wpt = ndt_list_item(leg->xpfms, i)))
                {
                    if (yfms->data.fpln.xplm_last >= 99)
                    {
                        break;
                    }
                    yfms->data.fpln.xplm_last++;
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud = (float)ndt_position_getlongitude(wpt->position, NDT_ANGUNIT_DEG);
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude = (float)ndt_position_getlatitude (wpt->position, NDT_ANGUNIT_DEG);
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = XPLM_NAV_NOT_FOUND;
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].cst = leg->constraints;
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].legindex = i;
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].altitude = 0;
                }
            }
            if ((wpt = leg->dst))
            {
                if (yfms->data.fpln.xplm_last >= 99)
                {
                    break;
                }
                yfms->data.fpln.xplm_last++;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud = (float)ndt_position_getlongitude(wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude = (float)ndt_position_getlatitude (wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = XPLM_NAV_NOT_FOUND;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].cst = leg->constraints;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].legindex = i;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].altitude = 0;
                if (wpt->xplm.refSet)
                {
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = wpt->xplm.navRef;
                }
                else
                {
                    XPLMNavRef waypnt;
                    switch (wpt->type)
                    {
                        case NDT_WPTYPE_APT:
                            waypnt = XPLMFindNavAid(NULL, wpt->info.idnt,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud, NULL, xplm_Nav_Airport);
                            break;
                        case NDT_WPTYPE_NDB:
                            waypnt = XPLMFindNavAid(NULL, wpt->info.idnt,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud, NULL, xplm_Nav_NDB);
                            break;
                        case NDT_WPTYPE_VOR:
                            waypnt = XPLMFindNavAid(NULL, wpt->info.idnt,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud, NULL, xplm_Nav_VOR);
                            break;
                        case NDT_WPTYPE_LOC:
                            waypnt = XPLMFindNavAid(NULL, wpt->info.idnt,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud, NULL, xplm_Nav_ILS|xplm_Nav_Localizer);
                            break;
                        case NDT_WPTYPE_FIX:
                            waypnt = XPLMFindNavAid(NULL, wpt->info.idnt,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud, NULL, xplm_Nav_Fix);
                            break;
                        case NDT_WPTYPE_DME:
                            waypnt = XPLMFindNavAid(NULL, wpt->info.idnt,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude,
                                                    &yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud, NULL, xplm_Nav_DME);
                            break;
                        default:
                            waypnt = XPLM_NAV_NOT_FOUND;
                            break;
                    }
                    if (waypnt != XPLM_NAV_NOT_FOUND)
                    {
                        float la, lo; XPLMGetNavAidInfo(waypnt, NULL, &la, &lo, NULL, NULL, NULL, NULL, NULL, NULL);
                        ndt_distance d = ndt_position_calcdistance(wpt->position, ndt_position_init((double)la, (double)lo, NDT_DISTANCE_ZERO));
                        if ((INT64_C(003) > ndt_distance_get(d, NDT_ALTUNIT_NM) && wpt->type == NDT_WPTYPE_APT) ||
                            (INT64_C(660) > ndt_distance_get(d, NDT_ALTUNIT_FT)))
                        {
                            yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = waypnt; // a furlong (660ft) or less from waypoint
                        }
                    }
                    wpt->xplm.navRef = yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint;
                    wpt->xplm.refSet = 1;
                }
            }
            if ((wpt = leg->xpovf))
            {
                if (yfms->data.fpln.xplm_last >= 99)
                {
                    break;
                }
                yfms->data.fpln.xplm_last++;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud = (float)ndt_position_getlongitude(wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude = (float)ndt_position_getlatitude (wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].legindex = i == last_leg_idx ? i : i + 1; // not 100% part of this leg
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = XPLM_NAV_NOT_FOUND;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].cst = leg->constraints;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].altitude = 0;
            }
        }
    }

    /* TODO: VNAV */

    /* Clear all FMS entries - replace them w/our own */
    for (int i = XPLMCountFMSEntries() - 1; i >= 0; i--)
    {
        XPLMClearFMSEntry(i);
    }
    for (int i = 0, j = 1; i <= yfms->data.fpln.xplm_last; i++)
    {
        if (yfms->data.fpln.xplm_info[i].waypoint != XPLM_NAV_NOT_FOUND)
        {
            XPLMSetFMSEntryInfo(i,
                                yfms->data.fpln.xplm_info[i].waypoint,
                                yfms->data.fpln.xplm_info[i].altitude);
        }
        else
        {
            XPLMSetFMSEntryLatLon(i,
                                  yfms->data.fpln.xplm_info[i].latitude,
                                  yfms->data.fpln.xplm_info[i].longitud,
                                  yfms->data.fpln.xplm_info[i].altitude);
        }
        if (i && j && yfms->data.fpln.xplm_info[i].legindex >= yfms->data.fpln.lg_idx)
        {
            if (yfms->data.fpln.xplm_info[i].legindex > yfms->data.fpln.lg_idx)
            {
                j = 0; // destination now set to last waypoint of previous leg
            }
            if (j)
            {
                XPLMSetDestinationFMSEntry(i);
            }
        }
    }
    return; // should be fully synced now
}

void yfs_fpln_pageopen(yfms_context *yfms)
{
    if (yfms == NULL)
    {
        return; // no error
    }
    if (yfs_main_newpg(yfms, PAGE_FPLN))
    {
        return;
    }
    yfms->data.fpln.ln_off = 0;
    yfms->lsks[0][0].cback = yfms->lsks[1][0].cback =
    yfms->lsks[0][1].cback = yfms->lsks[1][1].cback =
    yfms->lsks[0][2].cback = yfms->lsks[1][2].cback =
    yfms->lsks[0][3].cback = yfms->lsks[1][3].cback =
    yfms->lsks[0][4].cback = yfms->lsks[1][4].cback =
    yfms->lsks[0][5].cback/*yfms->lsks[1][5].cback*/= (YFS_LSK_f)&yfs_lsk_callback_fpln;
    yfms->spcs. cback_lnup = (YFS_SPC_f)&fpl_spc_callback_lnup;
    yfms->spcs. cback_lndn = (YFS_SPC_f)&fpl_spc_callback_lndn;
    yfs_fpln_pageupdt(yfms); return;
}

void yfs_fpln_pageupdt(yfms_context *yfms)
{
    /* reset lines, set page title */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }

    /* ensure we're still synced w/navdlib, reset if necessary */
    yfs_fpln_fplnsync(yfms);

    /* update indexes for tracked and displayed legs */
    if (yfms->data.init.ialized)
    {
        int t = XPLMGetDestinationFMSEntry(),s = fpl_getindex_for_line(yfms, 1);
        if (t < yfms->data.fpln.xplm_last + 1)
        {
            yfms->data.fpln.lg_idx = yfms->data.fpln.xplm_info[t].legindex;     // track
        }
        for (int i = 1, set = 0; i <= yfms->data.fpln.xplm_last; i++)
        {
            if (set && yfms->data.fpln.xplm_info[i].legindex > s)
            {
                break;
            }
            if (yfms->data.fpln.xplm_info[i].legindex >= s)
            {
                XPLMSetDisplayedFMSEntry(i); set = 1;                           // display
            }
        }
    }

    /* mostly static data */
    if (fpl_getindex_for_line(yfms, 0) == yfms->data.fpln.lg_idx - 1)
    {
        yfs_printf_lft(yfms, 0, 0, COLR_IDX_WHITE, "%s", " FROM");
    }
    {
        if (yfms->data.init.flight_id[0])
        {
            yfs_printf_rgt(yfms, 0, 0, COLR_IDX_WHITE, "%s   ", yfms->data.init.flight_id);
//          yfs_printf_rgt(yfms, 0, 0, COLR_IDX_WHITE, "%s ->", yfms->data.init.flight_id);
        }
        yfs_printf_lft(yfms, 11, 0, COLR_IDX_WHITE, "%s", " DEST   TIME  DIST  EFOB");
        yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "%s", "        ----  ----  ----");
    }
    if (yfms->data.init.ialized == 0)
    {
        if (yfms->data.fpln.ln_off)
        {
            yfms->data.fpln.ln_off = 0; yfs_fpln_pageupdt(yfms); return;
        }
        yfs_printf_lft(yfms, 2, 0, COLR_IDX_WHITE, "%s", "------END OF F-PLN------");
        return;
    }

    /* final destination :D */
    yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "%-4s%-3s ----  ----  ----", // TODO: TIME, DIST
                   yfms->ndt.flp.rte->arr.apt->info.idnt,
                   yfms->ndt.flp.rte->arr.rwy ? yfms->ndt.flp.rte->arr.rwy->info.idnt : "");

    /* two lines per leg (header/course/distance, then name/constraints) */
    for (int i = 0; i < 5; i++)
    {
        ndt_route_leg *leg = NULL;
        int have_waypt = 0, index;
        switch ((index = fpl_getindex_for_line(yfms, i)))
        {
            case -2:
                yfs_printf_lft(yfms, (2 * (i + 1)), 0, COLR_IDX_WHITE, "%s", "------END OF F-PLN------");
                break;
            case -1: // departure airport
                fpl_print_airport_rwy(yfms, (2 * (i + 1)), yfms->ndt.flp.rte->dep.apt, yfms->ndt.flp.rte->dep.rwy);
                have_waypt = 1;
                break;
            default: // regular leg
                if (index == yfms->data.fpln.dindex)
                {
                    fpl_print_airport_rwy(yfms, (2 * (i + 1)), yfms->ndt.flp.rte->arr.apt, yfms->ndt.flp.rte->arr.rwy);
                    leg = yfms->data.fpln.d_leg;
                    have_waypt = 1;
                    break;
                }
                if ((leg = ndt_list_item(yfms->data.fpln.legs, index)) && (leg->type == NDT_LEGTYPE_ZZ))
                {
                    yfs_printf_lft(yfms, (2 * (i + 1)), 0, COLR_IDX_WHITE, "%s", "---F-PLN DISCONTINUITY--");
                    break;
                }
                fpl_print_leg_generic(yfms, (2 * (i + 1)), leg);
                have_waypt = 1;
                break;
        }
        if (have_waypt)
        {
            if (index == yfms->data.fpln.lg_idx)
            {
                // should be the full line (YFS_DISPLAY_NUMC), but I prefer this
                // limiting to 12 characters avoids conflict w/constraint colors
                for (int j = 0; j < 12; j++)
                {
                    yfms->mwindow.screen.colr[(2 * (i + 1))][j] = COLR_IDX_WHITE;
                }
            }
            if (leg)
            {
                double  distance_nmile = (double)ndt_distance_get(leg->dis, NDT_ALTUNIT_ME) / 1852.;
                switch (leg->rsg->type)
                {
                    case NDT_RSTYPE_PRC: // TODO
//                      switch (leg->type)
                        break;
                    case NDT_RSTYPE_AWY://fixme also print bearing next to airway identifier???
                        yfs_printf_lft(yfms, ((2 * (i + 1)) - 1), 0, COLR_IDX_WHITE, " %5s", leg->awyleg->awy->info.idnt);
                        yfs_printf_rgt(yfms, ((2 * (i + 1)) - 1), 0, COLR_IDX_GREEN, "%.0lf     ", distance_nmile);
                        if (i == 4) yfs_printf_rgt(yfms, ((2 * (i + 1)) - 1), 3, COLR_IDX_WHITE, "%s", "NM");
                        break;
                    default:
                        // note: we don't prepend 'C' to the course to e.g. prevent
                        //       confusion with a procedure or an airway identifier
                        yfs_printf_lft(yfms, ((2 * (i + 1)) - 1), 0, COLR_IDX_WHITE, " %03.0lf", leg->imb);
                        yfs_printf_rgt(yfms, ((2 * (i + 1)) - 1), 0, COLR_IDX_GREEN, "%.0lf     ", distance_nmile);
                        if (i == 4) yfs_printf_rgt(yfms, ((2 * (i + 1)) - 1), 3, COLR_IDX_WHITE, "%s", "NM");
                        break;
                }
            }
            if (i == 0) // column headers, overwrite other info on the right
            {
                yfs_printf_rgt(yfms, ((2 * (i + 1)) - 1), 0, COLR_IDX_WHITE, "%s", "TIME  SPD/ALT   ");
            }
        }
    }

    /* all good */
    return;
}

void yfs_fpln_fplnsync(yfms_context *yfms)
{
    if (yfms->data.init.ialized)
    {
        if (yfms->data.fpln.xplm_last != XPLMCountFMSEntries() - 1)
        {
            xplm_flpn_sync(yfms);
        }
    }
}

void yfs_fpln_fplnupdt(yfms_context *yfms)
{
    /* Skip navdlib resync as required */
    if (yfms->data.fpln.mod.operation == YFS_FPLN_MOD_NONE)
    {
        goto end;
    }

    /* Flight plan changed: full resync with navdlib */
    int tracking_destination = yfms->data.fpln.lg_idx == yfms->data.fpln.dindex;
    int index_for_ln_zero    = fpl_getindex_for_line(yfms, 0);
    if (ndt_list_count(yfms->data.fpln.legs))
    {
        ndt_list_empty(yfms->data.fpln.legs);
    }
    {
        yfms->data.fpln.d_fpl = yfms->ndt.flp.rte; // default plan if all empty
    }

    //fixme filter out leg 0 (departure to first wpt) when we have legs already,
    //      even for enroute and arrival plans (test this, test this, test this)

    /* Data */
    ndt_route_leg *leg; ndt_list *l;

    /* Go through all plans and add legs from each */
    if ((l = yfms->ndt.flp.dep->legs))
    {
        for (int i = 0, j = ndt_list_count(l); i < j; i++)
        {
            if ((leg = ndt_list_item(l, i)))
            {
                switch (leg->type)
                {
                    default:
                        yfms->data.fpln.d_fpl = yfms->ndt.flp.dep;
                        ndt_list_add(yfms->data.fpln.legs, leg);
                        break;
                }
            }
        }
    }
    if ((l = yfms->ndt.flp.rte->legs))
    {
        for (int i = 0, j = ndt_list_count(l); i < j; i++)
        {
            if ((leg = ndt_list_item(l, i)))
            {
                if (i == 0 && ndt_list_count(yfms->data.fpln.legs))
                {
                    // TODO: link with preceding element
                }
                switch (leg->type)
                {
                    default:
                        yfms->data.fpln.d_fpl = yfms->ndt.flp.rte;
                        ndt_list_add(yfms->data.fpln.legs, leg);
                        break;
                }
            }
        }
    }
    if ((l = yfms->ndt.flp.arr->legs))
    {
        for (int i = 0, j = ndt_list_count(l); i < j; i++)
        {
            if ((leg = ndt_list_item(l, i)))
            {
                if (i == 0 && ndt_list_count(yfms->data.fpln.legs))
                {
                    // TODO: link with preceding element
                }
                switch (leg->type)
                {
                    default:
                        yfms->data.fpln.d_fpl = yfms->ndt.flp.arr;
                        ndt_list_add(yfms->data.fpln.legs, leg);
                        break;
                }
            }
        }
    }
    if ((l = yfms->ndt.flp.iac->legs))
    {
        for (int i = 0, j = ndt_list_count(l); i < j; i++)
        {
            if ((leg = ndt_list_item(l, i)))
            {
                if (i == 0 && ndt_list_count(yfms->data.fpln.legs))
                {
                    // TODO: link with preceding element
                }
                switch (leg->type)
                {
                    default:
                        yfms->data.fpln.d_fpl = yfms->ndt.flp.iac;
                        ndt_list_add(yfms->data.fpln.legs, leg);
                        break;
                }
            }
        }
    }

    /* Last leg (arrival runway or airport) */
    ndt_list_add(yfms->data.fpln.legs, (yfms->data.fpln.d_leg = yfms->data.fpln.d_fpl->arr.last.rleg));
    yfms->data.fpln.dindex = ndt_list_count(yfms->data.fpln.legs) - 1;

    /* Future: missed approach legs */

    /* Update index to tracked leg as required */
    if (yfms->data.fpln.lg_idx > 0)
    {
        if (yfms->data.fpln.mod.operation == YFS_FPLN_MOD_REMV)
        {
            if (yfms->data.fpln.mod.index < yfms->data.fpln.lg_idx)
            {
                yfms->data.fpln.lg_idx--;
            }
            goto end;
        }
        if (yfms->data.fpln.mod.operation == YFS_FPLN_MOD_NSRT)
        {
            if (yfms->data.fpln.mod.index < yfms->data.fpln.lg_idx)
            {
                yfms->data.fpln.lg_idx++;
            }
            goto end;
        }
        if (tracking_destination)
        {
            yfms->data.fpln.lg_idx = yfms->data.fpln.dindex; goto end;
        }
        for (int i = 0, j = ndt_list_count(yfms->data.fpln.legs); i < j; i++)
        {
            {
                yfms->data.fpln.lg_idx = 0; // reset, then find last tracked leg
            }
            if ((leg = ndt_list_item(l, i)))
            {
                if (leg        == yfms->data.fpln.mod.source ||
                    leg->xpfms == yfms->data.fpln.mod.opaque)
                {
                    yfms->data.fpln.lg_idx = i; break; // exact same leg
                }
                if (yfms->data.fpln.mod.operation == YFS_FPLN_MOD_SIDP)
                {
                    continue; // changed SID, exact leg else keep tracking first
                }
                if (leg->dst && leg->dst == yfms->data.fpln.mod.opaque)
                {
                    yfms->data.fpln.lg_idx = i; continue; // track same waypoint
                }
                if (yfms->data.fpln.lg_idx == 0 && leg->rsg && leg->rsg->type == NDT_RSTYPE_PRC)
                {
                    if ((yfms->data.fpln.mod.operation == YFS_FPLN_MOD_STAR) &&
                        (leg->rsg == yfms->ndt.flp.arr->arr.star.enroute.rsgt ||
                         leg->rsg == yfms->ndt.flp.arr->arr.star.rsgt))
                    {
                        // changed our STAR and reached first segment of new STAR
                        yfms->data.fpln.lg_idx = i - 1; continue; // last enroute leg
                    }
                    if ((yfms->data.fpln.mod.operation == YFS_FPLN_MOD_APPR) &&
                        (leg->rsg == yfms->ndt.flp.arr->arr.apch.transition.rsgt ||
                         leg->rsg == yfms->ndt.flp.arr->arr.apch.rsgt))
                    {
                        // changed our APPR and reached first segment of new APPR
                        yfms->data.fpln.lg_idx = i - 1; continue; // last STAR/RT leg
                    }
                }
            }
        }
        goto end;
    }

end:/* We should be fully synced with navdlib now */
    /* Adjust line offset after line count change */
    switch (yfms->data.fpln.mod.operation)
    {
        case YFS_FPLN_MOD_NONE:
            break;
        case YFS_FPLN_MOD_REMV:
        case YFS_FPLN_MOD_NSRT:
            if ((index_for_ln_zero -= fpl_getindex_for_line(yfms, 0)))
            {
                yfms->data.fpln.ln_off += index_for_ln_zero;
            }
            break;
        default: // major modifications, reset
            yfms->data.fpln.ln_off = 0;
            break;
    }
    yfms->data.fpln.mod.index     = 0;
    yfms->data.fpln.mod.opaque    = NULL;
    yfms->data.fpln.mod.source    = NULL;
    yfms->data.fpln.mod.operation = YFS_FPLN_MOD_NONE;
    xplm_flpn_sync(yfms); yfs_fpln_pageupdt(yfms); return;
}

static void yfs_lsk_callback_fpln(yfms_context *yfms, int key[2], intptr_t refcon)
{
    if (key[0] == 0) // insert a waypoint, or open the lateral rev. page
    {
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf); ndt_route_leg *leg;
        int index = key[1] == 5 ? yfms->data.fpln.dindex : fpl_getindex_for_line(yfms, key[1]);
        if (index < 0) // next waypoint is origin or invalid
        {
            yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
        }
        if ((leg = ndt_list_item(yfms->data.fpln.legs, index)) == NULL)
        {
            yfs_spad_reset(yfms, "UNKNOWN ERROR 1 A", COLR_IDX_ORANGE); return;
        }
        if (key[1] == 5 || strnlen(buf, 1) == 0) // open lateral revision page
        {
            return; // TODO
        }
        if (strcmp(buf, "CLR") == 0)
        {
            if (index == ndt_list_count(yfms->data.fpln.legs) ||
                index == yfms->data.fpln.dindex)
            {
                // can't clear destination or last leg
                yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
            }
            if (leg->rsg == NULL)
            {
                yfs_spad_reset(yfms, "UNKNOWN ERROR 2 A", COLR_IDX_ORANGE); return;
            }
            if (leg->rsg->type == NDT_RSTYPE_MAP)
            {
                yfs_spad_reset(yfms, "NOT IMPLEMENTED", -1); return;
            }
            if (leg->rsg->type == NDT_RSTYPE_PRC)
            {
                ndt_flightplan *flp = NULL; // remove from correct sub-plan
                if (leg->rsg == yfms->ndt.flp.dep->dep.sid.enroute.rsgt     ||
                    leg->rsg == yfms->ndt.flp.dep->dep.sid.rsgt)
                {
                    flp = yfms->ndt.flp.dep;
                }
                if (leg->rsg == yfms->ndt.flp.arr->arr.star.enroute.rsgt    ||
                    leg->rsg == yfms->ndt.flp.arr->arr.star.rsgt)
                {
                    flp = yfms->ndt.flp.arr;
                }
                if (leg->rsg == yfms->ndt.flp.iac->arr.apch.transition.rsgt ||
                    leg->rsg == yfms->ndt.flp.iac->arr.apch.rsgt)
                {
                    flp = yfms->ndt.flp.iac;
                }
                if (flp == NULL)
                {
                    yfs_spad_reset(yfms, "UNKNOWN ERROR 2 B", COLR_IDX_ORANGE); return;
                }
                if (ndt_flightplan_remove_leg(flp, leg))
                {
                    yfs_spad_reset(yfms, "UNKNOWN ERROR 2 C", COLR_IDX_ORANGE); return;
                }
            }
            else
            {
                if (ndt_flightplan_remove_leg(yfms->ndt.flp.rte, leg))
                {
                    yfs_spad_reset(yfms, "UNKNOWN ERROR 2 D", COLR_IDX_ORANGE); return;
                }
            }
            yfms->data.fpln.mod.source    = leg;
            yfms->data.fpln.mod.opaque    = NULL;
            yfms->data.fpln.mod.index     = index;
            yfms->data.fpln.mod.operation = YFS_FPLN_MOD_REMV;
            yfs_spad_clear(yfms); yfs_fpln_fplnupdt(yfms); return;
        }
        char *suffix = buf, *prefix = strsep(&suffix, "/-"); ndt_waypoint *wpt;
        if   (prefix == NULL || strnlen(prefix, 1) == 0)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        // TODO: place/bearing/distance, place-bearing/place-bearing, along route, etc.
        // TODO: disambiguation page
        if ((wpt = ndt_navdata_get_wptnear2(yfms->ndt.ndb, prefix, NULL, yfms->data.aircraft_pos)) == NULL)
        {
            yfs_spad_reset(yfms, "NOT IN DATA BASE", -1); return;
        }
        if (wpt == leg->src || wpt == leg->dst) // duplicates easy to check here
        {
            yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
        }
        // TODO: latitude/longitude (after fixes, because 1234N etc.)
        if (index == yfms->data.fpln.dindex)
        {
            if (ndt_flightplan_insert_direct(yfms->data.fpln.d_fpl, wpt, leg, 0))
            {
                yfs_spad_reset(yfms, "UNKNOWN ERROR 3 A", COLR_IDX_ORANGE); return;
            }
            yfms->data.fpln.mod.source    = leg;
            yfms->data.fpln.mod.index     = index;
            yfms->data.fpln.mod.operation = YFS_FPLN_MOD_NSRT;
            yfms->data.fpln.mod.opaque    = leg->dst ? (void*)leg->dst : (void*)leg->xpfms;
            yfs_spad_clear(yfms); yfs_fpln_fplnupdt(yfms); return;
        }
        if (leg->rsg == yfms->ndt.flp.dep->dep.sid.enroute.rsgt ||
            leg->rsg == yfms->ndt.flp.dep->dep.sid.rsgt)
        {
            if (ndt_flightplan_insert_direct(yfms->ndt.flp.dep, wpt, leg, 0))
            {
                yfs_spad_reset(yfms, "UNKNOWN ERROR 3 B", COLR_IDX_ORANGE); return;
            }
            yfms->data.fpln.mod.source    = leg;
            yfms->data.fpln.mod.index     = index;
            yfms->data.fpln.mod.operation = YFS_FPLN_MOD_NSRT;
            yfms->data.fpln.mod.opaque    = leg->dst ? (void*)leg->dst : (void*)leg->xpfms;
            yfs_spad_clear(yfms); yfs_fpln_fplnupdt(yfms); return;
        }
        if (leg->rsg == yfms->ndt.flp.arr->arr.star.enroute.rsgt ||
            leg->rsg == yfms->ndt.flp.arr->arr.star.rsgt)
        {
            if (ndt_flightplan_insert_direct(yfms->ndt.flp.arr, wpt, leg, 0))
            {
                yfs_spad_reset(yfms, "UNKNOWN ERROR 3 C", COLR_IDX_ORANGE); return;
            }
            yfms->data.fpln.mod.source    = leg;
            yfms->data.fpln.mod.index     = index;
            yfms->data.fpln.mod.operation = YFS_FPLN_MOD_NSRT;
            yfms->data.fpln.mod.opaque    = leg->dst ? (void*)leg->dst : (void*)leg->xpfms;
            yfs_spad_clear(yfms); yfs_fpln_fplnupdt(yfms); return;
        }
        if (leg->rsg == yfms->ndt.flp.iac->arr.apch.transition.rsgt ||
            leg->rsg == yfms->ndt.flp.iac->arr.apch.rsgt)
        {
            if (ndt_flightplan_insert_direct(yfms->ndt.flp.iac, wpt, leg, 0))
            {
                yfs_spad_reset(yfms, "UNKNOWN ERROR 3 D", COLR_IDX_ORANGE); return;
            }
            yfms->data.fpln.mod.source    = leg;
            yfms->data.fpln.mod.index     = index;
            yfms->data.fpln.mod.operation = YFS_FPLN_MOD_NSRT;
            yfms->data.fpln.mod.opaque    = leg->dst ? (void*)leg->dst : (void*)leg->xpfms;
            yfs_spad_clear(yfms); yfs_fpln_fplnupdt(yfms); return;
        }
        if (ndt_flightplan_insert_direct(yfms->ndt.flp.rte, wpt, leg, 0))
        {
            yfs_spad_reset(yfms, "UNKNOWN ERROR 3 E", COLR_IDX_ORANGE); return;
        }
        yfms->data.fpln.mod.source    = leg;
        yfms->data.fpln.mod.index     = index;
        yfms->data.fpln.mod.operation = YFS_FPLN_MOD_NSRT;
        yfms->data.fpln.mod.opaque    = leg->dst ? (void*)leg->dst : (void*)leg->xpfms;
        yfs_spad_clear(yfms); yfs_fpln_fplnupdt(yfms); return;
    }
    if (key[0] == 1 && key[1] != 5) // constraints or vertical rev. page
    {
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        int index = fpl_getindex_for_line(yfms, key[1]); ndt_route_leg *leg;
        if (index == yfms->data.fpln.dindex || index < 0) // leg is dest. or invalid
        {
            yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
        }
        if ((leg = ndt_list_item(yfms->data.fpln.legs, index)) == NULL)
        {
            yfs_spad_reset(yfms, "UNKNOWN ERROR 1 B", COLR_IDX_ORANGE); return;
        }
        if (yfms->data.init.ialized == 0)
        {
            yfs_spad_reset(yfms, "UNKNOWN ERROR 4 A", COLR_IDX_ORANGE); return;
        }
        if (strnlen(buf, 1) == 0) // open vertical revision page
        {
            return; // TODO
        }
        ndt_restriction constraints = leg->constraints; int airspeed;
        char *sufx = buf, *prefix = strsep(&sufx, "/");
        if (prefix && strnlen(prefix, 1))
        {
            if (sscanf(prefix, "%d", &airspeed) != 1)
            {
                yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
            }
            if (airspeed)
            {
                if (airspeed < 150 || airspeed > 300)
                {
                    yfs_spad_reset(yfms, "ENTRY OUT OF RANGE", -1); return;
                }
                constraints.airspeed.max = ndt_airspeed_init(airspeed, NDT_SPDUNIT_KTS);
                constraints.airspeed.acf = NDT_ACFTYPE_ALL;
                constraints.airspeed.typ = NDT_RESTRICT_BL;
            }
            else
            {
                constraints.airspeed.acf = NDT_ACFTYPE_NON;
                constraints.airspeed.typ = NDT_RESTRICT_NO;
            }
        }
        if (sufx && strnlen(sufx, 1))
        {
            if (ndt_distance_get(yfms->data.init.crz_alt, NDT_ALTUNIT_NA) == 0)
            {
                yfs_spad_reset(yfms, "CRZ ALT NOT SET", -1); return;
            }
            ndt_distance alti; ndt_distance max = yfms->data.init.crz_alt; int altitude; char tmp[2];
            ndt_distance min = ndt_distance_add(ndt_position_getaltitude(yfms->ndt.flp.rte->arr.apt->coordinates), ndt_distance_init(1000, NDT_ALTUNIT_FT));
            if (sscanf(sufx, "%1[+]FL%3d", tmp, &altitude) == 2 ||
                sscanf(sufx, "FL%3d%1[+]", &altitude, tmp) == 2)
            {
                constraints.altitude.typ = NDT_RESTRICT_AB;
                alti = ndt_distance_init(altitude, NDT_ALTUNIT_FL);
                goto check_altitude;
            }
            if (sscanf(sufx, "%1[-]FL%3d", tmp, &altitude) == 2 ||
                sscanf(sufx, "FL%3d%1[-]", &altitude, tmp) == 2)
            {
                constraints.altitude.typ = NDT_RESTRICT_BL;
                alti = ndt_distance_init(altitude, NDT_ALTUNIT_FL);
                goto check_altitude;
            }
            if (sscanf(sufx, "FL%3d", &altitude) == 1 ||
                sscanf(sufx, "FL%3d", &altitude) == 1)
            {
                constraints.altitude.typ = NDT_RESTRICT_AT;
                alti = ndt_distance_init(altitude, NDT_ALTUNIT_FL);
                goto check_altitude;
            }
            if (sscanf(sufx, "%1[+]%5d", tmp, &altitude) == 2 ||
                sscanf(sufx, "%5d%1[+]", &altitude, tmp) == 2)
            {
                constraints.altitude.typ = NDT_RESTRICT_AB;
                alti = ndt_distance_init(altitude, NDT_ALTUNIT_FT);
                goto check_altitude;
            }
            if (sscanf(sufx, "%1[-]%5d", tmp, &altitude) == 2 ||
                sscanf(sufx, "%5d%1[-]", &altitude, tmp) == 2)
            {
                constraints.altitude.typ = NDT_RESTRICT_BL;
                alti = ndt_distance_init(altitude, NDT_ALTUNIT_FT);
                goto check_altitude;
            }
            if (sscanf(sufx, "%5d", &altitude) == 1 ||
                sscanf(sufx, "%5d", &altitude) == 1)
            {
                constraints.altitude.typ = NDT_RESTRICT_AT;
                alti = ndt_distance_init(altitude, NDT_ALTUNIT_FT);
                goto check_altitude;
            }
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        check_altitude:
            if (ndt_distance_get(alti, NDT_ALTUNIT_NA))
            {
                if (ndt_distance_get(alti, NDT_ALTUNIT_NA) < ndt_distance_get(min, NDT_ALTUNIT_NA) ||
                    ndt_distance_get(alti, NDT_ALTUNIT_NA) > ndt_distance_get(max, NDT_ALTUNIT_NA))
                {
                    yfs_spad_reset(yfms, "ENTRY OUT OF RANGE", -1); return;
                }
                constraints.altitude.min = alti;
                constraints.altitude.max = alti;
            }
            else
            {
                constraints.altitude.typ = NDT_RESTRICT_NO;
            }
        }
        if (ndt_route_leg_restrict(leg, constraints))
        {
            yfs_spad_reset(yfms, "UNKNOWN ERROR 4 B", COLR_IDX_ORANGE); return;
        }
        yfs_spad_clear(yfms); yfs_fpln_fplnupdt(yfms); return;
    }
    /* all good */
    return;
}

static void fpl_spc_callback_lnup(yfms_context *yfms)
{
    yfms->data.fpln.ln_off++; yfs_fpln_pageupdt(yfms); return;
}

static void fpl_spc_callback_lndn(yfms_context *yfms)
{
    yfms->data.fpln.ln_off--; yfs_fpln_pageupdt(yfms); return;
}

static void fpl_print_leg_generic(yfms_context *yfms, int row, ndt_route_leg *leg)
{
    const char *restr_type;
    ndt_restriction restrs = leg->constraints;
    int alt_const_ft, alt_const_fl, spd_const;
    int tr_alt = (int)ndt_distance_get(yfms->data.init.trans_a, NDT_ALTUNIT_FT);
    yfs_printf_lft(yfms, row, 0, COLR_IDX_GREEN, "%-7s", leg->dst ? leg->dst->info.idnt : leg->info.idnt);
    yfs_printf_rgt(yfms, row, 0, COLR_IDX_GREEN, "%s", "----  ---/ -----");

    /*
     * Speed and altitude restrictions are displayed in magenta here.
     * Does not match the real-world format, but I like it better :P
     */
    if (restrs.airspeed.acf == NDT_ACFTYPE_ALL ||
        restrs.airspeed.acf == NDT_ACFTYPE_JET)
    {
        switch (restrs.airspeed.typ)
        {   // note: we only support at or below constraints
            case NDT_RESTRICT_BL: // at or below
            case NDT_RESTRICT_AT: // at airspeed
            case NDT_RESTRICT_BT: // min and max
                yfs_printf_rgt(yfms, row, 7, COLR_IDX_MAGENTA, "%.3d", (int)ndt_airspeed_get(restrs.airspeed.max, NDT_SPDUNIT_KTS, NDT_MACH_DEFAULT));
                break;
            default:
                break;
        }
    }
    if (restrs.altitude.typ != NDT_RESTRICT_NO)
    {
        switch (restrs.altitude.typ)
        {
            case NDT_RESTRICT_BL:
                alt_const_ft = ndt_distance_get(restrs.altitude.max, NDT_ALTUNIT_FT);
                alt_const_fl = ndt_distance_get(restrs.altitude.max, NDT_ALTUNIT_FL);
                restr_type = "-";
                break;
            case NDT_RESTRICT_BT: // not fully supported (ignores below max part)
            case NDT_RESTRICT_AB:
                alt_const_ft = ndt_distance_get(restrs.altitude.min, NDT_ALTUNIT_FT);
                alt_const_fl = ndt_distance_get(restrs.altitude.min, NDT_ALTUNIT_FL);
                restr_type = "+";
                break;
            case NDT_RESTRICT_AT:
            default:
                alt_const_ft = ndt_distance_get(restrs.altitude.min, NDT_ALTUNIT_FT);
                alt_const_fl = ndt_distance_get(restrs.altitude.min, NDT_ALTUNIT_FL);
                restr_type = "";
                break;
        }
        if (tr_alt <= alt_const_ft)
        {
            yfs_printf_rgt(yfms, row, 0, COLR_IDX_MAGENTA, "FL%03d%s", alt_const_fl, restr_type);
        }
        else
        {
            yfs_printf_rgt(yfms, row, 0, COLR_IDX_MAGENTA,    "%5d%s", alt_const_ft, restr_type);
        }
    }
}

static void fpl_print_airport_rwy(yfms_context *yfms, int row, ndt_airport *apt, ndt_runway *rwy)
{
    char *waypoint_id = rwy ? rwy->waypoint->info.idnt : apt->info.idnt;
    ndt_position posn = rwy ? rwy->threshold           : apt->coordinates;
    int altitude_feet = ndt_distance_get(ndt_position_getaltitude(posn), NDT_ALTUNIT_FT);
    yfs_printf_lft(yfms, row, 0, COLR_IDX_GREEN, "%-7s",            waypoint_id);
    yfs_printf_rgt(yfms, row, 0, COLR_IDX_GREEN, "----  ---/%6d", altitude_feet);
}

static int fpl_getindex_for_line(yfms_context *yfms, int line)
{
    int legct = ndt_list_count(yfms->data.fpln.legs);
    int index = yfms->data.fpln.lg_idx + yfms->data.fpln.ln_off + line - 1;
    if (index < -1 || index >= legct)
    {
        while (index > legct - 1)
        {
            index -= legct + 2; // legct == -2, legct + 1 == -1, etc.
        }
        while (index < -2)
        {
            index += legct + 2; // -3 == legct-1, -4 == legct-2, etc.
        }
    }
    return index;
}
