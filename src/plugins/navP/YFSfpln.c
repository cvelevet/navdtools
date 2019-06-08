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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/common.h"
#include "compat/compat.h"
#include "lib/airport.h"
#include "lib/flightplan.h"
#include "lib/navdata.h"
#include "lib/waypoint.h"

#include "YFSdrto.h"
#include "YFSfpln.h"
#include "YFSmain.h"
#include "YFSmenu.h"
#include "YFSspad.h"

static void            yfs_lsk_callback_fpln(yfms_context *yfms, int key[2],                intptr_t refcon);
static void            yfs_msw_callback_fpln(yfms_context *yfms, int rx, int ry,                  int delta);
static int             yfs_msc_callback_fpln(yfms_context *yfms, int rx, int ry, int b,   XPWidgetMessage m);
static void            fpl_spc_callback_lnup(yfms_context *yfms                                            );
static void            fpl_spc_callback_lndn(yfms_context *yfms                                            );
static void            fpl_print_leg_generic(yfms_context *yfms, int row,                ndt_route_leg *leg);
static void            fpl_print_airport_rwy(yfms_context *yfms, int row, ndt_airport *apt, ndt_runway *rwy);
static int             fpl_getindex_for_line(yfms_context *yfms,                                   int line);
static ndt_flightplan* fpl_getfplan_for_leg (yfms_context *yfms,                         ndt_route_leg *leg);

static XPLMNavRef xplm_find_navaid(ndt_waypoint *wpt)
{
    if (wpt)
    {
        char outID[33]; float outLati[1], outLong[1]; XPLMNavType navTypes;
        char *inIDFrag; float inLatit[1], inLongi[1]; XPLMNavRef  navRefpt;
        ndt_distance distce_zero = NDT_DISTANCE_ZERO; size_t  inIDFrag_len;
        /*
         * Always match target waypoint type pretty exactly, to avoid e.g.
         * cases where we match an NDB adjacent to a identically-named NDB.
         */
        switch (wpt->type)
        {
            case NDT_WPTYPE_APT:
            case NDT_WPTYPE_XPA:
                navTypes = xplm_Nav_Airport;
                break;
            case NDT_WPTYPE_NDB:
                navTypes = xplm_Nav_NDB;
                break;
            case NDT_WPTYPE_VOR:
                navTypes = xplm_Nav_VOR;
                break;
            case NDT_WPTYPE_ILS:
            case NDT_WPTYPE_LOC:
                navTypes = xplm_Nav_ILS|xplm_Nav_Localizer;
                break;
            case NDT_WPTYPE_FIX:
                navTypes = xplm_Nav_Fix;
                break;
            case NDT_WPTYPE_DME:
                navTypes = xplm_Nav_DME;
                break;
            default:
                return XPLM_NAV_NOT_FOUND;
        }
        inIDFrag = (char*)wpt->info.idnt; inIDFrag_len = strlen(inIDFrag);
        *inLatit = (float)ndt_position_getlatitude (wpt->position, NDT_ANGUNIT_DEG);
        *inLongi = (float)ndt_position_getlongitude(wpt->position, NDT_ANGUNIT_DEG);
        navRefpt = XPLMFindNavAid(NULL, inIDFrag, inLatit, inLongi, NULL, navTypes);
        if (XPLM_NAV_NOT_FOUND == navRefpt && navTypes == xplm_Nav_Airport && inIDFrag_len == 4)
        {
            // note: we'll never be able to replicate the XP10 GNS430 behavior
            // XPLMGetFMSEntryInfo n the destination airport returns a "bogus"
            // navaid w/type xplm_Nav_Airport but outRef == XPLM_NAV_NOT_FOUND
            // instead we try converting the ICAO to the corresponding FAA LID
            inIDFrag = (char*)(wpt->info.idnt + 1); inIDFrag_len = strlen(inIDFrag);
            navRefpt = XPLMFindNavAid(NULL, inIDFrag, inLatit, inLongi, NULL, navTypes);
        }
        if (XPLM_NAV_NOT_FOUND != navRefpt)
        {
            XPLMGetNavAidInfo(navRefpt, NULL, outLati, outLong, NULL, NULL, NULL, outID, NULL, NULL);
            ndt_position positn = ndt_position_init((double)*outLati, (double)*outLong, distce_zero);
            ndt_distance dstnce = ndt_position_calcdistance(wpt->position, positn);
            /*
             * Example (Navigraph data, AIRAC 1707):
             * > $ grep TRS Waypoints.txt Navaids.txt | grep \,ES
             * > Waypoints.txt:TRS29,59.286033,18.150856,ES
             * > Navaids.txt:TRS,TROSA,114.300,1,1,195,58.937917,17.502222,213,ES,0
             *
             * Even worse (Aerosoft data, AIRAC 1707):
             * > $ grep TRS Waypoints.txt Navaids.txt | grep \,ES
             * > Waypoints.txt:27TRS,59.20740,18.17919,ES
             * > Waypoints.txt:65TRS,59.00434,17.66729,ES
             * > Waypoints.txt:TRS10,58.82436,17.26767,ES
             * > Waypoints.txt:TRS22,59.20709,17.98320,ES
             * > Waypoints.txt:TRS27,59.38652,17.47173,ES
             * > Waypoints.txt:TRS65,59.04594,17.49621,ES
             * > Navaids.txt:TRS,TROSA,114.300,1,1,195,58.93792,17.50222,213,ES,0
             *
             * Because of how XPLMFindNavAid works (by design), it could
             * e.g. find and return any of the above when inIDFrag is TRS.
             */
            if (strnlen(outID, 1 + inIDFrag_len) != inIDFrag_len)
            {
                return XPLM_NAV_NOT_FOUND;
            }
            if (navTypes == xplm_Nav_Airport)
            {
                // airports can be within a few miles from our target
                if (ndt_distance_get(dstnce, NDT_ALTUNIT_NM) < INT64_C(3))
                {
                    return navRefpt;
                }
            }
            else
            {
                // other navaids must be within a furlong from target
                if (ndt_distance_get(dstnce, NDT_ALTUNIT_FT) < INT64_C(660))
                {
                    return navRefpt;
                }
            }
        }
    }
    return XPLM_NAV_NOT_FOUND;
}

static void xplm_fpln_sync(yfms_context *yfms)
{
    /*
     * There may be more than 100 waypoints in our plan, we need to determine
     * the first and last legs so our waypoint count is limited to 99 or less.
     */
    int total_wpnts_count = 0, first_leg_idx = -1, last_leg_idx = 0;
    ndt_route_leg *leg; ndt_waypoint *wpt; yfms->data.fpln.xplm_last = -1;
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
            total_wpnts_count += ndt_list_count(leg->xpfms) + !!leg->dst;
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
            total_wpnts_count += ndt_list_count(leg->xpfms) + !!leg->dst;
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
                yfms->data.fpln.xplm_last++; // i == -1; xplm_last: -1 -> +0
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud = (float)ndt_position_getlongitude(wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude = (float)ndt_position_getlatitude (wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].cst = ndt_leg_const_init();
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].legindex = i;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].altitude = 0;
                if (wpt->xplm.refSet)
                {
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = wpt->xplm.navRef;
                }
                else
                {
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = xplm_find_navaid(wpt);
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
                yfms->data.fpln.xplm_last++; // i == -1; xplm_last: +0 -> +1
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].longitud = (float)ndt_position_getlongitude(wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].latitude = (float)ndt_position_getlatitude (wpt->position, NDT_ANGUNIT_DEG);
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].cst = ndt_leg_const_init();
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].legindex = i;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].altitude = 0;
                if (wpt->xplm.refSet)
                {
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = wpt->xplm.navRef;
                }
                else // wpt == yfms->ndt.flp.rte->dep.rwy->waypoint
                {
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = XPLM_NAV_NOT_FOUND;
                    wpt->xplm.navRef = yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint;
                    wpt->xplm.refSet = 1;
                }
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
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = XPLM_NAV_NOT_FOUND; // xpfms waypoints are always lat/lon based
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
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].cst = leg->constraints;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].legindex = i;
                yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].altitude = 0;
                if (wpt->xplm.refSet)
                {
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = wpt->xplm.navRef;
                }
                else
                {
                    yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint = xplm_find_navaid(wpt);
                    wpt->xplm.navRef = yfms->data.fpln.xplm_info[yfms->data.fpln.xplm_last].waypoint;
                    wpt->xplm.refSet = 1;
                }
            }
        }
    }

    /* TODO: VNAV */

    /* Clear all FMS entries - replace them w/our own */
    int  t_leg = XPLMGetDestinationFMSEntry();
    for (int i = XPLMCountFMSEntries() - 1; i >= 0; i--)
    {
        XPLMClearFMSEntry(i);
    }
    for (int i = 0; i <= yfms->data.fpln.xplm_last; i++)
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
    }
    yfs_fpln_trackleg(yfms, yfms->data.fpln.lg_idx);

    /* ensure we recompute distance remaining to destination */
    yfms->data.fpln.dist.ref_leg_id = -1;

    /* should be fully synced now */
    return;
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
    if (yfms->xpl.has_custom_navigation == 0)
    {
        yfms->data.fpln.ln_off = 0;
        yfms->lsks[0][0].cback = yfms->lsks[1][0].cback =
        yfms->lsks[0][1].cback = yfms->lsks[1][1].cback =
        yfms->lsks[0][2].cback = yfms->lsks[1][2].cback =
        yfms->lsks[0][3].cback = yfms->lsks[1][3].cback =
        yfms->lsks[0][4].cback = yfms->lsks[1][4].cback =
        yfms->lsks[0][5].cback = yfms->lsks[1][5].cback = (YFS_LSK_f)&yfs_lsk_callback_fpln;
        yfms->spcs. cback_lnup = (YFS_SPC_f)&fpl_spc_callback_lnup; yfms->spcs. cback_lndn = (YFS_SPC_f)&fpl_spc_callback_lndn;
        yfms->mousew_callback  = (YFS_MSW_f)&yfs_msw_callback_fpln; yfms->mousec_callback  = (YFS_MSC_f)&yfs_msc_callback_fpln;
    }
    return yfs_fpln_pageupdt(yfms);
}

static void awys_pageupdt(yfms_context *yfms)
{
    /*
     * do we have valid modifications to the flight plan?
     * colors as per youtube.com/watch?v=_skUOMfv-AQ&t=53
     */
    int have_valid_leg = yfms->data.fpln.awys.awy[0] && yfms->data.fpln.awys.dst[0];
    int text_colr_idx1 = have_valid_leg ? COLR_IDX_YELLOW : COLR_IDX_WHITE;
    int text_colr_idx2 = have_valid_leg ? COLR_IDX_YELLOW : COLR_IDX_BLUE;

    /* page title */
    yfs_printf_lft(yfms, 0,  0, text_colr_idx1, "%s",                   "   AIRWAYS FROM");
    yfs_printf_lft(yfms, 0, 16, COLR_IDX_GREEN, "%s", yfms->data.fpln.lrev.wpt->info.idnt);

    /* print up to 5 preselected airway legs */
    for (int i = 0, ii = 5; i < ii; i++)
    {
        int prev_airway_set = i == 0 || yfms->data.fpln.awys.awy[i - 1] != NULL;
        int curr_airway_set = yfms->data.fpln.awys.awy[i] != NULL;
        int curr_waypnt_set = yfms->data.fpln.awys.dst[i] != NULL;
        if (prev_airway_set)
        {
            if (curr_airway_set)
            {
                yfs_printf_lft(yfms, (2 * (i + 1)), 0, text_colr_idx2, "%s", yfms->data.fpln.awys.awy[i]->info.idnt);
            }
            else
            {
                yfs_printf_lft(yfms, (2 * (i + 1)), 0, COLR_IDX_BLUE,  "%s", "[   ]");
            }
            yfs_printf_lft    (yfms, ((2 * i) + 1), 0, COLR_IDX_WHITE, "%s",  " VIA");
        }
        if (curr_airway_set)
        {
            if (curr_waypnt_set)
            {
                yfs_printf_rgt(yfms, (2 * (i + 1)), 0, text_colr_idx2, "%s", yfms->data.fpln.awys.dst[i]->info.idnt);
            }
            else
            {
                yfs_printf_rgt(yfms, (2 * (i + 1)), 0, COLR_IDX_BLUE,  "%s", "[    ]");
            }
            yfs_printf_rgt    (yfms, ((2 * i) + 1), 0, COLR_IDX_WHITE, "%s",    "TO ");
        }
    }

    /* insert and erase/return */
    if (have_valid_leg)
    {
        yfs_printf_rgt(yfms, 12, 0, COLR_IDX_ORANGE, "%s", "INSERT*");
        yfs_printf_lft(yfms, 12, 0, text_colr_idx1,  "%s",  "<F-PLN");
    }
    else
    {
        yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE,  "%s", "<RETURN");
    }

    /* all good */
    yfms->mwindow.screen.redraw = 1; return;
}

static void lrev_pageupdt(yfms_context *yfms)
{
    /* page title + latitude and longitude coordinates */
    if (yfms->data.fpln.lrev.wpt)
    {
        yfs_printf_lft(yfms, 0, 16, COLR_IDX_GREEN, "%s", yfms->data.fpln.lrev.wpt->info.idnt);
        ndt_position posn = yfms->data.fpln.lrev.wpt->position; char buf[21];
        ndt_position_sprintllc(posn, NDT_LLCFMT_AIBX2, buf, sizeof(buf));
        yfs_printf_lft(yfms, 1,  0, COLR_IDX_GREEN, "   %s", buf);
    }
    else
    {
        yfs_printf_lft(yfms, 0, 16, COLR_IDX_GREEN, "%s", yfms->data.fpln.lrev.leg->info.idnt);
    }
    yfs_printf_lft(yfms, 0, 0, COLR_IDX_WHITE, "%s", "   LAT REV FROM");

    /* departure/arrival subpage links */
    if (yfms->data.fpln.lrev.idx == -1)
    {
        yfs_printf_lft(yfms, 2, 0, COLR_IDX_WHITE, "%s", "<DEPARTURE");
    }
    if (yfms->data.fpln.lrev.idx == yfms->data.fpln.dindex)
    {
        yfs_printf_rgt(yfms, 2, 0, COLR_IDX_WHITE, "%s", "ARRIVAL>");
    }

    /* additional waypoints, airways */
    {
//      // potential OPC options???
//      yfs_printf_rgt(yfms, 3, 0, COLR_IDX_WHITE, "%s", "LL XING/INCR/NO");
//      yfs_printf_rgt(yfms, 4, 0, COLR_IDX_BLUE,  "%s",   "[  ] /[ ] /[]");
//      yfs_printf_rgt(yfms, 3, 0, COLR_IDX_WHITE, "%s",       "VIA/GO TO");
//      yfs_printf_rgt(yfms, 4, 0, COLR_IDX_BLUE,  "%s",      "[  ]/[   ]");
        yfs_printf_rgt(yfms, 5, 0, COLR_IDX_WHITE, "%s",       "NEXT WPT ");
        yfs_printf_rgt(yfms, 6, 0, COLR_IDX_BLUE,  "%s",           "[   ]");
    }
    if (yfms->data.fpln.lrev.idx != yfms->data.fpln.dindex)
    {
        yfs_printf_rgt(yfms, 7, 0, COLR_IDX_WHITE, "%s", "NEW DEST ");
        yfs_printf_rgt(yfms, 8, 0, COLR_IDX_BLUE,  "%s",     "[   ]");
    }
    if (yfms->data.fpln.lrev.leg && yfms->data.fpln.lrev.leg->dst)
    {
        int type = yfms->data.fpln.lrev.leg->dst->type;
        if (type == NDT_WPTYPE_NDB || type == NDT_WPTYPE_VOR ||
            type == NDT_WPTYPE_FIX || type == NDT_WPTYPE_DME)
        {
            yfs_printf_rgt(yfms, 10, 0, COLR_IDX_WHITE, "%s", "AIRWAYS>");
        }
    }

    /* return (to FPLN) */
    yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "%s", "<RETURN");

    /* all good */
    yfms->mwindow.screen.redraw = 1; return;
}

void yfs_fpln_pageupdt(yfms_context *yfms)
{
    /* reset lines, set page title */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }
    if (yfms->xpl.has_custom_navigation)
    {
        yfms->mwindow.screen.redraw = 1;
        return yfs_printf_ctr(yfms, 6, COLR_IDX_WHITE, "%s", "PAGE INOP");
    }

    /* do we have a sub-page open? */
    if (yfms->data.fpln.awys.open)
    {
        return awys_pageupdt(yfms);
    }
    if (yfms->data.fpln.lrev.open)
    {
        return lrev_pageupdt(yfms);
    }

    /* ensure we're still synced w/navdlib, reset if necessary */
    yfs_fpln_fplnsync(yfms);

    /* update indexes for tracked and displayed legs */
    if (yfms->data.init.ialized)
    {
        int s = fpl_getindex_for_line(yfms, 1);
        if (yfms->data.phase > FMGS_PHASE_PRE) // flight: can update tracked leg
        {
            int t = XPLMGetDestinationFMSEntry();
            if (t < yfms->data.fpln.xplm_last + 1)
            {
                yfms->data.fpln.lg_idx = yfms->data.fpln.xplm_info[t].legindex;
            }
        }
        for (int i = 1, set = 0; i <= yfms->data.fpln.xplm_last; i++)
        {
            if (set > 0 && yfms->data.fpln.xplm_info[i].legindex > s)
            {
                if (yfms->xpl.atyp != YFS_ATYP_Q380) // A380 legs may not always be synced
                {
                    XPLMSetDisplayedFMSEntry(set);
                }
                break;
            }
            if (yfms->data.fpln.xplm_info[i].legindex >= s)
            {
                set = i;
            }
        }
    }

    /* mostly static data */
    if (yfms->data.init.flight_id[0])
    {
        yfs_printf_rgt(yfms, 0, 0, COLR_IDX_WHITE, "%s   ", yfms->data.init.flight_id);
//      yfs_printf_rgt(yfms, 0, 0, COLR_IDX_WHITE, "%s ->", yfms->data.init.flight_id);
    }
    if (yfms->data.init.ialized == 0)
    {
        if (yfms->data.fpln.ln_off)
        {
            yfms->data.fpln.ln_off = 0; yfs_fpln_pageupdt(yfms); return;
        }
        yfs_printf_lft(yfms,  2, 0, COLR_IDX_WHITE, "%s", "------END OF F-PLN------");
        yfs_printf_lft(yfms, 11, 0, COLR_IDX_WHITE, "%s", " DEST   TIME  DIST  EFOB");
        yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "%s", "        ----  ----  ----");
        yfms->mwindow.screen.redraw = 1; return;
    }

    /* two lines per leg (header/course/distance, then name/constraints) */
    ndt_route_leg *leg; ndt_distance distance, dtrack = ndt_distance_init(0, NDT_ALTUNIT_NA);
    if ((leg = ndt_list_item(yfms->data.fpln.legs, yfms->data.fpln.lg_idx)))
    {
        if (leg->dst) // tracked leg, compute dtrack
        {
            ndt_position to = leg->dst->position;
            ndt_position from = yfms->data.aircraft_pos;
            dtrack = ndt_position_calcdistance(from, to);
        }
    }
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
                if (index == yfms->data.fpln.lg_idx)
                {
                    if (leg->dst) // display remaining distance instead of leg's
                    {
                        distance = dtrack;
                    }
                    else  // distance unavailable, set to invalid value
                    {
                        distance = ndt_distance_init(-1, NDT_ALTUNIT_NA);
                    }
                }
                else
                {
                    distance = leg->dis;
                }
                if (leg->rsg == NULL)
                {
                    yfs_menu_resetall(yfms); return yfs_spad_reset(yfms, "ERROR -> RESET", COLR_IDX_ORANGE);
                }
                double  distance_nmile = (double)ndt_distance_get(distance, NDT_ALTUNIT_ME) / 1852.;
                switch (leg->rsg->type) // LSGG>BGTL(QPAC) suggests omb is used (as opposed to imb)
                {
                    case NDT_RSTYPE_PRC: // TODO
//                      switch (leg->type)
                        break;
                    case NDT_RSTYPE_AWY:
                        yfs_printf_lft(yfms, ((2 * (i + 1)) - 1), 1, COLR_IDX_WHITE, "%.6s", leg->awyleg->awy->info.idnt);
                        yfs_printf_lft(yfms, ((2 * (i + 1)) - 1), 8, COLR_IDX_WHITE,              "TRK%03.0lf", leg->omb);
                        yfs_printf_rgt(yfms, ((2 * (i + 1)) - 1), 0, COLR_IDX_GREEN,        "%.0lf     ", distance_nmile);
                        if (i == 4) yfs_printf_rgt(yfms, ((2 * (i + 1)) - 1), 3, COLR_IDX_WHITE,              "%s", "NM");
                        break;
                    default:
                        yfs_printf_lft(yfms, ((2 * (i + 1)) - 1), 1, COLR_IDX_WHITE,         "C%03.0lf", leg->omb);
                        yfs_printf_rgt(yfms, ((2 * (i + 1)) - 1), 0, COLR_IDX_GREEN, "%.0lf     ", distance_nmile);
                        if (i == 4) yfs_printf_rgt(yfms, ((2 * (i + 1)) - 1), 3, COLR_IDX_WHITE,       "%s", "NM");
                        break;
                }
                if (distance_nmile < 0.) // not valid/available: must unprint it
                {
                    yfs_printf_rgt(yfms, ((2 * (i + 1)) - 1), 0, COLR_IDX_GREEN, "   --     ");
                }
            }
            if (i == 0) // column headers, overwrite other info
            {
                if (fpl_getindex_for_line(yfms, 0) == yfms->data.fpln.lg_idx - 1)
                {
                    yfs_printf_lft(yfms, 0, 0, COLR_IDX_WHITE, "%s",                 " FROM");
                    yfs_printf_lft(yfms, 1, 0, COLR_IDX_WHITE, "%s", "        TIME  SPD/ALT");
                }
                else
                {
                    // don't overwite magnetic course and/or related information
                    yfs_printf_rgt(yfms, 1, 0, COLR_IDX_WHITE, "%s", " TIME  SPD/ALT   ");
                }
            }
        }
    }

    /*
     * update distance to destination
     *
     * future: when we can handle and detect discontinuities in the navigation
     *         API's entries, use the latitude and longitude from said entries
     *         to compute a total distance closer to what will really be flown
     *         than the theoretical distance we currently get from YFMS's data
     *         representation (which is accurate but nonetheless theoretical).
     */
    if (yfms->data.fpln.dist.ref_leg_id != yfms->data.fpln.lg_idx)
    {
        {
            distance = ndt_distance_init(0, NDT_ALTUNIT_NA);
        }
        for (int i = yfms->data.fpln.lg_idx + 1; i < ndt_list_count(yfms->data.fpln.legs); i++)
        {
            if ((leg = ndt_list_item(yfms->data.fpln.legs, i)))
            {
                distance = ndt_distance_add(distance, leg->dis);
            }
        }
        yfms->data.fpln.dist.remain = distance;
        yfms->data.fpln.dist.ref_leg_id = yfms->data.fpln.lg_idx;
        distance = ndt_distance_add(yfms->data.fpln.dist.remain, dtrack);
    }
    else
    {
        distance = ndt_distance_add(yfms->data.fpln.dist.remain, dtrack);
    }

    /* final destination :D */
    yfs_printf_lft(yfms, 11, 0, COLR_IDX_WHITE, "%s", " DEST   TIME  DIST  EFOB");
    yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "%-4s%-3s ----  %4.0lf  ----", // TODO: TIME
                   yfms->ndt.flp.rte->arr.apt->info.idnt,
                   yfms->ndt.flp.rte->arr.rwy ?
                   yfms->ndt.flp.rte->arr.rwy->info.idnt : "",
                   (double)ndt_distance_get(distance, NDT_ALTUNIT_ME) / 1852.);

    /* all good */
    yfms->mwindow.screen.redraw = 1; return;
}

void yfs_fpln_fplnsync(yfms_context *yfms)
{
    if (yfms->data.init.ialized)
    {
        // our XPLMNavigation sync will break Peter's direct-tos (in the A380-800)
        if (yfms->xpl.atyp != YFS_ATYP_Q380 || yfms->data.phase <= FMGS_PHASE_PRE)
        {
            if (yfms->data.fpln.xplm_last != XPLMCountFMSEntries() - 1)
            {
                ndt_log("YFMS [debug]: xplm_fpln_sync: last %d count %d\n", yfms->data.fpln.xplm_last, XPLMCountFMSEntries());
                xplm_fpln_sync(yfms);
            }
        }
        if (yfms->data.phase >= FMGS_PHASE_TOF)
        {
            if (XPLMGetDatai(yfms->xpl.fdir_mode) >= 1)
            {
                if (XPLMGetDatai(yfms->xpl.nav_mode_status) == 2) // NAV source captured
                {
                    XPLMCommandOnce(yfms->xpl.heading_sync); // heading: continuous sync
                }
            }
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
    yfms->data.fpln.dindex = ndt_list_count(yfms->data.fpln.legs) - 1; // future: may != due to MAP

    /* Future: missed approach legs */

    /* Update index to tracked leg as required */
    if (yfms->data.fpln.mod.operation != YFS_FPLN_MOD_DCTO)
    {
        if (yfms->data.phase <= FMGS_PHASE_PRE)
        {
            // user broke DIRTO on ground, reset
            // to tracking the first leg instead
            if (yfms->data.fpln.w_tp == NULL)
            {
                yfms->data.fpln.lg_idx = +0;
            }
            if (yfms->data.fpln.lg_idx == 0)
            {
                goto end;
            }
        }
        if (tracking_destination)
        {
            yfms->data.fpln.lg_idx = yfms->data.fpln.dindex; goto end;
        }
        yfms->data.fpln.lg_idx = 0; // reset, find leg, update index accordingly
    }
    else
    {
        yfms->data.fpln.lg_idx = 0; // reset, find leg, update index accordingly
    }
    for (int i = 0, ii = ndt_list_count(yfms->data.fpln.legs); i < ii; i++)
    {
        if ((leg = ndt_list_item(yfms->data.fpln.legs, i)))
        {
            if ((yfms->data.fpln.mod.source && yfms->data.fpln.mod.source == leg)  ||
                (yfms->data.fpln.mod.opaque && yfms->data.fpln.mod.opaque == leg->xpfms))
            {
                yfms->data.fpln.lg_idx = i; goto end; // exact same leg
            }
            if (yfms->data.fpln.mod.operation == YFS_FPLN_MOD_SIDP)
            {
                continue; // changed SID, exact leg else keep tracking first
            }
            if (leg->dst && yfms->data.fpln.mod.source->dst &&
                leg->dst == yfms->data.fpln.mod.source->dst)
            {
                yfms->data.fpln.lg_idx = i; continue; // last instance of same waypoint
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

end:/* We should be fully synced with navdlib now */
    /* Adjust line offset after line count change */
    switch (yfms->data.fpln.mod.operation)
    {
        case YFS_FPLN_MOD_NONE:
            break;
        case YFS_FPLN_MOD_SNGL:
        case YFS_FPLN_MOD_MULT:
            if ((index_for_ln_zero -= fpl_getindex_for_line(yfms, 0)))
            {
                yfms->data.fpln.ln_off += index_for_ln_zero;
            }
            break;
        default:
            yfms->data.fpln.ln_off = 0;
            break;
    }
    yfms->data.fpln.mod.index     = 0;
    yfms->data.fpln.mod.opaque    = NULL;
    yfms->data.fpln.mod.source    = NULL;
    yfms->data.fpln.mod.operation = YFS_FPLN_MOD_NONE;
    xplm_fpln_sync(yfms); return yfs_fpln_pageupdt(yfms);
}

void yfs_fpln_trackleg(yfms_context *yfms, int index)
{
    if (index < 0)
    {
        index = yfms->data.fpln.lg_idx;
    }
    for (int i = 0, j = 1; i <= yfms->data.fpln.xplm_last; i++)
    {
        if (i && j && yfms->data.fpln.xplm_info[i].legindex >= index)
        {
            if (yfms->data.fpln.xplm_info[i].legindex > index)
            {
                j = 0; // destination now set to last waypoint of previous leg
            }
            if (j)
            {
                yfms->data.fpln.lg_idx = index;
                XPLMSetDestinationFMSEntry(i);
                return;
            }
        }
    }
}

void yfs_fpln_directto(yfms_context *yfms, int index, ndt_waypoint *toinsert)
{
    // TODO: proper direct to "source" coord adjustment
    // https://en.wikipedia.org/wiki/Standard_rate_turn
    // sim/cockpit/autopilot/heading_roll_mode                int    y  enum        Bank limit - 0 = auto, 1-6 = 5-30 degrees of bank
    // sim/cockpit2/autopilot/bank_angle_mode                 int    y  enum        Maximum bank angle mode, 0->6. Higher number is steeper allowable bank.
    // sim/flightmodel/position/true_airspeed                 float  n  meters/sec  Air speed true - this does not take into account air density at altitude!
    // sim/flightmodel/position/groundspeed                   float  n  meters/sec  The ground speed of the aircraft
    ndt_route_leg *tmp, *leg, *t_p_leg, *dct_leg;
    float groundspeed = XPLMGetDataf(yfms->xpl.groundspeed);
    char buf[23]; int insert_after = 0; ndt_waypoint *t_p_wpt;
    float grd_trk_tru = XPLMGetDataf(yfms->xpl.mag_trk) - XPLMGetDataf(yfms->xpl.mag_var); // == true_psi on ground (-> no wind)
    ndt_distance dnxt = ndt_distance_init(2, NDT_ALTUNIT_NM); //fixme: debug: hardcode to 2 nautical miles for testing pourposes
//  ndt_distance dnxt = ndt_distance_init(groundspeed * 3, NDT_ALTUNIT_ME); // compute airc. pos. ~3 seconds from now (GS in m/s)
    ndt_position ppos = ndt_position_init(XPLMGetDatad(yfms->xpl.latitude), XPLMGetDatad(yfms->xpl.longitude), NDT_DISTANCE_ZERO);
    ndt_position p_tp = ndt_position_calcpos4pbd(ppos, grd_trk_tru, dnxt); snprintf(buf, sizeof(buf), "%+010.6lf/%+011.6lf",
                                                                                    ndt_position_getlatitude (p_tp, NDT_ANGUNIT_DEG),
                                                                                    ndt_position_getlongitude(p_tp, NDT_ANGUNIT_DEG));
#if 0
    ndt_log("YFMS [debug]: direct to PPOS %+010.6lf/%+011.6lf\n", ndt_position_getlatitude(ppos, NDT_ANGUNIT_DEG), ndt_position_getlongitude(ppos, NDT_ANGUNIT_DEG));
    ndt_log("YFMS [debug]: direct to T-P: %+010.6lf/%+011.6lf\n", ndt_position_getlatitude(p_tp, NDT_ANGUNIT_DEG), ndt_position_getlongitude(p_tp, NDT_ANGUNIT_DEG));
    ndt_log("YFMS [debug]: direct to act. di. is %"PRId64"(m)\n", ndt_distance_get(ndt_position_calcdistance(ppos, p_tp), NDT_ALTUNIT_ME));
    ndt_log("YFMS [debug]: direct to est. di. is %"PRId64"(m)\n", ndt_distance_get(dnxt, NDT_ALTUNIT_ME));
    ndt_log("YFMS [debug]: direct to act. HDG is %05.1lf\n",      ndt_position_calcbearing(ppos, p_tp));
    ndt_log("YFMS [debug]: direct to est. HDG is %05.1f\n",       trueheading);
    ndt_log("YFMS [debug]: direct to ground speed is %f\n",       groundspeed);
#endif
    if ((t_p_wpt = ndt_waypoint_llc(buf)) == NULL)
    {
        return yfs_spad_reset(yfms, "UNKNOWN ERROR 1 B", COLR_IDX_ORANGE);      // PAGE_DRTO
    }
    else
    {
        snprintf(t_p_wpt->info.idnt, sizeof(t_p_wpt->info.idnt), "%s", "T-P");
    }
    if (toinsert)
    {
        index = yfms->data.fpln.lg_idx;
        insert_after = index != yfms->data.fpln.dindex;
    }
    if ((leg = ndt_list_item(yfms->data.fpln.legs, index)) == NULL)
    {
        ndt_waypoint_close(&t_p_wpt);
        return yfs_spad_reset(yfms, "UNKNOWN ERROR 1 C", COLR_IDX_ORANGE);      // PAGE_DRTO
    }
    if (toinsert && leg->dst &&
        toinsert == leg->dst)
    {
        // toinsert being tracked already, should use list instead              // PAGE_DRTO
        ndt_waypoint_close(&t_p_wpt); yfms->data.drto.dctwp = NULL;             // PAGE_DRTO
        yfs_drto_pageupdt(yfms); return yfs_spad_reset(yfms, "NOT ALLOWED", -1);// PAGE_DRTO
    }
    if ((t_p_leg = ndt_flightplan_insert_direct(fpl_getfplan_for_leg(yfms, leg), t_p_wpt, leg, insert_after)) == NULL)
    {
        ndt_waypoint_close(&t_p_wpt);
        return yfs_spad_reset(yfms, "UNKNOWN ERROR 1 D", COLR_IDX_ORANGE);      // PAGE_DRTO
    }
    if (yfms->data.fpln.w_tp)
    {
        for (int i = 0, ii = ndt_list_count(yfms->data.fpln.legs); i < ii; i++)
        {
            if ((tmp = ndt_list_item(yfms->data.fpln.legs, i)) &&
                (tmp->dst && tmp->dst == yfms->data.fpln.w_tp))
            {
                if (ndt_flightplan_remove_leg(fpl_getfplan_for_leg(yfms, tmp), tmp) == 0)
                {
                    ndt_waypoint_close(&yfms->data.fpln.w_tp);
                    ndt_route_leg_close(&tmp);
                    break;
                }
            }
        }
        if (yfms->data.fpln.w_tp)
        {
            if (ndt_flightplan_remove_leg(fpl_getfplan_for_leg(yfms, t_p_leg), t_p_leg) == 0)
            {
                ndt_route_leg_close(&t_p_leg); ndt_waypoint_close(&t_p_wpt);
            }
            return yfs_spad_reset(yfms, "UNKNOWN ERROR 1 E", COLR_IDX_ORANGE);  // PAGE_DRTO
        }
    }
    if (toinsert)
    {
        if ((dct_leg = ndt_flightplan_insert_direct(fpl_getfplan_for_leg(yfms, t_p_leg), toinsert, t_p_leg, 1)) == NULL)
        {
            if (ndt_flightplan_remove_leg(fpl_getfplan_for_leg(yfms, t_p_leg), t_p_leg) == 0)
            {
                ndt_route_leg_close(&t_p_leg); ndt_waypoint_close(&t_p_wpt);
            }
            return yfs_spad_reset(yfms, "UNKNOWN ERROR 1 F", COLR_IDX_ORANGE);  // PAGE_DRTO
        }
        yfms->data.fpln.w_tp = t_p_wpt;
    }
    else
    {
        yfms->data.fpln.w_tp = t_p_wpt; dct_leg = leg;
    }
    /*
     * if (toinsert)
     *   - t_p_leg after leg (currently tracked)
     *   - dct_leg after t_p_leg
     * else
     *   - t_p_leg before dct_leg (pre-existing)
     */
    yfms->data.fpln.mod.source    = dct_leg;
    yfms->data.fpln.mod.operation = YFS_FPLN_MOD_DCTO; // compute dct_leg's index
    yfms->data.fpln.mod.opaque    = (void*)dct_leg->xpfms; yfs_fpln_fplnupdt(yfms);
    /*
     * Feature: auto-engage NAV like Airbus does,
     * but only if F/D or A/P is already enabled
     */
    if (XPLMGetDatai(yfms->xpl.fdir_mode) >= 1)
    {
        if (XPLMGetDatai(yfms->xpl.autopilot_source) == 1)
        {
            XPLMCommandOnce(yfms->xpl.gps_select_copilot);
        }
        else
        {
            XPLMCommandOnce(yfms->xpl.gps_select_captain);
        }
        XPLMCommandOnce(yfms->xpl.autopilot_nav);
    }
    return yfs_fpln_pageopen(yfms); // should be tracking correct entry, update page
}

static ndt_waypoint* get_waypoint_from_scratchpad(yfms_context *yfms)
{
    char scrpad[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, scrpad);
    char errbuf[YFS_ROW_BUF_SIZE]; ndt_waypoint *wpt;
    if ((wpt = yfs_main_usrwp(yfms, errbuf, scrpad)))
    {
        // we shouldn't get a standalone waypoint here, since the flight plan is
        // initialized, new waypoints should go in an opaque usrwpt list instead
        ndt_waypoint_close(&wpt); yfs_spad_reset(yfms, "UNKNOWN ERROR 5", COLR_IDX_ORANGE); return NULL;
    }
    if (*errbuf)
    {
        // yfs_main_usrwp matched but encountered error: abort
        yfs_spad_reset(yfms, errbuf, -1); return NULL;
    }
    if ((wpt = yfs_main_getwp(yfms, scrpad)))
    {
        return wpt;
    }
    yfs_spad_reset(yfms, "NOT IN DATA BASE", -1); return NULL;
}

static void lsk_callback_awys(yfms_context *yfms, int key[2], intptr_t refcon)
{
    if (key[1] == 5)
    {
        if (key[0] == 0) // ERASE/RETURN
        {
            yfms->data.fpln.awys.open = 0; return yfs_fpln_pageupdt(yfms);
        }
        if (yfms->data.fpln.awys.awy[0] &&
            yfms->data.fpln.awys.dst[0]) // INSERT
        {
            ndt_route_leg *tk = ndt_list_item(yfms->data.fpln.legs, yfms->data.fpln.lg_idx);
            ndt_route_leg *lg = yfms->data.fpln.awys.leg;
            ndt_waypoint *src = yfms->data.fpln.awys.wpt;
            for (int i = 0; i < 5; i++)
            {
                if (yfms->data.fpln.awys.awy[i] &&
                    yfms->data.fpln.awys.lgi[i] &&
                    yfms->data.fpln.awys.lgo[i] &&
                    yfms->data.fpln.awys.dst[i] && src)
                {
                    if ((lg = ndt_flightplan_insert_airway(yfms->ndt.flp.rte, src,
                                                           yfms->data.fpln.awys.dst[i],
                                                           yfms->data.fpln.awys.awy[i],
                                                           yfms->data.fpln.awys.lgi[i],
                                                           yfms->data.fpln.awys.lgo[i], lg)) == NULL)
                    {
                        yfs_spad_reset(yfms, "UNKNOWN ERROR 1", COLR_IDX_ORANGE);
                        yfms->data.fpln.awys.open  = 0;
                        return yfs_fpln_pageupdt(yfms);
                    }
                    src = yfms->data.fpln.awys.dst[i]; continue;
                }
                break;
            }
            if ((lg = ndt_list_item(yfms->data.fpln.legs, yfms->data.fpln.awys.idx + 1)) &&
                (lg->dst && lg->dst == src)) // src == last valid awys.dst[i]
            {
                // remove consecutive identical waypoints after airway insertion
                if (ndt_flightplan_remove_leg(fpl_getfplan_for_leg(yfms, lg), lg))
                {
                    yfs_spad_reset(yfms, "UNKNOWN ERROR 2", COLR_IDX_ORANGE);
                    yfms->data.fpln.awys.open  = 0;
                    return yfs_fpln_pageupdt(yfms);
                }
            }
            yfms->data.fpln.awys.open     = 0;
            yfms->data.fpln.mod.source    = tk;
            yfms->data.fpln.mod.operation = YFS_FPLN_MOD_MULT;
            yfms->data.fpln.mod.index     = yfms->data.fpln.lg_idx;
            yfms->data.fpln.mod.opaque    = tk ? (void*)tk->xpfms : NULL;
            return yfs_fpln_fplnupdt(yfms);
        }
        return;
    }

    if (key[0] == 0) // VIA
    {
        const char *awy1id; ndt_airway *awy1, *awy2, *valid_awy2 = NULL;
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf); ndt_waypoint *src;
        if (strnlen(buf, 1) == 0)
        {
            return; // no input
        }
        if (key[1] >= 1 && yfms->data.fpln.awys.awy[key[1] - 1] == NULL)
        {
            return; // no active input field
        }
        if (key[1] <= 3 && yfms->data.fpln.awys.awy[key[1] + 1])
        {
            yfs_spad_reset(yfms, "NOT ALLOWED", -1); return; // can only clear or set the last airway
        }
        if (strcmp(buf, "CLR") == 0)
        {
            if (yfms->data.fpln.awys.awy[key[1]] == NULL)
            {
                yfs_spad_reset(yfms, "NOT ALLOWED", -1); return; // don't have anything: can't clear
            }
            yfms->data.fpln.awys.awy[key[1]] = NULL;
            yfms->data.fpln.awys.dst[key[1]] = NULL;
            yfs_spad_clear(yfms); return yfs_fpln_pageupdt(yfms);
        }
        if (yfms->data.fpln.awys.dst[key[1]])
        {
            yfs_spad_reset(yfms, "NOT ALLOWED", -1); return; // can't overwrite leg with termination
        }
        if (key[1] >= 1 && yfms->data.fpln.awys.dst[key[1] - 1] == NULL)
        {
            // two consecutive airways, compute their intersection, if any
            // we already checked yfms->data.fpln.awys.awy[key[1] - 1] above
            // tested under AIRAC 1705 with:
            // LSGG MOLUS N871       Z55       Z50       Z119       UZ613       LSZH
            // LSGG MOLUS N871 BERSU Z55 GERSA Z50 PELAD Z119 RONAG UZ613 NEGRA LSZH
            {
                awy1id = yfms->data.fpln.awys.awy[key[1] - 1]->info.idnt;
                src = key[1] > 1 ? yfms->data.fpln.awys.dst[key[1] - 2] : yfms->data.fpln.awys.wpt;
            }
            for (size_t awy1idx = 0; (awy1 = ndt_navdata_get_airway(yfms->ndt.ndb, awy1id, &awy1idx)); awy1idx++)
            {
                if ((yfms->data.fpln.awys.lgi[key[1] - 1] = ndt_airway_startpoint(awy1, src->info.idnt, src->position)))
                {
                    for (size_t awy2idx = 0; (awy2 = ndt_navdata_get_airway(yfms->ndt.ndb, buf, &awy2idx)); awy2idx++)
                    {
                        if ((yfms->data.fpln.awys.lgo[key[1] - 1] = ndt_airway_intersect(yfms->data.fpln.awys.lgi[key[1] - 1], awy2)))
                        {
                            if ((yfms->data.fpln.awys.dst[key[1] - 1] = ndt_navdata_get_wpt4pos(yfms->ndt.ndb,
                                                                                                yfms->data.fpln.awys.lgo[key[1] - 1]->out.info.idnt, NULL,
                                                                                                yfms->data.fpln.awys.lgo[key[1] - 1]->out.position)))
                            {
                                yfms->data.fpln.awys.awy[key[1] - 1] = awy1;
                                yfms->data.fpln.awys.awy[key[1] + 0] = awy2;
                                yfs_spad_clear(yfms); return yfs_fpln_pageupdt(yfms);
                            }
                        }
                        valid_awy2 = awy2;
                    }
                }
            }
            if (valid_awy2)
            {
                yfms->data.fpln.awys.lgi[key[1] - 1] = NULL;
                yfms->data.fpln.awys.lgo[key[1] - 1] = NULL;
                yfms->data.fpln.awys.dst[key[1] - 1] = NULL;
                yfms->data.fpln.awys.awy[key[1] + 0] = NULL;
                yfs_spad_reset(yfms, "NO INTERSECTION", -1); return;
            }
            else
            {
                yfms->data.fpln.awys.lgi[key[1] - 1] = NULL;
                yfms->data.fpln.awys.lgo[key[1] - 1] = NULL;
                yfms->data.fpln.awys.dst[key[1] - 1] = NULL;
                yfms->data.fpln.awys.awy[key[1] + 0] = NULL;
                yfs_spad_reset(yfms, "NOT IN DATA BASE",-1); return;
            }
        }
        else
        {
            // standard airway segment starting from src == previous dst
            {
                src = key[1] > 0 ? yfms->data.fpln.awys.dst[key[1] - 1] : yfms->data.fpln.awys.wpt;
            }
            for (size_t awy2idx = 0; (awy2 = ndt_navdata_get_airway(yfms->ndt.ndb, buf, &awy2idx)); awy2idx++)
            {
                if (ndt_airway_startpoint(awy2, src->info.idnt, src->position))
                {
                    yfms->data.fpln.awys.awy[key[1] + 0] = awy2;
                    yfs_spad_clear(yfms); return yfs_fpln_pageupdt(yfms);
                }
                valid_awy2 = awy2;
            }
            if (valid_awy2)
            {
                yfms->data.fpln.awys.awy[key[1] + 0] = NULL;
                yfs_spad_reset(yfms,"WAYPOINT MISMATCH",-1); return;
            }
            else
            {
                yfms->data.fpln.awys.awy[key[1] + 0] = NULL;
                yfs_spad_reset(yfms, "NOT IN DATA BASE",-1); return;
            }
        }
    }

    if (key[0] == 1) // TO
    {
        const char *awy2id; ndt_airway *awy2; ndt_waypoint *src, *dst, *valid_dst = NULL;
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (strnlen(buf, 1) == 0)
        {
            return; // no input
        }
        if (key[1] >= 0 && yfms->data.fpln.awys.awy[key[1]] == NULL)
        {
            return; // no active input field
        }
        if (key[1] <= 3 && yfms->data.fpln.awys.awy[key[1] + 1])
        {
            yfs_spad_reset(yfms, "NOT ALLOWED", -1); return; // can only clear or set the last airway
        }
        if (strcmp(buf, "CLR") == 0)
        {
            if (yfms->data.fpln.awys.dst[key[1]] == NULL)
            {
                yfs_spad_reset(yfms, "NOT ALLOWED", -1); return; // don't have anything: can't clear
            }
            yfms->data.fpln.awys.dst[key[1]] = NULL;
            yfs_spad_clear(yfms); return yfs_fpln_pageupdt(yfms);
        }
        else
        {
            // standard airway segment starting from src == previous dst
            {
                awy2id = yfms->data.fpln.awys.awy[key[1]]->info.idnt;
                src = key[1] > 0 ? yfms->data.fpln.awys.dst[key[1] - 1] : yfms->data.fpln.awys.wpt;
            }
            for (size_t awy2idx = 0; (awy2 = ndt_navdata_get_airway(yfms->ndt.ndb, awy2id, &awy2idx)); awy2idx++)
            {
                if ((yfms->data.fpln.awys.lgi[key[1]] = ndt_airway_startpoint(awy2, src->info.idnt, src->position)))
                {
                    for (size_t dstidx = 0; (dst = ndt_navdata_get_waypoint(yfms->ndt.ndb, buf, &dstidx)); dstidx++)
                    {
                        if ((yfms->data.fpln.awys.lgo[key[1]] = ndt_airway_endpoint(yfms->data.fpln.awys.lgi[key[1]], dst->info.idnt, dst->position)))
                        {
                            yfms->data.fpln.awys.dst[key[1]] = dst;
                            yfms->data.fpln.awys.awy[key[1]] = awy2;
                            yfs_spad_clear(yfms); return yfs_fpln_pageupdt(yfms);
                        }
                        valid_dst = dst;
                    }
                }
            }
            if (valid_dst)
            {
                yfms->data.fpln.awys.dst[key[1]] = NULL;
                yfs_spad_reset(yfms,"WAYPOINT MISMATCH",-1); return;
            }
            else
            {
                yfms->data.fpln.awys.dst[key[1]] = NULL;
                yfs_spad_reset(yfms, "NOT IN DATA BASE",-1); return;
            }
        }
    }

    /* all good */
    return;
}

static void lsk_callback_lrev(yfms_context *yfms, int key[2], intptr_t refcon)
{
    if (key[0] == 0 && key[1] == 5)
    {
        yfms->data.fpln.lrev.open = 0; return yfs_fpln_pageupdt(yfms); // RETURN
    }

    if (key[1] == 0) // departure/arrival
    {
        if (key[0] == 0 && yfms->data.fpln.lrev.idx == -1)
        {
            if (!ndt_list_count(yfms->ndt.flp.rte->dep.apt->runways)) // XPLM-only airport
            {
                return yfs_spad_reset(yfms, "NO RUNWAY DATA", -1);
            }
            yfs_spad_reset(yfms, "NOT IMPLEMENTED", -1); return; // TODO        // DEPARTURE
        }
        if (key[0] == 1 && yfms->data.fpln.lrev.idx == yfms->data.fpln.dindex)
        {
            if (!ndt_list_count(yfms->ndt.flp.rte->arr.apt->runways)) // XPLM-only airport
            {
                return yfs_spad_reset(yfms, "NO RUNWAY DATA", -1);
            }
            yfs_spad_reset(yfms, "NOT IMPLEMENTED", -1); return; // TODO        // ARRIVAL
        }
        return;
    }

    if (key[0] == 1) // next wpt, new dest or airways
    {
        ndt_route_leg *trk = ndt_list_item(yfms->data.fpln.legs, yfms->data.fpln.lg_idx);
        ndt_route_leg *leg = yfms->data.fpln.lrev.leg; ndt_waypoint *wpt;
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (key[1] == 2)
        {
            if (strnlen(buf, 1))
            {
                if (yfms->data.fpln.lrev.idx == yfms->data.fpln.dindex)
                {
                    yfs_spad_reset(yfms, "NOT IMPLEMENTED", -1); return; // future
                }
                if ((wpt = get_waypoint_from_scratchpad(yfms)) == NULL)
                {
                    return;
                }
                if (leg && leg->dst && leg->dst == wpt) // check for duplicates
                {
                    yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
                }
                if (yfms->data.fpln.lrev.idx == -1 && leg && leg->src && leg->src == wpt)
                {
                    yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
                }
                // insert after unless we're from the departure's lat. rev. page
                // since in the latter case, leg is not the current but next leg
                if ((ndt_flightplan_insert_direct(fpl_getfplan_for_leg(yfms, leg), wpt, leg, yfms->data.fpln.lrev.idx != -1)) == NULL)
                {
                    yfs_spad_reset(yfms, "UNKNOWN ERROR 3", COLR_IDX_ORANGE); return;
                }
                yfms->data.fpln.lrev.open     = 0;
                yfms->data.fpln.mod.source    = trk;
                yfms->data.fpln.mod.operation = YFS_FPLN_MOD_SNGL;
                yfms->data.fpln.mod.index     = yfms->data.fpln.lg_idx;
                yfms->data.fpln.mod.opaque    = trk ? (void*)trk->xpfms : NULL;
                yfs_spad_clear(yfms); yfs_fpln_fplnupdt(yfms); return; // NEXT WPT
            }
            return;
        }
        if (key[1] == 3 && yfms->data.fpln.lrev.idx != yfms->data.fpln.dindex)
        {
            if (strnlen(buf, 1))
            {
                yfs_spad_reset(yfms, "NOT IMPLEMENTED", -1); return; // TODO    // NEW DEST
            }
            return;
        }
        if (key[1] == 4 && yfms->data.fpln.lrev.leg && yfms->data.fpln.lrev.leg->dst)
        {
            int type = yfms->data.fpln.lrev.leg->dst->type;
            if (type == NDT_WPTYPE_NDB || type == NDT_WPTYPE_VOR ||
                type == NDT_WPTYPE_FIX || type == NDT_WPTYPE_DME)
            {
                yfms->data.fpln.lrev.open = 0;
                yfms->data.fpln.awys.open = 1;
                yfms->data.fpln.awys.idx  = yfms->data.fpln.lrev.idx;
                yfms->data.fpln.awys.leg  = yfms->data.fpln.lrev.leg;
                yfms->data.fpln.awys.wpt  = yfms->data.fpln.lrev.wpt;
                for (int i = 0; i < 5; i++)
                {
                    yfms->data.fpln.awys.dst[i] = NULL;
                    yfms->data.fpln.awys.awy[i] = NULL;
                    yfms->data.fpln.awys.lgi[i] = NULL;
                    yfms->data.fpln.awys.lgo[i] = NULL;
                }
                return yfs_fpln_pageupdt(yfms); // AIRWAYS
            }
        }
        return;
    }

    /* all good */
    return;
}

static void yfs_lsk_callback_fpln(yfms_context *yfms, int key[2], intptr_t refcon)
{
    /* is the page inactive? */
    if (yfms->data.init.ialized == 0)
    {
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if  (strnlen(buf, 1))
        {
            return yfs_spad_reset(yfms, "NOT ALLOWED", -1);
        }
        return;
    }

    /* do we have a sub-page open? */
    if (yfms->data.fpln.awys.open)
    {
        return lsk_callback_awys(yfms, key, refcon);
    }
    if (yfms->data.fpln.lrev.open)
    {
        return lsk_callback_lrev(yfms, key, refcon);
    }

    /* standard line select key configuration */
    if (key[0] == 0) // insert a waypoint, or open the lateral rev. page
    {
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf); ndt_route_leg *leg, *trk; ndt_waypoint *wpt;
        int index = key[1] == 5 ? yfms->data.fpln.dindex : fpl_getindex_for_line(yfms, key[1]);
        if (index < -1) // invalid next waypoint
        {
            yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
        }
        if ((leg = ndt_list_item(yfms->data.fpln.legs, index)) == NULL)
        {
            yfs_spad_reset(yfms, "UNKNOWN ERROR 1 A", COLR_IDX_ORANGE); return;
        }
        if ((trk = ndt_list_item(yfms->data.fpln.legs, yfms->data.fpln.lg_idx)) == NULL)
        {
            yfs_spad_reset(yfms, "UNKNOWN ERROR 1 B", COLR_IDX_ORANGE); return;
        }
        if (strnlen(buf, 1) == 0) // lateral rev. page
        {
            // note: IRL, when:
            // - the leg linked to LSK2 (key[1] == 1) is the tracked leg
            // - the leg linked to LSK1 (key[1] == 0) is NOT the departure
            // pressing LSK1 will open a special "LAT REV FROM PPOS" page
            // not implemented here for the time being (maybe in the future)
            yfms->data.fpln.lrev.open = 1;
            yfms->data.fpln.lrev.idx  = index;
            yfms->data.fpln.lrev.leg  = index == -1 ? ndt_list_item(yfms->data.fpln.legs, 0) : leg;
            if (index == yfms->data.fpln.dindex)
            {
                yfms->data.fpln.lrev.wpt = (yfms->ndt.flp.rte->arr.rwy           ?
                                            yfms->ndt.flp.rte->arr.rwy->waypoint :
                                            yfms->ndt.flp.rte->arr.apt->waypoint);
            }
            else if (index == -1)
            {
                yfms->data.fpln.lrev.wpt = (yfms->ndt.flp.rte->dep.rwy           ?
                                            yfms->ndt.flp.rte->dep.rwy->waypoint :
                                            yfms->ndt.flp.rte->dep.apt->waypoint);
            }
            else
            {
                yfms->data.fpln.lrev.wpt = yfms->data.fpln.lrev.leg->dst;
            }
            return yfs_fpln_pageupdt(yfms);
        }
        if (index == -1 || key[1] == 5) // strnlen(buf, 1) != 0
        {
            // departure/destination: no clearing/insertion
            yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
        }
        if (strcmp(buf, "CLR") == 0)
        {
            if (index == yfms->data.fpln.dindex ||
                index == ndt_list_count(yfms->data.fpln.legs) - 1)
            {
                // clearing destination: would fuck us big time
                yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
            }
            if (index == yfms->data.fpln.lg_idx &&
                yfms->data.phase > FMGS_PHASE_PRE)
            {
                // can't clear currently tracked leg (in flight)
                yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
            }
            if (yfms->data.fpln.w_tp)
            {
                if (yfms->data.fpln.w_tp == leg->src &&
                    index == yfms->data.fpln.lg_idx)
                {
                    // leg is a direct to with T-P src, can't clear
                    yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
                }
                if (yfms->data.fpln.w_tp == leg->dst)
                {
                    if (yfms->data.phase > FMGS_PHASE_PRE &&
                        yfms->data.fpln.w_tp == trk->src)
                    {
                        // T-P source of trk (== cur. leg), can't clear
                        yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
                    }
                    if (ndt_flightplan_remove_leg(fpl_getfplan_for_leg(yfms, leg), leg))
                    {
                        yfs_spad_reset(yfms, "UNKNOWN ERROR 2 A", COLR_IDX_ORANGE); return;
                    }
                    ndt_waypoint_close(&yfms->data.fpln.w_tp);
                    ndt_route_leg_close(&leg);
                    goto do_fplnupdt;
                }
            }
            if (leg->rsg == NULL)
            {
                yfs_spad_reset(yfms, "UNKNOWN ERROR 2 B", COLR_IDX_ORANGE); return;
            }
            if (leg->rsg->type == NDT_RSTYPE_MAP)
            {
                yfs_spad_reset(yfms, "NOT IMPLEMENTED", -1); return;
            }
            if (leg->dst && yfs_main_is_usrwpt(yfms, leg->dst))
            {
                yfs_main_usrwp_unr(yfms, leg->dst);
            }
            if (ndt_flightplan_remove_leg(fpl_getfplan_for_leg(yfms, leg), leg))
            {
                yfs_spad_reset(yfms, "UNKNOWN ERROR 2 C", COLR_IDX_ORANGE); return;
            }
        do_fplnupdt:
            yfms->data.fpln.mod.source = trk;
            yfms->data.fpln.mod.opaque = (void*)trk->xpfms;
            yfms->data.fpln.mod.operation = YFS_FPLN_MOD_SNGL;
            yfms->data.fpln.mod.index = yfms->data.fpln.lg_idx;
            yfs_spad_clear(yfms); return yfs_fpln_fplnupdt(yfms);
        }
        if ((wpt = get_waypoint_from_scratchpad(yfms)) == NULL)
        {
            return;
        }
        if (wpt == leg->src || wpt == leg->dst) // duplicates easy to check here
        {
            yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
        }
        if (index == yfms->data.fpln.lg_idx)
        {
            /*
             * We're trying to insert a leg before the currently tracked leg,
             * which will invariably result in an immediate alteration of the
             * aircraft's flight path. Only allow it when we're not in flight.
             * Also forbid insertion before the currently tracked leg, even on
             * the ground, if we are tracking said leg after using a direct to.
             */
            if (yfms->data.phase > FMGS_PHASE_PRE)
            {
                yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
            }
            if (yfms->data.fpln.w_tp &&
                yfms->data.fpln.w_tp == leg->src)
            {
                // leg is a dirto with T-P src - cannot prepend
                yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
            }
        }
        if ((ndt_flightplan_insert_direct(fpl_getfplan_for_leg(yfms, leg), wpt, leg, 0)) == NULL)
        {
            yfs_spad_reset(yfms, "UNKNOWN ERROR 3", COLR_IDX_ORANGE); return;
        }
        yfms->data.fpln.mod.source    = trk;
        yfms->data.fpln.mod.opaque    = (void*)trk->xpfms;
        yfms->data.fpln.mod.operation = YFS_FPLN_MOD_SNGL;
        yfms->data.fpln.mod.index     = yfms->data.fpln.lg_idx;
        yfs_spad_clear(yfms);   return yfs_fpln_fplnupdt(yfms);
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

static void yfs_msw_callback_fpln(yfms_context *yfms, int rx, int ry, int delta)
{
    if (rx < yfms->mouse_regions[5][0].xmin || // bottom left
        rx > yfms->mouse_regions[0][2].xmax || // top right
        ry < yfms->mouse_regions[5][0].ymin || // bottom left
        ry > yfms->mouse_regions[0][2].ymax)   // top right
    {
        return; // out of bounds
    }
    yfms->data.fpln.ln_off -= delta; yfs_fpln_pageupdt(yfms); return;
}

static int yfs_msc_callback_fpln(yfms_context *yfms, int rx, int ry, int b, XPWidgetMessage m)
{
    if (b != 0 || (m != xpMsg_MouseUp && m != xpMsg_MouseDown))
    {
        return 0; // not applicable
    }
    if (rx < yfms->mouse_regions[5][0].xmin || // bottom left
        rx > yfms->mouse_regions[0][2].xmax || // top right
        ry < yfms->mouse_regions[5][0].ymin || // bottom left
        ry > yfms->mouse_regions[0][2].ymax)   // top right
    {
        return 0; // out of bounds
    }
    if (m == xpMsg_MouseDown)
    {
        return 1; // we don't get mouse up if we don't consume mouse down
    }
    yfms->data.fpln.ln_off = 0; yfs_fpln_pageupdt(yfms); return 1;
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
    int crz_fl = (int)ndt_distance_get(yfms->data.init.crz_alt, NDT_ALTUNIT_FT);
    switch (leg->type)
    {
        default:
            yfs_printf_lft(yfms, row, 0, COLR_IDX_GREEN, "%-7s", leg->dst ? leg->dst->info.idnt : leg->info.idnt);
            break;
    }
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
        if (tr_alt <= alt_const_ft || crz_fl <= alt_const_ft)
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
    if (rwy) // 50 feet above actual threshold elevation, but rounded to 10 feet
    {
        altitude_feet = (((altitude_feet + 5) / 10) * 10) + 50;
    }
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

static ndt_flightplan* fpl_getfplan_for_leg(yfms_context *yfms, ndt_route_leg *leg)
{
    if (leg)
    {
        if (leg == yfms->data.fpln.d_leg)
        {
            return yfms->data.fpln.d_fpl;
        }
        if (leg->rsg == yfms->ndt.flp.dep->dep.sid.enroute.rsgt     ||
            leg->rsg == yfms->ndt.flp.dep->dep.sid.rsgt)
        {
            return yfms->ndt.flp.dep;
        }
        if (leg->rsg == yfms->ndt.flp.arr->arr.star.enroute.rsgt    ||
            leg->rsg == yfms->ndt.flp.arr->arr.star.rsgt)
        {
            return yfms->ndt.flp.arr;
        }
        if (leg->rsg == yfms->ndt.flp.iac->arr.apch.transition.rsgt ||
            leg->rsg == yfms->ndt.flp.iac->arr.apch.rsgt)
        {
            return yfms->ndt.flp.iac;
        }
    }
    return yfms->ndt.flp.rte;
}
