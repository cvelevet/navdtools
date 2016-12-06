/*
 * YFSinit.c
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

static void yfs_lsk_callback_fpln(yfms_context *yfms, int key[2], intptr_t refcon)//fixme
{
    if (key[0] == 0) // insert a waypoint, or open the lateral rev. page
    {
        ndt_waypoint      *cur_wpt, *new_wpt;
        ndt_route_leg     *cur_leg, *new_leg;
        ndt_route_segment *cur_rsg, *new_wrsg;
        int legct = ndt_list_count(yfms->data.fpln.legs);
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        int index = key[1] == 5 ? yfms->data.fpln.dindex : fpl_getindex_for_line(yfms, key[1]);
        if (index < 0) // next waypoint is origin or invalid
        {
            yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
        }
        if (key[1] == 5 || strnlen(buf, 1) == 0) // open lateral revision page
        {
            // TODO: lateral revision page
        }
        //fixme
    }
    if (key[0] == 1 && key[1] != 5) // constraints or vertical rev. page
    {
        //fixme
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
            case NDT_RESTRICT_AB:
            case NDT_RESTRICT_BT: // not fully supported
                alt_const_ft = ndt_distance_get(restrs.altitude.min, NDT_ALTUNIT_FT);
                alt_const_fl = ndt_distance_get(restrs.altitude.min, NDT_ALTUNIT_FL);
                restr_type = "+";
                break;
            case NDT_RESTRICT_BL:
                alt_const_ft = ndt_distance_get(restrs.altitude.min, NDT_ALTUNIT_FT);
                alt_const_fl = ndt_distance_get(restrs.altitude.min, NDT_ALTUNIT_FL);
                restr_type = "-";
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
            yfs_printf_rgt(yfms, row, 0, COLR_IDX_MAGENTA,  "%5.5d%s", alt_const_ft, restr_type);
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