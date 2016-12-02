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

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>

#include "common/common.h"
#include "lib/airport.h"
#include "lib/flightplan.h"
#include "lib/navdata.h"

#include "YFSfpln.h"
#include "YFSmain.h"
#include "YFSspad.h"

static void yfs_lsk_callback_fpln(yfms_context *yfms, int key[2],                intptr_t refcon);
static void fpl_print_leg_generic(yfms_context *yfms, int row,                ndt_route_leg *leg);
static void fpl_print_airport_rwy(yfms_context *yfms, int row, ndt_airport *apt, ndt_runway *rwy);

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
    if (yfms->data.fpln.ln_off == 0)
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
        yfs_printf_lft(yfms,  2, 0, COLR_IDX_WHITE, "%s", "----- END OF F-PLN -----");
        yfs_printf_lft(yfms,  4, 0, COLR_IDX_WHITE, "%s", "----- NO ALTN FPLN -----");
        return;
    }

    /* final destination :D */
    yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "%-4s%-3s ----  ----  ----", // TODO: TIME, DIST
                   yfms->ndt.flp.rte->arr.apt->info.idnt,
                   yfms->ndt.flp.rte->arr.rwy ? yfms->ndt.flp.rte->arr.rwy->info.idnt : "");

    /* two lines per leg (header/course/distance, then name/constraints) */
    int legnum = ndt_list_count(yfms->data.fpln.legs);
    for (int i = -1; i < 4; i++)
    {
        int index = yfms->data.fpln.lg_idx + yfms->data.fpln.ln_off + i, actual_leg = 0;
        if (index < -1 || index >= legnum)
        {
            while (index > legnum)
            {
                index -= legnum + 4; // legnum + 1 == -3, legnum + 2 == -2, etc.
            }
            while (index < -4)
            {
                index += legnum + 4; // -5 == legnum - 1, -6 == legnum - 2, etc.
            }
            if (index == legnum)
            {
                index = -4;
            }
        }
        switch (index)
        {
            case -3:
                yfs_printf_lft       (yfms, (2 * (i + 2)), 0, COLR_IDX_WHITE, "%s",    "----- END OF F-PLN -----");
                break;
            case -2:
                yfs_printf_lft       (yfms, (2 * (i + 2)), 0, COLR_IDX_WHITE, "%s",    "----- NO ALTN FPLN -----");
                break;
            case -1: // departure airport
                fpl_print_airport_rwy(yfms, (2 * (i + 2)), yfms->ndt.flp.rte->dep.apt, yfms->ndt.flp.rte->dep.rwy);
                actual_leg = 1; break;//fixme
            case -4: // arrival airport
                fpl_print_airport_rwy(yfms, (2 * (i + 2)), yfms->ndt.flp.rte->arr.apt, yfms->ndt.flp.rte->arr.rwy);
                actual_leg = 1; break;
            default: // regular leg
                fpl_print_leg_generic(yfms, (2 * (i + 2)),             ndt_list_item(yfms->data.fpln.legs, index));
                actual_leg = 1; break;
        }
        if (actual_leg)
        {
            switch (i)
            {
                case -1: // column headers
                    yfs_printf_lft(yfms, ((2 * (i + 2)) - 1), 0, COLR_IDX_WHITE, "%s", "        TIME  SPD/ALT");
                    break;
                default: // course, distance
                    //fixme
                    break;
            }
        }
    }

    /* all good */
    return;
}

static void yfs_lsk_callback_fpln(yfms_context *yfms, int key[2], intptr_t refcon)//fixme
{
    /* all good */
    return;
}

static void fpl_print_leg_generic(yfms_context *yfms, int row, ndt_route_leg *leg)
{
    //fixme
}

static void fpl_print_airport_rwy(yfms_context *yfms, int row, ndt_airport *apt, ndt_runway *rwy)
{
    if (rwy)
    {
        yfs_printf_lft(yfms, row, 0, COLR_IDX_GREEN, "%-7s ----  ---/%6d", rwy->waypoint->info.idnt,
                       ndt_distance_get(ndt_position_getaltitude(rwy->threshold), NDT_ALTUNIT_FT)); return;
    }
    yfs_printf_lft(yfms, row, 0, COLR_IDX_GREEN, "%-7s ----  ---/%6d", apt->info.idnt,
                   ndt_distance_get(ndt_position_getaltitude(apt->coordinates), NDT_ALTUNIT_FT)); return;
}