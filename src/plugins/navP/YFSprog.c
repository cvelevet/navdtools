/*
 * YFSprog.c
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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/common.h"
#include "lib/navdata.h"
#include "wmm/wmm.h"

#include "YFSmain.h"
#include "YFSprog.h"
#include "YFSspad.h"

static void yfs_lsk_callback_prog(yfms_context *yfms, int key[2], intptr_t refcon);

void yfs_prog_pageopen(yfms_context *yfms)
{
    if (yfms == NULL)
    {
        return; // no error
    }
    if (yfs_main_newpg(yfms, PAGE_PROG))
    {
        return;
    }
    yfms->lsks[0][0].cback =
    yfms->lsks[0][3].cback = yfms->lsks[1][3].cback = (YFS_LSK_f)&yfs_lsk_callback_prog;
    yfs_prog_pageupdt(yfms); return;
}

void yfs_prog_pageupdt(yfms_context *yfms)
{
    /* reset lines before drawing */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }

    /* line 0: main header (green, left, offset 3) */
//  yfs_printf_lft(yfms,  0,  0, COLR_IDX_GREEN, "%s", "  PREFLIGHT"); // TODO: phase of flight
    yfs_printf_lft(yfms,  0,  0, COLR_IDX_GREEN, "%s", "  PROGRESS");
    if (yfms->data.init.flight_id[0])
    {
        yfs_printf_rgt(yfms, 0, 2, COLR_IDX_WHITE, "%s", yfms->data.init.flight_id);
    }

    /* line 1: headers (white) */
    yfs_printf_lft(yfms,  1,  0, COLR_IDX_WHITE, "%s", " CRZ");
    yfs_printf_lft(yfms,  1,  9, COLR_IDX_WHITE, "%s", " OPT");
    yfs_printf_rgt(yfms,  1,  0, COLR_IDX_WHITE, "%s", "REC MAX");

    /* lines 7, 8: headers (white) */
    yfs_printf_lft(yfms,  7,  0, COLR_IDX_WHITE, "%s", " BRG/DIST");

    /* lines 10, 11: headers (green, white) */
//  yfs_printf_rgt(yfms, 10,  0, COLR_IDX_GREEN, "%s",              "GPS PRIMARY"); // we don't simulate any IRS system(s) at this point
    yfs_printf_ctr(yfms, 11,     COLR_IDX_WHITE, "%s", "REQUIRED ACCUR ESTIMATED");
    yfs_printf_rgt(yfms, 12,  0, COLR_IDX_GREEN, "%s",           "----    -.--NM");
    yfs_printf_lft(yfms, 12,  0, COLR_IDX_BLUE,  "%s",                     "-.--");
    yfs_printf_lft(yfms, 12,  4, COLR_IDX_WHITE, "%s",                       "NM");

    /* line 2: flight levels */
    if (ndt_distance_get(yfms->data.init.crz_alt, NDT_ALTUNIT_NA))
    {
        int crzfl = (int)ndt_distance_get(yfms->data.init.crz_alt, NDT_ALTUNIT_FL);
        int crzft = (int)ndt_distance_get(yfms->data.init.crz_alt, NDT_ALTUNIT_FT);
        int trans = (int)ndt_distance_get(yfms->data.init.trans_a, NDT_ALTUNIT_FT);
        if (trans < crzft)
        {
            yfs_printf_lft(yfms, 2, 0, COLR_IDX_BLUE, "FL%03d", crzfl);
        }
        else
        {
            yfs_printf_lft(yfms, 2, 0, COLR_IDX_BLUE,    "%5d", crzft);
        }
    }
    else
    {
        yfs_printf_lft(yfms, 2,  0, COLR_IDX_WHITE, "%s", "-----");
    }
    yfs_printf_lft(yfms, 2,  9, COLR_IDX_WHITE,     "%s", "-----");
    yfs_printf_rgt(yfms, 2,  1, COLR_IDX_WHITE,     "%s", "-----");

    /* line 8: bearing, distance */
    if (yfms->data.prog.fix == NULL)
    {
        yfs_printf_lft(yfms,  8,  0, COLR_IDX_WHITE, "%s", " ---/--.-");
        yfs_printf_lft(yfms,  8, 11, COLR_IDX_WHITE, "%s",    "  TO  ");
        yfs_printf_rgt(yfms,  8,  0, COLR_IDX_BLUE,  "%s",   "[    ] ");
    }
    else
    {
        ndt_date      now = ndt_date_now();
        void         *wmm = yfms->ndt.ndb->wmm;
        ndt_position from = yfms->data.aircraft_pos;
        ndt_position   to = yfms->data.prog.fix->position;
        ndt_distance dist = ndt_position_calcdistance(from, to);
        double       trub = ndt_position_calcbearing (from, to);
        double       magb = ndt_wmm_getbearing_mag   (wmm, trub, from, now);
        yfs_printf_lft(yfms,  8,  0, COLR_IDX_GREEN, " %03.0lf/%-6.1lf", round(magb), (double)ndt_distance_get(dist, NDT_ALTUNIT_ME) / 1852.);
        yfs_printf_lft(yfms,  8, 11, COLR_IDX_WHITE, "  TO  ");
        yfs_printf_rgt(yfms,  8,  0, COLR_IDX_BLUE,  "%-7s",  yfms->data.prog.fix->info.idnt);
    }

    /* all good */
    return;
}

static void yfs_lsk_callback_prog(yfms_context *yfms, int key[2], intptr_t refcon)
{
    if (key[1] == 0) // updating cruise altitude (only works if already set)
    {
        return;      // TODO: implement
    }
    if (key[0] == 0 && key[1] == 3) // future: specify required accuracy
    {
        return;      // TODO: implement
    }
    if (key[0] == 1 && key[1] == 3) // setting the reference fix
    {
        char scrpad[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, scrpad);
        char errbuf[YFS_ROW_BUF_SIZE]; ndt_waypoint *wpt;
        if  (scrpad[0] == 0)
        {
            if (yfms->data.prog.fix)
            {
                snprintf(scrpad, sizeof(scrpad), "%s", yfms->data.prog.fix->info.idnt);
                yfs_spad_reset(yfms, scrpad, -1); return; // current fix to scratchpad
            }
            return;
        }
        if (!strcmp(scrpad, "CLR"))
        {
            if (yfs_main_is_usrwpt(yfms, yfms->data.prog.fix))
            {
                yfs_main_usrwp_unr(yfms, yfms->data.prog.fix);
            }
            else if (yfms->data.prog.usrwpt)
            {
                ndt_waypoint_close(&yfms->data.prog.usrwpt);
            }
            yfms->data.prog.fix = NULL; yfs_spad_clear(yfms); yfs_prog_pageupdt(yfms); return;
        }
        if ((wpt = yfs_main_usrwp(yfms, errbuf, scrpad)))
        {
            if (yfs_main_is_usrwpt(yfms, yfms->data.prog.fix))
            {
                yfs_main_usrwp_unr(yfms, yfms->data.prog.fix);
            }
            else if (yfms->data.prog.usrwpt)
            {
                ndt_waypoint_close(&yfms->data.prog.usrwpt);
            }
            if (yfs_main_is_usrwpt(yfms, wpt) == 0)
            {
                yfms->data.prog.usrwpt = wpt;
            }
            yfms->data.prog.fix = wpt; yfs_spad_clear(yfms); yfs_prog_pageupdt(yfms); return;
        }
        if (*errbuf)
        {
            // yfs_main_usrwp matched but encountered error: abort
            yfs_spad_reset(yfms, errbuf, -1); return;
        }
        if ((wpt = yfs_main_getwp(yfms, scrpad)))
        {
            if (yfs_main_is_usrwpt(yfms, yfms->data.prog.fix))
            {
                yfs_main_usrwp_unr(yfms, yfms->data.prog.fix);
            }
            else if (yfms->data.prog.usrwpt)
            {
                ndt_waypoint_close(&yfms->data.prog.usrwpt);
            }
            yfms->data.prog.fix = wpt; yfs_spad_clear(yfms); yfs_prog_pageupdt(yfms); return;
        }
        yfs_spad_reset(yfms, "NOT IN DATA BASE", -1); return;
    }
    /* all good */
    return;
}
