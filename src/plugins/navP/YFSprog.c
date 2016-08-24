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

    /* buffers */
    char buf1[YFS_DISPLAY_NUMC + 1], buf2[YFS_DISPLAY_NUMC + 1];

    /* relevant data */
    int fl_crz = (int)ndt_distance_get(yfms->ndt.alt.crz, NDT_ALTUNIT_FL);

    /* line 0: main header (green, left, offset 3) */
//  yfs_printf_lft(yfms,  0,  0, COLR_IDX_GREEN, "  PREFLIGHT"); // TODO: phase of flight
    yfs_printf_lft(yfms,  0,  0, COLR_IDX_GREEN, "  PROGRESS");
    if (yfms->ndt.flight_number[0])
    {
        yfs_printf_rgt(yfms, 0, 2, COLR_IDX_WHITE, yfms->ndt.flight_number);
    }

    /* line 1: headers (white) */
    yfs_printf_lft(yfms,  1,  0, COLR_IDX_WHITE, " CRZ");
    yfs_printf_lft(yfms,  1,  9, COLR_IDX_WHITE, " OPT");
    yfs_printf_rgt(yfms,  1,  0, COLR_IDX_WHITE, "REC MAX");

    /* lines 7, 8: headers (white) */
    yfs_printf_lft(yfms,  7,  0, COLR_IDX_WHITE, " BRG/DIST");

    /* lines 10, 11: headers (green, white) */
//  yfs_printf_rgt(yfms, 10,  0, COLR_IDX_GREEN,              "GPS PRIMARY"); // we don't simulate any IRS system(s) at this point
    yfs_printf_ctr(yfms, 11,     COLR_IDX_WHITE, "REQUIRED ACCUR ESTIMATED");
    yfs_printf_rgt(yfms, 12,  0, COLR_IDX_GREEN,           "----    -.--NM");
    yfs_printf_lft(yfms, 12,  0, COLR_IDX_BLUE,                      "-.--");
    yfs_printf_lft(yfms, 12,  4, COLR_IDX_WHITE,                       "NM");

    /* line 2: flight levels */
    if (fl_crz < 1)
    {
        yfs_printf_lft(yfms,  2,  0, COLR_IDX_WHITE,   "-----");
        yfs_printf_lft(yfms,  2,  9, COLR_IDX_GREEN,   "FL---");
        yfs_printf_rgt(yfms,  2,  1, COLR_IDX_MAGENTA, "FL---");
    }
    else
    {
        snprintf(buf1, 6, "FL%03d", fl_crz);
        yfs_printf_lft(yfms,  2,  0, COLR_IDX_BLUE,       buf1);
        yfs_printf_lft(yfms,  2,  9, COLR_IDX_GREEN,   "FL---");
        yfs_printf_rgt(yfms,  2,  1, COLR_IDX_MAGENTA, "FL---");
    }

    /* line 8: bearing, distance */
    if (yfms->ndt.fix_nfo == NULL)
    {
        yfs_printf_lft(yfms,  8,  0, COLR_IDX_WHITE, " ---/--.-");
        yfs_printf_lft(yfms,  8, 11, COLR_IDX_WHITE,    "  TO  ");
        yfs_printf_rgt(yfms,  8,  0, COLR_IDX_BLUE,    "[    ] ");
    }
    else
    {
        ndt_date      now = ndt_date_now();
        void         *wmm = yfms->ndt.ndb->wmm;
        ndt_position from = yfms->data.aircraft_pos;
        ndt_position   to = yfms->ndt.fix_nfo->position;
        ndt_distance dist = ndt_position_calcdistance(from, to);
        double       trub = ndt_position_calcbearing (from, to);
        double       magb = ndt_wmm_getbearing_mag   (wmm, trub, from, now);
        sprintf(buf1, " %03.0lf/%-6.1lf", round(magb), (double)ndt_distance_get(dist, NDT_ALTUNIT_ME) / 1852.);
        sprintf(buf2, "%-7s", yfms->ndt.fix_nfo->info.idnt);
        yfs_printf_lft(yfms,  8,  0, COLR_IDX_GREEN,     buf1);
        yfs_printf_lft(yfms,  8, 11, COLR_IDX_WHITE, "  TO  ");
        yfs_printf_rgt(yfms,  8,  0, COLR_IDX_BLUE,      buf2);
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
        ndt_waypoint *wpt; char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0 && yfms->ndt.fix_nfo)
        {
            snprintf(buf, sizeof(buf), "%s", yfms->ndt.fix_nfo->info.idnt);
            yfs_spad_reset(yfms, buf, -1); return; // current fix to scratchpad
        }
        if (!strcmp(buf, "CLR"))
        {
            yfms->ndt.fix_nfo = NULL; yfs_spad_clear(yfms); yfs_prog_pageupdt(yfms); return;
        }
        // TODO: place/bearing/distance and others
        // TODO: disambiguation page for duplicates
        if ((wpt = ndt_navdata_get_wptnear2(yfms->ndt.ndb, buf, NULL, yfms->data.aircraft_pos)) == NULL)
        {
            yfs_spad_reset(yfms, "NOT IN DATA BASE", -1); return;
        }
        yfms->ndt.fix_nfo = wpt; yfs_spad_clear(yfms); yfs_prog_pageupdt(yfms); return;
    }
    /* all good */
    return;
}
