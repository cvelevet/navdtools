/*
 * YFSdrto.c
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
#include "lib/flightplan.h"
#include "lib/navdata.h"

#include "YFSdrto.h"
#include "YFSfpln.h"
#include "YFSmain.h"
#include "YFSspad.h"

static void dct_spc_callback_lnup(yfms_context *yfms                             );
static void dct_spc_callback_lndn(yfms_context *yfms                             );
static void yfs_lsk_callback_drto(yfms_context *yfms, int key[2], intptr_t refcon);

void yfs_drto_pageopen(yfms_context *yfms)
{
    if (yfms == NULL)
    {
        return; // no error
    }
    if (yfms->data.init.ialized)
    {
        if (yfs_main_newpg(yfms, PAGE_DRTO))
        {
            return;
        }
    }
    else
    {
        return; // don't open this page unless we already have a flight plan
    }
    yfms->lsks[0][0].cback =
    yfms->lsks[0][1].cback =
    yfms->lsks[0][2].cback =
    yfms->lsks[0][3].cback =
    yfms->lsks[0][4].cback =
    yfms->lsks[0][5].cback = yfms->lsks[1][5].cback= (YFS_LSK_f)&yfs_lsk_callback_drto;
    yfms->spcs. cback_lnup = (YFS_SPC_f)&dct_spc_callback_lnup;
    yfms->spcs. cback_lndn = (YFS_SPC_f)&dct_spc_callback_lndn;
    yfms->data.drto.ln_off = 0;
    yfms->data.drto.dctlg  = NULL;
    yfms->data.drto.dctwp  = NULL;
    yfs_drto_pageupdt(yfms); return;
}

void yfs_drto_pageupdt(yfms_context *yfms)
{
    /* reset lines, set page title */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }
    yfms->data.drto.idx[0] =
    yfms->data.drto.idx[1] =
    yfms->data.drto.idx[2] =
    yfms->data.drto.idx[3] =
    yfms->data.drto.idx[4] = -1;

    /* default state: no direct selected yet */
    yfs_printf_ctr(yfms, 0,    COLR_IDX_WHITE, "%s",       "DIR TO");
    yfs_printf_lft(yfms, 1, 0, COLR_IDX_WHITE, "%s",     "WAYPOINT");
    yfs_printf_rgt(yfms, 1, 0, COLR_IDX_WHITE, "%s",  "UTC  DIST  ");
    yfs_printf_lft(yfms, 2, 0, COLR_IDX_BLUE,  "%s",      "[     ]");
    yfs_printf_rgt(yfms, 2, 0, COLR_IDX_WHITE, "%s",  "---  ----  ");
    yfs_printf_lft(yfms, 3, 0, COLR_IDX_WHITE, "%s",   "F-PLN WPTS");
    yfs_printf_rgt(yfms, 4, 0, COLR_IDX_BLUE,  "%s",  "DIRECT TO  ");

    /* remaining flight plan leg list */
    int legct = ndt_list_count(yfms->data.fpln.legs);
    int indx0 = yfms->data.fpln.lg_idx + yfms->data.drto.ln_off;
    for (int i = indx0, j = 0; i < legct && j < 5; i++, j++)
    {
        ndt_route_leg *leg = ndt_list_item(yfms->data.fpln.legs, i);
        while (leg != NULL) // skip discontinuities, holds etc.
        {
            // TODO: skip holds???
            if (leg->type == NDT_LEGTYPE_ZZ)
            {
                leg = ndt_list_item(yfms->data.fpln.legs, ++i);
                continue;
            }
            if (leg->dst == NULL && !ndt_list_count(leg->xpfms))
            {
                leg = ndt_list_item(yfms->data.fpln.legs, ++i);
                continue;
            }
            break;
        }
        if (leg)
        {
            yfms->data.drto.idx[j] = (i);
            yfs_printf_lft(yfms, 2 * (j + 2), 0, COLR_IDX_BLUE, "<%s", leg->dst ? leg->dst->info.idnt : leg->info.idnt);
            if (yfms->data.drto.dctlg && yfms->data.drto.dctidx == i)
            {
                yfs_printf_lft(yfms, 2 * (j + 2), 0, COLR_IDX_BLUE, "%s", " ");
            }
        }
    }

    /* alternate state: pre-selected direct to */
    if (yfms->data.drto.dctlg || yfms->data.drto.dctwp)
    {
        const char *drto_idnt = yfms->data.drto.dctlg ? (yfms->data.drto.dctlg->dst ? yfms->data.drto.dctlg->dst->info.idnt : yfms->data.drto.dctlg->info.idnt) : yfms->data.drto.dctwp->info.idnt;
        yfs_printf_ctr(yfms,  0,    COLR_IDX_YELLOW, "%s",       "DIR TO");
        yfs_printf_lft(yfms,  2, 0, COLR_IDX_YELLOW, "%-7s",    drto_idnt);
        yfs_printf_rgt(yfms,  2, 0, COLR_IDX_YELLOW, "%s",  "---  ----  ");
        yfs_printf_rgt(yfms,  4, 0, COLR_IDX_YELLOW, "%s",  "DIRECT TO  ");
        yfs_printf_lft(yfms, 11, 0, COLR_IDX_ORANGE, "%s",      " DIR TO");
        yfs_printf_lft(yfms, 12, 0, COLR_IDX_ORANGE, "%-9s",     "<ERASE");
        yfs_printf_rgt(yfms, 11, 0, COLR_IDX_ORANGE, "%s",      "DIR TO ");
        yfs_printf_rgt(yfms, 12, 0, COLR_IDX_ORANGE, "%s",      "INSERT*");
    }

    /* all good */
    return;
}

static void dct_spc_callback_lnup(yfms_context *yfms)
{
    yfms->data.drto.ln_off++;
    int legct, legrm = (legct = ndt_list_count(yfms->data.fpln.legs)) - yfms->data.fpln.lg_idx;
    int indx0 = yfms->data.fpln.lg_idx + yfms->data.drto.ln_off;
    int ndisp = legct - indx0;
    if (legrm > 5) legrm = 5;
    if (legrm > ndisp)
    {
        yfms->data.drto.ln_off -= legrm - ndisp;
    }
    yfs_drto_pageupdt(yfms); return;
}

static void dct_spc_callback_lndn(yfms_context *yfms)
{
    if (yfms->data.drto.ln_off < 1)
    {
        yfms->data.drto.ln_off = 0;
        yfs_drto_pageupdt(yfms); return;
    }
    yfms->data.drto.ln_off--; yfs_drto_pageupdt(yfms); return;
}

static void yfs_lsk_callback_drto(yfms_context *yfms, int key[2], intptr_t refcon)
{
    if (key[1] == 5 && (yfms->data.drto.dctlg || yfms->data.drto.dctwp))
    {
        if (key[0] == 0) // DIR TO ERASE
        {
            yfms->data.drto.dctlg = NULL;
            yfms->data.drto.dctwp = NULL;
            yfs_drto_pageupdt(yfms); return;
        }
        if (key[0] ==1) // DIR TO INSERT
        {
            if (yfms->data.drto.dctlg)
            {
                yfs_fpln_directto(yfms, yfms->data.drto.dctidx, NULL); return;
            }
            if (yfms->data.drto.dctwp)
            {
                yfs_fpln_directto(yfms, yfms->data.fpln.lg_idx, yfms->data.drto.dctwp); return;
            }
            yfs_spad_reset(yfms, "UNKNOWN ERROR 1 C", COLR_IDX_ORANGE); return;
        }
    }
    if (key[0] == 0) // select a waypoint to fly direct to
    {
        if (key[1] <= 0) // manually-entered waypoint, not in flight plan
        {
            char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
            if (!strnlen(buf, 1))
            {
                yfs_spad_reset(yfms, "NOT ALLOWED", -1); return;
            }
            if ((yfms->data.drto.dctwp = ndt_navdata_get_wptnear2(yfms->ndt.ndb, buf, NULL,
                                                                  yfms->data.aircraft_pos)) == NULL)
            {
                // TODO: disambiguation page
                yfs_spad_reset(yfms, "NOT IN DATA BASE", -1); return;
            }
            yfms->data.drto.dctlg = NULL; yfs_spad_clear(yfms); yfs_drto_pageupdt(yfms); return;
        }
        if (key[1] <= 5 && yfms->data.drto.idx[key[1] - 1] != -1) // from flight plan
        {
            yfms->data.drto.dctlg = ndt_list_item((yfms->data.fpln.legs),
                                                  (yfms->data.drto.dctidx =
                                                   yfms->data.drto.idx[key[1] - 1]));
            yfms->data.drto.dctwp = NULL; yfs_drto_pageupdt(yfms); return;
        }
        return; // nothing to do
    }
}
