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

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>

#include "common/common.h"
//#include "lib/airport.h"
//#include "lib/flightplan.h"
//#include "lib/navdata.h"

#include "YFSdrto.h"
#include "YFSfpln.h"
#include "YFSmain.h"
#include "YFSspad.h"

static void dct_spc_callback_lnup(yfms_context *yfms);
static void dct_spc_callback_lndn(yfms_context *yfms);

void yfs_drto_pageopen(yfms_context *yfms)//fixme
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
    yfms->data.drto.ln_off = 0;
    //fixme
    yfms->spcs. cback_lnup = (YFS_SPC_f)&dct_spc_callback_lnup;
    yfms->spcs. cback_lndn = (YFS_SPC_f)&dct_spc_callback_lndn;
    yfs_drto_pageupdt(yfms); return;
}

void yfs_drto_pageupdt(yfms_context *yfms)//fixme
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
    yfs_printf_rgt(yfms, 2, 0, COLR_IDX_WHITE, "%s", "----   ---  ");
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
            yfs_printf_lft(yfms, 2 * (j + 2), 0, COLR_IDX_BLUE, "<-%s", leg->dst ? leg->dst->info.idnt : leg->info.idnt);
            yfms->data.drto.idx[j] = (i);
        }
    }

    /* all good */
    return;
}

static void dct_spc_callback_lnup(yfms_context *yfms)
{
    int legct = ndt_list_count(yfms->data.fpln.legs);
    int indx0 = yfms->data.fpln.lg_idx + yfms->data.drto.ln_off;
    if (indx0 > legct - 2)
    {
        yfms->data.drto.ln_off = legct - yfms->data.fpln.lg_idx - 1;
        yfs_drto_pageupdt(yfms); return;
    }
    yfms->data.drto.ln_off++; yfs_drto_pageupdt(yfms); return;
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
