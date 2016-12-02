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
#include "lib/flightplan.h"
#include "lib/navdata.h"

#include "YFSinit.h"
#include "YFSmain.h"
#include "YFSspad.h"

static void yfs_lsk_callback_init(yfms_context *yfms, int key[2], intptr_t refcon);

void yfs_init_pageopen(yfms_context *yfms)
{
    if (yfms == NULL)
    {
        return; // no error
    }
    if (yfs_main_newpg(yfms, PAGE_INIT))
    {
        return;
    }
    yfms->lsks[0][0].cback = yfms->lsks[1][0].cback =
    yfms->lsks[0][2].cback = yfms->lsks[1][2].cback =
    yfms->lsks[0][4].cback = yfms->lsks[1][4].cback =
    yfms->lsks[0][5].cback = yfms->lsks[1][5].cback = (YFS_LSK_f)&yfs_lsk_callback_init;
    yfs_init_pageupdt(yfms); return;
}

void yfs_init_pageupdt(yfms_context *yfms)
{
    /* reset lines, set page title */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }
    yfs_printf_ctr(yfms, 0, COLR_IDX_WHITE, "%s", "INIT");

    /* aircraft GPS position */
    char gps_coodinates_buf[17];
    if (yfms->data.init.ialized)
    {
        if (yfms->data.init.aligned)
        {
            // TODO: animate transition between reference and actual position
            ndt_position_sprintllc(yfms->data.aircraft_pos, NDT_LLCFMT_AIBUS,
                                   gps_coodinates_buf, sizeof(gps_coodinates_buf));
        }
        else
        {
            ndt_position_sprintllc(yfms->data.init.from->coordinates, NDT_LLCFMT_AIBUS,
                                   gps_coodinates_buf, sizeof(gps_coodinates_buf));
        }
    }

    /* left column */
    yfs_printf_lft(yfms,  1, 0, COLR_IDX_WHITE,  "%s", "  CO RTE"); // TODO: implement company route support
    yfs_printf_lft(yfms,  2, 0, COLR_IDX_BLUE,   "%s", "N/A");
//  yfs_printf_lft(yfms,  3, 0, COLR_IDX_WHITE,  "%s", "ALTN RTE");
//  yfs_printf_lft(yfms,  4, 0, COLR_IDX_WHITE,  "%s", "--------");
    yfs_printf_lft(yfms,  5, 0, COLR_IDX_WHITE,  "%s", "FLT NBR");
    yfs_printf_lft(yfms,  6, 0, COLR_IDX_ORANGE, "%s", "#######");
    yfs_printf_lft(yfms,  7, 0, COLR_IDX_WHITE,  "%s", "LAT");
    yfs_printf_lft(yfms,  8, 0, COLR_IDX_WHITE,  "%s", "----.--");
    yfs_printf_lft(yfms,  9, 0, COLR_IDX_WHITE,  "%s", "COST INDEX");
    yfs_printf_lft(yfms, 10, 0, COLR_IDX_WHITE,  "%s", "---");
    yfs_printf_lft(yfms, 11, 0, COLR_IDX_WHITE,  "%s", "CRZ FL"/*"/TEMP"*/);
    yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE,  "%s", "-----"/*" /---"*/);
    if (yfms->data.init.flight_id[0])
    {
        yfs_printf_lft(yfms, 6, 0, COLR_IDX_BLUE, "%-7s", yfms->data.init.flight_id);
    }
    if (yfms->data.init.ialized)
    {
        if (ndt_distance_get(yfms->data.init.crz_alt, NDT_ALTUNIT_NA))
        {
            int crzfl = (int)ndt_distance_get(yfms->data.init.crz_alt, NDT_ALTUNIT_FL);
            int crzft = (int)ndt_distance_get(yfms->data.init.crz_alt, NDT_ALTUNIT_FT);
            int trans = (int)ndt_distance_get(yfms->data.init.trans_a, NDT_ALTUNIT_FT);
            if (trans < crzft)
            {
                yfs_printf_lft(yfms, 12, 0, COLR_IDX_BLUE, "FL%03d", crzfl);
            }
            else
            {
                yfs_printf_lft(yfms, 12, 0, COLR_IDX_BLUE,    "%5d", crzft);
            }
        }
        else
        {
            yfs_printf_lft(yfms, 12, 0, COLR_IDX_ORANGE, "%s", "#####"/*" /###"*/);
        }
        yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE, "%.7s", gps_coodinates_buf);
        yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE, "%3d", yfms->data.init.cost_index);
    }

    /* right column */
    yfs_printf_rgt(yfms,  1, 0, COLR_IDX_WHITE,  "%s", "FROM/TO  ");
    yfs_printf_rgt(yfms,  2, 0, COLR_IDX_ORANGE, "%s", "####/####");
//  yfs_printf_rgt(yfms,  3, 0, COLR_IDX_WHITE,  "%s",      "ALTN");
//  yfs_printf_rgt(yfms,  4, 0, COLR_IDX_WHITE,  "%s",      "----");
    yfs_printf_rgt(yfms,  7, 0, COLR_IDX_WHITE,  "%s",      "LONG");
    yfs_printf_rgt(yfms,  8, 0, COLR_IDX_WHITE,  "%s",  "-----.--");
//  yfs_printf_rgt(yfms, 11, 0, COLR_IDX_WHITE,  "%s",     "WIND>");
    yfs_printf_rgt(yfms, 11, 0, COLR_IDX_WHITE,  "%s",     "TROPO");
    yfs_printf_rgt(yfms, 12, 0, COLR_IDX_BLUE,   "%s",       "N/A"); // TODO: implement tropopause setting and temperature at cruise altitude
    if (yfms->data.init.from)
    {
        yfs_printf_rgt(yfms, 2, 5, COLR_IDX_BLUE, "%s", yfms->data.init.from->info.idnt);
    }
    if (yfms->data.init.to)
    {
        yfs_printf_rgt(yfms, 2, 0, COLR_IDX_BLUE, "%s", yfms->data.init.to->info.idnt);
    }
    if (yfms->data.init.ialized)
    {
        if (yfms->data.init.aligned == 0)
        {
            yfs_printf_rgt(yfms, 6, 0, COLR_IDX_ORANGE, "%s", "ALIGN IRS>");
        }
        yfs_printf_rgt(yfms, 2, 4, COLR_IDX_BLUE, "%s", "/");
        yfs_printf_rgt(yfms, 8, 0, COLR_IDX_BLUE, "%.8s", gps_coodinates_buf + 8);
    }

    /* all good */
    return;
}

static void yfs_lsk_callback_init(yfms_context *yfms, int key[2], intptr_t refcon)
{
    if (key[0] == 1 && key[1] == 0) // from/to
    {
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if  (strnlen(buf, 1))
        {
            ndt_airport *dst = NULL, *src = NULL;
            char *suffix = buf, *prefix = strsep(&suffix, "/");
            if (suffix && suffix[0] && (dst = ndt_navdata_get_airport(yfms->ndt.ndb, suffix)))
            {
                yfms->data.init.to = dst;
            }
            if (prefix && prefix[0] && (src = ndt_navdata_get_airport(yfms->ndt.ndb, prefix)))
            {
                yfms->data.init.from = src;
            }
            if (dst || src)
            {
                // departure or arrival changed, reset flight plans, leg lists
                if (yfms->ndt.flp.arr)
                {
                    ndt_flightplan_close(&yfms->ndt.flp.arr);
                }
                if (yfms->ndt.flp.dep)
                {
                    ndt_flightplan_close(&yfms->ndt.flp.dep);
                }
                if (yfms->ndt.flp.iac)
                {
                    ndt_flightplan_close(&yfms->ndt.flp.iac);
                }
                if (yfms->ndt.flp.rte)
                {
                    ndt_flightplan_close(&yfms->ndt.flp.rte);
                }
                if (ndt_list_count(yfms->data.fpln.legs))
                {
                    ndt_list_empty(yfms->data.fpln.legs);
                }
                if (yfms->data.init.from && yfms->data.init.to)
                {
                    // we have both airports, initialize flight plans
                    if ((yfms->data.fpln.legs == NULL) &&
                        (yfms->data.fpln.legs = ndt_list_init()) == NULL)
                    {
                        yfms->data.init.ialized = 0; yfms->data.init.from = yfms->data.init.to = NULL;
                        yfs_spad_reset(yfms, "MEMORY ERROR 1", COLR_IDX_ORANGE); yfs_init_pageupdt(yfms); return;
                    }
                    if ((yfms->ndt.flp.arr = ndt_flightplan_init(yfms->ndt.ndb)) == NULL ||
                        (yfms->ndt.flp.dep = ndt_flightplan_init(yfms->ndt.ndb)) == NULL ||
                        (yfms->ndt.flp.iac = ndt_flightplan_init(yfms->ndt.ndb)) == NULL ||
                        (yfms->ndt.flp.rte = ndt_flightplan_init(yfms->ndt.ndb)) == NULL)
                    {
                        yfms->data.init.ialized = 0; yfms->data.init.from = yfms->data.init.to = NULL;
                        yfs_spad_reset(yfms, "MEMORY ERROR 2", COLR_IDX_ORANGE); yfs_init_pageupdt(yfms); return;
                    }
                    if (ndt_flightplan_set_departure(yfms->ndt.flp.arr, yfms->data.init.from->info.idnt, NULL) ||
                        ndt_flightplan_set_departure(yfms->ndt.flp.dep, yfms->data.init.from->info.idnt, NULL) ||
                        ndt_flightplan_set_departure(yfms->ndt.flp.iac, yfms->data.init.from->info.idnt, NULL) ||
                        ndt_flightplan_set_departure(yfms->ndt.flp.rte, yfms->data.init.from->info.idnt, NULL))
                    {
                        if (src)
                        {
                            yfms->data.init.ialized = 0; yfms->data.init.from = NULL;
                            yfs_spad_reset(yfms, "UNKNOWN ERROR 1", COLR_IDX_ORANGE); yfs_init_pageupdt(yfms); return;
                        }
                    }
                    if (ndt_flightplan_set_arrival(yfms->ndt.flp.arr, yfms->data.init.to->info.idnt, NULL) ||
                        ndt_flightplan_set_arrival(yfms->ndt.flp.dep, yfms->data.init.to->info.idnt, NULL) ||
                        ndt_flightplan_set_arrival(yfms->ndt.flp.iac, yfms->data.init.to->info.idnt, NULL) ||
                        ndt_flightplan_set_arrival(yfms->ndt.flp.rte, yfms->data.init.to->info.idnt, NULL))
                    {
                        if (dst)
                        {
                            yfms->data.init.ialized = 0; yfms->data.init.to = NULL;
                            yfs_spad_reset(yfms, "UNKNOWN ERROR 2", COLR_IDX_ORANGE); yfs_init_pageupdt(yfms); return;
                        }
                    }
                    if (ndt_distance_get(yfms->data.init.from->tr_altitude, NDT_ALTUNIT_FT))
                    {
                        yfms->data.init.trans_a = yfms->data.init.from->tr_altitude;
                    }
                    else
                    {
                        yfms->data.init.trans_a = ndt_distance_init (10000, NDT_ALTUNIT_FT);
                    }
                    if (ndt_distance_get  (yfms->data.init.to->trans_level, NDT_ALTUNIT_FT))
                    {
                        yfms->data.init.trans_l = yfms->data.init.from->trans_level;
                    }
                    else
                    {
                        yfms->data.init.trans_l = ndt_distance_init (10000, NDT_ALTUNIT_FT);
                    }
                    yfms->data.init.ialized = 1;
                    yfms->data.fpln.lg_idx = 0;
                }
                yfs_spad_clear(yfms); yfs_init_pageupdt(yfms); return;
            }
            yfs_spad_reset(yfms, "NOT IN DATA BASE", -1); yfs_init_pageupdt(yfms); return;
        }
        yfs_spad_reset(yfms, "FORMAT ERROR", -1); yfs_init_pageupdt(yfms); return;
    }
    if (key[0] == 0 && key[1] == 2) // flight number
    {
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if  (strnlen(buf, 1))
        {
            if (!strcmp(buf, "CLR"))
            {
                yfms->data.init.flight_id[0] = '\0';
            }
            else
            {
                snprintf(yfms->data.init.flight_id, sizeof(yfms->data.init.flight_id), "%s", buf);
            }
            yfs_spad_clear(yfms); yfs_init_pageupdt(yfms); return;
        }
        yfs_spad_reset(yfms, "FORMAT ERROR", -1); yfs_init_pageupdt(yfms); return;
    }
    if (key[0] == 1 && key[1] == 2) // align IRS
    {
        if (yfms->data.init.ialized == 1 && yfms->data.init.aligned == 0)
        {
            yfms->data.init.aligned = 1;
        }
        yfs_init_pageupdt(yfms); return;
    }
    if (key[0] == 0 && key[1] == 4) // cost index
    {
        int ci; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (strnlen(buf, 1) && sscanf(buf, "%d", &ci) == 1 && ci >= 0 && ci <= 999)
        {
            yfms->data.init.cost_index = ci; yfs_spad_clear(yfms); yfs_init_pageupdt(yfms); return;
        }
        yfs_spad_reset(yfms, "ENTRY OUT OF RANGE", -1); yfs_init_pageupdt(yfms); return;
    }
    if (key[0] == 0 && key[1] == 5) // initial cruise altitude
    {
        int crz_alt; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (strnlen(buf, 1) && sscanf(buf, "%d", &crz_alt) == 1)
        {
            // Lockheed SR-71 Blackbird can go up to FL850 ;-)
            if (crz_alt >= (1600 - 50) && crz_alt <= 85000)
            {
                crz_alt = ((crz_alt + 50) / 100); // round to nearest flight level
                yfms->data.init.crz_alt = ndt_distance_init(crz_alt, NDT_ALTUNIT_FL);
                yfs_spad_clear(yfms); yfs_init_pageupdt(yfms); return;
            }
            if (crz_alt >= 16 && crz_alt <= 850)
            {
                yfms->data.init.crz_alt = ndt_distance_init(crz_alt, NDT_ALTUNIT_FL);
                yfs_spad_clear(yfms); yfs_init_pageupdt(yfms); return;
            }
        }
        yfs_spad_reset(yfms, "ENTRY OUT OF RANGE", -1); yfs_init_pageupdt(yfms); return;
    }
    /* all good */
    return;
}
