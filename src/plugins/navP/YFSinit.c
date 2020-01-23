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

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "XPLM/XPLMUtilities.h"

#include "common/common.h"
#include "compat/compat.h"
#include "lib/flightplan.h"
#include "lib/navdata.h"

#include "YFSfpln.h"
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
    if (yfms->xpl.has_custom_navigation == 0)
    {
        yfms->lsks[0][0].cback = yfms->lsks[1][0].cback =
        yfms->lsks[0][2].cback = yfms->lsks[1][2].cback =
        yfms->lsks[0][4].cback = yfms->lsks[1][4].cback =
        yfms->lsks[0][5].cback = yfms->lsks[1][5].cback = (YFS_LSK_f)&yfs_lsk_callback_init;
    }
    return yfs_init_pageupdt(yfms);
}

void yfs_init_pageupdt(yfms_context *yfms)
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
    yfs_printf_ctr(yfms, 0, COLR_IDX_WHITE, "%s", "INIT");

    /* aircraft GPS position */
    char gps_coodinates_buf[17];
    if (yfms->data.init.aligned)
    {
        // TODO: animate transition between reference and actual position
        ndt_position_sprintllc(yfms->data.aircraft_pos, NDT_LLCFMT_AIBUS,
                               gps_coodinates_buf, sizeof(gps_coodinates_buf));
    }
    else if (yfms->data.init.ialized)
    {
        ndt_position_sprintllc(yfms->data.init.from->coordinates, NDT_LLCFMT_AIBUS,
                               gps_coodinates_buf, sizeof(gps_coodinates_buf));
    }

    /* left column */
    if (yfms->data.init.ialized)
    {
        if (yfms->data.init.corte_name[0])
        {
            yfs_printf_lft(yfms, 2, 0, COLR_IDX_BLUE, "%.*s",
                           sizeof(yfms->data.init.corte_name),
                           yfms->data.init.corte_name);
        }
        else
        {
            yfs_printf_lft(yfms, 2, 0, COLR_IDX_BLUE, "%s", "NONE");
        }
        yfs_printf_lft(yfms, 4, 0, COLR_IDX_BLUE, "%s", "NONE");
    }
    else
    {
        yfs_printf_lft(yfms, 2, 0, COLR_IDX_ORANGE, "%s", "##########");
        yfs_printf_lft(yfms, 4, 0, COLR_IDX_WHITE,  "%s", "----/----------");
    }
    yfs_printf_lft(yfms,  1, 0, COLR_IDX_WHITE,  "%s", " CO RTE");
    yfs_printf_lft(yfms,  3, 0, COLR_IDX_WHITE,  "%s", "ALTN/CO RTE");
    yfs_printf_lft(yfms,  5, 0, COLR_IDX_WHITE,  "%s", "FLT NBR");
    yfs_printf_lft(yfms,  6, 0, COLR_IDX_ORANGE, "%s", "########");
    yfs_printf_lft(yfms,  7, 0, COLR_IDX_WHITE,  "%s", "LAT");
    yfs_printf_lft(yfms,  8, 0, COLR_IDX_WHITE,  "%s", "----.--");
    yfs_printf_lft(yfms,  9, 0, COLR_IDX_WHITE,  "%s", "COST INDEX");
    yfs_printf_lft(yfms, 10, 0, COLR_IDX_WHITE,  "%s", "---");
    yfs_printf_lft(yfms, 11, 0, COLR_IDX_WHITE,  "%s", "CRZ FL"/*"/TEMP"*/);
    yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE,  "%s", "-----"/*" /---"*/);
    if (yfms->data.init.flight_id[0])
    {
        yfs_printf_lft(yfms, 6, 0, COLR_IDX_BLUE, "%-8s", yfms->data.init.flight_id);
    }
    if (yfms->data.init.ialized)
    {
        if (ndt_distance_get(yfms->data.init.crz_alt, NDT_ALTUNIT_NA))
        {
            yfs_printf_lft(yfms, 12, 0, COLR_IDX_BLUE, "FL%03d", (int)ndt_distance_get(yfms->data.init.crz_alt, NDT_ALTUNIT_FL));
        }
        else
        {
            yfs_printf_lft(yfms, 12, 0, COLR_IDX_ORANGE, "%s", "#####"/*" /###"*/);
        }
    }
    if (yfms->data.init.ialized || yfms->data.init.aligned)
    {
        yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE, "%.7s", gps_coodinates_buf);
    }
    if (yfms->data.init.ialized)
    {
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
    yfs_printf_rgt(yfms, 12, 0, COLR_IDX_BLUE,   "%s",     "36090"); // TODO: implement tropopause setting and temperature at cruise altitude
    if (yfms->data.init.from)
    {
        yfs_printf_rgt(yfms, 2, 5, COLR_IDX_BLUE, "%+4s", yfms->data.init.from->info.idnt);
    }
    if (yfms->data.init.to)
    {
        yfs_printf_rgt(yfms, 2, 0, COLR_IDX_BLUE, "%-4s", yfms->data.init.to->info.idnt);
    }
    if (yfms->data.init.ialized || yfms->data.init.aligned)
    {
        yfs_printf_rgt(yfms, 8, 0, COLR_IDX_BLUE, "%.8s", gps_coodinates_buf + 8);
    }
    if (yfms->data.init.ialized)
    {
        if (yfms->data.init.aligned == 0)
        {
            yfs_printf_rgt(yfms, 6, 0, COLR_IDX_ORANGE, "%s", "ALIGN IRS>");
        }
        yfs_printf_rgt(yfms, 2, 4, COLR_IDX_BLUE, "%s", "/");
    }

    /* all good */
    yfms->mwindow.screen.redraw = 1; return;
}

static int check_subdirectories_recursive(yfms_context *yfms, const char *base, char outPath[512])
{
    char basePath[512]; strncpy(basePath, outPath, sizeof(basePath));
    const char *xlist[] = { ".fms", ".txt", ".fpl", }; // supported extensions by priority
    struct stat stats; char outBuffer[512*512]; char *outNames[512]; int outReturnedCount;
    /* log where we're looking for and what we're looking for */
    ndt_log("YFMS [info]: searching directory \"%s\" for company route \"%s\"\n", basePath, base);
    /* check basePath for coroute files first */
    for (int i = 0; i < sizeof(xlist) / sizeof(*xlist); i++)
    {
        if (snprintf(outPath, 512, "%s/%s%s", basePath, base, xlist[i]) < 512)
        {
            if (!stat(outPath, &stats) && S_ISREG(stats.st_mode))
            {
                ndt_log("YFMS [info]: found company route \"%s\"\n", outPath);
                return i ? /* ICAO */ 2 : /* X-Plane FMS */ 1;
            }
        }
    }
    /* list all files and directories within basePath */
    XPLMGetDirectoryContents(basePath, 0, outBuffer, sizeof(outBuffer), outNames, 512, NULL, &outReturnedCount);
    /* then start looking for files with supported extensions */
    for (int i = 0; i < outReturnedCount; i++)
    {
        if (snprintf(outPath, 512, "%s/%s", basePath, outNames[i]) < 512)
        {
            if (!stat(outPath, &stats) && S_ISDIR(stats.st_mode)) // subdirectory, check it
            {
                int ret = check_subdirectories_recursive(yfms, base, outPath);
                if (ret) return ret;
            }
        }
    }
    /* no company route found */
    return 0;
}

static int get_coroute_file(yfms_context *yfms, const char *base, char outPath[512])
{
    struct stat stats;
    if (snprintf(outPath, 512, "%s%s", yfms->ndt.xsystem_pth, "Output/FMS plans") >= 512)
    {
        return -1;
    }
    if (stat(outPath, &stats))
    {
        return -2;
    }
    if (!S_ISDIR(stats.st_mode))
    {
        return -3;
    }
    /* check any subdirectories recursively */
    return check_subdirectories_recursive(yfms, base, outPath);
}

static ndt_flightplan* file_to_flightplan(yfms_context *yfms, char path[512], ndt_fltplanformat fmt)
{
    int ret; char *contents = ndt_file_slurp(path, &ret); ndt_flightplan *corte; ndt_route_leg *leg;
    if (ret)
    {
        ndt_log("YFMS [error]: \"%s\" %d, ndt_file_slurp %d\n", path, fmt, ret); return NULL;
    }
    if ((corte = ndt_flightplan_init(yfms->ndt.ndb)) == NULL)
    {
        ndt_log("YFMS [error]: \"%s\" %d, ndt_flightplan_init\n",    path, fmt); return NULL;
    }
    switch (fmt)
    {
        case NDT_FLTPFMT_XPFMS:
        case NDT_FLTPFMT_ICAOR:
            if ((ret = ndt_flightplan_set_route(corte, contents, fmt)))
            {
                ndt_log("YFMS [error]: \"%s\" %d, route %d\n",  path, fmt, ret); return NULL;
                return NULL;
            }
            break;
        default:
            ndt_log("YFMS [error]: \"%s\" %d, unsupported format\n", path, fmt); return NULL;
    }
    for (int i = 0; i < ndt_list_count(corte->legs); i++)
    {
        if ((leg = ndt_list_item(corte->legs, i)))
        {
            if (leg->dst)
            {
                // make sure to only include supported waypoint types on import
                switch (leg->dst->type)
                {
                    case NDT_WPTYPE_APT:
                    case NDT_WPTYPE_DME:
                    case NDT_WPTYPE_FIX:
                    case NDT_WPTYPE_NDB:
                    case NDT_WPTYPE_VOR:
                    case NDT_WPTYPE_RWY:
                        break;
                    default: // otherwise, simply sanitize type and identifier
                        ndt_position_sprintllc(leg->dst->position, NDT_LLCFMT_DEFLT, leg->dst->info.idnt, sizeof(leg->dst->info.idnt));
                        leg->dst->type = NDT_WPTYPE_LLC;
                        break;
                }
            }
        }
    }
    return corte;
}

static void yfs_flightplan_reinit(yfms_context *yfms, ndt_airport *src, ndt_airport *dst, ndt_flightplan *corte)
{
    // departure or arrival changed, reset flight plans, leg lists
    if (yfms->data.init.ialized)
    {
        ndt_log("YFMS [debug]: phase change: yfs_flightplan_reinit: FMGS_PHASE_END\n");
        yfms->data.phase = FMGS_PHASE_END;
        yfms->data.init.ialized = 0;
    }
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
    if (corte)
    {
        yfms->data.init.from = src = corte->dep.apt;
        yfms->data.init.to   = dst = corte->arr.apt;
        // TODO: remove this when YFMS can add/remove/switch procedures directly
        if (corte->arr.rwy)
        {
            corte->arr.rwy = NULL;
        }
        if (corte->dep.rwy)
        {
            if (corte->dep.sid.proc)
            {
                yfs_spad_reset(yfms, "NOT IMPLEMENTED", -1);
                return yfs_flightplan_reinit(yfms, NULL, NULL, NULL);
            }
            ndt_route_segment *rsg = ndt_route_segment_direct(corte->dep.apt->waypoint, corte->dep.rwy->waypoint);
            if (rsg == NULL)
            {
                yfs_spad_reset(yfms, "MEMORY ERROR 0", COLR_IDX_ORANGE);
                return yfs_flightplan_reinit(yfms, NULL, NULL, NULL);
            }
            ndt_waypoint *rwy = corte->dep.rwy->waypoint; corte->dep.rwy = NULL;
            ndt_route_leg *leg = ndt_flightplan_insert_direct(corte, rwy, ndt_list_item(corte->legs, 0), 0);
        }
    }
    else
    {
        yfms->data.init.from = src;
        yfms->data.init.to   = dst;
    }
    if (yfms->data.init.from && yfms->data.init.to)
    {
        // we have both airports, initialize flight plans
        for (int i = 0; i < 20; i++)
        {
            if (yfms->data.fpln.usrwpts[i].wpt)
            {
                ndt_navdata_rem_waypoint(yfms->ndt.ndb, yfms->data.fpln.usrwpts[i].wpt);
                ndt_waypoint_close(&yfms->data.fpln.usrwpts[i].wpt);
            }
            yfms->data.fpln.usrwpts[i].wpt = NULL;
            yfms->data.fpln.usrwpts[i].ref = 0;
        }
        if (yfms->data.fpln.w_tp)
        {
            ndt_waypoint_close(&yfms->data.fpln.w_tp);
        }
        if ((yfms->data.fpln.legs == NULL) &&
            (yfms->data.fpln.legs = ndt_list_init()) == NULL)
        {
            yfms->data.fpln.awys.open = 0;
            yfms->data.fpln.lrev.open = 0;
            yfms->data.init.from = yfms->data.init.to = NULL;
            yfs_spad_reset(yfms, "MEMORY ERROR 1", COLR_IDX_ORANGE); return yfs_init_pageupdt(yfms);
        }
        if ((yfms->ndt.flp.arr = ndt_flightplan_init(yfms->ndt.ndb)) == NULL ||
            (yfms->ndt.flp.dep = ndt_flightplan_init(yfms->ndt.ndb)) == NULL ||
            (yfms->ndt.flp.iac = ndt_flightplan_init(yfms->ndt.ndb)) == NULL ||
            (yfms->ndt.flp.rte = ndt_flightplan_init(yfms->ndt.ndb)) == NULL)
        {
            yfms->data.fpln.awys.open = 0;
            yfms->data.fpln.lrev.open = 0;
            yfms->data.init.from = yfms->data.init.to = NULL;
            yfs_spad_reset(yfms, "MEMORY ERROR 2", COLR_IDX_ORANGE); return yfs_init_pageupdt(yfms);
        }
        if (ndt_flightplan_set_departure(yfms->ndt.flp.arr, yfms->data.init.from->info.idnt, NULL) ||
            ndt_flightplan_set_departure(yfms->ndt.flp.dep, yfms->data.init.from->info.idnt, NULL) ||
            ndt_flightplan_set_departure(yfms->ndt.flp.iac, yfms->data.init.from->info.idnt, NULL) ||
            ndt_flightplan_set_departure(yfms->ndt.flp.rte, yfms->data.init.from->info.idnt, NULL))
        {
            if (src)
            {
                yfms->data.fpln.awys.open = 0;
                yfms->data.fpln.lrev.open = 0;
                yfms->data.init.from      = NULL;
                yfs_spad_reset(yfms, "UNKNOWN ERROR 1", COLR_IDX_ORANGE); return yfs_init_pageupdt(yfms);
            }
        }
        if (ndt_flightplan_set_arrival(yfms->ndt.flp.arr, yfms->data.init.to->info.idnt, NULL) ||
            ndt_flightplan_set_arrival(yfms->ndt.flp.dep, yfms->data.init.to->info.idnt, NULL) ||
            ndt_flightplan_set_arrival(yfms->ndt.flp.iac, yfms->data.init.to->info.idnt, NULL) ||
            ndt_flightplan_set_arrival(yfms->ndt.flp.rte, yfms->data.init.to->info.idnt, NULL))
        {
            if (dst)
            {
                yfms->data.fpln.awys.open = 0;
                yfms->data.fpln.lrev.open = 0;
                yfms->data.init.to        = NULL;
                yfs_spad_reset(yfms, "UNKNOWN ERROR 2", COLR_IDX_ORANGE); return yfs_init_pageupdt(yfms);
            }
        }
        if (corte)
        {
            ndt_flightplan_close(&yfms->ndt.flp.rte); yfms->ndt.flp.rte = corte;
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
        if (corte == NULL)
        {
            yfms->data.init.corte_name[0] = 0;
        }
        if (yfms->xpl.atyp == YFS_ATYP_Q380)
        {
            XPLMSetDataf(yfms->xpl.q380.PeterCI, ((((float)yfms->data.init.cost_index) * 200.0f) / 650.0f));
            XPLMSetDataf(yfms->xpl.q380.PeterFLX, 50.0f); // XXX stopgap measure
            XPLMSetDataf(yfms->xpl.q380.PeterV1, 130.0f); // XXX stopgap measure
            XPLMSetDataf(yfms->xpl.q380.PeterVR, 140.0f); // XXX stopgap measure
            XPLMSetDataf(yfms->xpl.q380.PeterV2, 150.0f); // XXX stopgap measure
        }
        if (yfms->data.init.aligned)
        {
            // auto-select GPS (FMS) source when aligned
            XPLMCommandOnce(yfms->xpl.gps_select_captain);
            XPLMCommandOnce(yfms->xpl.gps_select_copilot);
        }
        XPLMDataRef swtid = XPLMFindDataRef("sim/weapons/target_index");
        XPLMDataRef sendv = XPLMFindDataRef("sim/custom/xap/sendv");
        if (sendv && swtid) // XXX: Falcon 7X by after
        {
            int index = 0;
            XPLMSetDatai(sendv, 1);
            XPLMSetDatavi(swtid, &index, 2, 1);
        }
        yfms->data.init.ialized         = 1;
        yfms->data.fpln.lg_idx          = 0;
        yfms->data.fpln.awys.open       = 0;
        yfms->data.fpln.lrev.open       = 0;
        yfms->data.init.flight_id[0]    = 0;
        yfms->data.phase                = FMGS_PHASE_PRE;
        yfms->data.fpln.dist.remain     = ndt_distance_init(0, NDT_ALTUNIT_NA);
        yfms->data.fpln.dist.ref_leg_id = -1; // XXX: force a full distance re-sync
        yfms->data.fpln.xplm_last       = 99; // XXX: force a full flight plan sync
        yfms->data.fpln.mod.operation   = YFS_FPLN_MOD_INIT; yfs_fpln_fplnupdt(yfms);
        ndt_log("YFMS [debug]: phase change: yfs_flightplan_reinit: FMGS_PHASE_PRE\n");
    }
    if (yfms->data.init.from == NULL && yfms->data.init.to == NULL) // reset all
    {
        ndt_log("YFMS [debug]: phase change: yfs_flightplan_reinit: FMGS_PHASE_END\n");
        yfms->data.init.crz_alt       = ndt_distance_init(0, NDT_ALTUNIT_NA);
        yfms->data.phase              = FMGS_PHASE_END;
        yfms->data.init.corte_name[0] = 0;
//      yfms->data.init.flight_id[0]  = 0; // don't reset required/broadcast by XPDR (mode S, ADS-B out)
        yfms->data.init.cost_index    = 0;
        yfms->data.fpln.awys.open     = 0;
        yfms->data.fpln.lrev.open     = 0;
        /*
         * this is not a full FMGS reset: we can clear the whole XPLMNavigation
         * plan too; we don't do this for full resets because this allows us to
         * set up routes for the GPS units but release navigation to said units
         * which override the XPLMNavigation active leg (unlike FMS aircraft)…
         */
        for (int i = XPLMCountFMSEntries() - 1; i >= 0; i--)
        {
            XPLMClearFMSEntry(i);
        }
    }
    /* all good */
    return;
}

void yfs_init_fplreset(yfms_context *yfms)
{
    yfs_flightplan_reinit(yfms, NULL, NULL, NULL); return;
}

static ndt_airport* xplm_create_airport(yfms_context *yfms, const char *code)
{
    if (yfms && code && *code)
    {
        size_t code_len = (size_t)strlen(code);
        float inLatitud = (float)ndt_position_getlatitude (yfms->data.aircraft_pos, NDT_ANGUNIT_DEG);
        float inLongitu = (float)ndt_position_getlongitude(yfms->data.aircraft_pos, NDT_ANGUNIT_DEG);
        XPLMNavRef xpap = XPLMFindNavAid(NULL, code, &inLatitud, &inLongitu, NULL, xplm_Nav_Airport);
        if (XPLM_NAV_NOT_FOUND != xpap)
        {
            char outID[33]; char outName[257]; float outLat[01]; float outLon[01]; float outHeight[01];
            XPLMGetNavAidInfo(xpap, NULL, outLat, outLon, outHeight, NULL, NULL, outID, outName, NULL);
            if (strnlen(outID, 1 + code_len) != code_len)
            {
                return NULL;
            }
            // our database should be uppercase-only
            for (size_t i = 0; outID[i] != '\0'; i++)
            {
                outID[i] = toupper(outID[i]);
            }
            for (size_t i = 0; outName[i] != '\0'; i++)
            {
                outName[i] = toupper(outName[i]);
            }
            int airfieldelevation = round((double)*outHeight / 0.3048);
            ndt_distance altitude = ndt_distance_init(airfieldelevation, NDT_ALTUNIT_FT);
            ndt_position position = ndt_position_init((double)*outLat, (double)*outLon, altitude);
            int ret = ndt_navdata_user_airport(yfms->ndt.ndb, outID, outName, position);
            if (ret)
            {
                ndt_log("YFMS [debug] ndt_navdata_user_airport failed (%d)\n", ret);
                return NULL;
            }
            return ndt_navdata_get_airport(yfms->ndt.ndb, outID);
        }
    }
    return NULL;
}

static void yfs_lsk_callback_init(yfms_context *yfms, int key[2], intptr_t refcon)
{
    if (key[0] == 0 && key[1] == 0) // company route
    {
        char outPath[512]; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf); ndt_flightplan *corte;
        if  (strnlen(buf, 1))
        {
            if (strnlen(buf, sizeof(yfms->data.init.corte_name)) == sizeof(yfms->data.init.corte_name))
            {
                yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
            }
            switch (get_coroute_file(yfms, buf, outPath))
            {
                case 1:
                    corte = file_to_flightplan(yfms, outPath, NDT_FLTPFMT_XPFMS);
                    break;
                case 2:
                    corte = file_to_flightplan(yfms, outPath, NDT_FLTPFMT_ICAOR);
                    break;
                case 0:
                    yfs_spad_reset(yfms, "NOT IN DATA BASE", -1); return;
                case -1:
                    yfs_spad_reset(yfms, "FILE PATH ERROR 1", COLR_IDX_ORANGE); return;
                case -2:
                    yfs_spad_reset(yfms, "FILE PATH ERROR 2", COLR_IDX_ORANGE); return;
                case -3:
                    yfs_spad_reset(yfms, "FILE PATH ERROR 3", COLR_IDX_ORANGE); return;
                default:
                    yfs_spad_reset(yfms, "UNKNOWN PATH ERR.", COLR_IDX_ORANGE); return;
            }
            if (corte == NULL)
            {
                yfs_spad_reset(yfms, "CO RTE LOAD FAIL", -1); return;
            }
            strncpy(yfms->data.init.corte_name, buf, sizeof(yfms->data.init.corte_name));
            yfs_spad_clear(yfms); yfs_flightplan_reinit(yfms, NULL, NULL, corte); return yfs_init_pageupdt(yfms);
        }
    }
    if (key[0] == 1 && key[1] == 0) // from/to
    {
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if  (strnlen(buf, 1))
        {
            ndt_airport *dst = NULL, *src = NULL; char icao[5];
            char *suffix = buf, *prefix = strsep(&suffix, "/");
            if  (!strcmp  (buf, "CLR"))
            {
                yfs_flightplan_reinit(yfms, NULL, NULL, NULL);
                yfs_spad_clear(yfms); return yfs_init_pageupdt(yfms);
            }
            if (prefix && prefix[0])
            {
                if (strnlen(prefix, 4) == 3)
                {
                    snprintf(icao, sizeof(icao), "K%s", prefix); // accept FAA LIDs but convert them
                }
                else
                {
                    snprintf(icao, sizeof(icao),  "%s", prefix);
                }
                if ((src = ndt_navdata_get_airport(yfms->ndt.ndb,   icao)) == NULL &&
                    (src = ndt_navdata_get_airport(yfms->ndt.ndb, prefix)) == NULL)
                {
                    (src = xplm_create_airport(yfms, prefix));
                }
            }
            else
            {
                prefix = NULL;
            }
            if (suffix && suffix[0])
            {
                if (strnlen(suffix, 4) == 3)
                {
                    snprintf(icao, sizeof(icao), "K%s", suffix); // accept FAA LIDs but convert them
                }
                else
                {
                    snprintf(icao, sizeof(icao),  "%s", suffix);
                }
                if ((dst = ndt_navdata_get_airport(yfms->ndt.ndb,   icao)) == NULL &&
                    (dst = ndt_navdata_get_airport(yfms->ndt.ndb, suffix)) == NULL)
                {
                    (dst = xplm_create_airport(yfms, suffix));
                }
            }
            else
            {
                suffix = NULL;
            }
            if ((!prefix || src) && (!suffix || dst))
            {
                yfs_spad_clear(yfms); yfs_flightplan_reinit(yfms, src, dst, NULL); return yfs_init_pageupdt(yfms);
            }
            yfs_spad_reset(yfms, "NOT IN DATA BASE", -1); return yfs_init_pageupdt(yfms);
        }
        yfs_spad_reset(yfms, "FORMAT ERROR", -1); return yfs_init_pageupdt(yfms);
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
        yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
    }
    if (key[0] == 1 && key[1] == 2) // align IRS
    {
        if (yfms->data.init.ialized == 1 && yfms->data.init.aligned == 0)
        {
            // auto-select GPS (FMS) source when aligned
            XPLMCommandOnce(yfms->xpl.gps_select_captain);
            XPLMCommandOnce(yfms->xpl.gps_select_copilot);
            yfms->data.init.aligned = 1;
        }
        yfs_init_pageupdt(yfms); return;
    }
    if (key[0] == 0 && key[1] == 4) // cost index
    {
        if (yfms->data.init.ialized)
        {
            int ci; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
            if (strnlen(buf, 1) && sscanf(buf, "%d", &ci) == 1 && ci >= 0 && ci <= 999)
            {
                if (yfms->xpl.atyp == YFS_ATYP_Q380)
                {
                    if (ci < 0 || ci > 999)
                    {
                        yfs_spad_reset(yfms, "ENTRY OUT OF RANGE", -1); return;
                    }
                    XPLMSetDataf(yfms->xpl.q380.PeterCI, ((((float)ci) * 200.0f) / 650.0f));
                }
                yfms->data.init.cost_index = ci; yfs_spad_clear(yfms); yfs_init_pageupdt(yfms); return;
            }
            yfs_spad_reset(yfms, "ENTRY OUT OF RANGE", -1); return;
        }
        return;
    }
    if (key[0] == 0 && key[1] == 5) // initial cruise altitude
    {
        if (yfms->data.init.ialized)
        {
            int crz_alt; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
            if (strnlen(buf, 1))
            {
                if (strcmp(buf, "CLR") == 0)
                {
                    if (yfms->xpl.atyp == YFS_ATYP_Q380)
                    {
                        XPLMSetDatai(yfms->xpl.q380.PeterCRZ, 0);
                    }
                    yfms->data.init.crz_alt = ndt_distance_init(0, NDT_ALTUNIT_NA);
                    yfs_spad_clear(yfms); return yfs_init_pageupdt(yfms);
                }
                if (sscanf(buf, "FL%d", &crz_alt) == 1 ||
                    sscanf(buf,   "%d", &crz_alt) == 1)
                {
                    // :-) SR-71 Blackbird can do FL850
                    if (crz_alt >= 25 && crz_alt <= 850)
                    {
                        if (yfms->xpl.atyp == YFS_ATYP_Q380)
                        {
                            if (crz_alt < 100 || crz_alt > 430)
                            {
                                yfs_spad_reset(yfms, "ENTRY OUT OF RANGE", -1); return;
                            }
                            XPLMSetDatai(yfms->xpl.q380.PeterCRZ, crz_alt);
                        }
                        yfms->data.init.crz_alt = ndt_distance_init(crz_alt, NDT_ALTUNIT_FL);
                        yfs_spad_clear(yfms); return yfs_init_pageupdt(yfms);
                    }
                    yfs_spad_reset(yfms, "ENTRY OUT OF RANGE", -1); return;
                }
                yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
            }
        }
        return;
    }
    if (key[0] == 1 && key[1] == 5) // tropopause
    {
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if  (strnlen(buf, 1))
        {
            yfs_spad_reset(yfms, "NOT IMPLEMENTED", -1); return;
        }
        yfs_spad_reset(yfms, "36090", -1); yfs_init_pageupdt(yfms); return;
    }
    /* all good */
    return;
}
