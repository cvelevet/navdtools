/*
 * YFSmenu.c
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
#include <stdint.h>
#include <string.h>

#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMProcessing.h"

#include "assert/includes.h"

#include "lib/flightplan.h"

#include "YFSdata.h"
#include "YFSdrto.h"
#include "YFSfuel.h"
#include "YFSfpln.h"
#include "YFSinit.h"
#include "YFSmain.h"
#include "YFSmenu.h"
#include "YFSperf.h"
#include "YFSprog.h"
#include "YFSrdio.h"
#include "YFSspad.h"

#define DEFAULT_CALLBACK_RATE (0.25f) /* every 1/4 second should (almost) not be perceivable by users */

static void  yfs_key_callback_menu(void *yfms);
static void  yfs_lsk_callback_menu(void *yfms, int key[2], intptr_t refcon);
static float yfs_flight_loop_cback(float, float, int, void*);

void yfs_menu_resetall(yfms_context *yfms)
{
    if (!yfms)
    {
        return; // no error
    }

    /* scratchpad */
    for (int i = 0; i < YFS_DISPLAY_NUMR; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }
    char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
    if  (strnlen(buf, 1))
    {
        yfs_spad_clear(yfms);
    }
    yfs_spad_reset(yfms, "YFMS INITIALIZED", COLR_IDX_ORANGE);

    /* flight loop callback to monitor dataref changes and update pages as required */
    if (yfms->xpl.fl_callback == NULL)
    {
        XPLMRegisterFlightLoopCallback(yfms->xpl.fl_callback = &yfs_flight_loop_cback, -1, yfms);
    }

    /* check whether plugins we potentially interact with are loaded */
    yfms->xpl.plugin_id_pe = XPLMFindPluginBySignature("com.pilotedge.plugin.xplane");
    yfms->xpl.plugin_id_xb = XPLMFindPluginBySignature("vatsim.protodev.clients.xsquawkbox");
    if (XPLM_NO_PLUGIN_ID != yfms->xpl.plugin_id_xb)
    {
        yfms->xpl.xsbcomm[0] = XPLMFindCommand("xsquawkbox/voice/ptt");
        yfms->xpl.xsbcomm[1] = XPLMFindCommand("xsquawkbox/text/end");
        yfms->xpl.xsbcomm[2] = XPLMFindCommand("xsquawkbox/text/start");
        yfms->xpl.xsbcomm[3] = XPLMFindCommand("xsquawkbox/text/nextpage");
        yfms->xpl.xsbcomm[4] = XPLMFindCommand("xsquawkbox/text/prevpage");
        yfms->xpl.xsbcomm[5] = XPLMFindCommand("xsquawkbox/command/reply_next");
        yfms->xpl.xsbcomm[6] = XPLMFindCommand("xsquawkbox/command/start_text_entry");
        yfms->xpl.xsbcomm[7] = XPLMFindCommand("xsquawkbox/command/toggle_text_window");
        yfms->xpl.xsbcomm[8] = XPLMFindCommand("xsquawkbox/command/toggle_whos_online");
        for (int i = 000; i <= 8; i++)
        {
            if (NULL == yfms->xpl.xsbcomm[i])
            {
                ndt_log("YFMS [error]: XSquawkBox detected: XPLMFindCommand(xsbcomm[%d]) failed\n", i);
                yfms->xpl.plugin_id_xb = XPLM_NO_PLUGIN_ID;
                break;
            }
        }
    }

    /* always reset aircraft type (and associated automatic features) */
    yfms->xpl.has_custom_nav_radios = yfms->xpl.has_custom_navigation = 0;
    yfms->xpl.otto.vclb_vdes = yfms->xpl.otto.vmax_auto = 0;
    yfms->xpl.otto.vswitched = yfms->xpl.otto.vmax_flch = 0;
    yfms->xpl.atyp = YFS_ATYP_NSET;

    /* callbacks for page-specific keys */
    yfms->spcs.cback_menu = (YFS_SPC_f)&yfs_menu_pageopen;
    yfms->spcs.cback_atcc = (YFS_SPC_f)&yfs_rad1_pageopen;
    yfms->spcs.cback_radn = (YFS_SPC_f)&yfs_rad2_pageopen;
    yfms->spcs.cback_prog = (YFS_SPC_f)&yfs_prog_pageopen;
    yfms->spcs.cback_init = (YFS_SPC_f)&yfs_init_pageopen;
    yfms->spcs.cback_fpln = (YFS_SPC_f)&yfs_fpln_pageopen;
    yfms->spcs.cback_drto = (YFS_SPC_f)&yfs_drto_pageopen;
    yfms->spcs.cback_data = (YFS_SPC_f)&yfs_data_pageopen;
    yfms->spcs.cback_perf = (YFS_SPC_f)&yfs_perf_pageopen;
    yfms->spcs.cback_fuel = (YFS_SPC_f)&yfs_fuel_pageopen;

    /* navigation backend */
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
    if (yfms->data.fpln.w_tp)
    {
        ndt_waypoint_close(&yfms->data.fpln.w_tp);
    }
    if (yfms->data.prog.usrwpt)
    {
        ndt_waypoint_close(&yfms->data.prog.usrwpt);
    }
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

    /*
     * NOTE: do NOT use yfs_init_fplreset() function here, see explanation:
     * this is a full FMGS reset: we *can't* clear the whole XPLMNavigation
     * plan too; we don't do this for full resets because this allows us to
     * set up routes for the GPS units but release navigation to said units
     * which override the XPLMNavigation active leg (unlike FMS aircraft)…
     */
    yfms->data.init.crz_alt       = ndt_distance_init(0, NDT_ALTUNIT_NA);
    yfms->data.prog.fix           = NULL;
    yfms->data.init.to            = NULL;
    yfms->data.init.from          = NULL;
    yfms->data.fpln.l_ils         = NULL;
    yfms->data.fpln.l_rwy         = NULL;
    yfms->data.init.aligned       = 0;
    yfms->data.init.ialized       = 0;
    yfms->data.fpln.awys.open     = 0;
    yfms->data.fpln.lrev.open     = 0;
    yfms->data.init.cost_index    = 0;
    yfms->data.init.flight_id[0]  = 0;
    yfms->data.init.corte_name[0] = 0;

    /* delayed setters */
    yfms->data.rdio.asrt_delayed_baro_s = 0;
    yfms->data.rdio.asrt_delayed_redraw = 0;

    /* flight phase */
    ndt_log("YFMS [debug]: phase change: full FMGS reset\n");
    yfms->data.phase = FMGS_PHASE_END;

    /* may need to re-position the window on a per-aircraft basis */
    yfms->mwindow.win_state = 0;

    /* all good */
    if (XPIsWidgetVisible(yfms->mwindow.id))
    {
        yfs_main_toggl(yfms);
    }
    yfs_idnt_pageopen(yfms); return;
}

void yfs_menu_pageopen(yfms_context *yfms)
{
    if (!yfms)
    {
        return; // no error
    }
    yfs_main_newpg(yfms, PAGE_MENU); yfs_menu_pageupdt(yfms);
    yfms->lsks[0][0].cback = yfms->lsks[1][0].cback = &yfs_lsk_callback_menu; return;
}

void yfs_menu_pageupdt(yfms_context *yfms)
{
    if (!yfms || yfms->mwindow.current_page != PAGE_MENU)
    {
        return; // no error
    }

    /* reset lines before drawing */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }

    /* line 0: main header (white, centered) */
    yfs_printf_ctr(yfms, 0,    COLR_IDX_WHITE, "%s", "MCDU MENU");

    /* line 2 left: ident page (green) */
    yfs_printf_lft(yfms, 2, 0, COLR_IDX_GREEN, "%s", "<FMGC");

    /* line 2 right: reset (white) */
    yfs_printf_rgt(yfms, 2, 0, COLR_IDX_WHITE, "%s", "RESET>");

    /* all good */
    yfms->mwindow.screen.redraw = 1; return;
}

void yfs_idnt_pageopen(yfms_context *yfms)
{
    if (!yfms)
    {
        return; // no error
    }
    yfs_main_newpg(yfms, PAGE_IDNT); yfs_idnt_pageupdt(yfms); return;
}

void yfs_idnt_pageupdt(yfms_context *yfms)
{
    if (!yfms || yfms->mwindow.current_page != PAGE_IDNT)
    {
        return; // no error
    }

    /* reset lines before drawing */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }

    /* line 0: aircraft ICAO identifier (white, centered) */
    char icao[YFS_ROW_BUF_SIZE]; icao[XPLMGetDatab(yfms->xpl.acf_ICAO, icao, 0, sizeof(icao) - 1)] = 0;
    yfs_printf_ctr(yfms, 0, COLR_IDX_WHITE, "%s", icao);

    /* line 1: header (white, offset 1 right) */
    yfs_printf_lft(yfms, 1, 0, COLR_IDX_WHITE, "%s", " ENG");

    /* line 2: engine count & type (green) */
    int engc   = XPLMGetDatai (yfms->xpl.acf_num_engines);
    int engt[8]; XPLMGetDatavi(yfms->xpl.acf_en_type, engt, 0, 8);
    for (int i = 1; i < engc && i < 8; i++)
    {
        if (engt[i] != engt[0])
        {
            engc = i; break; // dummy engines, used by e.g. Carenado
        }
    }
    switch (engt[0])
    {
        case 6: case 7:
            yfs_printf_lft(yfms, 2, 0, COLR_IDX_GREEN, "%d ROCKET",     engc);
            break;
        case 0: case 1:
            yfs_printf_lft(yfms, 2, 0, COLR_IDX_GREEN, "%d PISTON",     engc);
            break;
        case 4:
            yfs_printf_lft(yfms, 2, 0, COLR_IDX_GREEN, "%d TURBOJET",   engc);
            break;
        case 5:
            yfs_printf_lft(yfms, 2, 0, COLR_IDX_GREEN, "%d TURBOFAN",   engc);
            break;
        case 2: case 8:
            yfs_printf_lft(yfms, 2, 0, COLR_IDX_GREEN, "%d TURBOPROP",  engc);
            break;
        default:
            yfs_printf_lft(yfms, 2, 0, COLR_IDX_GREEN, "%d OTHER (%d)", engc, engt[0]);
            break;
    }

    /* line 3: header (white, offset 1 right) */
    yfs_printf_lft(yfms,  3, 0, COLR_IDX_WHITE, "%s", " ACTIVE DATA BASE");

    /* line 4: navigation database information (blue) */
    yfs_printf_lft(yfms,  4, 0, COLR_IDX_BLUE, "%.10s", yfms->ndt.ndb->info.idnt);

    /* line 5: header (white, offset 1 right) */
    yfs_printf_lft(yfms,  5, 0, COLR_IDX_WHITE, "%s", " SECOND DATA BASE");

    /* line 6: navigation database information (blue) */
    yfs_printf_lft(yfms,  6, 0, COLR_IDX_BLUE, "%s", "NONE");

#ifndef NDT_VERSION
#define NDT_VERSION "Unknown"
#endif
    /* line 9: header (white, offset 1 right) */
    yfs_printf_lft(yfms, 9, 0, COLR_IDX_WHITE, "%s", " VERSION");

    /* line 10: version information (blue) */
    yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE, "%s", NDT_VERSION);

    /* line 11: header (white, offset 1 right) */
    yfs_printf_lft(yfms, 11, 0, COLR_IDX_WHITE, "%s", " IDLE/PERF");

    /* line 12: version information (green) */
    yfs_printf_lft(yfms, 12, 0, COLR_IDX_GREEN, "%s", "+0.0/+0.0");

    /* all good */
    yfms->mwindow.screen.redraw = 1; return;
}

static void yfs_lsk_callback_menu(void *context, int key[2], intptr_t refcon)
{
    yfms_context *yfms = context;
    if   (NULL == yfms || yfms->mwindow.current_page != PAGE_MENU)
    {
        return; // callback not applicable to current page
    }
    if (key && key[0] == 0 && key[1] == 0) // lsk1
    {
        yfs_idnt_pageopen(yfms); return;
    }
    if (key && key[0] == 1 && key[1] == 0) // rsk1
    {
        yfs_menu_resetall(yfms);
        if (XPIsWidgetVisible(yfms->mwindow.id) == 0)
        {
            yfs_main_toggl(yfms);
        }
        return;
    }
}

void yfs_curr_pageupdt(yfms_context *yfms)
{
    if (yfms && XPIsWidgetVisible(yfms->mwindow.id))
    {
        /* miscellaneous dataref update */
//      // not yet used
//      yfms->data.trp_altitude = ndt_distance_init((int64_t)(XPLMGetDataf(yfms->xpl.tropopause) / .3048), NDT_ALTUNIT_FT);

        /* only update visible page */
        switch (yfms->mwindow.current_page)
        {
            case PAGE_FPLN:
                yfs_fpln_pageupdt(yfms);
                break;
            case PAGE_INIT:
                yfs_init_pageupdt(yfms);
                break;
            case PAGE_PROG:
                yfs_prog_pageupdt(yfms);
                break;
            case PAGE_RAD1:
            case PAGE_RAD2:
                yfs_rdio_pageupdt(yfms);
                break;
            default:
                break;
        }
    }
    return;
}

void yfs_fmgs_phase_set(yfms_context *yfms, int new_phase)
{
#if TIM_ONLY
    int open_radio_1_on_end = 1;
#else
    int open_radio_1_on_end = 0;
#endif
    XPLMDataRef swtid, sendv; int index;
    switch (new_phase)
    {
        case FMGS_PHASE_PRE:
            ndt_log("YFMS [debug]: phase change: FMGS_PHASE_PRE (was %d)\n", yfms->data.phase);
            ndt_log("YFMS [debug]: FMGS_PHASE_PRE can only be set by yfs_flightplan_reinit\n");
            return;

        case FMGS_PHASE_TOF:
            if ((sendv = XPLMFindDataRef("sim/custom/xap/sendv")) &&
                (swtid = XPLMFindDataRef("sim/weapons/target_index")))
            {
                index = 1; XPLMSetDatavi(swtid, &index, 2, 1); // XXX: Falcon 7X by after
            }
            ndt_log("YFMS [debug]: phase change: FMGS_PHASE_TOF (was %d)\n", yfms->data.phase);
            yfms->data.phase = FMGS_PHASE_TOF;
            yfs_fpln_trackleg(yfms, -1);
            return;

        case FMGS_PHASE_CLB:
            ndt_log("YFMS [debug]: phase change: FMGS_PHASE_CLB (was %d)\n", yfms->data.phase);
            yfms->data.phase = FMGS_PHASE_CLB;
            return;

        case FMGS_PHASE_CRZ:
            if ((sendv = XPLMFindDataRef("sim/custom/xap/sendv")) &&
                (swtid = XPLMFindDataRef("sim/weapons/target_index")))
            {
                index = 2; XPLMSetDatavi(swtid, &index, 2, 1); // XXX: Falcon 7X by after
            }
            ndt_log("YFMS [debug]: phase change: FMGS_PHASE_CRZ (was %d)\n", yfms->data.phase);
            yfms->data.phase = FMGS_PHASE_CRZ;
            return;

        case FMGS_PHASE_DES:
            if ((sendv = XPLMFindDataRef("sim/custom/xap/sendv")) &&
                (swtid = XPLMFindDataRef("sim/weapons/target_index")))
            {
                index = 3; XPLMSetDatavi(swtid, &index, 2, 1); // XXX: Falcon 7X by after
                XPLMSetDatai(sendv, 2); // TODO: when switching to approach phase instead
            }
            ndt_log("YFMS [debug]: phase change: FMGS_PHASE_DES (was %d)\n", yfms->data.phase);
            yfms->data.phase = FMGS_PHASE_DES;
            return;

        case FMGS_PHASE_APP:
            ndt_log("YFMS [debug]: phase change: FMGS_PHASE_APP (was %d)\n", yfms->data.phase);
            ndt_log("YFMS [error]: phase change: FMGS_PHASE_APP NOT IMPLEMENTED\n");
            return;

        case FMGS_PHASE_GOA:
            ndt_log("YFMS [debug]: phase change: FMGS_PHASE_GOA (was %d)\n", yfms->data.phase);
            ndt_log("YFMS [error]: phase change: FMGS_PHASE_GOA NOT IMPLEMENTED\n");
            return;

        case FMGS_PHASE_END:
            if ((sendv = XPLMFindDataRef("sim/custom/xap/sendv")) &&
                (swtid = XPLMFindDataRef("sim/weapons/target_index")))
            {
                index = 4; XPLMSetDatavi(swtid, &index, 2, 1); XPLMSetDatai(sendv, 0); // XXX: Falcon 7X by after
            }
            ndt_log("YFMS [debug]: phase change: FMGS_PHASE_END (was %d)\n", yfms->data.phase);
            yfs_init_fplreset(yfms); yfms->data.phase = FMGS_PHASE_END;
            if (open_radio_1_on_end)
            {
                return yfs_rad1_pageopen(yfms);
            }
            return yfs_idnt_pageopen(yfms);

        default:
            ndt_log("YFMS [error]: phase change: unknown phase %d (was %d)\n", new_phase, yfms->data.phase);
            return;
    }
}

static float yfs_flight_loop_cback(float inElapsedSinceLastCall,
                                   float inElapsedTimeSinceLastFlightLoop,
                                   int   inCounter,
                                   void *inRefcon)
{
    yfms_context *yfms = inRefcon;
    if (!yfms)
    {
        ndt_log("YFMS [warning]: no context in flight loop callback!\n");
        return 0; // we're screwed
    }

    /* always update aircraft position (TODO: update tracked leg?) */
    yfms->data.aircraft_pos = ndt_position_init(XPLMGetDatad(yfms->xpl.latitude),
                                                XPLMGetDatad(yfms->xpl.longitude),
                                                ndt_distance_init((int64_t)(XPLMGetDatad(yfms->xpl.elevation_msl) / .3048), NDT_ALTUNIT_FT));

    /*
     * process delayed actions from the radio page
     *
     * do it here because yfs_rad1_pageupdt can't, as it's called for the same
     * flight model iteration as data can be be written to applicable datarefs
     */
    if (yfms->data.rdio.hsi_obs_deg_mag_rest > 0)
    {
        if (yfms->data.rdio.hsi_obs_deg_mag_rest > 1)
        {
            yfms->data.rdio.hsi_obs_deg_mag_rest = 1; return 0.125f;
        }
        XPLMSetDataf(yfms->xpl.  hsi_obs_deg_mag_pilot, yfms->data.rdio.hsi_obs_deg_mag_left);
        XPLMSetDataf(yfms->xpl.hsi_obs_deg_mag_copilot, yfms->data.rdio.hsi_obs_deg_mag_rigt);
        yfms->data.rdio.hsi_obs_deg_mag_rest = 0;
    }
    if (yfms->data.rdio.asrt_delayed_baro_s)
    {
        float unip; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_f32_lunip, &unip);
        if (yfms->data.rdio.asrt_delayed_baro_u != (int32_t)roundf(unip))
        {
            return .125f;
        }
        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_s32_lvalu, &yfms->data.rdio.asrt_delayed_baro_v);
        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_s32_rvalu, &yfms->data.rdio.asrt_delayed_baro_v);
        yfms->data.rdio.asrt_delayed_baro_s = 0; return 0.125f;
    }
    if (yfms->data.rdio.asrt_delayed_redraw)
    {
        yfms->data.rdio.asrt_delayed_redraw = 0; return 0.250f;
    }

    /* if main window visible, update currently displayed page */
    yfs_curr_pageupdt(yfms);

    /* skip some computations when we're in the "done" flight phase */
    if (yfms->data.phase <= FMGS_PHASE_END &&
        yfms->xpl.otto.vclb_vdes == 0 &&
        yfms->xpl.otto.vmax_auto == 0)
    {
        return DEFAULT_CALLBACK_RATE;
    }

    /* update FMGS phase (inspired by switching conditions, in FMGS P. Guide) */
    int crzfeet = ndt_distance_get(yfms->data.init.crz_alt, NDT_ALTUNIT_FT);
    float gskts = XPLMGetDataf(yfms->xpl.groundspeed) * 3.6f / 1.852f;
    int aglfeet = XPLMGetDataf(yfms->xpl.elevation_agl) / 0.3048f;
    int mcpfeet = XPLMGetDataf(yfms->xpl.altitude_dial_mcp_feet);
    float askts = XPLMGetDataf(yfms->xpl.airspeed_kts_pilot);
    int mslfeet = XPLMGetDataf(yfms->xpl.altitude_ft_pilot);
    int vvi_fpm = XPLMGetDataf(yfms->xpl.vvi_fpm_pilot);
    switch (yfms->data.phase)
    {
        case FMGS_PHASE_END:
            goto vclb_vdes_vmax_auto;

        case FMGS_PHASE_PRE:
            if (crzfeet > 1000)
            {
                if ((gskts >=  50.0f || gskts >= XPLMGetDataf(yfms->xpl.acf_vs0)) &&
                    (askts >= 100.0f || askts >= XPLMGetDataf(yfms->xpl.acf_vs0)))
                {
                    yfs_fmgs_phase_set(yfms, FMGS_PHASE_TOF);
                    break;
                }
                break;
            }
            break;

        case FMGS_PHASE_TOF:
            if (aglfeet > 400 && !XPLMGetDatai(yfms->xpl.autothrottle_enabled))
            {
                if (2 == XPLMGetDatai(yfms->xpl.vvi_status) ||
                    2 == XPLMGetDatai(yfms->xpl.pitch_status))
                {
                    if (XPLMFindDataRef("sim/custom/xap/sendv") &&
                        XPLMFindDataRef("sim/weapons/target_index"))
                    {
                        //fixme "sim/custom/xap/V2"+20 (tested range: 112+20 to 142+20)
                        //fixme limits: SF1: 200, SF2: 190, SF3: 180, offset by -15kts?
                        XPLMSetDatai(yfms->xpl.autothrottle_enabled, 1); // XXX: Falcon 7X by after
                    }
                }
            }
            if (crzfeet > 1000)
            {
                // TODO: use acc. alt. instead of AGL elevation
                if (aglfeet > 800 || (crzfeet - mslfeet) < 1000)
                {
                    yfs_fmgs_phase_set(yfms, FMGS_PHASE_CLB);
                    break;
                }
                break;
            }
            break;
        case FMGS_PHASE_CLB:
            if (crzfeet > 1000)
            {
                if (aglfeet > 2500 || (crzfeet - mslfeet) < 1000)
                {
                    XPLMDataRef sendv = XPLMFindDataRef("sim/custom/xap/sendv");
                    if (sendv && XPLMFindDataRef("sim/weapons/target_index"))
                    {
                        XPLMSetDatai(sendv, 0); // XXX: Falcon 7X by after
                    }
                }
                if ((crzfeet - mslfeet) < 50)
                {
                    yfs_fmgs_phase_set(yfms, FMGS_PHASE_CRZ);
                    break;
                }
                break;
            }
            break;
        case FMGS_PHASE_CRZ:
            if (crzfeet > 1000)
            {
                // TODO: passing T/D condition
                if (vvi_fpm < -250) // descending
                {
                    if ((crzfeet - mcpfeet) > 1000) // TODO: don't descend if distance remaining > 200nm
                    {
                        yfs_fmgs_phase_set(yfms, FMGS_PHASE_DES);
                        break;
                    }
                }
                break;
            }
            break;
        case FMGS_PHASE_DES:
            if (crzfeet > 1000)
            {
                break; // TODO: auto-activate APP at deceleration point
            }
            break;
        case FMGS_PHASE_GOA:
            if (crzfeet > 1000)
            {
                break; // TODO: auto-activate APP at deceleration point
            }
            break;
        case FMGS_PHASE_APP:
            if (crzfeet > 1000)
            {
                break; // TODO: activate GOA when applicable
            }
            break;
	default:
            break;
    }
    if (yfms->data.phase >= FMGS_PHASE_TOF) // don't reset before takeoff, duh!
    {
        if (aglfeet < 40)
        {
            if (gskts < 40.0f && askts < 40.0f) // TODO: wait 30 seconds, ignore speeds
            {
                yfs_fmgs_phase_set(yfms, FMGS_PHASE_END); return DEFAULT_CALLBACK_RATE;
            }
        }
    }
    if (yfms->data.phase <= FMGS_PHASE_GOA)
    {
        if (yfms->data.phase >= FMGS_PHASE_TOF)
        {
            yfs_fpln_fplnsync(yfms); // continuously sync w/XPLMNavigation to avoid surprises
        }
        if (yfms->data.phase >= FMGS_PHASE_CLB)
        {
            /*
             * IRL conditions: phase CLB to GOA, 300nm direct to DEST, ILS/similar approach set
             */
            if (yfms->data.fpln.l_rwy && yfms->data.fpln.l_rwy != yfms->data.fpln.l_ils) // new runway since we last set ILS
            {
                if (INT64_C(300) > ndt_distance_get(ndt_position_calcdistance(yfms->data.fpln.l_rwy->threshold, yfms->data.aircraft_pos), NDT_ALTUNIT_NM))
                {
                    if (yfms->data.fpln.l_rwy->ils.avail)
                    {
                        yfs_rdio_ils_data(yfms, ndt_frequency_get(yfms->data.fpln.l_rwy->ils.freq), yfms->data.fpln.l_rwy->ils.course, 1); // ILS data to NAV1
                    }
                    yfms->data.fpln.l_ils = yfms->data.fpln.l_rwy;
                }
            }
        }
        // TODO: NEW CRZ ALT (may happen during either of CLB/CRZ), re-enter CLB if required
    }

vclb_vdes_vmax_auto:
    /* autopilot-related functions */
    if (yfms->xpl.otto.vclb_vdes)
    {
        if (vvi_fpm < -500)
        {
            if (yfms->xpl.otto.vswitched >= 00)
            {
                yfms->xpl.otto.vswitched  = -1;
            }
            if (yfms->xpl.otto.vswitched == -1) // only switch once per descent
            {
                if (XPLMGetDatai(yfms->xpl.airspeed_is_mach) != 0)
                {
                    if (XPLMGetDataf(yfms->xpl.airspeed_kts_pilot) >= (float)(yfms->xpl.otto.vdes_kias))
                    {
                        // passing DES KIAS descending, switch A/T target to KIAS
                        yfms->xpl.otto.vswitched = -2; XPLMCommandOnce(yfms->xpl.knots_mach_toggle);
                        XPLMSetDataf(yfms->xpl.airspeed_dial_kts_mach, (float)yfms->xpl.otto.vdes_kias);
                    }
                }
            }
            if (yfms->xpl.otto.vswitched >= -2) // 10,000ft limit transition
            {
                if (10600 >= mslfeet) // 10,500ft + 1s @ 6,000ft/min descent
                {
                    if (XPLMGetDatai(yfms->xpl.airspeed_is_mach) != 0)
                    {
                        XPLMCommandOnce(yfms->xpl.knots_mach_toggle);
                    }
                    if (XPLMGetDataf(yfms->xpl.airspeed_dial_kts_mach) > 250.0f)
                    {
                        XPLMSetDataf(yfms->xpl.airspeed_dial_kts_mach, 250.0f);
                    }
                    yfms->xpl.otto.vswitched = -3;
                }
            }
        }
        if (vvi_fpm > +500)
        {
            if (yfms->xpl.otto.vswitched <= 00)
            {
                yfms->xpl.otto.vswitched  = +1;
            }
            if (yfms->xpl.otto.vswitched == +1) // only switch once per climb
            {
                if (XPLMGetDatai(yfms->xpl.airspeed_is_mach) == 0)
                {
                    if (XPLMGetDataf(yfms->xpl.machno) >= (float)(yfms->xpl.otto.vclb_mach / 1000.0f))
                    {
                        // passing CLB Mach climbing, switch A/T target to Mach
                        yfms->xpl.otto.vswitched = +2; XPLMCommandOnce(yfms->xpl.knots_mach_toggle);
                        XPLMSetDataf(yfms->xpl.airspeed_dial_kts_mach, (float)yfms->xpl.otto.vclb_mach / 1000.0f);
                    }
                }
            }
        }
    }
    if (yfms->xpl.otto.vmax_auto)
    {
        int airspeed_is_mach = XPLMGetDatai(yfms->xpl.airspeed_is_mach);
        if (vvi_fpm < -500 && airspeed_is_mach != 0) // Mach Number Descent
        {
            if (yfms->xpl.otto.vmax_flch >= 00)
            {
                yfms->xpl.otto.vmax_flch  = -1;
            }
            if (yfms->xpl.otto.vmax_flch == -1 && // only switch once per descent
                yfms->xpl.otto.flt_vmo > (float)mslfeet)
            {
                // passed FL/Vmo descending, switch A/T target to KIAS
                yfms->xpl.otto.vmax_flch = -2; XPLMCommandOnce(yfms->xpl.knots_mach_toggle);
                if (yfms->xpl.otto.vmax_kias <= (int)(XPLMGetDataf(yfms->xpl.airspeed_dial_kts_mach)))
                {
                    XPLMSetDataf(yfms->xpl.airspeed_dial_kts_mach, (float)yfms->xpl.otto.vmax_kias);
                }
            }
        }
        if (vvi_fpm > +500 && airspeed_is_mach == 0) // Indicated KTS Climb
        {
            if (yfms->xpl.otto.vmax_flch <= 00)
            {
                yfms->xpl.otto.vmax_flch  = +1;
            }
            if (yfms->xpl.otto.vmax_flch == +1 && // only switch once per climb
                yfms->xpl.otto.flt_vmo < (float)mslfeet)
            {
                // passed FL/Vmo climbing, switch A/T target to Mach
                yfms->xpl.otto.vmax_flch = +2; XPLMCommandOnce(yfms->xpl.knots_mach_toggle);
                if (yfms->xpl.otto.vmax_mach <= (int)(XPLMGetDataf(yfms->xpl.airspeed_dial_kts_mach) * 1000))
                {
                    XPLMSetDataf(yfms->xpl.airspeed_dial_kts_mach, (float)yfms->xpl.otto.vmax_mach / 1000.0f);
                }
            }
        }
    }

    return DEFAULT_CALLBACK_RATE;
}
