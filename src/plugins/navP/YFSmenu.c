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

    /* always reset aircraft type (and associated automatic features) */
    yfms->xpl.otto.vmax_auto = yfms->xpl.otto.vmax_flch = 0;
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

    /* user-provided data */
    yfms->data.init.crz_alt       = ndt_distance_init(0, NDT_ALTUNIT_NA);
    yfms->data.prog.fix           = NULL;
    yfms->data.init.to            = NULL;
    yfms->data.init.from          = NULL;
    yfms->data.init.aligned       = 0;
    yfms->data.init.ialized       = 0;
    yfms->data.fpln.awys.open     = 0;
    yfms->data.fpln.lrev.open     = 0;
    yfms->data.init.cost_index    = 0;
    yfms->data.init.flight_id[0]  = 0;
    yfms->data.init.corte_name[0] = 0;

    /* delayed setters */
    yfms->data.rdio.delayed_swap = NULL;
    yfms->data.rdio.asrt_delayed_baro_s = 0;
    yfms->data.rdio.asrt_delayed_redraw = 0;

    /* all good */
    if (XPIsWidgetVisible(yfms->mwindow.id))
    {
        yfs_main_toggl(yfms);
    }
    yfs_idnt_pageopen (yfms); return;
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
    return;
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
    return;
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
                                                ndt_distance_init((int64_t)(XPLMGetDatad(yfms->xpl.elevation) / .3048), NDT_ALTUNIT_FT));

    //fixme implement our own transponder automatic mode???

    /*
     * process delayed standby <=> active frequency swap from the radio page
     *
     * do it here because yfs_rad1_pageupdt can't, as it's called for the same
     * flight model iteration as the frequency is being written to the dataref
     */
    if (yfms->data.rdio.delayed_swap)
    {
        XPLMCommandOnce(yfms->data.rdio.delayed_swap);
        yfms->data.rdio.delayed_swap = NULL;
        return .125f; // give time for the swap to occur before printing again
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
        yfms->data.rdio.asrt_delayed_baro_s = 0; return .125f;
    }
    if (yfms->data.rdio.asrt_delayed_redraw)
    {
        yfms->data.rdio.asrt_delayed_redraw = 0; return .250f;
    }

    /* if main window visible, update currently displayed page */
    yfs_curr_pageupdt(yfms);

    /* autopilot-related functions */
    if (yfms->xpl.otto.vmax_auto)
    {
        int vspeed_ft_min = XPLMGetDataf(yfms->xpl.vvi_fpm_pilot);
        int airspeed_ismn = XPLMGetDatai(yfms->xpl.airspeed_is_mach);
        if (vspeed_ft_min < -750 && airspeed_ismn != 0) // Mach Number Descent
        {
            if (yfms->xpl.otto.vmax_flch >= 00)
            {
                yfms->xpl.otto.vmax_flch  = -1;
            }
            if (yfms->xpl.otto.vmax_flch == -1 && // only switch once per descent
                yfms->xpl.otto.flt_vmo > XPLMGetDataf(yfms->xpl.altitude_ft_pilot))
            {
                // passed FL/Vmo descending, switch A/T target to KIAS
                {
                    yfms->xpl.otto.vmax_flch = -2;
                    XPLMCommandOnce(yfms->xpl.knots_mach_toggle);
                }
                if (yfms->xpl.otto.vmax_kias <= (int)(XPLMGetDataf(yfms->xpl.airspeed_dial_kts_mach)))
                {
                    XPLMSetDataf(yfms->xpl.airspeed_dial_kts_mach, (float)yfms->xpl.otto.vmax_kias);
                }
            }
        }
        if (vspeed_ft_min > +750 && airspeed_ismn == 0) // Indicated KTS Climb
        {
            if (yfms->xpl.otto.vmax_flch <= 00)
            {
                yfms->xpl.otto.vmax_flch  = +1;
            }
            if (yfms->xpl.otto.vmax_flch == +1 && // only switch once per climb
                yfms->xpl.otto.flt_vmo < XPLMGetDataf(yfms->xpl.altitude_ft_pilot))
            {
                // passed FL/Vmo climbing, switch A/T target to Mach
                {
                    yfms->xpl.otto.vmax_flch = +2;
                    XPLMCommandOnce(yfms->xpl.knots_mach_toggle);
                }
                if (yfms->xpl.otto.vmax_mach <= (int)(XPLMGetDataf(yfms->xpl.airspeed_dial_kts_mach) * 1000))
                {
                    XPLMSetDataf(yfms->xpl.airspeed_dial_kts_mach, (float)yfms->xpl.otto.vmax_mach / 1000.0f);
                }
            }
        }
    }

    /* every 1/4 second should (almost) not be perceivable by users */
    return .25f;
}
