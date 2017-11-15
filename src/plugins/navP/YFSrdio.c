/*
 * YFSrdio.c
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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMProcessing.h"
#include "XPLM/XPLMUtilities.h"

#include "assert/includes.h"

#include "common/common.h"

#include "YFSmain.h"
#include "YFSrdio.h"
#include "YFSspad.h"

#define FB76_BARO_MIN (26.966629f) // rotary at 0.0f
#define FB76_BARO_MAX (32.873302f) // rotary at 1.0f
/*
 * check if a given barometric pressure is 29.92 InHg (or 1013 hPa, rounded)
 *                                           10_12.97                10_13.51 */
#define BPRESS_IS_STD(bpress) ((((bpress) >= 29.913) && ((bpress) <= 29.929)))
#define NAVTYP_IS_ILS(navtyp) ((navtyp == 8) || (navtyp == 1024))
/*
 * Theoretically, the float representation of an integer may be inexact, for
 * example, 118 could be stored as 118.999999 or 119.000001. This is no issue
 * when using rounding operations, but may change the result of floor or ceil
 * operations, however.
 *
 * While this may never actually happen, to play it safe, add a negligible
 * offset to floating-point values when using the aforementioned operations.
 */
#define YVP_FLOORDBL (00.005)
#define YVP_CEIL_DBL (-0.005)
#define YVP_FLOORFLT (0.005f)
#define YVP_CEIL_FLT (-.005f)

/*
 * navaid type aliases that can be used in a bitmask
 */
#define YVP_NAV_NDB 1
#define YVP_NAV_VOR 2
#define YVP_NAV_LOC 4
#define YVP_NAV_DME 8

static void yfs_rad1_pageupdt    (yfms_context *yfms);
static void yfs_rad2_pageupdt    (yfms_context *yfms);
static void yfs_lsk_callback_rad1(yfms_context *yfms, int key[2], intptr_t refcon);
static void yfs_lsk_callback_rad2(yfms_context *yfms, int key[2], intptr_t refcon);

/*
 * Frequency parsers and sanitizers.
 */
static double get_com_frequency(const char *string)
{
    double freq; size_t len;
    if (!string)
    {
        return -1.;
    }
    else
    {
        len = strnlen(string, 8);
    }
    if (len < 3 || len > 7) // min: "122", max: "122.800"
    {
        return -1.;
    }
    if (sscanf(string, "%lf", &freq) != 1)
    {
        return -1.;
    }
    while (1000. <= freq + YVP_FLOORDBL) // no decimal separator provided
    {
        freq /= 10.;
    }
    {
        /*
         * For com. radios, the "tick" is 8.33 kHz. For frequencies that don't
         * divide cleanly by our tick, always use the next (i.e. higher) tick.
         */
        freq = ceil(freq * 120. + YVP_CEIL_DBL) / 120.;
    }
    if (freq + YVP_FLOORDBL <= 118.000 || freq + YVP_CEIL_DBL >= 136.999) // valid range 118.* -> 136.* Mhz
    {
        return -1.;
    }
    /* For display purposes, we must round the frequency to .005 Mhz (1/200) */
    return (round(freq * 200.) / 200.);
}

static double get_nav_frequency(const char *string)
{
    double freq; size_t len;
    if (!string)
    {
        return -1.;
    }
    else
    {
        len = strnlen(string, 8);
    }
    if (len < 3 || len > 7) // min: "109", max: "109.900"
    {
        return -1.;
    }
    if (sscanf(string, "%lf", &freq) != 1)
    {
        return -1.;
    }
    while (1000. <= freq + YVP_FLOORDBL) // no decimal separator provided
    {
        freq /= 10.;
    }
    {
        /*
         * For nav. radios, the "tick" is 25.0 kHz. For frequencies that don't
         * divide cleanly by our tick, always use the next (i.e. higher) tick.
         */
        freq = ceil(freq * 40. + YVP_CEIL_DBL) / 40.;
    }
    if (freq + YVP_FLOORDBL <= 108.000 || freq + YVP_CEIL_DBL >= 117.999) // valid range 108.* -> 117.* Mhz
    {
        return -1.;
    }
    /* For display purposes, we must floor the frequency to .01 Mhz (1/100) */
    return (floor(freq * 100. + YVP_FLOORDBL) / 100.);
}

static double get_adf_frequency(const char *string)
{
    double freq; size_t len;
    if (!string)
    {
        return -1.;
    }
    else
    {
        len = strnlen(string, 8);
    }
    if (len < 3 || len > 7) // min: "175", max: "1750.00"
    {
        return -1.;
    }
    if (sscanf(string, "%lf", &freq) != 1)
    {
        return -1.;
    }
    while (1750. <= freq + YVP_FLOORDBL) // no decimal separator provided
    {
        freq /= 10.;
    }
    {
        /*
         * For such radios, the "tick" is 0.5 kHz. For frequencies that don't
         * divide cleanly by our tick, always use previous (i.e. lower) tick.
         */
        freq = floor(freq * 2. + YVP_FLOORDBL) / 2.;
    }
    if (freq + YVP_FLOORDBL <= 175. || freq + YVP_CEIL_DBL >= 1750.) // valid range 175 -> 1750 kHz
    {
        return -1.;
    }
    /* For X-Plane purposes, we must floor the frequency to 1.0 kHz */
    return (floor(freq + YVP_FLOORDBL));
}

static double get_frequency4idt(const char *idt, yfms_context *yfms, int navaid_types)
{
    int idx = strlen(idt) - 1;
    if (idt[idx] < 'A' || idt[idx] > 'Z')
    {
        return -1.; // not an identifier
    }
    ndt_distance  distce, newdis;
    ndt_waypoint *navaid = NULL, *currnt;
    ndt_position  acfpos = yfms->data.aircraft_pos;
    for (size_t i = 0; (currnt = ndt_navdata_get_waypoint(yfms->ndt.ndb, idt, &i)); i++)
    {
        switch (currnt->type)
        {
            case NDT_WPTYPE_NDB:
                if (navaid_types & YVP_NAV_NDB) break;
                continue;
            case NDT_WPTYPE_VOR:
                if (navaid_types & YVP_NAV_VOR) break;
                continue;
            case NDT_WPTYPE_LOC:
                if (navaid_types & YVP_NAV_LOC) break;
                continue;
            case NDT_WPTYPE_DME:
                if (navaid_types & YVP_NAV_DME) break;
                continue;
            default:
                continue;
        }
        if (navaid == NULL)
        {
            distce = ndt_position_calcdistance(acfpos, currnt->position);
            navaid = currnt;
            continue;
        }
        if (ndt_distance_get((newdis = ndt_position_calcdistance(acfpos, currnt->position)), NDT_ALTUNIT_NA) <
            ndt_distance_get((distce), NDT_ALTUNIT_NA))
        {
            distce = newdis;
            navaid = currnt;
            continue;
        }
    }
    if (navaid)
    {
        double freq = ndt_frequency_get(navaid->frequency);
        switch(navaid->type)
        {
            case NDT_WPTYPE_NDB: // floor frequency (1.0 kHz, required by X-Plane)
                return (floor(freq + YVP_FLOORDBL));
            case NDT_WPTYPE_VOR:
            case NDT_WPTYPE_LOC:
            case NDT_WPTYPE_DME: // for display purposes, floor frequency (10.0 kHz)
                return (floor(freq * 100. + YVP_FLOORDBL) / 100.);
            default:
                return -1.;
        }
    }
    return -1.;
}

static double get_baro_pressure(const char *string)
{
    double press; size_t len;
    if (!string)
    {
        return -1.;
    }
    else
    {
        len = strnlen(string, 6);
    }
    if (len < 2 || len > 5) // min: "29", max: "29.92"
    {
        return -1.;
    }
    if (sscanf(string, "%lf", &press) != 1)
    {
        return -1.;
    }
    if (1356. <= press + YVP_FLOORDBL) // InHg, no decimal separator provided
    {
        press /= 100.;
    }
    else if (40. <= press + YVP_FLOORDBL) // between 41 and 1355: hPa
    {
        press = (press + .04) / 33.86389; // STD: 1013.00 -> 1013.04 -> 2991.51 -> 2992.00 -> 29.92
        if (press + YVP_CEIL_DBL >= 40.) press = 40.; // sanitize for next check: 1355 -> 1354.5556
    }
    if (press + YVP_FLOORDBL <= 1. || press + YVP_CEIL_DBL >= 40.) // valid range 1.00 -> 40.00 InHg
    {
        return -1.;
    }
    /* For display purposes, we must round the value to .01 InHg (1/100) */
    return (round(press * 100.) / 100.);
}

void yfs_rad1_pageopen(yfms_context *yfms)
{
    if (yfms == NULL)
    {
        return; // no error
    }
    if (yfs_main_newpg(yfms, PAGE_RAD1))
    {
        return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_QPAC)
    {
        XPLMCommandOnce(yfms->xpl.qpac.VHF1Capt);
        XPLMCommandOnce(yfms->xpl.qpac.VHF2Co);
    }
    yfms->lsks[0][0].cback = yfms->lsks[1][0].cback =
    yfms->lsks[0][1].cback = yfms->lsks[1][1].cback =
    yfms->lsks[0][3].cback = yfms->lsks[1][3].cback =
    yfms->lsks[0][4].cback = yfms->lsks[1][4].cback =
    yfms->lsks[0][5].cback = yfms->lsks[1][5].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
    yfs_rdio_pageupdt(yfms); return;
}

void yfs_rad2_pageopen(yfms_context *yfms)
{
    if (yfms == NULL)
    {
        return; // no error
    }
    if (yfs_main_newpg(yfms, PAGE_RAD2))
    {
        return;
    }
    yfms->lsks[0][0].cback = yfms->lsks[1][0].cback =
    yfms->lsks[0][1].cback = yfms->lsks[1][1].cback =
    yfms->lsks[0][2].cback = yfms->lsks[0][3].cback =
    yfms->lsks[0][4].cback = yfms->lsks[1][4].cback = (YFS_LSK_f)&yfs_lsk_callback_rad2;
    yfs_rdio_pageupdt(yfms); return;
}

void yfs_rdio_pageupdt(yfms_context *yfms)
{
    if (!yfms)
    {
        return; // no error
    }
    if (yfms->mwindow.current_page == PAGE_RAD1)
    {
        yfs_rad1_pageupdt(yfms); return;
    }
    if (yfms->mwindow.current_page == PAGE_RAD2)
    {
        yfs_rad2_pageupdt(yfms); return;
    }
}

static void get_altimeter(yfms_context *yfms, int out[3])
{
    if (yfms->xpl.atyp == YFS_ATYP_ASRT)
    {
        int32_t  lmode; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_s32_lmode, &lmode);
        int32_t  lunit; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_u32_lunit, &lunit);
        int32_t  lvalu; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_s32_lvalu, &lvalu);
        out[0] = lvalu; out[1] = lunit != 0; out[2] = lmode < 0 ? 1 : 2; return; // aircraft has dedicated STD mode
    }
    switch ((out[1] = yfms->ndt.alt.unit)) // our internal unit (X-Plane always uses InHg)
    {
        case 1: // hectoPascals
            out[0] = roundf(XPLMGetDataf(yfms->xpl.barometer_setting_in_hg_pilot) * 33.86389f);
            break;
        default: // inches of mercury
            out[0] = roundf(XPLMGetDataf(yfms->xpl.barometer_setting_in_hg_pilot) * 100.0f);
            break;
    }
    if (yfms->xpl.atyp == YFS_ATYP_QPAC) // aircraft has dedicated STD mode
    {
        out[2] = XPLMGetDatai(yfms->xpl.qpac.BaroStdCapt) ? 1 : 2; return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_Q380) // aircraft has dedicated STD mode
    {
        out[2] = XPLMGetDatai(yfms->xpl.q380.BaroStdCapt) ? 1 : 2; return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_Q350) // aircraft has dedicated STD mode
    {
        out[2] = XPLMGetDatai(yfms->xpl.q350.pressLeftButton) ? 1 : 2; return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_FB77) // aircraft has dedicated STD mode
    {
        out[2] = XPLMGetDatai(yfms->xpl.fb77.anim_175_button) ? 1 : 2; return;
    }
    out[2] = 0; return;
}

static void set_altimeter(yfms_context *yfms, int in[2])//fixme
{
    //fixme
}

enum
{
    XPDR_OFF,
    XPDR_SBY,
    XPDR_GND,
    XPDR_AUT,
    XPDR_ALT,
    XPDR_TAO,
    XPDR_TAR,
    XPDR_TST,
    XPDRTOGL,
};
static int get_transponder_mode(yfms_context *yfms)
{
    if (XPLMGetDatai(yfms->xpl.transponder_mode) == 0)
    {
        return XPDR_OFF;
    }
    if (yfms->xpl.atyp == YFS_ATYP_ASRT)
    {
        uint32_t altr; yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.xpdr.id_u32_altr, &altr);
        uint32_t mode; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.xpdr.id_u32_mode, &mode);
        uint32_t tcas; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.xpdr.id_u32_tcas, &tcas);
        if (mode > 1 && tcas > 0)
        {
            switch (tcas)
            {
                case 1:
                    return XPDR_TAO;
                default:
                    return XPDR_TAR;
            }
        }
        else
        {
            switch (mode)
            {
                case 0:
                    return XPDR_SBY;
                case 1:
                    return XPDR_AUT;
                default:
                    return altr ? XPDR_ALT : XPDR_GND;
            }
        }
    }
    if (yfms->xpl.atyp == YFS_ATYP_IXEG)
    {
        if ((int)roundf(XPLMGetDataf(yfms->xpl.ixeg.xpdr_stby_act)) == 2)
        {
            switch ((int)roundf(XPLMGetDataf(yfms->xpl.ixeg.xpdr_mode_act)))
            {
                case 2:
                    return XPDR_TAR;
                case 1:
                    return XPDR_TAO;
                default:
                    return XPDR_ALT;
            }
        }
        else
        {
            switch ((int)roundf(XPLMGetDataf(yfms->xpl.ixeg.xpdr_stby_act)))
            {
                case 0:
                    return XPDR_SBY;
                default:
                    return XPDR_AUT;
            }
        }
    }
    if (yfms->xpl.atyp == YFS_ATYP_FB76)
    {
        switch ((int)roundf(XPLMGetDataf(yfms->xpl.fb76.systemMode)))
        {
            case 5:
                return XPDR_TAR;
            case 4:
                return XPDR_TAO;
            case 2:
                return XPDR_GND;
            case 1:
                return XPDR_SBY;
            case 0:
                return XPDR_TST;
            default:
                return XPDR_ALT;
        }
    }
    if (yfms->xpl.atyp == YFS_ATYP_FB77)
    {
        switch (XPLMGetDatai(yfms->xpl.fb77.anim_85_switch))
        {
            case 4:
                return XPDR_TAR;
            case 3:
                return XPDR_TAO;
            case 1:
                return XPDR_GND;
            case 0:
                return XPDR_SBY;
            default:
                return XPDR_ALT;
        }
    }
    if (yfms->xpl.atyp == YFS_ATYP_QPAC)
    {
        switch (XPLMGetDatai(yfms->xpl.qpac.XPDRPower))
        {
            case 0:
                return XPDR_SBY;
            case 1:
                return XPDR_AUT;
            default:
                return XPDR_ALT;
        }
    }
    switch (XPLMGetDatai(yfms->xpl.transponder_mode))
    {
        case 0:
            return XPDR_OFF;
        case 1:
            return XPDR_SBY;
        default:
            return XPDR_ALT;
    }
}

static void set_transponder_mode(yfms_context *yfms, int mode)
{
    if (mode == XPDRTOGL) // ground <-> flight toggle
    {
        switch (get_transponder_mode(yfms))
        {
            case XPDR_SBY:
            case XPDR_AUT:
            case XPDR_GND:
                mode = XPDR_ALT; // ground: STBY/AUTO/GND -> flight: ALT
                break;
            default:
                mode = XPDR_AUT; // flight: ALT -> ground: AUTO or fallback
                break;
        }
    }
    if (yfms->xpl.atyp == YFS_ATYP_ASRT)
    {
        uint32_t tcas, tmod, xalt, xmod;
        switch  (mode)
        {
            case XPDR_OFF:
                tcas = 0; // THRT
                tmod = 0; // STBY
                xalt = 0; // OFF
                xmod = 0; // STBY
                break;
            case XPDR_SBY:
                tcas = 0; // THRT
                tmod = 0; // STBY
                xalt = 1; // ON
                xmod = 0; // STBY
                break;
            case XPDR_GND:
                tcas = 0; // THRT
                tmod = 0; // STBY
                xalt = 0; // OFF
                xmod = 2; // ON
                break;
            case XPDR_AUT:
                tcas = 0; // THRT
                tmod = 0; // STBY
                xalt = 1; // ON
                xmod = 1; // AUTO
                break;
            case XPDR_TAO:
                tcas = 1; // ALL
                tmod = 1; // TA
                xalt = 1; // ON
                xmod = 2; // ON
                break;
            case XPDR_TAR:
                tcas = 3; // BLW
                tmod = 2; // TA/RA
                xalt = 1; // ON
                xmod = 2; // ON
                break;
            case XPDR_TST:
            case XPDR_ALT:
            default:
                tcas = 0; // THRT
                tmod = 0; // STBY
                xalt = 1; // ON
                xmod = 2; // ON
                break;
        }
        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.xpdr.id_u32_tcas, &tcas);
        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.xpdr.id_u32_tmod, &tmod);
        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.xpdr.id_u32_altr, &xalt);
        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.xpdr.id_u32_mode, &xmod);
        return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_IXEG)
    {
        float mact, sact;
        switch (mode)
        {
            case XPDR_OFF:
            case XPDR_SBY:
                mact = 0.0f; // OFF
                sact = 0.0f; // STBY
                break;
            case XPDR_GND:
            case XPDR_AUT:
                mact = 0.0f; // OFF
                sact = 1.0f; // AUTO
                break;
            case XPDR_TAO:
                mact = 1.0f; // TA
                sact = 2.0f; // ON
                break;
            case XPDR_TAR:
                mact = 2.0f; // TA/RA
                sact = 2.0f; // ON
                break;
            case XPDR_TST:
            case XPDR_ALT:
            default:
                mact = 1.0f; // OFF
                sact = 2.0f; // ON
                break;
        }
        XPLMSetDataf(yfms->xpl.ixeg.xpdr_mode_act, mact);
        XPLMSetDataf(yfms->xpl.ixeg.xpdr_stby_act, sact);
        return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_QPAC)
    {
        int alt, pwr;
        switch (mode)
        {
            case XPDR_OFF:
                alt = 0; // OFF
                pwr = 0; // STBY
                break;
            case XPDR_SBY:
                alt = 1; // ON
                pwr = 0; // STBY
                break;
            case XPDR_GND: // currently write-only, TODO: implement+test reading
                alt = 0; // OFF
                pwr = 2; // ON
                break;
            case XPDR_AUT:
                alt = 1; // ON
                pwr = 1; // AUTO
                break;
            case XPDR_TAO:
            case XPDR_TAR:
            case XPDR_TST:
            case XPDR_ALT:
            default:
                alt = 1; // ON
                pwr = 2; // ON
                break;
        }
        XPLMSetDatai(yfms->xpl.qpac.XPDRAltitude, alt);
        XPLMSetDatai(yfms->xpl.qpac.XPDRPower,    pwr);
        return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_FB76)
    {
        switch (mode)
        {
            case XPDR_OFF:
            case XPDR_SBY:
            case XPDR_GND:
            case XPDR_AUT:
                XPLMSetDataf(yfms->xpl.fb76.systemMode, 1.0f); // STBY
                return;
            case XPDR_TAO:
                XPLMSetDataf(yfms->xpl.fb76.systemMode, 4.0f); // TA
                return;
            case XPDR_TAR:
                XPLMSetDataf(yfms->xpl.fb76.systemMode, 5.0f); // TA/RA
                return;
            case XPDR_TST:
                XPLMSetDataf(yfms->xpl.fb76.systemMode, 0.0f); // TEST
                return;
            case XPDR_ALT:
            default:
                XPLMSetDataf(yfms->xpl.fb76.systemMode, 3.0f); // ALT ON
                return;
        }
    }
    if (yfms->xpl.atyp == YFS_ATYP_FB77)
    {
        switch (mode)
        {
            case XPDR_OFF:
            case XPDR_SBY:
            case XPDR_GND:
            case XPDR_AUT:
                XPLMSetDatai(yfms->xpl.fb77.anim_85_switch, 0.0f); // STBY
                return;
            case XPDR_TAO:
                XPLMSetDatai(yfms->xpl.fb77.anim_85_switch, 3.0f); // TA
                return;
            case XPDR_TAR:
                XPLMSetDatai(yfms->xpl.fb77.anim_85_switch, 4.0f); // TA/RA
                return;
            case XPDR_TST:
            case XPDR_ALT:
            default:
                XPLMSetDatai(yfms->xpl.fb77.anim_85_switch, 2.0f); // XPNDR
                return;
        }
    }
    switch (mode)
    {
        case XPDR_OFF:
            XPLMSetDatai(yfms->xpl.transponder_mode, 0);
            return;
        case XPDR_SBY:
        case XPDR_GND:
        case XPDR_AUT:
            XPLMSetDatai(yfms->xpl.transponder_mode, 1);
            return;
        case XPDR_TAO:
        case XPDR_TAR:
        case XPDR_TST:
        case XPDR_ALT:
        default:
            XPLMSetDatai(yfms->xpl.transponder_mode, 2);
            return;
    }
}

static int get_transponder_code(yfms_context *yfms)
{
    if (yfms->xpl.atyp == YFS_ATYP_ASRT)
    {
        uint32_t u32_code; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.xpdr.id_u32_code, &u32_code); return u32_code;
    }
    return XPLMGetDatai(yfms->xpl.transponder_code);
}

static void set_transponder_code(yfms_context *yfms, int code)
{
    if (yfms->xpl.atyp == YFS_ATYP_ASRT)
    {
        uint32_t u32_code = code; yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.xpdr.id_u32_code, &u32_code); return;
    }
    XPLMSetDatai(yfms->xpl.transponder_code, code); return;
}

static void yfs_rad1_pageupdt(yfms_context *yfms)
{
    /* don't print updated data before processing delayed swap, if any */
    if (yfms->data.rdio.delayed_swap)
    {
        /* half the usual delay of a quarter second should work well enough */
        XPLMSetFlightLoopCallbackInterval(yfms->xpl.fl_callback, .125f, 1, yfms); return;
    }
    if (yfms->data.rdio.asrt_delayed_baro_s)
    {
        XPLMSetFlightLoopCallbackInterval(yfms->xpl.fl_callback, .125f, 1, yfms); return;
    }
    if (yfms->data.rdio.asrt_delayed_redraw)
    {
        return;
    }

    /* reset lines before drawing */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }
    yfs_printf_ctr(yfms, 0, COLR_IDX_WHITE, "%s", "ATC COMM");

    /* line 1: headers (white) */
    yfs_printf_lft(yfms, 1, 0, COLR_IDX_WHITE, "%s", "COM 1");
    yfs_printf_rgt(yfms, 1, 0, COLR_IDX_WHITE, "%s", "COM 2");

    /* line 3: headers (white) */
    yfs_printf_lft(yfms, 3, 0, COLR_IDX_WHITE, "%s", "STANDBY");
    yfs_printf_rgt(yfms, 3, 0, COLR_IDX_WHITE, "%s", "STANDBY");

    /* line 2: active frequencies (green) */
    yfs_printf_lft(yfms, 2, 0, COLR_IDX_GREEN, "%03d.%03d", XPLMGetDatai(yfms->xpl.com1_frequency_Mhz),         XPLMGetDatai(yfms->xpl.com1_frequency_khz));
    yfs_printf_rgt(yfms, 2, 0, COLR_IDX_GREEN, "%03d.%03d", XPLMGetDatai(yfms->xpl.com2_frequency_Mhz),         XPLMGetDatai(yfms->xpl.com2_frequency_khz));
    /* line 4: standby frequencies (blue) */
    yfs_printf_lft(yfms, 4, 0, COLR_IDX_BLUE,  "%03d.%03d", XPLMGetDatai(yfms->xpl.com1_standby_frequency_Mhz), XPLMGetDatai(yfms->xpl.com1_standby_frequency_khz));
    yfs_printf_rgt(yfms, 4, 0, COLR_IDX_BLUE,  "%03d.%03d", XPLMGetDatai(yfms->xpl.com2_standby_frequency_Mhz), XPLMGetDatai(yfms->xpl.com2_standby_frequency_khz));

    /* line 7: headers (white) */
    yfs_printf_ctr(yfms, 7,    COLR_IDX_WHITE, "%s", "XPDR");
    yfs_printf_lft(yfms, 7, 0, COLR_IDX_WHITE, "%s", "CODE");
    yfs_printf_rgt(yfms, 7, 0, COLR_IDX_WHITE, "%s", "MODE");

    /* line 8: XPDR information */
    int mode;
    {
        mode = get_transponder_mode(yfms);
    }
    switch (mode)
    {
        case XPDR_OFF:
            yfs_printf_lft(yfms, 8, 0, COLR_IDX_WHITE, "%s", "----");
            yfs_printf_rgt(yfms, 8, 0, COLR_IDX_WHITE, "%s", "OFF");
            break;
        case XPDR_SBY:
            yfs_printf_rgt(yfms, 8, 0, COLR_IDX_BLUE, "%s", "STBY");
            break;
        case XPDR_AUT:
            yfs_printf_rgt(yfms, 8, 0, COLR_IDX_BLUE, "%s", "AUTO");
            break;
        case XPDR_GND:
            yfs_printf_rgt(yfms, 8, 0, COLR_IDX_BLUE, "%s", "ON");
            break;
        case XPDR_TAO:
            yfs_printf_rgt(yfms, 8, 0, COLR_IDX_BLUE, "%s", "TA");
            break;
        case XPDR_TAR:
            yfs_printf_rgt(yfms, 8, 0, COLR_IDX_BLUE, "%s", "TA/RA");
            break;
        case XPDR_TST:
            yfs_printf_rgt(yfms, 8, 0, COLR_IDX_BLUE, "%s", "TEST");
            break;
        default:
            yfs_printf_rgt(yfms, 8, 0, COLR_IDX_BLUE, "%s", "ALT");
            break;
    }
    if (mode != XPDR_OFF)
    {
        yfs_printf_lft(yfms, 8, 0, COLR_IDX_BLUE, "%04d", XPLMGetDatai(yfms->xpl.transponder_code));
    }

    /* line 9: headers (white) */
    yfs_printf_ctr(yfms, 9,    COLR_IDX_WHITE, "%s", "PRESSURE");
    yfs_printf_lft(yfms, 9, 0, COLR_IDX_WHITE, "%s", "BARO");
    yfs_printf_rgt(yfms, 9, 0, COLR_IDX_WHITE, "%s", "UNIT");

    /* line 10: barometric altimeter information */
    int alt[3];
    {
        get_altimeter(yfms, alt);
    }
    switch (alt[1])
    {
        case 1: // hectoPascals
            yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE, "%s", "hPa");
            yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE, "%04.0f", (((float)alt[0])));
            break;
        default: // inches of mercury
            yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE, "%s", "InHg");
            yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE, "%05.2f", (((float)alt[0]) / 100.0f));
            break;
    }
    if (alt[2] == 1)
    {
        yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE, "%-5s", "STD");
    }

    /* line 12: switches (white) */
    if (alt[2] != 1 && ((alt[1] == 0 && alt[0] != 2992) ||
                        (alt[1] == 1 && alt[0] != 1013) || alt[2] == 2))
    {
            yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "%s", "<STD");
            yfms->lsks[0][5].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
    }
    else
    {
        yfms->lsks[0][5].cback = (YFS_LSK_f)NULL;
    }
    if (XPLMGetDatai(yfms->xpl.transponder_mode) >= 2)
    {
        switch (XPLMGetDatai(yfms->xpl.transponder_id))
        {
            case 1:
                yfs_printf_rgt(yfms, 12, 0, COLR_IDX_GREEN, "%s", "IDENT");
                break;
            default:
                yfs_printf_rgt(yfms, 12, 0, COLR_IDX_WHITE, "%s", "IDT>");
                break;
        }
    }
}

static void yfs_rad2_pageupdt(yfms_context *yfms)
{
    /* reset lines before drawing */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }
    yfs_printf_ctr(yfms, 0, COLR_IDX_WHITE, "%s", "RADIO NAV");

    /* buffers */
    char nav1_nav_id[5];
    char nav2_nav_id[5];
    char adf1_nav_id[4];
    char adf2_nav_id[4];

    /* relevant data */
    int nav1_course_deg_mag_pilot = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav1_course_deg_mag_pilot), 360.));
    int nav2_course_deg_mag_pilot = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav2_course_deg_mag_pilot), 360.));
    int nav1_obs_deg_mag_copilot  = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav1_obs_deg_mag_copilot ), 360.));
    int nav2_obs_deg_mag_copilot  = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav2_obs_deg_mag_copilot ), 360.));
    int nav2_obs_deg_mag_pilot    = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav2_obs_deg_mag_pilot   ), 360.));
    int nav1_obs_deg_mag_pilot    = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav1_obs_deg_mag_pilot   ), 360.));
    adf1_nav_id[XPLMGetDatab(yfms->xpl.adf1_nav_id, adf1_nav_id, 0, sizeof(adf1_nav_id) - 1)] = 0;
    adf2_nav_id[XPLMGetDatab(yfms->xpl.adf2_nav_id, adf2_nav_id, 0, sizeof(adf2_nav_id) - 1)] = 0;
    nav1_nav_id[XPLMGetDatab(yfms->xpl.nav1_nav_id, nav1_nav_id, 0, sizeof(nav1_nav_id) - 1)] = 0;
    nav2_nav_id[XPLMGetDatab(yfms->xpl.nav2_nav_id, nav2_nav_id, 0, sizeof(nav2_nav_id) - 1)] = 0;
    int HSI_source_select_copilot = XPLMGetDatai(yfms->xpl.HSI_source_select_copilot);
    int HSI_source_select_pilot   = XPLMGetDatai(yfms->xpl.HSI_source_select_pilot  );
    int adf1_frequency_hz         = XPLMGetDatai(yfms->xpl.adf1_frequency_hz        );
    int adf2_frequency_hz         = XPLMGetDatai(yfms->xpl.adf2_frequency_hz        );
    int nav1_frequency_hz         = XPLMGetDatai(yfms->xpl.nav1_frequency_hz        );
    int nav2_frequency_hz         = XPLMGetDatai(yfms->xpl.nav2_frequency_hz        );
    int autopilot_source          = XPLMGetDatai(yfms->xpl.autopilot_source         );
    int nav1_type                 = XPLMGetDatai(yfms->xpl.nav1_type                );
    int nav2_type                 = XPLMGetDatai(yfms->xpl.nav2_type                );
    int ils_in_use                = 0;
    if (nav1_course_deg_mag_pilot < 1)
    {
        nav1_course_deg_mag_pilot = 360;
    }
    if (nav2_course_deg_mag_pilot < 1)
    {
        nav2_course_deg_mag_pilot = 360;
    }
    if (nav1_obs_deg_mag_pilot    < 1)
    {
        nav1_obs_deg_mag_pilot    = 360;
    }
    if (nav2_obs_deg_mag_copilot  < 1)
    {
        nav2_obs_deg_mag_copilot  = 360;
    }

    /* line 1: headers (white) */
    yfs_printf_lft(yfms, 1, 0, COLR_IDX_WHITE, "%s", "VOR1/FREQ");
    yfs_printf_rgt(yfms, 1, 0, COLR_IDX_WHITE, "%s", "FREQ/VOR2");

    /* line 3: headers (white) */
    yfs_printf_lft(yfms, 3, 0, COLR_IDX_WHITE, "%s", "CRS");
    yfs_printf_rgt(yfms, 3, 0, COLR_IDX_WHITE, "%s", "CRS");

    /* line 5: headers (white) */
    yfs_printf_lft(yfms, 5, 0, COLR_IDX_WHITE, "%s", "ILS1/FREQ");
//  yfs_printf_rgt(yfms, 5, 0, COLR_IDX_WHITE, "%s", "CHANN/MLS");

    /* line 7: headers (white) */
    yfs_printf_lft(yfms, 7, 0, COLR_IDX_WHITE, "%s", "CRS");
//  yfs_printf_rgt(yfms, 7, 0, COLR_IDX_WHITE, "%s", "SLOPE/CRS");

    /* line 9: headers (white) */
    yfs_printf_lft(yfms, 9, 0, COLR_IDX_WHITE, "%s", "ADF1/FREQ");
    yfs_printf_rgt(yfms, 9, 0, COLR_IDX_WHITE, "%s", "FREQ/ADF2");

    /*
     * lines 2, 4, 6, 8: frequencies & associated courses (blue)
     *
     * autopilot 1 (copilot), HSI source 1 (nav2) => nav2 master
     * autopilot 1 (copilot), HSI source ? (nav1) => nav1 master
     * autopilot ? (> pilot), HSI source 1 (nav2) => nav2 master
     * autopilot ? (> pilot), HSI source ? (nav1) => nav1 master
     */
    if (yfms->xpl.atyp == YFS_ATYP_FB76)                                        // FlightFactor's Boeing 767
    {                                                                           // ILS is always X-Plane NAV2
        if (nav2_frequency_hz >= 10800)
        {
            if (yfms->xpl.ils.frequency_changed && NAVTYP_IS_ILS(nav2_type))
            {
                XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_pilot,   (nav2_obs_deg_mag_pilot   = nav2_course_deg_mag_pilot));
                XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_copilot, (nav2_obs_deg_mag_copilot = nav2_course_deg_mag_pilot));
            }
            {
                yfs_printf_lft(yfms, 6, 0, COLR_IDX_BLUE, "%4s/%06.2lf", nav2_nav_id[0] ? nav2_nav_id : " [ ]", nav2_frequency_hz / 100.);
            }
            if (NAVTYP_IS_ILS(nav2_type))
            {
                yfs_printf_lft(yfms, 8, 0, COLR_IDX_BLUE, "%03d", nav2_obs_deg_mag_pilot);
            }
            else // ILS frequency, but no signal, course not reliable
            {
                yfs_printf_lft(yfms, 8, 0, COLR_IDX_BLUE, "%s", " [ ]");
            }
            ils_in_use = 1;
        }
    }
    else if ((autopilot_source == 1 && HSI_source_select_copilot == 1) ||       // Generic plane
             (autopilot_source != 1 && HSI_source_select_pilot   == 1))         // w/NAV2 master
    {
        if (NAVTYP_IS_ILS(nav2_type))
        {
            if (yfms->xpl.ils.frequency_changed) // auto-set all OBS courses after user-requested ILS change
            {
                XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_pilot,   (nav1_obs_deg_mag_pilot   = nav2_course_deg_mag_pilot));
                XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_copilot, (nav1_obs_deg_mag_copilot = nav2_course_deg_mag_pilot));
                XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_pilot,   (nav2_obs_deg_mag_pilot   = nav2_course_deg_mag_pilot));
                XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_copilot, (nav2_obs_deg_mag_copilot = nav2_course_deg_mag_pilot));
            }
            yfs_printf_lft(yfms, 6, 0, COLR_IDX_BLUE, "%4s/%06.2lf", nav2_nav_id[0] ? nav2_nav_id : " [ ]", nav2_frequency_hz / 100.);
            yfs_printf_lft(yfms, 8, 0, COLR_IDX_BLUE, "%03d", nav2_course_deg_mag_pilot);
            ils_in_use = 1;
        }
    }
    else                                                                        // Generic plane
    {                                                                           // w/NAV1 master
        if (NAVTYP_IS_ILS(nav1_type))
        {
            if (yfms->xpl.ils.frequency_changed) // auto-set all OBS courses after user-requested ILS change
            {
                XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_pilot,   (nav1_obs_deg_mag_pilot   = nav1_course_deg_mag_pilot));
                XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_copilot, (nav1_obs_deg_mag_copilot = nav1_course_deg_mag_pilot));
                XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_pilot,   (nav2_obs_deg_mag_pilot   = nav1_course_deg_mag_pilot));
                XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_copilot, (nav2_obs_deg_mag_copilot = nav1_course_deg_mag_pilot));
            }
            yfs_printf_lft(yfms, 6, 0, COLR_IDX_BLUE, "%4s/%06.2lf", nav1_nav_id[0] ? nav1_nav_id : " [ ]", nav1_frequency_hz / 100.);
            yfs_printf_lft(yfms, 8, 0, COLR_IDX_BLUE, "%03d", nav1_course_deg_mag_pilot);
            ils_in_use = 1;
        }
    }
    if (ils_in_use == 0)
    {
        yfs_printf_lft(yfms, 6, 0, COLR_IDX_BLUE, "%s", " [ ]/[ ] ");
        yfs_printf_lft(yfms, 8, 0, COLR_IDX_BLUE, "%s", " [ ]");
    }
//  yfs_printf_rgt(yfms, 6, 0, COLR_IDX_WHITE, "%s", "---/---");
//  yfs_printf_rgt(yfms, 8, 0, COLR_IDX_WHITE, "%s", "-.-/---");

    /* NAV1/2 information: drawn earlier, but must be computed after ILS */
    if (yfms->xpl.atyp == YFS_ATYP_FB76)
    {
        nav2_obs_deg_mag_copilot = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.fb76.nav2_obs_deg_mag_pilot), 360.));
        nav2_nav_id[XPLMGetDatab(yfms->xpl.fb76.nav2_nav_id, nav2_nav_id, 0, sizeof(nav2_nav_id) - 1)] = 0;
        nav2_frequency_hz = XPLMGetDatai(yfms->xpl.fb76.nav2_frequency_hz);
        if (nav2_obs_deg_mag_copilot < 1)
        {
            nav2_obs_deg_mag_copilot = 360;
        }
    }
    yfs_printf_lft(yfms,  2, 0, COLR_IDX_BLUE,  "%4s/%06.2lf", nav1_nav_id[0] ? nav1_nav_id : " [ ]", nav1_frequency_hz / 100.);
    yfs_printf_rgt(yfms,  2, 0, COLR_IDX_BLUE, "%06.2lf/%-4s", nav2_frequency_hz / 100., nav2_nav_id[0] ? nav2_nav_id : "[ ] ");
    yfs_printf_lft(yfms,  4, 0, COLR_IDX_BLUE, "%03d", nav1_obs_deg_mag_pilot);
    yfs_printf_rgt(yfms,  4, 0, COLR_IDX_BLUE, "%03d", nav2_obs_deg_mag_copilot);

    /* line 10: ADF frequencies (blue) */
    yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE,  "%4s/%03d", adf1_nav_id[0] ? adf1_nav_id : " [ ]", adf1_frequency_hz);
    yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE, "%03d/%-4s", adf2_frequency_hz, adf2_nav_id[0] ? adf2_nav_id : "[ ] ");

    /* all good */
    yfms->xpl.ils.frequency_changed = 0; return;
}

static void yfs_lsk_callback_rad1(yfms_context *yfms, int key[2], intptr_t refcon)
{
    if (yfms == NULL || yfms->mwindow.current_page != PAGE_RAD1)
    {
        return; // callback not applicable to current page
    }
    if (key[1] == 0 && yfms->xpl.atyp == YFS_ATYP_QPAC)
    {
        switch (key[0])
        {
            case 0:
                XPLMCommandOnce(yfms->xpl.qpac.VHF1Capt);
                XPLMCommandOnce(yfms->xpl.qpac.RMPSwapCapt);
                break;
            default:
                XPLMCommandOnce(yfms->xpl.qpac.VHF2Co);
                XPLMCommandOnce(yfms->xpl.qpac.RMPSwapCo);
                break;
        }
        yfs_rad1_pageupdt(yfms); return;
    }
    if (key[0] == 0 && key[1] == 0)
    {
        if (yfms->xpl.atyp == YFS_ATYP_ASRT)
        {
            XPLMCommandOnce(yfms->xpl.com1_standy_flip);
            yfms->data.rdio.asrt_delayed_redraw = 1; return;
        }
        XPLMCommandOnce(yfms->xpl.com1_standy_flip); yfs_rad1_pageupdt(yfms); return;
    }
    if (key[0] == 1 && key[1] == 0)
    {
        if (yfms->xpl.atyp == YFS_ATYP_ASRT)
        {
            XPLMCommandOnce(yfms->xpl.com2_standy_flip);
            yfms->data.rdio.asrt_delayed_redraw = 1; return;
        }
        XPLMCommandOnce(yfms->xpl.com2_standy_flip); yfs_rad1_pageupdt(yfms); return;
    }
    if (key[1] == 1)
    {
        int hz8, mhz, khz; double freq; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            if (key[0] == 0)
            {
                snprintf(buf, sizeof(buf), "%.7s", yfms->mwindow.screen.text[4]);
                buf[7] = 0; yfs_spad_reset(yfms, buf, -1); return; // c1 sb to scratchpad
            }
            if (key[0] == 1)
            {
                snprintf(buf, sizeof(buf), "%.7s", yfms->mwindow.screen.text[4] + YFS_DISPLAY_NUMC - 7);
                buf[7] = 0; yfs_spad_reset(yfms, buf, -1); return; // c2 sb to scratchpad
            }
        }
        if ((freq = get_com_frequency(buf)) < 0.)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        else
        {
            hz8 = (int)round((freq * 1000.));
            mhz = (int)floor((freq + YVP_FLOORDBL));
            khz = (int)round((freq - (double)mhz) * 1000.);
        }
        if (key[0] == 0)
        {
            if (yfms->xpl.atyp == YFS_ATYP_QPAC)
            {
                XPLMCommandOnce(yfms->xpl.qpac.VHF1Capt);
                XPLMCommandOnce(yfms->xpl.qpac.RMPSwapCapt);
                XPLMSetDatai(yfms->xpl.com1_left_frequency_hz_833, hz8);
                yfms->data.rdio.delayed_swap = yfms->xpl.qpac.RMPSwapCapt;
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            XPLMSetDatai(yfms->xpl.com1_standby_frequency_Mhz, mhz);
            XPLMSetDatai(yfms->xpl.com1_standby_frequency_khz, khz);
            yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
        }
        if (key[0] == 1)
        {
            if (yfms->xpl.atyp == YFS_ATYP_QPAC)
            {
                XPLMCommandOnce(yfms->xpl.qpac.VHF2Co);
                XPLMCommandOnce(yfms->xpl.qpac.RMPSwapCo);
                XPLMSetDatai(yfms->xpl.com2_left_frequency_hz_833, hz8);
                yfms->data.rdio.delayed_swap = yfms->xpl.qpac.RMPSwapCo;
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            XPLMSetDatai(yfms->xpl.com2_standby_frequency_Mhz, mhz);
            XPLMSetDatai(yfms->xpl.com2_standby_frequency_khz, khz);
            yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
        }
    }
    if (key[0] == 0 && key[1] == 3)
    {
        size_t len; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            snprintf(buf, sizeof(buf), "%04d", get_transponder_code(yfms));
            yfs_spad_reset(yfms, buf, -1); return; // current code to scratchpad
        }
        if ((len = strnlen(buf, 5)) > 4)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        for (int i = 0; i < len; i++)
        {
            if (buf[i] < '0' || buf[i] > '7')
            {
                yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
            }
        }
        set_transponder_code(yfms, atoi(buf)); yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
    }
    if (key[0] == 1 && key[1] == 3)
    {
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf); int t = 0;
        if (strnlen(buf, 1) && !yfms->mwindow.screen.spad_reset)
        {
            if (!strcmp(buf, "OFF"))
            {
                set_transponder_mode(yfms, XPDR_OFF);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcmp(buf, "SBY") || !strcmp(buf, "STBY"))
            {
                set_transponder_mode(yfms, XPDR_SBY);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcmp(buf, "ON") || !strcmp(buf, "GND"))
            {
                set_transponder_mode(yfms, XPDR_GND);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcmp(buf, "AUTO"))
            {
                set_transponder_mode(yfms, XPDR_AUT);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcmp(buf, "ALT"))
            {
                set_transponder_mode(yfms, XPDR_ALT);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcmp(buf, "TA"))
            {
                set_transponder_mode(yfms, XPDR_TAO);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcmp(buf, "TARA") || !strcmp(buf, "TA/RA"))
            {
                set_transponder_mode(yfms, XPDR_TAR);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcmp(buf, "TEST"))
            {
                set_transponder_mode(yfms, XPDR_TST);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        else
        {
            set_transponder_mode(yfms, XPDRTOGL);
            yfs_rad1_pageupdt(yfms); return;
        }
    }
    if (key[0] == 0 && key[1] == 4)
    {
        float inhg, hpa; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            snprintf(buf, sizeof(buf), "%.5s", yfms->mwindow.screen.text[10]);
            buf[4 + !yfms->ndt.alt.unit] = 0;  yfs_spad_reset(yfms, buf, -1);
            return; // current baro to scratchpad
        }
        if ((inhg = (float)get_baro_pressure(buf)) < 0.0f)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        if (sscanf(buf, "%f", &hpa) != 1)
        {
            yfs_spad_reset(yfms, "UNEXPECTED ERROR", -1); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_ASRT)
        {
            uint32_t unit; int32_t lmode, rmode, value;
            yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_s32_lmode, &lmode);
            yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_s32_lmode, &rmode);
            if (lmode < 0) // STD -> corrected value
            {
                lmode = -lmode; yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_s32_lmode, &lmode);
            }
            if (rmode < 0) // STD -> corrected value
            {
                rmode = -rmode; yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_s32_rmode, &rmode);
            }
            if (roundf(hpa)          == roundf(inhg * 100.0f) || // e.g. 2992.0000 == 29.92*100
                roundf(hpa * 100.0f) == roundf(inhg * 100.0f))   // e.g. 29.92*100 == 29.92*100
            {
                unit = 0; value = (int32_t)roundf(inhg * 100.0f);
            }
            else
            {
                unit = 1; value = (int32_t)roundf(hpa);
            }
            // note: we cannot set the target unit and value in the same call
            yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_lunit, &unit);
            yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_runit, &unit);
            yfms->data.rdio.asrt_delayed_baro_v = value;
            yfms->data.rdio.asrt_delayed_baro_u = unit;
            yfms->data.rdio.asrt_delayed_baro_s = 1;
            yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_Q350)
        {
            float offset       = roundf(100.0f * (inhg - 29.92f));
            XPLMSetDatai(yfms->xpl.q350.pressLeftButton,       0);
            XPLMSetDatai(yfms->xpl.q350.pressRightButton,      0);
            XPLMSetDataf(yfms->xpl.q350.pressLeftRotary,  offset);
            XPLMSetDataf(yfms->xpl.q350.pressRightRotary, offset);
            yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_FB76)
        {
            if (inhg < FB76_BARO_MIN)   inhg = FB76_BARO_MIN;
            if (inhg > FB76_BARO_MAX)   inhg = FB76_BARO_MAX;
            float baro_range = inhg          - FB76_BARO_MIN;
            float full_range = FB76_BARO_MAX - FB76_BARO_MIN;
            float alt_rotary_value = baro_range / full_range;
            XPLMSetDataf(yfms->xpl.fb76.baroRotary_stby,  alt_rotary_value);
            XPLMSetDataf(yfms->xpl.fb76.baroRotary_left,  alt_rotary_value);
            XPLMSetDataf(yfms->xpl.fb76.baroRotary_right, alt_rotary_value);
            yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_FB77)
        {
            float alt_inhg = XPLMGetDataf(yfms->xpl.barometer_setting_in_hg_pilot);
            float rot_posn = XPLMGetDataf(yfms->xpl.fb77.anim_25_rotery);
            int   rot_indx = (int)roundf(100.0f * rot_posn);
            if   (rot_indx != 50 && BPRESS_IS_STD(alt_inhg))
            {
                // altimeter standard but rotary not halfway
                // we're in "STD" mode, switch to manual mode
                XPLMSetDatai(yfms->xpl.fb77.anim_175_button, 1);
            }
            // very limited rng. (29.42 -> 30.42)
            float rot_newpr = inhg - 29.92f + 0.5f;
            if (rot_newpr < 0.0f) rot_newpr = 0.0f;
            if (rot_newpr > 1.0f) rot_newpr = 1.0f;
            XPLMSetDataf(yfms->xpl.fb77.anim_25_rotery, rot_newpr);
            yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_QPAC)
        {
            XPLMSetDatai(yfms->xpl.qpac.BaroStdCapt, 0);
            XPLMSetDatai(yfms->xpl.qpac.BaroStdFO,   0);
        }
        if (yfms->xpl.atyp == YFS_ATYP_Q380)
        {
            XPLMSetDatai(yfms->xpl.q380.BaroStdCapt, 0);
            XPLMSetDatai(yfms->xpl.q380.BaroStdFO,   0);
        }
        if (yfms->xpl.atyp == YFS_ATYP_IXEG)
        {
            XPLMSetDataf(yfms->xpl.ixeg.baro_inhg_sby_0001_ind, inhg);
        }
        XPLMSetDataf(yfms->xpl.barometer_setting_in_hg_copilot, inhg);
        XPLMSetDataf(yfms->xpl.barometer_setting_in_hg_pilot,   inhg);
        yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
    }
    if (key[0] == 1 && key[1] == 4)
    {
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if  (strnlen(buf, 1) && !yfms->mwindow.screen.spad_reset)
        {
            if (!strcasecmp(buf, "InHg"))
            {
                if (yfms->xpl.atyp == YFS_ATYP_ASRT)
                {
                    uint32_t unit = 0;
                    yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_lunit, &unit);
                    yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_runit, &unit);
                    yfs_spad_clear(yfms); return;
                }
                yfms->ndt.alt.unit = 0; yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcasecmp(buf, "hPa"))
            {
                if (yfms->xpl.atyp == YFS_ATYP_ASRT)
                {
                    uint32_t unit = 1;
                    yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_lunit, &unit);
                    yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_runit, &unit);
                    yfs_spad_clear(yfms); return;
                }
                yfms->ndt.alt.unit = 1; yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        else
        {
            if (yfms->xpl.atyp == YFS_ATYP_ASRT)
            {
                uint32_t unit;
                yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_u32_lunit, &unit); unit = !unit;
                yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_lunit, &unit);
                yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_runit, &unit);
                return;
            }
            yfms->ndt.alt.unit = !yfms->ndt.alt.unit; yfs_rad1_pageupdt(yfms); return;
        }
    }
    if (key[0] == 0 && key[1] == 5)
    {
        if (yfms->xpl.atyp == YFS_ATYP_ASRT)
        {
            int32_t lmode, rmode;
            yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_s32_lmode, &lmode); lmode = -lmode;
            yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_s32_rmode, &rmode); rmode = -rmode;
            yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_s32_lmode, &lmode);
            yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_s32_rmode, &rmode);
            return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_QPAC)
        {
            XPLMSetDatai(yfms->xpl.qpac.BaroStdCapt, 1);
            XPLMSetDatai(yfms->xpl.qpac.BaroStdFO,   1);
            yfs_rad1_pageupdt(yfms); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_Q350)
        {
            XPLMSetDatai(yfms->xpl.q350.pressLeftButton,  1);
            XPLMSetDatai(yfms->xpl.q350.pressRightButton, 1);
            yfs_rad1_pageupdt(yfms); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_Q380)
        {
            XPLMSetDatai(yfms->xpl.q380.BaroStdCapt, 1);
            XPLMSetDatai(yfms->xpl.q380.BaroStdFO,   1);
            yfs_rad1_pageupdt(yfms); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_FB76)
        {
            XPLMSetDataf(yfms->xpl.fb76.baroRotary_stby,  0.5f);
            XPLMSetDataf(yfms->xpl.fb76.baroRotary_left,  0.5f);
            XPLMSetDataf(yfms->xpl.fb76.baroRotary_right, 0.5f);
            yfs_rad1_pageupdt(yfms); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_FB77)
        {
            // only reached w/altimeter in manual mode
            // switch altimeter to "STD" mode instead
            XPLMSetDatai(yfms->xpl.fb77.anim_175_button, 1);
            yfs_rad1_pageupdt(yfms); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_IXEG)
        {
            XPLMSetDataf(yfms->xpl.ixeg.baro_inhg_sby_0001_ind, 29.92f);
        }
        XPLMSetDataf(yfms->xpl.barometer_setting_in_hg_copilot, 29.92f);
        XPLMSetDataf(yfms->xpl.barometer_setting_in_hg_pilot,   29.92f);
        yfs_rad1_pageupdt(yfms); return;
    }
    if (key[0] == 1 && key[1] == 5)
    {
        if (XPLMGetDatai(yfms->xpl.transponder_mode) >= 2 &&
            XPLMGetDatai(yfms->xpl.transponder_id  ) == 0)
        {
            XPLMCommandOnce(yfms->xpl.transponder_ident);
        }
        yfs_rad1_pageupdt  (yfms); return;
    }
}

static void yfs_lsk_callback_rad2(yfms_context *yfms, int key[2], intptr_t refcon)
{
    if (yfms == NULL || yfms->mwindow.current_page != PAGE_RAD2)
    {
        return; // callback not applicable to current page
    }
    if (key[1] == 0) // NAV 1/2 frequency get/set
    {
        double freq; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            if (key[0] == 0)
            {
                snprintf(buf, sizeof(buf), "%06.2lf", (double)XPLMGetDatai(yfms->xpl.nav1_frequency_hz)      / 100.);
            }
            else if (yfms->xpl.atyp == YFS_ATYP_FB76)
            {
                snprintf(buf, sizeof(buf), "%06.2lf", (double)XPLMGetDatai(yfms->xpl.fb76.nav2_frequency_hz) / 100.);
            }
            else
            {
                snprintf(buf, sizeof(buf), "%06.2lf", (double)XPLMGetDatai(yfms->xpl.nav2_frequency_hz)      / 100.);
            }
            yfs_spad_reset(yfms, buf, -1); return; // frequency to scratchpad
        }
        if ((freq = get_frequency4idt(buf, yfms, YVP_NAV_VOR|YVP_NAV_LOC|YVP_NAV_DME)) < 0. &&
            (freq = get_nav_frequency(buf)) < 0.)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        if (key[0] == 0)
        {
            XPLMSetDatai(yfms->xpl.nav1_frequency_hz,      (int)round(freq * 100.));
        }
        else if (yfms->xpl.atyp == YFS_ATYP_FB76)
        {
            XPLMSetDatai(yfms->xpl.fb76.nav2_frequency_hz, (int)round(freq * 100.));
        }
        else
        {
            XPLMSetDatai(yfms->xpl.nav2_frequency_hz,      (int)round(freq * 100.));
        }
        /* don't update page (to avoid 2 consecutive user-noticeable redraws) */
        yfs_spad_clear(yfms); return;
    }
    if (key[1] == 1) // NAV 1/2 course get/set
    {
        int crs; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            if (key[0] == 0)
            {
                snprintf(buf, sizeof(buf), "%03d", (int)roundf(XPLMGetDataf(yfms->xpl.nav1_obs_deg_mag_pilot)));
            }
            else if (yfms->xpl.atyp == YFS_ATYP_FB76)
            {
                snprintf(buf, sizeof(buf), "%03d", (int)roundf(XPLMGetDataf(yfms->xpl.fb76.nav2_obs_deg_mag_pilot)));
            }
            else
            {
                snprintf(buf, sizeof(buf), "%03d", (int)roundf(XPLMGetDataf(yfms->xpl.nav2_obs_deg_mag_copilot)));
            }
            yfs_spad_reset(yfms, buf, -1); return; // course to scratchpad
        }
        if (sscanf(buf, "%d", &crs) != 1 || crs < 0 || crs > 360)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        if (crs < 1)
        {
            crs = 360;
        }
        if (key[0] == 0)
        {
            XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_pilot,   (float)crs);
            XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_copilot, (float)crs);
        }
        else if (yfms->xpl.atyp == YFS_ATYP_FB76)
        {
            XPLMSetDataf(yfms->xpl.fb76.nav2_obs_deg_mag_pilot, (float)crs);
        }
        else
        {
            XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_pilot,   (float)crs);
            XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_copilot, (float)crs);
        }
        yfs_spad_clear(yfms); yfs_rad2_pageupdt(yfms); return;
    }
    if (key[0] == 0 && key[1] == 2) // ILS 1 frequency get/set
    {
        double freq; int hz10; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            if (yfms->mwindow.screen.text[6][5] != '1')
            {
                return; // not an ILS frequency
            }
            strncpy(&buf[0], &yfms->mwindow.screen.text[6][5], 6); buf[6] = 0;
            yfs_spad_reset(yfms, buf, -1); return; // frequency to scratchpad
        }
        if ((freq = get_frequency4idt(buf, yfms, YVP_NAV_VOR|YVP_NAV_LOC|YVP_NAV_DME)) < 0. &&
            (freq = get_nav_frequency(buf)) < 0.)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        else
        {
            hz10 = (int)round(freq * 100.);
        }
        if (yfms->xpl.atyp == YFS_ATYP_FB76)
        {
            if ((((hz10)) > 11195) || (((hz10 / 10) % 2) == 0))
            {
                yfs_spad_reset(yfms, "FORMAT ERROR", -1); return; // non-ILS
            }
        }
        else
        {
            XPLMSetDatai(yfms->xpl.nav1_frequency_hz, hz10); // FB76: N/A
        }
        XPLMSetDatai(yfms->xpl.nav2_frequency_hz, hz10); yfs_spad_clear(yfms);
        /* don't update page, must wait */yfms->xpl.ils.frequency_changed = 1; return;
    }
    if (key[0] == 0 && key[1] == 3) // ILS 1 course get/set
    {
        int crs; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            if (yfms->mwindow.screen.text[6][5] != '1')
            {
                return; // not an ILS frequency: no associated course
            }
            strncpy(&buf[0], &yfms->mwindow.screen.text[8][0], 3); buf[3] = 0;
            yfs_spad_reset(yfms, buf, -1); return; // course to scratchpad
        }
        if (sscanf(buf, "%d", &crs) != 1 || crs < 0 || crs > 360)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        if (crs < 1)
        {
            crs = 360;
        }
        if (yfms->xpl.atyp != YFS_ATYP_FB76)
        {
            XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_pilot,    (float)crs);
            XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_copilot,  (float)crs);
            XPLMSetDataf(yfms->xpl.nav1_course_deg_mag_pilot, (float)crs);      // may reset itself (OK)
        }
        XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_pilot,    (float)crs);
        XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_copilot,  (float)crs);
        XPLMSetDataf(yfms->xpl.nav2_course_deg_mag_pilot, (float)crs);          // may reset itself (OK)
        yfs_spad_clear(yfms); yfs_rad2_pageupdt(yfms); return;
    }
    if (key[1] == 4) // ADF 1/2 frequency get/set
    {
        double freq; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            if (key[0] == 0)
            {
                snprintf(buf, sizeof(buf), "%03d", XPLMGetDatai(yfms->xpl.adf1_frequency_hz));
            }
            else
            {
                snprintf(buf, sizeof(buf), "%03d", XPLMGetDatai(yfms->xpl.adf2_frequency_hz));
            }
            yfs_spad_reset(yfms, buf, -1); return; // frequency to scratchpad
        }
        if ((freq = get_frequency4idt(buf, yfms, YVP_NAV_NDB)) < 0. &&
            (freq = get_adf_frequency(buf)) < 0.)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        switch (yfms->xpl.atyp)
        {
            case YFS_ATYP_FB76:
                {
                    int   i100 = ((int)round(freq)) / 100;
                    int   i010 = ((int)round(freq) - i100 * 100) / 10;
                    int   i001 = ((int)round(freq) - i100 * 100 - i010 * 10);
                    float f100 = (float)(16 + 1 - i100) / 16.0f; // 17 to 1: 17 positions, 16 increments
                    float f010 = (float)( 9 + 0 - i010) /  9.0f; //  9 to 0: 10 positions,  9 increments
                    float f001 = (float)( 9 + 0 - i001) /  9.0f; //  9 to 0: 10 positions,  9 increments
                    /*
                     * Seems like FlightFfactor's plugin doesn't always compare
                     * correctly, so let's floor to 2 decimals to help it a bit.
                     */
                    f100 = floorf(f100 * 100.0f + YVP_FLOORFLT) / 100.0f;
                    f010 = floorf(f010 * 100.0f + YVP_FLOORFLT) / 100.0f;
                    f001 = floorf(f001 * 100.0f + YVP_FLOORFLT) / 100.0f;
                    if (key[0] == 0)
                    {
                        XPLMSetDataf(yfms->xpl.fb76.leftBigRotary,   f100);
                        XPLMSetDataf(yfms->xpl.fb76.leftMidRotary,   f010);
                        XPLMSetDataf(yfms->xpl.fb76.leftSmallRotary, f001);
                    }
                    else
                    {
                        XPLMSetDataf(yfms->xpl.fb76.rightBigRotary,   1.00f - f100);
                        XPLMSetDataf(yfms->xpl.fb76.rightMidRotary,   1.00f - f010);
                        XPLMSetDataf(yfms->xpl.fb76.rightSmallRotary, 1.00f - f001);
                    }
                    break;
                }
            case YFS_ATYP_IXEG:
            {
                int i100 = ((int)round(freq)) / 100;
                int i010 = ((int)round(freq) - i100 * 100) / 10;
                int i001 = ((int)round(freq) - i100 * 100 - i010 * 10);
                if (key[0] == 0)
                {
                    XPLMSetDataf(yfms->xpl.ixeg.radios_adf1_100_act, (float)i100);
                    XPLMSetDataf(yfms->xpl.ixeg.radios_adf1_010_act, (float)i010);
                    XPLMSetDataf(yfms->xpl.ixeg.radios_adf1_001_act, (float)i001);
                }
                else
                {
                    XPLMSetDataf(yfms->xpl.ixeg.radios_adf2_100_act, (float)i100);
                    XPLMSetDataf(yfms->xpl.ixeg.radios_adf2_010_act, (float)i010);
                    XPLMSetDataf(yfms->xpl.ixeg.radios_adf2_001_act, (float)i001);
                }
                break;
            }
            default:
                {
                    if (key[0] == 0)
                    {
                        XPLMSetDatai(yfms->xpl.adf1_frequency_hz, (int)round(freq));
                    }
                    else
                    {
                        XPLMSetDatai(yfms->xpl.adf2_frequency_hz, (int)round(freq));
                    }
                }
                break;
        }
        /* don't update page (to avoid 2 consecutive user-noticeable redraws) */
        yfs_spad_clear(yfms); return;
    }

    /* all good */
    return;
}
