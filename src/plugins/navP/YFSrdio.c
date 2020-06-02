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

#define BARO_INHG2HPA (33.8638816) // 1 inHg in hPa
#define FB76_BARO_MIN (2696.6629f) // rotary at 0.0f
#define FB76_BARO_MAX (3287.3302f) // rotary at 1.0f
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
static void yfs_msw_callback_rad1(yfms_context *yfms, int rx, int ry,   int delta);
// TODO: mouse wheel callback for rad2 page

static inline int navaid_is_ils(int type)
{
    if (type & (xplm_Nav_ILS|xplm_Nav_Localizer))
    {
        return 1;
    }
    return 0;
}

/*
 * Frequency parsers and sanitizers.
 */
static double get_com_frequency(const char *string)
{
    double freq; int channel; size_t len;
    if (!string)
    {
        return -1.;
    }
    else
    {
        len = strnlen(string, 8);
    }
    if (len < 2 || len > 7) // min: "22", max: "122.800"
    {
        return -1.;
    }
    if (sscanf(string, "%lf", &freq) != 1)
    {
        return -1.;
    }
    if (*string != '1')
    {
        double i = 1.0;
        while (i < freq)
        {
            i *= 10.0;
        }
        freq += round(i);
    }
    while (1000. <= freq + YVP_FLOORDBL) // no decimal separator provided
    {
        freq /= 10.;
    }
    if (freq + YVP_FLOORDBL <= 118.000 || freq + YVP_CEIL_DBL >= 136.999) // valid range 118.* -> 136.* Mhz
    {
        return -1.;
    }
    else
    {
        channel = YVP_FLOORDBL + round(freq * 200.);
    }
    switch (channel % 20)
    {
        case 0:  // .?00: valid 25.0 kHz
        case 5:  // .?25: valid 25.0 kHz
        case 10: // .?50: valid 25.0 kHz
        case 15: // .?75: valid 25.0 kHz
            return (((double)channel) / 200.);

        case 4:  // .?20: mapped to 25.0 kHz
        case 9:  // .?45: mapped to 25.0 kHz
        case 14: // .?70: mapped to 25.0 kHz
        case 19: // .?95: mapped to 25.0 kHz
            return ((((double)channel) + 1) / 200.);

        case 1:  // .?05: valid 8.33 kHz
        case 2:  // .?10: valid 8.33 kHz
        case 3:  // .?15: valid 8.33 kHz
        case 6:  // .?30: valid 8.33 kHz
        case 7:  // .?35: valid 8.33 kHz
        case 8:  // .?40: valid 8.33 kHz
        case 11: // .?55: valid 8.33 kHz
        case 12: // .?60: valid 8.33 kHz
        case 13: // .?65: valid 8.33 kHz
        case 16: // .?80: valid 8.33 kHz
        case 17: // .?85: valid 8.33 kHz
        case 18: // .?90: valid 8.33 kHz
            return (((double)channel) / 200.);

        default: // unreachable
            return -1.;
    }
    return -1.; // unreachable
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
    if (*string != '1')
    {
        double i = 1.0;
        while (i < freq)
        {
            i *= 10.0;
        }
        freq += round(i);
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
        return -2.; // not an identifier
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

static int get_baro_pressure(const char *string, int out[2])
{
    double press; size_t len;
    if (!string)
    {
        return -1;
    }
    else
    {
        len = strnlen(string, 6);
    }
    if (len < 1 || len > 5) // min: "1", max: "40.00"
    {
        return -1;
    }
    if (sscanf(string, "%lf", &press) != 1)
    {
        return -1;
    }
    if (press >= (1.) && press <= (40.)) // inches of mercury, decimal separator
    {
        out[0] = round(press * 100.); out[1] = 0; return 0;
    }
    if (press >= (40. * BARO_INHG2HPA) && press <= (4000.)) // InHg, no separator
    {
        out[0] = round(press); out[1] = 0; return 0;
    }
    if (press >= (40.) && press <= (40. * BARO_INHG2HPA)) // hectoPascals
    {
        out[0] = round(press); out[1] = 1; return 0;
    }
    return -1;
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
    if (yfms->xpl.atyp == YFS_ATYP_TOLI || yfms->xpl.atyp == YFS_ATYP_QPAC)
    {
        XPLMCommandOnce(yfms->xpl.qpac.VHF1Capt);
        XPLMCommandOnce(yfms->xpl.qpac.VHF2Co);
    }
    yfms->lsks[0][0].cback = yfms->lsks[1][0].cback =
    yfms->lsks[0][1].cback = yfms->lsks[1][1].cback =
    yfms->lsks[0][3].cback = yfms->lsks[1][3].cback =
    yfms->lsks[0][4].cback = yfms->lsks[1][4].cback =
    yfms->lsks[0][5].cback = yfms->lsks[1][5].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
    yfms->mousew_callback  = (YFS_MSW_f)&yfs_msw_callback_rad1;
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
    if (yfms->xpl.has_custom_nav_radios == 0)
    {
        yfms->lsks[0][0].cback = yfms->lsks[1][0].cback =
        yfms->lsks[0][1].cback = yfms->lsks[1][1].cback =
        yfms->lsks[0][2].cback = yfms->lsks[0][3].cback =
        yfms->lsks[0][4].cback = yfms->lsks[1][4].cback = (YFS_LSK_f)&yfs_lsk_callback_rad2;
    }
    return yfs_rdio_pageupdt(yfms);
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

void yfs_rdio_ils_data(yfms_context *yfms, float frequency, int course, int mask)
{
    int hz10 = round(frequency * 100.) + YVP_FLOORDBL;
    if (course != -1)
    {
        if (course < 1 || course > 360)
        {
            course = 360;
        }
    }
    if (mask & 2)
    {
        if (frequency > 0.)
        {
            if (yfms->xpl.atyp == YFS_ATYP_FB76)
            {
                if ((((hz10)) > 11195) || (((hz10 / 10) % 2) == 0))
                {
                    yfs_spad_reset(yfms, "FORMAT ERROR", -1); return; // non-ILS
                }
            }
            XPLMSetDatai(yfms->xpl.nav2_frequency_hz, hz10);
        }
        if (course != -1)
        {
            XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_copilot, course);
            XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_pilot, course);
        }
    }
    if (mask & 1)
    {
        if (yfms->xpl.atyp == YFS_ATYP_FB76)
        {
            return; // FF-B767: NAV1: not an ILS-capable radio
        }
        if (frequency > 0.)
        {
            XPLMSetDatai(yfms->xpl.nav1_frequency_hz, hz10);
        }
        if (course != -1)
        {
            // X-Plane may overwrite the HSI course even if the OBS we've
            // updated isn't the current HSI source, we simply restore it
            if ((XPLMGetDatai(yfms->xpl.autopilot_source) == 0 && XPLMGetDatai(yfms->xpl.  HSI_source_select_pilot) == 2) ||
                (XPLMGetDatai(yfms->xpl.autopilot_source) == 1 && XPLMGetDatai(yfms->xpl.HSI_source_select_copilot) == 2))
            {
                yfms->data.rdio.hsi_obs_deg_mag_rigt = XPLMGetDataf(yfms->xpl.hsi_obs_deg_mag_copilot);
                yfms->data.rdio.hsi_obs_deg_mag_left = XPLMGetDataf(yfms->xpl.hsi_obs_deg_mag_pilot);
                yfms->data.rdio.hsi_obs_deg_mag_rest = 2;
            }
            XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_copilot, course);
            XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_pilot, course);
        }
    }
    return;
}

enum
{
    BARO_ANY = 0,
    BARO_STD = 1,
    BARO_SET = 2,
};

enum
{
    BARO_NHG = 0,
    BARO_HPA = 1,
};

static void get_altimeter(yfms_context *yfms, int out[3])
{
    if (yfms->xpl.atyp == YFS_ATYP_ASRT)
    {
        int32_t  lmode; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_s32_lmode, &lmode);
        int32_t  lunit; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_u32_lunit, &lunit);
        int32_t  lvalu; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_s32_lvalu, &lvalu);
        out[0] = lvalu; out[1] = lunit != 0; out[2] = lmode < 0 ? BARO_STD : BARO_SET; return; // aircraft has dedicated STD mode
    }
    switch ((out[1] = yfms->ndt.alt.unit)) // our internal unit (X-Plane always uses InHg)
    {
        case BARO_HPA: // hectoPascals
            out[0] = roundf(XPLMGetDataf(yfms->xpl.barometer_setting_in_hg_pilot) * (float)BARO_INHG2HPA);
            break;
        default: // inches of mercury
            out[0] = roundf(XPLMGetDataf(yfms->xpl.barometer_setting_in_hg_pilot) * 100.0f);
            break;
    }
    if (yfms->xpl.atyp == YFS_ATYP_TOLI) // aircraft has dedicated STD mode
    {
        out[2] = XPLMGetDatai(yfms->xpl.qpac.BaroStdCapt    ) ? BARO_STD : BARO_SET; return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_QPAC) // aircraft has dedicated STD mode
    {
        out[2] = XPLMGetDatai(yfms->xpl.qpac.BaroStdCapt    ) ? BARO_STD : BARO_SET; return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_Q380) // aircraft has dedicated STD mode
    {
        out[2] = XPLMGetDatai(yfms->xpl.q380.BaroStdCapt    ) ? BARO_STD : BARO_SET; return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_Q350) // aircraft has dedicated STD mode
    {
        out[2] = XPLMGetDatai(yfms->xpl.q350.pressLeftButton) ? BARO_STD : BARO_SET; return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_FB77) // aircraft has dedicated STD mode
    {
        // 1013 could be either of 29.92 or 29.91 so we need to check for it to avoid some issues
        if (((int)roundf(XPLMGetDataf(yfms->xpl.barometer_setting_in_hg_pilot) * 100.0f)) == 2992)
        {
            if (((int)roundf(100.0f * XPLMGetDataf(yfms->xpl.fb77.anim_25_rotery))) != 50)
            {
                out[2] = BARO_STD; return; // rotary not halfway -> we're in "STD" mode
            }
            out[2] = BARO_SET; return;
        }
        out[2] = BARO_SET; return;
    }
    out[2] = BARO_ANY; return;
}

static inline int standard_pressure(yfms_context *yfms)
{
    int alt[3]; get_altimeter(yfms, alt);
    if (alt[2] == BARO_STD) // always standard pressure
    {
        return 1;
    }
    if (alt[2] == BARO_SET) // never standard pressure
    {
        return 0;
    }
    if (((int)roundf(XPLMGetDataf(yfms->xpl.barometer_setting_in_hg_pilot) * 100.0f)) == 2992)
    {
        return 1;
    }
    return 0;
}

static void set_altimeter(yfms_context *yfms, int in[2])
{
    float hundred_inhg;
    {
        if (in[0] != -1) // setting a specific barometric pressure
        {
            if (in[1] != -1) // we'll be setting an actual baro value
            {
                do
                {
                    if (yfms->xpl.atyp == YFS_ATYP_ASRT)
                    {
                        uint32_t unit = yfms->ndt.alt.unit = !!in[1];
                        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_lunit, &unit);
                        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_runit, &unit);
                        break;
                    }
                    // X-Plane may round or even truncate the corresponding
                    // datarefs; our highest possible precision is .01 inHg
                    switch ((yfms->ndt.alt.unit = in[1]))
                    {
                        case BARO_HPA: // hectoPascals
                            hundred_inhg = roundf((float)(in[0] * 100) / (float)BARO_INHG2HPA);
                            break;
                        default: // inches of mercury
                            hundred_inhg = (float)in[0];
                            break;
                    }
                    if (yfms->xpl.atyp == YFS_ATYP_TOLI || yfms->xpl.atyp == YFS_ATYP_QPAC)
                    {
                        XPLMSetDatai(yfms->xpl.qpac.BaroUnitCapt,   !!yfms->ndt.alt.unit);
                        XPLMSetDatai(yfms->xpl.qpac.BaroUnitFO,     !!yfms->ndt.alt.unit);
                    }
                    if (yfms->xpl.atyp == YFS_ATYP_Q380)
                    {
                        XPLMSetDatai(yfms->xpl.q380.BaroUnitCapt,   !!yfms->ndt.alt.unit);
                        XPLMSetDatai(yfms->xpl.q380.BaroUnitFO,     !!yfms->ndt.alt.unit);
                    }
                    if (yfms->xpl.atyp == YFS_ATYP_Q350)
                    {
                        XPLMSetDatai(yfms->xpl.q350.pressModeLeft,  !!yfms->ndt.alt.unit);
                        XPLMSetDatai(yfms->xpl.q350.pressModeRight, !!yfms->ndt.alt.unit);
                    }
                    if (yfms->xpl.atyp == YFS_ATYP_FB77)
                    {
                        XPLMSetDatai(yfms->xpl.fb77.anim_67_switch, !!yfms->ndt.alt.unit);
                    }
                    break;
                }
                while (0);
            }
            else // not setting a value, just toggle our internal pressure unit
            {
                if (yfms->xpl.atyp == YFS_ATYP_ASRT)
                {
                    uint32_t lunit; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_u32_lunit, &lunit);
                    lunit = !lunit; yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_lunit, &lunit);
                    return          yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_u32_runit, &lunit);
                }
                if (yfms->xpl.atyp == YFS_ATYP_TOLI || yfms->xpl.atyp == YFS_ATYP_QPAC)
                {
                    XPLMSetDatai(yfms->xpl.qpac.BaroUnitCapt,   !yfms->ndt.alt.unit);
                    XPLMSetDatai(yfms->xpl.qpac.BaroUnitFO,     !yfms->ndt.alt.unit);
                }
                if (yfms->xpl.atyp == YFS_ATYP_Q380)
                {
                    XPLMSetDatai(yfms->xpl.q380.BaroUnitCapt,   !yfms->ndt.alt.unit);
                    XPLMSetDatai(yfms->xpl.q380.BaroUnitFO,     !yfms->ndt.alt.unit);
                }
                if (yfms->xpl.atyp == YFS_ATYP_Q350)
                {
                    XPLMSetDatai(yfms->xpl.q350.pressModeLeft,  !yfms->ndt.alt.unit);
                    XPLMSetDatai(yfms->xpl.q350.pressModeRight, !yfms->ndt.alt.unit);
                }
                if (yfms->xpl.atyp == YFS_ATYP_FB77)
                {
                    XPLMSetDatai(yfms->xpl.fb77.anim_67_switch, !yfms->ndt.alt.unit);
                }
                yfms->ndt.alt.unit = !yfms->ndt.alt.unit; return;
            }
        }
        else // standard barometric pressure requested
        {
            hundred_inhg = 2992.0f;
        }
    }
    if (yfms->xpl.atyp == YFS_ATYP_ASRT)
    {
        int32_t lmode;
        {
            yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.baro.id_s32_lmode, &lmode);
        }
        if (in[0] == -1) // dedicated STD pressure mode toggle
        {
            lmode = -lmode; // preserve captain-side QNH vs. QFE state
            yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_s32_lmode, &lmode);
            yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_s32_rmode, &lmode);
            return;
        }
        if (lmode < 0) // disable STD mode
        {
            lmode = -lmode; // preserve captain-side QNH vs. QFE state
            yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_s32_lmode, &lmode);
            yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.baro.id_s32_rmode, &lmode);
        }
        // note: we cannot set the target unit and value in the same call
        yfms->data.rdio.asrt_delayed_baro_u = yfms->ndt.alt.unit;
        yfms->data.rdio.asrt_delayed_baro_v = in[0];
        yfms->data.rdio.asrt_delayed_baro_s = 1;
        return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_Q350)
    {
        if (in[0] == -1) // dedicated STD pressure mode toggle
        {
            int std = XPLMGetDatai(yfms->xpl.q350.pressLeftButton);
            XPLMSetDatai   (yfms->xpl.q350.pressLeftButton,  !std);
            XPLMSetDatai   (yfms->xpl.q350.pressRightButton, !std);
            return;
        }
        else // disable STD mode
        {
            XPLMSetDatai(yfms->xpl.q350.pressLeftButton,  0);
            XPLMSetDatai(yfms->xpl.q350.pressRightButton, 0);
        }
        float offset    =    roundf(hundred_inhg  -  2992.0f);
        XPLMSetDataf(yfms->xpl.q350.pressLeftRotary,  offset);
        XPLMSetDataf(yfms->xpl.q350.pressRightRotary, offset);
        return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_FB77)
    {
        int alt[3]; get_altimeter(yfms, alt);
        if (in[0] == -1) // dedicated STD pressure mode toggle
        {
            if (alt[2] != BARO_STD)
            {
                /*
                 * Note: we detect STD mode in this specific aircraft when:
                 *
                 * - X-Plane's altimeter is set to standard pressure
                 * - plane's altimeter knob isn't in halfway position
                 *
                 * XXX: to ensure we can detect the mode correctly when going
                 * from 29.92 to the dedicated STD mode, we move said knob to
                 * to a non-halfway position before triggering the "STD" mode
                 * toggle button's associated dataref.
                 */
                XPLMSetDataf(yfms->xpl.fb77.anim_25_rotery, 0.49f); // 29.91
            }
            XPLMSetDatai(yfms->xpl.fb77.anim_175_button, 1); return;
        }
        if (alt[2] == BARO_STD)
        {
            XPLMSetDatai(yfms->xpl.fb77.anim_175_button, 1); // disable STD mode
        }
        float rotary;
        {
            ((rotary = (hundred_inhg / 100.0f) - 29.92f + 0.5f));
        }
        if (rotary < 0.0f)
        {
            rotary = 0.0f;
        }
        if (rotary > 1.0f)
        {
            rotary = 1.0f;
        }
        XPLMSetDataf(yfms->xpl.fb77.anim_25_rotery, rotary);
        return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_FB76)
    {
        if (hundred_inhg < FB76_BARO_MIN) hundred_inhg = FB76_BARO_MIN;
        if (hundred_inhg > FB76_BARO_MAX) hundred_inhg = FB76_BARO_MAX;
        float rotary_baro_range =         hundred_inhg - FB76_BARO_MIN;
        float rotary_full_range =        FB76_BARO_MAX - FB76_BARO_MIN;
        float baro_rotary_value = rotary_baro_range / rotary_full_range;
        XPLMSetDataf(yfms->xpl.fb76.baroRotary_stby,  baro_rotary_value);
        XPLMSetDataf(yfms->xpl.fb76.baroRotary_left,  baro_rotary_value);
        XPLMSetDataf(yfms->xpl.fb76.baroRotary_right, baro_rotary_value);
        return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_TOLI || yfms->xpl.atyp == YFS_ATYP_QPAC)
    {
        if (in[0] == -1) // dedicated STD pressure mode toggle
        {
            int std = XPLMGetDatai(yfms->xpl.qpac.BaroStdCapt);
            XPLMSetDatai    (yfms->xpl.qpac.BaroStdCapt, !std);
            XPLMSetDatai    (yfms->xpl.qpac.BaroStdFO,   !std);
            return;
        }
        XPLMSetDatai(yfms->xpl.qpac.BaroStdCapt, 0); // disable STD mode
        XPLMSetDatai(yfms->xpl.qpac.BaroStdFO,   0); // disable STD mode
    }
    if (yfms->xpl.atyp == YFS_ATYP_Q380)
    {
        if (in[0] == -1) // dedicated STD pressure mode toggle
        {
            int std = XPLMGetDatai(yfms->xpl.q380.BaroStdCapt);
            XPLMSetDatai    (yfms->xpl.q380.BaroStdCapt, !std);
            XPLMSetDatai    (yfms->xpl.q380.BaroStdFO,   !std);
            return;
        }
        XPLMSetDatai(yfms->xpl.q380.BaroStdCapt, 0); // disable STD mode
        XPLMSetDatai(yfms->xpl.q380.BaroStdFO,   0); // disable STD mode
    }
    if (yfms->xpl.atyp == YFS_ATYP_IXEG)
    {
        XPLMSetDataf(yfms->xpl.ixeg.baro_inhg_sby_0001_ind, hundred_inhg / 100.0f);
    }
    XPLMSetDataf(yfms->xpl.barometer_setting_in_hg_copilot, hundred_inhg / 100.0f);
    XPLMSetDataf(yfms->xpl.barometer_setting_in_hg_pilot,   hundred_inhg / 100.0f);
    return;
}

enum
{
    XPDR_OFF =  0,
    XPDR_SBY =  1,
    XPDR_AUT =  2,
    XPDR_GND =  3,
    XPDR_ALT =  4,
    XPDR_TAO =  5,
    XPDR_TAR =  6,
    XPDRTOGL = -1,
    XPDRCYCL = -2,
    XPDR_MIN = XPDR_OFF,
    XPDR_MAX = XPDR_TAR,
};

static int has_transponder_mode(yfms_context *yfms, int mode)
{
    switch (mode)
    {
        case XPDR_TAR:
        case XPDR_TAO:
            return (yfms->xpl.atyp == YFS_ATYP_ASRT ||
                    yfms->xpl.atyp == YFS_ATYP_IXEG ||
                    yfms->xpl.atyp == YFS_ATYP_FB76 ||
                    yfms->xpl.atyp == YFS_ATYP_FB77 ||
                    yfms->xpl.atyp == YFS_ATYP_TOLI);

        case XPDR_AUT:
            return (yfms->xpl.atyp == YFS_ATYP_ASRT ||
                    yfms->xpl.atyp == YFS_ATYP_IXEG ||
                    yfms->xpl.atyp == YFS_ATYP_QPAC);

        case XPDR_GND:
            return (yfms->xpl.atyp == YFS_ATYP_ASRT ||
                    yfms->xpl.atyp == YFS_ATYP_FB76 ||
                    yfms->xpl.atyp == YFS_ATYP_FB77 ||
                    yfms->xpl.atyp == YFS_ATYP_TOLI ||
                    yfms->xpl.atyp == YFS_ATYP_QPAC);

        default:
            break;
    }

    // all XP aircraft support these three basic modes
    return (mode == XPDR_OFF || mode == XPDR_SBY || mode == XPDR_ALT);
}

static int get_transponder_mode(yfms_context *yfms)
{
    if (yfms->xpl.atyp == YFS_ATYP_ASRT)
    {
        uint32_t altr; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.xpdr.id_u32_altr, &altr);
        uint32_t mode; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.xpdr.id_u32_mode, &mode);
        uint32_t tcas; yfms->xpl.asrt.api.ValueGet(yfms->xpl.asrt.xpdr.id_u32_tcas, &tcas);
        if (mode == 2)
        {
            switch (tcas)
            {
                case 1:
                    return XPDR_TAO;
                case 2:
                    return XPDR_TAR;
                default:
                    return altr > 0 ? XPDR_ALT : XPDR_GND;
            }
        }
        if (mode == 1)
        {
            return XPDR_AUT;
        }
        if (altr > 0)
        {
            return XPDR_SBY;
        }
        return XPDR_OFF;
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
            default:
                return XPDR_ALT;
        }
    }
    if (yfms->xpl.atyp == YFS_ATYP_FB77)
    {
        if (XPLMGetDatai(yfms->xpl.transponder_mode) == 0)
        {
            return XPDR_OFF;
        }
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
    if (yfms->xpl.atyp == YFS_ATYP_TOLI)
    {
        switch (XPLMGetDatai(yfms->xpl.qpac.XPDRPower))
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
            case 1:
                return XPDR_AUT;
            case 0:
                return XPLMGetDatai(yfms->xpl.qpac.XPDRAltitude) ? XPDR_SBY : XPDR_OFF;
            default:
                return XPLMGetDatai(yfms->xpl.qpac.XPDRAltitude) ? XPDR_ALT : XPDR_GND;
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
    if (mode == XPDRCYCL)
    {
        switch (get_transponder_mode(yfms))
        {
            case XPDR_TAR:
                mode = XPDR_OFF;
                break;
            case XPDR_TAO:
                if (has_transponder_mode(yfms, XPDR_TAR))
                {
                    mode = XPDR_TAR;
                    break;
                }
                mode = XPDR_OFF;
                break;
            case XPDR_ALT:
                if (has_transponder_mode(yfms, XPDR_TAO))
                {
                    mode = XPDR_TAO;
                    break;
                }
                mode = XPDR_OFF;
                break;
            case XPDR_AUT:
                mode = XPDR_ALT;
                break;
            case XPDR_GND:
            case XPDR_SBY:
                if (has_transponder_mode(yfms, XPDR_AUT))
                {
                    mode = XPDR_AUT;
                    break;
                }
                mode = XPDR_ALT;
                break;
            case XPDR_OFF:
            default:
                mode = XPDR_SBY;
                break;
        }
    }
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
                if (has_transponder_mode(yfms, XPDR_AUT))
                {
                    mode = XPDR_AUT;
                    break;
                }
                mode = XPDR_SBY; // flight or off -> ground: AUTO or SBY
                break;
        }
    }
    if (yfms->xpl.atyp == YFS_ATYP_ASRT)
    {
        uint32_t tmod, tcas, altr, xpdr;
        switch  (mode)
        {
            case XPDR_OFF:
            case XPDR_SBY:
                tmod = 1; // ALL
                tcas = 0; // STBY
                altr = 1; // ON
                xpdr = 0; // STBY
                break;
            case XPDR_AUT:
                tmod = 1; // ALL
                tcas = 2; // TA/RA
                altr = 1; // ON
                xpdr = 1; // AUTO
                break;
            case XPDR_TAO:
                tmod = 1; // ALL
                tcas = 1; // TA
                altr = 1; // ON
                xpdr = 2; // ON
                break;
            case XPDR_TAR:
                tmod = 1; // ALL
                tcas = 2; // TA/RA
                altr = 1; // ON
                xpdr = 2; // ON
                break;
            case XPDR_GND:// will not set X-Plane dataref to 2, so map it to ALT
            case XPDR_ALT:
            default:
                tmod = 1; // ALL
                tcas = 0; // STBY
                altr = 1; // ON
                xpdr = 2; // ON
                break;
        }
        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.xpdr.id_u32_tmod, &tmod);
        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.xpdr.id_u32_tcas, &tcas);
        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.xpdr.id_u32_altr, &altr);
        yfms->xpl.asrt.api.ValueSet(yfms->xpl.asrt.xpdr.id_u32_mode, &xpdr);
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
            case XPDR_AUT:
                mact = 2.0f; // TA/RA
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
            case XPDR_GND:
            case XPDR_ALT:
            default:
                mact = 0.0f; // OFF
                sact = 2.0f; // ON
                break;
        }
        XPLMSetDataf(yfms->xpl.ixeg.xpdr_mode_act, mact);
        XPLMSetDataf(yfms->xpl.ixeg.xpdr_stby_act, sact);
        return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_TOLI)
    {
        switch (mode)
        {
            case XPDR_OFF:
            case XPDR_SBY:
            case XPDR_AUT:
                XPLMSetDatai(yfms->xpl.qpac.XPDRPower, 0); // STBY
                XPLMSetDatai(yfms->xpl.qpac.XPDRTCASMode, 0); // AUTO
                XPLMSetDatai(yfms->xpl.qpac.XPDRTCASAltSelect, 1); // N
                break;
            case XPDR_TAO:
                XPLMSetDatai(yfms->xpl.qpac.XPDRPower, 3); // TA ONLY
                XPLMSetDatai(yfms->xpl.qpac.XPDRTCASMode, 1); // ON
                XPLMSetDatai(yfms->xpl.qpac.XPDRTCASAltSelect, 1); // N
                break;
            case XPDR_TAR:
                XPLMSetDatai(yfms->xpl.qpac.XPDRPower, 4); // TA/RA
                XPLMSetDatai(yfms->xpl.qpac.XPDRTCASMode, 1); // ON
                XPLMSetDatai(yfms->xpl.qpac.XPDRTCASAltSelect, 1); // N
                break;
            case XPDR_GND:
                XPLMSetDatai(yfms->xpl.qpac.XPDRPower, 1); // ALT RPTG OFF
                XPLMSetDatai(yfms->xpl.qpac.XPDRTCASMode, 0); // AUTO
                XPLMSetDatai(yfms->xpl.qpac.XPDRTCASAltSelect, 1); // N
                break;
            case XPDR_ALT:
            default:
                XPLMSetDatai(yfms->xpl.qpac.XPDRPower, 2); // XPDR
                XPLMSetDatai(yfms->xpl.qpac.XPDRTCASMode, 0); // AUTO
                XPLMSetDatai(yfms->xpl.qpac.XPDRTCASAltSelect, 1); // N
                break;
        }
        XPLMSetDatai(yfms->xpl.qpac.XPDRAltitude, 0);
        return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_QPAC)
    {
        switch (mode)
        {
            case XPDR_OFF:
            case XPDR_SBY:
                XPLMSetDatai(yfms->xpl.qpac.XPDRPower, 0); // STBY
                XPLMSetDatai(yfms->xpl.qpac.XPDRAltitude, mode != XPDR_OFF);
                break;
            case XPDR_AUT:
                XPLMSetDatai(yfms->xpl.qpac.XPDRPower,    1); // AUTO
                XPLMSetDatai(yfms->xpl.qpac.XPDRAltitude, 1); // ON
                break;
            case XPDR_TAO:
            case XPDR_TAR:
            case XPDR_GND:
            case XPDR_ALT:
            default:
                XPLMSetDatai(yfms->xpl.qpac.XPDRPower, 2); // ON
                XPLMSetDatai(yfms->xpl.qpac.XPDRAltitude, mode != XPDR_GND);
                break;
        }
        return;
    }
    if (yfms->xpl.atyp == YFS_ATYP_FB76)
    {
        switch (mode)
        {
            case XPDR_OFF:
            case XPDR_SBY:
            case XPDR_AUT:
                XPLMSetDataf(yfms->xpl.fb76.systemMode, 1.0f); // STBY
                return;
            case XPDR_TAO:
                XPLMSetDataf(yfms->xpl.fb76.systemMode, 4.0f); // TA
                return;
            case XPDR_TAR:
                XPLMSetDataf(yfms->xpl.fb76.systemMode, 5.0f); // TA/RA
                return;
            case XPDR_GND:
                XPLMSetDataf(yfms->xpl.fb76.systemMode, 2.0f); // ALT OFF
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
            case XPDR_AUT:
                XPLMSetDatai(yfms->xpl.fb77.anim_85_switch, 0.0f); // STBY
                return;
            case XPDR_TAO:
                XPLMSetDatai(yfms->xpl.fb77.anim_85_switch, 3.0f); // TA
                return;
            case XPDR_TAR:
                XPLMSetDatai(yfms->xpl.fb77.anim_85_switch, 4.0f); // TA/RA
                return;
            case XPDR_GND:
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
        case XPDR_AUT:
            XPLMSetDatai(yfms->xpl.transponder_mode, 1);
            return;
        case XPDR_GND:
        case XPDR_ALT:
        case XPDR_TAO:
        case XPDR_TAR:
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
    if (yfms->xpl.atyp == YFS_ATYP_TOLI || yfms->xpl.atyp == YFS_ATYP_QPAC)
    {
        int xpdr[4];
        xpdr[0] = (code / 1000); code = (code % 1000);
        xpdr[1] = (code /  100); code = (code %  100);
        xpdr[2] = (code /   10); code = (code %   10);
        xpdr[3] = (code /    1); code = (code %    1);
        XPLMSetDatai(yfms->xpl.qpac.XPDR[0], xpdr[0]);
        XPLMSetDatai(yfms->xpl.qpac.XPDR[1], xpdr[1]);
        XPLMSetDatai(yfms->xpl.qpac.XPDR[2], xpdr[2]);
        XPLMSetDatai(yfms->xpl.qpac.XPDR[3], xpdr[3]);
        return;
    }
    XPLMSetDatai(yfms->xpl.transponder_code, code); return;
}

static void yfs_rad1_pageupdt(yfms_context *yfms)
{
    /* don't print updated data before processing delayed actions, if any */
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
    yfs_printf_lft(yfms, 2, 0, COLR_IDX_GREEN, "%03d.%03d", XPLMGetDatai(yfms->xpl.com1_frequency_Mhz), XPLMGetDatai(yfms->xpl.com1_frequency_khz));
    yfs_printf_rgt(yfms, 2, 0, COLR_IDX_GREEN, "%03d.%03d", XPLMGetDatai(yfms->xpl.com2_frequency_Mhz), XPLMGetDatai(yfms->xpl.com2_frequency_khz));
    /* line 4: standby frequencies (blue) */
    if (yfms->xpl.atyp == YFS_ATYP_TOLI || yfms->xpl.atyp == YFS_ATYP_QPAC)
    {
        yfs_printf_lft(yfms, 4, 0, COLR_IDX_BLUE,  "%07.3lf", ((double)yfms->xpl.qpac.com1_sby_hz_833) / 1000.);
        yfs_printf_rgt(yfms, 4, 0, COLR_IDX_BLUE,  "%07.3lf", ((double)yfms->xpl.qpac.com2_sby_hz_833) / 1000.);
    }
    else
    {
        yfs_printf_lft(yfms, 4, 0, COLR_IDX_BLUE,  "%03d.%03d", XPLMGetDatai(yfms->xpl.com1_standby_frequency_Mhz), XPLMGetDatai(yfms->xpl.com1_standby_frequency_khz));
        yfs_printf_rgt(yfms, 4, 0, COLR_IDX_BLUE,  "%03d.%03d", XPLMGetDatai(yfms->xpl.com2_standby_frequency_Mhz), XPLMGetDatai(yfms->xpl.com2_standby_frequency_khz));
    }

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
        default:
            yfs_printf_rgt(yfms, 8, 0, COLR_IDX_BLUE, "%s", "ALT");
            break;
    }
    if (mode != XPDR_OFF)
    {
        yfs_printf_lft(yfms, 8, 0, COLR_IDX_BLUE, "%04d", get_transponder_code(yfms));
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
        case BARO_HPA: // hectoPascals
            yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE, "%s", "hPa");
            yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE, "%04.0f", (((float)alt[0])));
            if (standard_pressure(yfms))
            {
                yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE, "%06.1f", 1013.2f);
            }
            break;
        default: // inches of mercury
            yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE, "%s", "InHg");
            yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE, "%05.2f", (((float)alt[0]) / 100.0f));
            break;
    }
    if (alt[2] == BARO_STD)
    {
        yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE, "%-6s", "STD"); // %-6s: strlen("1013.2")
    }

    /* line 12: switches (white) */
    if (standard_pressure(yfms))
    {
        yfms->lsks[0][5].cback = (YFS_LSK_f)NULL;
    }
    else
    {
        yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "%s", "<STD");
        yfms->lsks[0][5].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
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

    /* all good */
    yfms->mwindow.screen.redraw = 1; return;
}

static void yfs_rad2_pageupdt(yfms_context *yfms)
{
    /* reset lines before drawing */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }
    if (yfms->xpl.has_custom_nav_radios)
    {
        yfms->mwindow.screen.redraw = 1;
        return yfs_printf_ctr(yfms, 6, COLR_IDX_WHITE, "%s", "PAGE INOP");
    }
    yfs_printf_ctr(yfms, 0, COLR_IDX_WHITE, "%s", "RADIO NAV");

    /* buffers */
    char nav1_nav_id[5];
    char nav2_nav_id[5];
    char adf1_nav_id[4];
    char adf2_nav_id[4];

    /* relevant data */
    int nav1_obs_deg_mag_copilot  = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav1_obs_deg_mag_copilot), 360.));
    int nav2_obs_deg_mag_copilot  = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav2_obs_deg_mag_copilot), 360.));
    int nav2_obs_deg_mag_pilot    = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav2_obs_deg_mag_pilot  ), 360.));
    int nav1_obs_deg_mag_pilot    = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav1_obs_deg_mag_pilot  ), 360.));
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
    if (nav1_obs_deg_mag_copilot < 1)
    {
        nav1_obs_deg_mag_copilot = 360;
    }
    if (nav2_obs_deg_mag_copilot < 1)
    {
        nav2_obs_deg_mag_copilot = 360;
    }
    if (nav1_obs_deg_mag_pilot < 1)
    {
        nav1_obs_deg_mag_pilot = 360;
    }
    if (nav2_obs_deg_mag_pilot < 1)
    {
        nav2_obs_deg_mag_pilot = 360;
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
    if (yfms->xpl.atyp == YFS_ATYP_FB76)                                        // FlightFactor Boeing 757/767
    {                                                                           // ILS is using X-Plane's NAV2
        if (nav2_frequency_hz >= 10800)
        {
            yfs_printf_lft(yfms, 6, 0, COLR_IDX_BLUE, "%4s/%06.2lf", nav2_nav_id[0] ? nav2_nav_id : " [ ]", nav2_frequency_hz / 100.);
            yfs_printf_lft(yfms, 8, 0, COLR_IDX_BLUE, "%03d", nav2_obs_deg_mag_pilot);
            ils_in_use = 1;
        }
    }
    else if ((autopilot_source == 1 && HSI_source_select_copilot == 1) ||       // Generic plane
             (autopilot_source != 1 && HSI_source_select_pilot   == 1))         // w/NAV2 master
    {
        if (navaid_is_ils(nav2_type))
        {
            yfs_printf_lft(yfms, 8, 0, COLR_IDX_BLUE, "%03d", autopilot_source == 1 ? nav2_obs_deg_mag_copilot : nav2_obs_deg_mag_pilot);
            yfs_printf_lft(yfms, 6, 0, COLR_IDX_BLUE,    "%4s/%06.2lf", nav2_nav_id[0] ? nav2_nav_id : " [ ]", nav2_frequency_hz / 100.);
            ils_in_use = 1;
        }
    }
    else                                                                        // Generic plane
    {                                                                           // w/NAV1 master
        if (navaid_is_ils(nav1_type))
        {
            yfs_printf_lft(yfms, 8, 0, COLR_IDX_BLUE, "%03d", autopilot_source == 1 ? nav1_obs_deg_mag_copilot : nav1_obs_deg_mag_pilot);
            yfs_printf_lft(yfms, 6, 0, COLR_IDX_BLUE,    "%4s/%06.2lf", nav1_nav_id[0] ? nav1_nav_id : " [ ]", nav1_frequency_hz / 100.);
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
        int nav2_obs_deg_mag = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.fb76.nav2_obs_deg_mag_pilot), 360.));
        nav2_nav_id[XPLMGetDatab(yfms->xpl.fb76.nav2_nav_id, nav2_nav_id, 0, sizeof(nav2_nav_id) - 1)] = 0;
        nav2_frequency_hz = XPLMGetDatai(yfms->xpl.fb76.nav2_frequency_hz);
        if (nav2_obs_deg_mag < 1)
        {
            nav2_obs_deg_mag = 360;
        }
        nav2_obs_deg_mag_copilot = nav2_obs_deg_mag_pilot = nav2_obs_deg_mag;
    }
    yfs_printf_lft(yfms,  2, 0, COLR_IDX_BLUE,    "%4s/%06.2lf", nav1_nav_id[0] ? nav1_nav_id : " [ ]", nav1_frequency_hz / 100.);
    yfs_printf_rgt(yfms,  2, 0, COLR_IDX_BLUE,   "%06.2lf/%-4s", nav2_frequency_hz / 100., nav2_nav_id[0] ? nav2_nav_id : "[ ] ");
    yfs_printf_lft(yfms,  4, 0, COLR_IDX_BLUE, "%03d", autopilot_source == 1 ? nav1_obs_deg_mag_copilot : nav1_obs_deg_mag_pilot);
    yfs_printf_rgt(yfms,  4, 0, COLR_IDX_BLUE, "%03d", autopilot_source == 1 ? nav2_obs_deg_mag_copilot : nav2_obs_deg_mag_pilot);

    /* line 10: ADF frequencies (blue) */
    yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE,  "%4s/%03d", adf1_nav_id[0] ? adf1_nav_id : " [ ]", adf1_frequency_hz);
    yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE, "%03d/%-4s", adf2_frequency_hz, adf2_nav_id[0] ? adf2_nav_id : "[ ] ");

    /* all good */
    yfms->mwindow.screen.redraw = 1; return;
}

static void yfs_lsk_callback_rad1(yfms_context *yfms, int key[2], intptr_t refcon)
{
    if (yfms == NULL || yfms->mwindow.current_page != PAGE_RAD1)
    {
        return; // callback not applicable to current page
    }
    if (key[1] == 0 && (yfms->xpl.atyp == YFS_ATYP_TOLI || yfms->xpl.atyp == YFS_ATYP_QPAC))
    {
        int new_activ_com_frequency_hz_833;
        switch (key[0])
        {
            case 0:
                XPLMCommandOnce(yfms->xpl.qpac.VHF1Capt);
                new_activ_com_frequency_hz_833 = yfms->xpl.qpac.com1_sby_hz_833;
                yfms->xpl.qpac.com1_sby_hz_833 = XPLMGetDatai(yfms->xpl.com1_left_frequency_hz_833);
                XPLMSetDatai (yfms->xpl.com1_left_frequency_hz_833, new_activ_com_frequency_hz_833);
                break;
            default:
                XPLMCommandOnce(yfms->xpl.qpac.VHF2Co);
                new_activ_com_frequency_hz_833 = yfms->xpl.qpac.com2_sby_hz_833;
                yfms->xpl.qpac.com2_sby_hz_833 = XPLMGetDatai(yfms->xpl.com2_left_frequency_hz_833);
                XPLMSetDatai (yfms->xpl.com2_left_frequency_hz_833, new_activ_com_frequency_hz_833);
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
            if (yfms->xpl.atyp == YFS_ATYP_TOLI || yfms->xpl.atyp == YFS_ATYP_QPAC)
            {
                yfs_spad_clear(yfms);
                yfms->xpl.qpac.com1_sby_hz_833 = hz8;
                return yfs_rad1_pageupdt(yfms);
            }
            XPLMSetDatai(yfms->xpl.com1_standby_frequency_Mhz, mhz);
            XPLMSetDatai(yfms->xpl.com1_standby_frequency_khz, khz);
            yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
        }
        if (key[0] == 1)
        {
            if (yfms->xpl.atyp == YFS_ATYP_TOLI || yfms->xpl.atyp == YFS_ATYP_QPAC)
            {
                yfs_spad_clear(yfms);
                yfms->xpl.qpac.com2_sby_hz_833 = hz8;
                return yfs_rad1_pageupdt(yfms);
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
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        else
        {
            set_transponder_mode(yfms, XPDRCYCL);
            yfs_rad1_pageupdt(yfms); return;
        }
    }
    if (key[0] == 0 && key[1] == 4)
    {
        char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if  (buf[0] == 0)
        {
            int alt[3]; get_altimeter(yfms, alt);
            if (alt[2] == BARO_STD)
            {
                // STD, toggle out of it
                int toggle[2] = { -1, -1, };
                set_altimeter(yfms, toggle);
                yfs_rad1_pageupdt(yfms); return;
            }
            switch (alt[1])
            {
                case BARO_HPA: // hectoPascals
                    snprintf(buf, sizeof(buf), "%04.0f", (float)alt[0]);
                    break;
                default: // inches of mercury
                    snprintf(buf, sizeof(buf), "%05.2f", (float)alt[0] / 100.0f);
                    break;
            }
            yfs_spad_reset(yfms, buf, -1); return; // current baro to scratchpad
        }
        int baro[2];
        {
            if (get_baro_pressure(buf, baro))
            {
                yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
            }
        }
        set_altimeter(yfms, baro); yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
    }
    if (key[0] == 1 && key[1] == 4) // toggle pressure unit (tog[1] == -1)
    {
        int tog[2], al[3]; get_altimeter(yfms, al); tog[0] = al[0]; tog[1] = -1;
        set_altimeter(yfms, tog); yfs_rad1_pageupdt(yfms); return;
    }
    if (key[0] == 0 && key[1] == 5)
    {
        if (standard_pressure(yfms) == 0) // not STD, toggle into it
        {
            int altstd[3]; get_altimeter(yfms, altstd);
            int hpa=altstd[1]==BARO_HPA;
            altstd[0]=hpa ? 1013 : 2992;
            set_altimeter(yfms, altstd);
            int toggle[2] = { -1, -1, };
            set_altimeter(yfms, toggle);
            return yfs_rad1_pageupdt(yfms);
        }
        return;
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

static void yfs_msw_callback_rad1(yfms_context *yfms, int rx, int ry, int delta)
{
    //fixme assign some zone to increase/descrease ATC volume
    //fixme feedback via scratchpad reset with the new volume
    //fixme adjust up and down by .05f per scroll wheel notch
    if (rx >= yfms->mouse_regions[4][0].xmin && rx <= yfms->mouse_regions[4][0].xmax &&
        ry >= yfms->mouse_regions[4][0].ymin && ry <= yfms->mouse_regions[4][0].ymax)
    {
        // LSK 4 L: barometric pressure
        int alt[3]; get_altimeter(yfms, alt);
        if (alt[2] == BARO_STD)
        {
            // STD, toggle out of it
            int toggle[2] = { -1, -1, };
            set_altimeter(yfms, toggle);
            yfs_rad1_pageupdt(yfms); return;
        }
        if (alt[1] == BARO_HPA && delta < 0 && standard_pressure(yfms))
        {
            // alt[0] == 1013 but actual pressure is ~1013.2
            // make it so we don't skip 1013 when going down
            int new[2]; new[1] = alt[1]; alt[0] = 1014 + delta;
            set_altimeter(yfms, new); yfs_rad1_pageupdt(yfms); return;
        }
        int new[2]; new[1] = alt[1]; new[0] = alt[0] + delta;
        set_altimeter(yfms, new); yfs_rad1_pageupdt(yfms); return;
    }
    if (rx >= yfms->mouse_regions[3][2].xmin && rx <= yfms->mouse_regions[3][2].xmax &&
        ry >= yfms->mouse_regions[3][2].ymin && ry <= yfms->mouse_regions[3][2].ymax)
    {
        /*
         * LSK 3 R: transponder mode
         *
         * Notes:
         *
         * - a new mode may not activate right away, so we can't check
         *   whether it applied correctly here and try again
         *
         * - some modes don't apply to all aircraft, thus we can't just set
         *   the transponder to mode + 1: it may simply be mapped to mode,
         *   in which case the increment would have no effect; instead we
         *   must be smart: increment/decrement to a mode that's unlikely
         *   to be mapped to the current mode; however it's not foolproof
         */
        if (delta < 0)
        {
            int mode = get_transponder_mode(yfms);
            if (mode >= XPDR_TAR)
            {
                set_transponder_mode(yfms, XPDR_TAO);
                return;
            }
            if (mode >= XPDR_TAO)
            {
                set_transponder_mode(yfms, XPDR_ALT);
                return;
            }
            if (mode >= XPDR_ALT)
            {
                if (has_transponder_mode(yfms, XPDR_AUT))
                {
                    set_transponder_mode(yfms, XPDR_AUT);
                    return;
                }
                set_transponder_mode(yfms, XPDR_SBY);
                return;
            }
            if (mode >= XPDR_GND)
            {
                if (has_transponder_mode(yfms, XPDR_AUT))
                {
                    set_transponder_mode(yfms, XPDR_AUT);
                    return;
                }
                set_transponder_mode(yfms, XPDR_SBY);
                return;
            }
            if (mode >= XPDR_AUT)
            {
                set_transponder_mode(yfms, XPDR_SBY);
                return;
            }
//            // quite impractical
//            if (mode >= XPDR_SBY)
//            {
//                set_transponder_mode(yfms, XPDR_OFF);
//                return;
//            }
            return;
        }
        if (delta > 0)
        {
            int mode = get_transponder_mode(yfms);
            if (mode >= XPDR_TAR)
            {
                return;
            }
            if (mode >= XPDR_TAO)
            {
                set_transponder_mode(yfms, XPDR_TAR);
                return;
            }
            if (mode >= XPDR_ALT)
            {
                set_transponder_mode(yfms, XPDR_TAO);
                return;
            }
            if (mode >= XPDR_GND)
            {
                set_transponder_mode(yfms, XPDR_ALT);
                return;
            }
            if (mode >= XPDR_AUT)
            {
                set_transponder_mode(yfms, XPDR_ALT);
                return;
            }
            if (mode >= XPDR_SBY)
            {
                if (has_transponder_mode(yfms, XPDR_AUT))
                {
                    set_transponder_mode(yfms, XPDR_AUT);
                    return;
                }
                set_transponder_mode(yfms, XPDR_ALT);
                return;
            }
            if (mode >= XPDR_OFF)
            {
                set_transponder_mode(yfms, XPDR_SBY);
                return;
            }
            return;
        }
        return;
    }
    return;
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
        if ((freq = get_frequency4idt(buf, yfms, YVP_NAV_VOR|YVP_NAV_LOC|YVP_NAV_DME)) < 0.)
        {
            if ((freq != -2.))
            {
                return yfs_spad_reset(yfms, "NOT IN DATA BASE", -1);
            }
            if ((freq = get_nav_frequency(buf)) < 0.)
            {
                return yfs_spad_reset(yfms, "FORMAT ERROR", -1);
            }
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
            // X-Plane may overwrite the HSI course even if the OBS we've
            // updated isn't the current HSI source, we simply restore it
            if ((XPLMGetDatai(yfms->xpl.autopilot_source) == 0 && XPLMGetDatai(yfms->xpl.  HSI_source_select_pilot) != 0) ||
                (XPLMGetDatai(yfms->xpl.autopilot_source) == 1 && XPLMGetDatai(yfms->xpl.HSI_source_select_copilot) != 0))
            {
                yfms->data.rdio.hsi_obs_deg_mag_rigt = XPLMGetDataf(yfms->xpl.hsi_obs_deg_mag_copilot);
                yfms->data.rdio.hsi_obs_deg_mag_left = XPLMGetDataf(yfms->xpl.hsi_obs_deg_mag_pilot);
                yfms->data.rdio.hsi_obs_deg_mag_rest = 2;
            }
            XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_copilot, (float)crs);
            XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_pilot, (float)crs);
        }
        else if (yfms->xpl.atyp == YFS_ATYP_FB76)
        {
            XPLMSetDataf(yfms->xpl.fb76.nav2_obs_deg_mag_pilot, (float)crs);
        }
        else
        {
            // X-Plane may overwrite the HSI course even if the OBS we've
            // updated isn't the current HSI source, we simply restore it
            if ((XPLMGetDatai(yfms->xpl.autopilot_source) == 0 && XPLMGetDatai(yfms->xpl.  HSI_source_select_pilot) != 1) ||
                (XPLMGetDatai(yfms->xpl.autopilot_source) == 1 && XPLMGetDatai(yfms->xpl.HSI_source_select_copilot) != 1))
            {
                yfms->data.rdio.hsi_obs_deg_mag_rigt = XPLMGetDataf(yfms->xpl.hsi_obs_deg_mag_copilot);
                yfms->data.rdio.hsi_obs_deg_mag_left = XPLMGetDataf(yfms->xpl.hsi_obs_deg_mag_pilot);
                yfms->data.rdio.hsi_obs_deg_mag_rest = 2;
            }
            XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_copilot, (float)crs);
            XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_pilot, (float)crs);
        }
        yfs_spad_clear(yfms); return yfs_rad2_pageupdt(yfms);
    }
    if (key[0] == 0 && key[1] == 2) // ILS 1 frequency get/set
    {
        double freq; char buf[YFS_ROW_BUF_SIZE]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            if (yfms->mwindow.screen.text[6][5] != '1')
            {
                return; // not an ILS frequency
            }
            strncpy(&buf[0], &yfms->mwindow.screen.text[6][5], 6); buf[6] = 0;
            yfs_spad_reset(yfms, buf, -1); return; // frequency to scratchpad
        }
        if ((freq = get_frequency4idt(buf, yfms, YVP_NAV_VOR|YVP_NAV_LOC|YVP_NAV_DME)) < 0.)
        {
            if ((freq != -2.))
            {
                return yfs_spad_reset(yfms, "NOT IN DATA BASE", -1);
            }
            if ((freq = get_nav_frequency(buf)) < 0.)
            {
                return yfs_spad_reset(yfms, "FORMAT ERROR", -1);
            }
            yfs_spad_clear(yfms);
            yfs_rdio_ils_data(yfms, freq, -1, 1|2);
            return yfs_rad2_pageupdt(yfms);
        }
        yfs_spad_clear(yfms);
        yfs_rdio_ils_data(yfms, freq, -1, 1|2);
        return yfs_rad2_pageupdt(yfms);
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
        yfs_spad_clear(yfms);
        yfs_rdio_ils_data(yfms, -1., crs, 1|2);
        return yfs_rad2_pageupdt(yfms);
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
        if ((freq = get_frequency4idt(buf, yfms, YVP_NAV_NDB)) < 0.)
        {
            if ((freq != -2.))
            {
                return yfs_spad_reset(yfms, "NOT IN DATA BASE", -1);
            }
            if ((freq = get_nav_frequency(buf)) < 0.)
            {
                return yfs_spad_reset(yfms, "FORMAT ERROR", -1);
            }
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
