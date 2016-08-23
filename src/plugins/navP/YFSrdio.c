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
#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMUtilities.h"

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
    while (999.999 < freq) // no decimal separator provided
    {
        freq /= 10.;
    }
    if (freq < 118.000 || freq > 136.999) // valid range 118.* -> 136.* Mhz
    {
        return -1.;
    }
    else
    {
        /*
         * For com. radios, the "tick" is 8.33 kHz. For frequencies that don't
         * divide cleanly by our tick, always use the next (i.e. higher) tick.
         */
        freq = ceil(freq * 120. + YVP_CEIL_DBL) / 120.;
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
    while (999.999 < freq) // no decimal separator provided
    {
        freq /= 10.;
    }
    if (freq < 108.000 || freq > 117.999) // valid range 108.* -> 117.* Mhz
    {
        return -1.;
    }
    else
    {
        /*
         * For nav. radios, the "tick" is 25.0 kHz. For frequencies that don't
         * divide cleanly by our tick, always use the next (i.e. higher) tick.
         */
        freq = ceil(freq * 40. + YVP_CEIL_DBL) / 40.;
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
    while (1750. < freq) // no decimal separator provided
    {
        freq /= 10.;
    }
    if (freq < 175. || freq > 1750.) // valid range 175 -> 1750 kHz
    {
        return -1.;
    }
    else
    {
        /*
         * For such radios, the "tick" is 0.5 kHz. For frequencies that don't
         * divide cleanly by our tick, always use previous (i.e. lower) tick.
         */
        freq = floor(freq * 2. + YVP_FLOORDBL) / 2.;
    }
    /* For X-Plane purposes, we must floor the frequency to 1.0 kHz */
    return (floor(freq + YVP_FLOORDBL));
}

//fixme get_baro_pressure(str), converts anything in the range ]40; 2000[ to hPa, and everything else to InHg (handle the case of InHg w/out separator, too)
static double get_baro_pressure(const char *string);

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

static void yfs_rad1_pageupdt(yfms_context *yfms)
{
    /* reset lines before drawing */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }

    /* row buffer */
    char buf[YFS_DISPLAY_NUMC + 1];

    /* line 0: main header (white, centered) */
    yfs_printf_ctr(yfms,  0,    COLR_IDX_WHITE, "ATC COMM");

    /* line 1: headers (white) */
    yfs_printf_lft(yfms,  1, 0, COLR_IDX_WHITE, "COM 1");
    yfs_printf_rgt(yfms,  1, 0, COLR_IDX_WHITE, "COM 2");

    /* line 2: active frequencies (green) */
    snprintf(buf, sizeof(buf), "%03d.%03d", XPLMGetDatai(yfms->xpl.com1_frequency_Mhz),         XPLMGetDatai(yfms->xpl.com1_frequency_khz));
    yfs_printf_lft(yfms,  2, 0, COLR_IDX_GREEN, buf);
    snprintf(buf, sizeof(buf), "%03d.%03d", XPLMGetDatai(yfms->xpl.com2_frequency_Mhz),         XPLMGetDatai(yfms->xpl.com2_frequency_khz));
    yfs_printf_rgt(yfms,  2, 0, COLR_IDX_GREEN, buf);

    /* line 3: headers (white) */
    yfs_printf_lft(yfms,  3, 0, COLR_IDX_WHITE, "STANDBY");
    yfs_printf_rgt(yfms,  3, 0, COLR_IDX_WHITE, "STANDBY");

    /* line 4: standby frequencies (blue) */
    snprintf(buf, sizeof(buf), "%03d.%03d", XPLMGetDatai(yfms->xpl.com1_standby_frequency_Mhz), XPLMGetDatai(yfms->xpl.com1_standby_frequency_khz));
    yfs_printf_lft(yfms,  4, 0, COLR_IDX_BLUE,  buf);
    snprintf(buf, sizeof(buf), "%03d.%03d", XPLMGetDatai(yfms->xpl.com2_standby_frequency_Mhz), XPLMGetDatai(yfms->xpl.com2_standby_frequency_khz));
    yfs_printf_rgt(yfms,  4, 0, COLR_IDX_BLUE,  buf);

    /* line 7: headers (white) */
    yfs_printf_ctr(yfms,  7,    COLR_IDX_WHITE, "XPDR");
    yfs_printf_lft(yfms,  7, 0, COLR_IDX_WHITE, "CODE");
    yfs_printf_rgt(yfms,  7, 0, COLR_IDX_WHITE, "MODE");

    /* line 8: XPDR information */
    if (yfms->xpl.atyp == YFS_ATYP_IXEG)
    {
        if ((int)roundf(XPLMGetDataf(yfms->xpl.ixeg.xpdr_mode_act)) == 0)
        {
            yfs_printf_lft(yfms,  8, 0, COLR_IDX_WHITE,  "----");
            yfs_printf_rgt(yfms,  8, 0, COLR_IDX_WHITE,   "OFF");
        }
        else switch ((int)roundf(XPLMGetDataf(yfms->xpl.ixeg.xpdr_stby_act)))
        {
            case 0:
                snprintf(buf, sizeof(buf), "%04d", XPLMGetDatai(yfms->xpl.transponder_code));
                yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,      buf);
                yfs_printf_rgt(yfms,  8, 0, COLR_IDX_BLUE,   "STBY");
                break;
            case 2:
                snprintf(buf, sizeof(buf), "%04d", XPLMGetDatai(yfms->xpl.transponder_code));
                yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,      buf);
                yfs_printf_rgt(yfms,  8, 0, COLR_IDX_BLUE,  "TA/RA");
                break;
            default:
                snprintf(buf, sizeof(buf), "%04d", XPLMGetDatai(yfms->xpl.transponder_code));
                yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,      buf);
                yfs_printf_rgt(yfms,  8, 0, COLR_IDX_BLUE,   "AUTO");
                break;
        }
    }
    else if (yfms->xpl.atyp == YFS_ATYP_QPAC)
    {
        switch (XPLMGetDatai(yfms->xpl.qpac.XPDRPower))
        {
            case 0:
                snprintf(buf, sizeof(buf), "%04d", XPLMGetDatai(yfms->xpl.transponder_code));
                yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,      buf);
                yfs_printf_rgt(yfms,  8, 0, COLR_IDX_BLUE,   "STBY");
                break;
            case 2:
                snprintf(buf, sizeof(buf), "%04d", XPLMGetDatai(yfms->xpl.transponder_code));
                yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,      buf);
                yfs_printf_rgt(yfms,  8, 0, COLR_IDX_BLUE,  "TA/RA");
                break;
            default:
                snprintf(buf, sizeof(buf), "%04d", XPLMGetDatai(yfms->xpl.transponder_code));
                yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,      buf);
                yfs_printf_rgt(yfms,  8, 0, COLR_IDX_BLUE,   "AUTO");
                break;
        }
    }
    else switch (XPLMGetDatai(yfms->xpl.transponder_mode))
    {
        case 0:
            yfs_printf_lft(yfms,  8, 0, COLR_IDX_WHITE,  "----");
            yfs_printf_rgt(yfms,  8, 0, COLR_IDX_WHITE,   "OFF");
            break;
        case 1:
            snprintf(buf, sizeof(buf), "%04d", XPLMGetDatai(yfms->xpl.transponder_code));
            yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,      buf);
            yfs_printf_rgt(yfms,  8, 0, COLR_IDX_BLUE,   "STBY");
            break;
        default:
            snprintf(buf, sizeof(buf), "%04d", XPLMGetDatai(yfms->xpl.transponder_code));
            yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,      buf);
            yfs_printf_rgt(yfms,  8, 0, COLR_IDX_BLUE,  "TA/RA");
            break;
    }

    /* line 9: headers (white) */
    yfs_printf_ctr(yfms,  9,    COLR_IDX_WHITE, "PRESSURE");
    yfs_printf_lft(yfms,  9, 0, COLR_IDX_WHITE, "BARO");
    yfs_printf_rgt(yfms,  9, 0, COLR_IDX_WHITE, "UNIT");

    /* line 10: barometric altimeter information */
    float alt, alt_inhg = XPLMGetDataf(yfms->xpl.barometer_setting_in_hg_pilot);
    switch (yfms->ndt.alt.unit)
    {
        case 1: // hectoPascals
            alt = roundf(alt_inhg * 33.86389f);
            snprintf(buf, sizeof(buf), "%04.0f", alt);
            yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE,  "hPa");
            break;
        default: // inches of mercury
            alt = roundf(alt_inhg * 100.0f);
            snprintf(buf, sizeof(buf), "%05.2f", alt / 100.0f);
            yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE, "InHg");
            break;
    }
    if ((yfms->xpl.atyp == YFS_ATYP_QPAC && XPLMGetDatai(yfms->xpl.qpac.BaroStdCapt)) ||
        (yfms->xpl.atyp == YFS_ATYP_Q380 && XPLMGetDatai(yfms->xpl.q380.BaroStdCapt)) ||
        (yfms->xpl.atyp == YFS_ATYP_Q350 && XPLMGetDatai(yfms->xpl.q350.pressLeftButton)))
    {
        yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE, "STD");
    }
    else
    {
        yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE, buf);
    }

    /* line 12: switches (white) */
    if (yfms->xpl.atyp == YFS_ATYP_QPAC)
    {
        if (!XPLMGetDatai(yfms->xpl.qpac.BaroStdCapt))
        {
            yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "<ALT STD");
            yfms->lsks[0][5].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
        }
        else
        {
            yfms->lsks[0][5].cback = (YFS_LSK_f)NULL;
        }
    }
    else if (yfms->xpl.atyp == YFS_ATYP_Q350)
    {
        if (!XPLMGetDatai(yfms->xpl.q350.pressLeftButton))
        {
            yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "<ALT STD");
            yfms->lsks[0][5].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
        }
        else
        {
            yfms->lsks[0][5].cback = (YFS_LSK_f)NULL;
        }
    }
    else if (yfms->xpl.atyp == YFS_ATYP_Q380)
    {
        if (!XPLMGetDatai(yfms->xpl.q380.BaroStdCapt))
        {
            yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "<ALT STD");
            yfms->lsks[0][5].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
        }
        else
        {
            yfms->lsks[0][5].cback = (YFS_LSK_f)NULL;
        }
    }
    else if (BPRESS_IS_STD(alt_inhg) == 0)
    {
        yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "<ALT STD");
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
                yfs_printf_rgt(yfms, 12, 0, COLR_IDX_GREEN,  "IDENT");
                break;
            default:
                yfs_printf_rgt(yfms, 12, 0, COLR_IDX_WHITE, "XPDR ID>");
                break;
        }
    }
}

static void yfs_rad2_pageupdt(yfms_context *yfms)
{
    /* auto-select ILS course after user-requested frequency change */
    if (yfms->xpl.ils.frequency_changed && NAVTYP_IS_ILS(XPLMGetDatai(yfms->xpl.nav2_type)))
    {
        float ils_crs = XPLMGetDataf(yfms->xpl.nav2_course_deg_mag_pilot);
        XPLMSetDataf        (yfms->xpl.nav2_obs_deg_mag_pilot,   ils_crs);
        XPLMSetDataf        (yfms->xpl.nav2_obs_deg_mag_copilot, ils_crs);
        if (yfms->xpl.atyp != YFS_ATYP_FB76)
        {
            XPLMSetDataf    (yfms->xpl.nav1_obs_deg_mag_pilot,   ils_crs);
            XPLMSetDataf    (yfms->xpl.nav1_obs_deg_mag_copilot, ils_crs);
        }
    }
    yfms->xpl.ils.frequency_changed = 0;

    /* reset lines before drawing */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }

    /* buffers */
    char buf1[YFS_DISPLAY_NUMC + 1], buf2[YFS_DISPLAY_NUMC + 1];
    char nav1_nav_id[5];
    char nav2_nav_id[5];
    char adf1_nav_id[4];
    char adf2_nav_id[4];

    /* relevant data */
    int nav1_course_deg_mag_pilot = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav1_course_deg_mag_pilot), 360.));
    int nav2_course_deg_mag_pilot = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav2_course_deg_mag_pilot), 360.));
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
    if (yfms->xpl.atyp == YFS_ATYP_FB76)
    {
        nav2_obs_deg_mag_copilot = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.fb76.nav2_obs_deg_mag_pilot), 360.));
        nav2_nav_id[XPLMGetDatab(yfms->xpl.fb76.nav2_nav_id, nav2_nav_id, 0, sizeof(nav2_nav_id) - 1)] = 0;
        nav2_frequency_hz = XPLMGetDatai(yfms->xpl.fb76.nav2_frequency_hz);
    }
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

    /* line 0: main header (white, centered) */
    yfs_printf_ctr(yfms,  0,    COLR_IDX_WHITE, "RADIO NAV");

    /* line 1: headers (white) */
    yfs_printf_lft(yfms,  1, 0, COLR_IDX_WHITE, "VOR1/FREQ");
    yfs_printf_rgt(yfms,  1, 0, COLR_IDX_WHITE, "FREQ/VOR2");

    /* line 3: headers (white) */
    yfs_printf_lft(yfms,  3, 0, COLR_IDX_WHITE, "CRS");
    yfs_printf_rgt(yfms,  3, 0, COLR_IDX_WHITE, "CRS");

    /* line 5: headers (white) */
    yfs_printf_lft(yfms,  5, 0, COLR_IDX_WHITE, "ILS1/FREQ");
//  yfs_printf_rgt(yfms,  5, 0, COLR_IDX_WHITE, "CHANN/MLS");

    /* line 7: headers (white) */
    yfs_printf_lft(yfms,  7, 0, COLR_IDX_WHITE, "CRS");
//  yfs_printf_rgt(yfms,  7, 0, COLR_IDX_WHITE, "SLOPE/CRS");

    /* line 9: headers (white) */
    yfs_printf_lft(yfms,  9, 0, COLR_IDX_WHITE, "ADF1/FREQ");
    yfs_printf_rgt(yfms,  9, 0, COLR_IDX_WHITE, "FREQ/ADF2");

    /* line 2: frequencies (blue) */
    sprintf(buf1,  "%4s/%06.2lf", strnlen(nav1_nav_id, 1) ? nav1_nav_id : " [ ]", nav1_frequency_hz / 100.);
    sprintf(buf2, "%06.2lf/%-4s", nav2_frequency_hz / 100., strnlen(nav2_nav_id, 1) ? nav2_nav_id : "[ ] ");
    yfs_printf_lft(yfms,  2, 0, COLR_IDX_BLUE,  buf1);
    yfs_printf_rgt(yfms,  2, 0, COLR_IDX_BLUE,  buf2);

    /* line 4: courses (blue) */
    sprintf(buf1, "%03d",     nav1_obs_deg_mag_pilot);
    sprintf(buf2, "%03d",   nav2_obs_deg_mag_copilot);
    yfs_printf_lft(yfms,  4, 0, COLR_IDX_BLUE,  buf1);
    yfs_printf_rgt(yfms,  4, 0, COLR_IDX_BLUE,  buf2);

    /*
     * lines 6, 8: frequencies & associated courses (blue)
     *
     * autopilot 1 (copilot), HSI source 1 (nav2) => nav2 master
     * autopilot 1 (copilot), HSI source ? (nav1) => nav1 master
     * autopilot ? (> pilot), HSI source 1 (nav2) => nav2 master
     * autopilot ? (> pilot), HSI source ? (nav1) => nav1 master
     * only type 8 means ILS…
     */
    if (yfms->xpl.atyp == YFS_ATYP_FB76) // ILS is X-Plane default NAV 2
    {
        nav2_obs_deg_mag_copilot = (int)round(ndt_mod((double)XPLMGetDataf(yfms->xpl.nav2_obs_deg_mag_pilot), 360.));
        nav2_nav_id[XPLMGetDatab(yfms->xpl.nav2_nav_id, nav2_nav_id, 0, sizeof(nav2_nav_id) - 1)] = 0;
        nav2_frequency_hz = XPLMGetDatai(yfms->xpl.nav2_frequency_hz);
        if (nav2_obs_deg_mag_copilot < 1)
        {
            nav2_obs_deg_mag_copilot = 360;
        }
    }
    if (((yfms->xpl.atyp != YFS_ATYP_FB76)) &&
        ((autopilot_source == 1 && HSI_source_select_copilot == 1 && NAVTYP_IS_ILS(nav2_type)) ||
         (autopilot_source != 1 && HSI_source_select_pilot   == 1 && NAVTYP_IS_ILS(nav2_type))))
    {
        sprintf(buf1, "%4s/%06.2lf", strnlen(nav2_nav_id, 1) ? nav2_nav_id : " [ ]", nav2_frequency_hz / 100.);
        sprintf(buf2, "%03d",   nav2_course_deg_mag_pilot);
        yfs_printf_lft(yfms,  6, 0, COLR_IDX_BLUE,   buf1);
        yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,   buf2);
    }
    else if (yfms->xpl.atyp != YFS_ATYP_FB76 && NAVTYP_IS_ILS(nav1_type))
    {
        sprintf(buf1, "%4s/%06.2lf", strnlen(nav1_nav_id, 1) ? nav1_nav_id : " [ ]", nav1_frequency_hz / 100.);
        sprintf(buf2, "%03d",   nav1_course_deg_mag_pilot);
        yfs_printf_lft(yfms,  6, 0, COLR_IDX_BLUE,   buf1);
        yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,   buf2);
    }
    else if (yfms->xpl.atyp == YFS_ATYP_FB76 && nav2_frequency_hz >= 10800 &&
             HSI_source_select_copilot  == 2 && HSI_source_select_pilot == 2)
    {
        sprintf(buf1, "%4s/%06.2lf", strnlen(nav2_nav_id, 1) ? nav2_nav_id : " [ ]", nav2_frequency_hz / 100.);
        sprintf(buf2, "%03d",    nav2_obs_deg_mag_copilot);
        yfs_printf_lft(yfms,  6, 0, COLR_IDX_BLUE,   buf1);
        yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,   buf2);
    }
    else
    {
        yfs_printf_lft(yfms,  6, 0, COLR_IDX_BLUE,  " [ ]/[ ] ");
        yfs_printf_lft(yfms,  8, 0, COLR_IDX_BLUE,       " [ ]");
    }
//  yfs_printf_rgt(yfms,  6, 0, COLR_IDX_WHITE, "---/---");
//  yfs_printf_rgt(yfms,  8, 0, COLR_IDX_WHITE, "-.-/---");

    /*
     * line 10: frequencies (blue)
     * don't display ADF frequencies if outside 175-999 kHz range…
     */
    sprintf(buf1,  "%4s/%03d", strnlen(adf1_nav_id, 1) ? adf1_nav_id : " [ ]", adf1_frequency_hz);
    sprintf(buf2, "%03d/%-4s", adf2_frequency_hz, strnlen(adf2_nav_id, 1) ? adf2_nav_id : "[ ] ");
    yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE,  " [ ]/[ ] ");
    yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE,  " [ ]/[ ] ");
    if (adf1_frequency_hz >= 175 && adf1_frequency_hz <= 999)
    {
        yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE,  buf1);
    }
    if (adf2_frequency_hz >= 175 && adf2_frequency_hz <= 999)
    {
        yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE,  buf2);
    }

    /* all good */
    return;
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
        XPLMCommandOnce(yfms->xpl.com1_standy_flip); yfs_rad1_pageupdt(yfms); return;
    }
    if (key[0] == 1 && key[1] == 0)
    {
        XPLMCommandOnce(yfms->xpl.com2_standy_flip); yfs_rad1_pageupdt(yfms); return;
    }
    if (key[1] == 1)
    {
        int mhz, khz; double freq; char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
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
            mhz = (int)round((freq));
            khz = (int)round((freq - (double)mhz) * 1000.);
        }
        if (key[0] == 0)
        {
            if (yfms->xpl.atyp == YFS_ATYP_QPAC)
            {
                XPLMCommandOnce(yfms->xpl.qpac.VHF1Capt);
                khz = khz / 25 * 25; // QPAC radios only have 25 kHz precision
                int qpac_mhz = XPLMGetDatai(yfms->xpl.com1_standby_frequency_Mhz);
                int qpac_khz = XPLMGetDatai(yfms->xpl.com1_standby_frequency_khz);
                if (qpac_mhz < mhz)
                {
                    for (int i = 0; i < mhz - qpac_mhz; i++)
                    {
                        XPLMCommandOnce(yfms->xpl.qpac.RMP1FreqUpLrg);
                    }
                }
                if (qpac_mhz > mhz)
                {
                    for (int i = 0; i < qpac_mhz - mhz; i++)
                    {
                        XPLMCommandOnce(yfms->xpl.qpac.RMP1FreqDownLrg);
                    }
                }
                if (qpac_khz < khz)
                {
                    for (int i = 0; i < khz - qpac_khz; i += 25)
                    {
                        XPLMCommandOnce(yfms->xpl.qpac.RMP1FreqUpSml);
                    }
                }
                if (qpac_khz > khz)
                {
                    for (int i = 0; i < qpac_khz - khz; i += 25)
                    {
                        XPLMCommandOnce(yfms->xpl.qpac.RMP1FreqDownSml);
                    }
                }
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
                khz = khz / 25 * 25; // QPAC radios only have 25 kHz precision
                int qpac_mhz = XPLMGetDatai(yfms->xpl.com2_standby_frequency_Mhz);
                int qpac_khz = XPLMGetDatai(yfms->xpl.com2_standby_frequency_khz);
                if (qpac_mhz < mhz)
                {
                    for (int i = 0; i < mhz - qpac_mhz; i++)
                    {
                        XPLMCommandOnce(yfms->xpl.qpac.RMP2FreqUpLrg);
                    }
                }
                if (qpac_mhz > mhz)
                {
                    for (int i = 0; i < qpac_mhz - mhz; i++)
                    {
                        XPLMCommandOnce(yfms->xpl.qpac.RMP2FreqDownLrg);
                    }
                }
                if (qpac_khz < khz)
                {
                    for (int i = 0; i < khz - qpac_khz; i += 25)
                    {
                        XPLMCommandOnce(yfms->xpl.qpac.RMP2FreqUpSml);
                    }
                }
                if (qpac_khz > khz)
                {
                    for (int i = 0; i < qpac_khz - khz; i += 25)
                    {
                        XPLMCommandOnce(yfms->xpl.qpac.RMP2FreqDownSml);
                    }
                }
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            XPLMSetDatai(yfms->xpl.com2_standby_frequency_Mhz, mhz);
            XPLMSetDatai(yfms->xpl.com2_standby_frequency_khz, khz);
            yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
        }
    }
    if (key[0] == 0 && key[1] == 3)
    {
        if (XPLMGetDatai(yfms->xpl.transponder_mode) <= 0)
        {
            return; // transponder off
        }
        char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            snprintf(buf, sizeof(buf), "%04d", XPLMGetDatai(yfms->xpl.transponder_code));
            yfs_spad_reset(yfms, buf, -1); return; // current code to scratchpad
        }
        if (strnlen(buf, 5) != 4)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        for (int i = 0; i < 4; i++)
        {
            if (buf[i] < '0' || buf[i] > '7')
            {
                yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
            }
        }
        XPLMSetDatai(yfms->xpl.transponder_code, atoi(buf));
        yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
    }
    if (key[0] == 1 && key[1] == 3)
    {
        char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
        if  (strnlen(buf, 1) && !yfms->mwindow.screen.spad_reset)
        {
            if (!strcmp(buf, "OFF"))
            {
                if (yfms->xpl.atyp == YFS_ATYP_IXEG)
                {
                    XPLMSetDataf(yfms->xpl.ixeg.xpdr_mode_act, 0.0f);
                    XPLMSetDataf(yfms->xpl.ixeg.xpdr_stby_act, 1.0f);
                    yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
                }
                XPLMSetDatai(yfms->xpl.transponder_mode, 0);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcmp(buf, "AUTO"))
            {
                if (yfms->xpl.atyp == YFS_ATYP_IXEG)
                {
                    XPLMSetDataf(yfms->xpl.ixeg.xpdr_mode_act, 2.0f);
                    XPLMSetDataf(yfms->xpl.ixeg.xpdr_stby_act, 1.0f);
                    yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
                }
                if (yfms->xpl.atyp == YFS_ATYP_QPAC)
                {
                    XPLMSetDatai(yfms->xpl.qpac.XPDRAltitude, 1);
                    XPLMSetDatai(yfms->xpl.qpac.XPDRPower,    1);
                    yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
                }
                sprintf(buf, "%s", "STBY"); // fall through
            }
            if (!strcmp(buf, "STBY"))
            {
                if (yfms->xpl.atyp == YFS_ATYP_IXEG)
                {
                    XPLMSetDataf(yfms->xpl.ixeg.xpdr_mode_act, 2.0f);
                    XPLMSetDataf(yfms->xpl.ixeg.xpdr_stby_act, 0.0f);
                    yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
                }
                if (yfms->xpl.atyp == YFS_ATYP_QPAC)
                {
                    XPLMSetDatai(yfms->xpl.qpac.XPDRPower,    0);
                    yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
                }
                if (yfms->xpl.atyp == YFS_ATYP_FB76)
                {
                    XPLMSetDataf(yfms->xpl.fb76.systemMode, 1.0f);
                    yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
                }
                if (yfms->xpl.atyp == YFS_ATYP_FB77)
                {
                    XPLMSetDatai(yfms->xpl.fb77.anim_85_switch, 0);
                    yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
                }
                XPLMSetDatai(yfms->xpl.transponder_mode, 1);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcmp(buf, "TA/RA"))
            {
                if (yfms->xpl.atyp == YFS_ATYP_IXEG)
                {
                    XPLMSetDataf(yfms->xpl.ixeg.xpdr_mode_act, 2.0f);
                    XPLMSetDataf(yfms->xpl.ixeg.xpdr_stby_act, 2.0f);
                    yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
                }
                if (yfms->xpl.atyp == YFS_ATYP_QPAC)
                {
                    XPLMSetDatai(yfms->xpl.qpac.XPDRAltitude, 1);
                    XPLMSetDatai(yfms->xpl.qpac.XPDRPower,    2);
                    yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
                }
                if (yfms->xpl.atyp == YFS_ATYP_FB76)
                {
                    XPLMSetDataf(yfms->xpl.fb76.systemMode, 5.0f);
                    yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
                }
                if (yfms->xpl.atyp == YFS_ATYP_FB77)
                {
                    XPLMSetDatai(yfms->xpl.fb77.anim_85_switch, 4);
                    yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
                }
                XPLMSetDatai(yfms->xpl.transponder_mode, 3);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        else if (yfms->xpl.atyp == YFS_ATYP_IXEG)
        {
            if ((int)roundf(XPLMGetDataf(yfms->xpl.ixeg.xpdr_mode_act)) == 0 ||
                (int)roundf(XPLMGetDataf(yfms->xpl.ixeg.xpdr_stby_act)) != 1)
            {
                XPLMSetDataf(yfms->xpl.ixeg.xpdr_mode_act, 2.0f);
                XPLMSetDataf(yfms->xpl.ixeg.xpdr_stby_act, 1.0f); // AUTO
            }
            else
            {
                XPLMSetDataf(yfms->xpl.ixeg.xpdr_mode_act, 2.0f);
                XPLMSetDataf(yfms->xpl.ixeg.xpdr_stby_act, 2.0f); // TA/RA
            }
            yfs_rad1_pageupdt(yfms); return;
        }
        else if (yfms->xpl.atyp == YFS_ATYP_QPAC)
        {
            if (XPLMGetDatai(yfms->xpl.qpac.XPDRPower) != 1)
            {
                XPLMSetDatai(yfms->xpl.qpac.XPDRAltitude, 1);
                XPLMSetDatai(yfms->xpl.qpac.XPDRPower,    1); // AUTO
            }
            else
            {
                XPLMSetDatai(yfms->xpl.qpac.XPDRAltitude, 1);
                XPLMSetDatai(yfms->xpl.qpac.XPDRPower,    2); // TA/RA
            }
            yfs_rad1_pageupdt(yfms); return;
        }
        else if (yfms->xpl.atyp == YFS_ATYP_FB76)
        {
            if ((int)roundf(XPLMGetDataf(yfms->xpl.fb76.systemMode)) != 1)
            {
                XPLMSetDataf(yfms->xpl.fb76.systemMode, 1.0f); // STBY
            }
            else
            {
                XPLMSetDataf(yfms->xpl.fb76.systemMode, 5.0f); // TA/RA
            }
            yfs_rad1_pageupdt(yfms); return;
        }
        else if (yfms->xpl.atyp == YFS_ATYP_FB77)
        {
            if (XPLMGetDatai(yfms->xpl.fb77.anim_85_switch) != 0)
            {
                XPLMSetDatai(yfms->xpl.fb77.anim_85_switch, 0); // STBY
            }
            else
            {
                XPLMSetDatai(yfms->xpl.fb77.anim_85_switch, 4); // TA/RA
            }
            yfs_rad1_pageupdt(yfms); return;
        }
        else if (XPLMGetDatai(yfms->xpl.transponder_mode) == 1)
        {
            XPLMSetDatai(yfms->xpl.transponder_mode, 3);
            yfs_rad1_pageupdt(yfms); return;
        }
        else
        {
            XPLMSetDatai(yfms->xpl.transponder_mode, 1);
            yfs_rad1_pageupdt(yfms); return;
        }
    }
    if (key[0] == 0 && key[1] == 4)
    {
        float alt; char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            snprintf(buf, sizeof(buf), "%.5s", yfms->mwindow.screen.text[10]);
            buf[4 + !yfms->ndt.alt.unit] = 0; yfs_spad_reset(yfms, buf, -1);
            return; // current baro to scratchpad
        }
        if (strnlen(buf, 6) != 4 + !yfms->ndt.alt.unit)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        if (sscanf(buf, "%f", &alt) != 1)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        switch (yfms->ndt.alt.unit)
        {
            case 1: // STD: 1013.00 -> 1013.04 -> 2991.51 -> 2992.00 -> 29.92
                alt = alt + 0.04f;
                alt = roundf(alt / 0.3386389f) / 100.0f;
                break;
            default:
                alt = roundf(alt * 100.00000f) / 100.0f;
                break;
        }
        if (alt > 40.0f || alt < 0.0f)
        {
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_Q350)
        {
            float offset        = roundf(100.0f * (alt - 29.92f));
            XPLMSetDatai(yfms->xpl.q350.pressLeftButton,       0);
            XPLMSetDatai(yfms->xpl.q350.pressRightButton,      0);
            XPLMSetDataf(yfms->xpl.q350.pressLeftRotary,  offset);
            XPLMSetDataf(yfms->xpl.q350.pressRightRotary, offset);
            yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
        }
        if (yfms->xpl.atyp == YFS_ATYP_FB76)
        {
            if (alt < FB76_BARO_MIN)     alt = FB76_BARO_MIN;
            if (alt > FB76_BARO_MAX)     alt = FB76_BARO_MAX;
            float baro_range = alt           - FB76_BARO_MIN;
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
            float rot_newp = alt - 29.92f + 0.5f;
            if (rot_newp < 0.0f) rot_newp = 0.0f;
            if (rot_newp > 1.0f) rot_newp = 1.0f;
            XPLMSetDataf(yfms->xpl.fb77.anim_25_rotery, rot_newp);
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
            XPLMSetDataf(yfms->xpl.ixeg.baro_inhg_sby_0001_ind, alt);
        }
        XPLMSetDataf(yfms->xpl.barometer_setting_in_hg_copilot, alt);
        XPLMSetDataf(yfms->xpl.barometer_setting_in_hg_pilot,   alt);
        yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
    }
    if (key[0] == 1 && key[1] == 4)
    {
        char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
        if  (strnlen(buf, 1) && !yfms->mwindow.screen.spad_reset)
        {
            if (!strcasecmp(buf, "InHg"))
            {
                yfms->ndt.alt.unit = 0; yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcasecmp(buf, "HPa"))
            {
                yfms->ndt.alt.unit = 1; yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        else
        {
            yfms->ndt.alt.unit = !yfms->ndt.alt.unit; yfs_rad1_pageupdt(yfms); return;
        }
    }
    if (key[0] == 0 && key[1] == 5)
    {
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
        double freq; char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
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
        if ((freq = get_nav_frequency(buf)) < 0.)
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
        int crs; char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
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
        double freq; int hz10; char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
        if (buf[0] == 0)
        {
            if (yfms->mwindow.screen.text[6][5] != '1')
            {
                return; // not an ILS frequency
            }
            strncpy(&buf[0], &yfms->mwindow.screen.text[6][5], 6); buf[6] = 0;
            yfs_spad_reset(yfms, buf, -1); return; // frequency to scratchpad
        }
        if ((freq = get_nav_frequency(buf)) < 0.)
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
        int crs; char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
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
        double freq; char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
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
        if ((freq = get_adf_frequency(buf)) < 0.)
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
