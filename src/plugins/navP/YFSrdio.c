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

#include "YFSmain.h"
#include "YFSrdio.h"
#include "YFSspad.h"

static void yfs_rad1_pageupdt    (yfms_context *yfms);
static void yfs_rad2_pageupdt    (yfms_context *yfms);
static void yfs_lsk_callback_rad1(yfms_context *yfms, int key[2], intptr_t refcon);

void yfs_rdio_pageopen(yfms_context *yfms)
{
    if (yfms == NULL)
    {
        return; // no error
    }
    int page = yfms->mwindow.current_page;
    if (page == PAGE_MENU)
    {
        return; // special page, FMGC disabled
    }
    if (page == PAGE_RAD1)
    {
        yfms->mwindow.current_page = PAGE_RAD2;
        //fixme callbacks
    }
    else
    {
        yfms->mwindow.current_page = PAGE_RAD1;
        yfms->lsks[0][0].cback = yfms->lsks[1][0].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
        yfms->lsks[0][1].cback = yfms->lsks[1][1].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
        yfms->lsks[0][3].cback = yfms->lsks[1][3].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
        yfms->lsks[0][4].cback = yfms->lsks[1][4].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
        yfms->lsks[0][5].cback = yfms->lsks[1][5].cback = (YFS_LSK_f)&yfs_lsk_callback_rad1;
    }
    yfs_rdio_pageupdt(yfms);
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
    snprintf(buf, sizeof(buf), "%*s", (YFS_DISPLAY_NUMC + 10) / 2, "COM RADIO");
    yfs_printf_lft(yfms,  0, 0, COLR_IDX_WHITE,    buf);
    yfs_printf_rgt(yfms,  0, 0, COLR_IDX_WHITE, "<-> ");

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
    snprintf(buf, sizeof(buf), "%*s", (YFS_DISPLAY_NUMC + 5) / 2, "XPDR");
    yfs_printf_lft(yfms,  7, 0, COLR_IDX_WHITE,    buf);
    yfs_printf_lft(yfms,  7, 0, COLR_IDX_WHITE, "CODE");
    yfs_printf_rgt(yfms,  7, 0, COLR_IDX_WHITE, "MODE");

    /* line 8: XPDR information */
    switch (XPLMGetDatai(yfms->xpl.transponder_mode))
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
    snprintf(buf, sizeof(buf), "%*s", (YFS_DISPLAY_NUMC + 10) / 2, "ALTIMETER");
    yfs_printf_lft(yfms,  9, 0, COLR_IDX_WHITE,    buf);
    yfs_printf_lft(yfms,  9, 0, COLR_IDX_WHITE,  "BARO");
    yfs_printf_rgt(yfms,  9, 0, COLR_IDX_WHITE,  "UNIT");

    /* line 10: barometric altimeter information */
    float alt, alt_inhg = XPLMGetDataf(yfms->xpl.barometer_setting_in_hg_pilot);
    switch (yfms->ndt.alt.unit)
    {
        case 1: // hPa
            alt = roundf(alt_inhg * 33.86389f);
            snprintf (buf, sizeof(buf), "%04.0f", alt);
            yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE,    buf);
            yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE,  "hPa");
            break;
        default: // inches of mercury
            alt = roundf(alt_inhg * 100.0f);
            snprintf (buf, sizeof(buf), "%05.2f", alt / 100.0f);
            yfs_printf_lft(yfms, 10, 0, COLR_IDX_BLUE,    buf);
            yfs_printf_rgt(yfms, 10, 0, COLR_IDX_BLUE, "InHg");
            break;
    }

    /* line 12: switches (white) */
    if (alt_inhg <= 29.913 /* 1012.97 */ || alt_inhg >= 29.929 /* 1013.51 */)
    {
        yfs_printf_lft(yfms, 12, 0, COLR_IDX_WHITE, "<ALT STD");
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
    //fixme
}

static void yfs_lsk_callback_rad1(yfms_context *yfms, int key[2], intptr_t refcon)
{
    if (yfms == NULL || yfms->mwindow.current_page != PAGE_RAD1)
    {
        return; // callback not applicable to current page
    }
    if (key[0] == 0 && key[1] == 0)
    {
        XPLMCommandOnce(yfms->xpl.com1_standy_flip); yfs_rad1_pageupdt(yfms); return;
    }
    if (key[0] == 1 && key[1] == 0)
    {
        XPLMCommandOnce(yfms->xpl.com2_standy_flip); yfs_rad1_pageupdt(yfms); return;
    }
    //fixme standby radio frequencies
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
                XPLMSetDatai(yfms->xpl.transponder_mode, 0);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcmp(buf, "STBY"))
            {
                XPLMSetDatai(yfms->xpl.transponder_mode, 1);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            if (!strcmp(buf, "TA/RA"))
            {
                XPLMSetDatai(yfms->xpl.transponder_mode, 2);
                yfs_spad_clear(yfms); yfs_rad1_pageupdt(yfms); return;
            }
            yfs_spad_reset(yfms, "FORMAT ERROR", -1); return;
        }
        else if (XPLMGetDatai(yfms->xpl.transponder_mode) == 1)
        {
            XPLMSetDatai(yfms->xpl.transponder_mode, 2);
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
