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
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMProcessing.h"

#include "YFSmain.h"
#include "YFSmenu.h"
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
    char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
    if  (strnlen(buf, 1))
    {
        yfs_spad_clear(yfms);
    }
    yfs_spad_reset(yfms, "YFMS INITIALIZED", COLR_IDX_ORANGE);

    /* flight loop callback to monitor dataref changes and update pages as required */
    if (yfms->xpl.fl_callback)
    {
        XPLMUnregisterFlightLoopCallback(yfms->xpl.fl_callback, yfms);
    }
    {
        XPLMRegisterFlightLoopCallback(yfms->xpl.fl_callback = &yfs_flight_loop_cback, -1, yfms);
    }

    /* callbacks for page-specific keys */
    yfms->spcs.cback_menu = &yfs_key_callback_menu;

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
    yfms->lsks[0][0].cback = yfms->lsks[1][0].cback = &yfs_lsk_callback_menu;
    yfms->mwindow.current_page = PAGE_MENU; yfs_menu_pageupdt(yfms); return;
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
    yfs_main_rline(yfms, 0, COLR_IDX_WHITE); int len = strlen("MCDU MENU");
    sprintf(yfms->mwindow.screen.text[0], "%*s", (1 + YFS_DISPLAY_NUMC + len) / 2, "MCDU MENU");

    /* line 2 left: ident page (green) */
    yfs_printf_lft(yfms, 2, 0, COLR_IDX_GREEN, "<FMGC");

    /* line 2 right: reset (white) */
    yfs_printf_rgt(yfms, 2, 0, COLR_IDX_WHITE, "RESET>");

    /* all good */
    return;
}

void yfs_idnt_pageopen(yfms_context *yfms)
{
    if (!yfms)
    {
        return; // no error
    }
    yfms->mwindow.current_page = PAGE_IDNT; yfs_idnt_pageupdt(yfms); return;
}

void yfs_idnt_pageupdt(yfms_context *yfms)
{
    if (!yfms || yfms->mwindow.current_page != PAGE_IDNT)
    {
        return; // no error
    }

    /* row buffer */
    int len; char buf[YFS_DISPLAY_NUMC + 1];

    /* reset lines before drawing */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }

    /* line 0: aircraft ICAO identifier (white, centered) */
    yfs_main_rline(yfms, 0, COLR_IDX_WHITE);
    len = XPLMGetDatab(yfms->xpl.acf_ICAO, buf, 0, sizeof(buf) - 1); buf[len] = 0; len = strlen(buf);
    sprintf(yfms->mwindow.screen.text[0], "%*s", (1 + YFS_DISPLAY_NUMC + len) / 2, buf);

    /* line 1: header (white, offset 1 right) */
    yfs_main_rline(yfms,  1, COLR_IDX_WHITE); sprintf(yfms->mwindow.screen.text[ 1], "%s", " ENG");

    /* line 2: engine count & type (green) */
    yfs_main_rline(yfms,  2, COLR_IDX_GREEN);
    XPLMGetDatavi (yfms->xpl.acf_en_type, &len, 0, 1);
    switch (len)
    {
        case 2:
            sprintf(yfms->mwindow.screen.text[2], "%d TURBOPROP", XPLMGetDatai(yfms->xpl.acf_num_engines));
            break;
        case 5:
            sprintf(yfms->mwindow.screen.text[2], "%d TURBOFAN",  XPLMGetDatai(yfms->xpl.acf_num_engines));
            break;
        default:
            sprintf(yfms->mwindow.screen.text[2], "%d OTHER",     XPLMGetDatai(yfms->xpl.acf_num_engines));
            break;
    }

    /* line 3: header (white, offset 1 right) */
    yfs_main_rline(yfms,  3, COLR_IDX_WHITE); sprintf(yfms->mwindow.screen.text[ 3], "%s", " ACTIVE DATA BASE");

    /* line 4: navigation database information (blue) */
    yfs_main_rline(yfms,  4, COLR_IDX_BLUE);
    sprintf(yfms->mwindow.screen.text[4], "%.10s", yfms->ndt.ndb->info.idnt);

    /* line 5: header (white, offset 1 right) */
    yfs_main_rline(yfms,  5, COLR_IDX_WHITE); sprintf(yfms->mwindow.screen.text[ 5], "%s", " SECOND DATA BASE");

    /* line 6: navigation database information (blue) */
    yfs_main_rline(yfms,  6, COLR_IDX_BLUE);  sprintf(yfms->mwindow.screen.text[ 6], "%s", "NONE");

#ifndef NDT_VERSION
#define NDT_VERSION "Unknown"
#endif
    /* line 9: header (white, offset 1 right) */
    yfs_main_rline(yfms,  9, COLR_IDX_WHITE); sprintf(yfms->mwindow.screen.text[ 9], "%s", " VERSION");

    /* line 10: version information (blue) */
    yfs_main_rline(yfms, 10, COLR_IDX_BLUE);  sprintf(yfms->mwindow.screen.text[10], "%s", NDT_VERSION);

    /* line 11: header (white, offset 1 right) */
    yfs_main_rline(yfms, 11, COLR_IDX_WHITE); sprintf(yfms->mwindow.screen.text[11], "%s", " IDLE/PERF");

    /* line 12: version information (green) */
    yfs_main_rline(yfms, 12, COLR_IDX_GREEN); sprintf(yfms->mwindow.screen.text[12], "%s", "+0.0/+0.0");

    /* all good */
    return;
}

static void yfs_key_callback_menu(void *yfms)
{
    yfs_menu_pageopen(yfms); return;
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

// TODO: monitor radio frequencies, etc.
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
    return 0.5f; // half a second should do fine
}
