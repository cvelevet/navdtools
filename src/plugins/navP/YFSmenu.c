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

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDataAccess.h"

#include "YFSmain.h"
#include "YFSmenu.h"
#include "YFSspad.h"

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

    /* all good */
    if (XPIsWidgetVisible(yfms->mwindow.id))
    {
        yfs_main_toggl(yfms);
    }
    yfs_idnt_pageopen (yfms); return;
}

//fixme page needs to be updated after each aircraft (livery?) load
void yfs_idnt_pageopen(yfms_context *yfms)
{
    if (!yfms)
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
    yfms->mwindow.current_page = PAGE_IDNT;

    /* line 0: aircraft ICAO identifier (white, centered) */
    yfs_main_rline(yfms, 0, COLR_IDX_WHITE);
    len = XPLMGetDatab(yfms->xpl.acf_ICAO, buf, 0, sizeof(buf) - 1); buf[len] = 0;
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
