/*
 * NVPplugin.c
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2017 Timothy D. Walker and others.
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

#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMUtilities.h"

#include "common/common.h"

#include "NVPchandlers.h"
#include "NVPmenu.h"

//#define NVP_DEBUG 1

/* version number */
#ifndef NDT_VERSION
#define NDT_VERSION "unknown :-("
#endif

/* Miscellaneous data */
int          xplane_first_load = 1;
void         *chandler_context = NULL;
void         *navpmenu_context = NULL;

int nvp_plugin_start(char *outName,
                     char *outSig,
                     char *outDesc)
{
    strncpy(outName,                 "navP", 255);
    strncpy(outSig,         "Rodeo314.navP", 255);
    strncpy(outDesc, "Yet Another X-Plugin", 255);

    /* Initialize command handling context */
    if ((chandler_context = nvp_chandlers_init()) == NULL)
    {
        XPLMDebugString("navP [error]: nvp_chandlers_init() failed\n");
        return 0;
    }

    /* all good */
    XPLMDebugString("navP [info]: version " NDT_VERSION "\n");
    XPLMDebugString("navP [info]: nvp_plugin_start OK\n"); return 1;
}

void nvp_plugin_stop(void)
{
    /* close command handling context */
    if (chandler_context)
    {
        nvp_chandlers_close(&chandler_context);
    }
}

int nvp_plugin_enable(void)
{
    /* navP features a menu :-) */
    if ((navpmenu_context = nvp_menu_init()) == NULL)
    {
        XPLMDebugString("navP [error]: nvp_menu_init() failed\n");
        return 0;
    }
    nvp_menu_reset                        (navpmenu_context);
    nvp_chandlers_setmnu(chandler_context, navpmenu_context);

#if 0 // could never get this code (or variations thereof) to work
#if TIM_ONLY
    /* update date before custom scenery is loaded (e.g. SAM Seasons) */
    XPLMDataRef date_days = XPLMFindDataRef("sim/time/local_date_days");
    if (date_days)
    {
        ndt_date now = ndt_date_now();
        // note: X-Plane doesn't seem to know 02/29 (makes our job that much easier :-)
        int month2days[12] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, };
        int xplm_date_days = month2days[now.month - 1] + now.day - 1;
        XPLMSetDatai(date_days, xplm_date_days);
    }
#endif
#endif

    /* all good */
    XPLMDebugString("navP [info]: nvp_plugin_enable OK\n"); return 1;
}

void nvp_plugin_disable(void)
{
    /* reset command handlers */
    nvp_chandlers_reset(chandler_context);

    /* kill the menu */
    if (navpmenu_context)
    {
        nvp_chandlers_setmnu(chandler_context, NULL);
        nvp_menu_close(&navpmenu_context);
    }

    /* all good */
    XPLMDebugString("navP [info]: nvp_plugin_disable OK\n");
}

void nvp_plugin_message(XPLMPluginID inFromWho,
                        long         inMessage,
                        void        *inParam)
{
#if NVP_DEBUG
    ndt_log("DEBUG: ---------------------------\nDEBUG: nvp_plugin_message begin\nDEBUG: with inMessage == %ld\n", inMessage);
#endif
    switch (inMessage)
    {
        case XPLM_MSG_PLANE_CRASHED:
            if (inParam == XPLM_USER_AIRCRAFT)
            {
                break;
            }
            break;

        case XPLM_MSG_PLANE_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT)
            {
                if (xplane_first_load)
                {
                    xplane_first_load = 0;
                }
                nvp_chandlers_onload(chandler_context);
                break;
            }
            break;

        case XPLM_MSG_AIRPORT_LOADED:
            nvp_menu_reset(navpmenu_context); // TODO: document why we also call this here
            break;

        case XPLM_MSG_SCENERY_LOADED:
            nvp_chandlers_scload(chandler_context);
            nvp_menu_reset      (navpmenu_context); // TODO: document why we also call this here
            break;

        case XPLM_MSG_AIRPLANE_COUNT_CHANGED:
            break;

        case XPLM_MSG_PLANE_UNLOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                nvp_menu_reset     (navpmenu_context);
                nvp_chandlers_reset(chandler_context);
                break;
            }
            break;

        case XPLM_MSG_WILL_WRITE_PREFS:
            break;

        case XPLM_MSG_LIVERY_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // custom plugins loaded
            {
                nvp_chandlers_update(chandler_context);
                nvp_chandlers_on_run(chandler_context);
                nvp_menu_setup      (navpmenu_context);
                break;
            }
            break;

        default:
            break;
    }
#if NVP_DEBUG
    ndt_log("DEBUG: still inMessage == %ld\nDEBUG: nvp_plugin_message end\nDEBUG: ---------------------------\n", inMessage);
#endif    
}
