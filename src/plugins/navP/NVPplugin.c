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

#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMUtilities.h"

#include "common/common.h"

#include "NVPchandlers.h"
#include "NVPmenu.h"

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
        return 0;
    }

    /* all good */
    XPLMDebugString("navP [info]: version " NDT_VERSION "\n");
    XPLMDebugString("navP [info]: XPluginStart OK\n"); return 1;
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
    /* reset command handlers */
    nvp_chandlers_reset(chandler_context);

    /* navP features a menu :-) */
    if ((navpmenu_context = nvp_menu_init()) == NULL)
    {
        return 0; // menu creation failed :(
    }
    nvp_menu_reset                        (navpmenu_context);
    nvp_chandlers_setmnu(chandler_context, navpmenu_context);

    /* all good */
    XPLMDebugString("navP [info]: XPluginEnable OK\n"); return 1;
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
    XPLMDebugString("navP [info]: XPluginDisable OK\n");
}

void nvp_plugin_message(XPLMPluginID inFromWho,
                        long         inMessage,
                        void        *inParam)
{
    switch (inMessage)
    {
        case XPLM_MSG_PLANE_CRASHED:
            break;

        case XPLM_MSG_PLANE_LOADED:
            if (xplane_first_load)
            {
                XPLMPluginID xfsr = XPLMFindPluginBySignature("ivao.xivap");
                if (XPLM_NO_PLUGIN_ID != xfsr) // X-FlightServer's X-IvAp
                {
                    XPLMDisablePlugin(xfsr);
                }
                xplane_first_load = 0;
            }
            break;

        case XPLM_MSG_AIRPORT_LOADED:
            nvp_menu_reset(navpmenu_context);
            break;

        case XPLM_MSG_SCENERY_LOADED:
            nvp_menu_reset(navpmenu_context);
            break;

        case XPLM_MSG_AIRPLANE_COUNT_CHANGED:
            break;

        case XPLM_MSG_PLANE_UNLOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                nvp_menu_reset     (navpmenu_context);
                nvp_chandlers_reset(chandler_context);
            }
            break;

        case XPLM_MSG_WILL_WRITE_PREFS:
            break;

        case XPLM_MSG_LIVERY_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // custom plugins loaded
            {
                nvp_chandlers_update(chandler_context);
                nvp_menu_setup      (navpmenu_context);
            }
            break;

        default:
            break;
    }
}
