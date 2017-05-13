/*
 * NVPplugin.c
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2015 Timothy D. Walker and others.
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

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMDefs.h"
#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMUtilities.h"

#include "common/common.h"

#include "NVPchandlers.h"
#include "NVPmenu.h"
#include "YFSmain.h"
#include "YFSmenu.h"

/* YFMS without navP? */
#if defined(YFMS_ONLY)
#define NAVP_ENABLED 0
#define PLUGIN_NAME "YFMS"
#else    // YFMS_ONLY
#define NAVP_ENABLED 1
#define PLUGIN_NAME "navP"
#endif   // YFMS_ONLY

/* version number */
#ifndef NDT_VERSION
#define NDT_VERSION "unknown :-("
#endif

/* Logging callback */
static int log_with_sdk(const char *format, va_list ap);

/* Miscellaneous data */
int          xplane_first_load = 1;
void         *chandler_context = NULL;
void         *navpmenu_context = NULL;
yfms_context *navpyfms_context = NULL;

#if IBM
#include <windows.h>
BOOL APIENTRY DllMain(HANDLE hModule,
                      DWORD  ul_reason_for_call,
                      LPVOID lpReserved)
{
    switch (ul_reason_for_call)
    {
        case DLL_PROCESS_ATTACH:
        case DLL_THREAD_ATTACH:
        case DLL_THREAD_DETACH:
        case DLL_PROCESS_DETACH:
            break;
    }
    return TRUE;
}
#endif

PLUGIN_API int XPluginStart(char *outName,
                            char *outSig,
                            char *outDesc)
{
    strncpy(outName,            PLUGIN_NAME, 255);
    strncpy(outSig,  "Rodeo314."PLUGIN_NAME, 255);
    strncpy(outDesc, "Yet Another X-Plugin", 255);

    /* set ndt_log callback so we write everything to the X-Plane log */
    ndt_log_set_callback(&log_with_sdk);

    /* Initialize command handling context */
    if ((chandler_context = nvp_chandlers_init()) == NULL)
    {
        return 0;
    }

#if APL
    /* use native (POSIX) paths under OS X */
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
#endif

    /* all good */
    XPLMDebugString(PLUGIN_NAME " [info]: version " NDT_VERSION "\n");
    XPLMDebugString(PLUGIN_NAME " [info]: XPluginStart OK\n"); return 1;
}

PLUGIN_API void XPluginStop(void)
{
    /* close command handling context */
    if (NAVP_ENABLED)
    {
        if (chandler_context) nvp_chandlers_close(&chandler_context);
    }

    /* …and the FMS */
    if (navpyfms_context) yfs_main_close(&navpyfms_context);

    /* unset ndt_log callback */
    ndt_log_set_callback(NULL);
}

PLUGIN_API int XPluginEnable(void)
{
    if (NAVP_ENABLED)
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
    }

    /* …and an FMS, too! */
    if ((navpyfms_context = yfs_main_init()) == NULL)
    {
        return 0; // menu creation failed :(
    }
    yfs_menu_resetall(navpyfms_context);

    /* all good */
    XPLMDebugString(PLUGIN_NAME " [info]: XPluginEnable OK\n"); return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    if (NAVP_ENABLED)
    {
        /* reset command handlers */
        nvp_chandlers_reset(chandler_context);

        /* kill the menu */
        nvp_chandlers_setmnu(chandler_context, NULL);
        if (navpmenu_context) nvp_menu_close(&navpmenu_context);
    }

    /* …and the FMS */
    yfs_menu_resetall(navpyfms_context);

    /* all good */
    XPLMDebugString(PLUGIN_NAME " [info]: XPluginDisable OK\n");
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho,
                                      long         inMessage,
                                      void        *inParam)
{
    switch (inMessage)
    {
        case XPLM_MSG_PLANE_CRASHED:
            break;

        case XPLM_MSG_PLANE_LOADED:
            if (NAVP_ENABLED)
            {
                if (xplane_first_load)
                {
                    XPLMPluginID xfsr = XPLMFindPluginBySignature("ivao.xivap");
                    if (XPLM_NO_PLUGIN_ID != xfsr) // X-FlightServer's X-IvAp
                    {
                        XPLMDisablePlugin(xfsr);
                    }
                    xplane_first_load = 0;
                }
            }
            break;

        case XPLM_MSG_AIRPORT_LOADED:
            if (NAVP_ENABLED) nvp_menu_reset(navpmenu_context);
            break;

        case XPLM_MSG_SCENERY_LOADED:
            if (NAVP_ENABLED) nvp_menu_reset(navpmenu_context);
            break;

        case XPLM_MSG_AIRPLANE_COUNT_CHANGED:
            break;

        case XPLM_MSG_PLANE_UNLOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                if (NAVP_ENABLED)
                {
                    nvp_menu_reset     (navpmenu_context);
                    nvp_chandlers_reset(chandler_context);
                }
                yfs_menu_resetall(navpyfms_context);
            }
            break;

        case XPLM_MSG_WILL_WRITE_PREFS:
            break;

        case XPLM_MSG_LIVERY_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // custom plugins loaded
            {
                if (NAVP_ENABLED)
                {
                    // nvp_chandlers_update fixes the value of some default XP
                    // datarefs read by yfs_idnt_pageupdt, we must run it first
                    nvp_chandlers_update(chandler_context);
                    nvp_menu_setup      (navpmenu_context);
                }
                yfs_idnt_pageupdt(navpyfms_context);
            }
            break;

        default:
            break;
    }
}

static int log_with_sdk(const char *format, va_list ap)
{
    int ret;
    char string[1024];
    ret = vsnprintf(string, sizeof(string), format, ap);
    XPLMDebugString(string);
    return ret;
}
