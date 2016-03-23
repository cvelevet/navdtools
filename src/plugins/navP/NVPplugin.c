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

/* Logging callback */
static int log_with_sdk(const char *format, va_list ap);

/* Miscellaneous data */
void *chandler_context = NULL;
void *navpmenu_context = NULL;

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
    strncpy(outName, "navP",                 255);
    strncpy(outSig,  "Rodeo314.navP",        255);
    strncpy(outDesc, "Miscellaneous stuff.", 255);

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
#ifdef NDT_VERSION
    XPLMDebugString("navP [info]: version %s\n", NDT_VERSION);
#else
    XPLMDebugString("navP [info]: unknown version :-(\n");
#endif
    XPLMDebugString("navP [info]: XPluginStart OK\n"); return 1;
}

PLUGIN_API void XPluginStop(void)
{
    if (chandler_context)
    {
        nvp_chandlers_close(&chandler_context);
    }
}

PLUGIN_API int XPluginEnable(void)
{
    /* set ndt_log callback so we write everything to the X-Plane log */
    ndt_log_set_callback(&log_with_sdk);

    /* navP features a menu :-) */
    if ((navpmenu_context = nvp_menu_init()) == NULL)
    {
        return 0; // menu creation failed :(
    }

    /* all good */
    XPLMDebugString("navP [info]: XPluginEnable OK\n"); XPLMSpeakString("nav P OK"); return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    /* unset ndt_log callback */
    ndt_log_set_callback(NULL);

    /* reset command handlers */
    nvp_chandlers_reset(chandler_context);

    /* kill the menu */
    if (navpmenu_context) nvp_menu_close(&navpmenu_context);

    /* all good */
    XPLMDebugString("navP [info]: XPluginDisable OK\n");
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
            if (inParam == XPLM_USER_AIRCRAFT)
            {
                nvp_chandlers_reset(chandler_context); // user's plane changed
            }
            break;

        case XPLM_MSG_AIRPORT_LOADED:
            break;

        case XPLM_MSG_SCENERY_LOADED:
            break;

        case XPLM_MSG_AIRPLANE_COUNT_CHANGED:
            break;

        case XPLM_MSG_PLANE_UNLOADED:
            break;

        case XPLM_MSG_WILL_WRITE_PREFS:
            break;

        case XPLM_MSG_LIVERY_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT)
            {
                nvp_chandlers_update(chandler_context); // custom plugins loaded
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
