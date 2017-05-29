/*
 * XPLplugin.c
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

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMUtilities.h"

#include "common/common.h"

#include "NVPplugin.h"
#include "YFSplugin.h"

/* Logging callback */
static int log_with_sdk(const char *format, va_list ap);

/* Miscellaneous data */
int navp_init_ok = 0;
int yfms_init_ok = 0;

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
#if APL
    /* use native (POSIX) paths under OS X */
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
#endif

    /* set ndt_log callback so we write everything to the X-Plane log */
    ndt_log_set_callback(&log_with_sdk);

    /* start our sub-plugins */
#ifndef YFMS_ONLY
    if ((navp_init_ok = nvp_plugin_start(outName, outSig, outDesc)) != 1)
    {
        XPLMDebugString("Rodeo314 [error]: couldn't start navP\n");
    }
#endif
#ifndef NAVP_ONLY
    if ((yfms_init_ok = yfs_plugin_start(outName, outSig, outDesc)) != 1)
    {
        XPLMDebugString("Rodeo314 [error]: couldn't start YFMS\n");
    }
#endif
    return 1;
}

PLUGIN_API void XPluginStop(void)
{
    /* stop our sub-plugins */
    if (navp_init_ok == 1)
    {
        nvp_plugin_stop();
    }
    if (yfms_init_ok == 1)
    {
        yfs_plugin_stop();
    }

    /* unset ndt_log callback */
    ndt_log_set_callback(NULL);
}

PLUGIN_API int XPluginEnable(void)
{
    /* enable our sub-plugins */
    if (navp_init_ok == 1)
    {
        if (nvp_plugin_enable() != 1)
        {
            XPLMDebugString("Rodeo314 [error]: couldn't enable navP\n");
            navp_init_ok = 0;
        }
    }
    if (yfms_init_ok == 1)
    {
        if (yfs_plugin_enable() != 1)
        {
            XPLMDebugString("Rodeo314 [error]: couldn't enable YFMS\n");
            yfms_init_ok = 0;
        }
    }
    return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    /* disable our sub-plugins */
    if (navp_init_ok == 1)
    {
        nvp_plugin_disable();
    }
    if (yfms_init_ok == 1)
    {
        yfs_plugin_disable();
    }
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho,
                                      long         inMessage,
                                      void        *inParam)
{
    /* forward all messages to out sub-plugins */
    if (navp_init_ok == 1)
    {
        nvp_plugin_message(inFromWho, inMessage, inParam);
    }
    if (yfms_init_ok == 1)
    {
        yfs_plugin_message(inFromWho, inMessage, inParam);
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
