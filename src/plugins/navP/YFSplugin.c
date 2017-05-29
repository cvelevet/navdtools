/*
 * YFSplugin.c
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

#include "YFSmain.h"
#include "YFSmenu.h"
#include "YFSplugin.h"

/* version number */
#ifndef NDT_VERSION
#define NDT_VERSION "unknown :-("
#endif

/* Miscellaneous data */
yfms_context *navpyfms_context = NULL;

int yfs_plugin_start(char *outName,
                     char *outSig,
                     char *outDesc)
{
    strncpy(outName,                     "YFMS", 255);
    strncpy(outSig,             "Rodeo314.YFMS", 255);
    strncpy(outDesc, "Yet Another X-Plane FMGS", 255);

    /* all good */
    XPLMDebugString("YFMS [info]: version " NDT_VERSION "\n");
    XPLMDebugString("YFMS [info]: yfs_plugin_start OK\n"); return 1;
}

void yfs_plugin_stop(void)
{
    // nothing to do here
}

int yfs_plugin_enable(void)
{
    /* initialize FMGS context and its menu */
    if ((navpyfms_context = yfs_main_init()) == NULL)
    {
        return 0; // menu creation failed :(
    }
    yfs_menu_resetall(navpyfms_context);

    /* all good */
    XPLMDebugString("YFMS [info]: yfs_plugin_enable OK\n"); return 1;
}

void yfs_plugin_disable(void)
{
    /* close FMGS context */
    if (navpyfms_context)
    {
        yfs_main_close(&navpyfms_context);
    }

    /* all good */
    XPLMDebugString("YFMS [info]: yfs_plugin_disable OK\n");
}

void yfs_plugin_message(XPLMPluginID inFromWho,
                        long         inMessage,
                        void        *inParam)
{
    switch (inMessage)
    {
        case XPLM_MSG_PLANE_CRASHED:
            break;

        case XPLM_MSG_PLANE_LOADED:
            break;

        case XPLM_MSG_AIRPORT_LOADED:
            break;

        case XPLM_MSG_SCENERY_LOADED:
            break;

        case XPLM_MSG_AIRPLANE_COUNT_CHANGED:
            break;

        case XPLM_MSG_PLANE_UNLOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                yfs_menu_resetall(navpyfms_context);
            }
            break;

        case XPLM_MSG_WILL_WRITE_PREFS:
            break;

        case XPLM_MSG_LIVERY_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // custom plugins loaded
            {
                yfs_idnt_pageupdt(navpyfms_context);
            }
            break;

        default:
            break;
    }
}
