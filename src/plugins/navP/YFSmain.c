/*
 * YFSmain.c
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

#include <errno.h>
#include <stdlib.h>

#include "Widgets/XPStandardWidgets.h"
#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDisplay.h"

#include "common/common.h"

#include "YFSmain.h"

#define YFS_MAINWINDOW_W 480
#define YFS_MAINWINDOW_H 480

typedef struct
{
    struct
    {
        XPWidgetID id;
        int win_state;
        struct
        {
            //fixme
        } keys;
    }
    mwindow;
}
yfms_context;

static int yfs_mwindowh(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);

/*
 * Create and place the main window and its contents.
 * Do not show it until requested by the user (hide root by default).
 *
 * x and y axes start at bottom left (obviously, except for me).
 * We create widgets directly there: 0 -> (height - 1)
 *                                   0 -> (width  - 1)
 * We move the widgets just before showing them.
 * Thankfully the child widgets move with their parents :-)
 */
static int create_main_window(yfms_context *yfms)
{
    if (!yfms)
    {
        return ENOMEM;
    }

    /* Main window (root, container) */
    int inBM = +0;
    int inLT = +0;
    int inTP = -1 + YFS_MAINWINDOW_H;
    int inRT = -1 + YFS_MAINWINDOW_W;
    yfms->mwindow.id = XPCreateWidget(inLT, inTP, inRT, inBM, 0,
                                      "YFMS", 1, NULL,
                                      xpWidgetClass_MainWindow);
    if (!yfms->mwindow.id)
    {
        ndt_log("YFMS [warning]: could not create main window\n");
        return -1;
    }
    XPSetWidgetProperty(yfms->mwindow.id, xpProperty_MainWindowHasCloseBoxes, 1);
    XPSetWidgetProperty(yfms->mwindow.id, xpProperty_MainWindowType, xpMainWindowStyle_MainWindow);
    XPAddWidgetCallback(yfms->mwindow.id, &yfs_mwindowh);

    /* all good */
    return 0;
}

static void toggle_main_window(yfms_context *yfms)
{
    if (!yfms)
    {
        return;
    }
    if (yfms->mwindow.win_state == 0) // place window (for now, display center)
    {
        int scrw, scrh;
        yfms->mwindow.win_state += 1;
        XPLMGetScreenSize(&scrw, &scrh);
        int inBM = (scrh - YFS_MAINWINDOW_H) / 2;
        int inLT = (scrw - YFS_MAINWINDOW_W) / 2;
        int inTP = (inBM + YFS_MAINWINDOW_H) - 1;
        int inRT = (inLT + YFS_MAINWINDOW_W) - 1;
        XPSetWidgetGeometry(yfms->mwindow.id, inLT, inTP, inRT, inBM);
    }
    if (XPIsWidgetVisible(yfms->mwindow.id))
    {
        XPHideWidget(yfms->mwindow.id);
        return;
    }
    XPShowWidget(yfms->mwindow.id);
    return;
}

void* yfs_main_init(void)
{
    yfms_context *yfms = calloc(1, sizeof(yfms_context));
    if (!yfms)
    {
        return NULL;
    }

    /* */
    if (create_main_window(yfms))
    {
        goto fail;
    }

    /* all good */
    return yfms;

fail:
    yfs_main_close((void**)&yfms);
    return NULL;
}

int yfs_main_close(void **_yfms_ctx)
{
    yfms_context *yfms = *_yfms_ctx;
    *_yfms_ctx = NULL;
    if (!yfms)
    {
        return -1;
    }

    /* Destroy the main window */
    if (yfms->mwindow.id)
    {
        XPDestroyWidget(yfms->mwindow.id, 1); // destroy children recursively
    }

    /* all good */
    free(yfms);
    return 0;
}

void yfs_main_toggl(void *yfms_ctx)
{
    yfms_context  *yfms = yfms_ctx;
    if (!yfms)
    {
        return;
    }
    toggle_main_window(yfms);
    return;
}

static int yfs_mwindowh(XPWidgetMessage inMessage,
                        XPWidgetID      inWidget,
                        intptr_t        inParam1,
                        intptr_t        inParam2)
{
    if (inMessage == xpMessage_CloseButtonPushed)
    {
        XPHideWidget(inWidget);
        return 1;
    }
    return 0;
}
