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

#include "common/common.h"

#include "YFSmain.h"

#define YFS_MAINWINDOW_W 100
#define YFS_MAINWINDOW_H 100

typedef struct
{
    struct
    {
        XPWidgetID id;
    }
    mwindow;
}
yfms_context;

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
    int inLT = +0;
    int inBM = +0;
    int inRT = -1 + YFS_MAINWINDOW_W;
    int inTP = -1 + YFS_MAINWINDOW_H;
    yfms->mwindow.id = XPCreateWidget(inLT, inTP, inRT, inBM, 0,
                                      "YFMS", 1, NULL,
                                      xpWidgetClass_MainWindow);
    if (!yfms->mwindow.id)
    {
        ndt_log("YFMS [warning]: could not create main window\n");
        return -1;
    }
    XPSetWidgetProperty(yfms->mwindow.id, xpProperty_MainWindowHasCloseBoxes, 1);
    XPSetWidgetProperty(yfms->mwindow.id, xpProperty_MainWindowType, xpMainWindowStyle_Translucent);
//    XPAddWidgetCallback(yfms->mwindow.id, &widget_hdlr1);

    /* all good */
    return 0;
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

int yfs_main_close(void **_yfms_context)
{
    yfms_context *yfms = *_yfms_context;
    *_yfms_context = NULL;
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
