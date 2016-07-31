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
#define YFS_SOFT_KEY_1_W 50 // 9 buttons in 480 pixels, 3 separators
#define YFS_SOFT_KEY_1_H 38 // 6 buttons in 240 pixels, 1 separator
#define YFS_SOFT_KEY_1_B 3  // ~8% height: 3px border x2, per button

typedef struct
{
    struct
    {
        XPWidgetID id;
        int win_state;
        struct
        {
            // row 1, right-to-left
            XPWidgetID keyid_num3;
            XPWidgetID keyid_num2;
            XPWidgetID keyid_num1;
            XPWidgetID keyid_prev;
            XPWidgetID keyid_list;
            XPWidgetID keyid__dto;
            XPWidgetID keyid_vnav;
            XPWidgetID keyid__nav;
            XPWidgetID keyid_data;
        } keys;
    }
    mwindow;
}
yfms_context;

static int yfs_mwindowh(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);

typedef struct
{
    XPWidgetID *_wid;
    const char *desc;
    int         btnW;
    int         btnH;
    int         btBW;
    int         btBH;
    int         inBM;
    int         inLT;
    int         inTP;
    int         inRT;
} yfms_buttn_ctx;
static int row_prepend_button(yfms_buttn_ctx *b, XPWidgetID container_id)
{
    if (!b || !b->_wid || !b->desc || !container_id)
    {
        return ENOMEM;
    }
    /*
     * Draw within the designated area, top right to bottom left,
     * using the provided dimensions: width, height, and borders.
     */
    int inTP = b->inTP - b->btBH;
    int inRT = b->inRT - b->btBW;
    int inBM = b->inTP + b->btBH - b->btnH;
    int inLT = b->inRT + b->btBW - b->btnW;
    XPWidgetID wid = XPCreateWidget(inLT, inTP, inRT, inBM, 1,
                                    b->desc, 0, container_id,
                                    xpWidgetClass_Button);
    if (wid == NULL)
    {
        return EINVAL;
    }
    // update right coordinates as per the button width, border
    // top coordinates remain unchanged (we're on the same row)
    b->inRT = inLT - b->btBW;
    *b->_wid = wid; return 0;
}

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

    /*
     * Add a bunch of not-so-pretty buttons.
     *
     * Width:  we have 9 keys across and 3 "separators".
     * Height: we have 6 keys across and 1 "separator" in half the total height.
     */
    int separatrW = (YFS_MAINWINDOW_W / 1 - 9 * YFS_SOFT_KEY_1_W) / 3;
    int separatrH = (YFS_MAINWINDOW_H / 2 - 6 * YFS_SOFT_KEY_1_H) / 1;
    int keybordBM = (inBM / 1);
    int keybordLT = (inLT / 1);
    int keybordTP = (inTP / 2);
    int keybordRT = (inRT / 1);
    int mainwinBM = (inBM);
    int mainwinLT = (inLT);
    int mainwinTP = (inTP);
    int mainwinRT = (inRT);
    int            r_value;
    yfms_buttn_ctx softkey;
    // we start at the keyboard's top right, with a separator on the right-hand side
    softkey.btnW = YFS_SOFT_KEY_1_W; softkey.btnH = YFS_SOFT_KEY_1_H;
    softkey.btBW = softkey.btBH = YFS_SOFT_KEY_1_B;
    softkey.inRT = keybordRT - separatrW;
    softkey.inTP = keybordTP;
    // first row of buttons
    softkey.desc = "3";
    softkey._wid = &yfms->mwindow.keys.keyid_num3;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "2";
    softkey._wid = &yfms->mwindow.keys.keyid_num2;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "1";
    softkey._wid = &yfms->mwindow.keys.keyid_num1;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    {
        softkey.inRT -= separatrW; // horizontal separator
    }
    softkey.desc = "PREV";
    softkey._wid = &yfms->mwindow.keys.keyid_prev;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "LIST";
    softkey._wid = &yfms->mwindow.keys.keyid_list;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "DTO";
    softkey._wid = &yfms->mwindow.keys.keyid__dto;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "VNAV";
    softkey._wid = &yfms->mwindow.keys.keyid_vnav;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "NAV";
    softkey._wid = &yfms->mwindow.keys.keyid__nav;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "DATA";
    softkey._wid = &yfms->mwindow.keys.keyid_data;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }

    /* all good */
    return 0;

create_button_fail:
    ndt_log("YFMS [warning]: could not create button with description \"%s\"\n", softkey.desc);
    return r_value;
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

    /* create the main window */
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
