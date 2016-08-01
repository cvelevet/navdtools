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

#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "Widgets/XPStandardWidgets.h"
#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDisplay.h"
#include "XPLM/XPLMGraphics.h"
#include "XPLM/XPLMMenus.h"
#include "XPLM/XPLMUtilities.h"

#include "common/common.h"

#include "YFSkeys.h"
#include "YFSmain.h"
#include "YFSmenu.h"
#include "YFSspad.h"

static int YFS_FONT_BASIC_W =   8;  // 2 + xplmFont_Basic width
static int YFS_FONT_BASIC_H =  14;  // 4 + xplmFont_Basic height
static int YFS_MAINSCREEN_W = 208;  // 2 + YFS_DISPLAY_NUMC characters per line
static int YFS_MAINSCREEN_H = 196;  // YFS_DISPLAY_NUMR rows * YFS_FONT_BASIC_H
static int YFS_MAINWINDOW_W = 336;  // margins + up to 8 buttons across (1.5 b. margin)
static int YFS_MAINWINDOW_H = 470;  // display + up to 10 rows below it (3 rows margin)
static int YFS_SOFT_KEY_1_W =  51;  // 3x 15 pixels plus top & bottom borders
static int YFS_SOFT_KEY_1_H =  21;  // 1x 15 pixels plus top & bottom borders
static int YFS_SOFT_KEY_1_B =   3;  // 2x  3 pixels of border between each button
static int YFS_SOFT_KEY_2_W =  36;  // 2x 15 pixels plus top & bottom borders
static int YFS_SOFT_KEY_2_H =  21;  // 1x 15 pixels plus top & bottom borders
static int YFS_SOFT_KEY_2_B =   3;  // 2x  3 pixels of border between each button
static float COLR_BLACK  [] = { 0.0f, 0.0f, 0.0f, };
static float COLR_WHITE  [] = { 1.0f, 1.0f, 1.0f, };
static float COLR_RED    [] = { 1.0f, 0.0f, 0.0f, };
static float COLR_GREEN  [] = { 0.0f, 1.0f, 0.0f, };
static float COLR_BLUE   [] = { 0.0f, 0.0f, 1.0f, };
static float COLR_MAGENTA[] = { 1.0f, 0.0f, 1.0f, };
static float COLR_ORANGE [] = { 1.0f, .66f, 0.0f, };
static float COLR_YELLOW [] = { 1.0f, 1.0f, 0.0f, };

static void menu_handler(void *inMenuRef,                void *inItemRef);
static int  yfs_mwindowh(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
static int  yfs_mcdubgrh(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
static int  yfs_mcdudish(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
static int  chandler_tog(XPLMCommandRef, XPLMCommandPhase, void*inRefcon);

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
    /* typo protection: don't assign new button to ID that's already set */
    assert(*b->_wid == NULL);
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
 *
 * note1: we offset numerical keys (cf. real thing) in order to align them
 *        with alphabetical keys, as it creates a more pleasing appearance.
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
    if ((yfms->mwindow.id = XPCreateWidget(inLT, inTP, inRT, inBM,
                                           0, "YFMS", 1, NULL,
                                           xpWidgetClass_MainWindow)) == NULL)
    {
        ndt_log("YFMS [error]: could not create main window\n");
        return -1;
    }
    XPSetWidgetProperty(yfms->mwindow.id, xpProperty_MainWindowType, xpMainWindowStyle_MainWindow);
    XPSetWidgetProperty(yfms->mwindow.id, xpProperty_MainWindowHasCloseBoxes, 1);
    XPSetWidgetProperty(yfms->mwindow.id, xpProperty_Refcon, (intptr_t)yfms);
    XPAddWidgetCallback(yfms->mwindow.id, &yfs_mwindowh);

    /*
     * Add a bunch of not-so-pretty buttons (10 rows + 2 rows as margins).
     */
    int separat1W = (YFS_MAINWINDOW_W - 6 * YFS_SOFT_KEY_1_W) / 2;
    int separat2W = (YFS_MAINWINDOW_W - 8 * YFS_SOFT_KEY_2_W) / 2;
    int keybordTP = (inBM + 12 * YFS_SOFT_KEY_1_H - 2 * YFS_SOFT_KEY_1_B);
    int keybordBM = (inBM);
    int keybordLT = (inLT);
    int keybordRT = (inRT);
    int mainwinBM = (inBM);
    int mainwinLT = (inLT);
    int mainwinTP = (inTP);
    int mainwinRT = (inRT);
    int            r_value;
    yfms_buttn_ctx softkey;
    softkey.btnW = YFS_SOFT_KEY_1_W; softkey.btnH = YFS_SOFT_KEY_1_H;
    softkey.btBW = softkey.btBH = YFS_SOFT_KEY_1_B;
    // row 1
    {
        softkey.inRT = keybordRT - separat1W - softkey.btnW * 1; // missing 1 column
        softkey.inTP = keybordTP             - softkey.btnH * 1; // set top position
    }
    softkey.desc = "DATA";
    softkey._wid = &yfms->mwindow.keys.keyid_data;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "INIT";
    softkey._wid = &yfms->mwindow.keys.keyid_init;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "PERF";
    softkey._wid = &yfms->mwindow.keys.keyid_perf;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "PROG";
    softkey._wid = &yfms->mwindow.keys.keyid_prog;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "DIR";
    softkey._wid = &yfms->mwindow.keys.keyid_dirt;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    // row 2
    {
        softkey.inRT = keybordRT - separat1W - softkey.btnW * 0; // all columns here
        softkey.inTP = keybordTP             - softkey.btnH * 2; // set top position
    }
    softkey.desc = "MENU";
    softkey._wid = &yfms->mwindow.keys.keyid_menu;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "";
    softkey._wid = &yfms->mwindow.keys.keyid_atcc;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "";
    softkey._wid = &yfms->mwindow.keys.keyid_sfpl;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "FUEL";
    softkey._wid = &yfms->mwindow.keys.keyid_fuel;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "RADIO";
    softkey._wid = &yfms->mwindow.keys.keyid_radn;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "F-PLN";
    softkey._wid = &yfms->mwindow.keys.keyid_fpln;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    // row 3
    {
        softkey.inRT = keybordRT - separat1W - softkey.btnW * 4; // missing 4 columns
        softkey.inTP = keybordTP             - softkey.btnH * 3; // set top position
    }
    softkey.desc = "";
    softkey._wid = &yfms->mwindow.keys.keyid_null;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "";
    softkey._wid = &yfms->mwindow.keys.keyid_arpt;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    // row 4
    {
        softkey.inRT = keybordRT - separat1W - softkey.btnW * 4; // missing 4 columns
        softkey.inTP = keybordTP             - softkey.btnH * 4; // set top position
    }
    softkey.desc = "UP";
    softkey._wid = &yfms->mwindow.keys.keyid_lnup;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "<-";
    softkey._wid = &yfms->mwindow.keys.keyid_left;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    // row 5
    {
        softkey.inRT = keybordRT - separat1W - softkey.btnW * 4; // missing 4 columns
        softkey.inTP = keybordTP             - softkey.btnH * 5; // set top position
    }
    softkey.desc = "DOWN";
    softkey._wid = &yfms->mwindow.keys.keyid_lndn;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "->";
    softkey._wid = &yfms->mwindow.keys.keyid_rigt;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    // switch to narrower button type 2
    {
        softkey.btnW = YFS_SOFT_KEY_2_W; softkey.btnH = YFS_SOFT_KEY_2_H;
        softkey.btBW = softkey.btBH = YFS_SOFT_KEY_2_B;
    }
    // row 5, part 2
    {
        softkey.inRT = keybordRT - separat2W - softkey.btnW * 0; // all columns here
        softkey.inTP = keybordTP             - softkey.btnH * 5; // set top position
    }
    softkey.desc = "E";
    softkey._wid = &yfms->mwindow.keys.keyid_al_e;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "D";
    softkey._wid = &yfms->mwindow.keys.keyid_al_d;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "C";
    softkey._wid = &yfms->mwindow.keys.keyid_al_c;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "B";
    softkey._wid = &yfms->mwindow.keys.keyid_al_b;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "A";
    softkey._wid = &yfms->mwindow.keys.keyid_al_a;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    // row 6
    {
        softkey.inRT = keybordRT - separat2W - softkey.btnW * 0; // all columns here
        softkey.inTP = keybordTP             - softkey.btnH * 6; // set top position
    }
    softkey.desc = "J";
    softkey._wid = &yfms->mwindow.keys.keyid_al_j;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "I";
    softkey._wid = &yfms->mwindow.keys.keyid_al_i;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "H";
    softkey._wid = &yfms->mwindow.keys.keyid_al_h;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "G";
    softkey._wid = &yfms->mwindow.keys.keyid_al_g;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "F";
    softkey._wid = &yfms->mwindow.keys.keyid_al_f;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    // row 7
    {
        softkey.inRT = keybordRT - separat2W - softkey.btnW * 0; // all columns here
        softkey.inTP = keybordTP             - softkey.btnH * 7; // set top position
    }
    softkey.desc = "O";
    softkey._wid = &yfms->mwindow.keys.keyid_al_o;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "N";
    softkey._wid = &yfms->mwindow.keys.keyid_al_n;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "M";
    softkey._wid = &yfms->mwindow.keys.keyid_al_m;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "L";
    softkey._wid = &yfms->mwindow.keys.keyid_al_l;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "K";
    softkey._wid = &yfms->mwindow.keys.keyid_al_k;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
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
    // row 8
    {
        softkey.inRT = keybordRT - separat2W - softkey.btnW * 0; // all columns here
        softkey.inTP = keybordTP             - softkey.btnH * 8; // set top position
    }
    softkey.desc = "T";
    softkey._wid = &yfms->mwindow.keys.keyid_al_t;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "S";
    softkey._wid = &yfms->mwindow.keys.keyid_al_s;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "R";
    softkey._wid = &yfms->mwindow.keys.keyid_al_r;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "Q";
    softkey._wid = &yfms->mwindow.keys.keyid_al_q;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "P";
    softkey._wid = &yfms->mwindow.keys.keyid_al_p;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "6";
    softkey._wid = &yfms->mwindow.keys.keyid_num6;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "5";
    softkey._wid = &yfms->mwindow.keys.keyid_num5;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "4";
    softkey._wid = &yfms->mwindow.keys.keyid_num4;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    // row 9
    {
        softkey.inRT = keybordRT - separat2W - softkey.btnW * 0; // all columns here
        softkey.inTP = keybordTP             - softkey.btnH * 9; // set top position
    }
    softkey.desc = "Y";
    softkey._wid = &yfms->mwindow.keys.keyid_al_y;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "X";
    softkey._wid = &yfms->mwindow.keys.keyid_al_x;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "X";
    softkey._wid = &yfms->mwindow.keys.keyid_al_w;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "V";
    softkey._wid = &yfms->mwindow.keys.keyid_al_v;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "U";
    softkey._wid = &yfms->mwindow.keys.keyid_al_u;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "9";
    softkey._wid = &yfms->mwindow.keys.keyid_num9;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "8";
    softkey._wid = &yfms->mwindow.keys.keyid_num8;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "7";
    softkey._wid = &yfms->mwindow.keys.keyid_num7;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    // row 10
    {
        softkey.inRT = keybordRT - separat2W - softkey.btnW *  0; // all columns here
        softkey.inTP = keybordTP             - softkey.btnH * 10; // set top position
    }
    softkey.desc = "CLR";
    softkey._wid = &yfms->mwindow.keys.keyid_clir;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "OVFY";
    softkey._wid = &yfms->mwindow.keys.keyid_ovfy;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "SP";
    softkey._wid = &yfms->mwindow.keys.keyid_spce;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "/";
    softkey._wid = &yfms->mwindow.keys.keyid_slsh;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "Z";
    softkey._wid = &yfms->mwindow.keys.keyid_al_z;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "+/-";
    softkey._wid = &yfms->mwindow.keys.keyid_plus;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "0";
    softkey._wid = &yfms->mwindow.keys.keyid_num0;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = ".";
    softkey._wid = &yfms->mwindow.keys.keyid_pird;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }

    /*
     * Now the MCDU's screen sub-window and associated labels.
     */
    int align_scW = (YFS_MAINWINDOW_W - YFS_MAINSCREEN_W) / 2;
    inTP = keybordTP - 1 + YFS_MAINSCREEN_H;    // top
    inRT = mainwinRT + 1 - align_scW;           // right
    inLT =   inRT - YFS_MAINSCREEN_W;           // left
    for (int i = 0; i < YFS_DISPLAY_NUMR; i++)
    {
        inBM = inTP - YFS_FONT_BASIC_H; // set height for each line here
        yfms->mwindow.screen.ln_inBM[i] = inBM;
        yfms->mwindow.screen.ln_inLT[i] = inLT;
        yfms->mwindow.screen.ln_inTP[i] = inTP;
        yfms->mwindow.screen.ln_inRT[i] = inRT;
        inTP                            = inBM; // next line starts below this
#if 0
        ndt_log("YFMS [debug]: (%d, %d) -> (%d, %d) (width: %d, height: %d)\n",
                yfms->mwindow.screen.ln_inBM[i],
                yfms->mwindow.screen.ln_inLT[i],
                yfms->mwindow.screen.ln_inTP[i],
                yfms->mwindow.screen.ln_inRT[i],
                yfms->mwindow.screen.ln_inRT[i] -
                yfms->mwindow.screen.ln_inLT[i],
                yfms->mwindow.screen.ln_inTP[i] -
                yfms->mwindow.screen.ln_inBM[i]);
#endif
    }
    {
        // XPLMDrawString will write some pixels below bottom (e.g. underscores)
        yfms->mwindow.screen.sw_inBM = yfms->mwindow.screen.ln_inBM[YFS_DISPLAY_NUMR - 1] - 6;
        yfms->mwindow.screen.sw_inLT = yfms->mwindow.screen.ln_inLT[0];
        yfms->mwindow.screen.sw_inTP = yfms->mwindow.screen.ln_inTP[0];
        yfms->mwindow.screen.sw_inRT = yfms->mwindow.screen.ln_inRT[0];
    }
    if ((yfms->mwindow.screen.bgrd_id = // add first, draw first
         XPCreateWidget(yfms->mwindow.screen.sw_inLT, yfms->mwindow.screen.sw_inTP,
                        yfms->mwindow.screen.sw_inRT, yfms->mwindow.screen.sw_inBM,
                        1, "", 0, yfms->mwindow.id, xpWidgetClass_SubWindow)) &&
        (yfms->mwindow.screen.subw_id = // draw after background
         XPCreateWidget(yfms->mwindow.screen.sw_inLT, yfms->mwindow.screen.sw_inTP,
                        yfms->mwindow.screen.sw_inRT, yfms->mwindow.screen.sw_inBM,
                        1, "", 0, yfms->mwindow.id, xpWidgetClass_SubWindow)))
    {
        XPSetWidgetProperty(yfms->mwindow.screen.subw_id, xpProperty_Refcon, (intptr_t)yfms);
        XPAddWidgetCallback(yfms->mwindow.screen.subw_id, &yfs_mcdudish);
        XPAddWidgetCallback(yfms->mwindow.screen.bgrd_id, &yfs_mcdubgrh);
    }
    else
    {
        ndt_log("YFMS [error]: could not create MCDU display sub-window\n");
        return -1;
    }
    for (int i = 0; i < YFS_DISPLAY_NUMR; i++)
    {
        if ((yfms->mwindow.screen.line_id[i] =
             XPCreateWidget(yfms->mwindow.screen.ln_inLT[i] + 1 + YFS_FONT_BASIC_W,
                            yfms->mwindow.screen.ln_inTP[i] - 1,
                            yfms->mwindow.screen.ln_inRT[i] - 1 - YFS_FONT_BASIC_W,
                            yfms->mwindow.screen.ln_inBM[i] + 1,
                            1, "", 0, yfms->mwindow.id, xpWidgetClass_Caption)) == NULL)
        {
            ndt_log("YFMS [error]: could not create MCDU display, line %d\n", i + 1);
            return -1;
        }
        if (i >= 2 && i % 2 == 0) // line select keys here
        {
            softkey.btBW = softkey.btBH = YFS_SOFT_KEY_2_B;
            softkey.btnW                = YFS_SOFT_KEY_2_W;
            softkey.btnH                = YFS_FONT_BASIC_H;
            softkey.inTP = yfms->mwindow.screen.ln_inTP[i] - 3;
            softkey.inRT = yfms->mwindow.screen.ln_inLT[i] - YFS_SOFT_KEY_2_B;
            softkey.inLT = softkey.inRT - softkey.btnW + 1;
            softkey.desc = "-"; softkey._wid = &yfms->mwindow.keys.keyid_lsk[i/2-1];
            if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
            {
                goto create_button_fail;
            }
            softkey.inLT = yfms->mwindow.screen.ln_inRT[i] + 2 * YFS_SOFT_KEY_2_B;
            softkey.inRT = softkey.inLT + softkey.btnW - 1;
            softkey.desc = "-"; softkey._wid = &yfms->mwindow.keys.keyid_rsk[i/2-1];
            if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
            {
                goto create_button_fail;
            }
        }
    }

    /* all good */
    yfs_menu_resetall(yfms); return 0;

create_button_fail:
    ndt_log("YFMS [error]: could not create button with description \"%s\"\n", softkey.desc);
    return r_value;
}

static void toggle_main_window(yfms_context *yfms)
{
    if (!yfms)
    {
        return;
    }
    if (yfms->mwindow.win_state == 0) // place window near top left by default
    {
        int windowTP, windowLT = 0;
        yfms->mwindow.win_state += 1;
        XPLMGetScreenSize(NULL, &windowTP);
        int inTP = (windowTP - 100); int inBM = (inTP - YFS_MAINWINDOW_H) + 1;
        int inLT = (windowLT +  50); int inRT = (inLT + YFS_MAINWINDOW_W) - 1;
        XPSetWidgetGeometry(yfms->mwindow.id, inLT, inTP, inRT, inBM);
    }
    if (XPIsWidgetVisible(yfms->mwindow.id))
    {
        XPHideWidget       (yfms->mwindow.id);
        XPLoseKeyboardFocus(yfms->mwindow.id);
        return;
    }
    XPShowWidget            (yfms->mwindow.id);
    XPSetKeyboardFocus      (yfms->mwindow.id);
    XPBringRootWidgetToFront(yfms->mwindow.id);
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

    /* easy to use X-Plane command to toggle YFMS window */
    if ((yfms->toggle.command =
         XPLMCreateCommand("YFMS/toggle", "toggle YFMS window")) == NULL)
    {
        ndt_log("YFMS [error]: could not create X-Plane command for toggle\n");
        goto fail;
    }
    XPLMRegisterCommandHandler(yfms->toggle.command,
                               yfms->toggle.handler = &chandler_tog,
                               yfms->toggle.before  = 0,
                               yfms->toggle.refcon  = yfms);

    /* also add a menu (with an additional toggle) */
    if ((yfms->menu.id = XPLMCreateMenu("YFMS", NULL, 0, &menu_handler, yfms)) == NULL)
    {
        ndt_log("YFMS [error]: could not create menu\n");
        goto fail;
    }
    if ((yfms->menu.items.toggle = XPLMAppendMenuItem(yfms->menu.id, "toggle", &yfms->menu.items.toggle, 0)) < 0)
    {
        ndt_log("YFMS [error]: could not create menu item for toggle\n");
        goto fail;
    }

    /* all good */
    return yfms;

fail:
    yfs_main_close(&yfms);
    return NULL;
}

void yfs_main_rline(yfms_context *yfms, int idx, int col)
{
    if  (yfms && idx < YFS_DISPLAY_NUMR)
    {
        for (int i = 0; i < YFS_DISPLAY_NUMC; i++)
        {
            yfms->mwindow.screen.colr[idx][i] = col >= 0 ? col : COLR_IDX_GREEN;
        }
        memset(yfms->mwindow.screen.text[idx], 0, YFS_DISPLAY_NUMC + 1);
    }
    return;
}

int yfs_main_close(yfms_context **_yfms)
{
    yfms_context *yfms = *_yfms; *_yfms = NULL;
    if (!yfms)
    {
        return -1;
    }

    /* destroy the main window */
    if (yfms->mwindow.id)
    {
        XPDestroyWidget(yfms->mwindow.id, 1); // destroy children recursively
    }

    /* remove custom command(s) */
    if (yfms->toggle.command && yfms->toggle.handler)
    {
        XPLMUnregisterCommandHandler(yfms->toggle.command,
                                     yfms->toggle.handler,
                                     yfms->toggle.before,
                                     yfms->toggle.refcon);
        yfms->toggle.command = NULL;
        yfms->toggle.handler = NULL;
    }

    /* and the menu */
    if (yfms->menu.id)
    {
        XPLMDestroyMenu(yfms->menu.id);
    }

    /* all good */
    free(yfms);
    return 0;
}

void yfs_main_toggl(yfms_context *yfms)
{
    if (!yfms)
    {
        return;
    }
    toggle_main_window(yfms);
    return;
}

static void menu_handler(void *inMenuRef, void *inItemRef)
{
    yfms_context *yfms = inMenuRef;
    int *menu_item_ref = inItemRef;
    if (!yfms || !menu_item_ref)
    {
        return;
    }
    if (*menu_item_ref == yfms->menu.items.toggle)
    {
        toggle_main_window(yfms);
        return;
    }
    return;
}

static int yfs_mwindowh(XPWidgetMessage inMessage,
                        XPWidgetID      inWidget,
                        intptr_t        inParam1,
                        intptr_t        inParam2)
{
    if (inMessage == xpMessage_CloseButtonPushed)
    {
        XPHideWidget       (inWidget);
        XPLoseKeyboardFocus(inWidget);
        return 1;
    }
    if (inMessage == xpMsg_PushButtonPressed)
    {
        yfms_context  *yfms = (yfms_context*)XPGetWidgetProperty(inWidget, xpProperty_Refcon, NULL);
        yfs_keypressed(yfms,  (XPWidgetID)inParam1);
        return 1;
    }
    return 0;
}

static int yfs_mcdubgrh(XPWidgetMessage inMessage,
                        XPWidgetID      inWidget,
                        intptr_t        inParam1,
                        intptr_t        inParam2)
{
    if (inMessage == xpMsg_Draw)
    {
        if (XPIsWidgetVisible(inWidget))
        {
            int g[4];
            XPGetWidgetGeometry(inWidget, &g[0], &g[1], &g[2], &g[3]);
            XPLMDrawTranslucentDarkBox(    g[0],  g[1],  g[2],  g[3]);
        }
        return 1;
    }
    return 0;
}

static void draw_display(yfms_context *yfms)
{
    char buf[2]; int x, y;
    for (int i = 0; i < YFS_DISPLAY_NUMR; i++)
    {
        XPGetWidgetGeometry(yfms->mwindow.screen.line_id[i], &x, NULL, NULL, &y);
        for (int j = strlen(yfms->mwindow.screen.text[i]); j < YFS_DISPLAY_NUMC; j++)
        {
            yfms->mwindow.screen.text[i][j] = ' ';
        }
        for (int j = 0; j < YFS_DISPLAY_NUMC; j++)
        {
            snprintf(buf, sizeof(buf), "%c", yfms->mwindow.screen.text[i][j]);
            if (i == YFS_DISPLAY_NUMR - 1 && *buf == '_')
            {
                 // scratchpad parser strips trailing spaces, we use underscores instead
                *buf = ' '; // display the intended character, not the stored workaround
            }
            switch  (yfms->mwindow.screen.colr[i][j])
            {
                case COLR_IDX_BLACK:
                    XPLMDrawString(COLR_BLACK,   x, y, buf, NULL, xplmFont_Basic);
                    break;
                case COLR_IDX_BLUE:
                    XPLMDrawString(COLR_BLUE,    x, y, buf, NULL, xplmFont_Basic);
                    break;
                case COLR_IDX_MAGENTA:
                    XPLMDrawString(COLR_MAGENTA, x, y, buf, NULL, xplmFont_Basic);
                    break;
                case COLR_IDX_ORANGE:
                    XPLMDrawString(COLR_ORANGE,  x, y, buf, NULL, xplmFont_Basic);
                    break;
                case COLR_IDX_RED:
                    XPLMDrawString(COLR_RED,     x, y, buf, NULL, xplmFont_Basic);
                    break;
                case COLR_IDX_WHITE:
                    XPLMDrawString(COLR_WHITE,   x, y, buf, NULL, xplmFont_Basic);
                    break;
                case COLR_IDX_YELLOW:
                    XPLMDrawString(COLR_YELLOW,  x, y, buf, NULL, xplmFont_Basic);
                    break;
                case COLR_IDX_GREEN:
                default:
                    XPLMDrawString(COLR_GREEN,   x, y, buf, NULL, xplmFont_Basic);
                    break;
            }
            x += YFS_FONT_BASIC_W; // move position for next character
        }
    }
}

static int yfs_mcdudish(XPWidgetMessage inMessage,
                        XPWidgetID      inWidget,
                        intptr_t        inParam1,
                        intptr_t        inParam2)
{
    if (inMessage == xpMsg_Draw)
    {
        if (XPIsWidgetVisible(inWidget))
        {
            int y, x; yfms_context *yfms = (yfms_context*)XPGetWidgetProperty(inWidget, xpProperty_Refcon, NULL);
            if (yfms == NULL)
            {
                ndt_log("YFMS [debug]: no context for MCDU display!\n");
                return 0;
            }
            draw_display(yfms);
        }
        return 1;
    }
    return 0;
}

static int chandler_tog(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    yfms_context  *yfms = inRefcon;
    yfs_main_toggl(yfms);
    return 0;
}
