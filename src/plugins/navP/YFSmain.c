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
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#if IBM
#include <windows.h>
#endif
#if APL
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#ifndef GL_BGRA
#define GL_BGRA 0x80E1
#endif

#include "cairo/cairo.h"
#if CAIRO_HAS_FT_FONT
#include "cairo/cairo-ft.h"
#include "ft2build.h"
#include FT_FREETYPE_H
#endif

#include "Widgets/XPStandardWidgets.h"
#include "Widgets/XPWidgetDefs.h"
#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDisplay.h"
#include "XPLM/XPLMGraphics.h"
#include "XPLM/XPLMMenus.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMProcessing.h"
#include "XPLM/XPLMUtilities.h"

#include "assert/includes.h"

#include "common/common.h"
#include "compat/compat.h"
#include "lib/flightplan.h"
#include "lib/navdata.h"
#include "lib/waypoint.h"
#include "wmm/wmm.h"

#include "YFSfpln.h"
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
static double COLR_BLACK  [] = { 0.0, 0.0, 0.0, };
static double COLR_WHITE  [] = { 0.9, 1.0, 1.0, };
static double COLR_RED    [] = { 1.0, 0.1, 0.1, };
static double COLR_GREEN  [] = { 0.0, 1.0, 0.5, };
static double COLR_BLUE   [] = { 0.0, 1.0, 1.0, };
static double COLR_MAGENTA[] = { 1.0, .33, 1.0, };
static double COLR_ORANGE [] = { 1.0, 0.5, 0.0, };
static double COLR_YELLOW [] = { 0.9, 0.9, 0.0, };
static double COLR_SCREEN [] = { 0.2, 0.2, 0.2, };

static void menu_handler(void *inMenuRef,                void *inItemRef);
static int  yfs_mwindowh(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
static int  yfs_mcdubgrh(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
static int  yfs_mcdudish(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
static int  yfs_captionh(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
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
#define CAIRO4SCRN(FIELD) (yfms->mwindow.screen.cairo.FIELD)
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
    softkey._wid = &yfms->mwindow.keys.keyid_drto;
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
    softkey.desc = "ATC";
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
    softkey.desc = "W";
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
        XPSetWidgetProperty(yfms->mwindow.screen.bgrd_id, xpProperty_Refcon, (intptr_t)yfms);
        XPSetWidgetProperty(yfms->mwindow.screen.subw_id, xpProperty_Refcon, (intptr_t)yfms);
        XPAddWidgetCallback(yfms->mwindow.screen.bgrd_id, &yfs_mcdubgrh);
        XPAddWidgetCallback(yfms->mwindow.screen.subw_id, &yfs_mcdudish);
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
        else
        {
            XPAddWidgetCallback(yfms->mwindow.screen.line_id[i], &yfs_captionh);
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
            softkey.inLT = yfms->mwindow.screen.ln_inRT[i] + YFS_SOFT_KEY_2_B;
            softkey.inRT = softkey.inLT + softkey.btnW - 1;
            softkey.desc = "-"; softkey._wid = &yfms->mwindow.keys.keyid_rsk[i/2-1];
            if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
            {
                goto create_button_fail;
            }
        }
    }

    /* cairo: setup default font to be used if we cannot load anything better */
    CAIRO4SCRN(font) = cairo_toy_font_face_create("cairo:monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_status_t status = cairo_font_face_status(CAIRO4SCRN(font));
    CAIRO4SCRN(whitebox) = "□"; CAIRO4SCRN(fly_over) = "*";
    if (CAIRO_STATUS_SUCCESS != status)
    {
        ndt_log("YFMS [error]: cairo_font_face_status(\"%s\"): '%s'\n", "cairo:monospace", cairo_status_to_string(status));
        return -1;
    }

//#define USE_DEFAULT_FONT
#ifndef USE_DEFAULT_FONT
#if CAIRO_HAS_FT_FONT
    /* freetype: load better font if possible */
    FT_Error err = FT_Init_FreeType(&CAIRO4SCRN(ft.flib));
    if (0 != err)
    {
        ndt_log("YFMS [error]: FT_Init_FreeType: %d\n", err);
        return -1;
    }
    else
    {
        char *path = NULL; int ret, len; XPLMGetSystemPath(yfms->ndt.xsystem_pth);
        struct
        {
            int     relative;
            const char *wbox;
            const char *ofly;
            const char *path;
        }
        font_paths[] =
        {
            { 1, "□", "Δ", "Resources/plugins/YFMS/fonts/AnonymousPro-Bold.ttf", }, // https://fonts.google.com/specimen/Anonymous+Pro
            { 0, "□", "Δ", "/System/Library/Fonts/Monaco.dfont", },
            { 0, NULL, NULL, NULL, },
        };
        for (int i = 0; font_paths[i].path != NULL; i++)
        {
            if (0 == (ret = ndt_file_getpath(font_paths[i].relative ? yfms->ndt.xsystem_pth : NULL, font_paths[i].path, &path, &len)))
            {
                if (0 == (err = FT_New_Face(CAIRO4SCRN(ft.flib), path, 0, &CAIRO4SCRN(ft.face))) != 0)
                {
                    if (NULL == (CAIRO4SCRN(ft.font) = cairo_ft_font_face_create_for_ft_face(CAIRO4SCRN(ft.face), 0)))
                    {
                        ndt_log("YFMS [error]: cairo_ft_font_face_create_for_ft_face(\"%s\"): (NULL)\n", path);
                        FT_Done_Face(CAIRO4SCRN(ft.face)); CAIRO4SCRN(ft.face) = NULL;
                        continue;
                    }
                    if (CAIRO_STATUS_SUCCESS != (status = cairo_font_face_status(CAIRO4SCRN(ft.font))))
                    {
                        ndt_log("YFMS [error]: cairo_font_face_status(\"%s\"): '%s'\n", path, cairo_status_to_string(status));
                        FT_Done_Face(CAIRO4SCRN(ft.face)); CAIRO4SCRN(ft.face) = NULL;
                        continue;
                    }
                    CAIRO4SCRN(whitebox) = font_paths[i].wbox; CAIRO4SCRN(fly_over) = font_paths[i].ofly;
                    cairo_font_face_destroy(CAIRO4SCRN(font)); CAIRO4SCRN(font) = CAIRO4SCRN(ft.font);
                    ndt_log("YFMS [info]: loaded font file: \"%s\"\n", path);
                    break;
                }
            }
        }
        if (path)
        {
            free(path);
        }
    }
#endif
#endif

    /* cairo */
    CAIRO4SCRN(data.wid) = yfms->mwindow.screen.sw_inRT - yfms->mwindow.screen.sw_inLT + 2;
    CAIRO4SCRN(data.hei) = yfms->mwindow.screen.sw_inTP - yfms->mwindow.screen.sw_inBM + 2;
    CAIRO4SCRN(data.str) = cairo_format_stride_for_width(CAIRO_FORMAT_ARGB32, CAIRO4SCRN(data.wid));
    if (NULL == (CAIRO4SCRN(data.buf) = malloc(CAIRO4SCRN(data.str) * CAIRO4SCRN(data.hei))))
    {
        ndt_log("YFMS [error]: could not allocate buffer for cairo surface\n");
        return ENOMEM;
    }
    CAIRO4SCRN(surf) = cairo_image_surface_create_for_data(CAIRO4SCRN(data.buf), CAIRO_FORMAT_ARGB32, CAIRO4SCRN(data.wid), CAIRO4SCRN(data.hei), CAIRO4SCRN(data.str));
    if (CAIRO_STATUS_SUCCESS != (status = cairo_surface_status(CAIRO4SCRN(surf))))
    {
        ndt_log("YFMS [error]: cairo_surface_status: '%s'\n", cairo_status_to_string(status));
        return -1;
    }
    CAIRO4SCRN(cr) = cairo_create(CAIRO4SCRN(surf));
    if (CAIRO_STATUS_SUCCESS != (status = cairo_status(CAIRO4SCRN(cr))))
    {
        ndt_log("YFMS [error]: cairo_status: '%s'\n", cairo_status_to_string(status));
        return -1;
    }
    cairo_set_font_face(CAIRO4SCRN(cr), CAIRO4SCRN(font));
    if (CAIRO_STATUS_SUCCESS != (status = cairo_status(CAIRO4SCRN(cr))))
    {
        ndt_log("YFMS [error]: cairo_status: '%s'\n", cairo_status_to_string(status));
        return -1;
    }
    cairo_matrix_t matrix; cairo_get_font_matrix(CAIRO4SCRN(cr), &matrix); matrix.xx = (double)YFS_FONT_BASIC_W - 1.0;
    do // XXX: exhaustive but we only run it once
    {
        matrix.yy = -matrix.xx; cairo_set_font_matrix(CAIRO4SCRN(cr), &matrix);
        for (char c = 'A'; c <= 'Z'; c++)
        {
            char buf[2];
            cairo_text_extents_t te;
            snprintf(buf, sizeof(buf), "%c", c);
            cairo_text_extents(CAIRO4SCRN(cr), buf, &te);
            if (((double)YFS_FONT_BASIC_W + 0.1) < (te.width))
            {
                ndt_log("YFMS [debug]: '%s': size %04.1lf |"
                        " x_bearing %+04.1lf | w %04.1lf | x_advance %+04.1lf |"
                        " y_bearing %+04.1lf | h %04.1lf | y_advance %+04.1lf\n",
                        matrix.xx, buf, te.x_bearing, te.width, te.x_advance, te.y_bearing, te.height, te.y_advance);
                matrix.xx -= 1.0; matrix.yy = -matrix.xx;
                goto set_font_size;
            }
            if (((double)YFS_FONT_BASIC_H + 0.1) < (te.height))
            {
                ndt_log("YFMS [debug]: '%s': size %04.1lf |"
                        " x_bearing %+04.1lf | w %04.1lf | x_advance %+04.1lf |"
                        " y_bearing %+04.1lf | h %04.1lf | y_advance %+04.1lf\n",
                        matrix.xx, buf, te.x_bearing, te.width, te.x_advance, te.y_bearing, te.height, te.y_advance);
                matrix.xx -= 1.0; matrix.yy = -matrix.xx;
                goto set_font_size;
            }
            if (((double)YFS_FONT_BASIC_W + 0.1) < (te.x_advance + 1.0))
            {
                ndt_log("YFMS [debug]: '%s': size %04.1lf |"
                        " x_bearing %+04.1lf | w %04.1lf | x_advance %+04.1lf |"
                        " y_bearing %+04.1lf | h %04.1lf | y_advance %+04.1lf\n",
                        matrix.xx, buf, te.x_bearing, te.width, te.x_advance, te.y_bearing, te.height, te.y_advance);
                matrix.yy = -matrix.xx; // first font size with x_advance of 8.0
                goto set_font_size;
            }
            if (((double)YFS_FONT_BASIC_H + 0.1) < (te.y_bearing + te.height + 1.0))
            {
                ndt_log("YFMS [debug]: '%s': size %04.1lf |"
                        " x_bearing %+04.1lf | w %04.1lf | x_advance %+04.1lf |"
                        " y_bearing %+04.1lf | h %04.1lf | y_advance %+04.1lf\n",
                        matrix.xx, buf, te.x_bearing, te.width, te.x_advance, te.y_bearing, te.height, te.y_advance);
                matrix.xx -= 1.0; matrix.yy = -matrix.xx;
                goto set_font_size;
            }
        }
        matrix.xx += 1.0;
        continue;
    }
    while (1);

set_font_size:
    cairo_set_font_matrix(CAIRO4SCRN(cr), &matrix);
    CAIRO4SCRN(sfnt) = cairo_get_scaled_font(CAIRO4SCRN(cr));
    ndt_log("YFMS [info]: computed font size %lf\n", matrix.xx);
#if 0
    for (char c = '!'; c <= '~'; c++)
    {
        char buf[2]; snprintf(buf, sizeof(buf), "%c", c);
        cairo_text_extents_t te; cairo_text_extents(CAIRO4SCRN(cr), buf, &te);
        ndt_log("YFMS [debug]: '%s': size %04.1lf |"
                " x_bearing %+04.1lf | w %04.1lf | x_advance %+04.1lf |"
                " y_bearing %+04.1lf | h %04.1lf | y_advance %+04.1lf\n",
                matrix.xx, buf, te.x_bearing, te.width, te.x_advance, te.y_bearing, te.height, te.y_advance);
    }
#endif

    /*
     * Predefined regions for mouse wheel and click support,
     * set relative to the main display bottom right corner:
     *
     * xmin == ymin == 0
     * xmax == YFS_MAINSCREEN_W
     * ymax == YFS_MAINSCREEN_H
     */
    double rowheight = (((double)YFS_MAINSCREEN_H) / (14.));
    int column_width = (((YFS_MAINSCREEN_W) * (3)) / (10)); // ~30% * 3 + ~10%
    for (int row = 0; row < 6; row++)
    {
        int lsk_center = ((int)((rowheight) * ((11.5) - ((double)(row * 2)))));     // 11.5 -> 1.5 rows
        yfms->mouse_regions[row][0].ymin =
        yfms->mouse_regions[row][1].ymin =
        yfms->mouse_regions[row][2].ymin = (lsk_center - YFS_FONT_BASIC_H);
        yfms->mouse_regions[row][0].ymax =
        yfms->mouse_regions[row][1].ymax =
        yfms->mouse_regions[row][2].ymax = (lsk_center + YFS_FONT_BASIC_H);
        yfms->mouse_regions[row][0].xmin = (0);                                     // left-aligned
        yfms->mouse_regions[row][0].xmax = (column_width);                          // left-aligned
        yfms->mouse_regions[row][2].xmax = (YFS_MAINSCREEN_W);                      // right-aligned
        yfms->mouse_regions[row][2].xmin = (YFS_MAINSCREEN_W - column_width);       // right-aligned
        yfms->mouse_regions[row][1].xmin = (YFS_MAINSCREEN_W - column_width) / 2;   // fully-centered
        yfms->mouse_regions[row][1].xmax = (YFS_MAINSCREEN_W + column_width) / 2;   // fully-centered
    }

    /* all good */
    return 0;

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
    if (yfms->mwindow.win_state == 0) // top left, adjusted for XSB & X-IvAp
    {
        int windowTP, windowLT = 0;
        yfms->mwindow.win_state += 1;
        XPLMGetScreenSize(NULL, &windowTP);
        int inTP = (windowTP - 200); int inBM = (inTP - YFS_MAINWINDOW_H) + 1;
        int inLT = (windowLT +  20); int inRT = (inLT + YFS_MAINWINDOW_W) - 1;
        XPSetWidgetGeometry(yfms->mwindow.id, inLT, inTP, inRT, inBM);
    }
    if (XPIsWidgetVisible(yfms->mwindow.id))
    {
        XPHideWidget       (yfms->mwindow.id);
        XPLoseKeyboardFocus(yfms->mwindow.id);
        return;
    }
    /*
     * First time displayed since reset: miscellaneous setup.
     */
    if (yfms->xpl.atyp == YFS_ATYP_NSET)
    {
#if 0
        /*
         * Make sure there is no conflict between our "toggle"
         * keysniffer and one or more plugin-defined hotkey(s).
         *
         * enum
         * {
         *     AW_D_KEY_DEFAULT = XPLM_VK_TAB,
         *     AW_D_KEY_LTERNAT = XPLM_VK_RETURN,
         * }   aw_d_vkey;
         */
        if (AW_D_KEY_DEFAULT != yfms->mwindow.aw_d_vkey)
        {
            yfms->mwindow.aw_d_vkey = AW_D_KEY_DEFAULT;
            ndt_log("YFMS [info]: first key press, re-enable default toggle key\n");
        }
        if (XPLM_NO_PLUGIN_ID == XPLMFindPluginBySignature("com.pilotedge.plugin.xplane"))
        {
            for (int i = 0; i < XPLMCountHotKeys(); i++)
            {
                XPLMHotKeyID hotkey = XPLMGetNthHotKey(i);
                if (hotkey)
                {
                    char ovk; XPLMGetHotKeyInfo(hotkey, &ovk, NULL, NULL, NULL);
                    if (AW_D_KEY_DEFAULT == ovk)
                    {
                        ndt_log("YFMS [info]: first key press, now using alternate toggle key\n");
                        yfms->mwindow.aw_d_vkey = AW_D_KEY_LTERNAT;
                        break;
                    }
                }
            }
        }
#endif
        /*
         * Check all (supported) custom datarefs and commands;
         * use them to determine custom aircraft type, if any.
         */
        do
        {
            yfms->xpl.ixeg.xpdr_mode_act          = XPLMFindDataRef("ixeg/733/xpdr/xpdr_mode_act"               );
            yfms->xpl.ixeg.xpdr_stby_act          = XPLMFindDataRef("ixeg/733/xpdr/xpdr_stby_act"               );
            yfms->xpl.ixeg.radios_adf1_100_act    = XPLMFindDataRef("ixeg/733/radios/radios_adf1_100_act"       );
            yfms->xpl.ixeg.radios_adf1_010_act    = XPLMFindDataRef("ixeg/733/radios/radios_adf1_010_act"       );
            yfms->xpl.ixeg.radios_adf1_001_act    = XPLMFindDataRef("ixeg/733/radios/radios_adf1_001_act"       );
            yfms->xpl.ixeg.radios_adf2_100_act    = XPLMFindDataRef("ixeg/733/radios/radios_adf2_100_act"       );
            yfms->xpl.ixeg.radios_adf2_010_act    = XPLMFindDataRef("ixeg/733/radios/radios_adf2_010_act"       );
            yfms->xpl.ixeg.radios_adf2_001_act    = XPLMFindDataRef("ixeg/733/radios/radios_adf2_001_act"       );
            yfms->xpl.ixeg.baro_inhg_sby_0001_ind = XPLMFindDataRef("ixeg/733/altimeter/baro_inhg_sby_0001_ind" );
            yfms->xpl.qpac.XPDR[0]                = XPLMFindDataRef("AirbusFBW/XPDR4"                           );
            yfms->xpl.qpac.XPDR[1]                = XPLMFindDataRef("AirbusFBW/XPDR3"                           );
            yfms->xpl.qpac.XPDR[2]                = XPLMFindDataRef("AirbusFBW/XPDR2"                           );
            yfms->xpl.qpac.XPDR[3]                = XPLMFindDataRef("AirbusFBW/XPDR1"                           );
            yfms->xpl.qpac.XPDRPower              = XPLMFindDataRef("AirbusFBW/XPDRPower"                       );
            yfms->xpl.qpac.XPDRAltitude           = XPLMFindDataRef("AirbusFBW/XPDRAltitude"                    );
            yfms->xpl.qpac.XPDRTCASMode           = XPLMFindDataRef("AirbusFBW/XPDRTCASMode"                    );
            yfms->xpl.qpac.XPDRTCASAltSelect      = XPLMFindDataRef("AirbusFBW/XPDRTCASAltSelect"               );
            yfms->xpl.qpac.BaroUnitCapt           = XPLMFindDataRef("AirbusFBW/BaroUnitCapt"                    );
            yfms->xpl.qpac.BaroStdCapt            = XPLMFindDataRef("AirbusFBW/BaroStdCapt"                     );
            yfms->xpl.qpac.BaroUnitFO             = XPLMFindDataRef("AirbusFBW/BaroUnitFO"                      );
            yfms->xpl.qpac.BaroStdFO              = XPLMFindDataRef("AirbusFBW/BaroStdFO"                       );
            yfms->xpl.qpac.RMPSwapCapt            = XPLMFindCommand("AirbusFBW/RMPSwapCapt"                     );
            yfms->xpl.qpac.RMPSwapCo              = XPLMFindCommand("AirbusFBW/RMPSwapCo"                       );
            yfms->xpl.qpac.VHF1Capt               = XPLMFindCommand("AirbusFBW/VHF1Capt"                        );
            yfms->xpl.qpac.VHF2Co                 = XPLMFindCommand("AirbusFBW/VHF2Co"                          );
            yfms->xpl.qpac.RMP1FreqUpLrg          = XPLMFindCommand("AirbusFBW/RMP1FreqUpLrg"                   );
            yfms->xpl.qpac.RMP1FreqUpSml          = XPLMFindCommand("AirbusFBW/RMP1FreqUpSml"                   );
            yfms->xpl.qpac.RMP1FreqDownLrg        = XPLMFindCommand("AirbusFBW/RMP1FreqDownLrg"                 );
            yfms->xpl.qpac.RMP1FreqDownSml        = XPLMFindCommand("AirbusFBW/RMP1FreqDownSml"                 );
            yfms->xpl.qpac.RMP2FreqUpLrg          = XPLMFindCommand("AirbusFBW/RMP2FreqUpLrg"                   );
            yfms->xpl.qpac.RMP2FreqUpSml          = XPLMFindCommand("AirbusFBW/RMP2FreqUpSml"                   );
            yfms->xpl.qpac.RMP2FreqDownLrg        = XPLMFindCommand("AirbusFBW/RMP2FreqDownLrg"                 );
            yfms->xpl.qpac.RMP2FreqDownSml        = XPLMFindCommand("AirbusFBW/RMP2FreqDownSml"                 );
            yfms->xpl.q350.pressModeLeft          = XPLMFindDataRef("1-sim/pres/pressModeLeft/switch"           );
            yfms->xpl.q350.pressModeRight         = XPLMFindDataRef("1-sim/pres/pressModeRight/switch"          );
            yfms->xpl.q350.pressLeftButton        = XPLMFindDataRef("1-sim/pres/pressLeftButton"                );
            yfms->xpl.q350.pressLeftRotary        = XPLMFindDataRef("1-sim/pres/pressLeftRotary"                );
            yfms->xpl.q350.pressRightButton       = XPLMFindDataRef("1-sim/pres/pressRightButton"               );
            yfms->xpl.q350.pressRightRotary       = XPLMFindDataRef("1-sim/pres/pressRightRotary"               );
            yfms->xpl.q380.BaroUnitCapt           = XPLMFindDataRef("com/petersaircraft/airbus/BaroUnitCapt"    );
            yfms->xpl.q380.BaroStdCapt            = XPLMFindDataRef("com/petersaircraft/airbus/BaroStdCapt"     );
            yfms->xpl.q380.BaroUnitFO             = XPLMFindDataRef("com/petersaircraft/airbus/BaroUnitFO"      );
            yfms->xpl.q380.BaroStdFO              = XPLMFindDataRef("com/petersaircraft/airbus/BaroStdFO"       );
            yfms->xpl.q380.PeterCRZ               = XPLMFindDataRef("com/petersaircraft/airbus/CruiseFL"        );
            yfms->xpl.q380.PeterFLX               = XPLMFindDataRef("com/petersaircraft/airbus/FlexTemp"        );
            yfms->xpl.q380.PeterCI                = XPLMFindDataRef("com/petersaircraft/airbus/CostIndex"       );
            yfms->xpl.q380.PeterV1                = XPLMFindDataRef("com/petersaircraft/airbus/V1Setting"       );
            yfms->xpl.q380.PeterV2                = XPLMFindDataRef("com/petersaircraft/airbus/V2Setting"       );
            yfms->xpl.q380.PeterVR                = XPLMFindDataRef("com/petersaircraft/airbus/VRSetting"       );
            yfms->xpl.fb76.systemMode             = XPLMFindDataRef("1-sim/transponder/systemMode"              );
            yfms->xpl.fb76.baroRotary_stby        = XPLMFindDataRef("1-sim/gauges/baroRotary_stby"              );
            yfms->xpl.fb76.baroRotary_left        = XPLMFindDataRef("1-sim/gauges/baroRotary_left"              );
            yfms->xpl.fb76.baroRotary_right       = XPLMFindDataRef("1-sim/gauges/baroRotary_right"             );
            yfms->xpl.fb76.nav2_nav_id            = XPLMFindDataRef("1-sim/radios/nav3_nav_id"                  );
            yfms->xpl.fb76.nav2_frequency_hz      = XPLMFindDataRef("1-sim/radios/nav3_frequency_hz"            );
            yfms->xpl.fb76.nav2_obs_deg_mag_pilot = XPLMFindDataRef("1-sim/radios/nav3_obs_deg_mag_pilot"       );
            yfms->xpl.fb76.leftBigRotary          = XPLMFindDataRef("1-sim/adf/leftBigRotary"                   );
            yfms->xpl.fb76.leftMidRotary          = XPLMFindDataRef("1-sim/adf/leftMidRotary"                   );
            yfms->xpl.fb76.leftSmallRotary        = XPLMFindDataRef("1-sim/adf/leftSmallRotary"                 );
            yfms->xpl.fb76.rightBigRotary         = XPLMFindDataRef("1-sim/adf/rightBigRotary"                  );
            yfms->xpl.fb76.rightMidRotary         = XPLMFindDataRef("1-sim/adf/rightMidRotary"                  );
            yfms->xpl.fb76.rightSmallRotary       = XPLMFindDataRef("1-sim/adf/rightSmallRotary"                );
            yfms->xpl.fb77.anim_25_rotery         = XPLMFindDataRef("anim/25/rotery"                            );
            yfms->xpl.fb77.anim_67_switch         = XPLMFindDataRef("anim/67/switch"                            );
            yfms->xpl.fb77.anim_85_switch         = XPLMFindDataRef("anim/85/switch"                            );
            yfms->xpl.fb77.anim_175_button        = XPLMFindDataRef("anim/175/button"                           );
            yfms->xpl.tekt.cl3_fms_selector       = XPLMFindDataRef("cl300/fms_selector"                        );
            yfms->xpl.tekt.CruiseSpeed_Mach       = XPLMFindDataRef("Tekton_FMS/CruiseSpeed_Mach"               );
            yfms->xpl.tekt.CruiseSpeed_KIAS       = XPLMFindDataRef("Tekton_FMS/CruiseSpeed_KIAS"               );
            yfms->xpl.tekt.HA4T_shared_KIAS       = XPLMFindDataRef("Hawker4000/shared/KIAS_MACH"               );
            yfms->xpl.tekt.E175_mouse_x_pos       = XPLMFindDataRef("XCrafts/ERJ_175/mouse_x_pos"               );
            yfms->xpl.tekt.E195_mouse_x_pos       = XPLMFindDataRef("XCrafts/ERJ_195/mouse_x_pos"               );

           /*
            * FlightFactor A320 Ultimate
            *
            * s32 Aircraft.FMGS.FCU1.Lateral                           heading     DEG
            * s32 Aircraft.FMGS.FCU1.Vertical                          v/speed     100FT
            * s32 Aircraft.FMGS.FCU1.Altitude                          altitude    100FT
            * s32 Aircraft.FMGS.FCU1.Speed                             airspeed    KTS/0.01M
            * s32 Aircraft.FMGS.FCU1.SpeedIsMach                                   boolean
            * s32 Aircraft.Cockpit.Panel.FCU_Mach.Click                            boolean
            * s32 Aircraft.FMGS.FCU1.BaroL                             pressure    100INHG/1HPA
            * s32 Aircraft.FMGS.FCU1.BaroR                             pressure    100INHG/1HPA
            * s32 Aircraft.FMGS.FCU1.BaroModeL                         QNH:2/STD:-2/QFE:1/STD:-1
            * s32 Aircraft.FMGS.FCU1.BaroModeR                         QNH:2/STD:-2/QFE:1/STD:-1
            * u32 Aircraft.Cockpit.Panel.EFIS_BaroTypeL.Target         InHg/hPa    0/1
            * f32 Aircraft.Cockpit.Panel.EFIS_BaroTypeL.Position       InHg/hPa    0/1
            * u32 Aircraft.Cockpit.Panel.EFIS_BaroTypeR.Target         InHg/hPa    0/1
            * f32 Aircraft.Cockpit.Panel.EFIS_BaroTypeL.Position       InHg/hPa    0/1
            * u32 Aircraft.Cockpit.Pedestal.TCAS_Traffic.Target        index       0-based
            * u32 Aircraft.Cockpit.Pedestal.TCAS_Show.Target           index       0-based
            * u32 Aircraft.Cockpit.Pedestal.ATC_Mode.Target            index       0-based
            * u32 Aircraft.Cockpit.Pedestal.ATC_Alt.Target             index       0-based
            * u32 Aircraft.Navigation.ATC.CodeSet                      squawk      4-digit
            */
            if (XPLM_NO_PLUGIN_ID != (yfms->xpl.asrt.xid = XPLMFindPluginBySignature(XPLM_FF_SIGNATURE)))
            {
                XPLMSendMessageToPlugin(yfms->xpl.asrt.xid, XPLM_FF_MSG_GET_SHARED_INTERFACE, &yfms->xpl.asrt.api);
                if (yfms->xpl.asrt.api.DataVersion == NULL || yfms->xpl.asrt.api.DataAddUpdate == NULL)
                {
                    ndt_log("YFMS [warning]: couldn't initialize \"%s\" API\n", XPLM_FF_SIGNATURE);
                    yfms->xpl.atyp = YFS_ATYP_XPLN; break;
                }
                yfms->xpl.asrt.baro.id_s32_lvalu = yfms->xpl.asrt.api.ValueIdByName("Aircraft.FMGS.FCU1.BaroL");
                yfms->xpl.asrt.baro.id_s32_rvalu = yfms->xpl.asrt.api.ValueIdByName("Aircraft.FMGS.FCU1.BaroR");
                yfms->xpl.asrt.baro.id_s32_lmode = yfms->xpl.asrt.api.ValueIdByName("Aircraft.FMGS.FCU1.BaroModeL");
                yfms->xpl.asrt.baro.id_s32_rmode = yfms->xpl.asrt.api.ValueIdByName("Aircraft.FMGS.FCU1.BaroModeR");
                yfms->xpl.asrt.baro.id_u32_lunit = yfms->xpl.asrt.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_BaroTypeL.Target");
                yfms->xpl.asrt.baro.id_u32_runit = yfms->xpl.asrt.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_BaroTypeR.Target");
                yfms->xpl.asrt.baro.id_f32_lunip = yfms->xpl.asrt.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_BaroTypeL.Position");
                yfms->xpl.asrt.baro.id_f32_runip = yfms->xpl.asrt.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_BaroTypeR.Position");
                yfms->xpl.asrt.xpdr.id_u32_code  = yfms->xpl.asrt.api.ValueIdByName("Aircraft.Navigation.ATC.CodeSet");
                yfms->xpl.asrt.xpdr.id_u32_altr  = yfms->xpl.asrt.api.ValueIdByName("Aircraft.Cockpit.Pedestal.ATC_Alt.Target");
                yfms->xpl.asrt.xpdr.id_u32_mode  = yfms->xpl.asrt.api.ValueIdByName("Aircraft.Cockpit.Pedestal.ATC_Mode.Target");
                yfms->xpl.asrt.xpdr.id_u32_tmod  = yfms->xpl.asrt.api.ValueIdByName("Aircraft.Cockpit.Pedestal.TCAS_Show.Target");
                yfms->xpl.asrt.xpdr.id_u32_tcas  = yfms->xpl.asrt.api.ValueIdByName("Aircraft.Cockpit.Pedestal.TCAS_Traffic.Target");
                if (yfms->xpl.asrt.baro.id_s32_lvalu <= 0 ||
                    yfms->xpl.asrt.baro.id_s32_rvalu <= 0 ||
                    yfms->xpl.asrt.baro.id_s32_lmode <= 0 ||
                    yfms->xpl.asrt.baro.id_s32_rmode <= 0 ||
                    yfms->xpl.asrt.baro.id_u32_lunit <= 0 ||
                    yfms->xpl.asrt.baro.id_u32_runit <= 0 ||
                    yfms->xpl.asrt.baro.id_f32_lunip <= 0 ||
                    yfms->xpl.asrt.baro.id_f32_runip <= 0 ||
                    yfms->xpl.asrt.xpdr.id_u32_code  <= 0 ||
                    yfms->xpl.asrt.xpdr.id_u32_altr  <= 0 ||
                    yfms->xpl.asrt.xpdr.id_u32_mode  <= 0 ||
                    yfms->xpl.asrt.xpdr.id_u32_tmod  <= 0 ||
                    yfms->xpl.asrt.xpdr.id_u32_tcas  <= 0)
                {
                    ndt_log("YFMS [warning]: couldn't initialize \"%s\" data\n", XPLM_FF_SIGNATURE);
                    yfms->xpl.atyp = YFS_ATYP_XPLN; break;
                }
                yfms->xpl.has_custom_nav_radios = 1;
                yfms->xpl.has_custom_navigation = 1;
                yfms->xpl.atyp = YFS_ATYP_ASRT; break;
            }
            if (yfms->xpl.ixeg.xpdr_mode_act       && yfms->xpl.ixeg.xpdr_stby_act       &&
                yfms->xpl.ixeg.radios_adf1_100_act && yfms->xpl.ixeg.radios_adf2_100_act &&
                yfms->xpl.ixeg.radios_adf1_010_act && yfms->xpl.ixeg.radios_adf2_010_act &&
                yfms->xpl.ixeg.radios_adf1_001_act && yfms->xpl.ixeg.radios_adf2_001_act &&
                yfms->xpl.ixeg.baro_inhg_sby_0001_ind)
            {
                yfms->xpl.has_custom_navigation = 1;
                yfms->xpl.atyp = YFS_ATYP_IXEG; break;
            }
            if (yfms->xpl.qpac.XPDR[0]         && yfms->xpl.qpac.XPDR[1]           &&
                yfms->xpl.qpac.XPDR[2]         && yfms->xpl.qpac.XPDR[3]           &&
                yfms->xpl.qpac.XPDRPower       && yfms->xpl.qpac.XPDRAltitude      &&
                yfms->xpl.qpac.XPDRTCASMode    && yfms->xpl.qpac.XPDRTCASAltSelect &&
                yfms->xpl.qpac.BaroUnitCapt    && yfms->xpl.qpac.BaroStdCapt       &&
                yfms->xpl.qpac.BaroUnitFO      && yfms->xpl.qpac.BaroStdFO         &&
                yfms->xpl.qpac.RMPSwapCapt     && yfms->xpl.qpac.RMPSwapCo         &&
                yfms->xpl.qpac.VHF1Capt        && yfms->xpl.qpac.VHF2Co            &&
                yfms->xpl.qpac.RMP1FreqUpLrg   && yfms->xpl.qpac.RMP1FreqUpSml     &&
                yfms->xpl.qpac.RMP1FreqDownLrg && yfms->xpl.qpac.RMP1FreqDownSml   &&
                yfms->xpl.qpac.RMP2FreqUpLrg   && yfms->xpl.qpac.RMP2FreqUpSml     &&
                yfms->xpl.qpac.RMP2FreqDownLrg && yfms->xpl.qpac.RMP2FreqDownSml)
            {
                yfms->xpl.has_custom_nav_radios = 1;
                yfms->xpl.has_custom_navigation = 1;
                yfms->xpl.atyp = YFS_ATYP_TOLI; break;
            }
            if (yfms->xpl.qpac.XPDR[0]         && yfms->xpl.qpac.XPDR[1]         &&
                yfms->xpl.qpac.XPDR[2]         && yfms->xpl.qpac.XPDR[3]         &&
                yfms->xpl.qpac.XPDRPower       && yfms->xpl.qpac.XPDRAltitude    &&
                yfms->xpl.qpac.BaroUnitCapt    && yfms->xpl.qpac.BaroStdCapt     &&
                yfms->xpl.qpac.BaroUnitFO      && yfms->xpl.qpac.BaroStdFO       &&
                yfms->xpl.qpac.RMPSwapCapt     && yfms->xpl.qpac.RMPSwapCo       &&
                yfms->xpl.qpac.VHF1Capt        && yfms->xpl.qpac.VHF2Co          &&
                yfms->xpl.qpac.RMP1FreqUpLrg   && yfms->xpl.qpac.RMP1FreqUpSml   &&
                yfms->xpl.qpac.RMP1FreqDownLrg && yfms->xpl.qpac.RMP1FreqDownSml &&
                yfms->xpl.qpac.RMP2FreqUpLrg   && yfms->xpl.qpac.RMP2FreqUpSml   &&
                yfms->xpl.qpac.RMP2FreqDownLrg && yfms->xpl.qpac.RMP2FreqDownSml)
            {
                yfms->xpl.has_custom_nav_radios = 1;
                yfms->xpl.has_custom_navigation = 1;
                yfms->xpl.atyp = YFS_ATYP_QPAC; break;
            }
            if (yfms->xpl.q350.pressModeLeft    && yfms->xpl.q350.pressModeRight  &&
                yfms->xpl.q350.pressLeftButton  && yfms->xpl.q350.pressLeftRotary &&
                yfms->xpl.q350.pressRightButton && yfms->xpl.q350.pressRightRotary)
            {
                yfms->xpl.has_custom_nav_radios = 1;
                yfms->xpl.has_custom_navigation = 1;
                yfms->xpl.atyp = YFS_ATYP_Q350; break;
            }
            if (yfms->xpl.q380.PeterV1      && yfms->xpl.q380.PeterV2   &&
                yfms->xpl.q380.PeterVR      && yfms->xpl.q380.PeterCI   &&
                yfms->xpl.q380.PeterCRZ     && yfms->xpl.q380.PeterFLX  &&
                yfms->xpl.q380.BaroUnitFO   && yfms->xpl.q380.BaroStdFO &&
                yfms->xpl.q380.BaroUnitCapt && yfms->xpl.q380.BaroStdCapt)
            {
                yfms->xpl.atyp = YFS_ATYP_Q380; break;
            }
            if (yfms->xpl.fb76.systemMode      && yfms->xpl.fb76.baroRotary_stby   &&
                yfms->xpl.fb76.baroRotary_left && yfms->xpl.fb76.baroRotary_right  &&
                yfms->xpl.fb76.leftBigRotary   && yfms->xpl.fb76.leftMidRotary     && yfms->xpl.fb76.leftSmallRotary  &&
                yfms->xpl.fb76.rightBigRotary  && yfms->xpl.fb76.rightMidRotary    && yfms->xpl.fb76.rightSmallRotary &&
                yfms->xpl.fb76.nav2_nav_id     && yfms->xpl.fb76.nav2_frequency_hz && yfms->xpl.fb76.nav2_obs_deg_mag_pilot)
            {
                yfms->xpl.has_custom_navigation = 1;
                yfms->xpl.atyp = YFS_ATYP_FB76; break;
            }
            if (yfms->xpl.fb77.anim_25_rotery && yfms->xpl.fb77.anim_67_switch &&
                yfms->xpl.fb77.anim_85_switch && yfms->xpl.fb77.anim_175_button)
            {
                yfms->xpl.has_custom_nav_radios = 1;
                yfms->xpl.has_custom_navigation = 1;
                yfms->xpl.atyp = YFS_ATYP_FB77; break;
            }
            if (XPLMGetDatai(yfms->xpl.override_fms))
            {
                ndt_log("YFMS [info]: override_fms_advance set, assuming custom navigation\n");
                yfms->xpl.has_custom_navigation = 1;
            }
            /*
             * http://www.hochwarth.com/misc/AviationCalculator.html#CrossoverAltitude
             *
             * The Crossosver Altitude is the altitude at which a specified
             * CAS and Mach value represent the same TAS value. The curves
             * for constant CAS and constant Mach intersect at this point.
             * Above this altitude, Mach number used to reference speeds.
             *
             * Note: take good margin vis-à-vis Vmo speed {~25) due to X-Plane's
             *       default autopilot being very unstable near said speed limit.
             */
            if (yfms->xpl.tekt.cl3_fms_selector)
            {
//              yfms->xpl.otto.vmax_auto = 1;
//              yfms->xpl.otto.vmax_kias = 295; // Vmo: 320 KIAS
//              yfms->xpl.otto.vmax_mach = 820; // Mmo: .83 MACH
//              yfms->xpl.otto.flt_vmo = 29482.35074f; // FL/Vmo
//              ndt_log("YFMS [info]: vmax_auto enabled (0.%03d, %d)\n",
//                      yfms->xpl.otto.vmax_mach, yfms->xpl.otto.vmax_kias);
                yfms->xpl.otto.vclb_vdes = 1;
                yfms->xpl.otto.vclb_mach = 750; // CLB/250/0.75@35,562feet (MSL)
                yfms->xpl.otto.vdes_kias = 300; // DES/0.80/300@30,595feet (MSL)
                ndt_log("YFMS [info]: vclb_vdes enabled (0.%03d, %d)\n",
                        yfms->xpl.otto.vclb_mach, yfms->xpl.otto.vdes_kias);
                yfms->xpl.atyp = YFS_ATYP_XPLN; break;
            }
            if (yfms->xpl.tekt.CruiseSpeed_Mach &&
                yfms->xpl.tekt.CruiseSpeed_KIAS)
            {
                if (yfms->xpl.tekt.HA4T_shared_KIAS)
                {
//                  yfms->xpl.otto.vmax_auto = 1;
//                  yfms->xpl.otto.vmax_kias = 325; // Vmo: 350 KIAS
//                  yfms->xpl.otto.vmax_mach = 830; // Mmo: .84 MACH
//                  yfms->xpl.otto.flt_vmo = 25839.22806f; // FL/Vmo
//                  ndt_log("YFMS [info]: vmax_auto enabled (%d, .%03d)\n",
//                          yfms->xpl.otto.vmax_kias, yfms->xpl.otto.vmax_mach);
                    yfms->xpl.otto.vclb_vdes = 1;
                    yfms->xpl.otto.vclb_mach = 700;
                    yfms->xpl.otto.vdes_kias = 300;
                    ndt_log("YFMS [info]: vclb_vdes enabled (0.%03d, %d)\n",
                            yfms->xpl.otto.vclb_mach, yfms->xpl.otto.vdes_kias);
                    yfms->xpl.atyp = YFS_ATYP_XPLN; break;
                }
                if (yfms->xpl.tekt.E175_mouse_x_pos ||
                    yfms->xpl.tekt.E195_mouse_x_pos)
                {
//                  yfms->xpl.otto.vmax_auto = 1;
//                  yfms->xpl.otto.vmax_kias = 295; // Vmo: 320 KIAS
//                  yfms->xpl.otto.vmax_mach = 810; // Mmo: .82 MACH
//                  yfms->xpl.otto.flt_vmo = 28858.06702f; // FL/Vmo
//                  ndt_log("YFMS [info]: vmax_auto enabled (0.%03d, %d)\n",
//                          yfms->xpl.otto.vmax_mach, yfms->xpl.otto.vmax_kias);
                    yfms->xpl.otto.vclb_vdes = 1;
                    yfms->xpl.otto.vclb_mach = 720;
                    yfms->xpl.otto.vdes_kias = 280;
                    ndt_log("YFMS [info]: vclb_vdes enabled (0.%03d, %d)\n",
                            yfms->xpl.otto.vclb_mach, yfms->xpl.otto.vdes_kias);
                    yfms->xpl.atyp = YFS_ATYP_XPLN; break;
                }
            }

            /* no custom type found, all default X-Plane systems used */
            yfms->xpl.atyp = YFS_ATYP_XPLN; break;
        }
        while (0);
        ndt_log("YFMS [info]: first key press, determined type %d\n", yfms->xpl.atyp);

        /*
         * Sync IXEG ADF selectors with X-Plane datarefs (they aren't at start).
         */
        if (yfms->xpl.atyp == YFS_ATYP_IXEG)
        {
            int adf1_htz = XPLMGetDatai(yfms->xpl.adf1_frequency_hz);
            int adf1_100 = (adf1_htz) / 100;
            int adf1_010 = (adf1_htz - adf1_100 * 100) / 10;
            int adf1_001 = (adf1_htz - adf1_100 * 100 - adf1_010 * 10);
            XPLMSetDataf(yfms->xpl.ixeg.radios_adf1_100_act, (float)adf1_100);
            XPLMSetDataf(yfms->xpl.ixeg.radios_adf1_010_act, (float)adf1_010);
            XPLMSetDataf(yfms->xpl.ixeg.radios_adf1_001_act, (float)adf1_001);
            int adf2_htz = XPLMGetDatai(yfms->xpl.adf2_frequency_hz);
            int adf2_100 = (adf2_htz) / 100;
            int adf2_010 = (adf2_htz - adf2_100 * 100) / 10;
            int adf2_001 = (adf2_htz - adf2_100 * 100 - adf2_010 * 10);
            XPLMSetDataf(yfms->xpl.ixeg.radios_adf2_100_act, (float)adf2_100);
            XPLMSetDataf(yfms->xpl.ixeg.radios_adf2_010_act, (float)adf2_010);
            XPLMSetDataf(yfms->xpl.ixeg.radios_adf2_001_act, (float)adf2_001);
        }

        /*
         * Sync VOR/LOC courses between pilot & copilot;
         * pilot crs1 master, copilot crs2 master (ideal for IXEG/B733).
         */
        XPLMSetDataf(yfms->xpl.nav1_obs_deg_mag_copilot, XPLMGetDataf(yfms->xpl.nav1_obs_deg_mag_pilot));
        XPLMSetDataf(yfms->xpl.nav2_obs_deg_mag_pilot, XPLMGetDataf(yfms->xpl.nav2_obs_deg_mag_copilot));
    }
    if (yfms->mwindow.current_page == PAGE_FPLN)
    {
        yfms->data.fpln.ln_off = 0; // reset page's line offset before updating
        yfs_fpln_pageupdt(yfms); // update the page while window's still hidden
    }
    XPShowWidget            (yfms->mwindow.id);
    XPSetKeyboardFocus      (yfms->mwindow.id);
    XPBringRootWidgetToFront(yfms->mwindow.id);
    yfs_curr_pageupdt       (yfms            ); // call last as it does nothing when hidden
    return;
}

/*
 * Note: yfs_main_init/close() are called from XPluginEnable/Disable(), so we
 *       can actually reload navigation data by disabling and re-enabling the
 *       plugin, without having to relaunch X-Plane itself :D
 */
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

    /* the navigation database remains valid for the whole YFMS session */
    XPLMGetSystemPath(yfms->ndt.xsystem_pth);
    {
        struct stat stats;
        int    pathlen, ret = 0;
        char  *path         = NULL;
        do
        {
            // X-Plane 10's WMM coefficients are a bit outdated (2019/12/31?)
            // our data starts 2020/01/01, any date in 2019 should match XP10
            ndt_date xpdate = ndt_date_now(); xpdate.year = 2019;
            if (0 == (ret = ndt_file_getpath(yfms->ndt.xsystem_pth, "Custom Data/GNS430/navdata/ATS.txt", &path, &pathlen)) &&
                0 == (stat(path, &stats)) && S_ISREG(stats.st_mode)                                                         &&
                0 == (ret = ndt_file_getpath(yfms->ndt.xsystem_pth, "Custom Data/GNS430/navdata",         &path, &pathlen)) &&
                0 == (stat(path, &stats)) && S_ISDIR(stats.st_mode))
            {
                if ((yfms->ndt.ndb = ndt_navdatabase_init(path, NDT_NAVDFMT_XPGNS, xpdate)) == NULL)
                {
                    ndt_log("YFMS [error]: could not load navigation database at \"%s\"\n", path);
                    free(path); goto fail;
                }
                ndt_log("YFMS [info]: loaded navigation database from \"%s\"\n", path);
                free(path); break;
            }
            if (0 == (ret = ndt_file_getpath(yfms->ndt.xsystem_pth, "Resources/GNS430/navdata/ATS.txt", &path, &pathlen)) &&
                0 == (stat(path, &stats)) && S_ISREG(stats.st_mode)                                                       &&
                0 == (ret = ndt_file_getpath(yfms->ndt.xsystem_pth, "Resources/GNS430/navdata",         &path, &pathlen)) &&
                0 == (stat(path, &stats)) && S_ISDIR(stats.st_mode))
            {
                if ((yfms->ndt.ndb = ndt_navdatabase_init(path, NDT_NAVDFMT_XPGNS, xpdate)) == NULL)
                {
                    ndt_log("YFMS [error]: could not load navigation database at \"%s\"\n", path);
                    free(path); goto fail;
                }
                ndt_log("YFMS [info]: loaded navigation database from \"%s\"\n", path);
                free(path); break;
            }
            ndt_log("YFMS [error]: could not load navigation database for \"%s\"\n", yfms->ndt.xsystem_pth);
            free(path); goto fail;
        }
        while (0);
    }
    ndt_log("YFMS [info]: %s\n", yfms->ndt.ndb->info.desc);

    /* aicraft specific X-Plane data */
    if ((yfms->xpl.acf_ICAO                        = XPLMFindDataRef("sim/aircraft/view/acf_ICAO"                                   )) == NULL || // PAGE_IDNT
        (yfms->xpl.acf_en_type                     = XPLMFindDataRef("sim/aircraft/prop/acf_en_type"                                )) == NULL ||
        (yfms->xpl.acf_num_engines                 = XPLMFindDataRef("sim/aircraft/engine/acf_num_engines"                          )) == NULL ||
        (yfms->xpl.com1_standy_flip                = XPLMFindCommand("sim/radios/com1_standy_flip"                                  )) == NULL || // PAGE_RAD1
        (yfms->xpl.com2_standy_flip                = XPLMFindCommand("sim/radios/com2_standy_flip"                                  )) == NULL ||
        (yfms->xpl.transponder_ident               = XPLMFindCommand("sim/transponder/transponder_ident"                            )) == NULL ||
        (yfms->xpl.transponder_id                  = XPLMFindDataRef("sim/cockpit2/radios/indicators/transponder_id"                )) == NULL ||
        (yfms->xpl.transponder_mode                = XPLMFindDataRef("sim/cockpit2/radios/actuators/transponder_mode"               )) == NULL ||
        (yfms->xpl.transponder_code                = XPLMFindDataRef("sim/cockpit2/radios/actuators/transponder_code"               )) == NULL ||
        (yfms->xpl.com1_frequency_Mhz              = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_frequency_Mhz"             )) == NULL ||
        (yfms->xpl.com1_standby_frequency_Mhz      = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_standby_frequency_Mhz"     )) == NULL ||
        (yfms->xpl.com1_frequency_khz              = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_frequency_khz"             )) == NULL ||
        (yfms->xpl.com1_standby_frequency_khz      = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_standby_frequency_khz"     )) == NULL ||
        (yfms->xpl.com2_frequency_Mhz              = XPLMFindDataRef("sim/cockpit2/radios/actuators/com2_frequency_Mhz"             )) == NULL ||
        (yfms->xpl.com2_standby_frequency_Mhz      = XPLMFindDataRef("sim/cockpit2/radios/actuators/com2_standby_frequency_Mhz"     )) == NULL ||
        (yfms->xpl.com2_frequency_khz              = XPLMFindDataRef("sim/cockpit2/radios/actuators/com2_frequency_khz"             )) == NULL ||
        (yfms->xpl.com2_standby_frequency_khz      = XPLMFindDataRef("sim/cockpit2/radios/actuators/com2_standby_frequency_khz"     )) == NULL ||
        (yfms->xpl.com1_left_frequency_hz_833      = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_left_frequency_hz_833"     )) == NULL ||
        (yfms->xpl.com2_left_frequency_hz_833      = XPLMFindDataRef("sim/cockpit2/radios/actuators/com2_left_frequency_hz_833"     )) == NULL ||
        (yfms->xpl.barometer_setting_in_hg_pilot   = XPLMFindDataRef("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot"  )) == NULL ||
        (yfms->xpl.barometer_setting_in_hg_copilot = XPLMFindDataRef("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_copilot")) == NULL ||
        (yfms->xpl.nav1_type                       = XPLMFindDataRef("sim/cockpit2/radios/indicators/nav1_type"                     )) == NULL || // PAGE_RAD2
        (yfms->xpl.nav2_type                       = XPLMFindDataRef("sim/cockpit2/radios/indicators/nav2_type"                     )) == NULL ||
        (yfms->xpl.adf1_nav_id                     = XPLMFindDataRef("sim/cockpit2/radios/indicators/adf1_nav_id"                   )) == NULL ||
        (yfms->xpl.adf2_nav_id                     = XPLMFindDataRef("sim/cockpit2/radios/indicators/adf2_nav_id"                   )) == NULL ||
        (yfms->xpl.nav1_nav_id                     = XPLMFindDataRef("sim/cockpit2/radios/indicators/nav1_nav_id"                   )) == NULL ||
        (yfms->xpl.nav2_nav_id                     = XPLMFindDataRef("sim/cockpit2/radios/indicators/nav2_nav_id"                   )) == NULL ||
        (yfms->xpl.autopilot_source                = XPLMFindDataRef("sim/cockpit2/autopilot/autopilot_source"                      )) == NULL ||
        (yfms->xpl.adf1_frequency_hz               = XPLMFindDataRef("sim/cockpit2/radios/actuators/adf1_frequency_hz"              )) == NULL ||
        (yfms->xpl.adf2_frequency_hz               = XPLMFindDataRef("sim/cockpit2/radios/actuators/adf2_frequency_hz"              )) == NULL ||
        (yfms->xpl.nav1_frequency_hz               = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_frequency_hz"              )) == NULL ||
        (yfms->xpl.nav2_frequency_hz               = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav2_frequency_hz"              )) == NULL ||
        (yfms->xpl.nav1_obs_deg_mag_pilot          = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot"         )) == NULL ||
        (yfms->xpl.nav2_obs_deg_mag_pilot          = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav2_obs_deg_mag_pilot"         )) == NULL ||
        (yfms->xpl.nav1_obs_deg_mag_copilot        = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_obs_deg_mag_copilot"       )) == NULL ||
        (yfms->xpl.nav2_obs_deg_mag_copilot        = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav2_obs_deg_mag_copilot"       )) == NULL ||
        (yfms->xpl.hsi_obs_deg_mag_pilot           = XPLMFindDataRef("sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot"          )) == NULL ||
        (yfms->xpl.hsi_obs_deg_mag_copilot         = XPLMFindDataRef("sim/cockpit2/radios/actuators/hsi_obs_deg_mag_copilot"        )) == NULL ||
        (yfms->xpl.HSI_source_select_pilot         = XPLMFindDataRef("sim/cockpit2/radios/actuators/HSI_source_select_pilot"        )) == NULL ||
        (yfms->xpl.HSI_source_select_copilot       = XPLMFindDataRef("sim/cockpit2/radios/actuators/HSI_source_select_copilot"      )) == NULL ||
        (yfms->xpl.override_fms                    = XPLMFindDataRef("sim/operation/override/override_fms_advance"                  )) == NULL || // PAGE_FPLN
        (yfms->xpl.acf_vs0                         = XPLMFindDataRef("sim/aircraft/view/acf_Vso"                                    )) == NULL || // miscellaneous
        (yfms->xpl.mag_trk                         = XPLMFindDataRef("sim/cockpit2/gauges/indicators/ground_track_mag_pilot"        )) == NULL ||
        (yfms->xpl.mag_var                         = XPLMFindDataRef("sim/flightmodel/position/magnetic_variation"                  )) == NULL ||
        (yfms->xpl.latitude                        = XPLMFindDataRef("sim/flightmodel/position/latitude"                            )) == NULL ||
        (yfms->xpl.longitude                       = XPLMFindDataRef("sim/flightmodel/position/longitude"                           )) == NULL ||
        (yfms->xpl.groundspeed                     = XPLMFindDataRef("sim/flightmodel/position/groundspeed"                         )) == NULL ||
        (yfms->xpl.elevation_agl                   = XPLMFindDataRef("sim/flightmodel/position/y_agl"                               )) == NULL ||
        (yfms->xpl.elevation_msl                   = XPLMFindDataRef("sim/flightmodel/position/elevation"                           )) == NULL ||
        (yfms->xpl.tropopause                      = XPLMFindDataRef("sim/weather/tropo_alt_mtr"                                    )) == NULL ||
        (yfms->xpl.gps_select_copilot              = XPLMFindCommand("sim/autopilot/hsi_select_copilot_gps"                         )) == NULL || // autopilot
        (yfms->xpl.gps_select_captain              = XPLMFindCommand("sim/autopilot/hsi_select_gps"                                 )) == NULL ||
        (yfms->xpl.knots_mach_toggle               = XPLMFindCommand("sim/autopilot/knots_mach_toggle"                              )) == NULL ||
        (yfms->xpl.autopilot_nav                   = XPLMFindCommand("sim/autopilot/NAV"                                            )) == NULL ||
        (yfms->xpl.heading_sync                    = XPLMFindCommand("sim/autopilot/heading_sync"                                   )) == NULL ||
        (yfms->xpl.machno                          = XPLMFindDataRef("sim/flightmodel/misc/machno"                                  )) == NULL ||
        (yfms->xpl.fdir_mode                       = XPLMFindDataRef("sim/cockpit2/autopilot/flight_director_mode"                  )) == NULL ||
        (yfms->xpl.vvi_fpm_pilot                   = XPLMFindDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot"                 )) == NULL ||
        (yfms->xpl.nav_mode_status                 = XPLMFindDataRef("sim/cockpit2/autopilot/nav_status"                            )) == NULL ||
        (yfms->xpl.airspeed_is_mach                = XPLMFindDataRef("sim/cockpit2/autopilot/airspeed_is_mach"                      )) == NULL ||
        (yfms->xpl.altitude_ft_pilot               = XPLMFindDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot"             )) == NULL ||
        (yfms->xpl.airspeed_kts_pilot              = XPLMFindDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot"            )) == NULL ||
        (yfms->xpl.heading_dial_deg_mag_copilot    = XPLMFindDataRef("sim/cockpit2/autopilot/heading_dial_deg_mag_copilot"          )) == NULL ||
        (yfms->xpl.heading_dial_deg_mag_pilot      = XPLMFindDataRef("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot"            )) == NULL ||
        (yfms->xpl.airspeed_dial_kts_mach          = XPLMFindDataRef("sim/cockpit2/autopilot/airspeed_dial_kts_mach"                )) == NULL ||
        (yfms->xpl.altitude_dial_mcp_feet          = XPLMFindDataRef("sim/cockpit2/autopilot/altitude_dial_ft"                      )) == NULL ||
        (yfms->xpl.autothrottle_enabled            = XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_enabled"                  )) == NULL ||
        (yfms->xpl.vvi_status                      = XPLMFindDataRef("sim/cockpit2/autopilot/vvi_status"                            )) == NULL ||
        (yfms->xpl.pitch_status                    = XPLMFindDataRef("sim/cockpit2/autopilot/pitch_status"                          )) == NULL ||
        (yfms->xpl.contact_atc                     = XPLMFindCommand("sim/operation/contact_atc"                                    )) == NULL)
    {
        ndt_log("YFMS [error]: could not load aircraft-related datarefs and commands\n");
        goto fail;
    }

    /* we have a couple very useful key sniffers */
    if ((yfms->mwindow.ks_rgstrd = XPLMRegisterKeySniffer(&yfs_keysniffer, 1, yfms)) == 0)
    {
        ndt_log("YFMS [warning]: failed to register key sniffer #1\n");
    }
    if ((yfms->mwindow.aw_rgstrd = XPLMRegisterKeySniffer(&yfs_afterwindw, 0, yfms)) == 0)
    {
        ndt_log("YFMS [warning]: failed to register key sniffer #2\n");
    }
    yfms->mwindow.ks_d_mode = YFS_KSM_WIN; // default: mouse within  main window

    /* all good */
    yfs_menu_resetall(yfms); return yfms;

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
            yfms->mwindow.screen.colr[idx][i] = col >= 0 ? col : COLR_IDX_BLUE;
        }
        memset(yfms->mwindow.screen.text[idx], 0, YFS_ROW_BUF_SIZE);
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

    /* navigation */
    for (int i = 0; i < 20; i++)
    {
        if (yfms->data.fpln.usrwpts[i].wpt)
        {
            ndt_navdata_rem_waypoint(yfms->ndt.ndb, yfms->data.fpln.usrwpts[i].wpt);
            ndt_waypoint_close(&yfms->data.fpln.usrwpts[i].wpt);
        }
        yfms->data.fpln.usrwpts[i].wpt = NULL;
        yfms->data.fpln.usrwpts[i].ref = 0;
    }
    if (yfms->data.fpln.w_tp)
    {
        ndt_waypoint_close(&yfms->data.fpln.w_tp);
    }
    if (yfms->data.prog.usrwpt)
    {
        ndt_waypoint_close(&yfms->data.prog.usrwpt);
    }
    if (yfms->ndt.ndb)
    {
        ndt_navdatabase_close(&yfms->ndt.ndb);
    }
    if (yfms->ndt.flp.arr)
    {
        ndt_flightplan_close(&yfms->ndt.flp.arr);
    }
    if (yfms->ndt.flp.dep)
    {
        ndt_flightplan_close(&yfms->ndt.flp.dep);
    }
    if (yfms->ndt.flp.iac)
    {
        ndt_flightplan_close(&yfms->ndt.flp.iac);
    }
    if (yfms->ndt.flp.rte)
    {
        ndt_flightplan_close(&yfms->ndt.flp.rte);
    }

    /* key sniffer */
    if (yfms->mwindow.ks_rgstrd)
    {
        yfms->mwindow.ks_rgstrd = XPLMUnregisterKeySniffer(&yfs_keysniffer, 1, yfms);
    }
    if (yfms->mwindow.aw_rgstrd)
    {
        yfms->mwindow.aw_rgstrd = XPLMUnregisterKeySniffer(&yfs_afterwindw, 0, yfms);
    }

    /* flight loop callback */
    if (yfms->xpl.fl_callback)
    {
        XPLMUnregisterFlightLoopCallback(yfms->xpl.fl_callback, yfms);
    }

    /* cairo */
    if (CAIRO4SCRN(font))     cairo_font_face_destroy(CAIRO4SCRN(font));
    if (CAIRO4SCRN(surf))     cairo_surface_destroy(CAIRO4SCRN(surf));
    if (CAIRO4SCRN(cr))       cairo_destroy(CAIRO4SCRN(cr));
    if (CAIRO4SCRN(data.buf)) free(CAIRO4SCRN(data.buf));

#ifndef USE_DEFAULT_FONT
#if CAIRO_HAS_FT_FONT
    /* freetype */
    if (CAIRO4SCRN(ft.face)) FT_Done_Face(CAIRO4SCRN(ft.face));
    FT_Done_FreeType(CAIRO4SCRN(ft.flib));
#endif
#endif

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

int yfs_main_newpg(yfms_context *yfms, int new_page)
{
    if (yfms == NULL)
    {
        return ENOMEM;
    }
    if (yfms->mwindow.current_page == PAGE_MENU)
    {
        switch (new_page)
        {
            case PAGE_MENU:
                break;      // re-opening itself must never fail
            case PAGE_IDNT:
                break;      // only available from MENU page itself
            default:
                return -1;  // MENU is special, cannot open FMGC pages from it
        }
    }
    // some keys' functions are always page-specific, so their callbacks
    // need to be (unconditionally) reset on any and every page change
    yfms->lsks[0][0].cback = yfms->lsks[1][0].cback =
    yfms->lsks[0][1].cback = yfms->lsks[1][1].cback =
    yfms->lsks[0][3].cback = yfms->lsks[1][3].cback =
    yfms->lsks[0][4].cback = yfms->lsks[1][4].cback =
    yfms->lsks[0][5].cback = yfms->lsks[1][5].cback = (YFS_LSK_f)NULL;
    yfms->spcs.cback_left  = yfms->spcs.cback_rigt  =
    yfms->spcs.cback_lnup  = yfms->spcs.cback_lndn  = (YFS_SPC_f)NULL;
    yfms->mousew_callback                           = (YFS_MSW_f)NULL;
    yfms->mousec_callback                           = (YFS_MSC_f)NULL;
    // YFMS should always know which page is being displayed
    yfms->mwindow.current_page = new_page; return 0;
}

ndt_waypoint* yfs_main_getwp(yfms_context *yfms, const char *name, ndt_waypoint *from)
{
    if (yfms && name && *name)
    {
        ndt_waypoint *wpt = ndt_navdata_get_wptnear2(yfms->ndt.ndb, name, NULL, from ? from->position : yfms->data.aircraft_pos);
        if (wpt)
        {
            if (yfs_main_is_usrwpt(yfms, wpt))
            {
                yfs_main_usrwp_ref(yfms, wpt);
            }
            return wpt;
        }
        if (strnlen(name, 4) == 3) // could also be the FAA location identifier
        {
            // use the 4-letter ICAO equivalent instead, if present in database
            char icao[5]; ndt_airport *apt;
            {
                snprintf(icao, sizeof(icao), "K%s", name);
            }
            if ((apt = ndt_navdata_get_airport(yfms->ndt.ndb, icao)))
            {
                return apt->waypoint;
            }
        }
        XPLMNavType nav_aid_types = xplm_Nav_Airport|xplm_Nav_NDB|xplm_Nav_VOR|xplm_Nav_ILS|xplm_Nav_Localizer|xplm_Nav_Fix|xplm_Nav_DME;
        float inLatitude  = (float)ndt_position_getlatitude (yfms->data.aircraft_pos, NDT_ANGUNIT_DEG);
        float inLongitude = (float)ndt_position_getlongitude(yfms->data.aircraft_pos, NDT_ANGUNIT_DEG);
        XPLMNavRef navRef = XPLMFindNavAid(NULL, name, &inLatitude, &inLongitude, NULL, nav_aid_types);
        if (XPLM_NAV_NOT_FOUND != navRef)
        {
            XPLMNavType outTypes; size_t sname = strlen(name); char outID[33]; float outLat, outLon;
            XPLMGetNavAidInfo(navRef, &outTypes, &outLat, &outLon, NULL, NULL, NULL, outID, NULL, NULL);
            /*
             * Example (Navigraph data, AIRAC 1707):
             * > $ grep TRS Waypoints.txt Navaids.txt | grep \,ES
             * > Waypoints.txt:TRS29,59.286033,18.150856,ES
             * > Navaids.txt:TRS,TROSA,114.300,1,1,195,58.937917,17.502222,213,ES,0
             *
             * Even worse (Aerosoft data, AIRAC 1707):
             * > $ grep TRS Waypoints.txt Navaids.txt | grep \,ES
             * > Waypoints.txt:27TRS,59.20740,18.17919,ES
             * > Waypoints.txt:65TRS,59.00434,17.66729,ES
             * > Waypoints.txt:TRS10,58.82436,17.26767,ES
             * > Waypoints.txt:TRS22,59.20709,17.98320,ES
             * > Waypoints.txt:TRS27,59.38652,17.47173,ES
             * > Waypoints.txt:TRS65,59.04594,17.49621,ES
             * > Navaids.txt:TRS,TROSA,114.300,1,1,195,58.93792,17.50222,213,ES,0
             *
             * Because of how XPLMFindNavAid works (by design), it could
             * e.g. find and return any of the above when inIDFrag is TRS.
             */
            if (strnlen(outID, 1 + sname) != sname)
            {
                return NULL;
            }
            char fmt[23]; ndt_waypoint *wpt;
            {
                snprintf(fmt, sizeof(fmt), "%+.6lf/%+.6lf", outLat, outLon);
            }
            if ((wpt = ndt_waypoint_llc(fmt)) == NULL)
            {
                ndt_log("YFMS [debug]: ndt_waypoint_llc(\"%s\" %04d %+9.6f/%+10.6f) failed\n", outID, outTypes, outLat, outLon);
                return NULL;
            }
            switch (outTypes)
            {
                default: // we don't need cases, but we have to be able to break
                    if ((outTypes & xplm_Nav_Airport))
                    {
                        wpt->type = NDT_WPTYPE_XPA;
                        break;
                    }
                    if ((outTypes & xplm_Nav_NDB))
                    {
                        wpt->type = NDT_WPTYPE_NDB;
                        break;
                    }
                    if ((outTypes & xplm_Nav_VOR))
                    {
                        wpt->type = NDT_WPTYPE_VOR;
                        break;
                    }
                    if ((outTypes & xplm_Nav_ILS) ||
                        (outTypes & xplm_Nav_Localizer))
                    {
                        wpt->type = NDT_WPTYPE_LOC;
                        break;
                    }
                    if ((outTypes & xplm_Nav_Fix))
                    {
                        wpt->type = NDT_WPTYPE_FIX;
                        break;
                    }
                    if ((outTypes & xplm_Nav_DME))
                    {
                        wpt->type = NDT_WPTYPE_DME;
                        break;
                    }
                    ndt_waypoint_close(&wpt); return NULL;
            }
            if (wpt->type != NDT_WPTYPE_XPA)
            {
                ndt_log("YFMS [info]: added XPLM-only waypoint: \"%s\", "
                        "of type %04d (internal %2d) at %+9.6f/%+10.6f\n",
                        outID, outTypes, wpt->type, outLat, outLon);
            }
            snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "%s", outID);
            wpt->xplm.navRef = navRef; wpt->xplm.refSet = 1;
            ndt_navdata_add_waypoint(yfms->ndt.ndb, wpt);
            return wpt;
        }
    }
    return NULL;
}

int yfs_main_is_usrwpt(yfms_context *yfms, ndt_waypoint *wpt)
{
    if (yfms && wpt && yfms->data.init.ialized)
    {
        for (int i = 0; i < 20; i++)
        {
            if (yfms->data.fpln.usrwpts[i].ref > 0 &&
                yfms->data.fpln.usrwpts[i].wpt == wpt)
            {
                return 1;
            }
        }
    }
    return 0;
}

void yfs_main_usrwp_ref(yfms_context *yfms, ndt_waypoint *wpt)
{
    if (yfms && wpt && yfms->data.init.ialized)
    {
        for (int i = 0; i < 20; i++)
        {
            if (yfms->data.fpln.usrwpts[i].ref > 0 &&
                yfms->data.fpln.usrwpts[i].wpt == wpt)
            {
#if 0
                ndt_log("314 \"%s\" ++ref: %d -> %d\n",
                        yfms->data.fpln.usrwpts[i].wpt->info.idnt,
                        yfms->data.fpln.usrwpts[i].ref,
                        yfms->data.fpln.usrwpts[i].ref + 1);
#endif
                yfms->data.fpln.usrwpts[i].ref++; return;
            }
        }
    }
}

void yfs_main_usrwp_unr(yfms_context *yfms, ndt_waypoint *wpt)
{
    if (yfms && wpt && yfms->data.init.ialized)
    {
        for (int i = 0; i < 20; i++)
        {
            if (yfms->data.fpln.usrwpts[i].ref > 0 &&
                yfms->data.fpln.usrwpts[i].wpt == wpt)
            {
                if (yfms->data.fpln.usrwpts[i].ref <= 2)
                {
#if 0
                    ndt_log("314 \"%s\" --ref: %d -> %d (free)\n",
                            yfms->data.fpln.usrwpts[i].wpt->info.idnt,
                            yfms->data.fpln.usrwpts[i].ref,
                            yfms->data.fpln.usrwpts[i].ref - 1);
#endif
                    // default value is 1: not referenced, but
                    // will pass testing by yfs_main_is_usrwpt
                    // going from 2 to 1: no longer referenced
                    ndt_navdata_rem_waypoint(yfms->ndt.ndb, wpt);
                    yfms->data.fpln.usrwpts[i].wpt = NULL;
                    yfms->data.fpln.usrwpts[i].ref = 0;
                    ndt_waypoint_close(&wpt); return;
                }
#if 0
                ndt_log("314 \"%s\" --ref: %d -> %d\n",
                        yfms->data.fpln.usrwpts[i].wpt->info.idnt,
                        yfms->data.fpln.usrwpts[i].ref,
                        yfms->data.fpln.usrwpts[i].ref - 1);
#endif
                yfms->data.fpln.usrwpts[i].ref--; return;
            }
        }
    }
}

ndt_waypoint* yfs_main_usrwp(yfms_context *yfms, char *errbuf, char *buffer, ndt_waypoint *from)
{
    if (yfms && errbuf && buffer && *buffer)
    {
        char plce1[8], plce2[8], de1[2], de2[2]; int err;
        double brg1, brg2, dstce, lat, lon, latm, lonm;
        ndt_waypoint *wpt, *place1, *place2;
        ndt_distance distance;
        if (sscanf(buffer, "%7[^/]/%lf/%lf%c", plce1, &brg1, &dstce, plce2) == 3)
        {
            /* PLACE/BRG/DIST (PBD) */
            if (dstce <= 0.5)
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "FORMAT ERROR"); return NULL;
            }
            else
            {
                distance = ndt_distance_init((int64_t)(dstce * 1852.), NDT_ALTUNIT_ME);
            }
            if (brg1 < 0. || brg1 > 360.)
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "FORMAT ERROR"); return NULL;
            }
            if ((place1 = yfs_main_getwp(yfms, plce1, from)) == NULL)
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "NOT IN DATA BASE"); return NULL;
            }
            if ((wpt = ndt_waypoint_pbd(place1, brg1, distance, yfms->ndt.ndb->wmm)))
            {
                snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "%s", "PBD");
                goto wpt_created;
            }
            snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "YFS MAIN USRWP BUG 1"); return NULL;
        }
        if (sscanf(buffer, "%7[^-]-%lf/%7[^-]-%lf%c", plce1, &brg1, plce2, &brg2, plce2) == 4)
        {
            /* PLACE-BRG/PLACE-BRG (PBX) */
            if (brg1 < 0. || brg1 > 360. || brg2 < 0. || brg2 > 360.)
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "FORMAT ERROR"); return NULL;
            }
            if ((place1 = yfs_main_getwp(yfms, plce1,   from)) == NULL ||
                (place2 = yfs_main_getwp(yfms, plce2, place1)) == NULL)
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "NOT IN DATA BASE"); return NULL;
            }
            else
            {
                err = ndt_position_calcpos4pbpb(NULL,
                                                place1->position,
                                                ndt_wmm_getbearing_tru(yfms->ndt.ndb->wmm, brg1, place1->position),
                                                place2->position,
                                                ndt_wmm_getbearing_tru(yfms->ndt.ndb->wmm, brg2, place1->position));
            }
            if (err == EDOM)
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "INFINITE INTERSECTS"); return NULL;
            }
            if (err == ERANGE)
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "NO INTERSECT"); return NULL;
            }
            if ((wpt = ndt_waypoint_pbpb(place1, brg1, place2, brg2, yfms->ndt.ndb->wmm)))
            {
                snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "%s", "PBX");
                goto wpt_created;
            }
            snprintf(errbuf, YFS_ROW_BUF_SIZE, "YFS MAIN USRWP BUG 2 (%d)", err); return NULL;
        }
        if (sscanf(buffer, "%7[^/]/%lf%c", plce1, &dstce, plce2) == 2)
        {
            /* PLACE/DIST (PD), a.k.a. along track distance */
            if (fabs(round(dstce)) < 1.)
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "FORMAT ERROR"); return NULL;
            }
            else
            {
                distance = ndt_distance_init((int64_t)(dstce * 1852.), NDT_ALTUNIT_ME);
            }
            if ((place1 = yfs_main_getwp(yfms, plce1, from)) == NULL)
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "NOT IN DATA BASE"); return NULL;
            }
            /*
             * XXX: special case: from departure runway: use runway heading
             * If implemented: only for user convenience: not a feature IRL
             */
            if (dstce > 0.                           &&
                yfms->ndt.flp.rte->dep.apt           &&
                yfms->ndt.flp.rte->dep.rwy           &&
                yfms->ndt.flp.rte->dep.rwy->waypoint &&
                yfms->ndt.flp.rte->dep.rwy->waypoint == place1)
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "NOT IMPLEMENTED"); return NULL; // TODO: future
            }
            /*
             * XXX: special case: toward arrival runway: use runway heading
             * If implemented: only for user convenience: not a feature IRL
             */
            if (dstce < 0.                           &&
                yfms->ndt.flp.rte->arr.apt           &&
                yfms->ndt.flp.rte->arr.rwy           &&
                yfms->ndt.flp.rte->arr.rwy->waypoint &&
                yfms->ndt.flp.rte->arr.rwy->waypoint == place1)
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "NOT IMPLEMENTED"); return NULL; // TODO: future
            }
            snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "NOT IMPLEMENTED"); return NULL;
        }
#if 0
        if (0)
        {
            /* Abeam reference point (AB) -- only via PAGE_DRTO */
        }
#endif
        char *suffix = buffer, *prefix = strsep(&suffix, "/");
        if   (suffix && strnlen(suffix, 1) && prefix && strnlen(prefix, 1))
        {
            /*
             * LAT/LONG (LL)
             *
             * Latitude:   DDMM.MB or BDDMM.M
             * Longitude: DDDMM.MB or BDDDMM.M
             *
             * 16-character format: N4411.9/W06622.1 (N 44° 11.9' W 066° 22.1')
             * 16-character format: 4411.9N/06622.1W (N 44° 11.9' W 066° 22.1')
             *
             * Note: leading zeroes should be strictly optional.
             * TODO: prepend them (implement the above feature).
             */
            if (sscanf(prefix, "%1[NS]%2lf%4lf", de1, &lat, &latm) == 3 ||
                sscanf(prefix, "%2lf%4lf%1[NS]", &lat, &latm, de1) == 3)
            {
                if (latm > 60. || (lat + latm / 60.) > 90.)
                {
                    snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "FORMAT ERROR"); return NULL;
                }
                else
                {
                    lat = (lat + latm / 60.);
                }
                switch (*de1)
                {
                    case 'N': // north latitude
                        break;
                    case 'S': // south latitude
                        lat = -lat;
                        break;
                    default:  // invalid value
                        snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "FORMAT ERROR"); return NULL;
                }
            }
            else
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "FORMAT ERROR"); return NULL;
            }
            if (sscanf(suffix, "%1[EW]%3lf%4lf", de2, &lon, &lonm) == 3 ||
                sscanf(suffix, "%3lf%4lf%1[EW]", &lon, &lonm, de2) == 3)
            {
                if (lonm > 60. || (lon + lonm / 60.) > 180.)
                {
                    snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "FORMAT ERROR"); return NULL;
                }
                else
                {
                    lon = (lon + lonm / 60.);
                }
                switch (*de2)
                {
                    case 'E': // east longitude
                        break;
                    case 'W': // west longitude
                        lon = -lon;
                        break;
                    default:  // invalid value
                        snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "FORMAT ERROR"); return NULL;
                }
            }
            else
            {
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "FORMAT ERROR"); return NULL;
            }
            char fmt[23];
            {
                snprintf(fmt, sizeof(fmt), "%+.6lf/%+.6lf", lat, lon);
            }
            if ((wpt = ndt_waypoint_llc(fmt)))
            {
                snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "%s", "LL");
                goto wpt_created;
            }
            snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "YFS MAIN USRWP BUG 3"); return NULL;
        }
        goto no_match;

    wpt_created:
        if (wpt)
        {
            if (yfms->data.init.ialized)
            {
                for (int i = 0; i < 20; i++)
                {
                    if (yfms->data.fpln.usrwpts[i].ref <= 0 &&
                        yfms->data.fpln.usrwpts[i].wpt == NULL)
                    {
                        // note: we return NULL, but overwrite the buffer with
                        // our newly-created user waypoint's ID; the next call
                        // to yfs_main_getwp will then return and reference it
                        // don't forget to set ID before adding to DB (sorting)
                        //
                        // default value is 1: not referenced, but
                        // will pass testing by yfs_main_is_usrwpt
                        yfms->data.fpln.usrwpts[i].ref = 1;
                        yfms->data.fpln.usrwpts[i].wpt = wpt;
                        snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "");
                        snprintf(buffer, YFS_ROW_BUF_SIZE, "%s%02d", wpt->info.idnt, i + 1);
                        snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "%s", buffer);
                        ndt_navdata_add_waypoint(yfms->ndt.ndb, wpt); return NULL;
                    }
                }
                snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "MAX 20 USER WPTS"); // TODO: wording???
                ndt_waypoint_close(&wpt); return NULL;
            }
            // no flightplan: can't track, refcount or add it to main database
            snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "%s", "CUST-WP");
            snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", "");  return wpt;
        }
    }

no_match:
    /*
     * signalling of an optional error is done via an empty error string
     * nothing found matching provided buffer's format: make it optional
     * (the caller may decide to look for named waypoints instead, etc.)
     */
    snprintf(errbuf, YFS_ROW_BUF_SIZE, "%s", ""); return NULL;
}

void yfs_printf_lft(void *context, int index, int offset, int color, char *fmt, ...)
{
    int len; char buf[YFS_ROW_BUF_SIZE];
    yfms_context *yfms = context;
    if (!yfms || !fmt)
    {
        return;
    }
    if (index  < 0 || index  >  YFS_DISPLAY_NUMR - 2)
    {
        return; // scratchpad or beyond
    }
    if (offset < 0 || offset >= YFS_DISPLAY_NUMC)
    {
        return; // out of room
    }
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    len = strnlen(buf, YFS_DISPLAY_NUMC - offset);
    for (int i = 0; i < len; i++)
    {
        if (color >= 0)
        {
            yfms->mwindow.screen.colr[index][i + offset] = color;
        }
        yfms->mwindow.screen.text[index][i + offset] = buf[i];
    }
}

void yfs_printf_rgt(void *context, int index, int offset, int color, char *fmt, ...)
{
    int len; char buf[YFS_ROW_BUF_SIZE];
    yfms_context *yfms = context;
    if (!yfms || !fmt)
    {
        return;
    }
    if (index  < 0 || index  >  YFS_DISPLAY_NUMR - 2)
    {
        return; // scratchpad or beyond
    }
    if (offset < 0 || offset >= YFS_DISPLAY_NUMC)
    {
        return; // out of room
    }
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    len = strnlen(buf, YFS_DISPLAY_NUMC - offset);
    for (int i = len - 1,  j = YFS_DISPLAY_NUMC - 1; i >= 0; i--, j--)
    {
        if (color >= 0)
        {
            yfms->mwindow.screen.colr[index][j - offset] = color;
        }
        yfms->mwindow.screen.text[index][j - offset] = buf[i];
    }
}

void yfs_printf_ctr(void *context, int index, int color, char *fmt, ...)
{
    int len; char buf[YFS_ROW_BUF_SIZE];
    if (fmt == NULL)
    {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    len = strnlen(buf, YFS_DISPLAY_NUMC);
    va_end(ap);
    yfs_printf_lft(context, index, (YFS_DISPLAY_NUMC - len) / 2, color, "%s", buf);
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
    if (inMessage == xpMsg_MouseUp   ||
        inMessage == xpMsg_MouseDown ||
        inMessage == xpMsg_MouseDrag ||
        inMessage == xpMsg_MouseWheel)
    {
        if (XPIsWidgetVisible(inWidget))
        {
            yfms_context   *yfms = (yfms_context*)XPGetWidgetProperty(inWidget, xpProperty_Refcon, NULL);
            XPMouseState_t *maus = (XPMouseState_t*)inParam1;
            return   yfs_mouseevent(yfms, maus, inMessage);
            // TODO: handle in YFSkeys.c, IMHO; YFSkeys responsible to check bounds (is required), compute relative_xy, then check
            // for a registered mouse wheel callback and call it: yfms->foo.callback(relative_xy[0], relative_xy[1], axis, delta);
            // we may also want it to filter out any unsupported mouse axes, before handing it off to the pre-registered callback?
            // we most likely want relative_xy relative to the main display's boundaries, rather than be relative to entire window
            // TODO: future: mouse up events can be used for "click to sync" functionality (e.g. otto HDG/ALT/SPD, NAV CRS direct)
            // else: use two LSKs per item (HDG, ALT, VSI etc.); LSK for value: click to sync; LSK for header: click to engage
            //       in that latter case, we could have e.g. the following display area:
            /*
             *     ––––––––––––––––––––––––––     ||     ––––––––––––––––––––––––––     |
             *     |          OTTO          |     ||     |          OTTO          |     |
             *     |                        |     ||     |                        |     |
             * [-] |<HDG                ALT>| [-] || [-] |<NAV                V/S>| [-] |
             *     |                        |     ||     |                        |     |
             * [-] |  cur  HDG    ALT 33000 | [-] || [-] |  360  HDG    ALT 33000 | [-] |
             *     |                        |     ||     |                        |     |
             * [-] |  250  SPD    V/S +1500 | [-] || [-] | 0.780 SPD    V/S ----- | [-] |
             *     |                        |     ||     |                        |     |
             * [-] |<MACH              FLCH>| [-] || [-] |<KTS                IAS>| [-] |
             *     |                        |     ||     |                        |     |
             * [-] |                        | [-] || [-] |                        | [-] |
             *     |                        |     ||     |                        |     |
             * [-] |                        | [-] || [-] |                        | [-] |
             *     |                        |     ||     |                        |     |
             *     ––––––––––––––––––––––––––     ||     ––––––––––––––––––––––––––     |
             *
             * and variants thereof. Check X-Crafts/Tekton visual presentation for ideas.
             * LSK mode selectors: white color. Center labels: T.B.D. (probably green??).
             * Note: values blue unless dashed or not engaged, e.g. cur. HDG in NAV mode.
             * Dashed/unselected values not same as center label e.g. white if green etc.
             *
             * Heck, let's even add mode annunciators at the top/bottom, like on a PFD??
             */
        }
        return 0;
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
        return 1; // bypass widget's built-in drawing code
    }
    return 0;
}

#define COLR2CAIRO(context, color) (cairo_set_source_rgb(context, color[0], color[1], color[2]))
static void draw_display(yfms_context *yfms)
{
    char buf[5]; int x0, y0, x1, y1;

    // draw display background
    cairo_rectangle(CAIRO4SCRN(cr), 0, 0, CAIRO4SCRN(data.wid), CAIRO4SCRN(data.hei));
    cairo_set_operator(CAIRO4SCRN(cr), CAIRO_OPERATOR_SOURCE);
    COLR2CAIRO(CAIRO4SCRN(cr), COLR_SCREEN);
    cairo_fill(CAIRO4SCRN(cr));

    // font properties, options, size etc.
    cairo_set_scaled_font(CAIRO4SCRN(cr), CAIRO4SCRN(sfnt));

    // actual text rendering
    for (int i = 0; i < YFS_DISPLAY_NUMR; i++)
    {
        XPGetWidgetGeometry(yfms->mwindow.screen.subw_id,    &x0, NULL, NULL, &y0);
        XPGetWidgetGeometry(yfms->mwindow.screen.line_id[i], &x1, NULL, NULL, &y1);
        for (int j = 0; j < YFS_DISPLAY_NUMC; j++)
        {
            switch (yfms->mwindow.screen.colr[i][j])
            {
                case COLR_IDX_BLACK:
                    COLR2CAIRO(CAIRO4SCRN(cr), COLR_BLACK);
                    break;
                case COLR_IDX_BLUE:
                    COLR2CAIRO(CAIRO4SCRN(cr), COLR_BLUE);
                    break;
                case COLR_IDX_MAGENTA:
                    COLR2CAIRO(CAIRO4SCRN(cr), COLR_MAGENTA);
                    break;
                case COLR_IDX_ORANGE:
                    COLR2CAIRO(CAIRO4SCRN(cr), COLR_ORANGE);
                    break;
                case COLR_IDX_RED:
                    COLR2CAIRO(CAIRO4SCRN(cr), COLR_RED);
                    break;
                case COLR_IDX_WHITE:
                    COLR2CAIRO(CAIRO4SCRN(cr), COLR_WHITE);
                    break;
                case COLR_IDX_YELLOW:
                    COLR2CAIRO(CAIRO4SCRN(cr), COLR_YELLOW);
                    break;
                case COLR_IDX_GREEN:
                default:
                    COLR2CAIRO(CAIRO4SCRN(cr), COLR_GREEN);
                    break;
            }
            switch (yfms->mwindow.screen.text[i][j])
            {
                case '#':
                    snprintf(buf, sizeof(buf), "%s", CAIRO4SCRN(whitebox));
                    break;
                case '^':
                    snprintf(buf, sizeof(buf), "%s", CAIRO4SCRN(fly_over));
                    break;
                case '\0':
                    goto next_character; // no drawing required
                case '_':
                    if (i == YFS_DISPLAY_NUMR - 1)
                    {
                        // scratchpad: parser strips trailing spaces, we store underscores
                        // instead; here: restore character initially intended for display
                        goto next_character; // whitespace doesn't need drawing
                    }
                    // fall through
                default:
                    snprintf(buf, sizeof(buf), "%c", yfms->mwindow.screen.text[i][j]);
                    break;
            }
            cairo_text_extents_t te; cairo_text_extents(CAIRO4SCRN(cr), buf, &te);
            double xalign = (double)YFS_FONT_BASIC_W / 2.0 - te.width / 2.0 - te.x_bearing - 1.0;
            if ((double)YFS_FONT_BASIC_W == te.x_advance) xalign = 0.0; // gives slightly better results with e.g. Monaco
            cairo_move_to(CAIRO4SCRN(cr), (double)(x1 - x0) + xalign, (double)(y1 - y0));
            cairo_show_text(CAIRO4SCRN(cr), buf);
        next_character:
            x1 += YFS_FONT_BASIC_W;
            continue;
        }
    }

    /* done */
    cairo_surface_flush(CAIRO4SCRN(surf));
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
            yfms_context *yfms = (yfms_context*)XPGetWidgetProperty(inWidget, xpProperty_Refcon, NULL);
            if (yfms == NULL)
            {
                ndt_log("YFMS [debug]: no context for MCDU display!\n");
                return 0;
            }
            if (yfms->mwindow.screen.redraw)
            {
                draw_display(yfms); yfms->mwindow.screen.redraw = 0;
            }
            int g[4]; XPGetWidgetGeometry(inWidget, &g[0], &g[1], &g[2], &g[3]);
            XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0); glRasterPos4i(g[0] - 1, g[3] - 1, 0, 1);
            glDrawPixels(CAIRO4SCRN(data.wid), CAIRO4SCRN(data.hei), GL_BGRA, GL_UNSIGNED_BYTE, CAIRO4SCRN(data.buf));
        }
        return 1;
    }
    return 0;
}

static int yfs_captionh(XPWidgetMessage inMessage,
                        XPWidgetID      inWidget,
                        intptr_t        inParam1,
                        intptr_t        inParam2)
{
    if (inMessage == xpMsg_Draw)
    {
        return 1;
    }
    return 0;
}

static int chandler_tog(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        yfms_context  *yfms = inRefcon;
        yfs_main_toggl(yfms);
    }
    return 0;
}

#undef COLR2CAIRO
#undef CAIRO4SCRN
