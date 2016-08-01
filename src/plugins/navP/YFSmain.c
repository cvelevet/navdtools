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

#include "Widgets/XPStandardWidgets.h"
#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDisplay.h"
#include "XPLM/XPLMGraphics.h"
#include "XPLM/XPLMMenus.h"
#include "XPLM/XPLMUtilities.h"

#include "common/common.h"

#include "YFSmain.h"

static int YFS_FONT_BASIC_W =   8; // 2 + xplmFont_Basic width
static int YFS_FONT_BASIC_H =  14; // 4 + xplmFont_Basic height
static int YFS_MAINSCREEN_W = 208; // 2 + 24 characters per line
static int YFS_MAINSCREEN_H = 154; // 11 lines x 24 usable characters
static int YFS_MAINWINDOW_W = 420; // display + up to 10 buttons across
static int YFS_MAINWINDOW_H = 420; // display + up to 6 rows in lower half
static int YFS_SOFT_KEY_1_W =  42; // 9 buttons in 420 pixels, 3 separators
static int YFS_SOFT_KEY_1_H =  32; // 6 buttons in 210 pixels, 1 separator
static int YFS_SOFT_KEY_1_B =   2; // 2x 2 pixels of border between each button
static int YFS_SOFT_KEY_2_W =  36; // 7 buttons in 252 pixels (6x soft_key_1_w)
static int YFS_SOFT_KEY_2_H =  32; // 6 buttons in 210 pixels
static int YFS_SOFT_KEY_2_B =   2; // 2x 2 pixels of border between each button
static float COLR_BLACK  [] = { 1.0f, 1.0f, 1.0f, };
static float COLR_WHITE  [] = { 0.0f, 0.0f, 0.0f, };
static float COLR_RED    [] = { 1.0f, 0.0f, 0.0f, };
static float COLR_GREEN  [] = { 0.0f, 1.0f, 0.0f, };
static float COLR_BLUE   [] = { 0.0f, 0.0f, 1.0f, };
enum
{
    IDX_BLACK,
    IDX_WHITE,
    IDX_RED  ,
    IDX_GREEN,
    IDX_BLUE ,
};

typedef struct
{
    struct
    {
       XPLMMenuID id;
        struct
        {
            int toggle;
        }
        items;
    }
    menu;

    struct
    {
        XPWidgetID id;
        int win_state;
        struct
        {
            // line select keys
            XPWidgetID keyid_lsk[5];
            XPWidgetID keyid_rsk[5];
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
            // row 2, right-to-left
            XPWidgetID keyid_num6;
            XPWidgetID keyid_num5;
            XPWidgetID keyid_num4;
            XPWidgetID keyid_next;
            XPWidgetID keyid_menu;
            XPWidgetID keyid_tune;
            XPWidgetID keyid_perf;
            XPWidgetID keyid__fpl;
            XPWidgetID keyid_fuel;
            // row 3, right-to-left
            XPWidgetID keyid_num9;
            XPWidgetID keyid_num8;
            XPWidgetID keyid_num7;
            XPWidgetID keyid_al_g;
            XPWidgetID keyid_al_f;
            XPWidgetID keyid_al_e;
            XPWidgetID keyid_al_d;
            XPWidgetID keyid_al_c;
            XPWidgetID keyid_al_b;
            XPWidgetID keyid_al_a;
            // row 4, right-to-left
            XPWidgetID keyid__msg;
            XPWidgetID keyid_num0;
            XPWidgetID keyid_back;
            XPWidgetID keyid_al_n;
            XPWidgetID keyid_al_m;
            XPWidgetID keyid_al_l;
            XPWidgetID keyid_al_k;
            XPWidgetID keyid_al_j;
            XPWidgetID keyid_al_i;
            XPWidgetID keyid_al_h;
            // row 5, right-to-left
            XPWidgetID keyid_plus;
            XPWidgetID keyid_onof;
            XPWidgetID keyid_al_t;
            XPWidgetID keyid_al_s;
            XPWidgetID keyid_al_r;
            XPWidgetID keyid_al_q;
            XPWidgetID keyid_al_p;
            XPWidgetID keyid_al_o;
            // row 6, right-to-left
            XPWidgetID keyid_entr;
            XPWidgetID keyid_al_z;
            XPWidgetID keyid_al_y;
            XPWidgetID keyid_al_x;
            XPWidgetID keyid_al_w;
            XPWidgetID keyid_al_v;
            XPWidgetID keyid_al_u;
        }
        keys;

        struct
        {
            int   colr[11][24];
            char  text[11][25];
            XPWidgetID bgrd_id;
            XPWidgetID subw_id;
            int        sw_inBM;
            int        sw_inLT;
            int        sw_inTP;
            int        sw_inRT;
            XPWidgetID line_id[11];
            int        ln_inBM[11];
            int        ln_inLT[11];
            int        ln_inTP[11];
            int        ln_inRT[11];
        }
        screen;

        struct
        {
            int                    before;
            void                  *refcon;
            XPLMCommandRef        command;
            XPLMCommandCallback_f handler;
        }
        toggle;
    }
    mwindow;
}
yfms_context;

static void menu_handler(void *inMenuRef,                void *inItemRef);
static int  yfs_mwindowh(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
static int  yfs_mcdubgrh(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
static int  yfs_mcdudish(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
static int  chandler_tog(XPLMCommandRef, XPLMCommandPhase, void*inRefcon);

/*
 * TODO: xpWidgetClass_Button limited to 15px height; use custom widget instead.
 *
 * In the meantime, center the button vertically, and use an
 * other widget behind it to draw some sort of "background". // TODO still
 */
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
     * Add a bunch of not-so-pretty buttons.
     *
     * Width:  we have 9, or 10 keys across and 3 "separators".
     * Height: we always have 6 keys across and 1 "separator" in half the total height.
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
    // row 1
    {
        softkey.inTP -= separatrH; // note1
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
    {
        softkey.inTP += separatrH; // note1
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
    // row 2
    {
        softkey.inTP -= separatrH; // note1
    }
    {
        softkey.inRT = keybordRT - separatrW;           // new row, back right
        softkey.inTP = softkey.inTP - YFS_SOFT_KEY_1_H; // lose 1 row height
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
    {
        softkey.inTP += separatrH; // note1
        softkey.inRT -= separatrW; // horizontal separator
    }
    softkey.desc = "NEXT";
    softkey._wid = &yfms->mwindow.keys.keyid_next;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "MENU";
    softkey._wid = &yfms->mwindow.keys.keyid_menu;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "TUNE";
    softkey._wid = &yfms->mwindow.keys.keyid_tune;
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
    softkey.desc = "FPL";
    softkey._wid = &yfms->mwindow.keys.keyid__fpl;
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
    // row 3
    {
        softkey.inTP -= separatrH; // note1
    }
    {
        softkey.inRT = keybordRT - separatrW;           // new row, back right
        softkey.inTP = softkey.inTP - YFS_SOFT_KEY_1_H; // lose 1 row height
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
    {
        softkey.inTP = softkey.inTP + separatrH; // note1
        softkey.inTP = softkey.inTP - separatrH; // vertical separator
        softkey.inRT = softkey.inRT - separatrW; // horizon. separator
        softkey.btnW                = YFS_SOFT_KEY_2_W; // nar. key width
        softkey.btnH                = YFS_SOFT_KEY_2_H; // same key height
        softkey.btBW = softkey.btBH = YFS_SOFT_KEY_2_B; // same borders
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
    // row 4
    {
        softkey.inTP -= separatrH; // note1
    }
    {
        softkey.inRT = keybordRT    - separatrW; // new row, back right
        softkey.inTP = softkey.inTP + separatrH - YFS_SOFT_KEY_1_H; // align h.
        softkey.btnW                = YFS_SOFT_KEY_1_W; // reset key width
        softkey.btnH                = YFS_SOFT_KEY_1_H; // reset key height
        softkey.btBW = softkey.btBH = YFS_SOFT_KEY_1_B; // reset borders
    }
    softkey.desc = "MSG";
    softkey._wid = &yfms->mwindow.keys.keyid__msg;
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
    softkey.desc = "BACK";
    softkey._wid = &yfms->mwindow.keys.keyid_back;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    {
        softkey.inTP = softkey.inTP + separatrH; // note1
        softkey.inTP = softkey.inTP - separatrH; // vertical separator
        softkey.inRT = softkey.inRT - separatrW; // horizon. separator
        softkey.btnW                = YFS_SOFT_KEY_2_W; // nar. key width
        softkey.btnH                = YFS_SOFT_KEY_2_H; // same key height
        softkey.btBW = softkey.btBH = YFS_SOFT_KEY_2_B; // same borders
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
    // row 5
    {
        softkey.inTP -= separatrH; // note1
    }
    {
        softkey.inRT = keybordRT    - separatrW - YFS_SOFT_KEY_1_W; // -1 key
        softkey.inTP = softkey.inTP + separatrH - YFS_SOFT_KEY_1_H; // align h.
        softkey.btnW                = YFS_SOFT_KEY_1_W; // reset key width
        softkey.btnH                = YFS_SOFT_KEY_1_H; // reset key height
        softkey.btBW = softkey.btBH = YFS_SOFT_KEY_1_B; // reset borders
    }
    softkey.desc = "+/-";
    softkey._wid = &yfms->mwindow.keys.keyid_plus;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    softkey.desc = "DIM";
    softkey._wid = &yfms->mwindow.keys.keyid_onof;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    {
        softkey.inTP = softkey.inTP + separatrH; // note1
        softkey.inTP = softkey.inTP - separatrH; // vertical separator
        softkey.inRT = softkey.inRT - separatrW; // horizon. separator
        softkey.btnW                = YFS_SOFT_KEY_2_W; // nar. key width
        softkey.btnH                = YFS_SOFT_KEY_2_H; // same key height
        softkey.btBW = softkey.btBH = YFS_SOFT_KEY_2_B; // same borders
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
    softkey.desc = "O";
    softkey._wid = &yfms->mwindow.keys.keyid_al_o;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    // row 6
    {
        softkey.inRT = keybordRT    - separatrW - YFS_SOFT_KEY_1_W; // -1 key
        softkey.inTP = softkey.inTP             - YFS_SOFT_KEY_1_H; // align h.
        softkey.btnW           = 2 *  YFS_SOFT_KEY_1_W; // wider key width
        softkey.btnH                = YFS_SOFT_KEY_1_H; // reset key height
        softkey.btBW = softkey.btBH = YFS_SOFT_KEY_1_B; // reset borders
    }
    softkey.desc = "ENTER";
    softkey._wid = &yfms->mwindow.keys.keyid_entr;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
    }
    {
        softkey.inRT = softkey.inRT - separatrW; // horizon. separator
        softkey.btnW                = YFS_SOFT_KEY_2_W; // nar. key width
        softkey.btnH                = YFS_SOFT_KEY_2_H; // same key height
        softkey.btBW = softkey.btBH = YFS_SOFT_KEY_2_B; // same borders
    }
    softkey.desc = "Z";
    softkey._wid = &yfms->mwindow.keys.keyid_al_z;
    if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
    {
        goto create_button_fail;
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

    /*
     * Now the MCDU's screen sub-window and associated labels.
     */
    int align_scW = (YFS_MAINWINDOW_W - YFS_MAINSCREEN_W) / 2;
    inTP =  keybordTP + separatrH - 1 + YFS_MAINSCREEN_H;   // top
    inRT =  mainwinRT - align_scW + 1;                      // right
    inLT =    inRT - YFS_MAINSCREEN_W;                      // left
    for (int i = 0; i < 11; i++)
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
        yfms->mwindow.screen.sw_inBM = yfms->mwindow.screen.ln_inBM[10] - 6;
        yfms->mwindow.screen.sw_inLT = yfms->mwindow.screen.ln_inLT[ 0];
        yfms->mwindow.screen.sw_inTP = yfms->mwindow.screen.ln_inTP[ 0];
        yfms->mwindow.screen.sw_inRT = yfms->mwindow.screen.ln_inRT[ 0];
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
    for (int i = 0; i < 11; i++)
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
            softkey.inRT = yfms->mwindow.screen.ln_inLT[i] - YFS_FONT_BASIC_W;
            softkey.inLT = softkey.inRT - softkey.btnW + 1;
            softkey.desc = "-"; softkey._wid = &yfms->mwindow.keys.keyid_lsk[i/2-1];
            if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
            {
                goto create_button_fail;
            }
            softkey.inLT = yfms->mwindow.screen.ln_inRT[i] + YFS_FONT_BASIC_W;
            softkey.inRT = softkey.inLT + softkey.btnW - 1;
            softkey.desc = "-"; softkey._wid = &yfms->mwindow.keys.keyid_rsk[i/2-1];
            if ((r_value = row_prepend_button(&softkey, yfms->mwindow.id)))
            {
                goto create_button_fail;
            }
        }
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
    XPShowWidget      (yfms->mwindow.id);
    XPSetKeyboardFocus(yfms->mwindow.id);
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
    if ((yfms->mwindow.toggle.command =
         XPLMCreateCommand("YFMS/toggle", "toggle YFMS window")) == NULL)
    {
        ndt_log("YFMS [error]: could not create X-Plane command for toggle\n");
        goto fail;
    }
    XPLMRegisterCommandHandler(yfms->mwindow.toggle.command,
                               yfms->mwindow.toggle.handler = &chandler_tog,
                               yfms->mwindow.toggle.before  = 0,
                               yfms->mwindow.toggle.refcon  = yfms);

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

    /* destroy the main window */
    if (yfms->mwindow.id)
    {
        XPDestroyWidget(yfms->mwindow.id, 1); // destroy children recursively
    }

    /* remove custom command(s) */
    if (yfms->mwindow.toggle.command && yfms->mwindow.toggle.handler)
    {
        XPLMUnregisterCommandHandler(yfms->mwindow.toggle.command,
                                     yfms->mwindow.toggle.handler,
                                     yfms->mwindow.toggle.before,
                                     yfms->mwindow.toggle.refcon);
        yfms->mwindow.toggle.command = NULL;
        yfms->mwindow.toggle.handler = NULL;
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

static void menu_handler(void *inMenuRef, void *inItemRef)
{
    //fixme
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
        return 0; // TODO: handle
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

static int yfs_mcdudish(XPWidgetMessage inMessage,
                        XPWidgetID      inWidget,
                        intptr_t        inParam1,
                        intptr_t        inParam2)
{
    if (inMessage == xpMsg_Draw) // TODO: implement (properly, not inline)
    {
        if (XPIsWidgetVisible(inWidget))
        {
            int y, x; yfms_context *yfms = (yfms_context*)XPGetWidgetProperty(inWidget, xpProperty_Refcon, NULL);
            if (yfms == NULL)
            {
                ndt_log("YFMS [debug]: no context for MCDU display!\n");
                return 0;
            }
            XPGetWidgetGeometry(yfms->mwindow.screen.line_id[10], &x, NULL, NULL, &y);
            XPLMDrawString     (COLR_GREEN, x, y, "READY_", NULL,     xplmFont_Basic);
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
