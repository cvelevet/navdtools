/*
 * YFSmain.h
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

#ifndef YFS_MAIN_H
#define YFS_MAIN_H

#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMMenus.h"
#include "XPLM/XPLMUtilities.h"

// TODO: when new aircraft loaded, reset YFMS automatically

#define YFS_DISPLAY_NUMC 24 // number of positions per row
#define YFS_DISPLAY_NUMR 14 // number of rows

typedef struct
{
    struct
    {
        XPWidgetID id;
        int win_state;
        struct
        {
            // line select keys
            XPWidgetID keyid_lsk[6];
            XPWidgetID keyid_rsk[6];
            // alphanumeric keys
            XPWidgetID keyid_num0;
            XPWidgetID keyid_num1;
            XPWidgetID keyid_num2;
            XPWidgetID keyid_num3;
            XPWidgetID keyid_num4;
            XPWidgetID keyid_num5;
            XPWidgetID keyid_num6;
            XPWidgetID keyid_num7;
            XPWidgetID keyid_num8;
            XPWidgetID keyid_num9;
            XPWidgetID keyid_al_a;
            XPWidgetID keyid_al_b;
            XPWidgetID keyid_al_c;
            XPWidgetID keyid_al_d;
            XPWidgetID keyid_al_e;
            XPWidgetID keyid_al_f;
            XPWidgetID keyid_al_g;
            XPWidgetID keyid_al_h;
            XPWidgetID keyid_al_i;
            XPWidgetID keyid_al_j;
            XPWidgetID keyid_al_k;
            XPWidgetID keyid_al_l;
            XPWidgetID keyid_al_m;
            XPWidgetID keyid_al_n;
            XPWidgetID keyid_al_o;
            XPWidgetID keyid_al_p;
            XPWidgetID keyid_al_q;
            XPWidgetID keyid_al_r;
            XPWidgetID keyid_al_s;
            XPWidgetID keyid_al_t;
            XPWidgetID keyid_al_u;
            XPWidgetID keyid_al_v;
            XPWidgetID keyid_al_w;
            XPWidgetID keyid_al_x;
            XPWidgetID keyid_al_y;
            XPWidgetID keyid_al_z;
            // special keys
            XPWidgetID keyid_clir;
            XPWidgetID keyid_ovfy;
            XPWidgetID keyid_plus;
            XPWidgetID keyid_pird;
            XPWidgetID keyid_spce;
            XPWidgetID keyid_slsh;
            // FMS pages
            XPWidgetID keyid_dirt;
            XPWidgetID keyid_prog;
            XPWidgetID keyid_perf;
            XPWidgetID keyid_init;
            XPWidgetID keyid_data;
            XPWidgetID keyid_fpln;
            XPWidgetID keyid_radn;
            XPWidgetID keyid_fuel;
            XPWidgetID keyid_sfpl;
            XPWidgetID keyid_atcc;
            XPWidgetID keyid_menu;
            XPWidgetID keyid_arpt;
            XPWidgetID keyid_null;
            XPWidgetID keyid_left;
            XPWidgetID keyid_rigt;
            XPWidgetID keyid_lnup;
            XPWidgetID keyid_lndn;
        }
        keys;

        struct
        {
            XPWidgetID bgrd_id;
            XPWidgetID subw_id;
            int        sw_inBM;
            int        sw_inLT;
            int        sw_inTP;
            int        sw_inRT;
            XPWidgetID line_id[YFS_DISPLAY_NUMR];
            int        ln_inBM[YFS_DISPLAY_NUMR];
            int        ln_inLT[YFS_DISPLAY_NUMR];
            int        ln_inTP[YFS_DISPLAY_NUMR];
            int        ln_inRT[YFS_DISPLAY_NUMR];
            int           colr[YFS_DISPLAY_NUMR][YFS_DISPLAY_NUMC];
            char          text[YFS_DISPLAY_NUMR][YFS_DISPLAY_NUMC + 1];
            enum
            {
                COLR_IDX_BLACK   = 0,
                COLR_IDX_BLUE    = 1,
                COLR_IDX_GREEN   = 2,
                COLR_IDX_MAGENTA = 3,
                COLR_IDX_RED     = 4,
                COLR_IDX_WHITE   = 5,
                COLR_IDX_ORANGE  = 6,
                COLR_IDX_YELLOW  = 7,
            };
        }
        screen;
    }
    mwindow;

    struct
    {
        int                    before;
        void                  *refcon;
        XPLMCommandRef        command;
        XPLMCommandCallback_f handler;
    }
    toggle;

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
}
yfms_context;

void* yfs_main_init (void                );
void  yfs_main_toggl(void   *yfms_context);
int   yfs_main_close(void **_yfms_context);

#endif /* YFS_MAIN_H */
