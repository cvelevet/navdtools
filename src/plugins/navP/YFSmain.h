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

#include <stdarg.h>
#include <stdint.h>

#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMMenus.h"
#include "XPLM/XPLMProcessing.h"
#include "XPLM/XPLMNavigation.h"
#include "XPLM/XPLMUtilities.h"

#include "assert/includes.h"

#include "common/common.h"
#include "lib/flightplan.h"
#include "lib/navdata.h"
#include "lib/waypoint.h"

// TODO: when new aircraft loaded, reset YFMS automatically

#define YFS_DISPLAY_NUMC 24 // number of positions per row
#define YFS_DISPLAY_NUMR 14 // number of rows
#define YFS_ROW_BUF_SIZE (YFS_DISPLAY_NUMC + 1)

typedef void (*YFS_SPC_f)(void *yfms                             );
typedef void (*YFS_LSK_f)(void *yfms, int key[2], intptr_t refcon);
typedef void (*YFS_MSW_f)(void *yfms, int rx, int ry, int delta_v);

typedef struct
{
    // callbacks for each special key: ensure we can switch to page, then do it
    struct
    {
        YFS_SPC_f cback_drto;
        YFS_SPC_f cback_prog;
        YFS_SPC_f cback_perf;
        YFS_SPC_f cback_init;
        YFS_SPC_f cback_data;
        YFS_SPC_f cback_fpln;
        YFS_SPC_f cback_radn;
        YFS_SPC_f cback_fuel;
        YFS_SPC_f cback_sflp;
        YFS_SPC_f cback_atcc;
        YFS_SPC_f cback_menu;
        YFS_SPC_f cback_arpt;
        YFS_SPC_f cback_null;
        YFS_SPC_f cback_left;
        YFS_SPC_f cback_lnup;
        YFS_SPC_f cback_rigt;
        YFS_SPC_f cback_lndn;
    }
    spcs;

    // callbacks for all line select keys (must be implemented by each page)
    struct
    {
        YFS_LSK_f cback;
        intptr_t refcon;
    }
    lsks[2][6];

    // mouse event callbacks (one of each per page)
    YFS_MSW_f mousew_callback;

    struct
    {
        XPWidgetID id;
        int win_state;
        int ks_rgstrd;
        enum
        {
            YFS_KSM_OFF,
            YFS_KSM_DSP,
            YFS_KSM_NUM,
            YFS_KSM_WIN,
            YFS_KSM_ALL,
        }
        ks_mode;
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
            XPWidgetID keyid_drto;
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
            uint8_t    bgra_pix_buf[256][256][4];
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
            }    spad_color; // declare an unused variable (ensure GCC is happy)
            int  spad_reset;
            int  spad_backup;
            char spad_bupbuf[YFS_DISPLAY_NUMC + 1];
        }
        screen;

        enum
        {
            PAGE0     =  0,
            PAGE_IDNT =  1,
            PAGE_MENU =  2,
            PAGE_RAD1 =  3,
            PAGE_RAD2 =  4,
            PAGE_PROG =  5,
            PAGE_INIT =  6,
            PAGE_PERF =  7,
            PAGE_FPLN =  8,
            PAGE_DRTO =  9,
            PAGE_DATA = 10,
            PAGE_FUEL = 11,
        }
        current_page;
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

    struct
    {
        ndt_position aircraft_pos;
        ndt_distance trp_altitude;
        struct
        {
            int          ialized; // valid origin and destination
            int          aligned; // fake IRS system is aligned
            int       cost_index; // requested performance
            char    flight_id[9]; // flight "number"
            char  corte_name[11]; // company route name
            ndt_distance crz_alt; // initial cruise altitude
            ndt_distance trans_a; // transition altitude in climb
            ndt_distance trans_l; // transition f. level in descent
            ndt_airport *from;    // depart. airport
            ndt_airport *to;      // arrival airport
        }
        init;
        struct
        {
            int               lg_idx; // currently tracked leg in list
            int               ln_off; // currently topmost line offset
            int               dindex; // index of d_leg (in legs list)
            int               usridx; // user-defined waypoint, 01->20
            ndt_waypoint *usrwpt[20]; // user-defined waypoint, 01->20
            ndt_waypoint       *w_tp; // present pos. waypoint for DIR TO
            ndt_flightplan    *d_fpl; // last non-empty flight plan element
            ndt_route_leg     *d_leg; // leg to the arrival airport or runway
            ndt_list           *legs; // list of all flight plan legs, if any
            struct
            {
                int       ref_leg_id; // reference leg for distance calculation
                ndt_distance  remain; // remaining distance after reference leg
            } dist;
            struct
            {
                ndt_route_leg *source; // may be invalid + non-NULL (closed leg)
                void          *opaque;
                int             index;
                enum
                {
                    YFS_FPLN_MOD_NONE = 0,
                    YFS_FPLN_MOD_INIT, // flight plan (re-)initialized
                    YFS_FPLN_MOD_DCTO, // direct to some waypoint
                    YFS_FPLN_MOD_SNGL, // remove/add a single leg
                    YFS_FPLN_MOD_MULT, // remove/add multiple legs
                    YFS_FPLN_MOD_SIDP, // changed SID, or transition
                    YFS_FPLN_MOD_STAR, // changed STAR or transition
                    YFS_FPLN_MOD_APPR, // changed final appr./trans.
                    YFS_FPLN_MOD_OTHR, // other, must check anything
                }
                operation;
            }
            mod;
            struct
            {
                XPLMNavRef waypoint;
                int        altitude;
                float      latitude;
                float      longitud;
                int        legindex;
                ndt_restriction cst;
            }
            xplm_info[100];
            int xplm_last;
            struct
            {
                ndt_route_leg *leg;
                ndt_waypoint  *wpt;
                int            idx;
                int           open;
            }
            lrev;
            struct
            {
                ndt_route_leg     *leg;
                ndt_waypoint      *wpt;
                int                idx;
                int               open;
                ndt_waypoint   *dst[5];
                ndt_airway     *awy[5];
                ndt_airway_leg *lgi[5];
                ndt_airway_leg *lgo[5];
            }
            awys;
        }
        fpln;
        struct
        {
            int           ln_off; // currently topmost line offset
            int           idx[5]; // leg index for corresponding line select key
            int           dctidx; // leg index for pre-selected direct to
            ndt_route_leg *dctlg; // leg pntr. for pre-selected direct to
            ndt_waypoint  *dctwp; // new wayp. for pre-selected direct to
        }
        drto;
        struct
        {
            ndt_waypoint *fix;
        }
        prog;
        struct
        {
            XPLMCommandRef delayed_swap;
            int32_t asrt_delayed_baro_s;
            int32_t asrt_delayed_baro_u;
            int32_t asrt_delayed_baro_v;
            int32_t asrt_delayed_redraw;
        }
        rdio;
    }
    data;

    struct
    {
        char xsystem_pth[513];
        ndt_navdatabase  *ndb;
        struct
        {
            ndt_flightplan *arr;
            ndt_flightplan *dep;
            ndt_flightplan *iac;
            ndt_flightplan *rte;
        } flp;
        struct
        {
            int unit; // barometric pressure: InHg (0), hPa (1)
        } alt;
    }
    ndt;

    struct
    {
        // PAGE_IDNT
        XPLMDataRef acf_ICAO;
        XPLMDataRef acf_en_type;
        XPLMDataRef acf_num_engines;
        // PAGE_RAD1
        XPLMCommandRef com1_standy_flip;
        XPLMCommandRef com2_standy_flip;
        XPLMCommandRef transponder_ident;
        XPLMDataRef transponder_id;
        XPLMDataRef transponder_mode;
        XPLMDataRef transponder_code;
        XPLMDataRef com1_frequency_Mhz;
        XPLMDataRef com1_standby_frequency_Mhz;
        XPLMDataRef com1_frequency_khz;
        XPLMDataRef com1_standby_frequency_khz;
        XPLMDataRef com2_frequency_Mhz;
        XPLMDataRef com2_standby_frequency_Mhz;
        XPLMDataRef com2_frequency_khz;
        XPLMDataRef com2_standby_frequency_khz;
        XPLMDataRef com1_left_frequency_hz_833;
        XPLMDataRef com2_left_frequency_hz_833;
        XPLMDataRef barometer_setting_in_hg_pilot;
        XPLMDataRef barometer_setting_in_hg_copilot;
        // PAGE_RAD2
        XPLMDataRef nav1_type;
        XPLMDataRef nav2_type;
        XPLMDataRef adf1_nav_id;
        XPLMDataRef adf2_nav_id;
        XPLMDataRef nav1_nav_id;
        XPLMDataRef nav2_nav_id;
        XPLMDataRef autopilot_source;
        XPLMDataRef adf1_frequency_hz;
        XPLMDataRef adf2_frequency_hz;
        XPLMDataRef nav1_frequency_hz;
        XPLMDataRef nav2_frequency_hz;
        XPLMDataRef nav1_obs_deg_mag_pilot;
        XPLMDataRef nav2_obs_deg_mag_pilot;
        XPLMDataRef nav1_obs_deg_mag_copilot;
        XPLMDataRef nav2_obs_deg_mag_copilot;
        XPLMDataRef HSI_source_select_pilot;
        XPLMDataRef HSI_source_select_copilot;
        XPLMDataRef nav1_course_deg_mag_pilot;
        XPLMDataRef nav2_course_deg_mag_pilot;
        // PAGE_FPLN
        XPLMCommandRef direct_to;
        // miscellaneous
        XPLMDataRef true_psi;
        XPLMDataRef latitude;
        XPLMDataRef longitude;
        XPLMDataRef elevation;
        XPLMDataRef tropopause;
        XPLMDataRef groundspeed;
        // autopilot
        XPLMDataRef machno;
        XPLMDataRef vvi_fpm_pilot;
        XPLMDataRef airspeed_is_mach;
        XPLMDataRef altitude_ft_pilot;
        XPLMDataRef airspeed_kts_pilot;
        XPLMDataRef airspeed_dial_kts_mach;
        XPLMCommandRef knots_mach_toggle;
        // miscellaneous
        XPLMFlightLoop_f fl_callback;
        enum
        {
            YFS_ATYP_NSET =  0,
            YFS_ATYP_XPLN =  1,
            YFS_ATYP_QPAC = 32,
            YFS_ATYP_Q350 = 35,
            YFS_ATYP_Q380 = 38,
            YFS_ATYP_IXEG = 73,
            YFS_ATYP_FB76 = 76,
            YFS_ATYP_FB77 = 77,
            YFS_ATYP_ASRT = 99,
        }
        atyp;
        struct
        {
            XPLMDataRef xpdr_mode_act;
            XPLMDataRef xpdr_stby_act;
            XPLMDataRef radios_adf1_100_act;
            XPLMDataRef radios_adf1_010_act;
            XPLMDataRef radios_adf1_001_act;
            XPLMDataRef radios_adf2_100_act;
            XPLMDataRef radios_adf2_010_act;
            XPLMDataRef radios_adf2_001_act;
            XPLMDataRef baro_inhg_sby_0001_ind;
        } ixeg;
        struct
        {
            XPLMDataRef XPDRPower;
            XPLMDataRef XPDRAltitude;
            XPLMDataRef BaroStdCapt;
            XPLMDataRef BaroStdFO;
            XPLMCommandRef VHF2Co;
            XPLMCommandRef VHF1Capt;
            XPLMCommandRef RMPSwapCo;
            XPLMCommandRef RMPSwapCapt;
            XPLMCommandRef RMP1FreqUpLrg;
            XPLMCommandRef RMP2FreqUpLrg;
            XPLMCommandRef RMP1FreqUpSml;
            XPLMCommandRef RMP2FreqUpSml;
            XPLMCommandRef RMP1FreqDownLrg;
            XPLMCommandRef RMP2FreqDownLrg;
            XPLMCommandRef RMP1FreqDownSml;
            XPLMCommandRef RMP2FreqDownSml;
        }
        qpac;
        struct
        {
            XPLMDataRef pressLeftButton;
            XPLMDataRef pressLeftRotary;
            XPLMDataRef pressRightButton;
            XPLMDataRef pressRightRotary;
        }
        q350;
        struct
        {
            XPLMDataRef BaroStdCapt;
            XPLMDataRef BaroStdFO;
        }
        q380;
        struct
        {
            XPLMDataRef systemMode;
            XPLMDataRef baroRotary_stby;
            XPLMDataRef baroRotary_left;
            XPLMDataRef baroRotary_right;
            XPLMDataRef nav2_nav_id;
            XPLMDataRef nav2_frequency_hz;
            XPLMDataRef nav2_obs_deg_mag_pilot;
            XPLMDataRef leftBigRotary;
            XPLMDataRef leftMidRotary;
            XPLMDataRef leftSmallRotary;
            XPLMDataRef rightBigRotary;
            XPLMDataRef rightMidRotary;
            XPLMDataRef rightSmallRotary;
        }
        fb76;
        struct
        {
            XPLMDataRef anim_25_rotery;
            XPLMDataRef anim_85_switch;
            XPLMDataRef anim_175_button;
        }
        fb77;
        struct
        {
            XPLMDataRef cl3_fms_selector;
            XPLMDataRef CruiseSpeed_Mach;
            XPLMDataRef CruiseSpeed_KIAS;
            XPLMDataRef HA4T_shared_KIAS;
            XPLMDataRef E175_mouse_x_pos;
//          XPLMDataRef E195_mouse_x_pos; // TODO
        }
        tekt;
        struct
        {
            XPLMPluginID          xid;
            SharedValuesInterface api;
            struct
            {
                int id_u32_altr;
                int id_u32_code;
                int id_u32_mode;
                int id_u32_tcas;
                int id_u32_tmod;
            }
            xpdr;
            struct
            {
                int id_s32_lmode;
                int id_s32_rmode;
                int id_u32_lunit;
                int id_u32_runit;
                int id_f32_lunip;
                int id_f32_runip;
                int id_s32_lvalu;
                int id_s32_rvalu;
            }
            baro;
        }
        asrt;
        struct
        {
            int vmax_auto;
            int vmax_flch;
            int vmax_kias;
            int vmax_mach;
            float flt_vmo;
        }
        otto;
        struct
        {
            int frequency_changed;
        }
        ils;
    }
    xpl;
}
yfms_context;

void*         yfs_main_init (void                            );
void          yfs_main_toggl(yfms_context*                   );
void          yfs_main_rline(yfms_context*,   int, int       );
int           yfs_main_close(yfms_context**                  );
int           yfs_main_newpg(yfms_context*,   int            );
ndt_waypoint* yfs_main_getwp(yfms_context*,        char*     );
ndt_waypoint* yfs_main_usrwp(yfms_context*,        char*     );
void          yfs_printf_lft(void*, int, int, int, char*, ...);
void          yfs_printf_rgt(void*, int, int, int, char*, ...);
void          yfs_printf_ctr(void*, int,      int, char*, ...);

#endif /* YFS_MAIN_H */
