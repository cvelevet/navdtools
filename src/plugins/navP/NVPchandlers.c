/*
 * NVPchandlers.c
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2015 Timothy D. Walker and others.
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
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if APL
#include <CoreGraphics/CoreGraphics.h>
#endif

#include "Widgets/XPStandardWidgets.h"
#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMDisplay.h"
#include "XPLM/XPLMMenus.h"
#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMProcessing.h"
#include "XPLM/XPLMUtilities.h"

#include "assert/includes.h"

#include "common/common.h"

#include "ACFtypes.h"
#include "ACFvolumes.h"
#include "NVPchandlers.h"
#include "NVPmenu.h"

typedef struct
{
    int                    before;
    void                  *refcon;
    XPLMCommandRef        command;
    XPLMCommandCallback_f handler;
} chandler_callback;

typedef struct
{
    const char    *name;
    XPLMCommandRef xpcr;
} chandler_command;

typedef struct
{
    void        *assert;
    float       rtio[3];
    float       flt_var;
    int         int_var;
    int         use_pkb;
    XPLMDataRef p_b_flt;
    XPLMDataRef p_b_int;
    XPLMDataRef p_b_rat;
    XPLMDataRef l_b_rat;
    XPLMDataRef r_b_rat;
    XPLMDataRef protate;
    XPLMDataRef g_speed;
    XPLMDataRef a_b_lev;
    XPLMCommandRef abto;
    XPLMCommandRef pcmd;
    chandler_command rg;
    chandler_command mx;
    chandler_command ro;
} refcon_braking;

typedef struct
{
    acf_info_context *info;
    struct
    {
        struct
        {
            chandler_callback cb;
        } tog;

        struct
        {
            chandler_callback cb;
        } fwd;

        struct
        {
            chandler_callback cb;
        } bet;

        struct
        {
            chandler_callback cb;
        } rev;

        void          *assert;
        XPLMDataRef prop_mode;
        XPLMCommandRef propdn;
        XPLMCommandRef propup;
        XPLMCommandRef tgb[9];
        XPLMCommandRef tgr[9];
    } rev;

    chandler_callback mu;
    chandler_callback md;
    XPLMDataRef mixratio;

    chandler_callback rp;
    XPLMDataRef rpmratio;

    chandler_callback dn;
    XPLMCommandRef thrdn;

    chandler_callback pt;
    XPLMCommandRef thptt;
    int atc_is_connected;

    chandler_callback dd;
    XPLMCommandRef thrdo;

    chandler_callback up;
    chandler_callback ul;
    XPLMCommandRef thrup;

    chandler_callback uu;
    XPLMCommandRef thrul;

    XPLMDataRef tbm9erng;

    XPLMDataRef throttle;
} refcon_thrust;

typedef struct
{
    int              ready;
    chandler_callback mwcb;
    XPLMDataRef    popup_x;
    XPLMDataRef    popup_y;
    XPLMDataRef    xpliver;
} refcon_tolifbw;

typedef struct
{
    int    kc_is_registered;
    XPLMDataRef    datar[5];
    XPLMCommandRef c[3][48];
} refcon_a319kbc;

typedef struct
{
    int kc_is_registered;
    XPLMDataRef datar[1];
    XPLMCommandRef c[10];
} refcon_a350kbc;

typedef struct
{
    int        ready;
    XPLMDataRef slat;
} refcon_ixeg733;

typedef struct
{
    int            ready;
    XPLMCommandRef sparm;
    XPLMCommandRef spret;
    XPLMCommandRef pt_up;
    XPLMCommandRef pt_dn;
} refcon_eadt738;

typedef struct
{
#if ((APL) && (CGFLOAT_IS_DOUBLE))
    struct
    {
        int current_evnt_index;
        CGEventRef     evnt[6];
        XPLMCommandRef cmmd[6];
        XPLMFlightLoop_f flc_t;
    } tbm9;
#endif
    const char          *auth;
    const char          *desc;
    const char          *icao;
    int                  atyp;
    int            i_cycletyp;
    int            i_cycle_id;
    int            i_disabled;
    int            i_value[2];
    XPLMDataRef    dataref[4];
    XPLMCommandRef command[6];
} refcon_cdu_pop;

typedef struct
{
    refcon_thrust      *pt;
    void           *assert;
    void         *nvp_menu;
    int  last_cycle_number;
    int  last_second_index;
    float curr_period_durr;
    float elapsed_fr_reset;
    XPLMDataRef auto_p_sts;
    XPLMDataRef auto_t_sts;
    XPLMDataRef elev_m_agl;
    XPLMFlightLoop_f flc_g;
    XPLMDataRef ongrnd_any;
    XPLMDataRef thrott_all;
    struct
    {
        XPLMDataRef vol_com0;
        XPLMDataRef vol_com1;
        XPLMDataRef vol_com2;
        XPLMDataRef pe_is_on;
        XPLMDataRef xb_is_on;
        XPLMFlightLoop_f flc;
        int pe_was_connected;
        int xb_was_connected;
        int aircraft_type[1];
    } oatc;
    struct
    {
        XPLMDataRef     sim_pause;
        XPLMDataRef     view_type;
        XPLMDataRef zulu_time_xpl;
        XPLMDataRef zulu_time_sec;
        XPLMDataRef zulu_time_min;
        XPLMDataRef zulu_time_hrs;
    } time;
} refcon_ground;

typedef struct
{
    chandler_callback landing_gear_toggle;
    chandler_callback   landing_gear_down;
    chandler_callback     landing_gear_up;
    XPLMDataRef          acf_gear_retract;
    XPLMDataRef          gear_handle_down;
    int              has_retractable_gear;
    void                          *assert;
    struct
    {
        XPLMDataRef ref;
        int       atype;
    } callouts;
} refcon_gear;

typedef struct
{
    chandler_callback aft;
    chandler_callback bef;
    XPLMCommandRef at_cmd;
    XPLMDataRef    at_ref;
    XPLMDataRef    fd_ref;
    int            at_val;
} refcon_apd;

typedef struct
{
    XPLMDataRef f_pitch;
    XPLMDataRef vfpitch;
    const char *ap_toga;
    const char *ptrimto;
    float init_cl_speed;
    float init_cl_pitch;
    int vfpitch_array_i;
} refcon_app;

typedef struct
{
    chandler_callback dn;
    chandler_callback up;
    chandler_callback pd;
    chandler_callback pu;
    XPLMCommandRef vs_dn;
    XPLMCommandRef vs_up;
    XPLMDataRef ot_vs_on;
} refcon_vvi;

typedef struct
{
    int reset_count;
    int acf_num_tanks;
    int acf_num_engines;
    float time_elapsed_total;
    XPLMFlightLoop_f flc_fuel;
} refcon_fueltw;

typedef struct
{
    int        initialized;
    int        onrun_items;
    int        first_fcall;
    int        kill_daniel;
    void     *menu_context;
    acf_info_context *info;
    refcon_a319kbc  a319kc;
    refcon_a350kbc  a350kc;
    refcon_fueltw   fueltw;
    refcon_ground   ground;
    refcon_thrust    throt;
    refcon_gear       gear;
    refcon_apd         apd;
    refcon_vvi         vvi;

    struct
    {
        XPLMDataRef  nonp;
        XPLMDataRef  data;
        int   round_value;
        float float_value;
    } fov;

    struct
    {
        int         var_speedbrake;
        XPLMDataRef ref_speedbrake;
        int         var_flap_lever;
        XPLMDataRef ref_flap_lever;
        int         var_gear_lever;
        XPLMDataRef ref_gear_lever;

        // named, plane-specific (hardcoded) flap callouts
        chandler_callback cb_flapu;
        chandler_callback cb_flapd;
        XPLMDataRef ref_flap_ratio;
        XPLMDataRef ref_flaps_e55p;
        XPLMFlightLoop_f flc_flapd;
        XPLMFlightLoop_f flc_flaps;
        XPLMFlightLoop_f flc_flapu;
    } callouts;

    struct
    {
        refcon_tolifbw t319;
        refcon_ixeg733 i733;
        refcon_eadt738 x738;
    } acfspec;

    struct
    {
        chandler_callback cb;
    } turnaround;

    struct
    {
        struct
        {
            chandler_callback cb;
        } ext;

        struct
        {
            chandler_callback cb;
        } ret;

        XPLMDataRef    ha4t;
        XPLMDataRef    srat;
        XPLMCommandRef e55e;
        XPLMCommandRef sext;
        XPLMCommandRef e55r;
        XPLMCommandRef sret;
    } spbrk;

    struct
    {
        struct
        {
            struct
            {
                XPLMCommandRef    cd;
                chandler_callback cb;
            } up;

            struct
            {
                XPLMCommandRef    cd;
                chandler_callback cb;
            } dn;
        } pch;

        struct
        {
            struct
            {
                XPLMCommandRef    cd;
                chandler_callback cb;
            } lt;

            struct
            {
                XPLMCommandRef    cd;
                chandler_callback cb;
            } rt;
        } ail;

        struct
        {
            struct
            {
                XPLMCommandRef    cd;
                chandler_callback cb;
            } lt;

            struct
            {
                XPLMCommandRef    cd;
                chandler_callback cb;
            } rt;
        } rud;
    } trims;

    struct
    {
        struct
        {
            chandler_callback cb;
        } prev;

        struct
        {
            chandler_callback cb;
        } next;

        int               idx[10];
        chandler_callback cbs[10];
    } views;

    struct
    {
        chandler_callback ap_conn;
        chandler_callback ap_disc;
    } asrt;

    struct
    {
        struct
        {
            chandler_callback cb;
            XPLMDataRef      *dr;
        } ffst;

        struct
        {
            chandler_callback cb;
            chandler_command  cc;
        } conn;

        struct
        {
            chandler_callback cb;
            chandler_command  cc;
        } disc;

        struct
        {
            chandler_callback cb;
            chandler_command  cc;
            refcon_app        rc;
        } clmb;
    } otto;

    struct
    {
        chandler_callback cb;
        refcon_cdu_pop    rc;
    } mcdu;

    struct
    {
        XPLMDataRef dataref[8];
        XPLMCommandRef comm[8];
    } debug;

    struct
    {
        chandler_callback cb;
        XPLMCommandRef coatc;
        XPLMPluginID pe_plid;
        XPLMPluginID xb_plid;
    } coatc;

    struct
    {
        XPLMDataRef  axis[3];
        float sensitivity[3];
    } axes;
} chandler_context;

/* Callout default values */
#define CALLOUT_SPEEDBRAK (-1)
#define CALLOUT_FLAPLEVER (-1)
#define CALLOUT_GEARLEVER (-1)

/* Flap lever position name constants */
static       char  _flap_callout_st[11];
static const char* _flap_names_1530[10] = {    "up",    "15",    "30",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_1545[10] = {    "up",    "15",    "45",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_1735[10] = {    "up",    "17",    "35",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_2POS[10] = {    "up",  "half",  "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_1230[10] = {    "up",    "10",    "20",   "30",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_CSNA[10] = {    "up",    "10",    "20", "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_FA78[10] = {    "up",     "1",     "2",    "3",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_HA4T[10] = {    "up",    "12",    "20",   "35",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_NO18[10] = {    "up",     "9",    "22",   "45",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_PC12[10] = {    "up",    "15",    "30",   "40",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_PIPR[10] = {    "up",    "10",    "25",   "40",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_TBM8[10] = { "8 5 0",   "up",  "half",  "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_3POS[10] = {    "up",     "1",     "2", "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_EMB1[10] = {    "up",     "9",    "18",   "22",   "45",   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_4POS[10] = {    "up",     "1",     "2",    "3", "full",   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_B52G[10] = {    "up",    "10",    "20",   "30",   "40",   "55",   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_BOE1[10] = {    "up",     "1",     "5",   "15",   "20",   "25",   "30",   NULL,   NULL,   NULL, };
static const char* _flap_names_DC10[10] = {    "up",     "5",    "15",   "25",   "30",   "35",   "40",   NULL,   NULL,   NULL, };
static const char* _flap_names_EMB2[10] = {    "up",     "1",     "2",    "3",    "4",    "5", "full",   NULL,   NULL,   NULL, };
static const char* _flap_names_MD80[10] = {    "up",     "0",     "5",   "11",   "15",   "28",   "40",   NULL,   NULL,   NULL, };
static const char* _flap_names_C130[10] = {    "up",     "1",     "2",    "3",    "4",    "5",    "6", "full",   NULL,   NULL, };
static const char* _flap_names_BOE2[10] = {    "up",     "1",     "2",    "5",   "10",   "15",   "25",   "30",   "40",   NULL, };
static       void   flap_callout_setst(const char *names[], int index)
{
    if (index < 0) index = 0; else if (index > 9) index = 9;
    if (names[index])
    {
        snprintf(_flap_callout_st, sizeof(_flap_callout_st), "Flaps %s", names[index]);
    }
    else
    {
        snprintf(_flap_callout_st, sizeof(_flap_callout_st), "%s", "");
    }
}
static void flap_callout_speak(void)
{
    if (strnlen(_flap_callout_st, 1))
    {
        XPLMSpeakString(_flap_callout_st);
    }
}

/* Quicklook view index and accessors */
static int         _var_ql_idx = 0; // default view minus 1
static int*         var_ql_idx_get(void)
{
    return &_var_ql_idx;
}
static XPLMDataRef _ref_ql_idx = NULL;
static void         ref_ql_idx_set(XPLMDataRef xdr)
{
    _ref_ql_idx = xdr;
}
static XPLMDataRef  ref_ql_idx_get(void)
{
    return _ref_ql_idx;
}
static int          ref_ql_idx_val(void)
{
    return XPLMGetDatai(_ref_ql_idx);
}

/* Readability macros to (un)register custom command handlers */
#define REGISTER_CHANDLER(_callback, _handler, _before, _refcon)                \
{                                                                               \
    XPLMRegisterCommandHandler(((_callback).command),                           \
                               ((_callback).handler = (&(_handler))),           \
                               ((_callback).before  = (((_before)))),           \
                               ((_callback).refcon  = (((_refcon)))));          \
}
#define UNREGSTR_CHANDLER(_callback)                                            \
{                                                                               \
    if ((_callback).handler)                                                    \
    {                                                                           \
        XPLMUnregisterCommandHandler((_callback).command,                       \
                                     (_callback).handler,                       \
                                     (_callback).before,                        \
                                     (_callback).refcon);                       \
        (_callback).handler = NULL;                                             \
    }                                                                           \
}

/* Convenience macro to automatically check and assign a dataref */
#define _DO(_verbose, _func, _val, _name) { if ((d_ref = XPLMFindDataRef(_name))) _func(d_ref, _val); else if (_verbose) ndt_log("navP [warning]: dataref not found: \"%s\"\n", _name); }

static int tol_keysniffer(char inCh, XPLMKeyFlags inFlags, char inVirtualKey, void *inRefcon);
static int a35_keysniffer(char inCh, XPLMKeyFlags inFlags, char inVirtualKey, void *inRefcon);
static int chandler_turna(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_swtch(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_twosw(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_twos2(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_apclb(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_sp_ex(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_sp_re(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_pt_up(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_pt_dn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_at_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_at_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_rt_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_rt_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_r_tog(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_r_fwd(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_r_bet(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_r_rev(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_mixdn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_mixdt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_mixut(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_rpmdn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_thrdn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_thrup(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_thrul(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_thptt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_thrdd(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_thruu(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_sview(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_qlprv(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_qlnxt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_flchg(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_mcdup(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_ffap1(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_32apc(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_32apd(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_32atd(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_31isc(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_ghndl(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_apbef(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_apaft(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_p2vvi(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_coatc(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static float flc_flap_cmmd (                                        float, float, int, void*);
static float flc_flap_func (                                        float, float, int, void*);
static float flc_oatc_func (                                        float, float, int, void*);
static float gnd_stab_hdlr (                                        float, float, int, void*);
static float fuel_t_w_hdlr (                                        float, float, int, void*);
static float tbm9mousehdlr (                                        float, float, int, void*);
static int   first_fcall_do(                                           chandler_context *ctx);
static int   tliss_fbw_init(                                             refcon_tolifbw *fbw);
static int   boing_733_init(                                             refcon_ixeg733 *i33);
static int   boing_738_init(                                             refcon_eadt738 *x38);
static int   priv_getdata_i(                                   void *inRefcon               );
static void  priv_setdata_i(                                   void *inRefcon, int   inValue);
static float priv_getdata_f(                                   void *inRefcon               );
static void  priv_setdata_f(                                   void *inRefcon, float inValue);

void* nvp_chandlers_init(void)
{
    chandler_context *ctx = calloc(1, sizeof(chandler_context));
    if (!ctx)
    {
        return NULL;
    }

    /* process-global aircraft-specific info context */
    if ((ctx->info = acf_type_info_get()) == NULL)
    {
        goto fail;
    }

    /*
     * Private datarefs: callouts.
     *
     * Defaults are set by the menu code, but we
     * also initialize the variables here anyway.
     *
     * Note: XPLMGet/SetDatai don't seem to work from XPluginStart().
     */
    ctx->callouts.var_speedbrake = CALLOUT_SPEEDBRAK;
    ctx->callouts.ref_speedbrake = XPLMRegisterDataAccessor("navP/callouts/speedbrake",
                                                            xplmType_Int, 1,
                                                            &priv_getdata_i, &priv_setdata_i,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            &ctx->callouts.var_speedbrake,
                                                            &ctx->callouts.var_speedbrake);
    ctx->callouts.var_flap_lever = CALLOUT_FLAPLEVER;
    ctx->callouts.ref_flap_lever = XPLMRegisterDataAccessor("navP/callouts/flap_lever",
                                                            xplmType_Int, 1,
                                                            &priv_getdata_i, &priv_setdata_i,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            &ctx->callouts.var_flap_lever,
                                                            &ctx->callouts.var_flap_lever);
    ctx->callouts.var_gear_lever = CALLOUT_GEARLEVER;
    ctx->callouts.ref_gear_lever = XPLMRegisterDataAccessor("navP/callouts/gear_lever",
                                                            xplmType_Int, 1,
                                                            &priv_getdata_i, &priv_setdata_i,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            &ctx->callouts.var_gear_lever,
                                                            &ctx->callouts.var_gear_lever);
    if (!ctx->callouts.ref_speedbrake ||
        !ctx->callouts.ref_flap_lever ||
        !ctx->callouts.ref_gear_lever)
    {
        ndt_log("navP [error]: nvp_chandlers_init: could not create dataref\n");
        goto fail;
    }

    /* Field of view save/restore */
    if ((ctx->fov.nonp = XPLMFindDataRef("sim/graphics/settings/non_proportional_vertical_FOV")) == NULL ||
        (ctx->fov.data = XPLMFindDataRef(                "sim/graphics/view/field_of_view_deg")) == NULL)
    {
        ndt_log("navP [error]: nvp_chandlers_init: dataref not found\n");
        goto fail;
    }

    /* Custom commands: automatic turnaround */
    ctx->turnaround.cb.command = XPLMCreateCommand("navP/turnaround_set", "friendly cold & dark");
    if (!ctx->turnaround.cb.command)
    {
        ndt_log("navP [error]: nvp_chandlers_init: command or dataref not found\n");
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->turnaround.cb, chandler_turna, 0, ctx);
    }

    /* Custom commands: speedbrakes/spoilers */
    ctx->spbrk.ext.cb.command = XPLMCreateCommand("navP/spoilers/extend",  "speedbrakes extend one");
    ctx->spbrk.ret.cb.command = XPLMCreateCommand("navP/spoilers/retract", "speedbrakes retract one");
    ctx->spbrk.sext           = XPLMFindCommand  ("sim/flight_controls/speed_brakes_down_one");
    ctx->spbrk.sret           = XPLMFindCommand  ("sim/flight_controls/speed_brakes_up_one");
    ctx->spbrk.srat           = XPLMFindDataRef  ("sim/cockpit2/controls/speedbrake_ratio");
    if (!ctx->spbrk.ext.cb.command || !ctx->spbrk.ret.cb.command ||
        !ctx->spbrk.sext || !ctx->spbrk.sret || !ctx->spbrk.srat)
    {
        ndt_log("navP [error]: nvp_chandlers_init: command or dataref not found\n");
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->spbrk.ext.cb, chandler_sp_ex, 0, ctx);
        REGISTER_CHANDLER(ctx->spbrk.ret.cb, chandler_sp_re, 0, ctx);
    }

    /* Custom commands: trims */
    ctx->trims.pch.up.cb.command = XPLMCreateCommand("navP/trims/pitch_up",      "pitch trim up one");
    ctx->trims.pch.up.cd         = XPLMFindCommand  ("sim/flight_controls/pitch_trim_up");
    ctx->trims.pch.dn.cb.command = XPLMCreateCommand("navP/trims/pitch_down",    "pitch trim down one");
    ctx->trims.pch.dn.cd         = XPLMFindCommand  ("sim/flight_controls/pitch_trim_down");
    ctx->trims.ail.lt.cb.command = XPLMCreateCommand("navP/trims/aileron_left",  "aileron trim left one");
    ctx->trims.ail.lt.cd         = XPLMFindCommand  ("sim/flight_controls/aileron_trim_left");
    ctx->trims.ail.rt.cb.command = XPLMCreateCommand("navP/trims/aileron_right", "aileron trim right one");
    ctx->trims.ail.rt.cd         = XPLMFindCommand  ("sim/flight_controls/aileron_trim_right");
    ctx->trims.rud.lt.cb.command = XPLMCreateCommand("navP/trims/rudder_left",   "rudder trim left one");
    ctx->trims.rud.lt.cd         = XPLMFindCommand  ("sim/flight_controls/rudder_trim_left");
    ctx->trims.rud.rt.cb.command = XPLMCreateCommand("navP/trims/rudder_right",  "rudder trim right one");
    ctx->trims.rud.rt.cd         = XPLMFindCommand  ("sim/flight_controls/rudder_trim_right");
    if (!ctx->trims.pch.up.cb.command || !ctx->trims.pch.up.cd ||
        !ctx->trims.pch.dn.cb.command || !ctx->trims.pch.dn.cd ||
        !ctx->trims.ail.lt.cb.command || !ctx->trims.ail.lt.cd ||
        !ctx->trims.ail.rt.cb.command || !ctx->trims.ail.rt.cd ||
        !ctx->trims.rud.lt.cb.command || !ctx->trims.rud.lt.cd ||
        !ctx->trims.rud.rt.cb.command || !ctx->trims.rud.rt.cd)
    {
        ndt_log("navP [error]: nvp_chandlers_init: command not found or created\n");
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->trims.pch.up.cb, chandler_pt_up, 0, ctx);
        REGISTER_CHANDLER(ctx->trims.pch.dn.cb, chandler_pt_dn, 0, ctx);
        REGISTER_CHANDLER(ctx->trims.ail.lt.cb, chandler_at_lt, 0, ctx);
        REGISTER_CHANDLER(ctx->trims.ail.rt.cb, chandler_at_rt, 0, ctx);
        REGISTER_CHANDLER(ctx->trims.rud.lt.cb, chandler_rt_lt, 0, ctx);
        REGISTER_CHANDLER(ctx->trims.rud.rt.cb, chandler_rt_rt, 0, ctx);
    }

    /* Custom commands: reverser control */
    ctx->throt.rev.tog.cb.command = XPLMCreateCommand("navP/thrust/toggler", "toggle thrust reverse");
    ctx->throt.rev.fwd.cb.command = XPLMCreateCommand("navP/thrust/forward", "stow thrust reversers");
    ctx->throt.rev.bet.cb.command = XPLMCreateCommand("navP/thrust/betarng", "propeller beta range");
    ctx->throt.rev.rev.cb.command = XPLMCreateCommand("navP/thrust/reverse", "deploy thrust reversers");
    ctx->throt.rev.prop_mode      = XPLMFindDataRef  ("sim/cockpit2/engine/actuators/prop_mode");
    ctx->throt.rev.tgr[1]         = XPLMFindCommand  ("sim/engines/thrust_reverse_toggle_1");
    ctx->throt.rev.tgr[2]         = XPLMFindCommand  ("sim/engines/thrust_reverse_toggle_2");
    ctx->throt.rev.tgr[3]         = XPLMFindCommand  ("sim/engines/thrust_reverse_toggle_3");
    ctx->throt.rev.tgr[4]         = XPLMFindCommand  ("sim/engines/thrust_reverse_toggle_4");
    ctx->throt.rev.tgr[5]         = XPLMFindCommand  ("sim/engines/thrust_reverse_toggle_5");
    ctx->throt.rev.tgr[6]         = XPLMFindCommand  ("sim/engines/thrust_reverse_toggle_6");
    ctx->throt.rev.tgr[7]         = XPLMFindCommand  ("sim/engines/thrust_reverse_toggle_7");
    ctx->throt.rev.tgr[8]         = XPLMFindCommand  ("sim/engines/thrust_reverse_toggle_8");
    ctx->throt.rev.tgr[0]         = XPLMFindCommand  ("sim/engines/thrust_reverse_toggle");
    ctx->throt.rev.tgb[1]         = XPLMFindCommand  ("sim/engines/beta_toggle_1");
    ctx->throt.rev.tgb[2]         = XPLMFindCommand  ("sim/engines/beta_toggle_2");
    ctx->throt.rev.tgb[3]         = XPLMFindCommand  ("sim/engines/beta_toggle_3");
    ctx->throt.rev.tgb[4]         = XPLMFindCommand  ("sim/engines/beta_toggle_4");
    ctx->throt.rev.tgb[5]         = XPLMFindCommand  ("sim/engines/beta_toggle_5");
    ctx->throt.rev.tgb[6]         = XPLMFindCommand  ("sim/engines/beta_toggle_6");
    ctx->throt.rev.tgb[7]         = XPLMFindCommand  ("sim/engines/beta_toggle_7");
    ctx->throt.rev.tgb[8]         = XPLMFindCommand  ("sim/engines/beta_toggle_8");
    ctx->throt.rev.tgb[0]         = XPLMFindCommand  ("sim/engines/beta_toggle");
    if (!ctx->throt.rev.tog.cb.command ||
        !ctx->throt.rev.fwd.cb.command ||
        !ctx->throt.rev.rev.cb.command ||
        !ctx->throt.rev.prop_mode      ||
        !ctx->throt.rev.tgb[0]         ||
        !ctx->throt.rev.tgb[1]         ||
        !ctx->throt.rev.tgb[2]         ||
        !ctx->throt.rev.tgb[3]         ||
        !ctx->throt.rev.tgb[4]         ||
        !ctx->throt.rev.tgb[5]         ||
        !ctx->throt.rev.tgb[6]         ||
        !ctx->throt.rev.tgb[7]         ||
        !ctx->throt.rev.tgb[8]         ||
        !ctx->throt.rev.tgr[0]         ||
        !ctx->throt.rev.tgr[1]         ||
        !ctx->throt.rev.tgr[2]         ||
        !ctx->throt.rev.tgr[3]         ||
        !ctx->throt.rev.tgr[4]         ||
        !ctx->throt.rev.tgr[5]         ||
        !ctx->throt.rev.tgr[6]         ||
        !ctx->throt.rev.tgr[7]         ||
        !ctx->throt.rev.tgr[8])
    {
        ndt_log("navP [error]: nvp_chandlers_init: command or dataref not found\n");
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->throt.rev.tog.cb, chandler_r_tog, 0, &ctx->throt);
        REGISTER_CHANDLER(ctx->throt.rev.fwd.cb, chandler_r_fwd, 0, &ctx->throt);
        REGISTER_CHANDLER(ctx->throt.rev.bet.cb, chandler_r_bet, 0, &ctx->throt);
        REGISTER_CHANDLER(ctx->throt.rev.rev.cb, chandler_r_rev, 0, &ctx->throt);
    }

    /* Custom commands and handlers: thrust control */
    ctx->throt.  rpmratio = XPLMFindDataRef  ("sim/cockpit2/engine/actuators/prop_rotation_speed_rad_sec_all");
    ctx->throt.  mixratio = XPLMFindDataRef  ("sim/cockpit2/engine/actuators/mixture_ratio_all");
    ctx->throt.dd.command = XPLMCreateCommand("navP/thrust/dn_cont", "throttle down + once");
    ctx->throt.dn.command = XPLMCreateCommand("navP/thrust/dn_once", "throttle down once");
    ctx->throt.uu.command = XPLMCreateCommand("navP/thrust/up_cont", "throttle up + 12pc");
    ctx->throt.up.command = XPLMCreateCommand("navP/thrust/up_once", "throttle up once");
    ctx->throt.ul.command = XPLMCreateCommand("navP/thrust/up_lots", "throttle up 12pc");
    ctx->throt.pt.command = XPLMCreateCommand("private/ptt/dn/once", "NOT TO BE USED");
    ctx->throt.md.command = XPLMFindCommand  ("sim/engines/mixture_down");
    ctx->throt.mu.command = XPLMFindCommand  ("sim/engines/mixture_up");
    ctx->throt.rp.command = XPLMFindCommand  ("sim/engines/prop_down");
    ctx->throt.     thrdn = XPLMFindCommand  ("sim/engines/throttle_down");
    ctx->throt.     thrup = XPLMFindCommand  ("sim/engines/throttle_up");
    if (!ctx->throt.dn.command || !ctx->throt.thrdn || !ctx->throt.md.command || !ctx->throt.mixratio ||
        !ctx->throt.up.command || !ctx->throt.thrup || !ctx->throt.mu.command || !ctx->throt.rpmratio ||
        !ctx->throt.rp.command || !ctx->throt.ul.command || !ctx->throt.pt.command || !ctx->throt.dd.command || !ctx->throt.uu.command)
    {
        ndt_log("navP [error]: nvp_chandlers_init: command not found or created\n");
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->throt.dn, chandler_thrdn, 0, &ctx->throt);
        REGISTER_CHANDLER(ctx->throt.up, chandler_thrup, 0, &ctx->throt);
        REGISTER_CHANDLER(ctx->throt.ul, chandler_thrul, 0, &ctx->throt);
        REGISTER_CHANDLER(ctx->throt.pt, chandler_thptt, 0, &ctx->throt);
        REGISTER_CHANDLER(ctx->throt.dd, chandler_thrdd, 0, &ctx->throt);
        REGISTER_CHANDLER(ctx->throt.uu, chandler_thruu, 0, &ctx->throt);
        ctx->throt.thptt = NULL; ctx->throt.atc_is_connected = 0;
        ctx->throt.thrdo = ctx->throt.dn.command;
        ctx->throt.thrul = ctx->throt.ul.command;
    }

    /* Custom commands: quick look views */
    ctx->views.cbs[0].command = XPLMFindCommand("sim/view/quick_look_0");
    ctx->views.cbs[1].command = XPLMFindCommand("sim/view/quick_look_1");
    ctx->views.cbs[2].command = XPLMFindCommand("sim/view/quick_look_2");
    ctx->views.cbs[3].command = XPLMFindCommand("sim/view/quick_look_3");
    ctx->views.cbs[4].command = XPLMFindCommand("sim/view/quick_look_4");
    ctx->views.cbs[5].command = XPLMFindCommand("sim/view/quick_look_5");
    ctx->views.cbs[6].command = XPLMFindCommand("sim/view/quick_look_6");
    ctx->views.cbs[7].command = XPLMFindCommand("sim/view/quick_look_7");
    ctx->views.cbs[8].command = XPLMFindCommand("sim/view/quick_look_8");
    ctx->views.cbs[9].command = XPLMFindCommand("sim/view/quick_look_9");
    for (int i = 0; i < 10; i++)
    {
        if (!ctx->views.cbs[i].command)
        {
            ndt_log("navP [error]: nvp_chandlers_init: command not found\n");
            goto fail;
        }
        else
        {
            ctx->views.idx[i] = i;
            REGISTER_CHANDLER(ctx->views.cbs[i], chandler_sview, 0, &ctx->views.idx[i]);
        }
    }
    XPLMDataRef ref_ql_idx = XPLMRegisterDataAccessor("navP/views/quicklook_index",
                                                      xplmType_Int, 1,
                                                      &priv_getdata_i,
                                                      &priv_setdata_i,
                                                      NULL, NULL, NULL, NULL, NULL,
                                                      NULL, NULL, NULL, NULL, NULL,
                                                      var_ql_idx_get(), var_ql_idx_get());
    if (!ref_ql_idx)
    {
        ndt_log("navP [error]: nvp_chandlers_init: dataref not found\n");
        goto fail;
    }
    else
    {
        ref_ql_idx_set(ref_ql_idx);
    }
    ctx->views.prev.cb.command = XPLMCreateCommand("navP/views/quick_look_prev", "previous quick look view preset");
    ctx->views.next.cb.command = XPLMCreateCommand("navP/views/quick_look_next",     "next quick look view preset");
    if (!ctx->views.prev.cb.command || !ctx->views.next.cb.command)
    {
        ndt_log("navP [error]: nvp_chandlers_init: could not create command\n");
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->views.prev.cb, chandler_qlprv, 0, ctx);
        REGISTER_CHANDLER(ctx->views.next.cb, chandler_qlnxt, 0, ctx);
    }

    /* Custom commands: autopilot and autothrottle */
    ctx->asrt.ap_conn.command = XPLMCreateCommand( "private/ff320/ap_conn", "NOT TO BE USED");
    ctx->asrt.ap_disc.command = XPLMCreateCommand( "private/ff320/ap_disc", "NOT TO BE USED");
    ctx->otto.ffst.cb.command = XPLMCreateCommand( "private/ffsts/ap_cmdl", "NOT TO BE USED");
    ctx->otto.clmb.cb.command = XPLMCreateCommand( "navP/special/ap_to_ga", "A/P mode: TOGA"); // note: keep: custom behavior on ground
    ctx->otto.conn.cb.command = XPLMCreateCommand( "navP/switches/ap_conn", "A/P engagement"); // TODO: move to x-nullzones
    ctx->otto.disc.cb.command = XPLMCreateCommand( "navP/switches/ap_disc", "A/P disconnect"); // TODO: move to x-nullzones
    if (!ctx->asrt.ap_conn.command ||
        !ctx->asrt.ap_disc.command ||
        !ctx->otto.ffst.cb.command ||
        !ctx->otto.clmb.cb.command ||
        !ctx->otto.conn.cb.command ||
        !ctx->otto.disc.cb.command)
    {
        ndt_log("navP [error]: nvp_chandlers_init: could not create command or dataref not found\n");
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->asrt.ap_conn, chandler_32apc, 0, &ctx->info->assert);
        REGISTER_CHANDLER(ctx->asrt.ap_disc, chandler_32apd, 0, &ctx->info->assert);
        REGISTER_CHANDLER(ctx->otto.ffst.cb, chandler_ffap1, 0, &ctx->otto.ffst.dr);
        REGISTER_CHANDLER(ctx->otto.clmb.cb, chandler_apclb, 0, &ctx->otto.clmb.rc);
        REGISTER_CHANDLER(ctx->otto.conn.cb, chandler_swtch, 0, &ctx->otto.conn.cc);
        REGISTER_CHANDLER(ctx->otto.disc.cb, chandler_swtch, 0, &ctx->otto.disc.cc);
    }

    /* Default commands' handlers: flaps up or down */
    ctx->callouts.cb_flapu.command = XPLMFindCommand("sim/flight_controls/flaps_up");
    ctx->callouts.cb_flapd.command = XPLMFindCommand("sim/flight_controls/flaps_down");
    ctx->callouts.ref_flap_ratio   = XPLMFindDataRef("sim/cockpit2/controls/flap_ratio");
    if (!ctx->callouts.cb_flapu.command ||
        !ctx->callouts.cb_flapd.command ||
        !ctx->callouts.ref_flap_ratio)
    {
        ndt_log("navP [error]: nvp_chandlers_init: command or dataref not found\n");
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->callouts.cb_flapu, chandler_flchg, 0, ctx);
        REGISTER_CHANDLER(ctx->callouts.cb_flapd, chandler_flchg, 0, ctx);
    }
    XPLMRegisterFlightLoopCallback((ctx->callouts.flc_flaps = &flc_flap_func), 0, NULL);
    XPLMRegisterFlightLoopCallback((ctx->callouts.flc_flapd = &flc_flap_cmmd), 0, ctx->callouts.cb_flapd.command);
    XPLMRegisterFlightLoopCallback((ctx->callouts.flc_flapu = &flc_flap_cmmd), 0, ctx->callouts.cb_flapu.command);

    /* Default commands' handlers: gear up, down, toggle */
    ctx->gear.acf_gear_retract            = XPLMFindDataRef("sim/aircraft/gear/acf_gear_retract"     );
    ctx->gear.gear_handle_down            = XPLMFindDataRef("sim/cockpit2/controls/gear_handle_down" );
    ctx->gear.landing_gear_up.command     = XPLMFindCommand("sim/flight_controls/landing_gear_up"    );
    ctx->gear.landing_gear_down.command   = XPLMFindCommand("sim/flight_controls/landing_gear_down"  );
    ctx->gear.landing_gear_toggle.command = XPLMFindCommand("sim/flight_controls/landing_gear_toggle");
    if (!ctx->gear.landing_gear_toggle.command ||
        !ctx->gear.landing_gear_down.command   ||
        !ctx->gear.landing_gear_up.command     ||
        !ctx->gear.gear_handle_down            ||
        !ctx->gear.acf_gear_retract)
    {
        ndt_log("navP [error]: nvp_chandlers_init: command or dataref not found\n");
        goto fail;
    }
    else
    {
        ctx->gear.callouts.ref = ctx->callouts.ref_gear_lever;
        REGISTER_CHANDLER(ctx->gear.landing_gear_up,     chandler_ghndl, 1 /* before X-Plane */, &ctx->gear);
        REGISTER_CHANDLER(ctx->gear.landing_gear_down,   chandler_ghndl, 1 /* before X-Plane */, &ctx->gear);
        REGISTER_CHANDLER(ctx->gear.landing_gear_toggle, chandler_ghndl, 1 /* before X-Plane */, &ctx->gear);
    }

    /* Default commands' handlers: autopilot disconnect */
    ctx->apd.aft.command = ctx->apd.bef.command = XPLMFindCommand("sim/autopilot/fdir_servos_down_one");
    ctx->apd.fd_ref = XPLMFindDataRef("sim/cockpit2/autopilot/flight_director_mode");
    ctx->apd.at_ref = XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_enabled");
    ctx->apd.at_cmd = XPLMFindCommand("sim/autopilot/autothrottle_on");
    if (!ctx->apd.aft.command ||
        !ctx->apd.bef.command ||
        !ctx->apd.at_cmd      ||
        !ctx->apd.at_ref      ||
        !ctx->apd.fd_ref)
    {
        ndt_log("navP [error]: nvp_chandlers_init: command or dataref not found\n");
        goto fail;
    }

    /* Default commands' handlers: pitch hold up/down */
    ctx->vvi.up.command = XPLMFindCommand("sim/autopilot/nose_up");
    ctx->vvi.dn.command = XPLMFindCommand("sim/autopilot/nose_down");
    ctx->vvi.pu.command = XPLMFindCommand("sim/autopilot/nose_up_pitch_mode");
    ctx->vvi.pd.command = XPLMFindCommand("sim/autopilot/nose_down_pitch_mode");
    ctx->vvi.ot_vs_on   = XPLMFindDataRef("sim/cockpit2/autopilot/vvi_status");
    ctx->vvi.vs_dn      = XPLMFindCommand("sim/autopilot/vertical_speed_down");
    ctx->vvi.vs_up      = XPLMFindCommand("sim/autopilot/vertical_speed_up");
    if (!ctx->vvi.dn.command || !ctx->vvi.up.command ||
        !ctx->vvi.pu.command || !ctx->vvi.pd.command || !ctx->vvi.ot_vs_on ||
        !ctx->vvi.vs_dn      || !ctx->vvi.vs_up)
    {
        ndt_log("navP [error]: nvp_chandlers_init: command or dataref not found\n");
        goto fail;
    }

    /* Custom command: pop-up Control Display Unit */
    ctx->mcdu.cb.command = XPLMCreateCommand("navP/switches/cdu_toggle", "CDU pop-up/down");
    if (!ctx->mcdu.cb.command)
    {
        ndt_log("navP [error]: nvp_chandlers_init: could not create command\n");
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->mcdu.cb, chandler_mcdup, 0, &ctx->mcdu.rc);
    }

    /* Custom ground stabilization system (via flight loop callback) */
    ctx->ground.auto_t_sts         = XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_enabled");
    ctx->ground.auto_p_sts         = XPLMFindDataRef("sim/cockpit2/autopilot/servos_on");
    ctx->ground.elev_m_agl         = XPLMFindDataRef("sim/flightmodel/position/y_agl");
    ctx->ground.time.view_type     = XPLMFindDataRef("sim/graphics/view/view_type");
    ctx->ground.time.sim_pause     = XPLMFindDataRef("sim/time/paused");
    ctx->ground.time.zulu_time_xpl = XPLMFindDataRef("sim/time/zulu_time_sec");
    ctx->ground.time.zulu_time_hrs = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_hours");
    ctx->ground.time.zulu_time_min = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_minutes");
    ctx->ground.time.zulu_time_sec = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_seconds");
    ctx->ground.oatc.vol_com0      = XPLMFindDataRef("sim/operation/sound/radio_volume_ratio");
    ctx->ground.oatc.vol_com1      = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_volume_com1");
    ctx->ground.oatc.vol_com2      = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_volume_com2");
    ctx->ground.thrott_all         = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all");
    ctx->ground.ongrnd_any         = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    if (!ctx->ground.auto_t_sts         ||
        !ctx->ground.auto_p_sts         ||
        !ctx->ground.elev_m_agl         ||
        !ctx->ground.time.view_type     ||
        !ctx->ground.time.sim_pause     ||
        !ctx->ground.time.zulu_time_xpl ||
        !ctx->ground.time.zulu_time_hrs ||
        !ctx->ground.time.zulu_time_min ||
        !ctx->ground.time.zulu_time_sec ||
        !ctx->ground.oatc.vol_com0      ||
        !ctx->ground.oatc.vol_com1      ||
        !ctx->ground.oatc.vol_com2      ||
        !ctx->ground.thrott_all         ||
        !ctx->ground.ongrnd_any)
    {
        ndt_log("navP [error]: nvp_chandlers_init: command or dataref not found\n");
        goto fail;
    }
    else
    {
        ctx->ground.pt = &ctx->throt;
    }

    /* nominal axis sensitivity settings as per user preference files */
    if ((ctx->axes.axis[0] = XPLMFindDataRef("sim/joystick/joystick_heading_sensitivity")))
    {
        ctx->axes.sensitivity[0] = XPLMGetDataf(ctx->axes.axis[0]);
    }
    else
    {
        ctx->axes.sensitivity[0] = 0.5f;
    }
    if ((ctx->axes.axis[1] = XPLMFindDataRef("sim/joystick/joystick_pitch_sensitivity")))
    {
        ctx->axes.sensitivity[1] = XPLMGetDataf(ctx->axes.axis[0]);
    }
    else
    {
        ctx->axes.sensitivity[1] = 0.5f;
    }
    if ((ctx->axes.axis[2] = XPLMFindDataRef("sim/joystick/joystick_roll_sensitivity")))
    {
        ctx->axes.sensitivity[2] = XPLMGetDataf(ctx->axes.axis[0]);
    }
    else
    {
        ctx->axes.sensitivity[2] = 0.5f;
    }

    /* all OK, sanitize values, return */
    nvp_chandlers_reset(ctx); return ctx;

fail:
    ndt_log("navP [error]: nvp_chandlers_init: fail path reached\n");
    nvp_chandlers_close((void**)&ctx);
    return NULL;
}

int nvp_chandlers_close(void **_chandler_context)
{
    chandler_context *ctx = *_chandler_context;
    *_chandler_context = NULL;
    if (!ctx)
    {
        return -1;
    }

    /* unregister all handlers… */
    UNREGSTR_CHANDLER(ctx->turnaround.   cb);
    UNREGSTR_CHANDLER(ctx->spbrk.    ext.cb);
    UNREGSTR_CHANDLER(ctx->spbrk.    ret.cb);
    UNREGSTR_CHANDLER(ctx->trims. pch.up.cb);
    UNREGSTR_CHANDLER(ctx->trims. pch.dn.cb);
    UNREGSTR_CHANDLER(ctx->trims. ail.lt.cb);
    UNREGSTR_CHANDLER(ctx->trims. ail.rt.cb);
    UNREGSTR_CHANDLER(ctx->trims. rud.lt.cb);
    UNREGSTR_CHANDLER(ctx->trims. rud.rt.cb);
    UNREGSTR_CHANDLER(ctx->throt.rev.tog.cb);
    UNREGSTR_CHANDLER(ctx->throt.rev.fwd.cb);
    UNREGSTR_CHANDLER(ctx->throt.rev.bet.cb);
    UNREGSTR_CHANDLER(ctx->throt.rev.rev.cb);
    UNREGSTR_CHANDLER(ctx->throt.        md);
    UNREGSTR_CHANDLER(ctx->throt.        mu);
    UNREGSTR_CHANDLER(ctx->throt.        rp);
    UNREGSTR_CHANDLER(ctx->throt.        dn);
    UNREGSTR_CHANDLER(ctx->throt.        up);
    UNREGSTR_CHANDLER(ctx->throt.        ul);
    UNREGSTR_CHANDLER(ctx->asrt.    ap_conn);
    UNREGSTR_CHANDLER(ctx->asrt.    ap_disc);
    UNREGSTR_CHANDLER(ctx->otto.    ffst.cb);
    UNREGSTR_CHANDLER(ctx->otto.    conn.cb);
    UNREGSTR_CHANDLER(ctx->otto.    disc.cb);
    UNREGSTR_CHANDLER(ctx->views.   prev.cb);
    UNREGSTR_CHANDLER(ctx->views.   next.cb);
    for (int i = 0; i < 10; i++)
    {
        UNREGSTR_CHANDLER(ctx->views.cbs[i]);
    }
    UNREGSTR_CHANDLER(ctx->gear.landing_gear_toggle);
    UNREGSTR_CHANDLER(ctx->gear.  landing_gear_down);
    UNREGSTR_CHANDLER(ctx->gear.    landing_gear_up);
    UNREGSTR_CHANDLER(ctx->callouts.       cb_flapu);
    UNREGSTR_CHANDLER(ctx->callouts.       cb_flapd);
    UNREGSTR_CHANDLER(ctx->apd.                 aft);
    UNREGSTR_CHANDLER(ctx->apd.                 bef);
    UNREGSTR_CHANDLER(ctx->vvi.                  dn);
    UNREGSTR_CHANDLER(ctx->vvi.                  up);
    UNREGSTR_CHANDLER(ctx->vvi.                  pd);
    UNREGSTR_CHANDLER(ctx->vvi.                  pu);
    UNREGSTR_CHANDLER(ctx->mcdu.                 cb);
    UNREGSTR_CHANDLER(ctx->coatc.                cb);
    UNREGSTR_CHANDLER(ctx->throt.                dn);
    UNREGSTR_CHANDLER(ctx->throt.                up);
    UNREGSTR_CHANDLER(ctx->throt.                ul);
    UNREGSTR_CHANDLER(ctx->throt.                pt);
    UNREGSTR_CHANDLER(ctx->throt.                dd);
    UNREGSTR_CHANDLER(ctx->throt.                uu);

    /* …and all datarefs */
    if (ctx->callouts.ref_speedbrake)
    {
        XPLMUnregisterDataAccessor(ctx->callouts.ref_speedbrake);
        ctx->callouts.ref_speedbrake = NULL;
    }
    if (ctx->callouts.ref_flap_lever)
    {
        XPLMUnregisterDataAccessor(ctx->callouts.ref_flap_lever);
        ctx->callouts.ref_flap_lever = NULL;
    }
    if (ctx->callouts.ref_gear_lever)
    {
        XPLMUnregisterDataAccessor(ctx->callouts.ref_gear_lever);
        ctx->callouts.ref_gear_lever = NULL;
    }
    if (ref_ql_idx_get())
    {
        XPLMUnregisterDataAccessor(ref_ql_idx_get());
        ref_ql_idx_set            (NULL);
    }

    /* …and all callbacks */
    if (ctx->callouts.flc_flaps) XPLMUnregisterFlightLoopCallback(ctx->callouts.flc_flaps,   NULL);
    if (ctx->ground.flc_g)       XPLMUnregisterFlightLoopCallback(ctx->ground.flc_g, &ctx->ground);

    /* all good */
    free(ctx);
    return 0;
}

int nvp_chandlers_reset(void *inContext)
{
    chandler_context *ctx = inContext;
    XPLMDataRef d_ref;
    if (!ctx)
    {
        return -1;
    }

#if ((APL) && (CGFLOAT_IS_DOUBLE))
    if (ctx->info->ac_type == ACF_TYP_TBM9_HS && ctx->mcdu.rc.tbm9.flc_t)
    {
        XPLMUnregisterFlightLoopCallback(ctx->mcdu.rc.tbm9.flc_t, &ctx->mcdu.rc);
    }
    ctx->mcdu.rc.tbm9.flc_t = NULL;
#endif

    /* Unregister key sniffer for AirbusFBW */
    if (ctx->a319kc.kc_is_registered)
    {
        if (XPLMUnregisterKeySniffer(&tol_keysniffer, 1/*inBeforeWindows*/, &ctx->a319kc) == 1)
        {
            ctx->a319kc.kc_is_registered = 0;
        }
        else
        {
            ndt_log("navP [warning]: failed to de-register key sniffer for AirbusFBW\n");
        }
    }
    if (ctx->a350kc.kc_is_registered)
    {
        if (XPLMUnregisterKeySniffer(&a35_keysniffer, 1/*inBeforeWindows*/, &ctx->a350kc) == 1)
        {
            ctx->a350kc.kc_is_registered = 0;
        }
        else
        {
            ndt_log("navP [warning]: failed to de-register key sniffer for AirbusFBW\n");
        }
    }

    /* Reset aircraft properties (type, engine count, retractable gear, etc.) */
    ctx->gear.assert = ctx->ground.assert = ctx->throt.rev.assert = NULL;
    ctx->gear.has_retractable_gear = -1;
    acf_type_info_reset();

    /* Don't use 3rd-party commands/datarefs until we know the plane we're in */
    ctx->otto.clmb.rc.      ptrimto = "sim/flight_controls/pitch_trim_takeoff";
    ctx->otto.clmb.rc.      ap_toga = "sim/autopilot/take_off_go_around";
    ctx->otto.clmb.rc.init_cl_pitch = -1.0f;
    ctx->otto.clmb.rc.init_cl_speed = -1.0f;
    ctx->otto.clmb.rc.      f_pitch = NULL;
    ctx->otto.clmb.rc.      vfpitch = NULL;
    ctx->otto.ffst.              dr = NULL;
    ctx->otto.conn.cc.         name = NULL;
    ctx->otto.disc.cc.         name = NULL;
    ctx->throt.               thptt = NULL;
    ctx->throt.rev.          propdn = NULL;
    ctx->throt.rev.          propup = NULL;
    ctx->throt.            tbm9erng = NULL;
    ctx->throt.            throttle = NULL;
    ctx->callouts.   ref_flaps_e55p = NULL;
    ctx->acfspec.t319.        ready = 0;
    ctx->acfspec.i733.        ready = 0;
    ctx->acfspec.x738.        ready = 0;
    ctx->throt.    atc_is_connected = 0;

    /* Reset some datarefs to match X-Plane's defaults at startup */
    _DO(1, XPLMSetDataf, ctx->axes.sensitivity[0], "sim/joystick/joystick_heading_sensitivity");
    _DO(1, XPLMSetDataf, ctx->axes.sensitivity[1], "sim/joystick/joystick_pitch_sensitivity");
    _DO(1, XPLMSetDataf, ctx->axes.sensitivity[2], "sim/joystick/joystick_roll_sensitivity");
    _DO(1, XPLMSetDatai, 1, "sim/cockpit2/radios/actuators/com1_power");
    _DO(1, XPLMSetDatai, 1, "sim/cockpit2/radios/actuators/com2_power");
    _DO(1, XPLMSetDatai, 1, "sim/cockpit2/radios/actuators/nav1_power");
    _DO(1, XPLMSetDatai, 1, "sim/cockpit2/radios/actuators/nav2_power");
    _DO(1, XPLMSetDataf, 0.5f, "sim/joystick/joystick_heading_augment");
    _DO(1, XPLMSetDataf, 0.5f, "sim/joystick/joystick_pitch_augment");
    _DO(1, XPLMSetDataf, 0.5f, "sim/joystick/joystick_roll_augment");
    _DO(1, XPLMSetDatai, 0, "sim/graphics/misc/kill_map_fms_line");

    /* reset fuel tank selector workaround */
    if (ctx->fueltw.flc_fuel)
    {
        XPLMUnregisterFlightLoopCallback(ctx->fueltw.flc_fuel, &ctx->fueltw);
        ctx->fueltw.flc_fuel = NULL;
    }

    /* Reset turnaround-enabled flight loop callback */
    if (ctx->ground.flc_g)
    {
        XPLMUnregisterFlightLoopCallback(ctx->ground.flc_g, &ctx->ground);
        ctx->ground.flc_g = NULL;
    }

    /* Unregister aircraft-specific command handlers */
    UNREGSTR_CHANDLER(ctx->acfspec.t319.mwcb);
    UNREGSTR_CHANDLER(ctx->apd.          aft);
    UNREGSTR_CHANDLER(ctx->apd.          bef);
    UNREGSTR_CHANDLER(ctx->vvi.           dn);
    UNREGSTR_CHANDLER(ctx->vvi.           up);
    UNREGSTR_CHANDLER(ctx->vvi.           pd);
    UNREGSTR_CHANDLER(ctx->vvi.           pu);
    UNREGSTR_CHANDLER(ctx->throt.         md);
    UNREGSTR_CHANDLER(ctx->throt.         mu);
    UNREGSTR_CHANDLER(ctx->throt.         rp);

    /* Re-enable Gizmo64 if present */
    XPLMPluginID g64 = XPLMFindPluginBySignature("gizmo.x-plugins.com");
    if (XPLM_NO_PLUGIN_ID != g64)
    {
        if (XPLMIsPluginEnabled(g64) == 0)
        {
            XPLMEnablePlugin(g64);
        }
    }

    /* all good */
    ndt_log("navP [info]: nvp_chandlers_reset OK\n"); return (ctx->initialized = ctx->onrun_items = 0);
}

void nvp_chandlers_setmnu(void *inContext, void *inMenu)
{
    chandler_context *ctx = inContext;
    if (ctx)
    {
        ctx->menu_context = ctx->ground.nvp_menu = inMenu;
    }
}

void nvp_chandlers_onload(void *inContext)
{
    chandler_context *ctx = inContext;
    XPLMDataRef dref_temporary = NULL;
    char s[261] = ""; XPLMPluginID gi;
    // auto-disable Gizmo64 before other aircrafts get to load their own plugins
    // only disable for FF-A320: allow Gizmo to update our license for IXEG B733
    if ((dref_temporary = XPLMFindDataRef("sim/aircraft/view/acf_descrip")))
    {
        uf_dref_string_read(dref_temporary, s, sizeof(s));
        if (!STRN_CASECMP_AUTO(s, "FlightFactor Airbus"))
        {
            if (XPLM_NO_PLUGIN_ID != (gi = XPLMFindPluginBySignature("gizmo.x-plugins.com")))
            {
                if (XPLM_NO_PLUGIN_ID == XPLMFindPluginBySignature("SilverLiningV2.Clouds") &&
                    XPLM_NO_PLUGIN_ID == XPLMFindPluginBySignature("SilverLiningV3.Clouds") &&
                    XPLM_NO_PLUGIN_ID == XPLMFindPluginBySignature("SilverLiningV4.Clouds"))
                {
                    if (XPLMIsPluginEnabled(gi))
                    {
                        XPLMDisablePlugin(gi);
                    }
                }
            }
            if ((dref_temporary = XPLMFindDataRef("sim/operation/sound/ground_volume_ratio")))
            {
                XPLMSetDataf(dref_temporary, 0.0f); // mute (until A320 gets safely on tarmac)
            }
        }
    }
}

void nvp_chandlers_scload(void *chandler_context_in)
{
    if (chandler_context_in)
    {
        // reset fps computation when new scenery loads
        chandler_context *ctx = chandler_context_in;
        refcon_ground *grndp = &ctx->ground;
        grndp->last_cycle_number = -1;
        return;
    }
    return;
}

static void print_aircft_info(acf_info_context *info)
{
    ndt_log("navP [info]: %s (\"%s\", \"%s\", \"%s\", \"%s\", \"%s\")\n",
            acf_type_get_name(info->ac_type),
            info->icaoid, info->tailnb, info->author, info->descrp, info->afname);
}

static int fuel_tank_select(int acf_num_engines, int acf_num_tanks)
{
    if (acf_num_engines == 1)
    {
        XPLMDataRef d_ref = XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_selector");
        if (d_ref)
        {
            int fuel_tank_selector = XPLMGetDatai(d_ref);
            if (fuel_tank_selector < 1 ||  fuel_tank_selector > 3 || fuel_tank_selector > acf_num_tanks)
            {
                if ((d_ref = XPLMFindDataRef("sim/aircraft/overflow/acf_has_fuel_all")))
                {
                    if (XPLMGetDatai(d_ref))
                    {
                        if (fuel_tank_selector != 4)
                        {
                            if ((d_ref = XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_selector_right")))
                            {
                                XPLMSetDatai(d_ref, 4);
                            }
                            if ((d_ref = XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_selector_left")))
                            {
                                XPLMSetDatai(d_ref, 4);
                            }
                            if ((d_ref = XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_selector")))
                            {
                                XPLMSetDatai(d_ref, 4);
                            }
                            return 4; // dataref set
                        }
                        return 0; // 4 == XPLMGetDatai(XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_selector"))
                    }
                    if ((d_ref = XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_selector_right")))
                    {
                        XPLMSetDatai(d_ref, 1);
                    }
                    if ((d_ref = XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_selector_left")))
                    {
                        XPLMSetDatai(d_ref, 1);
                    }
                    if ((d_ref = XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_selector")))
                    {
                        XPLMSetDatai(d_ref, 1);
                    }
                    if ((d_ref = XPLMFindDataRef("aerobask/panthera/fuel_position")))
                    {
                        XPLMSetDatai(d_ref, 0);
                    }
                    if ((d_ref = XPLMFindDataRef("aerobask/victory/fuel_position")))
                    {
                        XPLMSetDatai(d_ref, 1);
                    }
                    if ((d_ref = XPLMFindDataRef("aerobask/E1000/fuel_position")))
                    {
                        XPLMSetDatai(d_ref, 1);
                    }
                    if ((d_ref = XPLMFindDataRef("aerobask/fuel_selector")))
                    {
                        XPLMSetDatai(d_ref, 1);
                    }
                    if ((d_ref = XPLMFindDataRef("sim/har/fueltank")))
                    {
                        XPLMSetDatai(d_ref, 0);
                    }
                    return 1; // dataref set
                }
                return -1; // NULL == XPLMFindDataRef("sim/aircraft/overflow/acf_has_fuel_all")
            }
            return 0; // (1 || 2 || acf_num_tanks) == XPLMGetDatai(XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_selector"))
        }
        return -1; // NULL == XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_selector")
    }
    return 0; // acf_num_engines != 1 || acf_num_tanks != 2
}

static float fuel_t_w_hdlr(float inElapsedSinceLastCall,
                           float inElapsedTimeSinceLastFlightLoop,
                           int   inCounter,
                           void *inRefcon)
{
    if (inRefcon)
    {
        if (fuel_tank_select(((refcon_fueltw*)inRefcon)->acf_num_engines, ((refcon_fueltw*)inRefcon)->acf_num_tanks) > 0)
        {
            if ((((refcon_fueltw*)inRefcon)->reset_count += 1) >= 2)
            {
                ndt_log("navP [info]: fuel_tank_select() changed selection\n");
                ndt_log("navP [info]: fuel_t_w_hdlr() mission accomplished\n");
                return 0; // no longer needed past this point
            }
            ndt_log("navP [info]: fuel_tank_select() changed selection\n");
            return -1;
        }
        if ((((refcon_fueltw*)inRefcon)->time_elapsed_total += inElapsedSinceLastCall) > 60.0f)
        {
            ndt_log("navP [info]: fuel_t_w_hdlr() timed out\n");
            return 0; // never run for more than a minute
        }
        return -1;
    }
    return 0;
}

void nvp_chandlers_on_run(void *inContext)
{
    chandler_context *ctx = inContext;
    if (ctx && ctx->initialized && ctx->onrun_items == 0)
    {
        XPLMDataRef d_ref = XPLMFindDataRef("sim/operation/prefs/startup_running");
        if (d_ref && XPLMGetDatai(d_ref))
        {
            if (ctx->info->engine_count == 1) // single-engine: select default fuel tank
            {
                ctx->fueltw.reset_count = 0; ctx->fueltw.time_elapsed_total = 0.0f;
                XPLMRegisterFlightLoopCallback(ctx->fueltw.flc_fuel = fuel_t_w_hdlr, -1, &ctx->fueltw);
                if (fuel_tank_select(ctx->fueltw.acf_num_engines = ctx->info->engine_count, ctx->fueltw.acf_num_tanks = ctx->info->ftanks_count) > 0)
                {
                    ndt_log("navP [info]: fuel_tank_select() changed selection\n");
                }
            }
            ctx->onrun_items = 1;
            return;
        }
        return;
    }
    return;
}

#define AFTER_7X_PATH(p) ((((p) + (0.0285f)) / ((0.5011f))))

int nvp_chandlers_update(void *inContext)
{
    chandler_context *ctx = inContext;
    if (!ctx)
    {
        return -1;
    }
    if (ctx->initialized)
    {
        return 0;
    }
    ctx->initialized = 1;
    ctx->first_fcall = 1;
    ctx->kill_daniel = 1;

    /* determine which plane we're flying */
    ctx->info = acf_type_info_update();
    print_aircft_info(ctx->info);
    XPLMDataRef d_ref;

    /* aircraft-specific custom commands and miscellaneous stuff */
    switch ((ctx->throt.info = ctx->info)->ac_type)
    {
        case ACF_TYP_A320_FF:
            ctx->otto.conn.cc.name = "private/ff320/ap_conn";
            ctx->otto.disc.cc.name = "private/ff320/ap_disc";
            ctx->otto.clmb.rc.ap_toga = ctx->otto.clmb.rc.ptrimto = NULL;
            ctx->gear.assert = ctx->ground.assert = ctx->throt.rev.assert = &ctx->info->assert;
            break;

        case ACF_TYP_A319_TL:
        case ACF_TYP_A321_TL:
            ctx->otto.conn.cc.name = "toliss_airbus/ap1_push";
            ctx->otto.disc.cc.name = "toliss_airbus/ap_disc_left_stick";
            ctx->throt.throttle = XPLMFindDataRef("AirbusFBW/throttle_input");
            ctx->otto.clmb.rc.ap_toga = "sim/autopilot/autothrottle_on";
            ctx->otto.clmb.rc.ptrimto = NULL;
            break;

        case ACF_TYP_A350_FF:
            if (NULL != XPLMFindDataRef("sim/version/xplane_internal_version")) // v1.6+ for XP11
            {
                ctx->otto.conn.cc.name = "airbus_qpac/ap1_push";
                ctx->otto.disc.cc.name = "airbus_qpac/ap_disc_left_stick";
            }
            else
            {
                ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
                ctx->otto.conn.cc.name = "airbus_qpac/ap1_push";
            }
            ctx->throt.throttle = XPLMFindDataRef("AirbusFBW/throttle_input");
            ctx->otto.clmb.rc.ap_toga = "sim/autopilot/autothrottle_on";
            ctx->otto.clmb.rc.ptrimto = NULL;
            break;

        case ACF_TYP_B737_EA:
            ctx->otto.conn.cc.name = "x737/mcp/CMDA_TOGGLE";
            ctx->otto.disc.cc.name = "x737/yoke/capt_AP_DISENG_BTN";
            ctx->throt.throttle = ctx->ground.thrott_all;
            ctx->otto.clmb.rc.ap_toga = NULL;
            break;

        case ACF_TYP_B737_XG:
            ctx->otto.disc.cc.name = "ixeg/733/autopilot/AP_disengage";
            ctx->otto.conn.cc.name = "ixeg/733/autopilot/AP_A_cmd_toggle";
            ctx->throt.throttle = ctx->ground.thrott_all;
            ctx->otto.clmb.rc.ap_toga = NULL;
            break;

        case ACF_TYP_B757_FF:
        case ACF_TYP_B767_FF:
            ctx->otto.conn.cc.name = "private/ffsts/ap_cmdl";
            ctx->otto.disc.cc.name = "1-sim/comm/AP/ap_disc";
            ctx->throt.throttle = ctx->ground.thrott_all;
            ctx->otto.clmb.rc.ap_toga = NULL;
            break;

        case ACF_TYP_B777_FF:
            ctx->otto.disc.cc.name = "777/ap_disc";
            ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            ctx->throt.throttle = ctx->ground.thrott_all;
            ctx->otto.clmb.rc.ap_toga = NULL;
            break;

        case ACF_TYP_CL30_DD:
            if (NULL != XPLMFindDataRef("sim/version/xplane_internal_version")) // lazy XP11+ detection
            {
                ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
                ctx->otto.disc.cc.name = "sim/autopilot/servos_yawd_off_any";
            }
            else
            {
                ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
                ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            }
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/autopilot/TOGA_pitch_deg")))
            {
                XPLMSetDataf(d_ref, 10.0f);
            }
            ctx->throt.throttle = ctx->ground.thrott_all;
            break;

        case ACF_TYP_E55P_AB:
            // custom G1000 PFD (always shows speed)
            ctx->otto.clmb.rc.init_cl_speed = 180.0f;
            ctx->otto.clmb.rc.ptrimto = NULL; // automatic
            ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            ctx->otto.disc.cc.name = "sim/autopilot/servos_yawd_off_any";
            ctx->callouts.ref_flaps_e55p = XPLMFindDataRef("aerobask/anim/sw_flap");
            ctx->throt.throttle = ctx->ground.thrott_all;
            break;

        case ACF_TYP_EMBE_SS:
            ctx->otto.conn.cc.name = "SSG/EJET/MCP/AP_COMM";
            ctx->otto.disc.cc.name = "SSG/EJET/MCP/AP_COMM";
            ctx->throt.throttle = ctx->ground.thrott_all;
            ctx->otto.clmb.rc.ap_toga = NULL;
            break;

        case ACF_TYP_EMBE_XC:
            if (NULL != XPLMFindDataRef("sim/version/xplane_internal_version")) // lazy XP11+ detection
            {
                ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
                ctx->otto.disc.cc.name = "sim/autopilot/servos_yawd_off_any";
            }
            else
            {
                ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
                ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            }
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/autopilot/TOGA_pitch_deg")))
            {
                XPLMSetDataf(d_ref, 10.0f);
            }
            ctx->throt.throttle = ctx->ground.thrott_all;
            break;

        case ACF_TYP_HA4T_RW:
            if (NULL != XPLMFindDataRef("sim/version/xplane_internal_version")) // lazy XP11+ detection
            {
                // XP11 version (intitial climb untested)
                ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
                ctx->otto.disc.cc.name = "sim/autopilot/servos_yawd_off_any";
            }
            else // lazy XP10- detection
            {
                if ((d_ref = XPLMFindDataRef("sim/cockpit2/autopilot/TOGA_pitch_deg")))
                {
                    XPLMSetDataf(d_ref, 8.75f); // intital climb @ MTOW slightly weak in testing
                }
                ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
                ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            }
            ctx->throt.throttle = ctx->ground.thrott_all;
            break;

        case ACF_TYP_LEGA_XC:
            // initial climb speed: V2 @ MTOW + ~20
            ctx->otto.clmb.rc.init_cl_speed = 160.0f;
            ctx->otto.clmb.rc.ap_toga = "XCrafts/ERJ/TOGA";
            ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            ctx->otto.disc.cc.name = "sim/autopilot/servos_yawd_off_any";
            ctx->throt.throttle = ctx->ground.thrott_all;
            break;

        case ACF_TYP_MD80_RO:
            ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            ctx->otto.disc.cc.name = "Rotate/md80/autopilot/ap_disc";
            ctx->throt.throttle = ctx->ground.thrott_all;
            ctx->otto.clmb.rc.ap_toga = NULL;
            break;

        case ACF_TYP_TBM9_HS:
            ctx->otto.conn.cc.name = "tbm900/actuators/ap/ap";
            ctx->otto.disc.cc.name = "tbm900/actuators/ap/disc";
            ctx->throt.throttle = ctx->ground.thrott_all;
            ctx->otto.clmb.rc.ptrimto = NULL;
            break;

        case ACF_TYP_GENERIC:
            if (ctx->info->has_rvrs_thr == -1 && ctx->info->has_beta_thr == -1) // XXX: prop-driven w/out reverse
            {
                ctx->throt.rev.propdn = XPLMFindCommand("sim/engines/prop_down");
                ctx->throt.rev.propup = XPLMFindCommand("sim/engines/prop_up");
            }
            if (NULL != XPLMFindDataRef("sim/version/xplane_internal_version")) // lazy XP11+ detection
            {
                ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
                ctx->otto.disc.cc.name = "sim/autopilot/servos_yawd_off_any";
            }
            else
            {
                ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
                ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            }
            if (!STRN_CASECMP_AUTO(ctx->info->icaoid, "FA7X"))
            {
                if ((d_ref = XPLMFindDataRef("sim/weapons/targ_h")))
                {
                    // climb path (not pitch); we default to 7.5 degrees
                    // range of values: min: ~5.0 (MTOW) max: ~15.0 (OEW)
                    ctx->otto.clmb.rc.init_cl_pitch = AFTER_7X_PATH(7.5f);
                    ctx->otto.clmb.rc.init_cl_speed = 165.0f;
                    ctx->otto.clmb.rc.vfpitch_array_i = 0;
                    ctx->otto.clmb.rc.vfpitch = d_ref;
                    ctx->otto.clmb.rc.ap_toga = NULL;
                }
            }
            else if (!STRN_CASECMP_AUTO(ctx->info->icaoid, "EA50"))
            {
                if ((d_ref = XPLMFindDataRef("sim/cockpit2/autopilot/TOGA_pitch_deg")))
                {
                    XPLMSetDataf(d_ref, 10.0f);
                }
                ctx->otto.clmb.rc.init_cl_speed = 160.0f; // SkyView
            }
            else if (!STRN_CASECMP_AUTO(ctx->info->icaoid, "EPIC"))
            {
                if ((d_ref = XPLMFindDataRef("sim/cockpit2/autopilot/TOGA_pitch_deg")))
                {
                    XPLMSetDataf(d_ref, 10.0f);
                }
                if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("1-sim.sasl"))
                {
                    ctx->otto.clmb.rc.init_cl_speed = 160.0f; // SkyView (G1000 version: custom SASL signature)
                } // else G1000 (FLC speed sync, defsult climb speed pointless)
            }
            else if (!STRN_CASECMP_AUTO(ctx->info->icaoid, "EVIC"))
            {
                if ((d_ref = XPLMFindDataRef("sim/cockpit2/autopilot/TOGA_pitch_deg")))
                {
                    XPLMSetDataf(d_ref, 10.0f);
                }
                if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("1-sim.sasl"))
                {
                    ctx->otto.clmb.rc.init_cl_speed = 160.0f; // SkyView (G1000 version: custom SASL signature)
                } // else G1000 (FLC speed sync, defsult climb speed pointless)
            }
            else if (!STRN_CASECMP_AUTO(ctx->info->icaoid, "PC12"))
            {
                if ((d_ref = XPLMFindDataRef("sim/cockpit2/autopilot/TOGA_pitch_deg")))
                {
                    /*
                     * https://www.flyingmag.com/iss-pc12-thrustsense-autothrottle-reviewed/
                     * "As soon as the takeoff/go-around button is pressed on the
                     *  runway, the autothrottle enters go-around mode and places
                     *  the flight director command bars at 7.5 degrees pitch up."
                     */
                    XPLMSetDataf(d_ref, 7.5f);
                }
            }
            else if (!STRN_CASECMP_AUTO(ctx->info->icaoid, "PIPA"))
            {
                ctx->otto.clmb.rc.init_cl_speed = 120.0f; // SkyView
            }
            ctx->throt.throttle = ctx->ground.thrott_all;
            break;

        default: // not generic but no usable commands
            ctx->throt.throttle = ctx->ground.thrott_all;
            break;
    }

#if ((APL) && (CGFLOAT_IS_DOUBLE))
    if (ctx->info->ac_type == ACF_TYP_TBM9_HS)
    {
        XPLMRegisterFlightLoopCallback((ctx->mcdu.rc.tbm9.flc_t = tbm9mousehdlr), 0, &ctx->mcdu.rc);
    }
    else
    {
        ctx->mcdu.rc.tbm9.flc_t = NULL;
    }
#endif

    // new addon type: clear datarefs
    ctx->otto.conn.cc.xpcr = NULL;
    ctx->otto.disc.cc.xpcr = NULL;

    /*
     * Reset MCDU pop-up handler status.
     *
     * Also pre-emptively disable plugin FMSes for all aircraft except
     * x737; if required, they will be re-enabled later automatically.
     *
     * Priority order for x737: x737FMC, then SimpleFMC, then X-FMC.
     */
    XPLMPluginID sfmc = XPLMFindPluginBySignature("pikitanga.xplane10.SimpleFMC");
    XPLMPluginID x737 = XPLMFindPluginBySignature("FJCC.x737FMC");
    XPLMPluginID xfmc = XPLMFindPluginBySignature("x-fmc.com");
    if (x737 != XPLM_NO_PLUGIN_ID)
    {
        if (ctx->info->ac_type == ACF_TYP_B737_EA)
        {
            if (XPLMIsPluginEnabled(x737) == 0)
            {
                XPLMEnablePlugin(x737);
            }
        }
        else
        {
            if (XPLMIsPluginEnabled(x737))
            {
                XPLMDisablePlugin(x737);
            }
        }
    }
    if (sfmc != XPLM_NO_PLUGIN_ID)
    {
        if (ctx->info->ac_type == ACF_TYP_B737_EA &&
            x737      == XPLM_NO_PLUGIN_ID)
        {
            if (XPLMIsPluginEnabled(sfmc) == 0)
            {
                XPLMEnablePlugin(sfmc);
            }
        }
        else
        {
            if (XPLMIsPluginEnabled(sfmc))
            {
                XPLMDisablePlugin(sfmc);
            }
        }
    }
    if (xfmc != XPLM_NO_PLUGIN_ID)
    {
        if (ctx->info->ac_type == ACF_TYP_B737_EA   &&
            sfmc      == XPLM_NO_PLUGIN_ID &&
            x737      == XPLM_NO_PLUGIN_ID)
        {
            if (XPLMIsPluginEnabled(xfmc) == 0)
            {
                XPLMEnablePlugin(xfmc);
            }
        }
        else
        {
            if (XPLMIsPluginEnabled(xfmc))
            {
                XPLMDisablePlugin(xfmc);
            }
        }
    }
    ctx->mcdu.rc.i_cycletyp = 0;
    ctx->mcdu.rc.i_disabled = -1;
    ctx->mcdu.rc.auth = ctx->info->author;
    ctx->mcdu.rc.desc = ctx->info->descrp;
    ctx->mcdu.rc.icao = ctx->info->icaoid;
    ctx->mcdu.rc.atyp = ctx->info->ac_type;

    /* for the gear handle callouts */
    ctx->gear.callouts.atype = ctx->info->ac_type;

    /* check for presence of online ATC plugins */
    ctx->coatc.pe_plid = XPLMFindPluginBySignature("com.pilotedge.plugin.xplane");
    ctx->coatc.xb_plid = XPLMFindPluginBySignature("vatsim.protodev.clients.xsquawkbox");
    if (ctx->coatc.pe_plid || ctx->coatc.xb_plid)
    {
        if (ctx->ground.oatc.flc == NULL)
        {
            ctx->ground.oatc.aircraft_type[0] = ctx->info->ac_type;
            XPLMRegisterFlightLoopCallback((ctx->ground.oatc.flc = &flc_oatc_func), 0, ctx->ground.oatc.aircraft_type);
        }
    }

    /* all good */
    ndt_log("navP [info]: nvp_chandlers_update OK\n"); XPLMSpeakString("nav P configured"); return 0;
}

/*
 * Custom key sniffer for ToLiSS A319 v1.2.x or later
 */
static int tol_keysniffer(char inChar, XPLMKeyFlags inFlags, char inVirtualKey, void *inRefcon)
{
    if (inRefcon == NULL)
    {
        return 1; // pass through
    }
    refcon_a319kbc tkb = *((refcon_a319kbc*)inRefcon);
    unsigned char invk = (unsigned char)inVirtualKey;
    if (invk == XPLM_VK_ESCAPE)
    {
        if ((inFlags & (xplm_OptionAltFlag|xplm_ControlFlag|xplm_ShiftFlag)) == 0)
        {
            if ((inFlags & (xplm_DownFlag)) != 0)
            {
                XPLMCommandOnce(tkb.c[2][0]);
            }
            return 0; // consume
        }
        return 1; // pass through
    }
    int w[3], cdu0 = 0, cdu1 = 0, ext_v = 1;
    XPLMGetDatavi(tkb.datar[0], &w[0], 0, 2);
    if (w[0] > 0 || w[1] > 0)
    {
        int h[2], m[2], x[2], y[2];
        ext_v = XPLMGetDatai(tkb.datar[4]);
        XPLMGetMouseLocation(&m[0], &m[1]);
        XPLMGetDatavi(tkb.datar[0], &w[2], 9, 1);
        XPLMGetDatavi(tkb.datar[1], &h[0], 0, 2);
        XPLMGetDatavi(tkb.datar[2], &x[0], 0, 2);
        XPLMGetDatavi(tkb.datar[3], &y[0], 0, 2);
        if (w[0] > 0) cdu0 = ((m[0] > x[0]) && (m[0] < (x[0] + w[0])) && (m[1] > y[0]) && (m[1] < (y[0] + h[0])));
        if (w[1] > 0) cdu1 = ((m[0] > x[1]) && (m[0] < (x[1] + w[1])) && (m[1] > y[1]) && (m[1] < (y[1] + h[1])));
    }
    if (ext_v || (!cdu0 && !cdu1)) // mouse must be over MCDU with internal view
    {
        return 1; // pass through
    }
    if ((inFlags & (xplm_OptionAltFlag|xplm_ControlFlag)) != 0)
    {
        return 1; // pass through
    }
    if ((inFlags & (xplm_ShiftFlag)) != 0)
    {
        if ((inFlags & (xplm_DownFlag)) == 0)
        {
            return 1; // pass through
        }
        if (inChar != '/')
        {
            return 1; // pass through
        }
        if (cdu0) { XPLMCommandOnce(tkb.c[0][4]); return 0; }
        if (cdu1) { XPLMCommandOnce(tkb.c[1][4]); return 0; }
        return 1; // pass through
    }
    if (w[2] < 1) // forward some keys (ISCS closed only)
    {
        if (inChar == '<') // PTT
        {
            if (tkb.c[2][47] == NULL)
            {
                return 1; // pass through
            }
            if (inFlags & xplm_DownFlag)
            {
                XPLMCommandBegin(tkb.c[2][47]);
                return 0; // consume
            }
            if (inFlags & xplm_UpFlag)
            {
                XPLMCommandEnd(tkb.c[2][47]);
                return 0; // consume
            }
            return 0; // neither up nor down: continue
        }
        if (invk == XPLM_VK_SPACE) // spacebar braking
        {
            if (tkb.c[2][41] == NULL)
            {
                return 1; // pass through
            }
            if (inFlags & xplm_DownFlag)
            {
                XPLMCommandBegin(tkb.c[2][41]);
                return 0; // consume
            }
            if (inFlags & xplm_UpFlag)
            {
                XPLMCommandEnd(tkb.c[2][41]);
                return 0; // consume
            }
            return 0; // neither up nor down: continue
        }
        if ((inFlags & (xplm_DownFlag)) == 0)
        {
            return 1; // pass through
        }
        switch (invk)
        {
            case XPLM_VK_TAB:
                if (tkb.c[2][44]) XPLMCommandOnce(tkb.c[2][44]);
                return 0;
            case XPLM_VK_RETURN:
                if (tkb.c[2][39]) XPLMCommandOnce(tkb.c[2][39]);
                return 0;
            case XPLM_VK_NUMPAD_ENT:
                if (tkb.c[2][40]) XPLMCommandOnce(tkb.c[2][40]);
                return 0;
            case XPLM_VK_NEXT:
                if (tkb.c[2][42]) XPLMCommandOnce(tkb.c[2][42]);
                return 0;
            case XPLM_VK_PRIOR:
                if (tkb.c[2][43]) XPLMCommandOnce(tkb.c[2][43]);
                return 0;
            case XPLM_VK_HOME:
                if (tkb.c[2][45]) XPLMCommandOnce(tkb.c[2][45]);
                return 0;
            case XPLM_VK_END:
                if (tkb.c[2][46]) XPLMCommandOnce(tkb.c[2][46]);
                return 0;
            default:
                break;
        }
    }
    if ((inFlags & (xplm_DownFlag)) == 0)
    {
        return 1; // pass through
    }
    switch (invk)
    {
        case XPLM_VK_UP:
            if (cdu0) { XPLMCommandOnce(tkb.c[0][1]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][1]); return 0; }
            return 1;
        case XPLM_VK_DOWN:
            if (cdu0) { XPLMCommandOnce(tkb.c[0][0]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][0]); return 0; }
            return 1;
        case XPLM_VK_LEFT:
            if (cdu0) { XPLMCommandOnce(tkb.c[0][2]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][2]); return 0; }
            return 1;
        case XPLM_VK_RIGHT:
            if (cdu0) { XPLMCommandOnce(tkb.c[0][3]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][3]); return 0; }
            return 1;
        case XPLM_VK_BACK:
        case XPLM_VK_DELETE:
            if (cdu0) { XPLMCommandOnce(tkb.c[0][7]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][7]); return 0; }
            return 1;
        default:
            break;
    }
    switch (inChar)
    {
        case '/':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][4]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][4]); return 0; }
            return 1;
        case ' ':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][5]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][5]); return 0; }
            return 1;
        case '.':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][8]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][8]); return 0; }
            return 1;
        case '+':
        case '-':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][9]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][9]); return 0; }
            return 1;
        case '1':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][11]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][11]); return 0; }
            return 1;
        case '2':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][12]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][12]); return 0; }
            return 1;
        case '3':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][13]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][13]); return 0; }
            return 1;
        case '4':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][14]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][14]); return 0; }
            return 1;
        case '5':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][15]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][15]); return 0; }
            return 1;
        case '6':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][16]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][16]); return 0; }
            return 1;
        case '7':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][17]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][17]); return 0; }
            return 1;
        case '8':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][18]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][18]); return 0; }
            return 1;
        case '9':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][19]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][19]); return 0; }
            return 1;
        case '0':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][20]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][20]); return 0; }
            return 1;
        case 'A':
        case 'a':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][21]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][21]); return 0; }
            return 1;
        case 'B':
        case 'b':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][22]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][22]); return 0; }
            return 1;
        case 'C':
        case 'c':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][23]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][23]); return 0; }
            return 1;
        case 'D':
        case 'd':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][24]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][24]); return 0; }
            return 1;
        case 'E':
        case 'e':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][25]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][25]); return 0; }
            return 1;
        case 'F':
        case 'f':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][26]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][26]); return 0; }
            return 1;
        case 'G':
        case 'g':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][27]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][27]); return 0; }
            return 1;
        case 'H':
        case 'h':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][28]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][28]); return 0; }
            return 1;
        case 'I':
        case 'i':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][29]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][29]); return 0; }
            return 1;
        case 'J':
        case 'j':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][30]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][30]); return 0; }
            return 1;
        case 'K':
        case 'k':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][31]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][31]); return 0; }
            return 1;
        case 'L':
        case 'l':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][32]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][32]); return 0; }
            return 1;
        case 'M':
        case 'm':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][33]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][33]); return 0; }
            return 1;
        case 'N':
        case 'n':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][34]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][34]); return 0; }
            return 1;
        case 'O':
        case 'o':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][35]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][35]); return 0; }
            return 1;
        case 'P':
        case 'p':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][36]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][36]); return 0; }
            return 1;
        case 'Q':
        case 'q':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][37]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][37]); return 0; }
            return 1;
        case 'R':
        case 'r':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][38]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][38]); return 0; }
            return 1;
        case 'S':
        case 's':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][39]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][39]); return 0; }
            return 1;
        case 'T':
        case 't':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][40]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][40]); return 0; }
            return 1;
        case 'U':
        case 'u':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][41]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][41]); return 0; }
            return 1;
        case 'V':
        case 'v':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][42]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][42]); return 0; }
            return 1;
        case 'W':
        case 'w':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][43]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][43]); return 0; }
            return 1;
        case 'X':
        case 'x':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][44]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][44]); return 0; }
            return 1;
        case 'Y':
        case 'y':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][45]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][45]); return 0; }
            return 1;
        case 'Z':
        case 'z':
            if (cdu0) { XPLMCommandOnce(tkb.c[0][46]); return 0; }
            if (cdu1) { XPLMCommandOnce(tkb.c[1][46]); return 0; }
            return 1;
        default:
            return 1;
    }
}

static int a35_keysniffer(char inChar, XPLMKeyFlags inFlags, char inVirtualKey, void *inRefcon)
{
    if ((inFlags & (xplm_ShiftFlag|xplm_OptionAltFlag|xplm_ControlFlag)) != 0)
    {
        return 1; // pass through
    }
    if (inRefcon == NULL)
    {
        return 1; // pass through
    }
    refcon_a350kbc kbc = *((refcon_a350kbc*)inRefcon);
    unsigned char invk = (unsigned char)inVirtualKey;
    if (XPLMGetDatai(kbc.datar[0])) // external view
    {
        return 1; // pass through
    }
    if (invk == XPLM_VK_SPACE)
    {
        // spacebar braking; can't do anything because we
        // can't check whether we're over the MCDU or not
        return 1; // pass through
    }
    if (invk == XPLM_VK_TAB)
    {
        return 1; // don't mess w/aircraft's KB capture (new MCDU: input fields)
    }
    if (inChar == '<')
    {
        if (kbc.c[2])
        {
            if (inFlags & xplm_DownFlag)
            {
                XPLMCommandBegin(kbc.c[2]); // "sim/operation/contact_atc"
                return 0;
            }
            if (inFlags & xplm_UpFlag)
            {
                XPLMCommandEnd(kbc.c[2]); // "sim/operation/contact_atc"
                return 0;
            }
            return 0;
        }
        return 1; // pass through
    }
    if ((inFlags & (xplm_DownFlag)) == 0)
    {
        return 1; // pass through
    }
    switch (invk)
    {
        case XPLM_VK_NUMPAD_ENT:
            if (kbc.c[0])
            {
                XPLMCommandOnce(kbc.c[0]); // "navP/switches/cdu_toggle"
                return 0;
            }
            return 1;
        case XPLM_VK_RETURN:
            if (kbc.c[1] &&
                kbc.c[3] == NULL) // don't interfere with XSB text input
            {
                XPLMCommandOnce(kbc.c[1]); // "YFMS/toggle"
                return 0;
            }
            return 1;
        case XPLM_VK_NEXT:
            if (kbc.c[4])
            {
                XPLMCommandOnce(kbc.c[4]); // "navP/spoilers/extend"
                return 0;
            }
            return 1;
        case XPLM_VK_PRIOR:
            if (kbc.c[5])
            {
                XPLMCommandOnce(kbc.c[5]); // "navP/spoilers/retract"
                return 0;
            }
            return 1;
        case XPLM_VK_HOME:
            if (kbc.c[6])
            {
                XPLMCommandOnce(kbc.c[6]); // "sim/flight_controls/flaps_up"
                return 0;
            }
            return 1;
        case XPLM_VK_END:
            if (kbc.c[7])
            {
                XPLMCommandOnce(kbc.c[7]); // "sim/flight_controls/flaps_down"
                return 0;
            }
            return 1;
        default:
            break;
    }
    return 1; // pass through
}

/*
 * action: set an aircraft's turnaround state to a pilot-friendly cold & dark variant.
 */
static int chandler_turna(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        chandler_context *ctx = inRefcon;
        XPLMPluginID pid; XPLMCommandRef cmd; XPLMDataRef data;
        /* this can happen after calling XPLMReloadPlugins() */
        if (ctx->initialized == 0)
        {
            nvp_chandlers_reset (inRefcon);
            nvp_chandlers_update(inRefcon);
        }
        /*
         * Do any additional aircraft-specific stuff that can't be done earlier.
         */
        if (ctx->info->ac_type & ACF_TYP_MASK_TOL)
        {
            if (ctx->acfspec.t319.ready == 0)
            {
                tliss_fbw_init(&ctx->acfspec.t319);
            }
        }
        if (ctx->first_fcall)
        {
            // if set to automatic, callouts become enabled on first turnaround
            if (XPLMGetDatai(ctx->callouts.ref_speedbrake) == -1)
            {
                XPLMSetDatai(ctx->callouts.ref_speedbrake,     1);
            }
            if (XPLMGetDatai(ctx->callouts.ref_flap_lever) == -1)
            {
                XPLMSetDatai(ctx->callouts.ref_flap_lever,     1);
            }
            if (XPLMGetDatai(ctx->callouts.ref_gear_lever) == -1)
            {
                XPLMSetDatai(ctx->callouts.ref_gear_lever,     1);
            }
            if (first_fcall_do(ctx) == 0 && ctx->first_fcall == 0)
            {
                if (ctx->fueltw.flc_fuel)
                {
                    XPLMUnregisterFlightLoopCallback(ctx->fueltw.flc_fuel, &ctx->fueltw);
                    ctx->fueltw.flc_fuel = NULL; // no longer needed past this point
                }
                if (XPLMGetDatai(ctx->fov.nonp) == 0)
                {
                    ctx->fov.float_value = XPLMGetDataf(ctx->fov.data);
                    ctx->fov.round_value = roundf(ctx->fov.float_value);
                }
                if ((cmd = XPLMFindCommand("xnz/brakes/park/on/set")))
                {
                    XPLMCommandOnce(cmd);
                }
                XPLMCommandOnce(ctx->views.cbs[1].command);
                XPLMSpeakString("turn around set");
            }
        }
        else
        {
            /* Restore FOV after e.g. IXEG 733 */
            if (XPLMGetDatai(ctx->fov.nonp) == 0)
            {
                if (ctx->fov.round_value != (int)roundf(XPLMGetDataf(ctx->fov.data)))
                {
                    XPLMSetDataf(ctx->fov.data, ctx->fov.float_value);
                    XPLMSpeakString("F O V set");
                }
            }
            /* Stop BetterPushback if there actually is a currently ongoing pushback operation */
            if (XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("skiselkov.BetterPushback")))
            {
                if (XPLMIsPluginEnabled(pid))
                {
                    if ((data = XPLMFindDataRef("bp/connected")))
                    {
                        if (XPLMGetDatai(data))
                        {
                            if ((cmd = XPLMFindCommand("BetterPushback/stop")))
                            {
                                XPLMCommandOnce(cmd);
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}

/*
 * Default speedbrake extend/retract command handlers. For most planes, we just
 * pass the default command through, but a few addons require special handling.
 */
static int chandler_sp_ex(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        acf_type_info_acf_ctx_init();
        refcon_ixeg733   *i33 = NULL;
        refcon_eadt738   *x38 = NULL;
        chandler_context *ctx = inRefcon;
        assert_context   *a32 = ctx->throt.rev.assert; float f_val;
        int speak = XPLMGetDatai(ctx->callouts.ref_speedbrake);
        switch (ctx->info->ac_type)
        {
            case ACF_TYP_A320_FF:
            {
                if (a32 == NULL || !a32->initialized) return 0;
                a32->api.ValueGet(a32->dat.id_f32_p_spoilers_lever, &f_val);
                float before = XPLMGetDataf(ctx->spbrk.srat);
                XPLMCommandOnce(ctx->spbrk.sext);
                if (before < XPLMGetDataf(ctx->spbrk.srat))
                {
                    if (f_val < -.01f)
                    {
                        if (speak > 0) XPLMSpeakString("spoilers disarmed");
                        return 0;
                    }
                    if (f_val < +.01f)
                    {
                        if (speak > 0) XPLMSpeakString("speedbrake");
                        return 0;
                    }
                }
                if (f_val > +.01f)
                {
                    if (speak > 0) XPLMSpeakString("speedbrake");
                    return 0;
                }
                return 0;
            }

            case ACF_TYP_B737_EA:
                x38 = &ctx->acfspec.x738;
                if (x38->ready == 0)
                {
                    boing_738_init(x38);
                }
                break;

            case ACF_TYP_B737_XG:
                i33 = &ctx->acfspec.i733;
                if (i33->ready == 0)
                {
                    boing_733_init(i33);
                }
                break;

            case ACF_TYP_E55P_AB:
                if (ctx->spbrk.e55e == NULL)
                {
                    ctx->spbrk.e55e = XPLMFindCommand("aerobask/speedbrakes_open");
                }
                if (ctx->spbrk.e55e)
                {
                    if (speak > 0) XPLMSpeakString("speedbrake");
                    XPLMCommandOnce(ctx->spbrk.e55e);
                    return 0;
                }
                return 0;

            case ACF_TYP_HA4T_RW:
                if (ctx->spbrk.ha4t == NULL)
                {
                    ctx->spbrk.ha4t = XPLMFindDataRef("Hawker4000/control/speedbrake_b");
                }
                if (ctx->spbrk.ha4t && XPLMGetDatai(ctx->spbrk.ha4t))
                {
                    // spoilers armed, disarm but don't extend
                    if (speak > 0) XPLMSpeakString("spoilers disarmed");
                    XPLMSetDatai(ctx->spbrk.ha4t, 0); return 0;
                }
                else
                {
                    XPLMCommandOnce(ctx->spbrk.sext);
                }
                if (XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    if (speak > 0) XPLMSpeakString("spoilers disarmed");
                    return 0;
                }
                if (XPLMGetDataf(ctx->spbrk.srat) > +.01f)
                {
                    if (speak > 0) XPLMSpeakString("speedbrake");
                    return 0;
                }
                return 0;

            default:
                if (ctx->spbrk.sext)
                {
                    XPLMCommandOnce(ctx->spbrk.sext);
                }
                if (XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    if (ctx->info->ac_type == ACF_TYP_LEGA_XC && ctx->spbrk.sext)
                    {
                        XPLMCommandOnce (ctx->spbrk.sext);
                        if (XPLMGetDataf(ctx->spbrk.srat) > +.01f)
                        {
                            if (speak > 0) XPLMSpeakString("speedbrake");
                            return 0;
                        }
                    }
                    if (speak > 0) XPLMSpeakString("spoilers disarmed");
                    return 0;
                }
                if (XPLMGetDataf(ctx->spbrk.srat) > +.01f)
                {
                    if (speak > 0) XPLMSpeakString("speedbrake");
                    return 0;
                }
                return 0;
        }
        float ratio = XPLMGetDataf(ctx->spbrk.srat);
        if (i33 && i33->ready)
        {
            if (ratio < -.01f)
            {
                XPLMSetDataf(i33->slat, 0.0f);    // armed: retract fully
                if (speak > 0) XPLMSpeakString("spoilers disarmed");
                return 0;
            }
            XPLMSetDataf(i33->slat, 0.8f);        // extend: flight detent
            if (speak > 0) XPLMSpeakString("speedbrake");
            return 0;
        }
        if (x38 && x38->ready)
        {
            if (ratio > .1f && ratio < .2f)
            {
                XPLMCommandOnce(x38->spret);      // extend: disarm spoilers
                if (speak > 0) XPLMSpeakString("spoilers disarmed");
                return 0;
            }
            XPLMCommandOnce(ctx->spbrk.sext);     // extend: one
            if (speak > 0) XPLMSpeakString("speedbrake");
            return 0;
        }
        return 0;
    }
    return 0;
}

static int chandler_sp_re(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        acf_type_info_acf_ctx_init();
        refcon_ixeg733   *i33 = NULL;
        refcon_eadt738   *x38 = NULL;
        chandler_context *ctx = inRefcon;
        assert_context   *a32 = ctx->throt.rev.assert; float f_val;
        int speak = XPLMGetDatai(ctx->callouts.ref_speedbrake);
        switch (ctx->info->ac_type)
        {
            case ACF_TYP_A320_FF:
            {
                if (a32 == NULL || !a32->initialized) return 0;
                a32->api.ValueGet(a32->dat.id_f32_p_spoilers_lever, &f_val);
                float before = XPLMGetDataf(ctx->spbrk.srat);
                XPLMCommandOnce(ctx->spbrk.sret);
                if (before > XPLMGetDataf(ctx->spbrk.srat))
                {
                    if (f_val < +.01f)
                    {
                        if (speak > 0) XPLMSpeakString("spoilers armed");
                        return 0;
                    }
                    if (f_val < +.51f &&
                        XPLMGetDataf(ctx->spbrk.srat) < +.26f)
                    {
                        if (speak > 0) XPLMSpeakString("speedbrake retracted");
                        return 0;
                    }
                    if (speak > 0) XPLMSpeakString("speedbrake");
                    return 0;
                }
                return 0;
            }

            case ACF_TYP_B737_EA:
                x38 = &ctx->acfspec.x738;
                if (x38->ready == 0)
                {
                    boing_738_init(x38);
                }
                break;

            case ACF_TYP_B737_XG:
                i33 = &ctx->acfspec.i733;
                if (i33->ready == 0)
                {
                    boing_733_init(i33);
                }
                break;

            case ACF_TYP_E55P_AB:
                if (ctx->spbrk.e55r == NULL)
                {
                    ctx->spbrk.e55r = XPLMFindCommand("aerobask/speedbrakes_close");
                }
                if (ctx->spbrk.e55r)
                {
                    if (speak > 0) XPLMSpeakString("speedbrake retracted");
                    XPLMCommandOnce(ctx->spbrk.e55r);
                    return 0;
                }
                return 0;

            case ACF_TYP_HA4T_RW:
                if (ctx->spbrk.ha4t == NULL)
                {
                    ctx->spbrk.ha4t = XPLMFindDataRef("Hawker4000/control/speedbrake_b");
                }
                if (ctx->spbrk.ha4t && XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    // already retracted, arm spoilers
                    if (speak > 0) XPLMSpeakString("spoilers armed");
                    XPLMSetDatai(ctx->spbrk.ha4t, 1); return 0;
                }
                else
                {
                    XPLMCommandOnce(ctx->spbrk.sret);
                }
                if (XPLMGetDataf(ctx->spbrk.srat) < -.01f)
                {
                    if (speak > 0) XPLMSpeakString("spoilers armed");
                    return 0;
                }
                if (XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    if (speak > 0) XPLMSpeakString("speedbrake retracted");
                    return 0;
                }
                if (speak > 0) XPLMSpeakString("speedbrake");
                return 0;

            default:
                if (ctx->info->ac_type == ACF_TYP_EMBE_SS &&
                    XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    // already retracted, we can't/needn't arm (automatic)
                    if (speak > 0) XPLMSpeakString("speedbrake retracted");
                    return 0;
                }
                if (ctx->spbrk.sret)
                {
                    XPLMCommandOnce(ctx->spbrk.sret);
                }
                if (XPLMGetDataf(ctx->spbrk.srat) < -.01f)
                {
                    if (speak > 0) XPLMSpeakString("spoilers armed");
                    return 0;
                }
                if (XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    if (ctx->info->ac_type == ACF_TYP_LEGA_XC && ctx->spbrk.sret)
                    {
                        XPLMCommandOnce (ctx->spbrk.sret);
                        if (XPLMGetDataf(ctx->spbrk.srat) < -.01f)
                        {
                            if (speak > 0) XPLMSpeakString("spoilers armed");
                            return 0;
                        }
                    }
                    if (speak > 0) XPLMSpeakString("speedbrake retracted");
                    return 0;
                }
                if (speak > 0) XPLMSpeakString("speedbrake");
                return 0;
        }
        float ratio = XPLMGetDataf(ctx->spbrk.srat);
        if (i33 && i33->ready)
        {
            if (ratio < +.01f)
            {
                if (ratio > -.01f)
                {
                    XPLMSetDataf(i33->slat, .15f);// retract: arm spoilers
                }
                if (speak > 0) XPLMSpeakString("spoilers armed");
                return 0;
            }
            if (ratio > +.51f)
            {
                XPLMSetDataf(i33->slat, 0.8f);    // retract: flight detent
                if (speak > 0) XPLMSpeakString("speedbrake");
                return 0;
            }
            XPLMSetDataf(i33->slat, 0.0f);        // retract: fully
            if (speak > 0) XPLMSpeakString("speedbrake retracted");
            return 0;
        }
        if (x38 && x38->ready)
        {
            if ((ratio < .01f) || (ratio > .1f && ratio < .2f))
            {
                if (ratio < .01f)
                {
                    XPLMCommandOnce(x38->sparm);  // retract: arm spoilers
                }
                if (speak > 0) XPLMSpeakString("spoilers armed");
                return 0;
            }
            XPLMCommandOnce (ctx->spbrk.sret);    // retract: one
            if (XPLMGetDataf(ctx->spbrk.srat) < .01f)
            {
                if (speak > 0) XPLMSpeakString("speedbrake retracted");
                return 0;
            }
            if (speak > 0) XPLMSpeakString("speedbrake");
            return 0;
        }
        return 0;
    }
    return 0;
}

/*
 * Default trim controls' command handlers. For most planes, we just pass
 * the default command through, but a few addons require special handling.
 */
static int chandler_pt_up(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    refcon_eadt738   *x38 = NULL;
    switch (ctx->info->ac_type)
    {
        case ACF_TYP_B737_EA:
            x38 = &ctx->acfspec.x738;
            if (x38->ready == 0)
            {
                boing_738_init(x38);
            }
            break;

        default:
            if (inPhase == xplm_CommandBegin) { XPLMCommandBegin(ctx->trims.pch.up.cd); return 0; }
            if (inPhase == xplm_CommandEnd)   { XPLMCommandEnd  (ctx->trims.pch.up.cd); return 0; }
            return 0;
    }
    if (x38 && x38->ready)
    {
        if (inPhase == xplm_CommandBegin) { XPLMCommandBegin(x38->pt_up); return 0; }
        if (inPhase == xplm_CommandEnd)   { XPLMCommandEnd  (x38->pt_up); return 0; }
        return 0;
    }
    return 0;
}

static int chandler_pt_dn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    refcon_eadt738   *x38 = NULL;
    switch (ctx->info->ac_type)
    {
        case ACF_TYP_B737_EA:
            x38 = &ctx->acfspec.x738;
            if (x38->ready == 0)
            {
                boing_738_init(x38);
            }
            break;

        default:
            if (inPhase == xplm_CommandBegin) { XPLMCommandBegin(ctx->trims.pch.dn.cd); return 0; }
            if (inPhase == xplm_CommandEnd)   { XPLMCommandEnd  (ctx->trims.pch.dn.cd); return 0; }
            return 0;
    }
    if (x38 && x38->ready)
    {
        if (inPhase == xplm_CommandBegin) { XPLMCommandBegin(x38->pt_dn); return 0; }
        if (inPhase == xplm_CommandEnd)   { XPLMCommandEnd  (x38->pt_dn); return 0; }
        return 0;
    }
    return 0;
}

static int chandler_at_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    switch (ctx->info->ac_type)
    {
        default:
            if (inPhase == xplm_CommandBegin) { XPLMCommandBegin(ctx->trims.ail.lt.cd); return 0; }
            if (inPhase == xplm_CommandEnd)   { XPLMCommandEnd  (ctx->trims.ail.lt.cd); return 0; }
            return 0;
    }
    return 0;
}

static int chandler_at_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    switch (ctx->info->ac_type)
    {
        default:
            if (inPhase == xplm_CommandBegin) { XPLMCommandBegin(ctx->trims.ail.rt.cd); return 0; }
            if (inPhase == xplm_CommandEnd)   { XPLMCommandEnd  (ctx->trims.ail.rt.cd); return 0; }
            return 0;
    }
    return 0;
}

static int chandler_rt_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    switch (ctx->info->ac_type)
    {
        default:
            if (inPhase == xplm_CommandBegin) { XPLMCommandBegin(ctx->trims.rud.lt.cd); return 0; }
            if (inPhase == xplm_CommandEnd)   { XPLMCommandEnd  (ctx->trims.rud.rt.cd); return 0; }
            return 0;
    }
    return 0;
}

static int chandler_rt_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    switch (ctx->info->ac_type)
    {
        default:
            if (inPhase == xplm_CommandBegin) { XPLMCommandBegin(ctx->trims.rud.rt.cd); return 0; }
            if (inPhase == xplm_CommandEnd)   { XPLMCommandEnd  (ctx->trims.rud.rt.cd); return 0; }
            return 0;
    }
    return 0;
}

#define T_ZERO (.000001f)

enum
{
    NVP_DIRECTION_DN,
    NVP_DIRECTION_UP,
};

static const float nvp_thrust_presets1_cl30[] =
{
    0.00000f,
    0.03125f,
    0.06250f,
    0.09375f,
    0.12500f,
    0.15625f,
    0.18750f,
    0.21875f,
    0.25000f,
    0.28125f,
    0.31250f,
    0.34375f,
    0.37500f,
    0.40625f,
    0.43750f,
    0.46875f,
    0.50000f,
    0.53125f,
    0.56250f,
    0.59375f,
    0.62500f,
    0.65625f,
    0.68750f,
    0.71875f,
    0.75000f,
    0.78125f,
    0.81250f, // manual thrust
    .833333f, // CRZ
    .866663f, // CLB
    .933333f, // T/O
    1.00000f, // APR
    -1.0000f,
};

static const float nvp_thrust_presets2_cl30[] =
{
    0.00000f,
    0.12500f,
    0.25000f,
    0.37500f,
    0.50000f,
    0.62500f,
    0.75000f, // manual thrust
    .833333f, // CRZ
    .866666f, // CLB
    .933333f, // T/O
    1.00000f, // APR
    -1.0000f,
};

static const float nvp_thrust_presets1_da62[] =
{
    0.00000f,
    0.03125f,
    0.06250f,
    0.09375f,
    0.12500f,
    0.15625f,
    0.18750f,
    0.21875f,
    0.25000f,
    0.28125f,
    0.31250f,
    0.34375f,
    0.37500f,
    0.40625f,
    0.43750f,
    0.46875f,
    0.50000f,
    0.53125f,
    0.56250f,
    0.59375f,
    0.62500f,
    0.65625f,
    0.68750f,
    0.71875f,
    0.75000f,
    0.78125f,
    0.81250f,
    0.84375f,
    0.87500f,
    0.90625f,
    0.93750f, // climb thrust
    1.00000f, // takeoff thrust
    -1.0000f,
};

static const float nvp_thrust_presets1_ea50[] =
{
    0.00000f,
    0.03125f,
    0.06250f,
    0.09375f,
    0.12500f,
    0.15625f,
    0.18750f,
    0.21875f,
    0.25000f,
    0.28125f,
    0.31250f,
    0.34375f,
    0.37500f,
    0.40625f,
    0.43750f,
    0.46875f,
    0.50000f,
    0.53125f,
    0.56250f,
    0.59375f,
    0.62500f,
    0.65625f,
    0.68750f,
    0.71875f,
    0.75000f,
    0.78125f,
    0.81250f,
    0.84375f,
    .872425f, // climb thrust
    .943875f, // ~100% N1
//  1.00000f, // takeoff
    -1.0000f,
};

static const float nvp_thrust_presets2_ea50[] =
{
    0.00000f,
    0.12500f,
    0.25000f,
    0.37500f,
    0.50000f,
    0.62500f,
    0.75000f,
    .872425f, // climb thrust
    .943875f, // ~100% N1
//  1.00000f, // takeoff
    -1.0000f,
};

static const float nvp_thrust_presets1_evic[] =
{
    0.00000f,
    0.03125f,
    0.06250f,
    0.09375f,
    0.12500f,
    0.15625f,
    0.18750f,
    0.21875f,
    0.25000f,
    0.28125f,
    0.31250f,
    0.34375f,
    0.37500f,
    0.40625f,
    0.43750f,
    0.46875f,
    0.50000f,
    0.53125f,
    0.56250f,
    0.59375f,
    0.62500f,
    0.65625f,
    0.68750f,
    0.71875f,
    0.75000f,
    0.78125f,
    0.81250f,
    0.84375f,
    .872725f, // climb thrust
    .971875f, // ~100% N1
//  1.00000f, // takeoff
    -1.0000f,
};

static const float nvp_thrust_presets2_evic[] =
{
    0.00000f,
    0.12500f,
    0.25000f,
    0.37500f,
    0.50000f,
    0.62500f,
    0.75000f,
    .872725f, // climb thrust
    .971875f, // ~100% N1
//  1.00000f, // takeoff
    -1.0000f,
};

/*
 * https://forums.x-plane.org/index.php?/forums/topic/244181-phenom-300-throttle-quadrant-detents/&do=findComment&comment=2178232
 * https://www.omnicalculator.com/math/rounding
 * travel = angle / max_angle = angle / 73
 * 69 to 73°: MAX RSV
 * 59 to 63°: MAX TO/GA
 * 49 to 53°: MAX CON/CLB
 * 38 to 42°: MAX CRZ
 * 00 to 04°: IDLE
 */
static const float nvp_thrust_presets1_e55p[] =
{
    0.00000f,
//  0.03125f, //<04/73
    0.06250f,
    0.09375f,
    0.12500f,
    0.15625f,
    0.18750f,
    0.21875f,
    0.25000f,
    0.28125f,
    0.31250f,
    0.34375f,
    0.37500f,
    0.40625f,
    0.43750f,
    0.46875f,
    0.50000f,
//  0.53125f, //>38/73
    .547945f, // 40/73
    .698630f, // 51/73
    .835616f, // 61/73
    1.00000f,
    -1.0000f,
};

static const float nvp_thrust_presets2_e55p[] =
{
    0.00000f,
    0.12500f,
    0.25000f,
    0.37500f,
    0.50000f,
    .547945f, // 40/73
    .698630f, // 51/73
    .835616f, // 61/73
    1.00000f,
    -1.0000f,
};

static const float nvp_thrust_presets1_pc12[] =
{
    0.00000f,
    0.03125f,
    0.06250f,
    0.09375f,
    0.12500f,
    0.15625f,
    0.18750f,
    0.21875f,
    0.25000f,
    0.28125f,
    0.31250f,
    0.34375f,
    0.37500f,
    0.40625f,
    0.43750f,
    0.46875f,
    0.50000f,
    0.53125f,
    0.56250f,
    0.59375f,
    0.62500f,
    0.65625f,
    0.68750f,
    0.71875f,
    0.75000f,
    0.78125f,
    0.81250f,
    0.84375f,
    0.87500f,
    .984375f, // (1-(1/64))
    -1.0000f,
};

static const float nvp_thrust_presets2_pc12[] =
{
    0.00000f,
    0.12500f,
    0.25000f,
    0.37500f,
    0.50000f,
    0.62500f,
    0.75000f,
    0.87500f,
    .984375f, // (1-(1/64))
    -1.0000f,
};

static const float nvp_thrust_presets1_tbm9[] =
{
    0.00000f, // rev. thrust
    0.05000f, // rev. thrust
    0.10000f, // rev. thrust
    0.15000f, // zero thrust
    0.20000f, // beta thrust
    0.25000f, // beta thrust
    0.30000f, // beta thrust
    0.35000f, // idle-beta/reverse gate
//  0.37500f, // too close to the above
    0.40625f,
    0.43750f,
    0.46875f,
    0.50000f,
    0.53125f,
    0.56250f,
    0.59375f,
    0.62500f,
    0.65625f,
    0.68750f,
    0.71875f,
    0.75000f,
    0.78125f,
    0.81250f,
    0.84375f,
    0.87500f,
    0.90625f,
    0.93750f,
    0.96875f,
    1.00000f,
    -1.0000f,
};

static const float nvp_thrust_presets2_tbm9[] =
{
    0.00000f, // rev. thrust
    0.15000f, // zero thrust
    0.35000f, // idle-beta/reverse gate
    0.50000f,
    0.62500f,
    0.75000f,
    0.87500f,
    1.00000f,
    -1.0000f,
};

#define TBM9_FLIGHT_IDLE_GATE (0.35f)

static const float nvp_thrust_presets1_toli[] =
{
    0.00000f,
    0.03125f,
    0.06250f,
    0.09375f,
    0.12500f,
    0.15625f,
    0.18750f,
    0.21875f,
    0.25000f,
    0.28125f,
    0.31250f,
    0.34375f,
    0.37500f,
    0.40625f,
    0.43750f,
    0.46875f,
    0.50000f,
    0.53125f,
    0.56250f,
    0.59375f,
    0.62500f,
    0.65625f,
    0.69250f, // ~CLB
    0.87000f, // ~FLX
    1.00000f, // TOGA
    -1.0000f,
};

static const float nvp_thrust_presets2_toli[] =
{
    0.00000f,
    0.12500f,
    0.25000f,
    0.37500f,
    0.50000f,
    0.62500f,
    0.69250f, // ~CLB
    0.87000f, // ~FLX
    1.00000f, // TOGA
    -1.0000f,
};

static const float nvp_thrust_presets1[] =
{
    0.00000f,
    0.03125f,
    0.06250f,
    0.09375f,
    0.12500f,
    0.15625f,
    0.18750f,
    0.21875f,
    0.25000f,
    0.28125f,
    0.31250f,
    0.34375f,
    0.37500f,
    0.40625f,
    0.43750f,
    0.46875f,
    0.50000f,
    0.53125f,
    0.56250f,
    0.59375f,
    0.62500f,
    0.65625f,
    0.68750f,
    0.71875f,
    0.75000f,
    0.78125f,
    0.81250f,
    0.84375f,
    0.87500f,
    0.90625f,
    0.93750f,
    0.96875f,
    1.00000f,
    -1.0000f,
};

static const float nvp_thrust_presets2[] =
{
    0.00000f,
    0.12500f,
    0.25000f,
    0.37500f,
    0.50000f,
    0.62500f,
    0.75000f,
    0.87500f,
    1.00000f,
    -1.0000f,
};

static float nvp_thrust_next(float current, const float *presets, int direction)
{
    if (direction == NVP_DIRECTION_DN)
    {
        int i = 1; float next = presets[0];
        while (presets[i] > (0.0f - T_ZERO))
        {
            if ((presets[i] + T_ZERO) < (current - T_ZERO))
            {
                next = presets[i];
                i++; continue;
            }
            return next;
        }
        return next;
    }
    if (direction == NVP_DIRECTION_UP)
    {
        int i = 1; float next = presets[0];
        while (presets[i] > (0.0f - T_ZERO))
        {
            if ((presets[i] - T_ZERO) > (current + T_ZERO))
            {
                return presets[i];
            }
            next = presets[i];
            i++; continue;
        }
        return next;
    }
    return current;
}

static int in_beta_at_index(refcon_thrust *t, int index)
{
    if (t)
    {
        if (t->info->ac_type == ACF_TYP_TBM9_HS)
        {
            if (t->tbm9erng == NULL)
            {
                t->tbm9erng = XPLMFindDataRef("tbm900/systems/engine/range");
            }
            if (index == 1)
            {
                if (t->tbm9erng)
                {
                    switch (XPLMGetDatai(t->tbm9erng))
                    {
                        case 3:
                        case 5:
                            return 0;
                        case 4:
                            return 1;
                        default:
                            break;
                    }
                    return -1;
                }
                return -1;
            }
            return -1;
        }
        if (t->info->has_beta_thr == 1)
        {
            if (t->info->engine_count >= 1)
            {
                if (index >= 1 && index <= 8 && index <= t->info->engine_count)
                {
                    int mode; XPLMGetDatavi(t->rev.prop_mode, &mode, index - 1, 1);
                    return mode == 2;
                }
                return -1;
            }
            return -1;
        }
        return -1;
    }
    return -1;
}

static int in_reverse_at_index(refcon_thrust *t, int index)
{
    if (t)
    {
        assert_context *a32 = t->rev.assert;
        if (a32)
        {
            if (index == 1)
            {
                return XPLMGetDataf(a32->dat.engine_reverse1) > 0.5f;
            }
            if (index == 2)
            {
                return XPLMGetDataf(a32->dat.engine_reverse2) > 0.5f;
            }
            return -1;
        }
        if (t->info->ac_type == ACF_TYP_TBM9_HS)
        {
            if (t->tbm9erng == NULL)
            {
                t->tbm9erng = XPLMFindDataRef("tbm900/systems/engine/range");
            }
            if (index == 1)
            {
                if (t->tbm9erng)
                {
                    switch (XPLMGetDatai(t->tbm9erng))
                    {
                        case 3:
                        case 4:
                            return 0;
                        case 5:
                            return 1;
                        default:
                            break;
                    }
                    return -1;
                }
                return -1;
            }
            return -1;
        }
        if (t->info->has_rvrs_thr == 1)
        {
            if (t->info->engine_count >= 1)
            {
                if (index >= 1 && index <= 8 && index <= t->info->engine_count)
                {
                    int mode; XPLMGetDatavi(t->rev.prop_mode, &mode, index - 1, 1);
                    return mode == 3;
                }
                return -1;
            }
            return -1;
        }
        return -1;
    }
    return -1;
}

static int at_least_one_engine_in_beta(refcon_thrust *t)
{
    if (t)
    {
        if (in_beta_at_index(t, 1) == -1)
        {
            return -1; // beta range not supported
        }
        for (int i = 1; i <= 8; i++)
        {
            if (in_beta_at_index(t, i) == 1)
            {
                return 1;
            }
            if (in_beta_at_index(t, i) == -1)
            {
                return 0; // i > t->info->engine_count
            }
            continue;
        }
        return 0;
    }
    return -1;
}

static int at_least_one_engine_in_reverse(refcon_thrust *t)
{
    if (t)
    {
        if (in_reverse_at_index(t, 1) == -1)
        {
            return -1; // reverse thrust not supported
        }
        for (int i = 1; i <= 8; i++)
        {
            if (in_reverse_at_index(t, i) == 1)
            {
                return 1;
            }
            if (in_reverse_at_index(t, i) == -1)
            {
                return 0; // i > t->info->engine_count
            }
            continue;
        }
        return 0;
    }
    return -1;
}

static int chandler_r_tog(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        int beta = at_least_one_engine_in_beta(inRefcon), reverse = at_least_one_engine_in_reverse(inRefcon);
        int beta_supported = beta >= 0, reverse_supported = reverse >= 0;
        int in_beta = beta >= 1, in_reverse = reverse >= 1;
        if (in_beta)
        {
            if (reverse_supported)
            {
                return chandler_r_rev(((refcon_thrust*)inRefcon)->rev.rev.cb.command, xplm_CommandEnd, inRefcon);
            }
            return chandler_r_fwd(((refcon_thrust*)inRefcon)->rev.fwd.cb.command, xplm_CommandEnd, inRefcon);
        }
        if (in_reverse)
        {
            /*
             * we cannot switch to beta here, else we'd toggle between beta and reverse only instead of cycling…
             */
            return chandler_r_fwd(((refcon_thrust*)inRefcon)->rev.fwd.cb.command, xplm_CommandEnd, inRefcon);
        }
        if (beta_supported) // not in beta, not in reverse, beta supported
        {
            return chandler_r_bet(((refcon_thrust*)inRefcon)->rev.bet.cb.command, xplm_CommandEnd, inRefcon);
        }
        if (reverse_supported) // not in beta, not in reverse, reverse supported
        {
            return chandler_r_rev(((refcon_thrust*)inRefcon)->rev.rev.cb.command, xplm_CommandEnd, inRefcon);
        }
        return 0; // neither beta nor reverse supported
    }
    return 0;
}

static int chandler_r_fwd(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    refcon_thrust *t = inRefcon;
    if (t->rev.propup)
    {
        switch (inPhase)
        {
            case xplm_CommandBegin:
                XPLMCommandBegin(t->rev.propup);
                return 0;
            case xplm_CommandEnd:
                XPLMCommandEnd(t->rev.propup);
                return 0;
            default:
                return 0;
        }
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (t->info->ac_type == ACF_TYP_TBM9_HS)
        {
            if (in_beta_at_index(t, 1) == 1 || in_reverse_at_index(t, 1) == 1)
            {
                /*
                 * cannot go reverse -> beta here, else chandler_r_tog would toggle between beta and reverse only instead of cycling…
                 */
                XPLMSetDataf(t->throttle, T_ZERO + TBM9_FLIGHT_IDLE_GATE); // trigger the gate
                return 0;
            }
            return 0;
        }
        for (int i = 1; i <= 8; i++)
        {
            if (in_beta_at_index(t, i) == 1)
            {
                XPLMCommandOnce(t->rev.tgb[i]); // does it also work if we're feathered?
                continue;
            }
            if (in_reverse_at_index(t, i) == 1)
            {
                XPLMCommandOnce(t->rev.tgr[i]); // does it also work if we're feathered?
                continue;
            }
            continue;
        }
        return 0;
    }
    return 0;
}

static int chandler_r_bet(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    refcon_thrust *t = inRefcon;
    if (t->rev.propdn)
    {
        switch (inPhase)
        {
            case xplm_CommandBegin:
                XPLMCommandBegin(t->rev.propdn);
                return 0;
            case xplm_CommandEnd:
                XPLMCommandEnd(t->rev.propdn);
                return 0;
            default:
                return 0;
        }
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (t->info->ac_type == ACF_TYP_TBM9_HS)
        {
            if (in_beta_at_index(t, 1) != 0)
            {
                return 0;  // already in beta or beta not available (cutoff or feathered propeller)
            }
            if (in_reverse_at_index(t, 1) == 1)
            {
                return 0; // already below beta range
            }
            XPLMSetDataf(t->throttle, nvp_thrust_next(TBM9_FLIGHT_IDLE_GATE, nvp_thrust_presets1_tbm9, NVP_DIRECTION_DN));
            XPLMCommandOnce(t->rev.tgr[0]);
            return 0;
        }
        for (int i = 1; i <= 8; i++)
        {
            if (in_beta_at_index(t, i) == 0) // not in beta and beta supported
            {
                XPLMCommandOnce(t->rev.tgb[i]); // does it also work if we're feathered or in reverse?
                continue;
            }
            continue;
        }
        return 0;
    }
    return 0;
}

static int chandler_r_rev(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    refcon_thrust *t = inRefcon;
    if (t->rev.propdn)
    {
        switch (inPhase)
        {
            case xplm_CommandBegin:
                XPLMCommandBegin(t->rev.propdn);
                return 0;
            case xplm_CommandEnd:
                XPLMCommandEnd(t->rev.propdn);
                return 0;
            default:
                return 0;
        }
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (t->info->ac_type == ACF_TYP_TBM9_HS)
        {
            if (in_reverse_at_index(t, 1) != 0)
            {
                return 0;  // already in reverse or reverse not available (cutoff or feathered propeller)
            }
            if (in_beta_at_index(t, 1) == 0)
            {
                // not in beta range yet, use the beta command to lift the throttle lever past the flight idle gate
                return chandler_r_bet(((refcon_thrust*)inRefcon)->rev.bet.cb.command, xplm_CommandEnd, inRefcon);
            }
            return 0; // already in or below beta range at this point, use thrust up/down commands to control thrust
        }
        for (int i = 1; i <= 8; i++)
        {
            if (in_reverse_at_index(t, i) == 1) // already in reverse
            {
                continue;
            }
            if (in_beta_at_index(t, i) == 0) // not in reverse, not in beta, beta supported
            {
                XPLMCommandOnce(t->rev.tgb[i]); // does it also work if we're feathered?
                continue;
            }
            if (in_reverse_at_index(t, i) == 0) // not in reverse, reverse supported
            {
                XPLMCommandOnce(t->rev.tgr[i]); // does it also work if we're feathered or in beta?
                continue;
            }
            continue;
        }
        return 0;
    }
    return 0;
}

static int chandler_mixdn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd && inRefcon)
    {
        float mix = roundf(30.0f * XPLMGetDataf(inRefcon)) / 30.0f;
        if   (mix > (1.00f/31.0f))
        {
            XPLMSetDataf(inRefcon, mix - (1.00f / 30.0f));
        }
    }
    return 0;
}

static int chandler_mixdt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd && inRefcon)
    {
        float mix = XPLMGetDataf(inRefcon);
        if (mix > 0.5f + T_ZERO)
        {
            XPLMSetDataf(inRefcon, 0.5f);
            return 0;
        }
        if (mix > 0.0f + T_ZERO)
        {
            XPLMSetDataf(inRefcon, 0.0f);
            return 0;
        }
        return 0;
    }
    return 0;
}

static int chandler_mixut(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd && inRefcon)
    {
        float mix = XPLMGetDataf(inRefcon);
        if (mix < 0.5f - T_ZERO)
        {
            XPLMSetDataf(inRefcon, 0.5f);
            return 0;
        }
        if (mix < 1.0f - T_ZERO)
        {
            XPLMSetDataf(inRefcon, 1.0f);
            return 0;
        }
        return 0;
    }
    return 0;
}

static int chandler_rpmdn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd && inRefcon)
    {
        // maximum precision: 60rpm == (2 * Pi) ~= 6.283 < 7.0f
        float rpm = 7.0f * roundf(XPLMGetDataf(inRefcon) / 7.0f);
        if   (rpm > 6.0f)
        {
            XPLMSetDataf(inRefcon, rpm - 7.0f);
        }
    }
    return 0;
}

static int toliss_throttle_set(XPLMDataRef throttle, int acf_type, float next)
{
    switch (acf_type)
    {
        case ACF_TYP_A319_TL:
        case ACF_TYP_A321_TL:
            XPLMSetDatavf(throttle, &next, 4, 1);
            break;

        case ACF_TYP_A350_FF:
        default:
        {
            float l[2]; l[0] = l[1] = next;
            XPLMSetDatavf(throttle, l, 0, 2);
            break;
        }
    }
    return 0;
}

static float custom_detents_toliss(float next, int direction)
{
    /*
     * ToLiSS "detents" vary based on whether thrust is going up or down :-(
     *
     * FLEX: 0.875 (up, A32-)
     * FLEX: 0.870 (dn, A32-)
     * FLEX: 0.870 (up, A35+)
     * FLEX: 0.865 (dn, A35+)
     *
     * CLB: 00.695 (up, both)
     * CLB: 00.690 (dn, both)
     *
     * return lower of the two when going down (works for both)
     * return higher of the two when going up (works for both)
     */
    float hi_lo_detents[3][2] =
    {
        { 1.00f, 1.00f },
        { .875f, .865f },
        { .695f, 0.69f },
    };
    if (next > (hi_lo_detents[1][0] + T_ZERO))
    {
        return hi_lo_detents[0][direction != NVP_DIRECTION_UP];
    }
    if (next > (hi_lo_detents[2][0] + T_ZERO))
    {
        return hi_lo_detents[1][direction != NVP_DIRECTION_UP];
    }
    if (next > (hi_lo_detents[2][1] - T_ZERO))
    {
        return hi_lo_detents[2][direction != NVP_DIRECTION_UP];
    }
    return next;
}

static int nvp_throttle_all(refcon_thrust *t, const float *presets, int direction)
{
    if (t->throttle)
    {
        float current, l[2];
        switch (t->info->ac_type)
        {
            case ACF_TYP_A319_TL:
            case ACF_TYP_A321_TL:
            case ACF_TYP_A350_FF:
                XPLMGetDatavf(t->throttle, l, 0, 2);
                current = ((l[0] + l[1]) / 2.0f);
                break;

            default:
                current = XPLMGetDataf(t->throttle);
                break;
        }
        float next = nvp_thrust_next(current, presets, direction);
        switch (t->info->ac_type)
        {
            case ACF_TYP_A319_TL:
            case ACF_TYP_A321_TL:
            case ACF_TYP_A350_FF:
                return toliss_throttle_set(t->throttle, t->info->ac_type, custom_detents_toliss(next, direction));

            case ACF_TYP_TBM9_HS:
                /*
                 * the code below is disabled but left in for future reference
                 * while it works well with the step commands, its behavior is
                 * not ideal with the continuous thrust commands; moreover, it
                 * doesn't work well to select the ideal thrust/power for taxi
                 */
                /*if (in_beta_at_index(t, 1) == 1 || in_reverse_at_index(t, 1) == 1)
                {
                    if (direction == NVP_DIRECTION_DN)
                    {
                        // in beta/reverse range, invert the direction of the commands
                        // to match behavior when in reverse range with other aircraft
                        // that is: thrust up: more reverse, thrust down: less reverse
                        XPLMSetDataf(t->throttle, nvp_thrust_next(current, presets, NVP_DIRECTION_UP));
                        return 0;
                    }
                    XPLMSetDataf(t->throttle, nvp_thrust_next(current, presets, NVP_DIRECTION_DN));
                    return 0;
                }*/
                if (in_beta_at_index(t, 1) == -1 || in_reverse_at_index(t, 1) == -1)
                {
                    XPLMSetDataf(t->throttle, TBM9_FLIGHT_IDLE_GATE);
                    return 0;  // beta/reverse not available (cutoff or feathered propeller)
                }
                if (in_beta_at_index(t, 1) == 0 && in_reverse_at_index(t, 1) == 0)
                {
                    if (next < TBM9_FLIGHT_IDLE_GATE)
                    {
                        XPLMSetDataf(t->throttle, TBM9_FLIGHT_IDLE_GATE);
                        return 0; // not in beta/reverse, cannot move throttle below gate
                    }
                    break;
                }
                break;

            default:
                break;
        }
        XPLMSetDataf(t->throttle, next);
        return 0;
    }
    return 0;
}

static int chandler_thrdn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd && inRefcon)
    {
        if (((refcon_thrust*)inRefcon)->throttle)
        {
            switch (((refcon_thrust*)inRefcon)->info->ac_type)
            {
                case ACF_TYP_A319_TL:
                case ACF_TYP_A321_TL:
                case ACF_TYP_A350_FF:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_toli, NVP_DIRECTION_DN);
                case ACF_TYP_CL30_DD:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_cl30, NVP_DIRECTION_DN);
                case ACF_TYP_E55P_AB:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_e55p, NVP_DIRECTION_DN);
                case ACF_TYP_TBM9_HS:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_tbm9, NVP_DIRECTION_DN);
                default:
                    break;
            }
            switch (((refcon_thrust*)inRefcon)->info->thrust_presets)
            {
                case NVP_TP_DA62:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_da62, NVP_DIRECTION_DN);
                case NVP_TP_EA50:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_ea50, NVP_DIRECTION_DN);
                case NVP_TP_EVIC:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_evic, NVP_DIRECTION_DN);
                case NVP_TP_PC12:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_pc12, NVP_DIRECTION_DN);
                default:
                    break;
            }
            return nvp_throttle_all(inRefcon, nvp_thrust_presets1, NVP_DIRECTION_DN);
        }
        switch (((refcon_thrust*)inRefcon)->info->ac_type)
        {
            case ACF_TYP_A320_FF:
            default:
                break;
        }
        XPLMCommandOnce(((refcon_thrust*)inRefcon)->thrdn);
        return 0;
    }
    return 0;
}

static int chandler_thrup(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd && inRefcon)
    {
        if (((refcon_thrust*)inRefcon)->throttle)
        {
            switch (((refcon_thrust*)inRefcon)->info->ac_type)
            {
                case ACF_TYP_A319_TL:
                case ACF_TYP_A321_TL:
                case ACF_TYP_A350_FF:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_toli, NVP_DIRECTION_UP);
                case ACF_TYP_CL30_DD:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_cl30, NVP_DIRECTION_UP);
                case ACF_TYP_E55P_AB:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_e55p, NVP_DIRECTION_UP);
                case ACF_TYP_TBM9_HS:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_tbm9, NVP_DIRECTION_UP);
                default:
                    break;
            }
            switch (((refcon_thrust*)inRefcon)->info->thrust_presets)
            {
                case NVP_TP_DA62:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_da62, NVP_DIRECTION_UP);
                case NVP_TP_EA50:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_ea50, NVP_DIRECTION_UP);
                case NVP_TP_EVIC:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_evic, NVP_DIRECTION_UP);
                case NVP_TP_PC12:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets1_pc12, NVP_DIRECTION_UP);
                default:
                    break;
            }
            return nvp_throttle_all(inRefcon, nvp_thrust_presets1, NVP_DIRECTION_UP);
        }
        switch (((refcon_thrust*)inRefcon)->info->ac_type)
        {
            case ACF_TYP_A320_FF:
            default:
                break;
        }
        XPLMCommandOnce(((refcon_thrust*)inRefcon)->thrup);
        return 0;
    }
    return 0;
}

static int chandler_thrul(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd && inRefcon)
    {
        if (((refcon_thrust*)inRefcon)->throttle)
        {
            switch (((refcon_thrust*)inRefcon)->info->ac_type)
            {
                case ACF_TYP_A319_TL:
                case ACF_TYP_A321_TL:
                case ACF_TYP_A350_FF:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets2_toli, NVP_DIRECTION_UP);
                case ACF_TYP_CL30_DD:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets2_cl30, NVP_DIRECTION_UP);
                case ACF_TYP_E55P_AB:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets2_e55p, NVP_DIRECTION_UP);
                case ACF_TYP_TBM9_HS:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets2_tbm9, NVP_DIRECTION_UP);
                default:
                    break;
            }
            switch (((refcon_thrust*)inRefcon)->info->thrust_presets)
            {
                case NVP_TP_EA50:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets2_ea50, NVP_DIRECTION_UP);
                case NVP_TP_EVIC:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets2_evic, NVP_DIRECTION_UP);
                case NVP_TP_PC12:
                    return nvp_throttle_all(inRefcon, nvp_thrust_presets2_pc12, NVP_DIRECTION_UP);
                default:
                    break;
            }
            return nvp_throttle_all(inRefcon, nvp_thrust_presets2, NVP_DIRECTION_UP);
        }
        switch (((refcon_thrust*)inRefcon)->info->ac_type)
        {
            case ACF_TYP_A320_FF:
            default:
                break;
        }
        XPLMCommandOnce(((refcon_thrust*)inRefcon)->thrup);
        XPLMCommandOnce(((refcon_thrust*)inRefcon)->thrup);
        XPLMCommandOnce(((refcon_thrust*)inRefcon)->thrup);
        return 0;
    }
    return 0;
}

static int chandler_thptt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    refcon_thrust *pt = inRefcon;
    if (pt)
    {
        if (inPhase == xplm_CommandBegin)
        {
            if (pt->atc_is_connected && pt->thptt)
            {
                XPLMCommandBegin(pt->thptt);
                return 0;
            }
            return 0;
        }
        if (inPhase == xplm_CommandEnd)
        {
            if (pt->atc_is_connected && pt->thptt)
            {
                XPLMCommandEnd(pt->thptt);
                return 0;
            }
            XPLMCommandOnce(pt->thrdo);
            return 0;
        }
        return 0;
    }
    return 0;
}

static int chandler_thrdd(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    /*
     * future: TBM9 inverted thrust in beta/reverse like in
     * nvp_throttle_all (probably have to disable it anyway)
     */
    if (inPhase == xplm_CommandBegin)
    {
        XPLMCommandBegin(((refcon_thrust*)inRefcon)->thrdn);
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        XPLMCommandEnd (((refcon_thrust*)inRefcon)->thrdn);
        XPLMCommandOnce(((refcon_thrust*)inRefcon)->thrdo);
        return 0;
    }
    return 0;
}

static int chandler_thruu(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    /*
     * future: TBM9 inverted thrust in beta/reverse like in
     * nvp_throttle_all (probably have to disable it anyway)
     */
    if (inPhase == xplm_CommandBegin)
    {
        XPLMCommandBegin(((refcon_thrust*)inRefcon)->thrup);
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        XPLMCommandEnd (((refcon_thrust*)inRefcon)->thrup);
        XPLMCommandOnce(((refcon_thrust*)inRefcon)->thrul);
        return 0;
    }
    return 0;
}

static int chandler_apclb(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        XPLMDataRef d_ref; XPLMCommandRef cr;
        if (((refcon_app*)inRefcon)->ap_toga)
        {
            if ((cr = XPLMFindCommand("sim/autopilot/fdir_on")))
            {
                XPLMCommandOnce(cr);
            }
            if ((cr = XPLMFindCommand(((refcon_app*)inRefcon)->ap_toga)))
            {
                XPLMCommandOnce(cr);
            }
        }
        else if ((d_ref = ((refcon_app*)inRefcon)->f_pitch) || (d_ref = ((refcon_app*)inRefcon)->vfpitch))
        {
            if ((cr = XPLMFindCommand("sim/autopilot/fdir_on")))
            {
                XPLMCommandOnce(cr);
            }
            if ((cr = XPLMFindCommand("sim/autopilot/pitch_sync")))
            {
                XPLMCommandOnce(cr);
            }
            if ((cr = XPLMFindCommand("sim/autopilot/wing_leveler")))
            {
                XPLMCommandOnce(cr);
            }
            if (d_ref == ((refcon_app*)inRefcon)->vfpitch)
            {
                XPLMSetDatavf(d_ref, &(((refcon_app*)inRefcon)->init_cl_pitch), ((refcon_app*)inRefcon)->vfpitch_array_i, 1);
            }
            else
            {
                XPLMSetDataf(d_ref, ((refcon_app*)inRefcon)->init_cl_pitch);
            }
        }
        /*
         * else: do nothing (e.g. ToLiSS-based aircraft)…
         */
        if ((d_ref = XPLMFindDataRef("sim/flightmodel/failures/onground_any")))
        {
            if (0 < XPLMGetDatai(d_ref))
            {
                if (((refcon_app*)inRefcon)->ptrimto)
                {
                    if ((cr = XPLMFindCommand(((refcon_app*)inRefcon)->ptrimto)))
                    {
                        XPLMCommandOnce(cr);
                    }
                }
                if (((refcon_app*)inRefcon)->init_cl_speed > 0.0f)
                {
                    if ((d_ref = XPLMFindDataRef("sim/cockpit2/autopilot/airspeed_is_mach")))
                    {
                        XPLMSetDatai(d_ref, 0);
                    }
                    if ((d_ref = XPLMFindDataRef("sim/cockpit2/autopilot/airspeed_dial_kts_mach")))
                    {
                        XPLMSetDataf(d_ref, ((refcon_app*)inRefcon)->init_cl_speed);
                    }
                    return 0;
                }
                return 0;
            }
            return 0;
        }
        return 0;
    }
    return 0;
}

/*
 * action: always-available custom command which applies the appropriate switch
 *         (be it default or custom) based on the plane/addon we'll be flying.
 *
 * rationale: easier to map a joystick button to a single command for
 *            all planes than to a custom one on a plane-specific basis.
 */
static int chandler_swtch(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        chandler_command *cc = inRefcon;
        if (cc->name)
        {
            if (cc->xpcr == NULL)
            {
                if ((cc->xpcr = XPLMFindCommand(cc->name)) == NULL)
                {
                    ndt_log("navP [error]: command not found: \"%s\"\n", cc->name);
                    XPLMSpeakString("failed to resolve command");
                    cc->name = NULL; return 0;
                }
            }
            XPLMCommandOnce(cc->xpcr);
            return 0;
        }
        return 0;
    }
    return 0;
}

/*
 * Quick look view presets, utilities:
 *
 * - store the latest quick look view index
 * - select previous/next quick look view preset
 */
static int chandler_sview(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        int index = *((int*)inRefcon);
        XPLMSetDatai(ref_ql_idx_get(), index);
    }
    return 1; // passthrough
}

static int chandler_qlprv(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        chandler_context *ctx = inRefcon;
        int index = ndt_mod(ref_ql_idx_val() - 1, 10);
        XPLMCommandOnce(ctx->views.cbs[index].command);
    }
    return 0;
}

static int chandler_qlnxt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        chandler_context *ctx = inRefcon;
        int index = ndt_mod(ref_ql_idx_val() + 1, 10);
        XPLMCommandOnce(ctx->views.cbs[index].command);
    }
    return 0;
}

static int chandler_flchg(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin && ((chandler_context*)inRefcon)->info->ac_type == ACF_TYP_TBM9_HS)
    {
        if (XPLMGetDatai(((chandler_context*)inRefcon)->callouts.ref_flap_lever) <= 0)
        {
            return 1;
        }
        int index = lroundf(2.0f * XPLMGetDataf(((chandler_context*)inRefcon)->callouts.ref_flap_ratio));
        if (inCommand == ((chandler_context*)inRefcon)->callouts.cb_flapd.command)
        {
            if ((index += 1) > 2)
            {
                (index = 2);
            }
        }
        if (inCommand == ((chandler_context*)inRefcon)->callouts.cb_flapu.command)
        {
            if ((index -= 1) < 0)
            {
                (index = 0);
            }
        }
        flap_callout_setst(_flap_names_2POS, lroundf(2.0f * XPLMGetDataf(((chandler_context*)inRefcon)->callouts.ref_flap_ratio)));
        XPLMSetFlightLoopCallbackInterval(((chandler_context*)inRefcon)->callouts.flc_flaps, 1.5f, 1, NULL);
        return 1;
    }
    if (inPhase == xplm_CommandBegin && ((chandler_context*)inRefcon)->info->ac_type == ACF_TYP_E55P_AB)
    {
        if (((chandler_context*)inRefcon)->callouts.ref_flaps_e55p)
        {
            int index = lroundf(4.0f * XPLMGetDataf(((chandler_context*)inRefcon)->callouts.ref_flaps_e55p));
            if (inCommand == ((chandler_context*)inRefcon)->callouts.cb_flapd.command)
            {
                if ((index += 1) > 4)
                {
                    (index = 4);
                }
                if (XPLMGetDatai(((chandler_context*)inRefcon)->ground.ongrnd_any) < 1)
                {
                    switch (index)
                    {
                        case 2:
                            XPLMSetFlightLoopCallbackInterval(((chandler_context*)inRefcon)->callouts.flc_flapd, 0.75f, 1, ((chandler_context*)inRefcon)->callouts.cb_flapd.command);
                            return 1;
                        case 3:
                            XPLMSetFlightLoopCallbackInterval(((chandler_context*)inRefcon)->callouts.flc_flaps, 0.75f, 1, NULL);
                            flap_callout_setst(_flap_names_4POS, index);
                            return 1;
                        default:
                            break;
                    }
                }
            }
            if (inCommand == ((chandler_context*)inRefcon)->callouts.cb_flapu.command)
            {
                if ((index -= 1) < 0)
                {
                    (index = 0);
                }
                if (XPLMGetDatai(((chandler_context*)inRefcon)->ground.ongrnd_any) < 1)
                {
                    switch (index)
                    {
                        case 2:
                            XPLMSetFlightLoopCallbackInterval(((chandler_context*)inRefcon)->callouts.flc_flapu, 0.75f, 1, ((chandler_context*)inRefcon)->callouts.cb_flapu.command);
                            return 1;
                        case 1:
                            XPLMSetFlightLoopCallbackInterval(((chandler_context*)inRefcon)->callouts.flc_flaps, 0.75f, 1, NULL);
                            flap_callout_setst(_flap_names_4POS, index);
                            return 1;
                        default:
                            break;
                    }
                }
            }
            XPLMSetFlightLoopCallbackInterval(((chandler_context*)inRefcon)->callouts.flc_flaps, 1.5f, 1, NULL);
            flap_callout_setst(_flap_names_4POS, index);
            return 1;
        }
        return 1;
    }
    if (inPhase == xplm_CommandEnd)
    {
        chandler_context *ctx = inRefcon;
        if (XPLMGetDatai(ctx->callouts.ref_flap_lever) <= 0)
        {
            return 1;
        }
        switch (ctx->info->ac_type)
        {
            case ACF_TYP_A319_TL:
            case ACF_TYP_A321_TL:
            case ACF_TYP_A350_FF:
            case ACF_TYP_A320_FF:
                flap_callout_setst(_flap_names_4POS, lroundf(4.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case ACF_TYP_B737_EA:
            case ACF_TYP_B737_XG:
                flap_callout_setst(_flap_names_BOE2, lroundf(8.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case ACF_TYP_B757_FF:
            case ACF_TYP_B767_FF:
            case ACF_TYP_B777_FF:
                flap_callout_setst(_flap_names_BOE1, lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case ACF_TYP_CL30_DD:
                flap_callout_setst(_flap_names_1230, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case ACF_TYP_EMBE_SS:
            case ACF_TYP_EMBE_XC:
                flap_callout_setst(_flap_names_EMB2, lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case ACF_TYP_LEGA_XC:
                if (ctx->info->flap_detents == 3) // X-Crafts disables position 18 in their ERJ/Legacy
                {
                    flap_callout_setst(_flap_names_NO18, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                flap_callout_setst(_flap_names_EMB1, lroundf(4.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case ACF_TYP_MD80_RO:
            {
                int index = lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio));
                // there is a delay in the dataref's value (caused by the animation??)
                if (inCommand == ctx->callouts.cb_flapd.command)
                {
                    if (++index == 2) // lever always skips 0.333 (index 2)
                    {
                        index++;
                    }
                    if (index > 6)
                    {
                        index = 6;
                    }
                }
                if (inCommand == ctx->callouts.cb_flapu.command)
                {
                    if (--index == 2) // lever always skips 0.333 (index 2)
                    {
                        index--;
                    }
                    if (index < 0)
                    {
                        index = 0;
                    }
                }
                flap_callout_setst(_flap_names_MD80, index);
                break;
            }
            case ACF_TYP_E55P_AB:
            case ACF_TYP_TBM9_HS:
                return 1; // handled on command begin, see above
            default:
                if (!strcasecmp(ctx->info->icaoid, "A10") ||
                    !strcasecmp(ctx->info->icaoid, "PIPA"))
                {
                    flap_callout_setst(_flap_names_1530, lroundf(2.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "A19N") ||
                    !strcasecmp(ctx->info->icaoid, "A20N") ||
                    !strcasecmp(ctx->info->icaoid, "A21N") ||
                    !strcasecmp(ctx->info->icaoid, "A318") ||
                    !strcasecmp(ctx->info->icaoid, "A319") ||
                    !strcasecmp(ctx->info->icaoid, "A320") ||
                    !strcasecmp(ctx->info->icaoid, "A321") ||
                    !strcasecmp(ctx->info->icaoid, "A330") ||
                    !strcasecmp(ctx->info->icaoid, "A332") ||
                    !strcasecmp(ctx->info->icaoid, "A333") ||
                    !strcasecmp(ctx->info->icaoid, "A337") ||
                    !strcasecmp(ctx->info->icaoid, "A338") ||
                    !strcasecmp(ctx->info->icaoid, "A339") ||
                    !strcasecmp(ctx->info->icaoid, "A340") ||
                    !strcasecmp(ctx->info->icaoid, "A342") ||
                    !strcasecmp(ctx->info->icaoid, "A343") ||
                    !strcasecmp(ctx->info->icaoid, "A345") ||
                    !strcasecmp(ctx->info->icaoid, "A346") ||
                    !strcasecmp(ctx->info->icaoid, "A350") ||
                    !strcasecmp(ctx->info->icaoid, "A359") ||
                    !strcasecmp(ctx->info->icaoid, "A35K") ||
                    !strcasecmp(ctx->info->icaoid, "A380") ||
                    !strcasecmp(ctx->info->icaoid, "A388") ||
                    !strcasecmp(ctx->info->icaoid, "E50P") ||
                    !strcasecmp(ctx->info->icaoid, "E55P"))
                {
                    flap_callout_setst(_flap_names_4POS, lroundf(8.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "AKOY") ||
                    !strcasecmp(ctx->info->icaoid, "BE20") ||
                    !strcasecmp(ctx->info->icaoid, "BE33") ||
                    !strcasecmp(ctx->info->icaoid, "BE35") ||
                    !strcasecmp(ctx->info->icaoid, "BE36") ||
                    !strcasecmp(ctx->info->icaoid, "BE58") ||
                    !strcasecmp(ctx->info->icaoid, "BE9L") ||
                    !strcasecmp(ctx->info->icaoid, "C404") ||
                    !strcasecmp(ctx->info->icaoid, "COL4") ||
                    !strcasecmp(ctx->info->icaoid, "DA40") ||
                    !strcasecmp(ctx->info->icaoid, "DA42") ||
                    !strcasecmp(ctx->info->icaoid, "DA62") ||
                    !strcasecmp(ctx->info->icaoid, "EA50") ||
                    !strcasecmp(ctx->info->icaoid, "EPIC") ||
                    !strcasecmp(ctx->info->icaoid, "EVIC") ||
                    !strcasecmp(ctx->info->icaoid, "LEG2") ||
                    !strcasecmp(ctx->info->icaoid, "P180") ||
                    !strcasecmp(ctx->info->icaoid, "SF50") ||
                    !strcasecmp(ctx->info->icaoid, "TBM9"))
                {
                    flap_callout_setst(_flap_names_2POS, lroundf(2.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "B190"))
                {
                    flap_callout_setst(_flap_names_1735, lroundf(2.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "B37M") ||
                    !strcasecmp(ctx->info->icaoid, "B38M") ||
                    !strcasecmp(ctx->info->icaoid, "B39M") ||
                    !strcasecmp(ctx->info->icaoid, "B3XM") ||
                    !strcasecmp(ctx->info->icaoid, "B732") ||
                    !strcasecmp(ctx->info->icaoid, "B733") ||
                    !strcasecmp(ctx->info->icaoid, "B734") ||
                    !strcasecmp(ctx->info->icaoid, "B735") ||
                    !strcasecmp(ctx->info->icaoid, "B736") ||
                    !strcasecmp(ctx->info->icaoid, "B737") ||
                    !strcasecmp(ctx->info->icaoid, "B738") ||
                    !strcasecmp(ctx->info->icaoid, "B739") ||
                    !strcasecmp(ctx->info->icaoid, "E737") ||
                    !strcasecmp(ctx->info->icaoid, "P8"))
                {
                    flap_callout_setst(_flap_names_BOE2, lroundf(8.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "B52"))
                {
                    flap_callout_setst(_flap_names_B52G, lroundf(5.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "B74D") ||
                    !strcasecmp(ctx->info->icaoid, "B74F") ||
                    !strcasecmp(ctx->info->icaoid, "B74R") ||
                    !strcasecmp(ctx->info->icaoid, "B74S") ||
                    !strcasecmp(ctx->info->icaoid, "B741") ||
                    !strcasecmp(ctx->info->icaoid, "B742") ||
                    !strcasecmp(ctx->info->icaoid, "B743") ||
                    !strcasecmp(ctx->info->icaoid, "B744") ||
                    !strcasecmp(ctx->info->icaoid, "B747") ||
                    !strcasecmp(ctx->info->icaoid, "B748") ||
                    !strcasecmp(ctx->info->icaoid, "B752") ||
                    !strcasecmp(ctx->info->icaoid, "B753") ||
                    !strcasecmp(ctx->info->icaoid, "B757") ||
                    !strcasecmp(ctx->info->icaoid, "B762") ||
                    !strcasecmp(ctx->info->icaoid, "B763") ||
                    !strcasecmp(ctx->info->icaoid, "B764") ||
                    !strcasecmp(ctx->info->icaoid, "B767") ||
                    !strcasecmp(ctx->info->icaoid, "B77F") ||
                    !strcasecmp(ctx->info->icaoid, "B77L") ||
                    !strcasecmp(ctx->info->icaoid, "B77W") ||
                    !strcasecmp(ctx->info->icaoid, "B772") ||
                    !strcasecmp(ctx->info->icaoid, "B773") ||
                    !strcasecmp(ctx->info->icaoid, "B777") ||
                    !strcasecmp(ctx->info->icaoid, "B778") ||
                    !strcasecmp(ctx->info->icaoid, "B779") ||
                    !strcasecmp(ctx->info->icaoid, "B787") ||
                    !strcasecmp(ctx->info->icaoid, "B788") ||
                    !strcasecmp(ctx->info->icaoid, "B789") ||
                    !strcasecmp(ctx->info->icaoid, "B78X") ||
                    !strcasecmp(ctx->info->icaoid, "BLCF") ||
                    !strcasecmp(ctx->info->icaoid, "BSCA"))
                {
                    flap_callout_setst(_flap_names_BOE1, lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "BD5J") ||
                    !strcasecmp(ctx->info->icaoid, "CL30") ||
                    !strcasecmp(ctx->info->icaoid, "CL35"))
                {
                    flap_callout_setst(_flap_names_1230, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "C130"))
                {
                    flap_callout_setst(_flap_names_C130, lroundf(7.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "C150") ||
                    !strcasecmp(ctx->info->icaoid, "C152") ||
                    !strcasecmp(ctx->info->icaoid, "C170") ||
                    !strcasecmp(ctx->info->icaoid, "C172") ||
                    !strcasecmp(ctx->info->icaoid, "C180") ||
                    !strcasecmp(ctx->info->icaoid, "C182") ||
                    !strcasecmp(ctx->info->icaoid, "C185") ||
                    !strcasecmp(ctx->info->icaoid, "C206") ||
                    !strcasecmp(ctx->info->icaoid, "C207") ||
                    !strcasecmp(ctx->info->icaoid, "C208") ||
                    !strcasecmp(ctx->info->icaoid, "C210") ||
                    !strcasecmp(ctx->info->icaoid, "P210") ||
                    !strcasecmp(ctx->info->icaoid, "T210") ||
                    !strcasecmp(ctx->info->icaoid, "PA46"))
                {
                    flap_callout_setst(_flap_names_CSNA, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "C340"))
                {
                    flap_callout_setst(_flap_names_1545, lroundf(2.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "CRUZ"))
                {
                    flap_callout_setst(_flap_names_3POS, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "DC10"))
                {
                    flap_callout_setst(_flap_names_DC10, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "E135") ||
                    !strcasecmp(ctx->info->icaoid, "E145") ||
                    !strcasecmp(ctx->info->icaoid, "E35L") ||
                    !strcasecmp(ctx->info->icaoid, "E45X"))
                {
                    flap_callout_setst(_flap_names_EMB1, lroundf(4.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "E170") ||
                    !strcasecmp(ctx->info->icaoid, "E175") ||
                    !strcasecmp(ctx->info->icaoid, "E190") ||
                    !strcasecmp(ctx->info->icaoid, "E195") ||
                    !strcasecmp(ctx->info->icaoid, "E275") ||
                    !strcasecmp(ctx->info->icaoid, "E290") ||
                    !strcasecmp(ctx->info->icaoid, "E295") ||
                    !strcasecmp(ctx->info->icaoid, "E75L") ||
                    !strcasecmp(ctx->info->icaoid, "E75S"))
                {
                    flap_callout_setst(_flap_names_EMB2, lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "FA7X") ||
                    !strcasecmp(ctx->info->icaoid, "FA8X"))
                {
                    flap_callout_setst(_flap_names_FA78, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "HA4T"))
                {
                    flap_callout_setst(_flap_names_HA4T, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "MD80") ||
                    !strcasecmp(ctx->info->icaoid, "MD82") ||
                    !strcasecmp(ctx->info->icaoid, "MD83") ||
                    !strcasecmp(ctx->info->icaoid, "MD88"))
                {
                    flap_callout_setst(_flap_names_MD80, lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "PA32") ||
                    !strcasecmp(ctx->info->icaoid, "PA34"))
                {
                    flap_callout_setst(_flap_names_PIPR, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "PC12"))
                {
                    flap_callout_setst(_flap_names_PC12, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "TBM8"))
                {
                    flap_callout_setst(_flap_names_TBM8, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                return 1;
        }
        XPLMSetFlightLoopCallbackInterval(ctx->callouts.flc_flaps, 1.5f, 1, NULL);
    }
    return 1;
}

#if ((APL) && (CGFLOAT_IS_DOUBLE))
static float tbm9mousehdlr(float inElapsedSinceLastCall,
                           float inElapsedTimeSinceLastFlightLoop,
                           int   inCounter,
                           void *inRefcon)
{
    refcon_cdu_pop *cdu = inRefcon;
    if (cdu && cdu->tbm9.current_evnt_index >= 0)
    {
        CGEventRef event = cdu->tbm9.evnt[cdu->tbm9.current_evnt_index];
        XPLMCommandRef c = cdu->tbm9.cmmd[cdu->tbm9.current_evnt_index];
        if (event)
        {
            cdu->tbm9.evnt[cdu->tbm9.current_evnt_index] = NULL;
            CGEventPost(kCGHIDEventTap, event);
            CFRelease(event);
            return -1;
        }
        if (c)
        {
            cdu->tbm9.cmmd[cdu->tbm9.current_evnt_index] = NULL;
            cdu->tbm9.current_evnt_index--;
            XPLMCommandOnce(c);
            return -1;
        }
        ndt_log("navP [error]: tbm9mousehdlr: CGEventRef AND XPLMCommandRef are NULL, disabling callback (index %d)\n", cdu->tbm9.current_evnt_index);
        return 0;
    }
    ndt_log("navP [info]: tbm9mousehdlr: end of event queue, disabling callback\n");
    return 0;
}
#endif

static int chandler_mcdup(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        XPLMPluginID plugin;
        refcon_cdu_pop *cdu = inRefcon;
        if (cdu->i_disabled == -1)
        {
            cdu->command[0] = cdu->command[1] = cdu->command[2] = cdu->command[3] = NULL;
            XPLMPluginID sfmc = XPLMFindPluginBySignature("pikitanga.xplane10.SimpleFMC");
            XPLMPluginID x737 = XPLMFindPluginBySignature("FJCC.x737FMC");
            XPLMPluginID xfmc = XPLMFindPluginBySignature("x-fmc.com");
            switch (cdu->atyp)
            {
                case ACF_TYP_B737_XG:
                case ACF_TYP_MD80_RO:
                    cdu->i_disabled = 1; break; // check for YFMS presence

                case ACF_TYP_A320_FF:
                {
                    if (XPLM_NO_PLUGIN_ID != (plugin = XPLMFindPluginBySignature(XPLM_FF_SIGNATURE)))
                    {
                        for (int i = 0; i < XPLMCountHotKeys(); i++)
                        {
                            XPLMPluginID outp_id; char outp_descr[513];
                            XPLMHotKeyID hot_key = XPLMGetNthHotKey(i);
                            if (hot_key == NULL)
                            {
                                continue;
                            }
                            else
                            {
                                XPLMGetHotKeyInfo(hot_key, NULL, NULL, outp_descr, &outp_id);
                            }
                            if (outp_id == plugin)
                            {
                                if (!STRN_CASECMP_AUTO(outp_descr, "Console"))
                                {
                                    XPLMSetHotKeyCombination(hot_key, XPLM_VK_ESCAPE, xplm_UpFlag);
                                    continue;
                                }
                                if (!STRN_CASECMP_AUTO(outp_descr, "mcdu 2"))
                                {
                                    XPLMSetHotKeyCombination(hot_key, XPLM_VK_NUMPAD_ENT, xplm_DownFlag);
                                    continue;
                                }
                                if (!STRN_CASECMP_AUTO(outp_descr, "mcdu 1"))
                                {
                                    XPLMSetHotKeyCombination(hot_key, XPLM_VK_NUMPAD_ENT, xplm_UpFlag);
                                    continue;
                                }
                                // set combination to a key almost guaranteed to be unused
                                XPLMSetHotKeyCombination(hot_key, XPLM_VK_F24, xplm_UpFlag);
                                continue;
                            }
                            continue;
                        }
                    }
                    cdu->i_disabled = 1; return 0; // here first from turnaround
                }

                case ACF_TYP_A319_TL:
                case ACF_TYP_A321_TL:
                    if (NULL == (cdu->command[0] = XPLMFindCommand("AirbusFBW/UndockMCDU1"     )) ||
                        NULL == (cdu->command[1] = XPLMFindCommand("AirbusFBW/UndockMCDU2"     )) ||
                        NULL == (cdu->dataref[0] = XPLMFindDataRef("AirbusFBW/PopUpHeightArray")) ||
                        NULL == (cdu->dataref[1] = XPLMFindDataRef("AirbusFBW/PopUpScale"      )) ||
                        NULL == (cdu->dataref[2] = XPLMFindDataRef("AirbusFBW/PopUpXCoordArray")) ||
                        NULL == (cdu->dataref[3] = XPLMFindDataRef("AirbusFBW/PopUpYCoordArray")))
                    {
                        cdu->i_disabled = 1; break; // check for YFMS presence
                    }
                    cdu->i_disabled = 0; break;

                case ACF_TYP_A350_FF:
                    if (NULL == (cdu->dataref[0] = XPLMFindDataRef("1-sim/misc/popupOis"  )) ||
                        NULL == (cdu->dataref[1] = XPLMFindDataRef("1-sim/misc/popupLeft" )) ||
                        NULL == (cdu->dataref[2] = XPLMFindDataRef("1-sim/misc/popupsHide")) ||
                        NULL == (cdu->command[0] = XPLMFindCommand("AirbusFBW/UndockMCDU1")))
                    {
                        cdu->i_disabled = 1; break; // check for YFMS presence
                    }
                    cdu->i_disabled = 0; break;

                case ACF_TYP_B757_FF:
                case ACF_TYP_B767_FF:
                    if (NULL == (cdu->dataref[0] = XPLMFindDataRef("757Avionics/cdu/popup" )) ||
                        NULL == (cdu->dataref[1] = XPLMFindDataRef("757Avionics/cdu2/popup")))
                    {
                        cdu->i_disabled = 1; break; // check for YFMS presence
                    }
                    cdu->i_disabled = 0; break;

                case ACF_TYP_B777_FF:
                    if (NULL == (cdu->dataref[0] = XPLMFindDataRef("T7Avionics/cdu/popup")))
                    {
                        cdu->i_disabled = 1; break; // check for YFMS presence
                    }
                    cdu->i_disabled = 0; break;

                case ACF_TYP_CL30_DD:
                    if (NULL == (cdu->command[0] = XPLMFindCommand("sim/operation/slider_12")))
                    {
                        cdu->i_disabled = 1; break; // check for YFMS presence
                    }
                    cdu->i_disabled = 0; break;

                case ACF_TYP_E55P_AB:
                    if ((cdu->command[0] = XPLMFindCommand("aerobask/gfc700_popup_show")) &&
                        (cdu->command[1] = XPLMFindCommand("aerobask/gfc700_popup_hide")) &&
                        (cdu->command[2] = XPLMFindCommand("aerobask/gcu477_popup_show")) &&
                        (cdu->command[3] = XPLMFindCommand("aerobask/gcu477_popup_hide")) &&
                        (cdu->command[4] = XPLMFindCommand("sim/GPS/g1000n1_popup")) &&
                        (cdu->command[5] = XPLMFindCommand("sim/GPS/g1000n3_popup")))
                    {
                        cdu->i_cycle_id = 0; // GCU477 + GFC700 + G1000 (x3)
                        cdu->i_disabled = 0; break; // Aerobask G1000 (E55P)
                    }
                    cdu->i_disabled = 1; break; // check for YFMS presence

                case ACF_TYP_EMBE_SS:
                {
                    if (XPLM_NO_PLUGIN_ID != (plugin = XPLMFindPluginBySignature("FJCC.SSGERJ")))
                    {
                        for (int i = 0; i < XPLMCountHotKeys(); i++)
                        {
                            XPLMPluginID outp_id; char outp_descr[513];
                            XPLMHotKeyID hot_key = XPLMGetNthHotKey(i);
                            if (hot_key == NULL)
                            {
                                continue;
                            }
                            else
                            {
                                XPLMGetHotKeyInfo(hot_key, NULL, NULL, outp_descr, &outp_id);
                            }
                            if (outp_id == plugin)
                            {
                                if (!STRN_CASECMP_AUTO(outp_descr, "F8"))
                                {
                                    XPLMSetHotKeyCombination(hot_key, XPLM_VK_NUMPAD_ENT, xplm_UpFlag);
                                    break;
                                }
                                continue;
                            }
                            continue;
                        }
                    }
                    cdu->i_disabled = 1; return 0; // here first from turnaround
                }

                case ACF_TYP_EMBE_XC:
                {
                    if (XPLM_NO_PLUGIN_ID != (plugin = XPLMFindPluginBySignature("ERJ_Functions")))
                    {
                        for (int i = 0; i < XPLMCountHotKeys(); i++)
                        {
                            XPLMPluginID outp_id; char outp_descr[513];
                            XPLMHotKeyID hot_key = XPLMGetNthHotKey(i);
                            if (hot_key == NULL)
                            {
                                continue;
                            }
                            else
                            {
                                XPLMGetHotKeyInfo(hot_key, NULL, NULL, outp_descr, &outp_id);
                            }
                            if (outp_id == plugin)
                            {
                                // set combination to a key almost guaranteed to be unused
                                XPLMSetHotKeyCombination(hot_key, XPLM_VK_F24, xplm_UpFlag);
                                continue;
                            }
                            continue;
                        }
                    }
                    if (NULL == (cdu->dataref[0] = XPLMFindDataRef("sim/cockpit2/switches/generic_lights_switch")) ||
                        NULL == (cdu->dataref[1] = XPLMFindDataRef("sim/cockpit2/switches/custom_slider_on")))
                    {
                        cdu->i_disabled = 1; break; // check for YFMS presence
                    }
                    cdu->i_disabled = 0; return 0; // here first from turnaround
                }

                case ACF_TYP_HA4T_RW:
                {
                    if (XPLM_NO_PLUGIN_ID != (plugin = XPLMFindPluginBySignature("Tekton_Functions")))
                    {
                        for (int i = 0; i < XPLMCountHotKeys(); i++)
                        {
                            XPLMPluginID outp_id; char outp_descr[513];
                            XPLMHotKeyID hot_key = XPLMGetNthHotKey(i);
                            if (hot_key == NULL)
                            {
                                continue;
                            }
                            else
                            {
                                XPLMGetHotKeyInfo(hot_key, NULL, NULL, outp_descr, &outp_id);
                            }
                            if (outp_id == plugin)
                            {
                                // set combination to a key almost guaranteed to be unused
                                XPLMSetHotKeyCombination(hot_key, XPLM_VK_F24, xplm_UpFlag);
                                continue;
                            }
                            continue;
                        }
                    }
                    if (NULL == (cdu->command[0] = XPLMFindCommand("xap/panels/0")) ||
                        NULL == (cdu->command[1] = XPLMFindCommand("xap/panels/5")) ||
                        NULL == (cdu->command[2] = XPLMFindCommand("xap/panels/6")))
                    {
                        cdu->i_disabled = 1; break; // check for YFMS presence
                    }
                    cdu->i_disabled = 0; return 0; // here first from turnaround
                }

                case ACF_TYP_LEGA_XC:
                {
                    if (XPLM_NO_PLUGIN_ID != (plugin = XPLMFindPluginBySignature("ERJ_Functions")))
                    {
                        for (int i = 0; i < XPLMCountHotKeys(); i++)
                        {
                            XPLMPluginID outp_id; char outp_descr[513];
                            XPLMHotKeyID hot_key = XPLMGetNthHotKey(i);
                            if (hot_key == NULL)
                            {
                                continue;
                            }
                            else
                            {
                                XPLMGetHotKeyInfo(hot_key, NULL, NULL, outp_descr, &outp_id);
                            }
                            if (outp_id == plugin)
                            {
                                // set combination to a key almost guaranteed to be unused
                                XPLMSetHotKeyCombination(hot_key, XPLM_VK_F24, xplm_UpFlag);
                                continue;
                            }
                            continue;
                        }
                    }
                    /*
                     * future: auto-set AviTab location preferences???
                     *
                     * if (2 == XPLMGetDatai(XPLMGetDataref("XCrafts/ERJ/avitab_location"))
                     * {
                     *     XPLMSetDataf(XPLMGetDataRef("XCrafts/ERJ/avitab_x_pos"), 0.5175f);
                     *     XPLMSetDataf(XPLMGetDataRef("XCrafts/ERJ/avitab_x_axis"),   0.0f);
                     *     XPLMSetDataf(XPLMGetDataRef("XCrafts/ERJ/avitab_y_pos"), 0.4575f);
                     *     XPLMSetDataf(XPLMGetDataRef("XCrafts/ERJ/avitab_y_axis"),   0.0f);
                     *     XPLMSetDataf(XPLMGetDataRef("XCrafts/ERJ/avitab_z_pos"),    0.0f);
                     *     XPLMSetDataf(XPLMGetDataRef("XCrafts/ERJ/avitab_z_axis"),   0.0f);
                     * }
                     */
                    if (NULL == (cdu->dataref[0] = XPLMFindDataRef("sim/cockpit2/switches/generic_lights_switch")) ||
                        NULL == (cdu->dataref[1] = XPLMFindDataRef("XCrafts/menu/FMS_popup")) ||
                        NULL == (cdu->command[0] = XPLMFindCommand("sim/FMS/CDU_popup")))
                    {
                        cdu->i_disabled = 1; break; // check for YFMS presence
                    }
                    cdu->i_disabled = 0; return 0; // here first from turnaround
                }

                case ACF_TYP_TBM9_HS:
#if ((APL) && (CGFLOAT_IS_DOUBLE))
                    if (NULL == (cdu->command[0] = XPLMFindCommand("tbm900/popups/ap"        )) ||
                        NULL == (cdu->command[1] = XPLMFindCommand("tbm900/popups/mfd"       )) ||
                        NULL == (cdu->command[2] = XPLMFindCommand("tbm900/popups/pfd1"      )) ||
                        NULL == (cdu->command[3] = XPLMFindCommand("tbm900/popups/tablet"    )) ||
                        NULL == (cdu->command[4] = XPLMFindCommand("tbm900/popups/esi2000"   )) ||
                        NULL == (cdu->command[5] = XPLMFindCommand("tbm900/popups/mfd_keypad")))
                    {
                        cdu->i_disabled = 1; break; // check for YFMS presence
                    }
                    cdu->i_cycle_id = 0; cdu->i_disabled = 0; break;
#else
                    cdu->i_disabled = 1; break; // check for YFMS presence
#endif

                case ACF_TYP_B737_EA:
                    if (x737 != XPLM_NO_PLUGIN_ID)
                    {
                        if (XPLMIsPluginEnabled(x737) == 0)
                        {
                            XPLMEnablePlugin(x737);
                        }
                        if (NULL == (cdu->command[0] = XPLMFindCommand("x737/UFMC/FMC_TOGGLE")))
                        {
                            cdu->i_disabled = 1; break; // check for YFMS presence
                        }
                        cdu->i_disabled = 0; break;
                    }
                    // fall through
                default:
                {
                    if (*cdu->auth || *cdu->desc) // we may see "" description
                    {
                        if (!STRN_CASECMP_AUTO(cdu->auth, "Aerobask") ||
                            !STRN_CASECMP_AUTO(cdu->auth, "Stephane Buon"))
                        {
                            if (!STRN_CASECMP_AUTO(cdu->icao, "DA62"))
                            {
                                if ((cdu->command[0] = XPLMFindCommand("sim/GPS/g1000n1_popup")) &&
                                    (cdu->command[1] = XPLMFindCommand("sim/GPS/g1000n3_popup")))
                                {
                                    cdu->i_cycle_id = 0;
                                    cdu->i_cycletyp = 1; // MD-302 + GFC700 + G1000 (x2)
                                    cdu->i_disabled = 0; break; // Aerobask G1000 (DA62)
                                }
                            }
                            if (!STRN_CASECMP_AUTO(cdu->icao, "EPIC") ||
                                !STRN_CASECMP_AUTO(cdu->icao, "EVIC"))
                            {
                                if ((cdu->command[0] = XPLMFindCommand("aerobask/gfc700_popup_show")) &&
                                    (cdu->command[1] = XPLMFindCommand("aerobask/gfc700_popup_hide")) &&
                                    (cdu->command[2] = XPLMFindCommand("aerobask/gcu477_popup_show")) &&
                                    (cdu->command[3] = XPLMFindCommand("aerobask/gcu477_popup_hide")) &&
                                    (cdu->command[4] = XPLMFindCommand("sim/GPS/g1000n1_popup")) &&
                                    (cdu->command[5] = XPLMFindCommand("sim/GPS/g1000n3_popup")))
                                {
                                    cdu->i_cycle_id = 0;
                                    cdu->i_cycletyp = 2; // GCU477 + GFC700 + G1000 (x3)
                                    cdu->i_disabled = 0; break; // Aerobask G1000 (EVIC)
                                }
                            }
                            if (!STRN_CASECMP_AUTO(cdu->icao, "EA50") ||
                                !STRN_CASECMP_AUTO(cdu->icao, "PIPA"))
                            {
                                if ((cdu->dataref[0] = XPLMFindDataRef("aerobask/eclipse/dynonL_Show")) &&
                                    (cdu->dataref[1] = XPLMFindDataRef("aerobask/eclipse/gtn650_Show")) &&
                                    (cdu->dataref[2] = XPLMFindDataRef("aerobask/eclipse/dynonR_Show")) &&
                                    (cdu->dataref[3] = XPLMFindDataRef("aerobask/eclipse/gtn750_Show")))
                                {
                                    cdu->i_cycletyp = 3; // GTN650 + GTN750 + Skyview (x2)
                                    cdu->i_disabled = 0; break; // Aerobask double SkyView
                                }
                                if ((cdu->dataref[0] = XPLMFindDataRef("aerobask/panthera/dynonL_Show")) &&
                                    (cdu->dataref[1] = XPLMFindDataRef("aerobask/panthera/gtn650_Show")) &&
                                    (cdu->dataref[2] = XPLMFindDataRef("aerobask/panthera/dynonR_Show")) &&
                                    (cdu->dataref[3] = XPLMFindDataRef("aerobask/panthera/gtn750_Show")))
                                {
                                    cdu->i_cycletyp = 3; // GTN650 + GTN750 + Skyview (x2)
                                    cdu->i_disabled = 0; break; // Aerobask double SkyView
                                }
                            }
                            if ((cdu->command[0] = XPLMFindCommand("aerobask/skyview/toggle_left")))
                            {
                                cdu->i_disabled = 0; break; // Aerobask SkyView
                            }
                            if ((cdu->command[0] = XPLMFindCommand("sim/GPS/g430n1_popup")) &&
                                (cdu->command[1] = XPLMFindCommand("sim/GPS/g430n2_popup")))
                            {
                                cdu->i_disabled = 0; break; // X-Plane GPS x2
                            }
                            if ((cdu->command[0] = XPLMFindCommand("sim/GPS/g430n1_popup")))
                            {
                                cdu->i_disabled = 0; break; // X-Plane GPS x1
                            }
                            cdu->i_disabled = 1; break; // check for YFMS presence
                        }
                        if (!STRN_CASECMP_AUTO(cdu->auth, "Alabeo") ||
                            !STRN_CASECMP_AUTO(cdu->auth, "Carenado"))
                        {
                            if (!STRN_CASECMP_AUTO(cdu->desc, "B1900"))
                            {
                                if ((cdu->command[0] = XPLMFindCommand("xap/panels/2")))
                                {
                                    cdu->i_disabled = 0; break; // A/P & EFIS CP
                                }
                            }
                            if (!STRN_CASECMP_AUTO(cdu->desc, "CT206H Stationair"))
                            {
                                if ((cdu->command[0] = XPLMFindCommand("xap/panels/2")) && // A/P
                                    (cdu->command[2] = XPLMFindCommand("sim/GPS/g430n1_popup")))
                                {
                                    cdu->i_disabled = 0; break; // X-Plane GPS
                                }
                            }
                            if (!STRN_CASECMP_AUTO(cdu->desc, "C207 Skywagon"))
                            {
                                if ((cdu->command[0] = XPLMFindCommand("xap/panels/2")) && // A/P
                                    (cdu->command[1] = XPLMFindCommand("xap/panels/3")) && // FF
                                    (cdu->command[2] = XPLMFindCommand("sim/GPS/g430n1_popup")))
                                {
                                    cdu->i_disabled = 0; break; // X-Plane GPS
                                }
                            }
                            if (!STRN_CASECMP_AUTO(cdu->desc, "T210M Centurion II"))
                            {
                                if ((cdu->command[0] = XPLMFindCommand("xap/panels/2"))/* A/P */&&
                                    (cdu->command[1] = XPLMFindCommand("sim/GPS/g430n1_popup")) &&
                                    (cdu->command[2] = XPLMFindCommand("sim/GPS/g430n2_popup")))
                                {
                                    cdu->i_disabled = 0; break; // X-Plane GPS
                                }
                            }
                            if (!STRN_CASECMP_AUTO(cdu->desc, "C404 Titan"))
                            {
                                if ((cdu->command[0] = XPLMFindCommand("xap/panels/2"))         && // A/P
                                    (cdu->command[1] = XPLMFindCommand("sim/GPS/g430n1_popup")) &&
                                    (cdu->command[2] = XPLMFindCommand("sim/GPS/g430n2_popup")))
                                {
                                    cdu->i_disabled = 0; break; // X-Plane GPS
                                }
                            }
                            if (!STRN_CASECMP_AUTO(cdu->desc, "Pilatus PC12"))
                            {
                                if ((cdu->command[0] = XPLMFindCommand("xap/panels/2")) && // A/P
                                    (cdu->command[1] = XPLMFindCommand("xap/panels/3")) && // EIS
                                    (cdu->command[2] = XPLMFindCommand("sim/GPS/g430n1_popup")) &&
                                    (cdu->command[3] = XPLMFindCommand("sim/GPS/g430n2_popup")))
                                {
                                    cdu->i_cycle_id = 0;
                                    cdu->i_cycletyp = 4;
                                    cdu->i_disabled = 0; break; // X-Plane GPS
                                }
                            }
                            if (!STRN_CASECMP_AUTO(cdu->desc, "Piper PA-34 Seneca V"))
                            {
                                if ((cdu->command[0] = XPLMFindCommand("xap/panels/2")) && // A/P
                                    (cdu->command[1] = XPLMFindCommand("xap/panels/3")) && // G500
                                    (cdu->command[2] = XPLMFindCommand("xap/panels/4")) && // XPDR
                                    (cdu->command[3] = XPLMFindCommand("sim/GPS/g430n1_popup")))
                                {
                                    cdu->i_disabled = 0; break; // Carenado G500
                                }
                            }
                            if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("Carenado.G1000.Database"))
                            {
                                cdu->i_disabled = 1; return 0; // Carenado G1000
                            }
                            if ((cdu->command[0] = XPLMFindCommand("sim/GPS/g430n1_popup")) &&
                                (cdu->command[1] = XPLMFindCommand("sim/GPS/g430n2_popup")))
                            {
                                cdu->i_disabled = 0; break; // X-Plane GPS x2
                            }
                            if ((cdu->command[0] = XPLMFindCommand("sim/GPS/g430n1_popup")))
                            {
                                cdu->i_disabled = 0; break; // X-Plane GPS x1
                            }
                            cdu->i_disabled = 1; break; // check for YFMS presence
                        }
                        if ((cdu->command[0] = XPLMFindCommand("sim/GPS/g430n1_popup")) &&
                            (cdu->command[1] = XPLMFindCommand("sim/GPS/g430n2_popup")))
                        {
                            cdu->i_disabled = 0; break; // X-Plane GPS x2
                        }
                        if ((cdu->command[0] = XPLMFindCommand("sim/GPS/g430n1_popup")))
                        {
                            cdu->i_disabled = 0; break; // X-Plane GPS x1
                        }
                    }
                    if (sfmc != XPLM_NO_PLUGIN_ID)
                    {
                        if (XPLMIsPluginEnabled(sfmc) == 0)
                        {
                            XPLMEnablePlugin(sfmc);
                        }
                        if (NULL == (cdu->command[0] = XPLMFindCommand("pikitanga/SimpleFMC/ToggleSimpleFMC")))
                        {
                            cdu->i_disabled = 1; break; // check for YFMS presence
                        }
                        cdu->i_disabled = 0; break;
                    }
                    if (xfmc != XPLM_NO_PLUGIN_ID)
                    {
                        if (XPLMIsPluginEnabled(xfmc) == 0)
                        {
                            XPLMEnablePlugin(xfmc);
                        }
                        if (NULL == (cdu->command[0] = XPLMFindCommand("xfmc/toggle")))
                        {
                            cdu->i_disabled = 1; break; // check for YFMS presence
                        }
                        cdu->i_disabled = 0; break;
                    }
                    cdu->i_disabled = 1; break; // check for YFMS presence
                }
            }
            if (cdu->i_disabled == -1)
            {
                ndt_log("navP [debug]: chandler_mcdup has a BUG\n");
                cdu->i_disabled = 1; return 0;
            }
        }
        if (cdu->i_disabled)
        {
#ifndef NAVP_ONLY
            if ((cdu->command[0] = XPLMFindCommand("YFMS/toggle")) == NULL)
            {
                return 0;
            }
            cdu->atyp = ACF_TYP_GENERIC;
            cdu->i_disabled = 0;
#else
            return 0;
#endif
        }
        switch (cdu->atyp)
        {
            case ACF_TYP_A319_TL:
            case ACF_TYP_A321_TL:
            {
                int PopUpHeightArray[2]; XPLMGetDatavi(cdu->dataref[0], PopUpHeightArray, 0, 2);
                if (PopUpHeightArray[0] <= 0 && PopUpHeightArray[1] <= 0) // both popups hidden
                {
                    // reset any relevant dataref to preset size/location and show both MCDUs
                    // ISI top right matches PFD top right (500 - 200 = 300, 500 - 200 = 300)
                    // L PFD: bottom left (0,0) R PFD offset to the top (0, 1120 - 500 = 620)
                    // R PFD and R ND: same position as their left counterparts, but inverted
                    // upper ECAM in upper right corner (1792 - 500 = 1292, 1120 - 500 = 620)
                    // lower ECAM goes right next to it (1292 - 500 = _792, 1120 - 500 = 620)
                    // R CDU: bottom right (1792 - 415 = 1377) L CDU: next to it (1377 - 415 = 962)
                    //                    { CDU1, CDU2, PFD1, PFD2, ND#1, ND#2, ECAM, ECAM, ISIS, };
                    float PopUpScale[9] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, };
                    int   PopUpXArry[9] = {  962, 1377,    0,  500,  500,    0, 1292,  792,  300, };
                    int   PopUpYArry[9] = {    0,    0,    0,    0,    0,    0,  620,  620,  300, };
                    XPLMSetDatavf(cdu->dataref[1], PopUpScale, 0, 9);
                    XPLMSetDatavi(cdu->dataref[2], PopUpXArry, 0, 9);
                    XPLMSetDatavi(cdu->dataref[3], PopUpYArry, 0, 9);
                    XPLMCommandOnce(cdu->command[0]);
                    XPLMCommandOnce(cdu->command[1]);
                    return 0;
                }
                // else either/both MCDU popups visible, let's hide them instead
                if (PopUpHeightArray[0] >= 1)
                {
                    XPLMCommandOnce(cdu->command[0]);
                }
                if (PopUpHeightArray[1] >= 1)
                {
                    XPLMCommandOnce(cdu->command[1]);
                }
                return 0;
            }

            case ACF_TYP_B757_FF:
            case ACF_TYP_B767_FF:
                XPLMSetDatai(cdu->dataref[1], 1); // auto-reset
                // fall through
            case ACF_TYP_B777_FF:
                XPLMSetDatai(cdu->dataref[0], 1); // auto-reset
                return 0;

            case ACF_TYP_CL30_DD:
                XPLMCommandOnce(cdu->command[0]); // toggle custom radio panel
                return 0;

            case ACF_TYP_E55P_AB:
                switch (cdu->i_cycle_id)
                {
                    case 0:
                        XPLMCommandOnce(cdu->command[4]); // G1000: Lt display (show)
                        XPLMCommandOnce(cdu->command[0]); // GFC700: autopilot (show)
//                      XPLMCommandOnce(cdu->command[5]); // G1000: Rt display (hide) (already hidden - we only have a toggle command)
                        XPLMCommandOnce(cdu->command[3]); // GCU-477: keyboard (hide)
                        cdu->i_cycle_id = 1;
                        return 0;
                    case 1:
                        XPLMCommandOnce(cdu->command[4]); // G1000: Lt display (hide)
                        XPLMCommandOnce(cdu->command[1]); // GFC700: autopilot (hide)
                        XPLMCommandOnce(cdu->command[5]); // G1000: Ct display (show)
                        XPLMCommandOnce(cdu->command[2]); // GCU-477: keyboard (show)
                        cdu->i_cycle_id = 2;
                        return 0;
                    case 2:
#if 0 // we resized popups so GCU-477 no longer interferes with EIS on MFD
//                      XPLMCommandOnce(cdu->command[4]); // G1000: Lt display (hide) (already hidden - we only have a toggle command)
                        XPLMCommandOnce(cdu->command[1]); // GFC700: autopilot (hide)
//                      XPLMCommandOnce(cdu->command[5]); // G1000: Ct display (show) (already showing, we only have a toggle command)
                        XPLMCommandOnce(cdu->command[3]); // GCU-477: keyboard (hide)
                        cdu->i_cycle_id = 3;
                        return 0;
                    case 3:
#endif
                    default:
//                      XPLMCommandOnce(cdu->command[4]); // G1000: Lt display (hide) (already hidden - we only have a otggle command)
                        XPLMCommandOnce(cdu->command[1]); // GFC700: autopilot (hide)
                        XPLMCommandOnce(cdu->command[5]); // G1000: Ct display (hide)
                        XPLMCommandOnce(cdu->command[3]); // GCU-477: keyboard (hide)
                        cdu->i_cycle_id = 0;
                        break;
                }
                return 0;

//          // unreachable (see above)
//          case ACF_TYP_EMBE_SS:
//              return 0;

            case ACF_TYP_EMBE_XC:
            {
                XPLMGetDatavi(cdu->dataref[1], &cdu->i_value[1], 16, 1);
                int ione = 1, itog = !cdu->i_value[1]; float ftog = itog;
                XPLMSetDatavf(cdu->dataref[0], &ftog, 22, 1); // toggle ND
                XPLMSetDatavf(cdu->dataref[0], &ftog, 21, 1); // toggle PFD
                XPLMSetDatavi(cdu->dataref[1], &itog, 16, 1); // toggle CDU
                XPLMSetDatavi(cdu->dataref[1], &itog, 19, 1); // toggle radios
                XPLMSetDatavi(cdu->dataref[1], &ione, 14, 1); // hide yoke left
                XPLMSetDatavi(cdu->dataref[1], &ione, 15, 1); // hide yoke right
                XPLMSetDatavi(cdu->dataref[1], &ione, 20, 1); // show toggle buttons
                return 0;
            }

            case ACF_TYP_LEGA_XC:
            {
                float ones = 1.0f, zero = 0.0f, tek;
                XPLMGetDatavf(cdu->dataref[0], &tek, 52, 1);
                if (tek < 0.5f) // is Tekton FMS popup down?
                {
//                  XPLMSetDatavf(cdu->dataref[0], &zero, 16, 1); // PFD
//                  XPLMSetDatavf(cdu->dataref[0], &zero, 17, 1); // ND
                    XPLMSetDatavf(cdu->dataref[0], &ones, 18, 1); // EICAS
                    XPLMSetDatavf(cdu->dataref[0], &ones, 50, 1); // radio
                    XPLMSetDatavf(cdu->dataref[0], &ones, 51, 1); // thrust
                    XPLMSetDatavf(cdu->dataref[0], &ones, 52, 1); // Tekton
                    return 0;
                }
//              XPLMSetDatavf(cdu->dataref[0], &zero, 16, 1); // PFD
//              XPLMSetDatavf(cdu->dataref[0], &zero, 17, 1); // ND
                XPLMSetDatavf(cdu->dataref[0], &zero, 18, 1); // EICAS
                XPLMSetDatavf(cdu->dataref[0], &zero, 50, 1); // radio
                XPLMSetDatavf(cdu->dataref[0], &zero, 51, 1); // thrust
                XPLMSetDatavf(cdu->dataref[0], &zero, 52, 1); // Tekton
                return 0;
            }

#if ((APL) && (CGFLOAT_IS_DOUBLE))
            case ACF_TYP_TBM9_HS:
            {
                /*
                 * Default size of popup windows (1792 x 1120 "Retina"):
                 *
                 * ap:         1636 x _288 -> 1636 x _244 -> _818 x _122
                 * mfd:        1792 x 1332 -> 1792 x 1288 -> _896 x _644
                 * pfd1:       1792 x 1212 -> 1792 x 1168 -> _896 x _584
                 * tablet:     1792 x 1252 -> 1792 x 1208 -> _896 x _604
                 * esi2000:    _800 x _728 -> _800 x _684 -> _400 x _342
                 * mfd_keypad: _984 x _796 -> _984 x _752 -> _492 x _376
                 *
                 * Note: units compatible w/display bounds for mouse moved events.
                 */
                double popupsizes[6][2] =
                {
                    { 818.0, 122.0, },
                    { 896.0, 644.0, },
                    { 896.0, 584.0, },
                    { 896.0, 604.0, },
                    { 400.0, 342.0, },
                    { 492.0, 376.0, },
                };
                cdu->tbm9.current_evnt_index = -1;
                CGRect r = CGDisplayBounds(kCGDirectMainDisplay);
                if ((cdu->tbm9.evnt[cdu->tbm9.current_evnt_index + 1] = CGEventCreateMouseEvent(NULL, kCGEventMouseMoved, // mfd_keypad
                                                                                                CGPointMake(r.origin.x + r.size.width                     - popupsizes[5][0] / 2.0, // full right
                                                                                                            r.origin.y + r.size.height - popupsizes[1][1] - popupsizes[5][1] / 2.0), // above mfd
                                                                                                kCGMouseButtonLeft)))
                {
                    cdu->tbm9.cmmd[cdu->tbm9.current_evnt_index + 1] = cdu->command[5]; // mfd_keypad
                    cdu->tbm9.current_evnt_index++;
                }
                if ((cdu->tbm9.evnt[cdu->tbm9.current_evnt_index + 1] = CGEventCreateMouseEvent(NULL, kCGEventMouseMoved, // mfd
                                                                                                CGPointMake(r.origin.x + r.size.width  - popupsizes[1][0] / 2.0, // full right
                                                                                                            r.origin.y + r.size.height - popupsizes[1][1] / 2.0), // at bottom
                                                                                                kCGMouseButtonLeft)))
                {
                    cdu->tbm9.cmmd[cdu->tbm9.current_evnt_index + 1] = cdu->command[1]; // mfd
                    cdu->tbm9.current_evnt_index++;
                }
                if ((cdu->tbm9.evnt[cdu->tbm9.current_evnt_index + 1] = CGEventCreateMouseEvent(NULL, kCGEventMouseMoved, // pfd1
                                                                                                CGPointMake(r.origin.x + r.size.width  - popupsizes[1][0] - popupsizes[2][0] / 2.0, // next 2 mfd
                                                                                                            r.origin.y + r.size.height                    - popupsizes[2][1] / 2.0), // at bottom
                                                                                                kCGMouseButtonLeft)))
                {
                    cdu->tbm9.cmmd[cdu->tbm9.current_evnt_index + 1] = cdu->command[2]; // pfd1
                    cdu->tbm9.current_evnt_index++;
                }
                if ((cdu->tbm9.evnt[cdu->tbm9.current_evnt_index + 1] = CGEventCreateMouseEvent(NULL, kCGEventMouseMoved, // ap
                                                                                                CGPointMake(r.origin.x + r.size.width  - popupsizes[1][0] - popupsizes[0][0] / 2.0, // next to mfd
                                                                                                            r.origin.y + r.size.height - popupsizes[2][1] - popupsizes[0][1] / 2.0), // above pfd1
                                                                                                kCGMouseButtonLeft)))
                {
                    cdu->tbm9.cmmd[cdu->tbm9.current_evnt_index + 1] = cdu->command[0]; // ap
                    cdu->tbm9.current_evnt_index++;
                }
                XPLMSetFlightLoopCallbackInterval(cdu->tbm9.flc_t, -1, 1, cdu);
                return 0;
            }
#endif

            case ACF_TYP_A350_FF:
                cdu->i_value[1] = !XPLMGetDatai(cdu->dataref[2]);
                cdu->i_value[0] = !XPLMGetDatai(cdu->dataref[1]);
                if (cdu->i_value[1]) // popupsHide == 0: MFD popups enabled
                {
                    XPLMSetDatai(cdu->dataref[1], cdu->i_value[0]); // popupLeft
                    XPLMSetDatai(cdu->dataref[0],             (0)); // popupOis
                    return 0;
                }
                // fall through
            default:
                if (cdu->i_cycletyp == 1)
                {
                    switch (cdu->i_cycle_id)
                    {
                        case 0:
                            XPLMCommandOnce(cdu->command[1]); // G1000: Ct display (show below)
                            XPLMCommandOnce(cdu->command[0]); // G1000: Lt display (show above)
                            cdu->i_cycle_id = 1;
                            break;
                        case 1:
                            XPLMCommandOnce(cdu->command[1]); // G1000: Ct display (hide first)
                            XPLMCommandOnce(cdu->command[1]); // G1000: Ct display (show above)
//                          XPLMCommandOnce(cdu->command[0]); // G1000: Lt display (show) (already showing, we only have a toggle command)
                            cdu->i_cycle_id = 2;
                            break;
                        case 2:
                        default:
                            XPLMCommandOnce(cdu->command[0]); // G1000: Lt display (hide)
                            XPLMCommandOnce(cdu->command[1]); // G1000: Ct display (hide)
                            cdu->i_cycle_id = 0;
                            break;
                    }
                    return 0;
                }
                if (cdu->i_cycletyp == 2)
                {
                    switch (cdu->i_cycle_id)
                    {
                        case 0:
                            XPLMCommandOnce(cdu->command[4]); // G1000: Lt display (show)
                            XPLMCommandOnce(cdu->command[0]); // GFC700: autopilot (show)
//                          XPLMCommandOnce(cdu->command[5]); // G1000: Rt display (hide) (already hidden - we only have a toggle command)
                            XPLMCommandOnce(cdu->command[3]); // GCU-477: keyboard (hide)
                            cdu->i_cycle_id = 1;
                            return 0;
                        case 1:
                            XPLMCommandOnce(cdu->command[4]); // G1000: Lt display (hide)
                            XPLMCommandOnce(cdu->command[1]); // GFC700: autopilot (hide)
                            XPLMCommandOnce(cdu->command[5]); // G1000: Ct display (show)
                            XPLMCommandOnce(cdu->command[2]); // GCU-477: keyboard (show)
                            cdu->i_cycle_id = 2;
                            return 0;
                        case 2:
#if 0 // we resized popups so GCU-477 no longer interferes with EIS on MFD
//                          XPLMCommandOnce(cdu->command[4]); // G1000: Lt display (hide) (already hidden - we only have a toggle command)
                            XPLMCommandOnce(cdu->command[1]); // GFC700: autopilot (hide)
//                          XPLMCommandOnce(cdu->command[5]); // G1000: Ct display (show) (already showing, we only have a toggle command)
                            XPLMCommandOnce(cdu->command[3]); // GCU-477: keyboard (hide)
                            cdu->i_cycle_id = 3;
                            return 0;
                        case 3:
#endif
                        default:
//                          XPLMCommandOnce(cdu->command[4]); // G1000: Lt display (hide) (already hidden - we only have a otggle command)
                            XPLMCommandOnce(cdu->command[1]); // GFC700: autopilot (hide)
                            XPLMCommandOnce(cdu->command[5]); // G1000: Ct display (hide)
                            XPLMCommandOnce(cdu->command[3]); // GCU-477: keyboard (hide)
                            cdu->i_cycle_id = 0;
                            break;
                    }
                    return 0;
                }
                if (cdu->i_cycletyp == 3)
                {
                    if (XPLMGetDatai(cdu->dataref[0]) == 0 && XPLMGetDatai(cdu->dataref[2]) == 0)
                    {
                        // Lt SkyView + GTN 650 hidden, switch to configuration 1
                        XPLMSetDatai(cdu->dataref[0], 1); // Lt SkyView (show)
                        XPLMSetDatai(cdu->dataref[1], 1); // Ct GTN 650 (show)
                        XPLMSetDatai(cdu->dataref[2], 0); // Rt SkyView (hide)
                        XPLMSetDatai(cdu->dataref[3], 0); // Ct GTN 750 (hide)
                        return 0;
                    }
                    if (XPLMGetDatai(cdu->dataref[0]) == 1 && XPLMGetDatai(cdu->dataref[1]) == 1)
                    {
                        // Lt SkyView + GTN 650 showing, switch to configuration 2
                        XPLMSetDatai(cdu->dataref[0], 0); // Lt SkyView (hide)
                        XPLMSetDatai(cdu->dataref[1], 0); // Ct GTN 650 (hide)
                        XPLMSetDatai(cdu->dataref[2], 1); // Rt SkyView (show)
                        XPLMSetDatai(cdu->dataref[3], 1); // Ct GTN 750 (show)
                        return 0;
                    }
#if 0 // we resized popups so GTN-750 no longer interferes with EIS on MFD
                    if (XPLMGetDatai(cdu->dataref[2]) == 1)
                    {
                        if (XPLMGetDatai(cdu->dataref[3]) == 1)
                        {
                            // Rt SkyView + GTN 750 showing, switch to configuration 3
                            XPLMSetDatai(cdu->dataref[0], 0); // Lt SkyView (hide)
                            XPLMSetDatai(cdu->dataref[1], 0); // Ct GTN 650 (hide)
                            XPLMSetDatai(cdu->dataref[2], 1); // Rt SkyView (show)
                            XPLMSetDatai(cdu->dataref[3], 0); // Ct GTN 750 (hide)
                            return 0;
                        }
                        // Rt SkyView showing only, turn everything off (fall through)
                    }
#endif
                    // other popup configuration, turn everything off
                    XPLMSetDatai(cdu->dataref[0], 0); // Lt SkyView (hide)
                    XPLMSetDatai(cdu->dataref[1], 0); // Ct GTN 650 (hide)
                    XPLMSetDatai(cdu->dataref[2], 0); // Rt SkyView (hide)
                    XPLMSetDatai(cdu->dataref[3], 0); // Ct GTN 750 (hide)
                    return 0;
                }
                if (cdu->i_cycletyp == 4)
                {
                    switch (cdu->i_cycle_id)
                    {
                        case 0:
                            XPLMCommandOnce(cdu->command[0]); // A/P: popup (show)
//                          XPLMCommandOnce(cdu->command[1]); // EIS: popup (hide) (already hidden - we only have a toggle command)
//                          XPLMCommandOnce(cdu->command[3]); // GNS2 popup (hide) (already hidden - we only have a toggle command)
//                          XPLMCommandOnce(cdu->command[2]); // GNS1 popup (hide) (already hidden - we only have a toggle command)
                            cdu->i_cycle_id = 1;
                            return 0;

                        case 1:
                            XPLMCommandOnce(cdu->command[0]); // A/P: popup (hide)
                            XPLMCommandOnce(cdu->command[1]); // EIS: popup (show)
//                          XPLMCommandOnce(cdu->command[3]); // GNS2 popup (hide) (already hidden - we only have a toggle command)
//                          XPLMCommandOnce(cdu->command[2]); // GNS1 popup (hide) (already hidden - we only have a toggle command)
                            cdu->i_cycle_id = 2;
                            return 0;

                        case 2:
//                          XPLMCommandOnce(cdu->command[0]); // A/P: popup (hide) (already hidden - we only have a toggle command)
                            XPLMCommandOnce(cdu->command[1]); // EIS: popup (hide)
                            XPLMCommandOnce(cdu->command[3]); // GNS2 popup (show)
                            XPLMCommandOnce(cdu->command[2]); // GNS1 popup (show)
                            cdu->i_cycle_id = 3;
                            return 0;

                        case 3:
                        default:
//                          XPLMCommandOnce(cdu->command[0]); // A/P: popup (hide) (already hidden - we only have a toggle command)
//                          XPLMCommandOnce(cdu->command[1]); // EIS: popup (hide) (already hidden - we only have a toggle command)
                            XPLMCommandOnce(cdu->command[3]); // GNS2 popup (hide)
                            XPLMCommandOnce(cdu->command[2]); // GNS1 popup (hide)
                            cdu->i_cycle_id = 0;
                            return 0;
                    }
                }
                if (cdu->command[0]) XPLMCommandOnce(cdu->command[0]);
                if (cdu->command[1]) XPLMCommandOnce(cdu->command[1]);
                if (cdu->command[2]) XPLMCommandOnce(cdu->command[2]);
                if (cdu->command[3]) XPLMCommandOnce(cdu->command[3]);
                return 0;
        }
    }
    return 0;
}

static int chandler_ffap1(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        XPLMDataRef dataref = *((XPLMDataRef*)inRefcon);
        if (dataref == NULL)
        {
            if ((dataref = XPLMFindDataRef("1-sim/AP/cmd_L_Button")) == NULL)
            {
                ndt_log("navP [warning]: chandler_ffap1: \"1-sim/AP/cmd_L_Button\" not found\n");
                return 0;
            }
            *((XPLMDataRef*)inRefcon) = dataref;
        }
        XPLMSetDatai(dataref, !XPLMGetDatai(dataref));
    }
    return 0;
}

static int chandler_32apc(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd && inRefcon)
    {
        if (acf_type_info_acf_ctx_init() == 0)
        {
            assert_context *a32 = inRefcon;
            int32_t ap1lite; a32->api.ValueGet(a32->dat.id_s32_light_autopilot1, &ap1lite);
            int32_t ap2lite; a32->api.ValueGet(a32->dat.id_s32_light_autopilot1, &ap2lite);
            if (ap1lite == 0 && ap2lite == 0)
            {
                XPLMCommandOnce(a32->dat.toggle_srvos);
            }
        }
    }
    return 0;
}

static int chandler_32apd(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd && inRefcon)
    {
        if (acf_type_info_acf_ctx_init() == 0)
        {
            assert_context *a32 = inRefcon;
            int32_t clicknow = 1; a32->api.ValueSet(a32->dat.id_s32_click_ss_tkovr_l, &clicknow);
        }
    }
    return 0;
}

static int chandler_32atd(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd && inRefcon)
    {
        if (acf_type_info_acf_ctx_init() == 0)
        {
            assert_context *a32 = inRefcon;
            int32_t clicknow = 1; a32->api.ValueSet(a32->dat.id_s32_click_thr_disc_l, &clicknow);
        }
    }
    return 0;
}

static int chandler_31isc(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inRefcon) // do it for all phases just in case
    {
        int x = 0; XPLMSetDatavi(((refcon_tolifbw*)inRefcon)->popup_x, &x, 9, 1);
        int y = 0; XPLMSetDatavi(((refcon_tolifbw*)inRefcon)->popup_y, &y, 9, 1);
        return 1; // pass through
    }
    return 1; // pass through
}

static int chandler_ghndl(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin) // before X-Plane moves the handle
    {
        refcon_gear    *gear = inRefcon;
        assert_context *a320 = gear->assert;
        if (gear->has_retractable_gear == -1)
        {
            if (a320)
            {
                gear->has_retractable_gear = 1;
            }
            else
            {
                gear->has_retractable_gear = !!XPLMGetDatai(gear->acf_gear_retract);
            }
        }
        if (gear->has_retractable_gear)
        {
            int speak = XPLMGetDatai(gear->callouts.ref);
            if (inCommand == gear->landing_gear_toggle.command ||
                inCommand == gear->landing_gear_down  .command)
            {
                if (a320)
                {
                    if (XPLMGetDataf(a320->dat.ldg_gears_lever) < 0.5f) // -> 1
                    {
                        if (speak > 0) XPLMSpeakString("gear down"); return 1;
                    }
                }
                if (XPLMGetDatai(gear->gear_handle_down) == 0) // 0 -> 1
                {
                    if (speak > 0) XPLMSpeakString("gear down"); return 1;
                }
            }
            if (inCommand == gear->landing_gear_toggle.command ||
                inCommand == gear->landing_gear_up    .command)
            {
                if (a320)
                {
                    if (XPLMGetDataf(a320->dat.ldg_gears_lever) > 0.5f) // -> 0
                    {
                        if (speak > 0) XPLMSpeakString("gear up"); return 1;
                    }
                }
                if (XPLMGetDatai(gear->gear_handle_down) != 0) // 1 -> 0
                {
                    if (speak > 0) XPLMSpeakString("gear up");   return 1;
                }
            }
            return 1; // let X-Plane actually move the handle
        }
        return 1; // let X-Plane actually move the handle
    }
    return 1; // let X-Plane actually move the handle
}

static int chandler_apbef(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        refcon_apd *apd = inRefcon;
        apd->at_val = !!XPLMGetDatai(apd->at_ref);
    }
    return 1;
}

static int chandler_apaft(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        refcon_apd *apd = inRefcon;
        if (apd->at_val)
        {
            if (XPLMGetDatai(apd->fd_ref) >= 1) // XP A/T won't work without F/D
            {
                XPLMCommandOnce(apd->at_cmd);
            }
        }
    }
    return 1;
}

static int chandler_p2vvi(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    refcon_vvi *vvi = inRefcon;
    if (XPLMGetDatai(vvi->ot_vs_on) >= 1)
    {
        if (inPhase == xplm_CommandBegin)
        {
            if (inCommand == vvi->dn.command ||
                inCommand == vvi->pd.command)
            {
                XPLMCommandOnce(vvi->vs_dn);
                return 0;
            }
            if (inCommand == vvi->up.command ||
                inCommand == vvi->pu.command)
            {
                XPLMCommandOnce(vvi->vs_up);
                return 0;
            }
        }
        return 0;
    }
    return 1; // pass through
}

static int chandler_coatc(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inRefcon)
    {
        switch (inPhase)
        {
            case xplm_CommandBegin:
                XPLMCommandBegin(inRefcon);
                break;
            case xplm_CommandEnd:
                XPLMCommandEnd(inRefcon);
                break;
            default:
                break;
        }
    }
    return 0; // suppress all default "contact ATC" functionality
}

static float flc_flap_cmmd(float inElapsedSinceLastCall,
                           float inElapsedTimeSinceLastFlightLoop,
                           int   inCounter,
                           void *inRefcon)
{
    if (inRefcon)
    {
        XPLMCommandOnce(inRefcon);
        return 0;
    }
    return 0;
}

static float flc_flap_func(float inElapsedSinceLastCall,
                           float inElapsedTimeSinceLastFlightLoop,
                           int   inCounter,
                           void *inRefcon)
{
    flap_callout_speak();
    return 0;
}

static float flc_oatc_func(float inElapsedSinceLastCall,
                           float inElapsedTimeSinceLastFlightLoop,
                           int   inCounter,
                           void *inRefcon)
{
    if (inRefcon)
    {
        acf_volume_context *volume_context = acf_volume_ctx_get();
        if (volume_context == NULL)
        {
            ndt_log("navP [error]: flc_oatc_func: acf_volume_ctx_get() failed\n");
            return 0;
        }
        acf_volume_reset(volume_context, *((int*)inRefcon));
        return 0;
    }
    return 0;
}

static float gnd_stab_hdlr(float inElapsedSinceLastCall,
                           float inElapsedTimeSinceLastFlightLoop,
                           int   inCounter,
                           void *inRefcon)
{
    refcon_ground *grndp = inRefcon;
    if (grndp)
    {
        // set default flight loop callback interval based on whether we're airborne or on ground
        float flightLoopCallbackInterval = !XPLMGetDatai(grndp->ongrnd_any) ? 1.0f : 0.25f;

#if TIM_ONLY
        /*
         * Partial time sync: minutes/seconds.
         * We overwrite once every IRL second.
         *
         * We don't overwrite if the time is within one minute of the hour
         * changeover to avoid accidentally going back/forward by one hour.
         */
        ndt_date now = ndt_date_now();
        if (grndp->last_second_index != 30 && now.seconds == 30)
        {
            int xphrs = XPLMGetDatai(grndp->time.zulu_time_hrs);
            int xpmin = XPLMGetDatai(grndp->time.zulu_time_min);
            int skip1 = xpmin == 59 && now.minutes == 0; // don't go backwards
            int skip2 = xpmin == 0 && now.minutes == 59; // nor forward either
            if (skip1 == 0 && skip2 == 0)
            {
                if (xpmin != now.minutes)
                {
                    int m = XPLMGetDatai(grndp->time.zulu_time_min);
                    int s = XPLMGetDatai(grndp->time.zulu_time_sec);
                    int d = (60 * now.minutes + now.seconds) - (60 * m + s);
                    if (d > +1800)
                    {
                        xphrs = (xphrs - 1) % 24; // increase by over 30 minutes -> decrease by under 30 minutes
                    }
                    if (d < -1800)
                    {
                        xphrs = (xphrs + 1) % 24; // descrese by over 30 minutes -> increase by under 30 minutes
                    }
                    if (abs(d) > 90)
                    {
                        ndt_log("navP [info]: zulu time resync: %02d:%02d:%02d -> %02d:%02d:%02d at %02d:%02d:%02d\n",
                                XPLMGetDatai(grndp->time.zulu_time_hrs), m, s, xphrs,
                                now.minutes, now.seconds, now.hours, now.minutes, now.seconds);
                    }
                    XPLMSetDataf(grndp->time.zulu_time_xpl, (float)(3600 * xphrs + 60 * now.minutes + now.seconds));
                }
                else
                {
#if 0
                    ndt_log("navP [debug]: zulu time resync: %02d:%02d:%02d -> %02d:%02d:%02d at %02d:%02d:%02d\n",
                            xphrs, XPLMGetDatai(grndp->time.zulu_time_min), XPLMGetDatai(grndp->time.zulu_time_sec),
                            xphrs, now.minutes, now.seconds, now.hours, now.minutes, now.seconds);
#endif
                    XPLMSetDataf(grndp->time.zulu_time_xpl, (float)(3600 * xphrs + 60 * now.minutes + now.seconds));
                }
            }
        }
        grndp->last_second_index = now.seconds;

        /*
         * Auto-enable/disable sim clouds based
         * on self-measured average frame rate.
         *
         * Reset on pause or UI/menu activity.
         */
        if ((inElapsedSinceLastCall - flightLoopCallbackInterval) > 1.0f || // triggered by e.g. UI/menu activity
            (XPLMGetDatai(grndp->time.sim_pause) > 0))
        {
            grndp->last_cycle_number = -1; // reset, start counting again on next call
        }
        else if (grndp->last_cycle_number < 0)
        {
            // callback re-registration or reset
            grndp->last_cycle_number = inCounter;
            grndp->curr_period_durr = 10.0f;
            grndp->elapsed_fr_reset = 0.0f;
        }
        else
        {
            switch (XPLMGetDatai(grndp->time.view_type)) // skip if menu showing etc.
            {
                case 1000: // panel
                case 1017: // chase
                case 1018: // circle
                case 1023: // 2D w/HUD
                case 1026: // 3D cockpit
                case 1031: // ride-along
                    if ((grndp->elapsed_fr_reset += inElapsedSinceLastCall) > grndp->curr_period_durr)
                    {
                        /*
                         * reset + compute average fps during last period
                         *
                         * XPLM_API int XPLMGetCycleNumber(void);
                         * This routine returns a counter starting at zero
                         * for each sim cycle computed/video frame rendered.
                         *
                         * Note: test results:
                         * - XPLMGetCycleNumber() == inCounter
                         * - average framerate calculated this
                         *   way appears unaffected by however
                         *   many flitemodels p/frame are set:
                         *   avg(fps) == cycles / duration / 2
                         * - tested with X-Plane 10.51r2 (macOS)
                         * - +verified w/X-Plane 11.41r1 (macOS)
                         */
                        float ncycles = roundf(inCounter - grndp->last_cycle_number);
                        float avg_fps = ncycles / grndp->elapsed_fr_reset / 2.0f;
//                      ndt_log("navP [debug]: XPLMGetCycleNumber() %d\n"
//                              "              inCounter            %d\n", XPLMGetCycleNumber(), inCounter);
//                      ndt_log("navP [debug]: cycle number inccreased by %.0f in last %.1f seconds\n"
//                              "             (average cycles per second %.1f, half %.1f)\n",
//                              ncycles,  grndp->elapsed_fr_reset,
//                              ncycles / grndp->elapsed_fr_reset,
//                              ncycles / grndp->elapsed_fr_reset / 2.0f);
                        if (grndp->nvp_menu)
                        {
                            // note: had cases where fps < 20 w/clouds vs. about 100 w/out!!!
                            float vlo_fps = 24000.0f / 1001.0f, vhi_fps = 144000.0f / 1001.0f;
                            if (avg_fps < vlo_fps)
                            {
                                XPLMDataRef cloud_skip = XPLMFindDataRef("sim/private/controls/clouds/skip_draw");
                                if (cloud_skip && XPLMGetDataf(cloud_skip) < 0.5f) // clouds showing
                                {
                                    ndt_log("navP [info]: fps %.3f < %.3f (%.1f s), "
                                            "disabling clouds for better sim speed\n",
                                            avg_fps, vlo_fps, grndp->elapsed_fr_reset);
                                }
                                nvp_menu_ckill(grndp->nvp_menu, xplm_Menu_Checked);
                                grndp->last_cycle_number = inCounter;
                                grndp->curr_period_durr = 60.0f;
                                grndp->elapsed_fr_reset = 0.0f;
                            }
                            else if (avg_fps > vhi_fps)
                            {
                                XPLMDataRef cloud_skip = XPLMFindDataRef("sim/private/controls/clouds/skip_draw");
                                if (cloud_skip && XPLMGetDataf(cloud_skip) > 0.5f) // clouds hidden
                                {
                                    ndt_log("navP [info]: fps %.3f > %.3f (%.1f s), "
                                            "enabling clouds for increased realism\n",
                                            avg_fps, vhi_fps, grndp->elapsed_fr_reset);
                                }
                                nvp_menu_ckill(grndp->nvp_menu, xplm_Menu_NoCheck);
                                grndp->last_cycle_number = inCounter;
                                grndp->curr_period_durr = 22.5f;
                                grndp->elapsed_fr_reset = 0.0f;
                            }
                            else
                            {
                                grndp->last_cycle_number = inCounter;
                                grndp->curr_period_durr = 15.0f;
                                grndp->elapsed_fr_reset = 0.0f;
                            }
                        }
                        else
                        {
                            grndp->last_cycle_number = inCounter;
                            grndp->curr_period_durr = 15.0f;
                            grndp->elapsed_fr_reset = 0.0f;
                        }
                    }
                    break;

                default:
                    grndp->last_cycle_number = inCounter;
                    if ((grndp->curr_period_durr -= 7.5f) < 7.6f)
                    {
                        (grndp->curr_period_durr = (7.5f));
                    }
                    grndp->elapsed_fr_reset = 0.0f;
                    break;
            }
        }
#endif//TIM_ONLY

        /* XXX: check whether we just connected to PilotEdge or VATSIM */
        if (grndp->oatc.pe_is_on)
        {
            grndp->pt->atc_is_connected = !!XPLMGetDatai(grndp->oatc.pe_is_on);
            if (grndp->pt->atc_is_connected)
            {
                if (grndp->oatc.pe_was_connected < 1)
                {
                    // something resets radios to silent right after connecting to PE, we work around it
                    ndt_log("navP [info]: PilotEdge connection detected, acf_volume_reset in 9 seconds\n");
                    XPLMSetFlightLoopCallbackInterval(grndp->oatc.flc, 9.0f, 1, grndp->oatc.aircraft_type);
                    grndp->pt->thptt = XPLMFindCommand("sim/operation/contact_atc");
                }
            }
            else
            {
                grndp->pt->thptt = NULL;
            }
            grndp->oatc.pe_was_connected = !!grndp->pt->atc_is_connected;
        }
        else if (grndp->oatc.xb_is_on)
        {
            grndp->pt->atc_is_connected = XPLMGetDatai(grndp->oatc.xb_is_on);
            if (grndp->pt->atc_is_connected)
            {
                if (grndp->oatc.xb_was_connected < 1)
                {
                    // not actually required, but we do the same as for PE above
                    ndt_log("navP [info]: XSquawkBox connection detected acf_volume_reset in 9 seconds\n");
/*XXX:XSB-specific*/XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_com_selection" ), 0);
/*XXX:XSB-specific*/XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_selection_com1"), 1);
/*XXX:XSB-specific*/XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_selection_com2"), 1);
                    XPLMSetFlightLoopCallbackInterval(grndp->oatc.flc, 9.0f, 1, grndp->oatc.aircraft_type);
                    grndp->pt->thptt = XPLMFindCommand("xsquawkbox/voice/ptt");
                }
            }
            else
            {
                grndp->pt->thptt = NULL;
            }
            grndp->oatc.xb_was_connected = !!grndp->pt->atc_is_connected;
        }

        // without A/P on (otherwise auto-landing),
        // disable A/T and command idle at 50ft AGL
        if (XPLMGetDatai(grndp->auto_t_sts) >= 1 &&
            XPLMGetDatai(grndp->auto_p_sts) <= 0 &&
            XPLMGetDataf(grndp->elev_m_agl) <= 15.24f)
        {
            XPLMSetDatai(grndp->auto_t_sts, 0);
            XPLMSetDataf(grndp->thrott_all, 0.0f);
        }
        // TODO: FlightFactor 757???
        // its A/T never seems to auto-disconnect (unlike e.g. IXEG's 737), and will
        // command thrust to maintain speed even after touchdown unless auto-landing

        return flightLoopCallbackInterval;
    }
    return 0;
}

static void nvp_efis_setup(void)
{
    XPLMDataRef d_ref;
    _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_copilot");
    _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_pilot");
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_copilot");
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_copilot");
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_pilot");
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_pilot");
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_airport_on");
    _DO(0, XPLMSetDatai, 0, "sim/cockpit2/EFIS/EFIS_fix_on");
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_ndb_on");
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_vor_on");
    _DO(0, XPLMSetDatai, 4, "sim/cockpit2/EFIS/map_range");
    _DO(0, XPLMSetDatai, 2, "sim/cockpit2/EFIS/map_mode");
    return;
}

static void nvp_skyv_setup(void)
{
//  _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_copilot"); // triple-skyview setup: overridden by plugin as well
//  _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_pilot"); // overridden by plugin, no dataref nor command control
    /*
     * the following datarefs are either overriden by plugin or changing them can cause display artifacts due to conflicts about how the plugin
     * writes on top of the default map (thus a specific mode is required); non-issue as such settings can be saved in Aerobask preference file
     */
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_copilot");
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_copilot");
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_pilot");
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_pilot");
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_airport_on");
//  _DO(0, XPLMSetDatai, 0, "sim/cockpit2/EFIS/EFIS_fix_on");
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_ndb_on");
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_vor_on");
//  _DO(0, XPLMSetDatai, 4, "sim/cockpit2/EFIS/map_range");
//  _DO(0, XPLMSetDatai, 2, "sim/cockpit2/EFIS/map_mode");
    XPLMDataRef d_ref;
    return;
}

static void nvp_x1000_setup(void)
{
    XPLMDataRef d_ref;
    _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_copilot");
    _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_pilot");
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_copilot"); // untested
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_copilot"); // untested
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_pilot"); // not tested
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_pilot"); // not tested
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_airport_on");
    _DO(0, XPLMSetDatai, 0, "sim/cockpit2/EFIS/EFIS_fix_on");
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_ndb_on");
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_vor_on");
//  _DO(0, XPLMSetDatai, 4, "sim/cockpit2/EFIS/map_range"); // un-tested
//  _DO(0, XPLMSetDatai, 2, "sim/cockpit2/EFIS/map_mode"); // not tested
    return;
}

static void nvp_xgps_setup(void)
{
    XPLMDataRef d_ref;
    _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_copilot");
    _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_pilot");
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_copilot"); // untested
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_copilot"); // untested
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_pilot"); // not tested
//  _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_pilot"); // not tested
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_airport_on");
    _DO(0, XPLMSetDatai, 0, "sim/cockpit2/EFIS/EFIS_fix_on");
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_ndb_on");
    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_vor_on");
//  _DO(0, XPLMSetDatai, 4, "sim/cockpit2/EFIS/map_range"); // un-tested
//  _DO(0, XPLMSetDatai, 2, "sim/cockpit2/EFIS/map_mode"); // not tested
    return;
}

static void nvp_xnz_setup(int engine_count, int engines_on)
{
    XPLMCommandRef cr;
    /*
     * Sync TCA Quadrant switches w/default state.
     */
    const char* master[4][2] =
    {
        { "xnz/tca/engines/1/on", "xnz/tca/engines/1/off", },
        { "xnz/tca/engines/2/on", "xnz/tca/engines/2/off", },
        { "xnz/tca/engines/3/on", "xnz/tca/engines/3/off", },
        { "xnz/tca/engines/4/on", "xnz/tca/engines/4/off", },
    };
    switch (engine_count)
    {
        case 8:
        case 7:
        case 6:
        case 5:
        case 4:
            if ((cr = XPLMFindCommand(master[3][0 == engines_on])))
            {
                XPLMCommandOnce(cr);
            }
        case 3:
            if ((cr = XPLMFindCommand(master[2][0 == engines_on])))
            {
                XPLMCommandOnce(cr);
            }
            if ((cr = XPLMFindCommand("xnz/tca/34/modes/norm")))
            {
                XPLMCommandOnce(cr);
            }
        case 2:
            if ((cr = XPLMFindCommand(master[1][0 == engines_on])))
            {
                XPLMCommandOnce(cr);
            }
        case 1:
            if ((cr = XPLMFindCommand(master[0][0 == engines_on])))
            {
                XPLMCommandOnce(cr);
            }
            if ((cr = XPLMFindCommand("xnz/tca/12/modes/norm")))
            {
                XPLMCommandOnce(cr);
            }
            return;

        default:
            break;
    }
    return;
}

static int first_fcall_do(chandler_context *ctx)
{
    XPLMDataRef d_ref; XPLMPluginID p_id; XPLMCommandRef cr;
    int absk = 0, x1000 = 0, xgps = 0, xnz = 0, simc = 0, r;
    if ((r = acf_type_info_acf_ctx_init()))
    {
        if (r == EAGAIN)
        {
            return 0; // try again later
        }
        ndt_log("navP [error]: first_fcall_do: acf_type_info_acf_ctx_init() failed\n");
        XPLMSpeakString("nav P first call failed");
        return (ctx->first_fcall = 0) - 1;
    }
    switch (ctx->info->ac_type)
    {
        case ACF_TYP_A319_TL:
        case ACF_TYP_A320_FF:
        case ACF_TYP_A321_TL:
        case ACF_TYP_A350_FF:
            _DO(1, XPLMSetDataf, 0.0f, "sim/joystick/joystick_heading_augment");
            _DO(1, XPLMSetDataf, 0.0f, "sim/joystick/joystick_pitch_augment");
            _DO(1, XPLMSetDataf, 0.0f, "sim/joystick/joystick_roll_augment");
            break;

        case ACF_TYP_E55P_AB:
            _DO(1, XPLMSetDataf, 1.0f, "sim/joystick/joystick_heading_sensitivity");
            _DO(1, XPLMSetDataf, 1.0f, "sim/joystick/joystick_pitch_sensitivity");
            _DO(1, XPLMSetDataf, 1.0f, "sim/joystick/joystick_roll_sensitivity");
            _DO(1, XPLMSetDataf, 0.0f, "sim/joystick/joystick_heading_augment");
            _DO(1, XPLMSetDataf, 0.0f, "sim/joystick/joystick_pitch_augment");
            _DO(1, XPLMSetDataf, 0.0f, "sim/joystick/joystick_roll_augment");
            break;

        default:
            break;
    }
    switch (ctx->info->ac_type)
    {
        case ACF_TYP_A320_FF:
            {
                assert_context *rca = &ctx->info->assert;
                if (acf_type_is_engine_running() == 0) // cold & dark
                {
                    int32_t request_true = 1; float p = 756.0f, f = 4200.0f;
                    rca->api.ValueSet(rca->dat.id_s32_acft_request_chk, &request_true);
                    rca->api.ValueSet(rca->dat.id_s32_acft_request_gpu, &request_true);
                    acf_type_load_set(ctx->info, &p); acf_type_fuel_set(ctx->info, &f);
                }
                if (ctx->mcdu.rc.i_disabled == -1)
                {
                    chandler_mcdup(ctx->mcdu.cb.command, xplm_CommandEnd, &ctx->mcdu.rc); // XXX: remap hotkeys
                }
                if (XPLM_NO_PLUGIN_ID != (p_id = XPLMFindPluginBySignature("skiselkov.xraas2_ff_a320")))
                {
                    if (XPLMIsPluginEnabled(p_id))
                    {
                        XPLMDisablePlugin(p_id);
                    }
                }
                uint32_t defaults_u32[10] = { 0, 2, 2, 2, 2, 0, 0, 0, 0, 0, };
                rca->api.ValueSet(rca->dat.id_u32_fcu_tgt_alt_step, &defaults_u32[0]); // FCU alt. sel. incre. (100ft)
                rca->api.ValueSet(rca->dat.id_u32_efis_nav_mod_lft, &defaults_u32[1]); // ND m. sel. (cap. side) (nav)
                rca->api.ValueSet(rca->dat.id_u32_efis_nav_mod_rgt, &defaults_u32[2]); // ND m. sel. (f/o. side) (nav)
                rca->api.ValueSet(rca->dat.id_u32_efis_nav_rng_lft, &defaults_u32[3]); // ND r. sel. (cap. side) ( 40)
                rca->api.ValueSet(rca->dat.id_u32_efis_nav_rng_rgt, &defaults_u32[4]); // ND r. sel. (f/o. side) ( 40)
                rca->api.ValueSet(rca->dat.id_u32_light_mode_belts, &defaults_u32[5]); // overhead (pas. signs: belts)
                rca->api.ValueSet(rca->dat.id_u32_light_mode_emerg, &defaults_u32[6]); // overhead (emer. exit lights)
                rca->api.ValueSet(rca->dat.id_u32_light_mode_smoke, &defaults_u32[7]); // overhead (pas. signs: smoke)
                rca->api.ValueSet(rca->dat.id_u32_light_mode_strob, &defaults_u32[8]); // overhead (strobe light mode)
                rca->api.ValueSet(rca->dat.id_u32_overhead_rmp3pow, &defaults_u32[9]); // overhead (RMP #3: power off)
                XPLMCommandOnce(rca->dat.throttles_dn); // workaround bug in A320's default "cold & dark" saved state
            }
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            break;

        case ACF_TYP_A319_TL:
        case ACF_TYP_A321_TL:
            if (acf_type_is_engine_running() == 0) // cold & dark
            {
                switch (ctx->info->ac_type)
                {
                    case ACF_TYP_A319_TL:
                    {
                        float load = 677.00f; acf_type_load_set(ctx->info, &load);
                        float fuel = 4200.0f; acf_type_fuel_set(ctx->info, &fuel);
                        break;
                    }
                    case ACF_TYP_A321_TL:
                    {
                        float load = 720.00f; acf_type_load_set(ctx->info, &load);
                        float fuel = 4200.0f; acf_type_fuel_set(ctx->info, &fuel);
                        break;
                    }
                    default:
                    {
                        float load = 700.00f; acf_type_load_set(ctx->info, &load);
                        float fuel = 4200.0f; acf_type_fuel_set(ctx->info, &fuel);
                        break;
                    }
                }
                _DO(1, XPLMSetDatai, 0, "AirbusFBW/GroundLPAir");                   // lo pressure ground air: off
                _DO(1, XPLMSetDatai, 0, "AirbusFBW/GroundHPAir");                   // hi pressure ground air: off
                _DO(1, XPLMSetDatai, 1, "AirbusFBW/EnableExternalPower");           // ensure we have ground power
            }
            if ((d_ref = XPLMFindDataRef("AirbusFBW/DUBrightness")))
            {
                float f[8] = { 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, };
                XPLMSetDatavf(d_ref, f, 0, 8);
            }
            if ((d_ref = XPLMFindDataRef("AirbusFBW/OHPLightSwitches")))
            {
                _DO(1, XPLMSetDataf, 0.8f, "AirbusFBW/WXAlphaND1");                 // ND1 weather to 80%
                _DO(1, XPLMSetDataf, 0.8f, "AirbusFBW/WXAlphaND2");                 // ND2 weather to 80%
            }
            _DO(1, XPLMSetDatai, 1, "params/wheel");                                // use scrollwheel
            _DO(1, XPLMSetDatai, 0, "AirbusFBW/ALT100_1000");                       // FCU alt. sel. incre. (100ft)
            _DO(1,XPLMSetDataf,1.0f,"AirbusFBW/AuralVolume");                       // Loudspeaker vol. (cap. side)
            _DO(1,XPLMSetDataf,1.0f,"AirbusFBW/AuralVolumeFO");                     // Loudspeaker vol. (f/o. side)
            _DO(1, XPLMSetDatai, 1, "AirbusFBW/RMP1Switch");                        // Radio management pan. 1 (on)
            _DO(1, XPLMSetDatai, 1, "AirbusFBW/RMP2Switch");                        // Radio management pan. 2 (on)
            _DO(1, XPLMSetDatai, 0, "AirbusFBW/ALT100_1000");                       // FCU alt. sel. incre. (100ft)
            _DO(1, XPLMSetDatai, 2, "AirbusFBW/NDmodeCapt");                        // ND m. sel. (cap. side) (nav)
            _DO(1, XPLMSetDatai, 2, "AirbusFBW/NDmodeFO");                          // ND m. sel. (f/o. side) (nav)
            _DO(1, XPLMSetDatai, 2, "AirbusFBW/NDrangeCapt");                       // ND r. sel. (cap. side) ( 40)
            _DO(1, XPLMSetDatai, 2, "AirbusFBW/NDrangeFO");                         // ND r. sel. (f/o. side) ( 40)
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_copilot");  // VOR1 on ND2 off
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_copilot");  // VOR2 on ND2 off
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_pilot");    // VOR1 on ND1 off
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_pilot");    // VOR2 on ND1 off
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/radios/actuators/audio_selection_com1"); // C1:TX/RX
            _DO(0, XPLMSetDatai, 6, "sim/cockpit2/radios/actuators/audio_com_selection");  // C1:TX/RX
            if (ctx->info->ac_type == ACF_TYP_A319_TL || ctx->info->ac_type == ACF_TYP_A321_TL)
            {
                float PopUpScale[9] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, };
                int   PopUpXArry[9] = {  962, 1377,    0,  500,  500,    0, 1292,  792,  300, };
                int   PopUpYArry[9] = {    0,    0,    0,    0,    0,    0,  620,  620,  300, };
                if ((d_ref = XPLMFindDataRef("AirbusFBW/PopUpScale")))
                {
                    XPLMSetDatavf(d_ref, PopUpScale, 0, 9);
                }
                if ((d_ref = XPLMFindDataRef("AirbusFBW/PopUpXCoordArray")))
                {
                    XPLMSetDatavi(d_ref, PopUpXArry, 0, 9);
                }
                if ((d_ref = XPLMFindDataRef("AirbusFBW/PopUpYCoordArray")))
                {
                    XPLMSetDatavi(d_ref, PopUpYArry, 0, 9);
                }
                if (ctx->a319kc.kc_is_registered)
                {
                    ndt_log("navP [warning]: AirbusFBW key sniffer already registered\n");
                    break;
                }
                if (NULL == (ctx->a319kc.datar[0] = XPLMFindDataRef("AirbusFBW/PopUpWidthArray"         )) ||
                    NULL == (ctx->a319kc.datar[1] = XPLMFindDataRef("AirbusFBW/PopUpHeightArray"        )) ||
                    NULL == (ctx->a319kc.datar[2] = XPLMFindDataRef("AirbusFBW/PopUpXCoordArray"        )) ||
                    NULL == (ctx->a319kc.datar[3] = XPLMFindDataRef("AirbusFBW/PopUpYCoordArray"        )) ||
                    NULL == (ctx->a319kc.datar[4] = XPLMFindDataRef("sim/graphics/view/view_is_external")))
                {
                    ndt_log("navP [warning]: failed to find AirbusFBW datarefs for key sniffer\n");
                    break;
                }
                if (NULL == (ctx->a319kc.c[2][ 0] = XPLMFindCommand("toliss_airbus/iscs_open"  )) ||
                    NULL == (ctx->a319kc.c[0][ 0] = XPLMFindCommand("AirbusFBW/MCDU1SlewDown"  )) ||
                    NULL == (ctx->a319kc.c[1][ 0] = XPLMFindCommand("AirbusFBW/MCDU2SlewDown"  )) ||
                    NULL == (ctx->a319kc.c[0][ 1] = XPLMFindCommand("AirbusFBW/MCDU1SlewUp"    )) ||
                    NULL == (ctx->a319kc.c[1][ 1] = XPLMFindCommand("AirbusFBW/MCDU2SlewUp"    )) ||
                    NULL == (ctx->a319kc.c[0][ 2] = XPLMFindCommand("AirbusFBW/MCDU1SlewLeft"  )) ||
                    NULL == (ctx->a319kc.c[1][ 2] = XPLMFindCommand("AirbusFBW/MCDU2SlewLeft"  )) ||
                    NULL == (ctx->a319kc.c[0][ 3] = XPLMFindCommand("AirbusFBW/MCDU1SlewRight" )) ||
                    NULL == (ctx->a319kc.c[1][ 3] = XPLMFindCommand("AirbusFBW/MCDU2SlewRight" )) ||
                    NULL == (ctx->a319kc.c[0][ 4] = XPLMFindCommand("AirbusFBW/MCDU1KeySlash"  )) ||
                    NULL == (ctx->a319kc.c[1][ 4] = XPLMFindCommand("AirbusFBW/MCDU2KeySlash"  )) ||
                    NULL == (ctx->a319kc.c[0][ 5] = XPLMFindCommand("AirbusFBW/MCDU1KeySpace"  )) ||
                    NULL == (ctx->a319kc.c[1][ 5] = XPLMFindCommand("AirbusFBW/MCDU2KeySpace"  )) ||
                    NULL == (ctx->a319kc.c[0][ 6] = XPLMFindCommand("AirbusFBW/MCDU1KeyOverfly")) ||
                    NULL == (ctx->a319kc.c[1][ 6] = XPLMFindCommand("AirbusFBW/MCDU2KeyOverfly")) ||
                    NULL == (ctx->a319kc.c[0][ 7] = XPLMFindCommand("AirbusFBW/MCDU1KeyClear"  )) ||
                    NULL == (ctx->a319kc.c[1][ 7] = XPLMFindCommand("AirbusFBW/MCDU2KeyClear"  )) ||
                    NULL == (ctx->a319kc.c[0][ 8] = XPLMFindCommand("AirbusFBW/MCDU1KeyDecimal")) ||
                    NULL == (ctx->a319kc.c[1][ 8] = XPLMFindCommand("AirbusFBW/MCDU2KeyDecimal")) ||
                    NULL == (ctx->a319kc.c[0][ 9] = XPLMFindCommand("AirbusFBW/MCDU1KeyPM"     )) ||
                    NULL == (ctx->a319kc.c[1][ 9] = XPLMFindCommand("AirbusFBW/MCDU2KeyPM"     )) ||
                    NULL == (ctx->a319kc.c[0][11] = XPLMFindCommand("AirbusFBW/MCDU1Key1"      )) ||
                    NULL == (ctx->a319kc.c[1][11] = XPLMFindCommand("AirbusFBW/MCDU2Key1"      )) ||
                    NULL == (ctx->a319kc.c[0][12] = XPLMFindCommand("AirbusFBW/MCDU1Key2"      )) ||
                    NULL == (ctx->a319kc.c[1][12] = XPLMFindCommand("AirbusFBW/MCDU2Key2"      )) ||
                    NULL == (ctx->a319kc.c[0][13] = XPLMFindCommand("AirbusFBW/MCDU1Key3"      )) ||
                    NULL == (ctx->a319kc.c[1][13] = XPLMFindCommand("AirbusFBW/MCDU2Key3"      )) ||
                    NULL == (ctx->a319kc.c[0][14] = XPLMFindCommand("AirbusFBW/MCDU1Key4"      )) ||
                    NULL == (ctx->a319kc.c[1][14] = XPLMFindCommand("AirbusFBW/MCDU2Key4"      )) ||
                    NULL == (ctx->a319kc.c[0][15] = XPLMFindCommand("AirbusFBW/MCDU1Key5"      )) ||
                    NULL == (ctx->a319kc.c[1][15] = XPLMFindCommand("AirbusFBW/MCDU2Key5"      )) ||
                    NULL == (ctx->a319kc.c[0][16] = XPLMFindCommand("AirbusFBW/MCDU1Key6"      )) ||
                    NULL == (ctx->a319kc.c[1][16] = XPLMFindCommand("AirbusFBW/MCDU2Key6"      )) ||
                    NULL == (ctx->a319kc.c[0][17] = XPLMFindCommand("AirbusFBW/MCDU1Key7"      )) ||
                    NULL == (ctx->a319kc.c[1][17] = XPLMFindCommand("AirbusFBW/MCDU2Key7"      )) ||
                    NULL == (ctx->a319kc.c[0][18] = XPLMFindCommand("AirbusFBW/MCDU1Key8"      )) ||
                    NULL == (ctx->a319kc.c[1][18] = XPLMFindCommand("AirbusFBW/MCDU2Key8"      )) ||
                    NULL == (ctx->a319kc.c[0][19] = XPLMFindCommand("AirbusFBW/MCDU1Key9"      )) ||
                    NULL == (ctx->a319kc.c[1][19] = XPLMFindCommand("AirbusFBW/MCDU2Key9"      )) ||
                    NULL == (ctx->a319kc.c[0][20] = XPLMFindCommand("AirbusFBW/MCDU1Key0"      )) ||
                    NULL == (ctx->a319kc.c[1][20] = XPLMFindCommand("AirbusFBW/MCDU2Key0"      )) ||
                    NULL == (ctx->a319kc.c[0][21] = XPLMFindCommand("AirbusFBW/MCDU1KeyA"      )) ||
                    NULL == (ctx->a319kc.c[1][21] = XPLMFindCommand("AirbusFBW/MCDU2KeyA"      )) ||
                    NULL == (ctx->a319kc.c[0][22] = XPLMFindCommand("AirbusFBW/MCDU1KeyB"      )) ||
                    NULL == (ctx->a319kc.c[1][22] = XPLMFindCommand("AirbusFBW/MCDU2KeyB"      )) ||
                    NULL == (ctx->a319kc.c[0][23] = XPLMFindCommand("AirbusFBW/MCDU1KeyC"      )) ||
                    NULL == (ctx->a319kc.c[1][23] = XPLMFindCommand("AirbusFBW/MCDU2KeyC"      )) ||
                    NULL == (ctx->a319kc.c[0][24] = XPLMFindCommand("AirbusFBW/MCDU1KeyD"      )) ||
                    NULL == (ctx->a319kc.c[1][24] = XPLMFindCommand("AirbusFBW/MCDU2KeyD"      )) ||
                    NULL == (ctx->a319kc.c[0][25] = XPLMFindCommand("AirbusFBW/MCDU1KeyE"      )) ||
                    NULL == (ctx->a319kc.c[1][25] = XPLMFindCommand("AirbusFBW/MCDU2KeyE"      )) ||
                    NULL == (ctx->a319kc.c[0][26] = XPLMFindCommand("AirbusFBW/MCDU1KeyF"      )) ||
                    NULL == (ctx->a319kc.c[1][26] = XPLMFindCommand("AirbusFBW/MCDU2KeyF"      )) ||
                    NULL == (ctx->a319kc.c[0][27] = XPLMFindCommand("AirbusFBW/MCDU1KeyG"      )) ||
                    NULL == (ctx->a319kc.c[1][27] = XPLMFindCommand("AirbusFBW/MCDU2KeyG"      )) ||
                    NULL == (ctx->a319kc.c[0][28] = XPLMFindCommand("AirbusFBW/MCDU1KeyH"      )) ||
                    NULL == (ctx->a319kc.c[1][28] = XPLMFindCommand("AirbusFBW/MCDU2KeyH"      )) ||
                    NULL == (ctx->a319kc.c[0][29] = XPLMFindCommand("AirbusFBW/MCDU1KeyI"      )) ||
                    NULL == (ctx->a319kc.c[1][29] = XPLMFindCommand("AirbusFBW/MCDU2KeyI"      )) ||
                    NULL == (ctx->a319kc.c[0][30] = XPLMFindCommand("AirbusFBW/MCDU1KeyJ"      )) ||
                    NULL == (ctx->a319kc.c[1][30] = XPLMFindCommand("AirbusFBW/MCDU2KeyJ"      )) ||
                    NULL == (ctx->a319kc.c[0][31] = XPLMFindCommand("AirbusFBW/MCDU1KeyK"      )) ||
                    NULL == (ctx->a319kc.c[1][31] = XPLMFindCommand("AirbusFBW/MCDU2KeyK"      )) ||
                    NULL == (ctx->a319kc.c[0][32] = XPLMFindCommand("AirbusFBW/MCDU1KeyL"      )) ||
                    NULL == (ctx->a319kc.c[1][32] = XPLMFindCommand("AirbusFBW/MCDU2KeyL"      )) ||
                    NULL == (ctx->a319kc.c[0][33] = XPLMFindCommand("AirbusFBW/MCDU1KeyM"      )) ||
                    NULL == (ctx->a319kc.c[1][33] = XPLMFindCommand("AirbusFBW/MCDU2KeyM"      )) ||
                    NULL == (ctx->a319kc.c[0][34] = XPLMFindCommand("AirbusFBW/MCDU1KeyN"      )) ||
                    NULL == (ctx->a319kc.c[1][34] = XPLMFindCommand("AirbusFBW/MCDU2KeyN"      )) ||
                    NULL == (ctx->a319kc.c[0][35] = XPLMFindCommand("AirbusFBW/MCDU1KeyO"      )) ||
                    NULL == (ctx->a319kc.c[1][35] = XPLMFindCommand("AirbusFBW/MCDU2KeyO"      )) ||
                    NULL == (ctx->a319kc.c[0][36] = XPLMFindCommand("AirbusFBW/MCDU1KeyP"      )) ||
                    NULL == (ctx->a319kc.c[1][36] = XPLMFindCommand("AirbusFBW/MCDU2KeyP"      )) ||
                    NULL == (ctx->a319kc.c[0][37] = XPLMFindCommand("AirbusFBW/MCDU1KeyQ"      )) ||
                    NULL == (ctx->a319kc.c[1][37] = XPLMFindCommand("AirbusFBW/MCDU2KeyQ"      )) ||
                    NULL == (ctx->a319kc.c[0][38] = XPLMFindCommand("AirbusFBW/MCDU1KeyR"      )) ||
                    NULL == (ctx->a319kc.c[1][38] = XPLMFindCommand("AirbusFBW/MCDU2KeyR"      )) ||
                    NULL == (ctx->a319kc.c[0][39] = XPLMFindCommand("AirbusFBW/MCDU1KeyS"      )) ||
                    NULL == (ctx->a319kc.c[1][39] = XPLMFindCommand("AirbusFBW/MCDU2KeyS"      )) ||
                    NULL == (ctx->a319kc.c[0][40] = XPLMFindCommand("AirbusFBW/MCDU1KeyT"      )) ||
                    NULL == (ctx->a319kc.c[1][40] = XPLMFindCommand("AirbusFBW/MCDU2KeyT"      )) ||
                    NULL == (ctx->a319kc.c[0][41] = XPLMFindCommand("AirbusFBW/MCDU1KeyU"      )) ||
                    NULL == (ctx->a319kc.c[1][41] = XPLMFindCommand("AirbusFBW/MCDU2KeyU"      )) ||
                    NULL == (ctx->a319kc.c[0][42] = XPLMFindCommand("AirbusFBW/MCDU1KeyV"      )) ||
                    NULL == (ctx->a319kc.c[1][42] = XPLMFindCommand("AirbusFBW/MCDU2KeyV"      )) ||
                    NULL == (ctx->a319kc.c[0][43] = XPLMFindCommand("AirbusFBW/MCDU1KeyW"      )) ||
                    NULL == (ctx->a319kc.c[1][43] = XPLMFindCommand("AirbusFBW/MCDU2KeyW"      )) ||
                    NULL == (ctx->a319kc.c[0][44] = XPLMFindCommand("AirbusFBW/MCDU1KeyX"      )) ||
                    NULL == (ctx->a319kc.c[1][44] = XPLMFindCommand("AirbusFBW/MCDU2KeyX"      )) ||
                    NULL == (ctx->a319kc.c[0][45] = XPLMFindCommand("AirbusFBW/MCDU1KeyY"      )) ||
                    NULL == (ctx->a319kc.c[1][45] = XPLMFindCommand("AirbusFBW/MCDU2KeyY"      )) ||
                    NULL == (ctx->a319kc.c[0][46] = XPLMFindCommand("AirbusFBW/MCDU1KeyZ"      )) ||
                    NULL == (ctx->a319kc.c[1][46] = XPLMFindCommand("AirbusFBW/MCDU2KeyZ"      )))
                {
                    ndt_log("navP [warning]: failed to find AirbusFBW commands for key sniffer\n");
                    break;
                }
                else
                {
                    ctx->a319kc.c[2][44] = NULL;
                    ctx->a319kc.c[2][47] = NULL;
                }
                if (NULL == (ctx->a319kc.c[2][39] = XPLMFindCommand("navP/switches/cdu_toggle")) ||
                    NULL == (ctx->a319kc.c[2][40] = ctx->a319kc.c[2][39]))
                {
                    ndt_log("navP [warning]: failed to find MCDU toggle for key sniffer\n");
                }
#ifndef NAVP_ONLY
                if (NULL == (ctx->a319kc.c[2][39] = XPLMFindCommand("YFMS/toggle")))
                {
                    ndt_log("navP [warning]: failed to find YFMS toggle for key sniffer\n");
                }
#endif
#if TIM_ONLY
                if (NULL == (ctx->a319kc.c[2][42] = XPLMFindCommand("navP/spoilers/extend"          )) ||
                    NULL == (ctx->a319kc.c[2][43] = XPLMFindCommand("navP/spoilers/retract"         )) ||
                    NULL == (ctx->a319kc.c[2][41] = XPLMFindCommand("xnz/brakes/regular/hold"       )) ||
                    NULL == (ctx->a319kc.c[2][45] = XPLMFindCommand("sim/flight_controls/flaps_up"  )) ||
                    NULL == (ctx->a319kc.c[2][46] = XPLMFindCommand("sim/flight_controls/flaps_down")))
                {
                    ndt_log("navP [warning]: failed to find TIM_ONLY commands for key sniffer\n");
                }
                if (XPLM_NO_PLUGIN_ID != ctx->coatc.pe_plid)
                {
                    ctx->a319kc.c[2][47] = XPLMFindCommand("sim/operation/contact_atc");
                }
                else if (XPLM_NO_PLUGIN_ID != ctx->coatc.xb_plid)
                {
                    ctx->a319kc.c[2][44] = XPLMFindCommand("xsquawkbox/command/start_text_entry");
                    ctx->a319kc.c[2][47] = XPLMFindCommand("xsquawkbox/voice/ptt");
                }
#endif
                if ((ctx->a319kc.kc_is_registered = XPLMRegisterKeySniffer(&tol_keysniffer, 1/*inBeforeWindows*/, &ctx->a319kc)) != 1)
                {
                    ndt_log("navP [warning]: failed to register key sniffer for AirbusFBW\n");
                    break;
                }
                ndt_log("navP [info]: AirbusFBW key sniffer registered\n");
                break;
            }
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            break;

        case ACF_TYP_A350_FF:
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_copilot");  // VOR1 on ND2 off
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_copilot");  // VOR2 on ND2 off
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_1_selection_pilot");    // VOR1 on ND1 off
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_2_selection_pilot");    // VOR2 on ND1 off
            _DO(1, XPLMSetDatai,      0, "1-sim/fcu/navL/flag");                    // sync with above
            _DO(1, XPLMSetDatai,      0, "1-sim/fcu/navL2/flag");                   // sync with above
            _DO(1, XPLMSetDatai,      0, "1-sim/fcu/navR/flag");                    // sync with above
            _DO(1, XPLMSetDatai,      0, "1-sim/fcu/navR2/flag");                   // sync with above
            _DO(1, XPLMSetDatai,      0, "1-sim/fcu/altModeSwitch");                // FCU alt. sel. incre. (100ft)
            _DO(1, XPLMSetDataf,   2.0f, "1-sim/fcu/ndModeLeft/switch");            // ND m. sel. (cap. side) (nav)
            _DO(1, XPLMSetDataf,   2.0f, "1-sim/fcu/ndModeRight/switch");           // ND m. sel. (f/o. side) (nav)
            _DO(1, XPLMSetDataf,   2.0f, "1-sim/fcu/ndZoomLeft/switch");            // ND r. sel. (cap. side) ( 40)
            _DO(1, XPLMSetDataf,   2.0f, "1-sim/fcu/ndZoomRight/switch");           // ND r. sel. (f/o. side) ( 40)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/NDShowARPTCapt");               // ND --APT-- (cap. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/NDShowARPTFO");                 // ND --APT-- (f/o. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/NDShowCSTRCapt");               // ND --CST-- (cap. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/NDShowCSTRFO");                 // ND --CST-- (f/o. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/NDShowNDBCapt");                // ND --NDB-- (cap. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/NDShowNDBFO");                  // ND --NDB-- (f/o. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/NDShowVORDCapt");               // ND --VOR-- (cap. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/NDShowVORDFO");                 // ND --VOR-- (f/o. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/NDShowWPTCapt");                // ND --WPT-- (cap. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/NDShowWPTFO");                  // ND --WPT-- (f/o. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/TCASSelectedND1");              // ND traffic (cap. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/TCASSelectedND2");              // ND traffic (f/o. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/TerrainSelectedND1");           // ND terrain (cap. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/TerrainSelectedND2");           // ND terrain (f/o. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/WXSelectedND1");                // ND weather (cap. side) (off)
            _DO(1, XPLMSetDatai,      0, "AirbusFBW/WXSelectedND2");                // ND weather (f/o. side) (off)
            _DO(1, XPLMSetDatai,      1, "1-sim/radio/button/1/16");                // RMP1: microph. on VHF1 (on)
            _DO(1, XPLMSetDatai,      1, "1-sim/radio/push/1/1");                   // RMP1: receiver on VHF1 (on)
            _DO(1, XPLMSetDataf, 270.0f, "1-sim/radio/1/1/rotary");                 // RMP1: receiver on VHF1 (volume)
            _DO(1, XPLMSetDatai,      1, "1-sim/radio/push/1/10");                  // RMP1: c. crew intercom (on)
            _DO(1, XPLMSetDataf, 270.0f, "1-sim/radio/1/10/rotary");                // RMP1: c. crew intercom (volume)
            _DO(1, XPLMSetDatai,      1, "1-sim/radio/push/1/11");                  // RMP1: p. announcements (on)
            _DO(1, XPLMSetDataf, 270.0f, "1-sim/radio/1/11/rotary");                // RMP1: p. announcements (volume)
            _DO(1, XPLMSetDatai,      1, "1-sim/1/switch");                         // Evac. panel: selector  (capt&purs)
            _DO(1, XPLMSetDatai,      1, "1-sim/37/switch");                        // Braking: anti-skid sw. (on)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/15/button");                        // Cabin panel: wireless  (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/16/button");                        // Cabin panel: pas. data (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/17/button");                        // Cabin panel: sat. com. (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/18/button");                        // Cabin panel: lan. cam. (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/19/button");                        // Cabin panel: FAR4 sys. (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/20/button");                        // Cabin panel: mob. com. (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/21/button");                        // Cabin panel: IFEC sys. (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/22/button");                        // Cabin panel: DER  sys. (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/38/button");                        // Oxyg. panel: crew sup. (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/42/button");                        // Vent. panel: cooling   (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/44/button");                        // Vent. panel: c. fans 1 (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/45/button");                        // Vent. panel: c. fans 2 (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/53/button");                        // NSS: data to avionics  (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/54/button");                        // NSS: cabin data to NSS (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/230/button");                       // NSS: gatelink          (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/55/button");                        // Cargo: A/C I. v. (fwd) (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/56/button");                        // Cargo: A/C I. v. (aft) (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/57/button");                        // Cargo: A/C I. v. (blk) (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/59/button");                        // Cargo: A/C heating sw. (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/65/button");                        // Hydraulics: YELL. p. 1 (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/67/button");                        // Hydraulics: GREEN p. 1 (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/75/button");                        // Hydraulics: YELL. p. 2 (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/77/button");                        // Hydraulics: GREEN p. 2 (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/84/button");                        // Fuel: C tank feed mode (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/96/button");                        // Elec. panel: AC bus 1A (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/99/button");                        // Elec. panel: AC bus 1B (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/100/button");                       // Elec. panel: AC bus t. (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/101/button");                       // Elec. panel: APU (gen) (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/103/button");                       // Elec. panel: AC bus 2B (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/106/button");                       // Elec. panel: AC bus 2A (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/109/button");                       // Air panel: bleed en. 1 (auto)
            _DO(1, XPLMSetDataf,   1.0f, "1-sim/115/button");                       // Air panel: bleed en. 2 (auto)
            _DO(1, XPLMSetDatai,      1, "1-sim/air/airFlowSwitch");                // Air panel: flow selec. (norm)
            _DO(1, XPLMSetDatai,      1, "1-sim/air/crossBeedSwitch");              // Air panel: c.b. selec. (auto)
            _DO(1, XPLMSetDataf,   0.4f, "1-sim/air/ckptSettingRotery");            // Air panel: cock. temp. (purser)
            _DO(1, XPLMSetDataf,   0.4f, "1-sim/air/cabinSettingRotery");           // Air panel: cabin temp. (purser)
            _DO(1, XPLMSetDatai,  18000, "1-sim/options/Transition Altitude");      // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/autoreverse");              // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/Auto Helper");              // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/Auto Pause");               // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/Real Limits");              // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/Baro Units");               // preferred default settings
            _DO(1, XPLMSetDatai,      1, "1-sim/options/Difficulty");               // preferred default settings
            _DO(1, XPLMSetDatai,      1, "1-sim/options/Draw Lines");               // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/ScreenGlow");               // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/ILS Align");                // preferred default settings
            _DO(1, XPLMSetDatai,      1, "1-sim/options/Flashing");                 // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/Pilots");                   // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/Popups");                   // preferred default settings
            _DO(1, XPLMSetDatai,      1, "1-sim/options/Wheel");                    // preferred default settings
            _DO(1, XPLMSetDatai,      1, "1-sim/options/Font");                     // preferred default settings
            _DO(1, XPLMSetDatai,      1, "1-sim/options/Time");                     // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/XFMC");                     // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/side");                     // preferred default settings
            _DO(0, XPLMSetDatai,      1, "AirbusFBW/PopupType");                    // default settings (v1.6.x+)
            _DO(1, XPLMSetDataf,   0.0f, "1-sim/fms/perf/toCG");                    // 1.4.8: reset perf settings
            _DO(1, XPLMSetDatai,      0, "1-sim/fms/init/crzFL");                   // 1.4.8: reset perf settings
            _DO(1, XPLMSetDatai,      2, "1-sim/fms/perf/toThrust");                // 1.4.8: reset perf settings
            _DO(1, XPLMSetDatai,     30, "1-sim/fms/perf/flexTemp"); /* 100.0% N1*/ // 1.4.8: reset perf settings
            _DO(1, XPLMSetDatai,      2, "1-sim/fms/perf/landConfig");              // 1.4.8: reset perf settings
            _DO(1, XPLMSetDatai,      1, "1-sim/fms/perf/toFlapSetting");           // 1.4.8: reset perf settings
            _DO(1, XPLMSetDataf,   0.0f, "airbus_qpac/performance/V1");             // 1.4.8: reset perf settings
            _DO(1, XPLMSetDataf,   0.0f, "airbus_qpac/performance/V2");             // 1.4.8: reset perf settings
            _DO(1, XPLMSetDataf,   0.0f, "airbus_qpac/performance/VR");             // 1.4.8: reset perf settings
            _DO(1, XPLMSetDataf,   0.0f, "sim/cockpit/misc/radio_altimeter_minimum"); // 4.8: reset perf settings
            if ((d_ref = XPLMFindDataRef("sim/flightmodel/position/elevation")))    // 1.4.8: reset perf settings
            {
                int feet_msl = 10 * roundf(XPLMGetDataf(d_ref) / 3.048f);
                _DO(1, XPLMSetDatai, 1500 + feet_msl, "1-sim/fms/perf/toAccAltitude");
                _DO(1, XPLMSetDatai, 1500 + feet_msl, "1-sim/fms/perf/thrReductionAlt");
                _DO(1, XPLMSetDatai, 1500 + feet_msl, "1-sim/fms/perf/engOutToAccAltitude");
            }
            if (acf_type_is_engine_running() == 0) // cold & dark
            {
                _DO(1, XPLMSetDatai,  1, "1-sim/ext/fuel_truck");                   // minimal ground config
                _DO(1, XPLMSetDatai,  1, "1-sim/ext/stairs");                       // minimal ground config
                _DO(1, XPLMSetDatai,  0, "1-sim/ext/clean");                        // minimal ground config
                _DO(1, XPLMSetDatai,  0, "1-sim/ext/train");                        // minimal ground config
                _DO(1, XPLMSetDatai,  0, "1-sim/ext/gate");                         // minimal ground config
                _DO(1, XPLMSetDatai,  1, "1-sim/ext/gpu1");                         // minimal ground config
                _DO(1, XPLMSetDatai,  1, "1-sim/ext/gpu2");                         // minimal ground config
                _DO(1, XPLMSetDatai,  1, "1-sim/ext/stop");                         // minimal ground config
                _DO(1, XPLMSetDatai,  0, "1-sim/ext/acu");                          // minimal ground config
                _DO(1, XPLMSetDatai,  1, "1-sim/ext/bus");                          // minimal ground config
                _DO(1, XPLMSetDatai,  0, "1-sim/ext/gau");                          // minimal ground config
                _DO(1, XPLMSetDatai,  1, "1-sim/ext/LSU");                          // minimal ground config
                if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/custom_slider_on")))
                {
                    int door_open[1] = { 1, };
                    XPLMSetDatavi(d_ref, &door_open[0], 0, 1);                      // passeng. 1L
                    XPLMSetDatavi(d_ref, &door_open[0], 2, 1);                      // passeng. 2L
                    XPLMSetDatavi(d_ref, &door_open[0], 8, 1);                      // luggage FWD
                    XPLMSetDatavi(d_ref, &door_open[0], 9, 1);                      // luggage AFT
                }
                float load = 1599.0f, fuel = 8400.0f;
                acf_type_load_set (ctx->info, &load);
                acf_type_fuel_set (ctx->info, &fuel);
            }
            if (ctx->a350kc.kc_is_registered)
            {
                ndt_log("navP [warning]: AirbusFBW key sniffer already registered\n");
                break;
            }
            if (NULL == (ctx->a350kc.datar[0] = XPLMFindDataRef("sim/graphics/view/view_is_external")))
            {
                ndt_log("navP [warning]: failed to find AirbusFBW datarefs for key sniffer\n");
                break;
            }
            if (NULL == (ctx->a350kc.c[0] = XPLMFindCommand("navP/switches/cdu_toggle")))
            {
                ndt_log("navP [warning]: failed to find MCDU toggle for key sniffer\n");
            }
#ifndef NAVP_ONLY
            if (NULL == (ctx->a350kc.c[1] = XPLMFindCommand("YFMS/toggle")))
            {
                ndt_log("navP [warning]: failed to find YFMS toggle for key sniffer\n");
            }
#endif
#if TIM_ONLY
            if (NULL == (ctx->a350kc.c[4] = XPLMFindCommand("navP/spoilers/extend"          )) ||
                NULL == (ctx->a350kc.c[5] = XPLMFindCommand("navP/spoilers/retract"         )) ||
                NULL == (ctx->a350kc.c[6] = XPLMFindCommand("sim/flight_controls/flaps_up"  )) ||
                NULL == (ctx->a350kc.c[7] = XPLMFindCommand("sim/flight_controls/flaps_down")))
            {
                ndt_log("navP [warning]: failed to find TIM_ONLY commands for key sniffer\n");
            }
            if (XPLM_NO_PLUGIN_ID != ctx->coatc.pe_plid)
            {
                ctx->a350kc.c[2] = XPLMFindCommand("sim/operation/contact_atc");
            }
            else if (XPLM_NO_PLUGIN_ID != ctx->coatc.xb_plid)
            {
                ctx->a350kc.c[2] = XPLMFindCommand("sim/operation/contact_atc");
                ctx->a350kc.c[3] = XPLMFindCommand("xsquawkbox/command/start_text_entry");
            }
#endif
            if ((ctx->a350kc.kc_is_registered = XPLMRegisterKeySniffer(&a35_keysniffer, 1/*inBeforeWindows*/, &ctx->a350kc)) != 1)
            {
                ndt_log("navP [warning]: failed to register key sniffer for AirbusFBW\n");
                break;
            }
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            ndt_log("navP [info]: AirbusFBW key sniffer registered\n");
            break;

        case ACF_TYP_B737_EA:
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            _DO(1, XPLMSetDatai, 0, "x737/cockpit/yoke/captYokeVisible");
            _DO(1, XPLMSetDatai, 0, "x737/cockpit/yoke/foYokeVisible");
            break;

        case ACF_TYP_B737_XG:
            _DO(1, XPLMSetDataf,   0.0f, "ixeg/733/aircond/aircond_cont_cabin_sel_act");    // Cont. cab. air temper. (normal)
            _DO(1, XPLMSetDataf,   0.0f, "ixeg/733/aircond/aircond_pass_cabin_sel_act");    // Pass. cab. air temper. (normal)
            _DO(1, XPLMSetDataf,   1.0f, "ixeg/733/bleedair/bleedair_recirc_fan_act");      // Bleed air recirc. fans (auto)
            _DO(1, XPLMSetDataf,   3.0f, "ixeg/733/ehsi/ehsi_mode_pt_act");                 // HSI m.sel. (cap. side) (ctr)
            _DO(1, XPLMSetDataf,   3.0f, "ixeg/733/ehsi/ehsi_mode_cpt_act");                // HSI m.sel. (f/o. side) (ctr)
            _DO(1, XPLMSetDataf,   2.0f, "ixeg/733/ehsi/ehsi_range_pt_act");                // HSI r.sel. (cap. side) ( 40)
            _DO(1, XPLMSetDataf,   2.0f, "ixeg/733/ehsi/ehsi_range_cpt_act");               // HSI r.sel. (f/o. side) ( 40)
            _DO(1, XPLMSetDataf,   0.0f, "ixeg/733/rheostats/light_breakers_act");          // Circuit breakers light (off)
            _DO(1, XPLMSetDataf,   0.8f, "ixeg/733/rheostats/light_pedpanel_act");          // Panel light (pedestal) (daylight)
            _DO(1, XPLMSetDataf,   0.8f, "ixeg/733/rheostats/light_overhead_act");          // Panel light (overhead) (daylight)
            _DO(1, XPLMSetDataf,   .21f, "ixeg/733/rheostats/light_pedflood_act");          // Flood light (pedestal) (daylight)
            _DO(1, XPLMSetDataf,   0.2f, "ixeg/733/rheostats/light_afds_act");              // Flood light (A.F.D.S.) (daylight)
            _DO(1, XPLMSetDataf,   0.8f, "ixeg/733/rheostats/light_fmc_pt_act");            // MCDU: lig. (cap. side) (daylight)
            _DO(1, XPLMSetDataf,   0.8f, "ixeg/733/rheostats/light_fmc_cpt_act");           // MCDU: lig. (f/o. side) (daylight)
            _DO(1, XPLMSetDataf,   0.0f, "ixeg/733/rheostats/light_mapbr_pt_act");          // Maps: lig. (cap. side) (off)
            _DO(1, XPLMSetDataf,   0.0f, "ixeg/733/rheostats/light_mapbr_cpt_act");         // Maps: lig. (f/o. side) (off)
            _DO(1, XPLMSetDataf,   1.5f, "ixeg/733/audio/audio_recv_vhf1_pt_act");          // setup radios for online ATC
            _DO(1, XPLMSetDataf,   1.0f, "ixeg/733/audio/audio_recp_vhf1_pt_act");          // setup radios for online ATC
            _DO(1, XPLMSetDataf,   1.0f, "ixeg/733/audio/audio_mic_vhf1_pt_act");           // setup radios for online ATC
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/instrument_brightness_ratio")))
            {
                float instrument_brightness_ratio[1] = { 0.8f, };
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0], 0, 1);                // Panel (cap. side): daylight
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0], 1, 1);                // Panel (f/o. side): daylight
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0], 2, 1);                // Panel (backgrnd.): neon off
            }
            if ((d_ref = XPLMFindDataRef("ixeg/733/misc/smoking_act")))
            {
                if (1.0f < XPLMGetDataf(d_ref))
                {
                    XPLMSetDataf(d_ref, 1.0f); // ON -> AUTO silent even w/power
                }
            }
            if (acf_type_is_engine_running() == 0)
            {
                float load = 669.0f, fuel = 3350.0f;
                acf_type_load_set(ctx->info, &load);
                acf_type_fuel_set(ctx->info, &fuel);
            }
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            break;

        case ACF_TYP_B757_FF:
        case ACF_TYP_B767_FF:
            // the following two are special, the buttons auto-revert to zero;
            // thankfully they always work (even without any electrical power)
            if ((d_ref = XPLMFindDataRef("1-sim/vor1/isAuto")) &&
                (0000 == XPLMGetDatai(d_ref)))
            {
                _DO(1, XPLMSetDatai,1, "anim/78/button"); // VOR 1: manual -> auto
            }
            if ((d_ref = XPLMFindDataRef("1-sim/vor2/isAuto")) &&
                (0000 == XPLMGetDatai(d_ref)))
            {
                _DO(1, XPLMSetDatai,1, "anim/79/button"); // VOR 2: manual -> auto
            }
            _DO(0, XPLMSetDataf,360.0f,"sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot"); // VOR 1: course selector: 360
            _DO(1, XPLMSetDataf,360.0f,"1-sim/radios/nav3_obs_deg_mag_pilot");                  // VOR 2: course selector: 360
            _DO(1, XPLMSetDataf,  .85f,"1-sim/adf/leftBigRotary");                  // ADF 1: 3 // LSGG GLA (Gland)
            _DO(1, XPLMSetDataf,  .20f,"1-sim/adf/leftMidRotary");                  // ADF 1: 7
            _DO(1, XPLMSetDataf,  .40f,"1-sim/adf/leftSmallRotary");                // ADF 1: 5
            _DO(1, XPLMSetDataf,  .10f,"1-sim/adf/rightBigRotary");                 // ADF 2: 2 // KJFK BBN (Babylon)
            _DO(1, XPLMSetDataf,  .80f,"1-sim/adf/rightMidRotary");                 // ADF 2: 7
            _DO(1, XPLMSetDataf,  .60f,"1-sim/adf/rightSmallRotary");               // ADF 2: 5
//          _DO(1, XPLMSetDatai,    1, "anim/75/button");                           // Terrain override switch (on)
//          _DO(1, XPLMSetDataf, 1.0f, "1-sim/gauges/terrOVRDcover");               // Terrain override switch (lift cover)
//          _DO(1, XPLMSetDataf, 1.0f, "1-sim/electrical/batteryCover");            // Battery on/off selector (lift cover)
//          _DO(1, XPLMSetDataf, 1.0f, "1-sim/elecengcont/leftCover");              // Engine elec. contr. (L) (lift cover)
//          _DO(1, XPLMSetDataf, 1.0f, "1-sim/elecengcont/rightCover");             // Engine elec. contr. (R) (lift cover)
            _DO(1, XPLMSetDatai,    1, "anim/3/button");                            // Engine elec. contr. (L) (on)
            _DO(1, XPLMSetDatai,    1, "anim/4/button");                            // Engine elec. contr. (R) (on)
            _DO(1, XPLMSetDatai,    1, "anim/8/button");                            // Engine-dr. hy. pump (L) (on)
            _DO(1, XPLMSetDatai,    1, "anim/11/button");                           // Engine-dr. hy. pump (R) (on)
            _DO(1, XPLMSetDatai,    1, "anim/59/button");                           // Air: L bleed ISLN valve (on/open)
            _DO(1, XPLMSetDatai,    1, "anim/87/button");                           // Air: R bleed ISLN valve (on/open)
            _DO(1, XPLMSetDatai,    1, "anim/90/button");                           // Air: C bleed ISLN valve (on/open)
            _DO(1, XPLMSetDatai,    1, "anim/60/button");                           // Air: L e. bleed air sw. (on)
            _DO(1, XPLMSetDatai,    1, "anim/62/button");                           // Air: R e. bleed air sw. (on)
            _DO(1, XPLMSetDataf, 0.5f, "1-sim/cond/fwdTempControl");                // Temp. control (fwd ca.) (auto)
            _DO(1, XPLMSetDataf, 0.5f, "1-sim/cond/midTempControl");                // Temp. control (mid ca.) (auto)
            _DO(1, XPLMSetDataf, 0.5f, "1-sim/cond/aftTempControl");                // Temp. control (aft ca.) (auto)
            _DO(1, XPLMSetDataf, 0.5f, "1-sim/cond/fltdkTempControl");              // Temp. control (f. deck) (auto)
            _DO(1, XPLMSetDatai,    0, "params/autosave");                          // DO THIS FIRST prior any changes
            _DO(1, XPLMSetDataf, 0.1f, "params/sound/avionics");                    // sounds: turn down whiny avionics
            _DO(1, XPLMSetDataf, 1.0f, "params/sound/pa");                          // sounds: hear PA from flight deck
            _DO(1, XPLMSetDataf, 0.0f, "params/sound/cc");                          // cabin crew: Daniel OFF (yaaaay!)
            _DO(1, XPLMSetDataf, 0.0f, "params/reflectLevelInstruments");           // custom reflections OFF
            _DO(1, XPLMSetDataf, 0.0f, "params/reflectLevelWindows");               // custom reflections OFF
            _DO(1, XPLMSetDataf, 0.0f, "params/reflectLevel");                      // custom reflections OFF
            _DO(1, XPLMSetDataf, 0.0f, "params/effectLevelGlowScreen");             // custom screen glow OFF
            _DO(1, XPLMSetDataf, 0.0f, "params/effectLevelGlowCDU");                // custom screen glow OFF
            _DO(1, XPLMSetDataf, 0.0f, "params/effectLevelBlick");                  // custom screen glow OFF
            _DO(1, XPLMSetDataf, 0.0f, "params/effectLevel");                       // custom screen glow OFF
            _DO(1, XPLMSetDataf, 0.1f, "params/wingflexLevel");                     // custom "wing flex" LOW
            _DO(1, XPLMSetDatai,    0, "params/constrol");                          // F/O. is in control OFF
            _DO(1, XPLMSetDatai,    0, "params/shakeOnTurb");                       // shake in turb. air OFF
            _DO(1, XPLMSetDatai,    0, "params/customEffects");                     // custom ef. control OFF
            _DO(1, XPLMSetDatai,    0, "params/windshearType");                     // advanced windshear OFF
            _DO(1, XPLMSetDatai,    0, "params/dynamicBlinds");                     // dyn. window blinds OFF
            _DO(1, XPLMSetDatai,    0, "params/autoGearLever");                     // automatic g. lever OFF
            _DO(1, XPLMSetDatai,    0, "params/last_position");                     // inter-flight data  OFF
            _DO(1, XPLMSetDatai,    0, "params/realism_level");                     // "challenge" level  OFF
            _DO(1, XPLMSetDatai,    1, "params/betterpushback");                    // use BetterPushback ON
            _DO(1, XPLMSetDatai,    1, "params/throttleblock");                     // throttle level bl. ON
            _DO(1, XPLMSetDatai,    1, "params/real_limits");                       // real aircr. limits ON
            _DO(1, XPLMSetDatai,    1, "params/real_time");                         // real aircr. timing ON
            _DO(1, XPLMSetDatai,    1, "params/realSound");                         // real sound config. ON
            _DO(1, XPLMSetDatai,    1, "params/yokehide");                          // hide CA & FO yokes ON
            _DO(1, XPLMSetDatai,    1, "params/wxrType");                           // real weather radar ON
            _DO(1, XPLMSetDatai,    1, "params/metric");                            // metric measurement ON
            _DO(1, XPLMSetDatai,    1, "params/wheel");                             // scr. wheel support ON
            _DO(0, XPLMSetDatai,    1, "params/Khz833");                            // 8.33kHz radios (B757)
            if (acf_type_is_engine_running() == 0) // cold & dark
            {
                _DO(1,XPLMSetDatai, 0, "params/ground_start_unit");                 // minimal ground config
                _DO(1,XPLMSetDatai, 1, "params/fuel_truck");                        // minimal ground config
                _DO(1,XPLMSetDatai, 1, "params/stairs");                            // minimal ground config
                _DO(1,XPLMSetDatai, 1, "params/stop");                              // minimal ground config
                _DO(1,XPLMSetDatai, 1, "params/bus");                               // minimal ground config
                _DO(1,XPLMSetDatai, 1, "params/gpu");                               // minimal ground config
                _DO(1,XPLMSetDatai, 0, "params/ACU");                               // minimal ground config
                _DO(1,XPLMSetDatai, 0, "params/LSU");                               // minimal ground config
                _DO(1,XPLMSetDatai, 0, "params/gate");                              // minimal ground config
                _DO(1,XPLMSetDatai, 0, "params/cover");                             // minimal ground config
                _DO(1,XPLMSetDatai, 0, "params/deice");                             // minimal ground config
            }
            _DO(1, XPLMSetDatai,    1, "757Avionics/options/AP/attHldAtApEngagement");  // avionics: ATT on CMD engage     (yes)
            _DO(1, XPLMSetDatai,    0, "757Avionics/options/AP/gsCaptBeforeLocCapt");   // avionics: GS before LOC         (N/A)
            _DO(1, XPLMSetDatai,    1, "757Avionics/options/AP/manualCmdSelForApp");    // avionics: CMD manual arm        (yes)
            _DO(1, XPLMSetDatai,    1, "757Avionics/options/ND/DigitWindBearing");      // avionics: Digital Wind Bearing  (yes)
            _DO(1, XPLMSetDatai,    0, "757Avionics/options/PFD/roundDialRa");          // avionics: Round Dial RA         (N/A)
            _DO(1, XPLMSetDatai,    1, "757Avionics/options/ND/advRangeArcs");          // avionics: Range Arcs            (yes)
            _DO(1, XPLMSetDatai,    0, "757Avionics/options/ND/adfPointers");           // avionics: ADF Pointers          (N/A)
            _DO(1, XPLMSetDatai,    0, "757Avionics/options/PFD/RisingRwy");            // avionics: Rising Runway         (N/A)
            _DO(1, XPLMSetDatai,    0, "757Avionics/options/ND/hdgUpMap");              // avionics: Heading Up Map        (N/A)
            _DO(1, XPLMSetDatai,    0, "757Avionics/options/PFD/FdType");               // avionics: Integrated Cue FD     (off)
            _DO(1, XPLMSetDatai,    1, "757Avionics/options/FMS/isGPS");                // avionics: GPS EQUIPPED          (yes)
            _DO(1, XPLMSetDatai,    1, "757Avionics/fms/type");                         // avionics: PIP FMC               (yes)
//          _DO(1, XPLMSetDatai,    1, "757Avionics/engine");                           // needs setting non-dataref variable(s)
            _DO(1, XPLMSetDatai,    1, "WINGLETS/WINGLETS");                            // airplane: winglets              (yes)
            if (ctx->info->ac_type == ACF_TYP_B757_FF)
            {
                if ((acf_type_is_engine_running() == 0) && // cold & dark
                    (d_ref = XPLMFindDataRef("sim/cockpit2/switches/custom_slider_on")))
                {
                    int door_open[1] = { 1, };
                    XPLMSetDatavi(d_ref, &door_open[0], 0, 1); // LF
                }
                _DO(1, XPLMSetDatai, 1, "1-sim/ndpanel/1/hsiModeButton");               // ND m. sel. (cap. side) (ctr)
                _DO(1, XPLMSetDatai, 2, "1-sim/ndpanel/1/hsiModeRotary");               // ND m. sel. (cap. side) (map)
                _DO(1, XPLMSetDatai, 2, "1-sim/ndpanel/1/hsiRangeRotary");              // ND r. sel. (cap. side) ( 40)
                _DO(1, XPLMSetDatai, 1, "1-sim/ndpanel/2/hsiModeButton");               // ND m. sel. (f/o. side) (ctr)
                _DO(1, XPLMSetDatai, 2, "1-sim/ndpanel/2/hsiModeRotary");               // ND m. sel. (f/o. side) (map)
                _DO(1, XPLMSetDatai, 2, "1-sim/ndpanel/2/hsiRangeRotary");              // ND r. sel. (f/o. side) ( 40)
                _DO(1, XPLMSetDatai, 1, "757Avionics/options/ND/advEfisPanel");         // avionics: Modern EFIS Panel     (yes) required for EGPWS, Terrain
                _DO(1, XPLMSetDatai, 0, "757Avionics/options/PFD/SpdTrendVector");      // avionics: Trend Vector          (off) requires speedtape, FMA/Top
                _DO(1, XPLMSetDatai, 0, "757Avionics/options/PFD/speedTape");           // avionics: Airspeed Tape         (off) always linked to fma on top
                _DO(1, XPLMSetDatai, 0, "757Avionics/options/PFD/fmaOnTop");            // avionics: FMA on Top            (off) always linked to speed tape
                _DO(1, XPLMSetDatai, 0, "757Avionics/options/ND/tasGs");                // avionics: TAS and GS            (off) already available elsewhere
                _DO(1, XPLMSetDatai, 1, "params/gpwsType");                             // avionics: EGPWS                 (yes) requires: Modern EFIS Panel
                _DO(1, XPLMSetDatai, 0, "params/redDisplays");                          // avionics: red displays          (off)
                _DO(1, XPLMSetDatai, 1, "757Avionics/options/PFD/IlsDevWarning");       // avionics: ILS Deviation Warning (yes)
                _DO(1, XPLMSetDatai, 2, "757Avionics/options/PFD/advRaAlerts");         // avionics: RA Alerts         (2,500ft)
                _DO(1, XPLMSetDatai, 1, "1-sim/optoins/eicas/apuRPM");                  // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 1, "1-sim/optoins/eicas/EngineFuelFlow");          // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 1, "1-sim/optoins/eicas/HydraulicPressure");       // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 0, "1-sim/optoins/eicas/TirePressureDisplay");     // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 0, "1-sim/optoins/eicas/APUOilQuantityDisplay");   // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 1, "1-sim/optoins/eicas/BrakeTemperatureDisplay"); // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 0, "1-sim/optoins/eicas/BulkCargoCompartmentTemperature");   // EICAS display settings
            }
            else // ACF_TYP_B767_FF
            {
                if ((acf_type_is_engine_running() == 0) && // cold & dark
                    (d_ref = XPLMFindDataRef("sim/cockpit2/switches/custom_slider_on")))
                {
                    int door_open[1] = { 1, };
                    XPLMSetDatavi(d_ref, &door_open[0], 0, 1); // LF
                    XPLMSetDatavi(d_ref, &door_open[0], 2, 1); // FWD
                    _DO(1, XPLMSetDatai, 1, "params/ACU"); // additional ground config
                    _DO(1, XPLMSetDatai, 1, "params/LSU"); // additional ground config
                }
//              _DO(1, XPLMSetDatai, 1, "1-sim/ndpanel/1/hsiModeButton");               // requires a Modern EFIS Panel
                _DO(1, XPLMSetDatai, 4, "1-sim/ndpanel/1/hsiModeRotary");               // ND m. sel. (cap. side) (map)
                _DO(1, XPLMSetDatai, 1, "1-sim/ndpanel/1/hsiRangeRotary");              // ND r. sel. (cap. side) ( 20)
//              _DO(1, XPLMSetDatai, 1, "1-sim/ndpanel/2/hsiModeButton");               // requires a Modern EFIS Panel
                _DO(1, XPLMSetDatai, 4, "1-sim/ndpanel/2/hsiModeRotary");               // ND m. sel. (f/o. side) (map)
                _DO(1, XPLMSetDatai, 1, "1-sim/ndpanel/2/hsiRangeRotary");              // ND r. sel. (f/o. side) ( 20)
                _DO(1, XPLMSetDatai, 0, "757Avionics/options/ND/advEfisPanel");         // avionics: Modern EFIS Panel     (off) required for EGPWS, Terrain
                _DO(1, XPLMSetDatai, 1, "757Avionics/options/PFD/SpdTrendVector");      // avionics: Trend Vector          (yes) requires speedtape, FMA/Top
                _DO(1, XPLMSetDatai, 1, "757Avionics/options/PFD/speedTape");           // avionics: Airspeed Tape         (yes) always linked to fma on top
                _DO(1, XPLMSetDatai, 1, "757Avionics/options/PFD/fmaOnTop");            // avionics: FMA on Top            (yes) always linked to speed tape
                _DO(1, XPLMSetDatai, 0, "757Avionics/options/ND/tasGs");                // avionics: TAS and GS            (off) already available elsewhere
                _DO(1, XPLMSetDatai, 0, "params/gpwsType");                             // avionics: EGPWS                 (off) requires: Modern EFIS Panel
                _DO(1, XPLMSetDatai, 2, "params/redDisplays");                          // avionics: red displays          (all)
                _DO(1, XPLMSetDatai, 0, "757Avionics/options/PFD/IlsDevWarning");       // avionics: ILS Deviation Warning (off)
                _DO(1, XPLMSetDatai, 1, "757Avionics/options/PFD/advRaAlerts");         // avionics: RA Alerts         (1,000ft)
                _DO(1, XPLMSetDatai, 1, "1-sim/optoins/eicas/apuRPM");                  // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 1, "1-sim/optoins/eicas/EngineFuelFlow");          // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 1, "1-sim/optoins/eicas/HydraulicPressure");       // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 0, "1-sim/optoins/eicas/TirePressureDisplay");     // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 0, "1-sim/optoins/eicas/APUOilQuantityDisplay");   // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 0, "1-sim/optoins/eicas/BrakeTemperatureDisplay"); // avionics: EICAS display settings
                _DO(1, XPLMSetDatai, 0, "1-sim/optoins/eicas/BulkCargoCompartmentTemperature");   // EICAS display settings
            }
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            break;

        case ACF_TYP_B777_FF:
            _DO(1, XPLMSetDatai,    0, "anim/31/switch");                           // VOR1 on ND1 off
            _DO(1, XPLMSetDatai,    0, "anim/32/switch");                           // VOR2 on ND1 off
            _DO(1, XPLMSetDatai,    2, "anim/64/switch");                           // ND m. sel. (capt. side) (map)
            _DO(1, XPLMSetDatai,    1, "anim/65/switch");                           // ND r. sel. (capt. side) (20)
//          _DO(1, XPLMSetDatai,    1, "anim/???/button"); // TODO: find dataref    // Terrain override switch (on)
//          _DO(1, XPLMSetDataf, 1.0f, "anim/33/cover");                            // Terrain override switch (lift cover)
//          _DO(1, XPLMSetDataf, 1.0f, "anim/14/cover");                            // Engine elec. contr. (L) (lift cover)
//          _DO(1, XPLMSetDataf, 1.0f, "anim/16/cover");                            // Engine elec. contr. (R) (lift cover)
            _DO(1, XPLMSetDatai,    1, "anim/108/button");                          // Engine-dr. hy. pump (L) (on)
            _DO(1, XPLMSetDatai,    1, "anim/111/button");                          // Engine-dr. hy. pump (R) (on)
            _DO(1, XPLMSetDatai,    1, "anim/116/button");                          // Engine elec. contr. (L) (norm)
            _DO(1, XPLMSetDatai,    1, "anim/117/button");                          // Engine elec. contr. (R) (norm)
            _DO(1, XPLMSetDatai,    1, "anim/154/button");                          // Engine autostart switch (on)
            _DO(1, XPLMSetDatai,    1, "anim/137/button");                          // Air: L t. air valve sw. (on)
            _DO(1, XPLMSetDatai,    1, "anim/138/button");                          // Air: R t. air valve sw. (on)
            _DO(1, XPLMSetDatai,    1, "anim/139/button");                          // Air: L bleed ISLN valve (auto)
            _DO(1, XPLMSetDatai,    1, "anim/140/button");                          // Air: C bleed ISLN valve (auto)
            _DO(1, XPLMSetDatai,    1, "anim/141/button");                          // Air: R bleed ISLN valve (auto)
            _DO(1, XPLMSetDatai,    1, "anim/143/button");                          // Air: APU  bleed air sw. (auto)
            _DO(1, XPLMSetDatai,    1, "anim/142/button");                          // Air: L e. bleed air sw. (on)
            _DO(1, XPLMSetDatai,    1, "anim/144/button");                          // Air: R e. bleed air sw. (on)
            _DO(1, XPLMSetDatai,    1, "anim/25/switch");                           // Cargo temp. cont. (aft) (low)
            _DO(1, XPLMSetDatai,    1, "anim/26/switch");                           // Cargo temp. cont. (blk) (low)
            _DO(1, XPLMSetDataf, 0.5f, "anim/5/rotery");                            // Temp. control (f. deck) (auto)
            _DO(1, XPLMSetDataf, 0.5f, "anim/6/rotery");                            // Temp. control (all ca.) (auto)
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            break;

        case ACF_TYP_CL30_DD:
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/custom_slider_on")))
            {
                int door_open[1] = { 1, };
                XPLMSetDatavi(d_ref, &door_open[0], 6, 1);  // cabin to bathroom
            }
            _DO(0, XPLMSetDataf, 0.8f, "sim/cockpit/electrical/instrument_brightness");
            _DO(1, XPLMSetDatai, 0, "cl300/position_saving");
            _DO(1, XPLMSetDatai, 1, "cl300/prflt_cam_lock");
            _DO(1, XPLMSetDatai, 1, "cl300/mmenu_athide");
            _DO(1, XPLMSetDatai, 1, "cl300/fms/alt_rep");
            _DO(1, XPLMSetDatai, 1, "cl300/hide_pilots");
            _DO(1, XPLMSetDatai, 1, "cl300/popup_mfd");
            _DO(1, XPLMSetDatai, 0, "cl300/baro_pref");
            _DO(1, XPLMSetDatai, 0, "cl300/alt_pref");
            _DO(1, XPLMSetDatai, 1, "cl300/com_pref");
            _DO(1, XPLMSetDatai, 0, "cl300/gpu_mode"); // ???
            if (acf_type_is_engine_running() == 0)
            {
                float load = 277.0f, fuel = 1750.0f;
                acf_type_load_set(ctx->info, &load);
                acf_type_fuel_set(ctx->info, &fuel);
            }
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            nvp_efis_setup();
            break;

        case ACF_TYP_E55P_AB:
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/custom_slider_on")))
            {
                int door_close[1] = { 0, };
                XPLMSetDatavi(d_ref, &door_close[0], 1, 1); // baggage FWD LH
                XPLMSetDatavi(d_ref, &door_close[0], 2, 1); // baggage FWD RH
                XPLMSetDatavi(d_ref, &door_close[0], 3, 1); // baggage AFT
                XPLMSetDatavi(d_ref, &door_close[0], 5, 1); // FUEL panel
                XPLMSetDatavi(d_ref, &door_close[0], 4, 1); // GPU panel
            }
            if ((cr = XPLMFindCommand("sim/flight_controls/door_close_1"))) // passenger door
            {
                XPLMCommandOnce(cr);
            }
            if ((d_ref = XPLMFindDataRef("aerobask/hide_static")))
            {
                if (XPLMGetDatai(d_ref) == 0)
                {
                    if ((cr = XPLMFindCommand("aerobask/toggle_static")))
                    {
                        XPLMCommandOnce(cr);
                    }
                }
            }
            if (acf_type_is_engine_running() == 0) // cold & dark
            {
                if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/custom_slider_on")))
                {
                    int door_open[1] = { 1, };
                    XPLMSetDatavi(d_ref, &door_open[0], 4, 1); // GPU panel
                }
                if ((d_ref = XPLMFindDataRef("aerobask/hide_gpu")))
                {
                    if (XPLMGetDatai(d_ref) != 0)
                    {
                        if ((cr = XPLMFindCommand("aerobask/electrical/gpu_connect_disconnect")))
                        {
                            XPLMCommandOnce(cr);
                        }
                    }
                }
                if ((d_ref = XPLMFindDataRef("aerobask/tablet/deployed")))
                {
                    if (XPLMGetDatai(d_ref) != 1)
                    {
                        if ((cr = XPLMFindCommand("aerobask/tablet/deploy_toggle")))
                        {
                            XPLMCommandOnce(cr);
                        }
                    }
                }
                /*
                 * -----------------------
                 * | Already done for us |
                 * -----------------------
                 * **************************************************** COCKPIT.INSPECTION ****************************************************
                 * SUPPLY CONTROL KNOB.................................................................................................PAX AUTO
                 * ELECTRICAL PANEL..................................BATT 1 & 2 Switches in the OFF position. BUS TIE Knob in the AUTO position
                 * TEST PANEL...............................................................................................................OFF
                 * FUEL PUMP 1 & 2 SWITCHES................................................................................................AUTO
                 * XFEED KNOB...............................................................................................................OFF
                 * ELT SWITCH.............................................................................................................ARMED
                 * PUSHER CUTOUT BUTTON..............................................................................................PUSHED OUT
                 * HEATING PANEL..............................WSHLD 1 & 2 Switches in the OFF position and ADS PROBES Knob in the AUTO position
                 * ICE PROTECTION PANEL.................................................................................................ALL OFF
                 * PRESSURIZATION PANEL..............................Pressurization MODE Switch in the AUTO position and DUMP Button pushed out
                 * AIR CONDITIONING PANEL...............................................................................................AS RQRD
                 * ENG FIRE EXTINGUISHER PANEL...............................SHUTOFF 1 & 2 Buttons pushed out and BOTTLE Switch in OFF position
                 *
                 * *************************************************** LEAVING.THE.AIRPLANE ***************************************************
                 * EMER LT SWITCH...........................................................................................................OFF
                 * LIGHTS...................................................................................................................OFF
                 *
                 * ------------------------------------------
                 * | Done and/or controlled via x-nullzones |
                 * ------------------------------------------
                 * **************************************************** COCKPIT.INSPECTION ****************************************************
                 * LANDING GEAR LEVER........................................................................................................DN
                 * SPEED BRAKE SWITCH.....................................................................................................CLOSE
                 *
                 * ******************************************************* BEFORE.START *******************************************************
                 * ENG IGNITION SWITCHES...................................................................................................AUTO
                 *
                 * ********************************************************* SHUTDOWN *********************************************************
                 * THRUST LEVERS...........................................................................................................IDLE
                 * PARKING BRAKE..........................................................................................................APPLY
                 * START/STOP KNOBS........................................................................................................STOP
                 *
                 * ---------------------------
                 * | Applied earlier by navP |
                 * ---------------------------
                 * **************************************************** COCKPIT.INSPECTION ****************************************************
                 * PROBES AND ENGINES COVERS............................................................................................REMOVED
                 *
                 * ----------------------
                 * | applied below here |
                 * ----------------------
                 * **************************************************** COCKPIT.INSPECTION ****************************************************
                 * OXYGEN BOTTLE VALVE HANDLE...................................................................................PUSH TO RESTORE
                 * ELECTRICAL PANEL.....................................................................GEN 1 & 2 Switches in the AUTO position
                 * BLEED 1 & 2 SWITCHES....................................................................................................AUTO
                 * XBLEED KNOB.............................................................................................................AUTO
                 * HYD PUMP SOV 1 & 2 SWITCHES.............................................................................................OPEN
                 * PRESSURIZATION PANEL...........................................................................ECS Knob in the BOTH position
                 */
                if ((d_ref = XPLMFindDataRef("aerobask/oxygen/sw_cut_out")))
                {
                    if (XPLMGetDataf(d_ref) < 0.5f)
                    {
                        if ((cr = XPLMFindCommand("aerobask/oxygen/cut_out")))
                        {
                            XPLMCommandOnce(cr);
                        }
                    }
                }
                if ((cr = XPLMFindCommand("aerobask/electrical/gen1_auto")))
                {
                    XPLMCommandOnce(cr);
                }
                if ((cr = XPLMFindCommand("aerobask/electrical/gen2_auto")))
                {
                    XPLMCommandOnce(cr);
                }
                if ((cr = XPLMFindCommand("aerobask/bleed/bleed1_auto")))
                {
                    XPLMCommandOnce(cr);
                }
                if ((cr = XPLMFindCommand("aerobask/bleed/bleed2_auto")))
                {
                    XPLMCommandOnce(cr);
                }
                if ((cr = XPLMFindCommand("aerobask/bleed/xbleed_auto")))
                {
                    XPLMCommandOnce(cr);
                }
                if ((d_ref = XPLMFindDataRef("aerobask/hyd/sw_pump1")))
                {
                    if (XPLMGetDataf(d_ref) < 0.5f)
                    {
                        if ((cr = XPLMFindCommand("aerobask/hyd/pump1_up")))
                        {
                            XPLMCommandOnce(cr);
                        }
                    }
                }
                if ((d_ref = XPLMFindDataRef("aerobask/hyd/sw_pump2")))
                {
                    if (XPLMGetDataf(d_ref) < 0.5f)
                    {
                        if ((cr = XPLMFindCommand("aerobask/hyd/pump2_up")))
                        {
                            XPLMCommandOnce(cr);
                        }
                    }
                }
                if ((cr = XPLMFindCommand("aerobask/press/ecs_rt")))
                {
                    if ((d_ref = XPLMFindDataRef("aerobask/press/knob_ecs")))
                    {
                        if (XPLMGetDataf(d_ref) < 1.5f)
                        {
                            if (XPLMGetDataf(d_ref) < 0.5f)
                            {
                                XPLMCommandOnce(cr); // OFF VENT -> 1
                            }
                            XPLMCommandOnce(cr); // 1 -> AUTO
                        }
                    }
                }
                float pload = 180.0f, fuelq = 710.0f;
                acf_type_load_set(ctx->info, &pload);
                acf_type_fuel_set(ctx->info, &fuelq);
            }
            _DO(0, XPLMSetDatai, 0, "aerobask/show_reflections_instruments");
            _DO(0, XPLMSetDatai, 0, "aerobask/show_reflections_windows");
            _DO(1, XPLMSetDatai, 0, "sim/graphics/view/hide_yoke");
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            nvp_x1000_setup();
            break;

        case ACF_TYP_EMBE_SS:
            if (ctx->mcdu.rc.i_disabled == -1)
            {
                chandler_mcdup(ctx->mcdu.cb.command, xplm_CommandEnd, &ctx->mcdu.rc); // XXX: remap hotkeys
            }
            _DO(1, XPLMSetDatai, 0, "ssg/EJET/GND/stair1_ON");     // Hide passenger stairs
            _DO(1, XPLMSetDatai, 1, "ssg/EJET/GND/rain_hide_sw");  // Disable rain effects
            _DO(1, XPLMSetDatai, 0, "ssg/EJET/GND/seats_hide_sw"); // Hide captain's seat
            _DO(1, XPLMSetDatai, 0, "ssg/EJET/GND/yokes_hide_sw"); // Hide both yokes
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            break;

        case ACF_TYP_EMBE_XC:
            if (ctx->mcdu.rc.i_disabled == -1)
            {
                chandler_mcdup(ctx->mcdu.cb.command, xplm_CommandEnd, &ctx->mcdu.rc);  // XXX: remap hotkeys
            }
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/electrical/generator_on")))
            {
                int generator_on[1] = { 1, };
                XPLMSetDatavi(d_ref, &generator_on[0], 0, 1);
                XPLMSetDatavi(d_ref, &generator_on[0], 1, 1);
                _DO(1, XPLMSetDatai, 1, "sim/cockpit2/electrical/APU_generator_on");
            }
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/electrical/inverter_on")))
            {
                int inverter_on[1] = { 1, };
                XPLMSetDatavi(d_ref, &inverter_on[0], 0, 1);
                XPLMSetDatavi(d_ref, &inverter_on[0], 1, 1);
                _DO(1, XPLMSetDatai, 1, "sim/cockpit2/electrical/cross_tie");
            }
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine")))
            {
                int ice_inlet_heat_on_per_engine[1] = { 1, };
                XPLMSetDatavi(d_ref, &ice_inlet_heat_on_per_engine[0], 0, 1);
                XPLMSetDatavi(d_ref, &ice_inlet_heat_on_per_engine[0], 1, 1);
                _DO(1, XPLMSetDatai, 1, "sim/cockpit2/ice/ice_surfce_heat_on");
                _DO(1, XPLMSetDatai, 1, "sim/cockpit2/ice/ice_window_heat_on");
            }
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/generic_lights_switch")))
            {
                float generic_lights_switch[1] = { 1.0f, };
                XPLMSetDatavf(d_ref, &generic_lights_switch[0], 14, 1); // pack 1
                XPLMSetDatavf(d_ref, &generic_lights_switch[0], 15, 1); // pack 2
                XPLMSetDatavf(d_ref, &generic_lights_switch[0], 28, 1); // bleed1
                XPLMSetDatavf(d_ref, &generic_lights_switch[0], 29, 1); // bleed2
            }
            if (acf_type_is_engine_running() == 0)
            {
                float load = 541.0f, fuel = 3150.0f;
                acf_type_load_set(ctx->info, &load);
                acf_type_fuel_set(ctx->info, &fuel);
            }
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            nvp_efis_setup();
            break;

        case ACF_TYP_HA4T_RW:
            if (ctx->mcdu.rc.i_disabled == -1)
            {
                chandler_mcdup(ctx->mcdu.cb.command, xplm_CommandEnd, &ctx->mcdu.rc);  // XXX: remap hotkeys
            }
            _DO(1, XPLMSetDatai, 0, "Hawker4000/hideshow/car");
            _DO(1, XPLMSetDatai, 1, "Hawker4000/yoke/hide_show");
            _DO(1, XPLMSetDatai, 0, "Hawker4000/pref/wingflex_b");
            _DO(1, XPLMSetDatai, 0, "Hawker4000/pref/map_detail_b");
            _DO(1, XPLMSetDatai, 0, "Hawker4000/hideshow/chockscones");
            _DO(1, XPLMSetDatai, 0, "Hawker4000/hideshow/engine_cover");
            _DO(1, XPLMSetDatai, 1, "Hawker4000/avionics/pilot/EFIS_range");
            _DO(1, XPLMSetDatai, 0, "Hawker4000/avionics/pilot/EFIS_fix_on");
            _DO(1, XPLMSetDatai, 2, "Hawker4000/avionics/copilot/EFIS_range");
            _DO(1, XPLMSetDatai, 0, "Hawker4000/avionics/copilot/EFIS_fix_on");
            _DO(1, XPLMSetDatai, 1, "Hawker4000/avionics/pilot/EFIS_navaid_on");
            _DO(1, XPLMSetDatai, 1, "Hawker4000/avionics/pilot/EFIS_airport_on");
            _DO(1, XPLMSetDatai, 1, "Hawker4000/avionics/copilot/EFIS_navaid_on");
            _DO(1, XPLMSetDatai, 1, "Hawker4000/avionics/copilot/EFIS_airport_on");
            _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_pilot");
            _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_copilot");
            if (acf_type_is_engine_running() == 0)
            {
                float load = 290.0f, fuel = 1750.0f;
                acf_type_load_set(ctx->info, &load);
                acf_type_fuel_set(ctx->info, &fuel);
            }
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            break;

        case ACF_TYP_LEGA_XC:
            if (ctx->mcdu.rc.i_disabled == -1)
            {
                chandler_mcdup(ctx->mcdu.cb.command, xplm_CommandEnd, &ctx->mcdu.rc);  // XXX: remap hotkeys
            }
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/generic_lights_switch")))
            {
                float generic_lights_switch[1] = { 1.0f, };
                XPLMSetDatavf(d_ref, &generic_lights_switch[0], 56, 1); // reflec. off
            }
            if (acf_type_is_engine_running() == 0)
            {
                float load = 320.0f, fuel = 2520.0f;
                acf_type_load_set(ctx->info, &load);
                acf_type_fuel_set(ctx->info, &fuel);
                _DO(0, XPLMSetDataf, 0.0f, "sim/flightmodel/misc/cgz_ref_to_default");
            }
            _DO(1, XPLMSetDatai, 0, "sim/graphics/view/hide_yoke");
            _DO(1, XPLMSetDatai, 1, "XCrafts/ERJ/weight_units");
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            nvp_efis_setup();
            break;

        case ACF_TYP_MD80_RO:
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/panel_brightness_ratio")))
            {
                float panel_brightness_ratio[2] = { 0.5f, .25f, };
                XPLMSetDatavf(d_ref, &panel_brightness_ratio[0], 1, 1);             // Inst. lights intensity control
                XPLMSetDatavf(d_ref, &panel_brightness_ratio[1], 3, 1);             // Flood lights intensity control
            }
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/instrument_brightness_ratio")))
            {
                float instrument_brightness_ratio[1] = { 1.0f, };
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0], 0, 1);        // LED readouts intensity control
            }
            if (acf_type_is_engine_running() == 0) // cold & dark
            {
                _DO(1, XPLMSetDatai, 1, "Rotate/md80/electrical/GPU_power_available");
            }
            _DO(1, XPLMSetDatai, 1, "Rotate/md80/misc/hide_yoke_button_clicked");
            _DO(1, XPLMSetDatai, 2, "Rotate/md80/instruments/nav_display_range");
            _DO(1, XPLMSetDatai, 2, "Rotate/md80/instruments/nav_display_mode");
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            break;

        case ACF_TYP_TBM9_HS:
            if ((d_ref = XPLMFindDataRef("tbm900/doors/pilot")))
            {
                if (0.01f < XPLMGetDataf(d_ref))
                {
                    if ((cr = XPLMFindCommand("tbm900/doors/auto_pilot")))
                    {
                        XPLMCommandOnce(cr);
                    }
                }
            }
            if ((d_ref = XPLMFindDataRef("tbm900/doors/main")))
            {
                if (0.01f < XPLMGetDataf(d_ref))
                {
                    if ((cr = XPLMFindCommand("tbm900/doors/auto_main")))
                    {
                        XPLMCommandOnce(cr);
                    }
                }
            }
            if ((d_ref = XPLMFindDataRef("tbm900/doors/front_cargo")))
            {
                if (0.01f < XPLMGetDataf(d_ref))
                {
                    if ((cr = XPLMFindCommand("tbm900/doors/front_cargo")))
                    {
                        XPLMCommandOnce(cr);
                    }
                }
            }
            if ((d_ref = XPLMFindDataRef("tbm900/anim/wing/wingtip_wick_flag")))
            {
                int off[2] = { 0, 0, };
                XPLMSetDatavi(d_ref, &off[0], 0, 2);
            }
            if ((d_ref = XPLMFindDataRef("tbm900/anim/tail/static_cover")))
            {
                int off[2] = { 0, 0, };
                XPLMSetDatavi(d_ref, &off[0], 0, 2);
            }
            if ((d_ref = XPLMFindDataRef("tbm900/anim/wing/pitot_cover")))
            {
                int off[2] = { 0, 0, };
                XPLMSetDatavi(d_ref, &off[0], 0, 2);
            }
            if ((d_ref = XPLMFindDataRef("tbm900/anim/yoke_show")))
            {
                int hide[2] = { 0, 0, };
                XPLMSetDatavi(d_ref, &hide[0], 0, 2);
            }
            if (acf_type_is_engine_running() == 0) // cold & dark
            {
                if ((cr = XPLMFindCommand("sim/electrical/GPU_on")))
                {
                    XPLMCommandOnce(cr);
                }
                float fuel = 456.0f; acf_type_fuel_set(ctx->info, &fuel); // half tanks
            }
            _DO(0, XPLMSetDataf, TBM9_FLIGHT_IDLE_GATE, "sim/cockpit2/engine/actuators/throttle_ratio_all"); // flight idle
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/fuel/fuel_tank_selector"); // left tank
            _DO(1, XPLMSetDatai, 1, "tbm900/switches/fuel/auto_man"); // FUEL SEL switch position. 0 = AUTO, 1 = MAN.
            _DO(1, XPLMSetDatai, 0, "tbm900/switches/gear/chocks");
            _DO(1, XPLMSetDatai, 0, "tbm900/anim/engine/tied");
            _DO(1, XPLMSetDatai, 1, "tbm900/tablet/visible");
            nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            nvp_x1000_setup();
            break;

        case ACF_TYP_GENERIC: // note: path is never verbose (don't warn for unapplicable datarefs)
            /*
             * Aerobask
             */
            _DO(0, XPLMSetDatai, 1, "sim/har/reflets");                             // LEG2: refl. off
            _DO(0, XPLMSetDatai, 0, "sim/har/pitchservo");                          // LEG2: 500ft/min
            _DO(0, XPLMSetDatai, 0, "aerobask/E1000/flags_on");                     // EPIC
            _DO(0, XPLMSetDatai, 1, "aerobask/E1000/yokeL_hidden");                 // EPIC
            _DO(0, XPLMSetDatai, 1, "aerobask/E1000/yokeR_hidden");                 // EPIC
            _DO(0, XPLMSetDatai, 0, "aerobask/flags_on");                           // DA62
            _DO(0, XPLMSetDatai, 0, "aerobask/reflections");                        // DA62
            _DO(0, XPLMSetDatai, 0, "aerobask/show_reflections_windows");           // G.1K
            _DO(0, XPLMSetDatai, 0, "aerobask/show_reflections_instruments");       // G.1K
            _DO(0, XPLMSetDatai, 0, "aerobask/E1000/reflections_skyview_on");       // EPIC
            _DO(0, XPLMSetDatai, 0, "aerobask/E1000/reflections_windshield_on");    // EPIC
            _DO(0, XPLMSetDatai, 0, "aerobask/eclipse/flags_on");                   // EA50
            _DO(0, XPLMSetDatai, 0, "aerobask/eclipse/reflections_skyview_on");     // EA50
            _DO(0, XPLMSetDatai, 0, "aerobask/eclipse/reflections_windshield_on");  // EA50
            _DO(0, XPLMSetDatai, 0, "aerobask/victory/flags_on");                   // EVIC
            _DO(0, XPLMSetDatai, 1, "aerobask/victory/yokeL_hidden");               // EVIC
            _DO(0, XPLMSetDatai, 1, "aerobask/victory/yokeR_hidden");               // EVIC
            _DO(0, XPLMSetDatai, 0, "aerobask/victory/reflections_skyview_on");     // EVIC
            _DO(0, XPLMSetDatai, 0, "aerobask/victory/reflections_windshield_on");  // EVIC
            _DO(0, XPLMSetDatai, 0, "aerobask/panthera/flags_on");                  // PIPA
            _DO(0, XPLMSetDatai, 0, "aerobask/panthera/reflections_skyview_on");    // PIPA
            _DO(0, XPLMSetDatai, 0, "aerobask/panthera/reflections_windshield_on"); // PIPA
            _DO(0,XPLMSetDataf,0.0f,"aerobask/panthera/reflections_EFIS");          // PIPA
            _DO(0,XPLMSetDataf,0.0f,"aerobask/panthera/reflections_annunciators");  // PIPA
            /*
             * Alabeo/Carenado, SimCoders REP
             */
            _DO(0, XPLMSetDatai, 1, "com/dkmp/HideYokeL");                          // various aircraft
            _DO(0, XPLMSetDatai, 1, "com/dkmp/HideYokeR");                          // various aircraft
            _DO(0, XPLMSetDatai, 0, "thranda/views/InstRefl");                      // various aircraft
            _DO(0, XPLMSetDatai, 0, "thranda/views/WindowRefl");                    // various aircraft
            _DO(0, XPLMSetDatai, 0, "thranda/views/staticelements");                // various aircraft
            _DO(0, XPLMSetDatai, 1, "thranda/cockpit/actuators/HideYokeL");         // various aircraft
            _DO(0, XPLMSetDatai, 1, "thranda/cockpit/actuators/HideYokeR");         // various aircraft
            if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("com.simcoders.rep"))
            {
                simc = 1;
                _DO(0, XPLMSetDatai,    0, "simcoders/rep/staticelements/visible");
                _DO(0, XPLMSetDataf, 1.0f, "simcoders/rep/engine/cowl/handle_ratio_0");
                _DO(0, XPLMSetDataf, 1.0f, "simcoders/rep/engine/cowl/handle_ratio_1");
            }
            if ((d_ref = XPLMFindDataRef("com/dkmp/WindowRefl")) &&
                (xplmType_Int & XPLMGetDataRefTypes(d_ref)))
            {
                _DO(0, XPLMSetDatai, 0, "com/dkmp/InstrRefl");
                _DO(0, XPLMSetDatai, 0, "com/dkmp/WindowRefl");
            }
            else if (((d_ref = XPLMFindDataRef("com/dkmp/windowrefl"))))
            {
                if ((XPLMGetDataRefTypes(d_ref) & xplmType_Int))
                {
                    _DO(0, XPLMSetDatai,    0, "com/dkmp/windowrefl");
                }
                else
                {
                    _DO(0, XPLMSetDataf, 0.0f, "com/dkmp/windowrefl");
                }
            }
            if ((d_ref = XPLMFindDataRef("com/dkmp/tintedwindows")))
            {
                if ((XPLMGetDataRefTypes(d_ref) & xplmType_Int))
                {
                    _DO(0, XPLMSetDatai,    0, "com/dkmp/tintedwindows");
                }
                else
                {
                    _DO(0, XPLMSetDataf, 0.0f, "com/dkmp/tintedwindows");
                }
            }
            if (XPLMFindDataRef("com/dkmp/static")    ||
                XPLMFindDataRef("thranda/views/bush") ||
                XPLMFindDataRef("com/dkmp/staticelements"))
            {
                _DO(0, XPLMSetDatai, 1, "thranda/views/bush");                      // C207
                _DO(0, XPLMSetDatai, 1, "com/dkmp/VCWindows");                      // C208
                _DO(0, XPLMSetDatai, 1, "com/dkmp/cargopod");                       // C208
                _DO(0, XPLMSetDatai, 0, "com/dkmp/static");                         // various aircraft
                _DO(0, XPLMSetDatai, 0, "com/dkmp/staticelements");                 // various aircraft
                if (strcasecmp(ctx->info->icaoid, "C404"))
                {
                    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/switches/no_smoking");        // HideYokeL
                    _DO(0, XPLMSetDatai, 1, "sim/cockpit2/switches/fasten_seat_belts"); // HideYokeR
                }
            }
            if (!strcasecmp(ctx->info->icaoid, "BE20"))
            {
                xgps = 1;
                _DO(0, XPLMSetDataf, 1.0f, "com/dkmp/winglets");
                _DO(0, XPLMSetDataf, 1.0f, "com/dkmp/PassengerDoorHandle");
            }
            if (!strcasecmp(ctx->info->icaoid, "C206"))
            {
                xgps = 1;
                _DO(0, XPLMSetDatai, 1, "com/dkmp/fairings");
                _DO(0, XPLMSetDatai, 0, "com/dkmp/InstRefl");
                _DO(0, XPLMSetDatai, 1, "com/dkmp/WindowRefl"); // inverted
            }
            if (!strcasecmp(ctx->info->icaoid, "TBM8"))
            {
                xgps = 1;
                _DO(0, XPLMSetDataf, -0.5f, "thranda/cockpit/actuators/VisorL");
                _DO(0, XPLMSetDataf, -0.5f, "thranda/cockpit/actuators/VisorR");
                _DO(0, XPLMSetDataf,  1.0f, "thranda/cockpit/actuators/VisorSwingL");
                _DO(0, XPLMSetDataf,  1.0f, "thranda/cockpit/actuators/VisorSwingR");
                _DO(0, XPLMSetDataf,  0.0f, "thranda/cockpit/actuators/VisorSlideL");
                _DO(0, XPLMSetDataf,  0.0f, "thranda/cockpit/actuators/VisorSlideR");
            }
            /*
             * X-Plane default
             */
            _DO(0, XPLMSetDataf, 0.8f, "sim/cockpit/electrical/instrument_brightness"); // set all at once
            /*
             * Aircraft-specific
             */
            if (ctx->info->author[0] || ctx->info->descrp[0]) // we may see "" description
            {
                if (!STRN_CASECMP_AUTO(ctx->info->author, "After"))
                {
                    if (!STRN_CASECMP_AUTO(ctx->info->icaoid, "FA7X"))
                    {
                        if ((d_ref = XPLMFindDataRef("sim/weapons/targ_h")))
                        {
                            float wings_level = AFTER_7X_PATH(0.0f);
                            XPLMSetDatavf(d_ref, &wings_level, 0, 1);
                        }
                        // relocating the aircraft while cold & dark resets many datarefs
                        if ((d_ref = XPLMFindDataRef("sim/cockpit/electrical/generator_on")))
                        {
                            int generator_on[3] = { 1, 1, 1, };
                            XPLMSetDatavi(d_ref, &generator_on[0], 0, 3);
                        }
                        if ((d_ref = XPLMFindDataRef("sim/cockpit2/controls/speedbrake_ratio")))
                        {
                            XPLMSetDataf(d_ref, -0.5f);
                        }
                        // no controls for pitot/aoa heat, always turn ON (even in cold/dark)
                        _DO(1, XPLMSetDatai, 1, "sim/cockpit2/ice/ice_pitot_heat_on_copilot");
                        _DO(1, XPLMSetDatai, 1, "sim/cockpit2/ice/ice_pitot_heat_on_pilot");
                        _DO(1, XPLMSetDatai, 1, "sim/cockpit2/ice/ice_AOA_heat_on_copilot");
                        _DO(1, XPLMSetDatai, 1, "sim/cockpit2/ice/ice_auto_ignite_on");
                        _DO(1, XPLMSetDatai, 1, "sim/cockpit2/ice/ice_AOA_heat_on");
                    }
                }
                if (!STRN_CASECMP_AUTO(ctx->info->author, "Alabeo") ||
                    !STRN_CASECMP_AUTO(ctx->info->author, "Carenado"))
                {
                    if (!STRN_CASECMP_AUTO(ctx->info->descrp, "Pilatus PC12"))
                    {
                        if (simc == 0) // X-Plane 10 and/or no REP package installed
                        {
                            // make the aircraft less tail-heavy to improve ground handling
                            _DO(0, XPLMSetDataf, +0.10f, "sim/aircraft/overflow/acf_cgZ_aft");
                            _DO(0, XPLMSetDataf, -0.30f, "sim/aircraft/overflow/acf_cgZ_fwd");
                            _DO(0, XPLMSetDataf, -0.20f, "sim/flightmodel/misc/cgz_ref_to_default");
                        }
                        else
                        {
                            _DO(0, XPLMSetDataf,  1.0000f, "thranda/cockpit/actuators/VisorL");
                            _DO(0, XPLMSetDataf,  1.0000f, "thranda/cockpit/actuators/VisorR");
                            _DO(0, XPLMSetDataf, -0.1125f, "thranda/cockpit/actuators/VisorSwingL");
                            _DO(0, XPLMSetDataf, -0.1125f, "thranda/cockpit/actuators/VisorSwingR");
                        }
                        // let's also skip drawing the FMS line from the HSI display's map
                        _DO(1, XPLMSetDatai, 1, "sim/graphics/misc/kill_map_fms_line");
                        // and fully declutter the HSI/Avidyne displays by default
                        _DO(0, XPLMSetDatai, 0, "com/dkmp/Avidyne/Declutter");
                        xgps = 1;
                    }
                    if (!STRN_CASECMP_AUTO(ctx->info->descrp, "C207 Skywagon"))
                    {
                        // make the aircraft less tail-heavy to improve ground handling
                        _DO(0, XPLMSetDataf, -0.20f, "sim/aircraft/overflow/acf_cgZ_fwd");
                        _DO(0, XPLMSetDataf, -0.10f, "sim/flightmodel/misc/cgz_ref_to_default");
                        _DO(0, XPLMSetDataf, +0.00f, "sim/aircraft/overflow/acf_cgZ_aft");
                        xgps = 1;
                    }
                    if (!STRN_CASECMP_AUTO(ctx->info->descrp, "C404 Titan"))
                    {
                        if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/instrument_brightness_ratio")))
                        {
                            float instrument_brightness_ratio[1] = { 0.5f, };
                            XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0], 4, 1); // autopilot/warning annunciator brightness
                        }
                        xgps = 1;
                    }
                    if (!STRN_CASECMP_AUTO(ctx->info->descrp, "Bonanza V35B"))
                    {
                        if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/custom_slider_on")))
                        {
                            int tip_tanks_enabled[1] = { 1, };
                            XPLMSetDatavi(d_ref, &tip_tanks_enabled[0], 10, 1);
                        }
                        xgps = 1;
                    }
                }
                if (!STRN_CASECMP_AUTO(ctx->info->author, "Aerobask") ||
                    !STRN_CASECMP_AUTO(ctx->info->author, "Stephane Buon"))
                {
                    absk = 1;
                }
                if (absk)
                {
                    if (!STRN_CASECMP_AUTO(ctx->info->descrp, "Pipistrel Panthera"))
                    {
//                      if (acf_type_is_engine_running() == 0) // cold & dark
//                      {
//                          if ((cr = XPLMFindCommand("sim/electrical/GPU_on")))
//                          {
//                              XPLMCommandOnce(cr);
//                          }
//                      }
                        _DO(1, XPLMSetDatai, 1, "aerobask/panthera/key_engaged");
                    }
                    else if (!STRN_CASECMP_AUTO(ctx->info->descrp, "Epic E1000") ||
                             !STRN_CASECMP_AUTO(ctx->info->descrp, "Epic Victory"))
                    {
                        if ((d_ref = XPLMFindDataRef("aerobask/hide_static")) &&
                            (cr = XPLMFindCommand("aerobask/toggle_static")))
                        {
                            if (XPLMGetDatai(d_ref) == 0)
                            {
                                XPLMCommandOnce(cr);
                            }
                            if ((d_ref = XPLMFindDataRef("sim/graphics/view/hide_yoke")) &&
                                (cr = XPLMFindCommand("sim/operation/toggle_yoke")))
                            {
                                if (XPLMGetDatai(d_ref) != 0)
                                {
                                    XPLMCommandOnce(cr);
                                }
                            }
                        }
                        if (XPLM_NO_PLUGIN_ID == XPLMFindPluginBySignature("1-sim.sasl"))
                        {
                            if ((d_ref = XPLMFindDataRef("aerobask/tablet/deployed")) &&
                                (cr = XPLMFindCommand("aerobask/tablet/deploy_toggle")))
                            {
                                if (acf_type_is_engine_running() == 0) // cold & dark
                                {
                                    if (XPLMGetDatai(d_ref) != 1)
                                    {
                                        XPLMCommandOnce(cr);
                                    }
                                }
                                x1000 = 1; // custom SASL signature: G1000 version
                            }
                        }
                        if (acf_type_is_engine_running() == 0) // cold & dark
                        {
                            if ((cr = XPLMFindCommand("sim/electrical/GPU_on")))
                            {
                                XPLMCommandOnce(cr);
                            }
                            _DO(0, XPLMSetDatai, 0, "sim/cockpit2/pressurization/actuators/bleed_air_mode");
                            _DO(0, XPLMSetDatai, 0, "sim/cockpit2/ice/ice_pitot_heat_on_copilot");
                            _DO(0, XPLMSetDatai, 0, "sim/cockpit2/ice/ice_pitot_heat_on_pilot");
                            _DO(0, XPLMSetDatai, 0, "sim/cockpit2/ice/ice_detect_on");
                        }
                    }
                    else if (!strcasecmp(ctx->info->icaoid, "DA62"))
                    {
                        x1000 = 1;
                        _DO(0, XPLMSetDataf,           0.0f, "aerobask/tablet/anim_x");
                        _DO(0, XPLMSetDataf, 73.0f / 150.0f, "aerobask/tablet/anim_z");
                    }
                    else if (!strcasecmp(ctx->info->icaoid, "EA50"))
                    {
                        if (acf_type_is_engine_running() == 0) // cold & dark
                        {
                            if ((cr = XPLMFindCommand("sim/electrical/GPU_on")))
                            {
                                XPLMCommandOnce(cr);
                            }
                        }
                        // no cockpit controls for pitot/aoa heat, always turn ON (even in cold & dark)
                        _DO(0, XPLMSetDatai, 0, "sim/cockpit2/pressurization/actuators/bleed_air_mode");
                        _DO(0, XPLMSetDatai, 1, "sim/cockpit2/ice/ice_pitot_heat_on_copilot");
                        _DO(0, XPLMSetDatai, 1, "sim/cockpit2/ice/ice_pitot_heat_on_pilot");
                        _DO(0, XPLMSetDatai, 1, "sim/cockpit2/ice/ice_AOA_heat_on_copilot");
                        _DO(0, XPLMSetDatai, 1, "sim/cockpit2/ice/ice_AOA_heat_on");
                    }
                }
            }
            if (ctx->info->engine_count == 1) // single-engine: select default fuel tank
            {
                fuel_tank_select(ctx->info->engine_count, ctx->info->ftanks_count);
            }
            if (x1000)
            {
                nvp_x1000_setup();
            }
            else if (xgps)
            {
                nvp_xgps_setup();
            }
            else if (absk == 0) // don't mess w/Aerobask's WXR radar
            {
                nvp_efis_setup();
            }
            else
            {
                nvp_skyv_setup();
            }
            if (xnz == 0) // unless we specifically skip it
            {
                nvp_xnz_setup(ctx->info->engine_count, acf_type_is_engine_running());
            }
            if (acf_type_is_engine_running() == 0 && absk == 0 && simc == 0)
            {
                float fmax; acf_type_fmax_get(ctx->info, &fmax); // fuel capacity
                float load = 77.0f; acf_type_load_set(ctx->info, &load); // PIC only
                float fuel = fmax / 4.0f; acf_type_fuel_set(ctx->info, &fuel); // 25% fuel
            }
            break;

        default:
            ndt_log("navP [error]: first_fcall_do: non-generic non-handled aircraft type (%d)\n", ctx->info->ac_type);
            XPLMSpeakString("turn around error");
            return ENOSYS;
    }

    /*
     * Set default transponder code:
     * 2000 is the default "non-discrete" code for IFR (e.g. oceanic operations)
     * 1200 (or 7000), on the other hand, only apply for VFR operations instead
     */
    switch (ctx->info->ac_type)
    {
        case ACF_TYP_A320_FF: // pointless: will reset when powering up aircraft
            break;
        case ACF_TYP_A319_TL:
        case ACF_TYP_A321_TL:
            _DO(1, XPLMSetDatai, 2, "AirbusFBW/XPDR4");
            _DO(1, XPLMSetDatai, 0, "AirbusFBW/XPDR3");
            _DO(1, XPLMSetDatai, 0, "AirbusFBW/XPDR2");
            _DO(1, XPLMSetDatai, 0, "AirbusFBW/XPDR1");
            break;
        default:
            _DO(0, XPLMSetDatai, 2000, "sim/cockpit2/radios/actuators/transponder_code");
            break;
    }

    /* Re-register gear callbacks (get calls before any newly-loaded plugins) */
    UNREGSTR_CHANDLER(ctx->gear.landing_gear_toggle);
    UNREGSTR_CHANDLER(ctx->gear.  landing_gear_down);
    UNREGSTR_CHANDLER(ctx->gear.    landing_gear_up);
    REGISTER_CHANDLER(ctx->gear.    landing_gear_up, chandler_ghndl, 1, &ctx->gear);
    REGISTER_CHANDLER(ctx->gear.  landing_gear_down, chandler_ghndl, 1, &ctx->gear);
    REGISTER_CHANDLER(ctx->gear.landing_gear_toggle, chandler_ghndl, 1, &ctx->gear);

    /* resolve addon-specific references early (might be faster?) */
    chandler_command *list[] =
    {
        &ctx->otto.conn.cc,
        &ctx->otto.disc.cc,
        NULL,
    };
    for (int i = 0; list[i]; i++)
    {
        if (list[i]->name && list[i]->xpcr == NULL)
        {
            list[i]->xpcr = XPLMFindCommand(list[i]->name);
        }
    }
    if (ctx->info->ac_type == ACF_TYP_B757_FF || ctx->info->ac_type == ACF_TYP_B767_FF)
    {
        ctx->otto.ffst.dr = XPLMFindDataRef("1-sim/AP/cmd_L_Button");
    }
    if (ctx->gear.has_retractable_gear == -1)
    {
        ctx->gear.has_retractable_gear = !!XPLMGetDatai(ctx->gear.acf_gear_retract);
    }

    /* Register custom autopilot disconnect handlers when applicable */
    if (ctx->info->has_auto_thr == 1 && ctx->otto.disc.cc.xpcr != NULL)
    {
        switch (ctx->info->ac_type)
        {
            case ACF_TYP_EMBE_XC:
            case ACF_TYP_HA4T_RW:
            case ACF_TYP_GENERIC:
                if (ctx->otto.disc.cc.xpcr == ctx->apd.bef.command)
                {
                    REGISTER_CHANDLER(ctx->apd.bef, chandler_apbef, 1, &ctx->apd);
                    REGISTER_CHANDLER(ctx->apd.aft, chandler_apaft, 0, &ctx->apd);
                }
                break;

            default:
                break;
        }
    }

    /* Register custom autopilot pitch hold handlers when applicable */
    if (ctx->info->ac_type == ACF_TYP_GENERIC)
    {
        REGISTER_CHANDLER(ctx->vvi.dn, chandler_p2vvi, 1, &ctx->vvi);
        REGISTER_CHANDLER(ctx->vvi.up, chandler_p2vvi, 1, &ctx->vvi);
        REGISTER_CHANDLER(ctx->vvi.pd, chandler_p2vvi, 1, &ctx->vvi);
        REGISTER_CHANDLER(ctx->vvi.pu, chandler_p2vvi, 1, &ctx->vvi);
    }

#if TIM_ONLY
    /*
     * Partial time and date sync: sync date with today, sync
     * minutes and seconds; however, we don't set hour of day.
     */
    if (ctx->ground.time.zulu_time_xpl)
    {
        ndt_date now = ndt_date_now();
        int xphrs = XPLMGetDatai(ctx->ground.time.zulu_time_hrs);
        if ((d_ref = XPLMFindDataRef("sim/time/local_date_days")))
        {
            // note: X-Plane doesn't seem to know 02/29 (makes our job that much easier :-)
            int month2days[12] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, };
            int xplm_date_days = month2days[now.month - 1] + now.day - 1;
            XPLMSetDatai(d_ref, xplm_date_days);
        }
        int m = XPLMGetDatai(ctx->ground.time.zulu_time_min);
        int s = XPLMGetDatai(ctx->ground.time.zulu_time_sec);
        int d = (60 * now.minutes + now.seconds) - (60 * m + s);
        if (d > +1800)
        {
            xphrs = (xphrs - 1) % 24; // increase by over 30 minutes -> decrease by under 30 minutes
        }
        if (d < -1800)
        {
            xphrs = (xphrs + 1) % 24; // descrese by over 30 minutes -> increase by under 30 minutes
        }
        if (abs(d) > 90)
        {
            ndt_log("navP [info]: zulu time resync: %02d:%02d:%02d -> %02d:%02d:%02d at %02d:%02d:%02d\n",
                    XPLMGetDatai(ctx->ground.time.zulu_time_hrs), m, s, xphrs,
                    now.minutes, now.seconds, now.hours, now.minutes, now.seconds);
        }
#if 0
        else
        {
            ndt_log("navP [debug]: zulu time resync: %02d:%02d:%02d -> %02d:%02d:%02d at %02d:%02d:%02d (%d -> %d, d  == %d)\n",
                    xphrs, XPLMGetDatai(ctx->ground.time.zulu_time_min), XPLMGetDatai(ctx->ground.time.zulu_time_sec),
                    xphrs, now.minutes, now.seconds, now.hours, now.minutes, now.seconds,
                    60 * now.minutes + now.seconds, 60 * m + s, d);
        }
#endif
        XPLMSetDataf(ctx->ground.time.zulu_time_xpl, (float)(3600 * xphrs + 60 * now.minutes + now.seconds));
    }
#endif

    if (ctx->ground.oatc.flc)
    {
        XPLMUnregisterFlightLoopCallback(ctx->ground.oatc.flc, &ctx);
    }
    if (ctx->ground.flc_g)
    {
        XPLMUnregisterFlightLoopCallback(ctx->ground.flc_g, &ctx->ground);
    }
    ctx->ground.last_cycle_number = -1;
    XPLMRegisterFlightLoopCallback((ctx->ground.flc_g = &gnd_stab_hdlr), 1, &ctx->ground);

    /* mixture and prop pitch command handlers */
    switch (ctx->info->engine_type1)
    {
        case 0: // piston (carburetor)
        case 1: // piston (injection)
            if (STRN_CASECMP_AUTO(ctx->info->icaoid, "DA62") != 0)
            {
                REGISTER_CHANDLER(ctx->throt.md, chandler_mixdn, 1, ctx->throt.mixratio);
                REGISTER_CHANDLER(ctx->throt.rp, chandler_rpmdn, 1, ctx->throt.rpmratio);
                break;
            }
            break;

        case 2: // turbine (free)
        case 8: // turbine (fixed)
        case 9: // turbine (XPL11)
            if (ctx->info->ac_type != ACF_TYP_TBM9_HS)
            {
                if (STRN_CASECMP_AUTO(ctx->info->icaoid, "PC12") != 0)
                {
                    REGISTER_CHANDLER(ctx->throt.rp, chandler_rpmdn, 1, ctx->throt.rpmratio);
                }
                REGISTER_CHANDLER(ctx->throt.md, chandler_mixdt, 1, ctx->throt.mixratio);
                REGISTER_CHANDLER(ctx->throt.mu, chandler_mixut, 1, ctx->throt.mixratio);
                break;
            }
            break;

        default:
            break;
    }

#if TIM_ONLY
    if (ctx->coatc.cb.handler == NULL)
    {
        if (XPLM_NO_PLUGIN_ID != ctx->coatc.pe_plid)
        {
            ctx->ground.oatc.pe_is_on = XPLMFindDataRef("pilotedge/status/connected");
            ctx->ground.oatc.pe_was_connected = 0;
        }
        else
        {
            if (XPLM_NO_PLUGIN_ID != ctx->coatc.xb_plid)
            {
                if ((ctx->coatc.coatc = XPLMFindCommand("xsquawkbox/voice/ptt")))
                {
                    ndt_log("navP [info]: XSquawkBox detected, \"sim/operation/contact_atc\" mapped to \"xsquawkbox/voice/ptt\"\n");
                }
                ctx->ground.oatc.xb_is_on = XPLMFindDataRef("xsquawkbox/login/status");
                ctx->ground.oatc.xb_was_connected = 0;
            }
            /* no PilotEdge plugin: we disable/remap X-Plane's "contact ATC" command */
            if ((ctx->coatc.cb.command = XPLMFindCommand("sim/operation/contact_atc")))
            {
                REGISTER_CHANDLER(ctx->coatc.cb, chandler_coatc, 1/*before*/, ctx->coatc.coatc);
            }
        }
    }

    /*
     * Kill X-Plane ATC (not needed, may/may not cause crashes in some places).
     * Do it as late as possible (avoid interfering w/init. of X-Plane itself).
     * Doing it too early might have caused a crash in the PilotEdge plugin :(
     *
     * This is all very much guesswork, really :-(
     */
    _DO(1, XPLMSetDataf, 1.0f, "sim/private/controls/perf/kill_atc");

    /*
     * Sound: default to 25% volume for all addons.
     */
    acf_volume_context *volume_context = acf_volume_ctx_get();
    if (volume_context == NULL)
    {
        XPLMSpeakString("default volume error");
        return EINVAL;
    }
    acf_volume_set(volume_context, 0.10f, ctx->info->ac_type);

    /* XP v11 experimental tweaks */
    if (volume_context->x_plane_v11)
    {
        _DO(1, XPLMSetDatai, 0, "sim/private/controls/reno/draw_fft_water");
//      _DO(1, XPLMSetDatai, TDFDRFOR, "sim/private/controls/reno/draw_for_05");
//      _DO(1, XPLMSetDatai, TDFDRCAR, "sim/private/controls/reno/draw_cars_05");
        _DO(1, XPLMSetDatai, TDFDRVEC, "sim/private/controls/reno/draw_vecs_03");
    }
#endif

    return (ctx->first_fcall = 0);
}

static int tliss_fbw_init(refcon_tolifbw *fbw)
{
    if (fbw && fbw->ready == 0)
    {
        if ((fbw->mwcb.command = XPLMFindCommand(   "toliss_airbus/iscs_open")) &&
            (fbw->popup_x      = XPLMFindDataRef("AirbusFBW/PopUpXCoordArray")) &&
            (fbw->popup_y      = XPLMFindDataRef("AirbusFBW/PopUpYCoordArray")))
        {
            fbw->xpliver = XPLMFindDataRef("sim/version/xplane_internal_version");
            REGISTER_CHANDLER(fbw->mwcb, chandler_31isc, 1/*before ToLiSS*/, fbw);
            fbw->ready = 1;
            return 0;
        }
        return -1;
    }
    return 0;
}

static int boing_733_init(refcon_ixeg733 *i33)
{
    if (i33 && i33->ready == 0)
    {
        if ((i33->slat = XPLMFindDataRef("ixeg/733/hydraulics/speedbrake_act")))
        {
            (i33->ready = 1); return 0;
        }
        return -1;
    }
    return 0;
}

static int boing_738_init(refcon_eadt738 *x38)
{
    if (x38 && x38->ready == 0)
    {
        if ((x38->sparm = XPLMFindCommand("x737/speedbrakes/SPEEDBRAKES_ARM" )) &&
            (x38->spret = XPLMFindCommand("x737/speedbrakes/SPEEDBRAKES_DOWN")) &&
            (x38->pt_up = XPLMFindCommand("x737/trim/CAPT_STAB_TRIM_UP_ALL"  )) &&
            (x38->pt_dn = XPLMFindCommand("x737/trim/CAPT_STAB_TRIM_DOWN_ALL")))
        {
            (x38->ready = 1); return 0;
        }
        return -1;
    }
    return 0;
}

static int priv_getdata_i(void *inRefcon)
{
    return *((int*)inRefcon);
}

static void priv_setdata_i(void *inRefcon, int inValue)
{
    *((int*)inRefcon) = inValue;
}

static float priv_getdata_f(void *inRefcon)
{
    return *((float*)inRefcon);
}

static void priv_setdata_f(void *inRefcon, float inValue)
{
    *((float*)inRefcon) = inValue;
}

#undef TBM9_FLIGHT_IDLE_GATE
#undef REGISTER_CHANDLER
#undef UNREGSTR_CHANDLER
#undef CALLOUT_SPEEDBRAK
#undef CALLOUT_FLAPLEVER
#undef CALLOUT_GEARLEVER
#undef AFTER_7X_PATH
#undef A320T_CLMB
#undef A320T_HALF
#undef A320T_IDLE
#undef A320T_TAXI
#undef T_ZERO
#undef _DO
