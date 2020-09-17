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
    chandler_callback mi;
    XPLMDataRef mixratio;

    chandler_callback rp;
    XPLMDataRef rpmratio;

    chandler_callback dn;
    XPLMCommandRef thrdn;

    chandler_callback up;
    chandler_callback ul;
    XPLMCommandRef thrup;

    XPLMDataRef throttle;
    int         acf_type;
} refcon_thrust;

typedef struct
{
    int              ready;
    chandler_callback mwcb;
    XPLMDataRef    popup_x;
    XPLMDataRef    popup_y;
    int            toaltbr;
    int            tolbctr;
    XPLMDataRef    tolb[3];
    XPLMDataRef    pkb_ref;
    XPLMCommandRef h_b_max;
    XPLMCommandRef h_b_reg;
} refcon_qpacfbw;

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
    const char          *auth;
    const char          *desc;
    int                  atyp;
    int            i_disabled;
    int            i_value[2];
    XPLMDataRef    dataref[4];
    XPLMCommandRef command[4];
} refcon_cdu_pop;

typedef struct
{
    void           *assert;
    int  every_ten_seconds;
    XPLMDataRef ground_spd;
    XPLMDataRef auto_p_sts;
    XPLMDataRef auto_t_sts;
    XPLMDataRef elev_m_agl;
    XPLMFlightLoop_f flc_g;
    struct
    {
        chandler_callback preset;
        XPLMDataRef throttle_all;
        XPLMDataRef thrott_array;
        XPLMDataRef onground_any;
        float r_t[2];
        float r_taxi;
        float r_idle;
        int minimums;
    } idle;
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
        XPLMDataRef     view_type;
        XPLMDataRef zulu_time_sec;
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
    XPLMDataRef ap_pmod;
    XPLMDataRef ap_pclb;
    XPLMDataRef to_pclb;
    XPLMDataRef ap_arry;
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
    int        initialized;
    int        first_fcall;
    int        kill_daniel;
    void     *menu_context;
    acf_info_context *info;
    refcon_a319kbc  a319kc;
    refcon_a350kbc  a350kc;
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
        int         var_park_brake;
        XPLMDataRef ref_park_brake;
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
        XPLMFlightLoop_f flc_flaps;
    } callouts;

    struct
    {
        refcon_qpacfbw qpac;
        refcon_ixeg733 i733;
        refcon_eadt738 x738;
    } acfspec;

    struct
    {
        struct
        {
            chandler_callback cb;
        } tur;

        struct
        {
            chandler_callback cb;
        } prk;

        struct
        {
            chandler_callback cb;
        } off;

        struct
        {
            chandler_callback cb;
        } max;

        struct
        {
            chandler_callback cb;
        } reg;

        refcon_braking rc_brk;
    } bking;

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
        XPLMCommandRef sext;
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
        } fwd;

        struct
        {
            chandler_callback cb;
        } rev;

        void          *assert;
        int         n_engines;
        XPLMDataRef prop_mode;
        XPLMCommandRef propdn;
        XPLMCommandRef propup;
        XPLMCommandRef propto;
    } revrs;

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
        chandler_callback at_disc;
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
        struct
        {
            chandler_callback cb;
            chandler_command  cc;
        } disc;

        struct
        {
            chandler_callback cb;
            chandler_command  cc;
        } toga;
    } athr;

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
} chandler_context;

/* Callout default values */
#define CALLOUT_PARKBRAKE (-1)
#define CALLOUT_SPEEDBRAK (-1)
#define CALLOUT_FLAPLEVER (-1)
#define CALLOUT_GEARLEVER (-1)

/* Flap lever position name constants */
static       char  _flap_callout_st[11];
static const char* _flap_names_2POS[10] = {    "up",  "half",  "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_3POS[10] = {    "up",     "1",     "2", "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_A10W[10] = {    "up",    "15",    "30",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_AIB1[10] = {    "up",     "1",     "2",    "3", "full",   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_B190[10] = {    "up",    "17",    "35",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_B52G[10] = {    "up",    "10",    "20",   "30",   "40",   "55",   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_BD5J[10] = {    "up",    "10",    "20",   "30",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_BNG1[10] = {    "up",     "1",     "2",    "5",   "10",   "15",   "25",   "30",   "40",   NULL, };
static const char* _flap_names_BNG2[10] = {    "up",     "1",     "5",   "15",   "20",   "25",   "30",   NULL,   NULL,   NULL, };
static const char* _flap_names_BOM1[10] = {    "up",    "10",    "20",   "30",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_BOM2[10] = {    "up",     "5",    "10",   "15",   "35",   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_C130[10] = {    "up",     "1",     "2",    "3",    "4",    "5",    "6", "full",   NULL,   NULL, };
static const char* _flap_names_C340[10] = {    "up",    "15",    "45",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_CSNA[10] = {    "up",    "10",    "20", "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_DC10[10] = {    "up",     "5",    "15",   "25",   "30",   "35",   "40",   NULL,   NULL,   NULL, };
static const char* _flap_names_NO18[10] = {    "up",     "9",    "22",   "45",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_EMB1[10] = {    "up",     "9",    "18",   "22",   "45",   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_EMB2[10] = {    "up",     "1",     "2",    "3",    "4",    "5", "full",   NULL,   NULL,   NULL, };
static const char* _flap_names_FA7X[10] = {    "up",     "1",     "2",    "3",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_HA4T[10] = {    "up",    "12",    "20",   "35",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_MD80[10] = {    "up",     "0",     "5",   "11",   "15",   "28",   "40",   NULL,   NULL,   NULL, };
static const char* _flap_names_PC12[10] = {    "up",    "15",    "30",   "40",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_PIPA[10] = {    "up",    "15",    "30",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_PIPR[10] = {    "up",    "10",    "25",   "40",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_TBM8[10] = { "8 5 0",   "up",  "half",  "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
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

/* thrust reverser mode constants */
static int _PROPMODE_FWD[8] = { 1, 1, 1, 1, 1, 1, 1, 1, };
static int _PROPMODE_REV[8] = { 3, 3, 3, 3, 3, 3, 3, 3, };
static int* propmode_fwd_get(void)
{
    return _PROPMODE_FWD;
}
static int* propmode_rev_get(void)
{
    return _PROPMODE_REV;
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
static int chandler_p_max(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_p_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_b_max(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_b_reg(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_swtch(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_apclb(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_sp_ex(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_sp_re(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_pt_up(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_pt_dn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_at_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_at_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_rt_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_rt_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_r_fwd(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_r_rev(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_mixdn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_rpmdn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_thrdn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_thrup(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_thrul(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
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
static int chandler_idleb(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_apbef(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_apaft(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_p2vvi(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_coatc(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static float flc_flap_func (                                        float, float, int, void*);
static float flc_oatc_func (                                        float, float, int, void*);
static float gnd_stab_hdlr (                                        float, float, int, void*);
static int   first_fcall_do(                                           chandler_context *ctx);
static int   aibus_fbw_init(                                             refcon_qpacfbw *fbw);
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
    ctx->callouts.var_park_brake = CALLOUT_PARKBRAKE;
    ctx->callouts.ref_park_brake = XPLMRegisterDataAccessor("navP/callouts/park_brake",
                                                            xplmType_Int, 1,
                                                            &priv_getdata_i, &priv_setdata_i,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            NULL, NULL,
                                                            &ctx->callouts.var_park_brake,
                                                            &ctx->callouts.var_park_brake);
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
    if (!ctx->callouts.ref_park_brake ||
        !ctx->callouts.ref_speedbrake ||
        !ctx->callouts.ref_flap_lever ||
        !ctx->callouts.ref_gear_lever)
    {
        goto fail;
    }

    /* Field of view save/restore */
    if ((ctx->fov.nonp = XPLMFindDataRef("sim/graphics/settings/non_proportional_vertical_FOV")) == NULL ||
        (ctx->fov.data = XPLMFindDataRef(                "sim/graphics/view/field_of_view_deg")) == NULL)
    {
        goto fail;
    }

    /* Private datarefs: braking */
    ctx->bking.rc_brk.p_b_int = XPLMRegisterDataAccessor("private/temp/integer/refcon_braking",
                                                         xplmType_Int, 1,
                                                         &priv_getdata_i, &priv_setdata_i,
                                                         NULL, NULL,
                                                         NULL, NULL,
                                                         NULL, NULL,
                                                         NULL, NULL,
                                                         NULL, NULL,
                                                         &ctx->bking.rc_brk.int_var,
                                                         &ctx->bking.rc_brk.int_var);
    ctx->bking.rc_brk.p_b_flt = XPLMRegisterDataAccessor("private/temp/floatpt/refcon_braking",
                                                         xplmType_Float, 1,
                                                         NULL, NULL,
                                                         &priv_getdata_f, &priv_setdata_f,
                                                         NULL, NULL,
                                                         NULL, NULL,
                                                         NULL, NULL,
                                                         NULL, NULL,
                                                         &ctx->bking.rc_brk.flt_var,
                                                         &ctx->bking.rc_brk.flt_var);
    if (!ctx->bking.rc_brk.p_b_int || !ctx->bking.rc_brk.p_b_flt)
    {
        goto fail;
    }

    /* Custom commands: braking */
    ctx->bking.tur.cb.command = XPLMCreateCommand("navP/turnaround_set", "friendly cold & dark");
    ctx->bking.prk.cb.command = XPLMCreateCommand("navP/brakes/parking", "apply max. park brake");
    ctx->bking.off.cb.command = XPLMCreateCommand("navP/brakes/release", "release parking brake");
    ctx->bking.max.cb.command = XPLMCreateCommand("navP/brakes/maximum", "maximum braking action");
    ctx->bking.reg.cb.command = XPLMCreateCommand("navP/brakes/regular", "regular braking action");
    ctx->bking.rc_brk.p_b_rat = XPLMFindDataRef  ("sim/cockpit2/controls/parking_brake_ratio");
    ctx->bking.rc_brk.r_b_rat = XPLMFindDataRef  ("sim/cockpit2/controls/right_brake_ratio");
    ctx->bking.rc_brk.l_b_rat = XPLMFindDataRef  ("sim/cockpit2/controls/left_brake_ratio");
    ctx->bking.rc_brk.a_b_lev = XPLMFindDataRef  ("sim/cockpit2/switches/auto_brake_level");
    ctx->bking.rc_brk.abto    = XPLMFindCommand  ("sim/flight_controls/brakes_toggle_auto");
    if (!ctx->bking.tur.cb.command ||
        !ctx->bking.prk.cb.command || !ctx->bking.off.cb.command ||
        !ctx->bking.max.cb.command || !ctx->bking.reg.cb.command ||
        !ctx->bking.rc_brk.p_b_rat || !XPLMCanWriteDataRef(ctx->bking.rc_brk.p_b_rat) ||
        !ctx->bking.rc_brk.r_b_rat || !XPLMCanWriteDataRef(ctx->bking.rc_brk.r_b_rat) ||
        !ctx->bking.rc_brk.l_b_rat || !XPLMCanWriteDataRef(ctx->bking.rc_brk.l_b_rat) ||
        !ctx->bking.rc_brk.a_b_lev || !XPLMCanWriteDataRef(ctx->bking.rc_brk.a_b_lev) || !ctx->bking.rc_brk.abto)
    {
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->bking.tur.cb, chandler_turna, 0, ctx);
        REGISTER_CHANDLER(ctx->bking.prk.cb, chandler_p_max, 0, ctx);
        REGISTER_CHANDLER(ctx->bking.off.cb, chandler_p_off, 0, ctx);
        REGISTER_CHANDLER(ctx->bking.max.cb, chandler_b_max, 0, ctx);
        REGISTER_CHANDLER(ctx->bking.reg.cb, chandler_b_reg, 0, ctx);
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
    ctx->revrs.fwd.cb.command = XPLMCreateCommand("navP/thrust/forward", "stow thrust reversers");
    ctx->revrs.rev.cb.command = XPLMCreateCommand("navP/thrust/reverse", "deploy thrust reversers");
    ctx->revrs.prop_mode      = XPLMFindDataRef  ("sim/cockpit2/engine/actuators/prop_mode");
    ctx->revrs.propto         = XPLMFindCommand  ("sim/engines/thrust_reverse_toggle");
    if (!ctx->revrs.fwd.cb.command || !ctx->revrs.rev.cb.command ||
        !ctx->revrs.prop_mode      || !ctx->revrs.propto)
    {
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->revrs.fwd.cb, chandler_r_fwd, 0, ctx);
        REGISTER_CHANDLER(ctx->revrs.rev.cb, chandler_r_rev, 0, ctx);
    }

    /* Custom commands and handlers: thrust control */
    ctx->throt.  rpmratio = XPLMFindDataRef  ("sim/cockpit2/engine/actuators/prop_rotation_speed_rad_sec_all");
    ctx->throt.  mixratio = XPLMFindDataRef  ("sim/cockpit2/engine/actuators/mixture_ratio_all");
    ctx->throt.dn.command = XPLMCreateCommand("navP/thrust/dn_once", "throttle down once");
    ctx->throt.up.command = XPLMCreateCommand("navP/thrust/up_once", "throttle up once");
    ctx->throt.ul.command = XPLMCreateCommand("navP/thrust/up_lots", "throttle up 12pc");
    ctx->throt.mi.command = XPLMFindCommand  ("sim/engines/mixture_down");
    ctx->throt.rp.command = XPLMFindCommand  ("sim/engines/prop_down");
    ctx->throt.     thrdn = XPLMFindCommand  ("sim/engines/throttle_down");
    ctx->throt.     thrup = XPLMFindCommand  ("sim/engines/throttle_up");
    if (!ctx->throt.dn.command || !ctx->throt.thrdn || !ctx->throt.mi.command || !ctx->throt.mixratio ||
        !ctx->throt.up.command || !ctx->throt.thrup || !ctx->throt.rp.command || !ctx->throt.rpmratio ||
        !ctx->throt.ul.command)
    {
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->throt.dn, chandler_thrdn, 0, &ctx->throt);
        REGISTER_CHANDLER(ctx->throt.up, chandler_thrup, 0, &ctx->throt);
        REGISTER_CHANDLER(ctx->throt.ul, chandler_thrul, 0, &ctx->throt);
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
    ctx->asrt.at_disc.command = XPLMCreateCommand( "private/ff320/at_disc", "NOT TO BE USED");
    ctx->otto.ffst.cb.command = XPLMCreateCommand( "private/ffsts/ap_cmdl", "NOT TO BE USED");
    ctx->otto.clmb.cb.command = XPLMCreateCommand( "navP/switches/ap_clmb", "A/P pitch: CLB");
    ctx->otto.conn.cb.command = XPLMCreateCommand( "navP/switches/ap_conn", "A/P engagement");
    ctx->otto.disc.cb.command = XPLMCreateCommand( "navP/switches/ap_disc", "A/P disconnect");
    ctx->athr.disc.cb.command = XPLMCreateCommand( "navP/switches/at_disc", "A/T disconnect");
    ctx->athr.toga.cb.command = XPLMCreateCommand( "navP/switches/at_toga", "A/T takeoff/GA");
    ctx->otto.clmb.rc.ap_pclb = XPLMFindDataRef("sim/cockpit2/autopilot/sync_hold_pitch_deg");
    ctx->otto.clmb.rc.to_pclb = XPLMFindDataRef(     "sim/cockpit2/autopilot/TOGA_pitch_deg");
    ctx->otto.clmb.rc.ap_pmod = XPLMFindDataRef(       "sim/cockpit2/autopilot/pitch_status");
    if (!ctx->asrt.ap_conn.command ||
        !ctx->asrt.ap_disc.command ||
        !ctx->asrt.at_disc.command ||
        !ctx->otto.ffst.cb.command ||
        !ctx->otto.clmb.cb.command ||
        !ctx->otto.conn.cb.command ||
        !ctx->otto.disc.cb.command ||
        !ctx->athr.disc.cb.command ||
        !ctx->athr.toga.cb.command ||
        !ctx->otto.clmb.rc.ap_pclb ||
        !ctx->otto.clmb.rc.to_pclb ||
        !ctx->otto.clmb.rc.ap_pmod)
    {
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->asrt.ap_conn, chandler_32apc, 0, &ctx->info->assert);
        REGISTER_CHANDLER(ctx->asrt.ap_disc, chandler_32apd, 0, &ctx->info->assert);
        REGISTER_CHANDLER(ctx->asrt.at_disc, chandler_32atd, 0, &ctx->info->assert);
        REGISTER_CHANDLER(ctx->otto.ffst.cb, chandler_ffap1, 0, &ctx->otto.ffst.dr);
        REGISTER_CHANDLER(ctx->otto.clmb.cb, chandler_apclb, 0, &ctx->otto.clmb.rc);
        REGISTER_CHANDLER(ctx->otto.conn.cb, chandler_swtch, 0, &ctx->otto.conn.cc);
        REGISTER_CHANDLER(ctx->otto.disc.cb, chandler_swtch, 0, &ctx->otto.disc.cc);
        REGISTER_CHANDLER(ctx->athr.disc.cb, chandler_swtch, 0, &ctx->athr.disc.cc);
        REGISTER_CHANDLER(ctx->athr.toga.cb, chandler_swtch, 0, &ctx->athr.toga.cc);
    }

    /* Default commands' handlers: flaps up or down */
    ctx->callouts.cb_flapu.command = XPLMFindCommand("sim/flight_controls/flaps_up");
    ctx->callouts.cb_flapd.command = XPLMFindCommand("sim/flight_controls/flaps_down");
    ctx->callouts.ref_flap_ratio   = XPLMFindDataRef("sim/cockpit2/controls/flap_ratio");
    if (!ctx->callouts.cb_flapu.command ||
        !ctx->callouts.cb_flapd.command ||
        !ctx->callouts.ref_flap_ratio)
    {
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->callouts.cb_flapu, chandler_flchg, 0, ctx);
        REGISTER_CHANDLER(ctx->callouts.cb_flapd, chandler_flchg, 0, ctx);
    }
    XPLMRegisterFlightLoopCallback((ctx->callouts.flc_flaps = &flc_flap_func), 0, NULL);

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
        goto fail;
    }

    /* Custom command: pop-up Control Display Unit */
    ctx->mcdu.cb.command = XPLMCreateCommand("navP/switches/cdu_toggle", "CDU pop-up/down");
    if (!ctx->mcdu.cb.command)
    {
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->mcdu.cb, chandler_mcdup, 0, &ctx->mcdu.rc);
    }

    /* Custom ground stabilization system (via flight loop callback) */
    ctx->ground.ground_spd          = XPLMFindDataRef  ("sim/flightmodel/position/groundspeed");
    ctx->ground.auto_t_sts          = XPLMFindDataRef  ("sim/cockpit2/autopilot/autothrottle_enabled");
    ctx->ground.auto_p_sts          = XPLMFindDataRef  ("sim/cockpit2/autopilot/servos_on");
    ctx->ground.elev_m_agl          = XPLMFindDataRef  ("sim/flightmodel/position/y_agl");
    ctx->ground.time.view_type      = XPLMFindDataRef  ("sim/graphics/view/view_type");
    ctx->ground.time.zulu_time_sec  = XPLMFindDataRef  ("sim/time/zulu_time_sec");
    ctx->ground.oatc.vol_com0       = XPLMFindDataRef  ("sim/operation/sound/radio_volume_ratio");
    ctx->ground.oatc.vol_com1       = XPLMFindDataRef  ("sim/cockpit2/radios/actuators/audio_volume_com1");
    ctx->ground.oatc.vol_com2       = XPLMFindDataRef  ("sim/cockpit2/radios/actuators/audio_volume_com2");
    ctx->ground.idle.onground_any   = XPLMFindDataRef  ("sim/flightmodel/failures/onground_any");
    ctx->ground.idle.throttle_all   = XPLMFindDataRef  ("sim/cockpit2/engine/actuators/throttle_ratio_all");
    ctx->ground.idle.preset.command = XPLMCreateCommand("navP/thrust/idle_boost", "apply just a bit of throttle");
    if (!ctx->ground.ground_spd         ||
        !ctx->ground.auto_t_sts         ||
        !ctx->ground.auto_p_sts         ||
        !ctx->ground.elev_m_agl         ||
        !ctx->ground.time.view_type     ||
        !ctx->ground.time.zulu_time_sec ||
        !ctx->ground.oatc.vol_com0      ||
        !ctx->ground.oatc.vol_com1      ||
        !ctx->ground.oatc.vol_com2      ||
        !ctx->ground.idle.throttle_all  ||
        !ctx->ground.idle.preset.command)
    {
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->ground.idle.preset, chandler_idleb, 0, &ctx->ground);
        ctx->bking.rc_brk.g_speed = ctx->ground.ground_spd;
    }

    /* all good */
    return ctx;

fail:
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
    UNREGSTR_CHANDLER(ctx->bking.   tur.cb);
    UNREGSTR_CHANDLER(ctx->bking.   prk.cb);
    UNREGSTR_CHANDLER(ctx->bking.   off.cb);
    UNREGSTR_CHANDLER(ctx->bking.   max.cb);
    UNREGSTR_CHANDLER(ctx->bking.   reg.cb);
    UNREGSTR_CHANDLER(ctx->spbrk.   ext.cb);
    UNREGSTR_CHANDLER(ctx->spbrk.   ret.cb);
    UNREGSTR_CHANDLER(ctx->trims.pch.up.cb);
    UNREGSTR_CHANDLER(ctx->trims.pch.dn.cb);
    UNREGSTR_CHANDLER(ctx->trims.ail.lt.cb);
    UNREGSTR_CHANDLER(ctx->trims.ail.rt.cb);
    UNREGSTR_CHANDLER(ctx->trims.rud.lt.cb);
    UNREGSTR_CHANDLER(ctx->trims.rud.rt.cb);
    UNREGSTR_CHANDLER(ctx->revrs.   fwd.cb);
    UNREGSTR_CHANDLER(ctx->revrs.   rev.cb);
    UNREGSTR_CHANDLER(ctx->throt.       mi);
    UNREGSTR_CHANDLER(ctx->throt.       rp);
    UNREGSTR_CHANDLER(ctx->throt.       dn);
    UNREGSTR_CHANDLER(ctx->throt.       up);
    UNREGSTR_CHANDLER(ctx->throt.       ul);
    UNREGSTR_CHANDLER(ctx->asrt.   ap_conn);
    UNREGSTR_CHANDLER(ctx->asrt.   ap_disc);
    UNREGSTR_CHANDLER(ctx->asrt.   at_disc);
    UNREGSTR_CHANDLER(ctx->otto.   ffst.cb);
    UNREGSTR_CHANDLER(ctx->otto.   conn.cb);
    UNREGSTR_CHANDLER(ctx->otto.   disc.cb);
    UNREGSTR_CHANDLER(ctx->athr.   disc.cb);
    UNREGSTR_CHANDLER(ctx->athr.   toga.cb);
    UNREGSTR_CHANDLER(ctx->views.  prev.cb);
    UNREGSTR_CHANDLER(ctx->views.  next.cb);
    for (int i = 0; i < 10; i++)
    {
        UNREGSTR_CHANDLER(ctx->views.cbs[i]);
    }
    UNREGSTR_CHANDLER(ctx->gear.landing_gear_toggle);
    UNREGSTR_CHANDLER(ctx->gear.  landing_gear_down);
    UNREGSTR_CHANDLER(ctx->gear.    landing_gear_up);
    UNREGSTR_CHANDLER(ctx->ground.      idle.preset);
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

    /* …and all datarefs */
    if (ctx->bking.rc_brk.p_b_int)
    {
        XPLMUnregisterDataAccessor(ctx->bking.rc_brk.p_b_int);
        ctx->bking.rc_brk.p_b_int = NULL;
    }
    if (ctx->bking.rc_brk.p_b_flt)
    {
        XPLMUnregisterDataAccessor(ctx->bking.rc_brk.p_b_flt);
        ctx->bking.rc_brk.p_b_flt = NULL;
    }
    if (ctx->callouts.ref_park_brake)
    {
        XPLMUnregisterDataAccessor(ctx->callouts.ref_park_brake);
        ctx->callouts.ref_park_brake = NULL;
    }
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
    ctx->bking.rc_brk.assert = ctx->gear.assert = ctx->ground.assert = ctx->revrs.assert = NULL;
    ctx->gear.has_retractable_gear = -1; ctx->revrs.n_engines = -1; acf_type_info_reset();

    /* Don't use 3rd-party commands/datarefs until we know the plane we're in */
    ctx->bking.rc_brk.use_pkb = 1;
    ctx->acfspec.  qpac.ready = 0;
    ctx->acfspec.  i733.ready = 0;
    ctx->acfspec.  x738.ready = 0;
    ctx->revrs.        propdn = NULL;
    ctx->revrs.        propup = NULL;
    ctx->otto.ffst.        dr = NULL;
    ctx->otto.conn.cc.   name = NULL;
    ctx->otto.disc.cc.   name = NULL;
    ctx->athr.disc.cc.   name = NULL;
    ctx->athr.toga.cc.   name = NULL;
    ctx->bking.rc_brk.rg.name = NULL;
    ctx->bking.rc_brk.mx.name = NULL;
    ctx->bking.rc_brk.ro.name = NULL;
    ctx->otto.clmb.rc.ap_arry = NULL;
    ctx->throt.      throttle = NULL;
    ctx->throt.      acf_type = ACF_TYP_GENERIC;

    /* Reset some datarefs to match X-Plane's defaults at startup */
    _DO(1, XPLMSetDatai, 1, "sim/cockpit2/radios/actuators/com1_power");
    _DO(1, XPLMSetDatai, 1, "sim/cockpit2/radios/actuators/com2_power");
    _DO(1, XPLMSetDatai, 1, "sim/cockpit2/radios/actuators/nav1_power");
    _DO(1, XPLMSetDatai, 1, "sim/cockpit2/radios/actuators/nav2_power");
    _DO(1, XPLMSetDatai, 0, "sim/graphics/misc/kill_map_fms_line");

    /* Reset turnaround-enabled flight loop callback */
    if (ctx->ground.flc_g)
    {
        XPLMUnregisterFlightLoopCallback(ctx->ground.flc_g, &ctx->ground);
        ctx->ground.idle.thrott_array = NULL;
        ctx->ground.idle.minimums = 0;
        ctx->ground.flc_g = NULL;
    }

    /* Unregister aircraft-specific command handlers */
    UNREGSTR_CHANDLER(ctx->acfspec.qpac.mwcb);
    UNREGSTR_CHANDLER(ctx->apd.          aft);
    UNREGSTR_CHANDLER(ctx->apd.          bef);
    UNREGSTR_CHANDLER(ctx->vvi.           dn);
    UNREGSTR_CHANDLER(ctx->vvi.           up);
    UNREGSTR_CHANDLER(ctx->vvi.           pd);
    UNREGSTR_CHANDLER(ctx->vvi.           pu);
    UNREGSTR_CHANDLER(ctx->throt.         mi);
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
    ndt_log("navP [info]: nvp_chandlers_reset OK\n"); return (ctx->initialized = 0);
}

void nvp_chandlers_setmnu(void *inContext, void *inMenu)
{
    chandler_context *ctx = inContext;
    if (ctx)
    {
        ctx->menu_context = inMenu;
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

static void print_aircft_info(acf_info_context *info)
{
    ndt_log("navP [info]: %s (\"%s\", \"%s\", \"%s\", \"%s\", \"%s\")\n",
            acf_type_get_name(info->ac_type),
            info->icaoid, info->tailnb, info->author, info->descrp, info->afname);
}

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

    /* aircraft-specific custom commands and miscellaneous stuff */
    switch (ctx->throt.acf_type = ctx->info->ac_type)
    {
        case ACF_TYP_A320_FF:
            ctx->otto.conn.cc.name   = "private/ff320/ap_conn";
            ctx->otto.disc.cc.name   = "private/ff320/ap_disc";
            ctx->athr.disc.cc.name   = "private/ff320/at_disc";
            ctx->bking.rc_brk.assert =
            ctx->gear.assert         =
            ctx->ground.assert       =
            ctx->revrs.assert        = &ctx->info->assert;
            break;

        case ACF_TYP_A319_TL:
        case ACF_TYP_A321_TL:
            ctx->otto.disc.cc.name = "toliss_airbus/ap_disc_left_stick";
            ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
            ctx->otto.conn.cc.name = "toliss_airbus/ap1_push";
            ctx->throt.throttle = XPLMFindDataRef("AirbusFBW/throttle_input");
            break;

        case ACF_TYP_A350_FF:
//          ctx->otto.disc.cc.name = "airbus_qpac/ap_disc_left_stick"; // only exists in version 1.6 or later
            ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
            ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
            ctx->otto.conn.cc.name = "airbus_qpac/ap1_push";
            ctx->throt.throttle = XPLMFindDataRef("AirbusFBW/throttle_input");;
            break;

        case ACF_TYP_B737_EA:
            ctx->otto.disc.cc.name = "x737/yoke/capt_AP_DISENG_BTN";
            ctx->athr.disc.cc.name = "x737/mcp/ATHR_ARM_TOGGLE";
            ctx->athr.toga.cc.name = "x737/mcp/TOGA_TOGGLE";
            ctx->otto.conn.cc.name = "x737/mcp/CMDA_TOGGLE";
            ctx->throt.throttle = ctx->ground.idle.throttle_all;
            break;

        case ACF_TYP_B737_XG:
            ctx->otto.conn.cc.name = "ixeg/733/autopilot/AP_A_cmd_toggle";
            ctx->otto.disc.cc.name = "ixeg/733/autopilot/AP_disengage";
            ctx->athr.disc.cc.name = "ixeg/733/autopilot/at_disengage";
            ctx->athr.toga.cc.name = "sim/engines/TOGA_power";
            ctx->throt.throttle = ctx->ground.idle.throttle_all;
            ctx->bking.rc_brk.use_pkb = 0;
            break;

        case ACF_TYP_B757_FF:
        case ACF_TYP_B767_FF:
            ctx->throt.throttle = ctx->ground.idle.throttle_all;
            ctx->otto.conn.cc.name = "private/ffsts/ap_cmdl";
            ctx->otto.disc.cc.name = "1-sim/comm/AP/ap_disc";
            ctx->athr.disc.cc.name = "1-sim/comm/AP/at_disc";
            ctx->athr.toga.cc.name = "1-sim/comm/AP/at_toga";
            ctx->bking.rc_brk.use_pkb = 0;
            break;

        case ACF_TYP_B777_FF:
            ctx->throt.throttle = ctx->ground.idle.throttle_all;
            ctx->otto.disc.cc.name = "777/ap_disc";
            ctx->athr.disc.cc.name = "777/at_disc";
            ctx->athr.toga.cc.name = "777/at_toga";
            ctx->bking.rc_brk.use_pkb = 0;
            break;

        case ACF_TYP_EMBE_SS:
            ctx->throt.throttle = ctx->ground.idle.throttle_all;
            ctx->otto.conn.cc.name = "SSG/EJET/MCP/AP_COMM";
            ctx->otto.disc.cc.name = "SSG/EJET/MCP/AP_COMM";
            ctx->athr.disc.cc.name = "SSG/EJET/MCP/AT_COMM";
            ctx->athr.toga.cc.name = "SSG/EJET/MCP/Toga";
            break;

        case ACF_TYP_EMBE_XC:
        case ACF_TYP_HA4T_RW:
            ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
            ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
            ctx->athr.toga.cc.name = "sim/autopilot/autothrottle_on";
            ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            ctx->throt.throttle = ctx->ground.idle.throttle_all;
            break;

        case ACF_TYP_LEGA_XC:
            ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
            ctx->otto.disc.cc.name = "sim/autopilot/servos_off_any";
            ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            ctx->athr.toga.cc.name = "navP/switches/ap_clmb";
            ctx->throt.throttle = ctx->ground.idle.throttle_all;
            break;

        case ACF_TYP_MD80_RO:
            if ((ctx->bking.rc_brk.protate = XPLMFindDataRef("Rotate/md80/systems/parking_brake_toggle_clicked")))
            {
                // special case: no need for deferred initialization with this command
                (ctx->bking.rc_brk.ro.name = "Rotate/md80/systems/parking_brake_toggle");
                (ctx->bking.rc_brk.ro.xpcr = XPLMFindCommand(ctx->bking.rc_brk.ro.name));
                if (ctx->bking.rc_brk.ro.xpcr == NULL)
                {
                    ctx->bking.rc_brk.ro.name  = NULL;
                }
            }
            ctx->athr.toga.cc.name = "Rotate/md80/autopilot/to_ga_button";
            ctx->athr.disc.cc.name = "Rotate/md80/autopilot/at_disc";
            ctx->otto.disc.cc.name = "Rotate/md80/autopilot/ap_disc";
            ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            ctx->throt.throttle = ctx->ground.idle.throttle_all;
            break;

        case ACF_TYP_CL30_DD:
            ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
//          ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
//          ctx->athr.toga.cc.name = "sim/autopilot/autothrottle_on";
            ctx->athr.toga.cc.name = "cl300/mach_hold"; // toggle
            ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            ctx->throt.throttle = ctx->ground.idle.throttle_all;
            break;

        case ACF_TYP_GENERIC:
        {
            if (ctx->info->has_rvrs_thr == -1) // XXX: prop-driven w/out reverse
            {
                ctx->revrs.propdn = XPLMFindCommand("sim/engines/prop_down");
                ctx->revrs.propup = XPLMFindCommand("sim/engines/prop_up");
            }
            if (ctx->info->has_auto_thr == 1)
            {
                ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
                ctx->athr.toga.cc.name = "sim/autopilot/autothrottle_on";
            }
            if (NULL != XPLMFindDataRef("sim/version/xplane_internal_version")) // lazy XP11+ detection
            {
                ctx->otto.disc.cc.name = "sim/autopilot/servos_off_any";
            }
            else
            {
                ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
            }
            ctx->otto.conn.cc.name = "sim/autopilot/servos_on";
            ctx->throt.throttle = ctx->ground.idle.throttle_all;
            break;
        }

        default: // not generic but no usable commands
            break;
    }
    // new addon type: clear refs
    ctx->otto.conn.cc.xpcr = NULL;
    ctx->otto.disc.cc.xpcr = NULL;
    ctx->athr.disc.cc.xpcr = NULL;

    /* plane-specific braking ratios */
    if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("com.simcoders.rep"))
    {
        ctx->bking.rc_brk.rtio[0] = 1.50f / 3.10f;
        ctx->bking.rc_brk.rtio[1] = 2.25f / 3.10f;
        ctx->bking.rc_brk.rtio[2] = 3.00f / 3.10f;
    }
    else if (ctx->info->ac_type == ACF_TYP_A320_FF)
    {
        ctx->bking.rc_brk.rtio[0] = 2.00f / 3.10f;
        ctx->bking.rc_brk.rtio[1] = 2.50f / 3.10f;
        ctx->bking.rc_brk.rtio[2] = 3.00f / 3.10f;
    }
    else // default values
    {
        ctx->bking.rc_brk.rtio[0] = 1.00f / 3.10f;
        ctx->bking.rc_brk.rtio[1] = 2.00f / 3.10f;
        ctx->bking.rc_brk.rtio[2] = 3.00f / 3.10f;
    }

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
    ctx->mcdu.rc.i_disabled = -1;
    ctx->mcdu.rc.auth = ctx->info->author;
    ctx->mcdu.rc.desc = ctx->info->descrp;
    ctx->mcdu.rc.atyp = ctx->info->ac_type;

    /* for the gear handle callouts */
    ctx->gear.callouts.atype = ctx->info->ac_type;

    /* for the reverse thrust commands */
     ctx->revrs.n_engines = ctx->info->engine_count;

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
        int speak = XPLMGetDatai(ctx->callouts.ref_park_brake);
        /* this can happen after calling XPLMReloadPlugins() */
        if (ctx->initialized == 0)
        {
            nvp_chandlers_reset (inRefcon);
            nvp_chandlers_update(inRefcon);
        }
        /*
         * Do any additional aircraft-specific stuff that can't be done earlier.
         */
        if (ctx->info->ac_type & ACF_TYP_MASK_QPC)
        {
            if (ctx->acfspec.qpac.ready == 0)
            {
                aibus_fbw_init(&ctx->acfspec.qpac);
            }
        }
        if (ctx->first_fcall)
        {
            // if set to automatic, callouts become enabled on first turnaround
            if (XPLMGetDatai(ctx->callouts.ref_park_brake) == -1)
            {
                XPLMSetDatai(ctx->callouts.ref_park_brake, (speak = 1));
            }
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
                if (XPLMGetDatai(ctx->fov.nonp) == 0)
                {
                    ctx->fov.float_value = XPLMGetDataf(ctx->fov.data);
                    ctx->fov.round_value = roundf(ctx->fov.float_value);
                }
                XPLMCommandOnce(ctx->views.cbs[1].command);
                XPLMSpeakString("turn around set");
            }
            XPLMSetDatai    (ctx->callouts.ref_park_brake, 0);
            XPLMCommandOnce (ctx->      bking.prk.cb.command);
            XPLMSetDatai(ctx->callouts.ref_park_brake, speak);
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
 * action: set or unset parking brake.
 *
 * rationale: X-Plane only has a toggle for this :(
 */
static int chandler_p_max(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        acf_type_info_acf_ctx_init();
        chandler_context *ctx = inRefcon;
        refcon_braking   *rcb = &ctx->bking.rc_brk;
        assert_context   *rca = rcb->assert;
        int speak = XPLMGetDatai(ctx->callouts.ref_park_brake);
        if (ctx->info->ac_type & ACF_TYP_MASK_QPC)
        {
            if (ctx->acfspec.qpac.ready == 0)
            {
                aibus_fbw_init(&ctx->acfspec.qpac);
            }
            if (ctx->acfspec.qpac.ready)
            {
                // the FlightFactor-QPAC A350 has its parking brake dataref inverted
                XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, (ctx->info->ac_type != ACF_TYP_A350_FF));
                XPLMSetDatai(rcb->p_b_int,              (ctx->info->ac_type != ACF_TYP_A350_FF));
                if (speak > 0) XPLMSpeakString("park brake set");
            }
            return 0;
        }
        if (rcb->ro.name)
        {
            if (XPLMGetDatai(rcb->protate) != 1)
            {
                XPLMCommandOnce(rcb->ro.xpcr);
            }
        }
        else if (rca)
        {
            if (rca->initialized && XPLMGetDataf(rcb->p_b_rat) < 0.5f)
            {
                XPLMCommandOnce(rca->dat.p_brk_toggle);
            }
        }
        else
        {
            XPLMSetDataf(rcb->p_b_rat, 1.0f);
            XPLMSetDataf(rcb->p_b_flt, 1.0f);
        }
        if (speak > 0) XPLMSpeakString("park brake set");
    }
    return 0;
}

static int chandler_p_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        acf_type_info_acf_ctx_init();
        chandler_context *ctx = inRefcon;
        refcon_braking   *rcb = &ctx->bking.rc_brk;
        assert_context   *rca = rcb->assert;
        int speak = XPLMGetDatai(ctx->callouts.ref_park_brake);
        if (ctx->info->ac_type & ACF_TYP_MASK_QPC)
        {
            if (ctx->acfspec.qpac.ready == 0)
            {
                aibus_fbw_init(&ctx->acfspec.qpac);
            }
            if (ctx->acfspec.qpac.ready)
            {
                // the FlightFactor-QPAC A350 has its parking brake dataref inverted
                XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, (ctx->info->ac_type == ACF_TYP_A350_FF));
                XPLMSetDatai(rcb->p_b_int,              (ctx->info->ac_type == ACF_TYP_A350_FF));
                if (speak > 0) XPLMSpeakString("park brake released");
            }
            return 0;
        }
        if (rcb->ro.name)
        {
            if (XPLMGetDatai(rcb->protate) != 0)
            {
                XPLMCommandOnce(rcb->ro.xpcr);
            }
        }
        else if (rca)
        {
            if (rca->initialized && XPLMGetDataf(rcb->p_b_rat) > 0.5f)
            {
                XPLMCommandOnce(rca->dat.p_brk_toggle);
            }
        }
        else
        {
            XPLMSetDataf(rcb->p_b_rat, 0.0f);
            XPLMSetDataf(rcb->p_b_flt, 0.0f);
        }
        if (speak > 0) XPLMSpeakString("park brake released");
    }
    return 0;
}

/*
 * action: apply symmetrical L/R braking w/out applying the parking brake.
 *
 * rationale: same braking effect, but no interference with parking brake
 *            operation (practical, one can set or unset said brake while
 *            simultaneously applying brake pressure).
 */
static int chandler_b_max(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    acf_type_info_acf_ctx_init();
    chandler_context *ctx = inRefcon;
    refcon_braking *rcb = &ctx->bking.rc_brk;
    float g_speed = XPLMGetDataf(rcb->g_speed) * 3.6f / 1.852f;
    assert_context *rca = rcb->assert;
    if (rca)
    {
        if (rca->initialized) switch (inPhase)
        {
            case xplm_CommandBegin:
                XPLMSetDataf(rca->dat.acf_brake_force, rca->dat.acf_brake_force_nomin);
                XPLMCommandBegin(rca->dat.h_brk_mximum);
                return 0;
            case xplm_CommandContinue:
                XPLMSetDataf(rca->dat.acf_brake_force, rca->dat.acf_brake_force_nomin);
                return 0;
            default:
                XPLMCommandEnd(rca->dat.h_brk_mximum);
                XPLMSetDataf(rca->dat.acf_brake_force, rca->dat.acf_brake_force_nomin);
                return 0;
        }
        return 0;
    }
    if (ctx->info->ac_type & ACF_TYP_MASK_QPC)
    {
        if (ctx->acfspec.qpac.ready == 0)
        {
            aibus_fbw_init(&ctx->acfspec.qpac);
        }
        if (ctx->acfspec.qpac.ready)
        {
            switch (inPhase)
            {
                case xplm_CommandBegin: // release parkbrake on manual brake application
                    // the FlightFactor-QPAC A350 has its parking brake dataref inverted
                    XPLMSetDatai(rcb->             p_b_int, (ctx->info->ac_type == ACF_TYP_A350_FF));
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, (ctx->info->ac_type == ACF_TYP_A350_FF));
                    if (ctx->acfspec.qpac.tolb[2])
                    {
                        ctx->acfspec.qpac.tolbctr = 0;
                        XPLMSetDataf(ctx->acfspec.qpac.tolb[0], 1.0f);
                        XPLMSetDataf(ctx->acfspec.qpac.tolb[1], 1.0f);
                        return 0;
                    }
                    if (rcb->use_pkb == 0)
                    {
                        XPLMSetDataf(rcb->l_b_rat, 1.0f);
                        XPLMSetDataf(rcb->r_b_rat, 1.0f);
                        return 0;
                    }
                    if (ctx->acfspec.qpac.h_b_max)
                    {
                        if (g_speed > 3.0f)
                        {
                            XPLMCommandBegin((rcb->pcmd = ctx->acfspec.qpac.h_b_max));
                            return 0;
                        }
                        else
                        {
                            rcb->pcmd = NULL;
                        }
                        // when aircraft is stationary, hold commands act as park
                        // brake toggles instead, don't use them but fall through
                    }
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, (ctx->info->ac_type != ACF_TYP_A350_FF)); // use parking brake directly
                    return 0;
                case xplm_CommandEnd:
                    if (ctx->acfspec.qpac.tolb[2])
                    {
                        ctx->acfspec.qpac.tolbctr = 0;
                        XPLMSetDataf(ctx->acfspec.qpac.tolb[0], 0.0f);
                        XPLMSetDataf(ctx->acfspec.qpac.tolb[1], 0.0f);
                    }
                    if (rcb->use_pkb == 0)
                    {
                        XPLMSetDataf(rcb->l_b_rat, 0.0f);
                        XPLMSetDataf(rcb->r_b_rat, 0.0f);
                    }
                    if (ctx->acfspec.qpac.h_b_max && rcb->pcmd)
                    {
                        XPLMCommandEnd(rcb->pcmd);
                    }
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, XPLMGetDatai(rcb->p_b_int));
                    return 0;
                default: // xplm_CommandContinue
                    if (ctx->acfspec.qpac.tolb[2])
                    {
                        if (!XPLMGetDatai(ctx->acfspec.qpac.pkb_ref))
                        {
                            if (ctx->acfspec.qpac.tolbctr < 10)
                            {
                                ctx->acfspec.qpac.tolbctr++;
                            }
                            else
                            {
                                if (0.01f > XPLMGetDataf(rcb->p_b_rat) &&
                                    0.01f < XPLMGetDataf(ctx->acfspec.qpac.tolb[0]))
                                {
                                    if (ctx->acfspec.qpac.toaltbr == 0)
                                    {
                                        const char *name; XPLMDataRef data; XPLMSpeakString("ToLiSS: falling back to parking brake");
                                        if ((data = XPLMFindDataRef((name = "sim/flightmodel/ground/surface_texture_type"))))
                                        {
                                            ndt_log("navP [debug]: %s: %d\n", name, XPLMGetDatai(data));
                                        }
                                        if ((data = XPLMFindDataRef((name = "sim/weather/runway_friction"))))
                                        {
                                            ndt_log("navP [debug]: %s: %f\n", name, XPLMGetDataf(data));
                                        }
                                        if ((data = XPLMFindDataRef((name = "sim/weather/rain_percent"))))
                                        {
                                            ndt_log("navP [debug]: %s: %f\n", name, XPLMGetDataf(data));
                                        }
                                        ctx->acfspec.qpac.toaltbr = 1;
                                    }
                                    ndt_log("navP [error]: ToLiSS braking override fail, "
                                            "using parking brake (%d %.2lf %.2lf %.2lf)\n",
                                            XPLMGetDatai(ctx->acfspec.qpac.tolb[2]),
                                            XPLMGetDataf(ctx->acfspec.qpac.tolb[0]),
                                            XPLMGetDataf(ctx->acfspec.qpac.tolb[1]),
                                            XPLMGetDataf(rcb->p_b_rat));
                                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, 1);
                                    return 0;
                                }
                                return 0;
                            }
                            return 0;
                        }
                        return 0;
                    }
                    if (rcb->use_pkb == 0)
                    {
                        XPLMSetDataf(rcb->l_b_rat, 1.0f);
                        XPLMSetDataf(rcb->r_b_rat, 1.0f);
                        return 0;
                    }
                    return 0;
            }
            return 0;
        }
        return 0;
    }
    else
    {
        if (rcb->mx.name)
        {
            if (rcb->mx.xpcr == NULL)
            {
                rcb->mx.xpcr = XPLMFindCommand(rcb->mx.name);
            }
        }
        else
        {
            rcb->mx.xpcr = NULL;
        }
    }
    switch (inPhase)
    {
        case xplm_CommandBegin:
            XPLMSetDataf(rcb->p_b_flt, 0.0f); // release park brake on manual brake application
            if (XPLMGetDatai(rcb->a_b_lev) > 1)
            {
                XPLMCommandOnce(rcb->abto); // disable A/BRK
            }
            if (rcb->mx.xpcr)
            {
                XPLMCommandBegin(rcb->mx.xpcr);
                return 0;
            }
            if (rcb->use_pkb)
            {
                XPLMSetDataf(rcb->p_b_rat, 1.0f);
                return 0;
            }
            XPLMSetDataf(rcb->l_b_rat, 1.0f);
            XPLMSetDataf(rcb->r_b_rat, 1.0f);
            XPLMSetDataf(rcb->p_b_rat, 0.0f);
            return 0;
        case xplm_CommandContinue:
            if (rcb->mx.xpcr)
            {
                return 0;
            }
            if (rcb->use_pkb)
            {
                XPLMSetDataf(rcb->p_b_rat, 1.0f);
                return 0;
            }
            XPLMSetDataf(rcb->l_b_rat, 1.0f);
            XPLMSetDataf(rcb->r_b_rat, 1.0f);
            return 0;
        default: // xplm_CommandEnd
            if (rcb->mx.xpcr)
            {
                XPLMCommandEnd(rcb->mx.xpcr);
            }
            if (rcb->use_pkb == 0)
            {
                XPLMSetDataf(rcb->l_b_rat, 0.0f);
                XPLMSetDataf(rcb->r_b_rat, 0.0f);
            }
            XPLMSetDataf(rcb->p_b_rat, XPLMGetDataf(rcb->p_b_flt));
            return 0;
    }
    return 0;
}

static int chandler_b_reg(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    acf_type_info_acf_ctx_init();
    chandler_context *ctx = inRefcon;
    refcon_braking *rcb = &ctx->bking.rc_brk;
    float p_b_flt = XPLMGetDataf(rcb->p_b_flt);
    float g_speed = XPLMGetDataf(rcb->g_speed) * 3.6f / 1.852f;
    float p_ratio = (g_speed < 20.0f) ? rcb->rtio[0] : (g_speed < 40.0f) ? rcb->rtio[1] : rcb->rtio[2];
    assert_context *rca = rcb->assert;
    if (rca)
    {
        if (rca->initialized) switch (inPhase)
        {
            case xplm_CommandBegin:
                XPLMSetDataf(rca->dat.acf_brake_force, rca->dat.acf_brake_force_nomin * p_ratio);
                XPLMCommandBegin((rcb->pcmd = rca->dat.h_brk_mximum));
                return 0;
            case xplm_CommandContinue:
                XPLMSetDataf(rca->dat.acf_brake_force, rca->dat.acf_brake_force_nomin * p_ratio);
                return 0;
            default:
                XPLMCommandEnd(rcb->pcmd);
                XPLMSetDataf(rca->dat.acf_brake_force, rca->dat.acf_brake_force_nomin);
                return 0;
        }
        return 0;
    }
    if (ctx->info->ac_type & ACF_TYP_MASK_QPC)
    {
        if (ctx->acfspec.qpac.ready == 0)
        {
            aibus_fbw_init(&ctx->acfspec.qpac);
        }
        if (ctx->acfspec.qpac.ready)
        {
            switch (inPhase)
            {
                case xplm_CommandBegin: // release parkbrake on manual brake application
                    // the FlightFactor-QPAC A350 has its parking brake dataref inverted
                    XPLMSetDatai(rcb->             p_b_int, (ctx->info->ac_type == ACF_TYP_A350_FF));
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, (ctx->info->ac_type == ACF_TYP_A350_FF));
                    if (ctx->acfspec.qpac.tolb[2])
                    {
                        ctx->acfspec.qpac.tolbctr = 0;
                        XPLMSetDataf(ctx->acfspec.qpac.tolb[0], p_ratio);
                        XPLMSetDataf(ctx->acfspec.qpac.tolb[1], p_ratio);
                        return 0;
                    }
                    if (rcb->use_pkb == 0)
                    {
                        XPLMSetDataf(rcb->l_b_rat, p_ratio);
                        XPLMSetDataf(rcb->r_b_rat, p_ratio);
                        return 0;
                    }
                    if (ctx->acfspec.qpac.h_b_reg && ctx->acfspec.qpac.h_b_max)
                    {
                        if (g_speed > 40.0f)
                        {
                            XPLMCommandBegin((rcb->pcmd = ctx->acfspec.qpac.h_b_max));
                            return 0;
                        }
                        if (g_speed > 3.0f)
                        {
                            XPLMCommandBegin((rcb->pcmd = ctx->acfspec.qpac.h_b_reg));
                            return 0;
                        }
                        else
                        {
                            rcb->pcmd = NULL;
                        }
                        // when aircraft is stationary, hold commands act as park
                        // brake toggles instead, don't use them but fall through
                    }
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, (ctx->info->ac_type != ACF_TYP_A350_FF)); // use parking brake directly
                    return 0;
                case xplm_CommandEnd:
                    if (ctx->acfspec.qpac.tolb[2])
                    {
                        ctx->acfspec.qpac.tolbctr = 0;
                        XPLMSetDataf(ctx->acfspec.qpac.tolb[0], 0.0f);
                        XPLMSetDataf(ctx->acfspec.qpac.tolb[1], 0.0f);
                    }
                    if (rcb->use_pkb == 0)
                    {
                        XPLMSetDataf(rcb->l_b_rat, 0.0f);
                        XPLMSetDataf(rcb->r_b_rat, 0.0f);
                    }
                    if (ctx->acfspec.qpac.h_b_reg && ctx->acfspec.qpac.h_b_max && rcb->pcmd)
                    {
                        XPLMCommandEnd(rcb->pcmd);
                    }
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, XPLMGetDatai(rcb->p_b_int));
                    return 0;
                default: // xplm_CommandContinue
                    if (ctx->acfspec.qpac.tolb[2])
                    {
                        if (!XPLMGetDatai(ctx->acfspec.qpac.pkb_ref))
                        {
                            if (ctx->acfspec.qpac.tolbctr < 10)
                            {
                                ctx->acfspec.qpac.tolbctr++;
                            }
                            else
                            {
                                if (0.01f > XPLMGetDataf(rcb->p_b_rat) &&
                                    0.01f < XPLMGetDataf(ctx->acfspec.qpac.tolb[0]))
                                {
                                    ndt_log("navP [error]: ToLiSS braking override fail, "
                                            "using parking brake (%d %.2lf %.2lf %.2lf)\n",
                                            XPLMGetDatai(ctx->acfspec.qpac.tolb[2]),
                                            XPLMGetDataf(ctx->acfspec.qpac.tolb[0]),
                                            XPLMGetDataf(ctx->acfspec.qpac.tolb[1]),
                                            XPLMGetDataf(rcb->p_b_rat));
                                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, 1);
                                    return 0;
                                }
                                XPLMSetDataf(ctx->acfspec.qpac.tolb[0], p_ratio);
                                XPLMSetDataf(ctx->acfspec.qpac.tolb[1], p_ratio);
                                return 0;
                            }
                            return 0;
                        }
                        return 0;
                    }
                    if (rcb->use_pkb == 0)
                    {
                        XPLMSetDataf(rcb->l_b_rat, p_ratio);
                        XPLMSetDataf(rcb->r_b_rat, p_ratio);
                        return 0;
                    }
                    return 0;
            }
            return 0;
        }
        return 0;
    }
    else
    {
        if (rcb->rg.name && rcb->mx.name)
        {
            if (rcb->rg.xpcr == NULL)
            {
                rcb->rg.xpcr = XPLMFindCommand(rcb->rg.name);
            }
            if (rcb->mx.xpcr == NULL)
            {
                rcb->mx.xpcr = XPLMFindCommand(rcb->mx.name);
            }
        }
        else
        {
            rcb->rg.xpcr = NULL;
            rcb->mx.xpcr = NULL;
        }
    }
    if (p_ratio < p_b_flt)
    {
        p_ratio = rcb->rtio[2];
    }
    switch (inPhase)
    {
        case xplm_CommandBegin:
            XPLMSetDataf(rcb->p_b_flt, 0.0f); // release park brake on manual brake application
            if (XPLMGetDatai(rcb->a_b_lev) > 1)
            {
                XPLMCommandOnce(rcb->abto); // disable A/BRK
            }
            if (rcb->rg.xpcr && rcb->mx.xpcr)
            {
                // always start with regular braking
                XPLMCommandBegin((rcb->pcmd = rcb->rg.xpcr));
                return 0;
            }
            if (rcb->use_pkb)
            {
                XPLMSetDataf(rcb->p_b_rat, p_ratio);
                return 0;
            }
            XPLMSetDataf(rcb->l_b_rat, p_ratio);
            XPLMSetDataf(rcb->r_b_rat, p_ratio);
            XPLMSetDataf(rcb->p_b_rat, 0.0f);
            return 0;
        case xplm_CommandEnd:
            if (rcb->rg.xpcr && rcb->mx.xpcr)
            {
                XPLMCommandEnd(rcb->mx.xpcr);
                XPLMCommandEnd(rcb->rg.xpcr);
            }
            if (rcb->use_pkb == 0)
            {
                XPLMSetDataf(rcb->l_b_rat, 0.0f);
                XPLMSetDataf(rcb->r_b_rat, 0.0f);
            }
            XPLMSetDataf(rcb->p_b_rat, XPLMGetDataf(rcb->p_b_flt));
            return 0;
        default: // xplm_CommandContinue
            if (rcb->rg.xpcr && rcb->mx.xpcr)
            {
                // adjust braking strength for speed
                if (g_speed > 30.0f)
                {
                    if (rcb->pcmd != rcb->mx.xpcr)
                    {
                        XPLMCommandEnd  ((rcb->pcmd));
                        XPLMCommandBegin((rcb->pcmd = rcb->mx.xpcr));
                        return 0;
                    }
                }
                else
                {
                    if (rcb->pcmd != rcb->rg.xpcr)
                    {
                        XPLMCommandEnd  ((rcb->pcmd));
                        XPLMCommandBegin((rcb->pcmd = rcb->rg.xpcr));
                        return 0;
                    }
                }
                return 0;
            }
            if (rcb->use_pkb)
            {
                XPLMSetDataf(rcb->p_b_rat, p_ratio);
                return 0;
            }
            XPLMSetDataf(rcb->l_b_rat, p_ratio);
            XPLMSetDataf(rcb->r_b_rat, p_ratio);
            return 0;
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
        assert_context   *a32 = ctx->revrs.assert; float f_val;
        int speak = XPLMGetDatai(ctx->callouts.ref_speedbrake);
        if (ctx->info->ac_type == ACF_TYP_HA4T_RW)
        {
            if (ctx->spbrk.ha4t == NULL)
            {
                ctx->spbrk.ha4t = XPLMFindDataRef("Hawker4000/control/speedbrake_b");
            }
        }
        else
        {
            ctx->spbrk.ha4t = NULL;
        }
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

            default:
                if (ctx->spbrk.ha4t && XPLMGetDatai(ctx->spbrk.ha4t))
                {
                    // spoilers armed, disarm but don't extend
                    if (speak > 0) XPLMSpeakString("spoilers disarmed");
                    XPLMSetDatai(ctx->spbrk.ha4t, 0); return 0;
                }
//              else
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
        }
        {
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
        }
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
        assert_context   *a32 = ctx->revrs.assert; float f_val;
        int speak = XPLMGetDatai(ctx->callouts.ref_speedbrake);
        if (ctx->info->ac_type == ACF_TYP_HA4T_RW)
        {
            if (ctx->spbrk.ha4t == NULL)
            {
                ctx->spbrk.ha4t = XPLMFindDataRef("Hawker4000/control/speedbrake_b");
            }
        }
        else
        {
            ctx->spbrk.ha4t = NULL;
        }
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

            default:
                if (ctx->info->ac_type == ACF_TYP_EMBE_SS &&
                    XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    // already retracted, we can't/needn't arm (automatic)
                    if (speak > 0) XPLMSpeakString("speedbrake retracted");
                    return 0;
                }
                if (ctx->spbrk.ha4t && XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    // already retracted, arm spoilers
                    if (speak > 0) XPLMSpeakString("spoilers armed");
                    XPLMSetDatai(ctx->spbrk.ha4t, 1); return 0;
                }
//              else
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
                return 0;
        }
        {
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
                }
                return 0;
            }
        }
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

/*
 * action: stow or deploy thrust reversers.
 *
 * rationale: X-Plane only has a toggle for this :(
 */
static int chandler_r_fwd(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    if (ctx->revrs.propup)
    {
        switch (inPhase)
        {
            case xplm_CommandBegin:
                XPLMCommandBegin(ctx->revrs.propup);
                return 0;
            case xplm_CommandEnd:
                XPLMCommandEnd(ctx->revrs.propup);
                return 0;
            default:
                return 0;
        }
    }
    if (inPhase == xplm_CommandEnd)
    {
        assert_context *a32 = ctx->revrs.assert;
        if (a32)
        {
            if (XPLMGetDataf(a32->dat.engine_reverse1) > 0.5f)
            {
                XPLMCommandOnce(a32->dat.toggle_r_ng1);
            }
            if (XPLMGetDataf(a32->dat.engine_reverse2) > 0.5f)
            {
                XPLMCommandOnce(a32->dat.toggle_r_ng2);
            }
            return 0;
        }
        if (ctx->info->ac_type == ACF_TYP_A319_TL ||
            ctx->info->ac_type == ACF_TYP_A321_TL)
        {
            int propmode; XPLMGetDatavi(ctx->revrs.prop_mode, &propmode, 0, 1);
            if (propmode == 3)
            {
                XPLMCommandOnce(ctx->revrs.propto);
                return 0;
            }
            return 0;
        }
        if (ctx->revrs.n_engines >= 1)
        {
            XPLMSetDatavi(ctx->revrs.prop_mode, propmode_fwd_get(), 0, ctx->revrs.n_engines);
            return 0;
        }
        return 0;
    }
    return 0;
}

static int chandler_r_rev(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    if (ctx->revrs.propdn)
    {
        switch (inPhase)
        {
            case xplm_CommandBegin:
                XPLMCommandBegin(ctx->revrs.propdn);
                return 0;
            case xplm_CommandEnd:
                XPLMCommandEnd(ctx->revrs.propdn);
                return 0;
            default:
                return 0;
        }
    }
    if (inPhase == xplm_CommandEnd)
    {
        assert_context *a32 = ctx->revrs.assert;
        if (a32)
        {
            if (XPLMGetDataf(a32->dat.engine_reverse1) < 0.5f)
            {
                XPLMCommandOnce(a32->dat.toggle_r_ng1);
            }
            if (XPLMGetDataf(a32->dat.engine_reverse2) < 0.5f)
            {
                XPLMCommandOnce(a32->dat.toggle_r_ng2);
            }
            return 0;
        }
        if (ctx->info->ac_type == ACF_TYP_A319_TL ||
            ctx->info->ac_type == ACF_TYP_A321_TL)
        {
            int propmode; XPLMGetDatavi(ctx->revrs.prop_mode, &propmode, 0, 1);
            if (propmode == 1)
            {
                XPLMCommandOnce(ctx->revrs.propto);
                return 0;
            }
            return 0;
        }
        if (ctx->revrs.n_engines >= 1)
        {
            XPLMSetDatavi(ctx->revrs.prop_mode, propmode_rev_get(), 0, ctx->revrs.n_engines);
            return 0;
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

#define T_ZERO (.0001f)

enum
{
    NVP_DIRECTION_DN,
    NVP_DIRECTION_UP,
};

static const float nvp_thrust_presets1[] =
{
    0.0250f,
    0.0500f,
    0.0750f,
    0.1000f,
    0.1250f,
    0.1875f,
    0.2500f,
    0.3125f,
    0.3750f,
    0.4375f,
    0.5000f,
    0.5625f,
    0.6250f,
    0.6875f,
    0.7500f,
    0.8125f,
    0.8750f,
    0.9000f,
    0.9250f,
    0.9500f,
    0.9750f,
    -1.000f,
};

static const float nvp_thrust_presets2[] =
{
    0.1250f,
    0.2500f,
    0.3750f,
    0.5000f,
    0.6250f,
    0.7500f,
    0.8750f,
    -1.000f,
};

static float nvp_thrust_next(float current, const float *presets, int direction)
{
    if (direction == NVP_DIRECTION_DN)
    {
        float next = 0.0f;
        for (int i = 0; presets[i] > 0.0f; i++)
        {
            if ((presets[i] + T_ZERO) < (current - T_ZERO))
            {
                next = presets[i];
                continue;
            }
            return next;
        }
        return next;
    }
    if (direction == NVP_DIRECTION_UP)
    {
        for (int i = 0; presets[i] > 0.0f; i++)
        {
            if ((presets[i] - T_ZERO) > (current + T_ZERO))
            {
                return presets[i];
            }
            continue;
        }
        return 1.0f;
    }
    return current;
}

#define T_CL30_APR      (1.0f) // 1.000
#define T_CL30_TOF (2.8f/3.0f) // 0.933
#define T_CL30_CLB (2.6f/3.0f) // 0.866
#define T_CL30_CRZ (2.5f/3.0f) // 0.833
#define T_CL30_MAN      (0.8f) // 0.800
static inline int custom_detents_cl30(XPLMDataRef throttle, int acf_type, float current, const float *presets, int direction)
{
    if (direction == NVP_DIRECTION_UP)
    {
        if (current > (T_CL30_TOF - T_ZERO)) // TO -> APR
        {
            XPLMSetDataf(throttle, T_CL30_APR);
            return 1;
        }
        if (current > (T_CL30_CLB - T_ZERO)) // CLB -> TO
        {
            XPLMSetDataf(throttle, T_CL30_TOF);
            return 1;
        }
        if (current > (T_CL30_CRZ - T_ZERO)) // CRZ -> CLB
        {
            XPLMSetDataf(throttle, T_CL30_CLB);
            return 1;
        }
        if (current > (T_CL30_MAN - T_ZERO)) // MAN -> CRZ
        {
            XPLMSetDataf(throttle, T_CL30_CRZ);
            return 1;
        }
        if (((nvp_thrust_next(current, presets, direction) > (T_CL30_MAN + T_ZERO)))) // -> MAN
        {
            XPLMSetDataf(throttle, T_CL30_MAN);
            return 1;
        }
        return 0;
    }
    if (direction == NVP_DIRECTION_DN)
    {
        if (current > (T_CL30_TOF + T_ZERO)) // APR -> TO
        {
            XPLMSetDataf(throttle, T_CL30_TOF);
            return 1;
        }
        if (current > (T_CL30_CLB + T_ZERO)) // TO -> CLB
        {
            XPLMSetDataf(throttle, T_CL30_CLB);
            return 1;
        }
        if (current > (T_CL30_CRZ + T_ZERO)) // CLB -> CRZ
        {
            XPLMSetDataf(throttle, T_CL30_CRZ);
            return 1;
        }
        if (current > (T_CL30_MAN + T_ZERO)) // CRZ ->
        {
            XPLMSetDataf(throttle, T_CL30_MAN);
            return 1;
        }
        return 0;
    }
    return 0;
}
#undef T_CL30_APR
#undef T_CL30_TOF
#undef T_CL30_CLB
#undef T_CL30_CRZ
#undef T_CL30_MAN

static inline int toliss_throttle_set(XPLMDataRef throttle, int acf_type, float next)
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

static inline int custom_detents_toli(XPLMDataRef throttle, int acf_type, float current, const float *presets, int direction)
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
     */
    if (direction == NVP_DIRECTION_UP)
    {
        if (current > (.865f - T_ZERO)) // FLEX (0.865..0.875) -> TOGA
        {
            toliss_throttle_set(throttle, acf_type, 1.0f);
            return 1;
        }
        if (current > (0.69f - T_ZERO)) // CLMB (0.690..0.695) -> FLEX
        {
            toliss_throttle_set(throttle, acf_type, .875f);
            return 1;
        }
        if (((nvp_thrust_next(current, presets, direction) > (0.69f + T_ZERO)))) // -> CLMB
        {
            toliss_throttle_set(throttle, acf_type, .695f);
            return 1;
        }
        return 0;
    }
    if (direction == NVP_DIRECTION_DN)
    {
        if (current > (.875f + T_ZERO)) // TOGA -> FLEX (0.875..0.865)
        {
            toliss_throttle_set(throttle, acf_type, .865f);
            return 1;
        }
        if (current > (.695f + T_ZERO)) // FLEX -> CLMB (0.695..0.690)
        {
            toliss_throttle_set(throttle, acf_type, 0.69f);
            return 1;
        }
        return 0;
    }
    return 0;
}

static inline int custom_throttle_all(XPLMDataRef throttle, int acf_type, float current, const float *presets, int direction)
{
    switch (acf_type)
    {
        case ACF_TYP_CL30_DD:
            if (custom_detents_cl30(throttle, acf_type, current, presets, direction))
            {
                return 0;
            }
            break;

        case ACF_TYP_A319_TL:
        case ACF_TYP_A321_TL:
        case ACF_TYP_A350_FF:
            if (custom_detents_toli(throttle, acf_type, current, presets, direction))
            {
                return 0;
            }
            break;

        default:
            break;
    }
    float next = nvp_thrust_next(current, presets, direction);
    if (next < 0.0f) next = 0.0f; if (next > 1.0f) next = 1.0f;
    switch (acf_type)
    {
        case ACF_TYP_A319_TL:
        case ACF_TYP_A321_TL:
        case ACF_TYP_A350_FF:
            return toliss_throttle_set(throttle, acf_type, next);

        default:
            break;
    }
    XPLMSetDataf(throttle, next);
    return 0;
}

static int chandler_thrdn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        refcon_thrust *t = inRefcon;
        if (t->throttle)
        {
            float avrg_throttle;
            switch (t->acf_type)
            {
                case ACF_TYP_A319_TL:
                case ACF_TYP_A321_TL:
                case ACF_TYP_A350_FF:
                {
                    float l[2];
                    XPLMGetDatavf(t->throttle, l, 0, 2);
                    avrg_throttle = ((l[0] + l[1]) / 2.0f);
                    break;
                }
                default:
                    avrg_throttle = XPLMGetDataf(t->throttle);
                    break;
            }
            return custom_throttle_all(t->throttle, t->acf_type, avrg_throttle, nvp_thrust_presets1, NVP_DIRECTION_DN);
        }
        XPLMCommandOnce(t->thrdn);
        return 0;
    }
    return 0;
}

static int chandler_thrup(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        refcon_thrust *t = inRefcon;
        if (t->throttle)
        {
            float avrg_throttle;
            switch (t->acf_type)
            {
                case ACF_TYP_A319_TL:
                case ACF_TYP_A321_TL:
                case ACF_TYP_A350_FF:
                {
                    float l[2];
                    XPLMGetDatavf(t->throttle, l, 0, 2);
                    avrg_throttle = ((l[0] + l[1]) / 2.0f);
                    break;
                }
                default:
                    avrg_throttle = XPLMGetDataf(t->throttle);
                    break;
            }
            return custom_throttle_all(t->throttle, t->acf_type, avrg_throttle, nvp_thrust_presets1, NVP_DIRECTION_UP);
        }
        XPLMCommandOnce(t->thrup);
        return 0;
    }
    return 0;
}

static int chandler_thrul(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        refcon_thrust *t = inRefcon;
        if (t->throttle)
        {
            float avrg_throttle;
            switch (t->acf_type)
            {
                case ACF_TYP_A319_TL:
                case ACF_TYP_A321_TL:
                case ACF_TYP_A350_FF:
                {
                    float l[2];
                    XPLMGetDatavf(t->throttle, l, 0, 2);
                    avrg_throttle = ((l[0] + l[1]) / 2.0f);
                    break;
                }
                default:
                    avrg_throttle = XPLMGetDataf(t->throttle);
                    break;
            }
            return custom_throttle_all(t->throttle, t->acf_type, avrg_throttle, nvp_thrust_presets2, NVP_DIRECTION_UP);
        }
        switch (t->acf_type)
        {
            case ACF_TYP_A320_FF:
                XPLMCommandOnce(t->thrup);
                XPLMCommandOnce(t->thrup);
                XPLMCommandOnce(t->thrup);
                return 0;
            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_apclb(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        if (XPLMGetDatai(((refcon_app*)inRefcon)->ap_pmod) > 0)
        {
            if (((refcon_app*)inRefcon)->ap_arry)
            {
                float  value = XPLMGetDataf(((refcon_app*)inRefcon)->to_pclb);
                XPLMSetDatavf(((refcon_app*)inRefcon)->ap_arry, &value, 0, 1);
                return 0;
            }
            XPLMSetDataf(((refcon_app*)inRefcon)->ap_pclb, XPLMGetDataf(((refcon_app*)inRefcon)->to_pclb));
            return 0;
        }
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
        }
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
                flap_callout_setst(_flap_names_AIB1, lroundf(4.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case ACF_TYP_B737_EA:
            case ACF_TYP_B737_XG:
                flap_callout_setst(_flap_names_BNG1, lroundf(8.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case ACF_TYP_B757_FF:
            case ACF_TYP_B767_FF:
            case ACF_TYP_B777_FF:
                flap_callout_setst(_flap_names_BNG2, lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
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
            default:
                if (!strcasecmp(ctx->info->icaoid, "A10"))
                {
                    flap_callout_setst(_flap_names_A10W, lroundf(2.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
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
                    !strcasecmp(ctx->info->icaoid, "SF50"))
                {
                    flap_callout_setst(_flap_names_2POS, lroundf(2.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "B52"))
                {
                    flap_callout_setst(_flap_names_B52G, lroundf(5.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "A318") || // most Airbus airliner variants
                    !strcasecmp(ctx->info->icaoid, "A319") ||
                    !strcasecmp(ctx->info->icaoid, "A320") ||
                    !strcasecmp(ctx->info->icaoid, "A321") ||
                    !strcasecmp(ctx->info->icaoid, "A330") ||
                    !strcasecmp(ctx->info->icaoid, "A332") ||
                    !strcasecmp(ctx->info->icaoid, "A333") ||
                    !strcasecmp(ctx->info->icaoid, "A340") ||
                    !strcasecmp(ctx->info->icaoid, "A342") ||
                    !strcasecmp(ctx->info->icaoid, "A343") ||
                    !strcasecmp(ctx->info->icaoid, "A345") ||
                    !strcasecmp(ctx->info->icaoid, "A346") ||
                    !strcasecmp(ctx->info->icaoid, "A350") ||
                    !strcasecmp(ctx->info->icaoid, "A358") ||
                    !strcasecmp(ctx->info->icaoid, "A359") ||
                    !strcasecmp(ctx->info->icaoid, "A380") ||
                    !strcasecmp(ctx->info->icaoid, "A388"))
                {
                    flap_callout_setst(_flap_names_AIB1, lroundf(8.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "B732") || // all Boeing 737 variants
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
                    flap_callout_setst(_flap_names_BNG1, lroundf(8.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "BSCA") || // all Boeing 747 variants
                    !strcasecmp(ctx->info->icaoid, "BLCF") ||
                    !strcasecmp(ctx->info->icaoid, "B74D") ||
                    !strcasecmp(ctx->info->icaoid, "B74F") ||
                    !strcasecmp(ctx->info->icaoid, "B74R") ||
                    !strcasecmp(ctx->info->icaoid, "B74S") ||
                    !strcasecmp(ctx->info->icaoid, "B741") ||
                    !strcasecmp(ctx->info->icaoid, "B742") ||
                    !strcasecmp(ctx->info->icaoid, "B743") ||
                    !strcasecmp(ctx->info->icaoid, "B744") ||
                    !strcasecmp(ctx->info->icaoid, "B747") ||
                    !strcasecmp(ctx->info->icaoid, "B748") ||
                    !strcasecmp(ctx->info->icaoid, "B752") || // all Boeing 757 variants
                    !strcasecmp(ctx->info->icaoid, "B753") ||
                    !strcasecmp(ctx->info->icaoid, "B757") ||
                    !strcasecmp(ctx->info->icaoid, "B762") || // all Boeing 767 variants
                    !strcasecmp(ctx->info->icaoid, "B763") ||
                    !strcasecmp(ctx->info->icaoid, "B764") ||
                    !strcasecmp(ctx->info->icaoid, "B767") ||
                    !strcasecmp(ctx->info->icaoid, "B77F") || // all Boeing 777 variants
                    !strcasecmp(ctx->info->icaoid, "B77L") ||
                    !strcasecmp(ctx->info->icaoid, "B77W") ||
                    !strcasecmp(ctx->info->icaoid, "B772") ||
                    !strcasecmp(ctx->info->icaoid, "B773") ||
                    !strcasecmp(ctx->info->icaoid, "B777") ||
                    !strcasecmp(ctx->info->icaoid, "B778") ||
                    !strcasecmp(ctx->info->icaoid, "B779") ||
                    !strcasecmp(ctx->info->icaoid, "B78X") || // all Boeing 787 variants
                    !strcasecmp(ctx->info->icaoid, "B787") ||
                    !strcasecmp(ctx->info->icaoid, "B788") ||
                    !strcasecmp(ctx->info->icaoid, "B789"))
                {
                    flap_callout_setst(_flap_names_BNG2, lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "B190"))
                {
                    flap_callout_setst(_flap_names_B190, lroundf(2.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "BD5J"))
                {
                    flap_callout_setst(_flap_names_BD5J, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
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
                    flap_callout_setst(_flap_names_C340, lroundf(2.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->info->icaoid, "CL30"))
                {
                    flap_callout_setst(_flap_names_BOM1, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
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
                if (!strcasecmp(ctx->info->icaoid, "FA7X") ||
                    !strcasecmp(ctx->info->icaoid, "FA8X"))
                {
                    flap_callout_setst(_flap_names_FA7X, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
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
                    int index = lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio));
                    if (ctx->info->ac_type == ACF_TYP_MD80_RO)
                    {
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
                    }
                    flap_callout_setst(_flap_names_MD80, index);
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
                if (!strcasecmp(ctx->info->icaoid, "PIPA"))
                {
                    flap_callout_setst(_flap_names_PIPA, lroundf(2.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
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
                                    // F5 defaults to mixture_down and is free on Apple's KBs
                                    XPLMSetHotKeyCombination(hot_key, XPLM_VK_F5, xplm_UpFlag);//fixme use other key
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
                     * if (2 == XPLMGetDadai(XPLMGetDataref("XCrafts/ERJ/avitab_location"))
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

                case ACF_TYP_B737_XG:
                case ACF_TYP_MD80_RO:
                    cdu->i_disabled = 1; break; // check for YFMS presence

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
                    if (*cdu->auth && *cdu->desc)
                    {
                        if (!STRN_CASECMP_AUTO(cdu->auth, "Aerobask") ||
                            !STRN_CASECMP_AUTO(cdu->auth, "Stephane Buon"))
                        {
                            if ((cdu->command[0] = XPLMFindCommand("aerobask/gtn650/Popup")) &&
                                (cdu->command[1] = XPLMFindCommand("aerobask/skyview/toggle_left")))
                            {
                                cdu->i_disabled = 0; break; // Aerobask GTN
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
                        if (!STRN_CASECMP_AUTO(cdu->auth, "Denis 'ddenn' Krupin"))
                        {
                            if (NULL == (cdu->command[0] = XPLMFindCommand("sim/operation/slider_12")))
                            {
                                cdu->i_disabled = 1; break; // check for YFMS presence
                            }
                            cdu->i_disabled = 0; break;
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
#endif
            {
                return 0;
            }
            cdu->i_disabled = 0;
        }
        switch (cdu->atyp)
        {
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
                // cycle: off, PFD-ND, EICAS-Tekton, off…
                float ones = 1.0f, zero = 0.0f, tek, pfd;
                XPLMGetDatavf(cdu->dataref[0], &tek, 52, 1);
                if (tek < 0.5f) // is Tekton FMS popup down?
                {
                    XPLMGetDatavf(cdu->dataref[0], &pfd, 16, 1);
                    if (pfd < 0.5f) // is PF display popup down?
                    {
                        XPLMSetDatavf(cdu->dataref[0], &ones, 16, 1); // PFD
                        XPLMSetDatavf(cdu->dataref[0], &ones, 17, 1); // ND
                        XPLMSetDatavf(cdu->dataref[0], &zero, 18, 1); // EICAS
                        XPLMSetDatavf(cdu->dataref[0], &zero, 50, 1); // radio
                        XPLMSetDatavf(cdu->dataref[0], &zero, 51, 1); // thrust
                        XPLMSetDatavf(cdu->dataref[0], &zero, 52, 1); // Tekton
                        return 0;
                    }
                    XPLMSetDatavf(cdu->dataref[0], &zero, 16, 1); // PFD
                    XPLMSetDatavf(cdu->dataref[0], &zero, 17, 1); // ND
                    XPLMSetDatavf(cdu->dataref[0], &ones, 18, 1); // EICAS
                    XPLMSetDatavf(cdu->dataref[0], &ones, 50, 1); // radio
                    XPLMSetDatavf(cdu->dataref[0], &ones, 51, 1); // thrust
                    XPLMSetDatavf(cdu->dataref[0], &ones, 52, 1); // Tekton
                    return 0;
                }
                XPLMSetDatavf(cdu->dataref[0], &zero, 16, 1); // PFD
                XPLMSetDatavf(cdu->dataref[0], &zero, 17, 1); // ND
                XPLMSetDatavf(cdu->dataref[0], &zero, 18, 1); // EICAS
                XPLMSetDatavf(cdu->dataref[0], &zero, 50, 1); // radio
                XPLMSetDatavf(cdu->dataref[0], &zero, 51, 1); // thrust
                XPLMSetDatavf(cdu->dataref[0], &zero, 52, 1); // Tekton
                return 0;
            }

            case ACF_TYP_A319_TL:
            case ACF_TYP_A321_TL:
            {
                int PopUpHeightArray[2]; XPLMGetDatavi(cdu->dataref[0], PopUpHeightArray, 0, 2);
                if (PopUpHeightArray[0] <= 0 && PopUpHeightArray[1] <= 0) // both popups hidden
                {
                    // reset relevant datarefs to preferred size/location and show both MCDUs
                    //                    { CDU1, CDU2, PFD1, PFD2, ND#1, ND#2, ECAM, ECAM, };
                    float PopUpScale[9] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, }; XPLMSetDatavf(cdu->dataref[1], PopUpScale, 0, 9);
                    int   PopUpXArry[9] = {    0,  448,    0, 1292, 1292, 1292,  646,  646,  473, }; XPLMSetDatavi(cdu->dataref[2], PopUpXArry, 0, 9);
                    int   PopUpYArry[9] = {    0,    0,  620,    0,  620,  620,  620,    0,  460, }; XPLMSetDatavi(cdu->dataref[3], PopUpYArry, 0, 9);
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
    if (inPhase == xplm_CommandBegin && inRefcon) // before any other processing takes place
    {
        int x; XPLMGetDatavi(((refcon_qpacfbw*)inRefcon)->popup_x, &x, 9, 1);
        int y; XPLMGetDatavi(((refcon_qpacfbw*)inRefcon)->popup_y, &y, 9, 1);
        if (((x >=   1 || y >=   1) &&
             (x != 768 && y != 352)))
        {
            x = 768; XPLMSetDatavi(((refcon_qpacfbw*)inRefcon)->popup_x, &x, 9, 1);
            y = 352; XPLMSetDatavi(((refcon_qpacfbw*)inRefcon)->popup_y, &y, 9, 1);
            return 1;
        }
        return 1;
    }
    return 1;
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

static int chandler_idleb(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        refcon_ground *grndp = inRefcon;
        assert_context *a320 = grndp->assert;
        if (a320)
        {
            // TODO: check reverse for other aircrafts
            if (XPLMGetDataf(a320->dat.engine_reverse1) > 0.5f ||
                XPLMGetDataf(a320->dat.engine_reverse2) > 0.5f)
            {
                return 0;
            }
            // there isn't a way to set throttle to a given position yet
            // instead, allow adjustments using XPLMCommandOnce but only
            // towards our "ideal" position - we may need multiple calls
            // Aircraft.Cockpit.Pedestal.EngineLever1: 20-65 (idle -> toga)
            float engine_lever_lt = XPLMGetDataf(a320->dat.engine_lever_lt);
            float engine_lever_rt = XPLMGetDataf(a320->dat.engine_lever_rt);
            if (((engine_lever_lt < (grndp->idle.r_taxi - T_ZERO))) &&
                ((engine_lever_rt < (grndp->idle.r_taxi - T_ZERO))))
            {
                XPLMCommandOnce(a320->dat.throttles_up);
                return 0;
            }
            if (((engine_lever_lt > (grndp->idle.r_taxi + T_ZERO))) ||
                ((engine_lever_rt > (grndp->idle.r_taxi + T_ZERO))))
            {
                XPLMCommandOnce(a320->dat.throttles_dn);
                return 0;
            }
            return 0;
        }
        if (XPLMGetDatai(grndp->idle.onground_any) != 1)
        {
            return 0;
        }
        if (grndp->idle.minimums > 0)
        {
            if (grndp->idle.thrott_array)
            {
                XPLMSetDatavf(grndp->idle.thrott_array, grndp->idle.r_t, 0, 2);
                return 0;
            }
            XPLMSetDataf(grndp->idle.throttle_all, grndp->idle.r_taxi);
            return 0;
        }
        XPLMSetDataf(grndp->idle.throttle_all, 0.2f);
        return 0;
    }
    return 0;
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
    if (inRefcon)
    {
        refcon_ground *grndp = inRefcon; assert_context *assrt = grndp->assert;
        float ground_spd_kts = XPLMGetDataf(grndp->ground_spd) * 3.6f / 1.852f;
        float thrott_cmd_all = XPLMGetDataf(grndp->idle.throttle_all), thra[2];
        if (grndp->idle.thrott_array)
        {
            XPLMGetDatavf(grndp->idle.thrott_array, thra, 0, 2);
            thrott_cmd_all = (((((thra[0] + thra[1]) / 2.0f))));
        }
        else if (assrt)
        {
            // Aircraft.Cockpit.Pedestal.EngineLever1: 0-20-65 (rev-idle-max)
            assrt->api.ValueGet(assrt->dat.id_f32_p_engines_lever1, &thra[0]);
            assrt->api.ValueGet(assrt->dat.id_f32_p_engines_lever1, &thra[1]);
            if ((thrott_cmd_all = (((thra[0] + thra[1]) / 2.0f) - 20.0f) / 45.0f) < 0.0f)
            {
                (thrott_cmd_all = (((thra[0] + thra[1]) / 2.0f) - 20.0f) / 20.0f);
            }
        }

#if TIM_ONLY
        if ((grndp->every_ten_seconds += inElapsedSinceLastCall) >= 10.0f)
        {
            // place here: check every 10 seconds only
            // partial time sync: minutes/seconds only
            switch (XPLMGetDatai(grndp->time.view_type)) // skip if menu showing etc.
            {
                case 1000: // 2D w/panel
                case 1017: // ext. chase
                case 1023: // 2D w/heads
                case 1026: // 3D cockpit
                case 1031: // ext. along
                    {
                        ndt_date datenow = ndt_date_now();
                        float minsec_new = (float)datenow.minutes * 60.0f + (float)datenow.seconds * 1.0f;
                        float time_currt = XPLMGetDataf(grndp->time.zulu_time_sec);
                        float minsec_cur = fmodf(time_currt, 3600.0f);
                        if (minsec_cur >   15.0f && // hours change
                            minsec_cur < 3585.0f && // don't resync
                            fabsf(minsec_cur - minsec_new) > 15.0f)
                        {
#if 0
                            float hh = floorf(time_currt / 3600.0f);
                            float mc = floorf(minsec_cur /   60.0f);
                            float mn = floorf(minsec_new /   60.0f);
                            float sc = fmodf (minsec_cur,    60.0f);
                            float sn = fmodf (minsec_new,    60.0f);
                            float nh = datenow.hours, nm = datenow.minutes, ns = datenow.seconds;
                            ndt_log("navP [info]: time: resync: difference: %lf\n", fabsf(minsec_cur-minsec_new));
                            ndt_log("navP [info]: time: resync: irl: now: %02.0lf:%02.0lf:%02.0lf\n", nh, nm, ns);
                            ndt_log("navP [info]: time: resync: sim: old: %02.0lf:%02.0lf:%02.0lf\n", hh, mc, sc);
                            ndt_log("navP [info]: time: resync: sim: new: %02.0lf:%02.0lf:%02.0lf\n", hh, mn, sn);
#endif
                            XPLMSetDataf(grndp->time.zulu_time_sec, time_currt - minsec_cur + minsec_new);
                        }
                    }
                    break;

                default:
                    break;
            }
            grndp->every_ten_seconds = 0.0f;
        }
#endif

        /* XXX: check whether we just connected to PilotEdge or VATSIM */
        if (grndp->oatc.pe_is_on)
        {
            int pe_is_connected = XPLMGetDatai(grndp->oatc.pe_is_on);
            if (pe_is_connected)
            {
                if (grndp->oatc.pe_was_connected < 1)
                {
                    // something resets radios to silent right after connecting to PE, we work around it
                    ndt_log("navP [info]: PilotEdge connection detected, acf_volume_reset in 3 seconds\n");
                    XPLMSetFlightLoopCallbackInterval(grndp->oatc.flc, 3.0f, 1, grndp->oatc.aircraft_type);
                }
                else
                {
                    float radio_volume_ratio = XPLMGetDataf(grndp->oatc.vol_com0);
                    if (0.1f > radio_volume_ratio) // global radio volume is zero
                    {
                        // assume PE has ATIS currently playing
                        // turn com 1 and 2 off to stop XP ATIS
                        // without it we hear the first message
                        // note: maybe doesn't work but it also
                        // doesn't seem to hurt either whatever
                        XPLMSetDataf(grndp->oatc.vol_com1, 0.0f);
                        XPLMSetDataf(grndp->oatc.vol_com2, 0.0f);
                    }
                    else if (0.1f > XPLMGetDataf(grndp->oatc.vol_com1) &&
                             0.1f > XPLMGetDataf(grndp->oatc.vol_com2))
                    {
                        // radios are on but both com 1 and com 2 are off, reset
                        acf_volume_context *volume_context = acf_volume_ctx_get();
                        if (volume_context == NULL)
                        {
                            ndt_log("navP [error]: gnd_stab_hdlr: acf_volume_ctx_get() failed\n");
                        }
                        else
                        {
                            acf_volume_reset(volume_context, *grndp->oatc.aircraft_type);
                        }
                    }
                }
            }
            grndp->oatc.pe_was_connected = !!pe_is_connected;
        }
        else if (grndp->oatc.xb_is_on)
        {
            int xb_is_connected = XPLMGetDatai(grndp->oatc.xb_is_on);
            if (xb_is_connected)
            {
                if (grndp->oatc.xb_was_connected < 1)
                {
                    // not actually required, but we do the same as for PE above
                    ndt_log("navP [info]: XSquawkBox connection detected acf_volume_reset in 3 seconds\n");
/*XXX:XSB-specific*/XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_com_selection" ), 0);
/*XXX:XSB-specific*/XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_selection_com1"), 1);
/*XXX:XSB-specific*/XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_selection_com2"), 1);
                    XPLMSetFlightLoopCallbackInterval(grndp->oatc.flc, 3.0f, 1, grndp->oatc.aircraft_type);
                }
            }
            grndp->oatc.xb_was_connected = !!xb_is_connected;
        }

        // without A/P on (otherwise auto-landing),
        // disable A/T and command idle at 50ft AGL
        if (XPLMGetDatai(grndp->auto_t_sts) >= 1 &&
            XPLMGetDatai(grndp->auto_p_sts) <= 0 &&
            XPLMGetDataf(grndp->elev_m_agl) <= 15.24f)
        {
            XPLMSetDatai(grndp->auto_t_sts, 0);
            XPLMSetDataf(grndp->idle.throttle_all, (thrott_cmd_all = 0.0f));
        }
        // TODO: FlightFactor 757???
        // its A/T never seems to auto-disconnect (unlike e.g. IXEG's 737), and will
        // command thrust to maintain speed even after touchdown unless auto-landing

        // raise our throttles to a minimum idle if required
        if (grndp->idle.minimums >= 2)
        {
            if (thrott_cmd_all < (grndp->idle.r_idle - T_ZERO))
            {
                XPLMSetDataf(grndp->idle.throttle_all, grndp->idle.r_idle);
            }
        }

        // check whether we are still on the ground or flying
        int airborne = !XPLMGetDatai(grndp->idle.onground_any);
        return airborne ? 1.0f : 0.25f;
    }
    return 0;
}

static int first_fcall_do(chandler_context *ctx)
{
    XPLMDataRef d_ref;
    XPLMPluginID p_id;
    XPLMCommandRef cr;
    int skview = 0, r;
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
        case ACF_TYP_A320_FF:
            {
                assert_context *rca = &ctx->info->assert;
                if (acf_type_is_engine_running() == 0) // cold & dark
                {
                    int32_t request_true = 1; float p = 500.0f, f = 3175.0f;
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
            break;

        case ACF_TYP_A319_TL:
        case ACF_TYP_A321_TL:
            if (acf_type_is_engine_running() == 0) // cold & dark
            {
                _DO(1, XPLMSetDatai, 1, "AirbusFBW/EnableExternalPower");   // ensure we have ground power
                _DO(1, XPLMSetDatai, 0, "AirbusFBW/GroundLPAir");           // lo pressure ground air: off
                _DO(1, XPLMSetDatai, 0, "AirbusFBW/GroundHPAir");           // hi pressure ground air: off
            }
            switch (ctx->info->ac_type)
            {
                case ACF_TYP_A321_TL:
                    if ((d_ref = XPLMFindDataRef("AirbusFBW/FuelOHPArray")))
                    {
                        int i[2] = { 0, 0, }; XPLMSetDatavi(d_ref, i, 2, 2);
                    }
                    _DO(1, XPLMSetDatai, 1, "params/wheel");                        // use scrollwheel
                    _DO(1, XPLMSetDatai, 0, "AirbusFBW/ALT100_1000");               // FCU alt. sel. incre. (100ft)
                    _DO(1, XPLMSetDatai, 2, "AirbusFBW/EngineType");                // CFM engines w/sharklets+web
                    _DO(1, XPLMSetDatai, 0, "AirbusFBW/SatComObjInhibit");          // CFM engines w/sharklets+web
                    _DO(1, XPLMSetDatai, 1, "AirbusFBW/WingtipDeviceType");         // CFM engines w/sharklets+web
                    _DO(1, XPLMSetDatai, 2, "AirbusFBW/FuelNumExtraTanks");         // use 5 fuel tanks by default
                    break;

                case ACF_TYP_A319_TL:
                default:
                    _DO(1, XPLMSetDatai, 1, "params/wheel");                        // use scrollwheel
                    _DO(1, XPLMSetDatai, 0, "AirbusFBW/ALT100_1000");               // FCU alt. sel. incre. (100ft)
                    _DO(1, XPLMSetDatai, 1, "AirbusFBW/EngineType");                // IAE engines w/sharklets+web
                    _DO(1, XPLMSetDatai, 0, "AirbusFBW/SatComObjInhibit");          // IAE engines w/sharklets+web
                    _DO(1, XPLMSetDatai, 1, "AirbusFBW/WingtipDeviceType");         // IAE engines w/sharklets+web
                    break;
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
            if (ctx->info->ac_type == ACF_TYP_A319_TL ||
                ctx->info->ac_type == ACF_TYP_A321_TL)
            {
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
                if (NULL == (ctx->a319kc.c[2][41] = XPLMFindCommand("navP/brakes/regular"           )) ||
                    NULL == (ctx->a319kc.c[2][42] = XPLMFindCommand("navP/spoilers/extend"          )) ||
                    NULL == (ctx->a319kc.c[2][43] = XPLMFindCommand("navP/spoilers/retract"         )) ||
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
            _DO(1, XPLMSetDatai,   7000, "1-sim/options/Transition Altitude");      // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/autoreverse");              // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/Auto Helper");              // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/Auto Pause");               // preferred default settings
            _DO(1, XPLMSetDatai,      1, "1-sim/options/Real Limits");              // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/Baro Units");               // preferred default settings
            _DO(1, XPLMSetDatai,      1, "1-sim/options/Difficulty");               // preferred default settings
            _DO(1, XPLMSetDatai,      1, "1-sim/options/Draw Lines");               // preferred default settings
            _DO(1, XPLMSetDatai,      0, "1-sim/options/ScreenGlow");               // preferred default settings
            _DO(1, XPLMSetDatai,      1, "1-sim/options/ILS Align");                // preferred default settings
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
                // grosswt: 334,000 lb == 151,500 kg
                float fuel = 0.45359207f * 020000.0f;
                float zwft = 0.45359207f * 314000.0f;
                acf_type_zfwt_set(ctx->info,  &zwft);
                acf_type_fuel_set(ctx->info,  &fuel);
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
            ndt_log("navP [info]: AirbusFBW key sniffer registered\n");
            break;

        case ACF_TYP_B737_EA:
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
                float gas = 3175.0f, zfw = 33333.3f;
                acf_type_fuel_set (ctx->info, &gas);
                acf_type_zfwt_set (ctx->info, &zfw);
            }
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
            break;

        case ACF_TYP_EMBE_SS:
            if (ctx->mcdu.rc.i_disabled == -1)
            {
                chandler_mcdup(ctx->mcdu.cb.command, xplm_CommandEnd, &ctx->mcdu.rc);  // XXX: remap hotkeys
            }
            _DO(1, XPLMSetDatai, 0, "ssg/EJET/GND/stair1_ON");                      // Hide passenger stairs
            _DO(1, XPLMSetDatai, 1, "ssg/EJET/GND/rain_hide_sw");                   // Disable rain effects
            _DO(1, XPLMSetDatai, 0, "ssg/EJET/GND/seats_hide_sw");                  // Hide captain's seat
            _DO(1, XPLMSetDatai, 0, "ssg/EJET/GND/yokes_hide_sw");                  // Hide both yokes
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
            _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_copilot");
            _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_pilot");
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_airport_on");
            _DO(0, XPLMSetDatai, 0, "sim/cockpit2/EFIS/EFIS_fix_on");
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_ndb_on");
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_vor_on");
            _DO(0, XPLMSetDatai, 4, "sim/cockpit2/EFIS/map_range");
            if (acf_type_is_engine_running() == 0)
            {
                float load = 500.0f, fuel = 3175.0f;
                acf_type_load_set(ctx->info, &load);
                acf_type_fuel_set(ctx->info, &fuel);
            }
            break;

        case ACF_TYP_LEGA_XC:
            if (ctx->mcdu.rc.i_disabled == -1)
            {
                chandler_mcdup(ctx->mcdu.cb.command, xplm_CommandEnd, &ctx->mcdu.rc);  // XXX: remap hotkeys
            }
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/generic_lights_switch")))
            {
                float generic_lights_switch[1] = { 1.0f, };
                XPLMSetDatavf(d_ref, &generic_lights_switch[0], 26, 1); // hide yoke L
                XPLMSetDatavf(d_ref, &generic_lights_switch[0], 27, 1); // hide yoke R
                XPLMSetDatavf(d_ref, &generic_lights_switch[0], 56, 1); // reflec. off
            }
            _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_copilot");
            _DO(0, XPLMSetDatai, 2, "sim/cockpit2/radios/actuators/HSI_source_select_pilot");
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_airport_on");
            _DO(0, XPLMSetDatai, 0, "sim/cockpit2/EFIS/EFIS_fix_on");
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_ndb_on");
            _DO(0, XPLMSetDatai, 1, "sim/cockpit2/EFIS/EFIS_vor_on");
            _DO(0, XPLMSetDatai, 4, "sim/cockpit2/EFIS/map_range");
            _DO(1, XPLMSetDatai, 1, "XCrafts/ERJ/weight_units");
            if (acf_type_is_engine_running() == 0)
            {
                float load = 250.0f, fuel = 1587.5f;
                acf_type_load_set(ctx->info, &load);
                acf_type_fuel_set(ctx->info, &fuel);
                _DO(0, XPLMSetDataf, 0.0f, "sim/flightmodel/misc/cgz_ref_to_default");
            }
//            if ((d_ref = XPLMFindDataRef("sim/aircraft/bodies/acf_fuse_cd")))
//            {
//                if (fabsf(XPLMGetDataf(d_ref) - 0.12f) > 0.005f)
//                {
//                    ndt_log("navP [error]: \"%s\" is not %.4f (%.4f)\n", "sim/aircraft/bodies/acf_fuse_cd", 0.12f, XPLMGetDataf(d_ref));
//                    XPLMSpeakString("turn around fail");
//                    return (ctx->first_fcall = 0) - 1;
//                }
//                XPLMSetDataf(d_ref, 0.196875f);
//            }
//            else
//            {
//                ndt_log("navP [error]: dataref not found: \"%s\"\n", "sim/aircraft/bodies/acf_fuse_cd");
//                XPLMSpeakString("turn around fail");
//                return (ctx->first_fcall = 0) - 1;
//            }
             // initial climb paramaters; V2(MTOW) is 139, +20 -> 160 KIAS
            _DO(0, XPLMSetDatai, 0, "sim/cockpit2/autopilot/airspeed_is_mach");
            _DO(0, XPLMSetDataf, 10.0f, "sim/cockpit2/autopilot/TOGA_pitch_deg");
            _DO(0, XPLMSetDataf, 160.0f, "sim/cockpit2/autopilot/airspeed_dial_kts_mach");
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
                float load = 250.0f, fuel = 1587.5f;
                acf_type_load_set(ctx->info, &load);
                acf_type_fuel_set(ctx->info, &fuel);
            }
            XPLMSetDataf(ctx->otto.clmb.rc.to_pclb, 8.5f); // initial CLB pitch
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
            _DO(1, XPLMSetDatai, 1, "Rotate/md80/electrical/GPU_power_available");
            _DO(1, XPLMSetDatai, 1, "Rotate/md80/misc/hide_yoke_button_clicked");
            _DO(1, XPLMSetDatai, 2, "Rotate/md80/instruments/nav_display_range");
            _DO(1, XPLMSetDatai, 2, "Rotate/md80/instruments/nav_display_mode");
            break;

        case ACF_TYP_CL30_DD:
            // datarefs: X-Plane default
            _DO(0, XPLMSetDataf, 0.8f, "sim/cockpit/electrical/instrument_brightness"); // set all at once
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
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/custom_slider_on")))
            {
                int door_open[1] = { 1, };
                XPLMSetDatavi(d_ref, &door_open[0], 6, 1);  // cabin to bathroom
            }
            _DO(1, XPLMSetDatai, 0, "cl300/position_saving");
            _DO(1, XPLMSetDatai, 1, "cl300/prflt_cam_lock");
            _DO(1, XPLMSetDatai, 1, "cl300/mmenu_athide");
            _DO(1, XPLMSetDatai, 1, "cl300/fms/alt_rep");
            _DO(1, XPLMSetDatai, 1, "cl300/hide_pilots");
            _DO(1, XPLMSetDatai, 1, "cl300/popup_mfd");
            _DO(1, XPLMSetDatai, 0, "cl300/baro_pref");
            _DO(1, XPLMSetDatai, 0, "cl300/alt_pref");
            _DO(1, XPLMSetDatai, 1, "cl300/com_pref");
            _DO(1, XPLMSetDatai, 0, "cl300/gpu_mode");
            if (acf_type_is_engine_running() == 0)
            {
                float load = 250.0f, fuel = 1587.5f;
                acf_type_load_set(ctx->info, &load);
                acf_type_fuel_set(ctx->info, &fuel);
            }
            XPLMSetDataf(ctx->otto.clmb.rc.to_pclb, 10.0f); // initial CLB pitch
            break;

        case ACF_TYP_GENERIC:
            // note: this path is always non-verbose (never warn for unapplicable datarefs)
            // datarefs: Aerobask
            _DO(0, XPLMSetDatai, 1, "sim/har/reflets");                             // LEG2: refl. off
            _DO(0, XPLMSetDatai, 0, "sim/har/pitchservo");                          // LEG2: 500ft/min
            _DO(0, XPLMSetDatai, 0, "aerobask/E1000/flags_on");                     // EPIC
            _DO(0, XPLMSetDatai, 1, "aerobask/E1000/yokeL_hidden");                 // EPIC
            _DO(0, XPLMSetDatai, 1, "aerobask/E1000/yokeR_hidden");                 // EPIC
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
            // datarefs: Alabeo, Carenado, SimCoders REP
            _DO(0, XPLMSetDatai, 1, "com/dkmp/HideYokeL");                          // various aircraft
            _DO(0, XPLMSetDatai, 1, "com/dkmp/HideYokeR");                          // various aircraft
            _DO(0, XPLMSetDatai, 0, "thranda/views/InstRefl");                      // various aircraft
            _DO(0, XPLMSetDatai, 0, "thranda/views/WindowRefl");                    // various aircraft
            _DO(0, XPLMSetDatai, 0, "thranda/views/staticelements");                // various aircraft
            _DO(0, XPLMSetDatai, 1, "thranda/cockpit/actuators/HideYokeL");         // various aircraft
            _DO(0, XPLMSetDatai, 1, "thranda/cockpit/actuators/HideYokeR");         // various aircraft
            if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("com.simcoders.rep"))
            {
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
                _DO(0, XPLMSetDataf, 1.0f, "com/dkmp/winglets");
                _DO(0, XPLMSetDataf, 1.0f, "com/dkmp/PassengerDoorHandle");
            }
            if (!strcasecmp(ctx->info->icaoid, "C206"))
            {
                _DO(0, XPLMSetDatai, 1, "com/dkmp/fairings");
                _DO(0, XPLMSetDatai, 0, "com/dkmp/InstRefl");
                _DO(0, XPLMSetDatai, 1, "com/dkmp/WindowRefl"); // inverted
            }
            if (!strcasecmp(ctx->info->icaoid, "TBM8"))
            {
                _DO(0, XPLMSetDataf, -0.5f, "thranda/cockpit/actuators/VisorL");
                _DO(0, XPLMSetDataf, -0.5f, "thranda/cockpit/actuators/VisorR");
                _DO(0, XPLMSetDataf,  1.0f, "thranda/cockpit/actuators/VisorSwingL");
                _DO(0, XPLMSetDataf,  1.0f, "thranda/cockpit/actuators/VisorSwingR");
                _DO(0, XPLMSetDataf,  0.0f, "thranda/cockpit/actuators/VisorSlideL");
                _DO(0, XPLMSetDataf,  0.0f, "thranda/cockpit/actuators/VisorSlideR");
            }
            // datarefs: X-Plane default
            _DO(0, XPLMSetDataf, 0.8f, "sim/cockpit/electrical/instrument_brightness"); // set all at once
            switch (ctx->info->engine_type1) // engine-specific takeoff pitch
            {
                case 4: case 5: // jet
                    XPLMSetDataf(ctx->otto.clmb.rc.to_pclb, 10.0f);
                    break;
                case 2: case 8: // turbine
                    XPLMSetDataf(ctx->otto.clmb.rc.to_pclb, 8.5f);
                    break;
                default:
                    XPLMSetDataf(ctx->otto.clmb.rc.to_pclb, 7.0f);
                    break;
            }
            if (ctx->info->author[0] && ctx->info->descrp[0])
            {
                if (!STRN_CASECMP_AUTO(ctx->info->author, "After"))
                {
                    if (!STRN_CASECMP_AUTO(ctx->info->icaoid, "FA7X"))
                    {
                        // climb "path" (not pitch); default to 7.5 degrees
                        // range of values -- min: ~5 (MTOW) max: ~15 (OEW)
                        if ((d_ref = XPLMFindDataRef("sim/weapons/targ_h")))
                        {
                            float value = ((0.0f + 0.0285f) / 0.5011f);
                            XPLMSetDatavf((ctx->otto.clmb.rc.ap_arry = d_ref), &value, 0, 1);
                            XPLMSetDataf(ctx->otto.clmb.rc.to_pclb, ((7.5f + 0.0285f) / 0.5011f));
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
                        // make the aircraft less tail-heavy to improve ground handling
                        _DO(0, XPLMSetDataf, -0.30f, "sim/aircraft/overflow/acf_cgZ_fwd");
                        _DO(0, XPLMSetDataf, -0.20f, "sim/flightmodel/misc/cgz_ref_to_default");
                        _DO(0, XPLMSetDataf, +0.10f, "sim/aircraft/overflow/acf_cgZ_aft");
                        // let's also skip drawing the FMS line from the HSI display's map
                        _DO(1, XPLMSetDatai, 1, "sim/graphics/misc/kill_map_fms_line");
                        // and fully declutter the HSI/Avidyne displays by default
                        _DO(0, XPLMSetDatai, 0, "com/dkmp/Avidyne/Declutter");
                    }
                    if (!STRN_CASECMP_AUTO(ctx->info->descrp, "C207 Skywagon"))
                    {
                        // make the aircraft less tail-heavy to improve ground handling
                        _DO(0, XPLMSetDataf, -0.20f, "sim/aircraft/overflow/acf_cgZ_fwd");
                        _DO(0, XPLMSetDataf, -0.10f, "sim/flightmodel/misc/cgz_ref_to_default");
                        _DO(0, XPLMSetDataf, +0.00f, "sim/aircraft/overflow/acf_cgZ_aft");
                    }
                    if (!STRN_CASECMP_AUTO(ctx->info->descrp, "C404 Titan"))
                    {
                        if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/instrument_brightness_ratio")))
                        {
                            float instrument_brightness_ratio[1] = { 0.5f, };
                            XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0], 4, 1); // autopilot/warning annunciator brightness
                        }
                    }
                    if (!STRN_CASECMP_AUTO(ctx->info->descrp, "Bonanza V35B"))
                    {
                        if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/custom_slider_on")))
                        {
                            int tip_tanks_enabled[1] = { 1, };
                            XPLMSetDatavi(d_ref, &tip_tanks_enabled[0], 10, 1);
                        }
                    }
                }
                if (!STRN_CASECMP_AUTO(ctx->info->author, "Aerobask") ||
                    !STRN_CASECMP_AUTO(ctx->info->author, "Stephane Buon"))
                {
                    if (!STRN_CASECMP_AUTO(ctx->info->descrp, "Pipistrel Panthera"))
                    {
                        skview = 1;
                        _DO(0, XPLMSetDatai,      0, "sim/cockpit2/autopilot/airspeed_is_mach");
                        _DO(0, XPLMSetDataf, 120.0f, "sim/cockpit2/autopilot/airspeed_dial_kts_mach");
                    }
                    if (!STRN_CASECMP_AUTO(ctx->info->descrp, "Epic E1000") ||
                        !STRN_CASECMP_AUTO(ctx->info->descrp, "Epic Victory"))
                    {
                        skview = 1;
                        _DO(0, XPLMSetDatai,      0, "sim/cockpit2/autopilot/airspeed_is_mach");
                        _DO(0, XPLMSetDataf, 150.0f, "sim/cockpit2/autopilot/airspeed_dial_kts_mach");
                        _DO(0, XPLMSetDatai,      0, "sim/cockpit2/pressurization/actuators/bleed_air_mode");
                        _DO(0, XPLMSetDatai,      0, "sim/cockpit2/ice/ice_pitot_heat_on_copilot");
                        _DO(0, XPLMSetDatai,      0, "sim/cockpit2/ice/ice_pitot_heat_on_pilot");
                        _DO(0, XPLMSetDatai,      0, "sim/cockpit2/EFIS/EFIS_weather_on");
                        _DO(0, XPLMSetDatai,      0, "sim/cockpit2/EFIS/EFIS_tcas_on");
                        _DO(0, XPLMSetDatai,      0, "sim/cockpit2/ice/ice_detect_on");
                    }
                    if (!strcasecmp(ctx->info->icaoid, "EA50"))
                    {
                        if (!STRN_CASECMP_AUTO(ctx->info->descrp, "The Eclipse 550"))
                        {
                            _DO(0, XPLMSetDatai,      0, "sim/cockpit2/autopilot/airspeed_is_mach");
                            _DO(0, XPLMSetDataf, 150.0f, "sim/cockpit2/autopilot/airspeed_dial_kts_mach");
                        }
                        skview = 1;
                        _DO(0, XPLMSetDatai,      0, "sim/cockpit2/pressurization/actuators/bleed_air_mode");
                        _DO(0, XPLMSetDatai,      1, "sim/cockpit2/ice/ice_pitot_heat_on_copilot");
                        _DO(0, XPLMSetDatai,      1, "sim/cockpit2/ice/ice_pitot_heat_on_pilot");
                        _DO(0, XPLMSetDatai,      1, "sim/cockpit2/ice/ice_AOA_heat_on_copilot");
                        _DO(0, XPLMSetDatai,      1, "sim/cockpit2/ice/ice_AOA_heat_on");
                    }
                }
            }
            if (skview == 0) // don't mess w/Aerobask's WXR radar
            {
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
            }
            if (ctx->revrs.n_engines == 1) // single-engine: select default fuel tank
            {
                if ((d_ref = XPLMFindDataRef("sim/aircraft/overflow/acf_has_fuel_all")))
                {
                    int acf_has_fuel_all, acf_num_tanks = 0; float rat[9];
                    {
                        acf_has_fuel_all = XPLMGetDatai(d_ref);
                    }
                    if ((d_ref = XPLMFindDataRef("sim/aircraft/overflow/acf_tank_rat")))
                    {
                        for (int i = 0, j = XPLMGetDatavf(d_ref, rat, 0, 9); i < j; i++)
                        {
                            if (rat[i] > .01f)
                            {
                                acf_num_tanks++;
                            }
                        }
                        if (acf_num_tanks == 4 && (fabsf(rat[0] - rat[1]) < .01f &&
                                                   fabsf(rat[2] - rat[3]) < .01f))
                        {
                            acf_num_tanks -= 2; // e.g. Bonanza V35B w/tip tanks
                        }
                        if (acf_num_tanks == 2)
                        {
                            if (acf_has_fuel_all) // can draw from all tanks at once
                            {
                                _DO(0, XPLMSetDatai, 4, "sim/cockpit2/fuel/fuel_tank_selector");
                            }
                            else
                            {
                                _DO(0, XPLMSetDatai, 1, "sim/cockpit2/fuel/fuel_tank_selector");
                                _DO(0, XPLMSetDatai, 0, "aerobask/panthera/fuel_position");
                                _DO(0, XPLMSetDatai, 1, "aerobask/victory/fuel_position");
                                _DO(0, XPLMSetDatai, 1, "aerobask/E1000/fuel_position");
                                _DO(0, XPLMSetDatai, 0, "sim/har/fueltank");
                            }
                        }
                    }
                }
            }
            if (acf_type_is_engine_running() == 0 && skview == 0 &&
                XPLMFindPluginBySignature("com.simcoders.rep") == XPLM_NO_PLUGIN_ID)
            {
                float load = 77.0f; acf_type_load_set(ctx->info, &load); //pilot
            }
            break;

        default:
            break;
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
        &ctx->athr.disc.cc,
        &ctx->athr.toga.cc,
        &ctx->bking.rc_brk.rg,
        &ctx->bking.rc_brk.mx,
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
    if (ctx->athr.toga.cc.xpcr && ctx->otto.disc.cc.xpcr)
    {
        if ((ctx->athr.toga.cc.xpcr == ctx->apd.at_cmd) &&
            (ctx->otto.disc.cc.xpcr == ctx->apd.bef.command))
        {
            REGISTER_CHANDLER(ctx->apd.bef, chandler_apbef, 1, &ctx->apd);
            REGISTER_CHANDLER(ctx->apd.aft, chandler_apaft, 0, &ctx->apd);
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
    if ((d_ref = XPLMFindDataRef("sim/time/local_date_days")))
    {
        ndt_date today_now = ndt_date_now(); // note: XP doesn't know 02/29 (easier :-)
        int month2days[12] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, };
        int xplm_date_days = month2days[today_now.month - 1] + today_now.day - 1;
        XPLMSetDatai(d_ref, xplm_date_days);
    }
    if (ctx->ground.time.zulu_time_sec)
    {
        ndt_date datenow = ndt_date_now();
        float time_currt = XPLMGetDataf(ctx->ground.time.zulu_time_sec);
        float minsec_new = (float)datenow.minutes * 60.0f + (float)datenow.seconds * 1.0f;
        XPLMSetDataf(ctx->ground.time.zulu_time_sec, time_currt - fmodf(time_currt, 3600.0f) + minsec_new);
    }
#endif

    /*
     * Custom ground stabilization system (via flight loop callback)
     *
     * Minimum ground throttle detent.
     * Testing parameters:
     * - weather: CAVOK preset
     * - version: X-Plane 10.51r2
     * - runways: follow terrain contour OFF
     * - airport: KNTD (Naval Base Ventura County)
     * - taxiing: ideal peak speed ~20.0 Knots ground speed
     * - pistons: minimum propeller speed ~1,000.0 r/minute
     */
    switch (ctx->info->ac_type)
    {
        case ACF_TYP_A320_FF:
            // Pedestal.EngineLever1: 20.0/24/26 (idle/fwd1/fwd2)
            ctx->ground.idle.r_taxi   = .400000f; // fwd pos. #2
            ctx->ground.idle.minimums = 0; break;

        case ACF_TYP_A319_TL:
        case ACF_TYP_A321_TL:
            ctx->ground.idle.thrott_array = XPLMFindDataRef("AirbusFBW/throttle_input");
            ctx->ground.idle.r_t[0]   = 0.25000f;
            ctx->ground.idle.r_t[1]   = 0.25000f;
            ctx->ground.idle.minimums = 1; break;

        case ACF_TYP_A350_FF:
            ctx->ground.idle.thrott_array = XPLMFindDataRef("AirbusFBW/throttle_input");
//          ctx->ground.idle.r_t[0]   = 0.10000f; // ~25.3% N1 @ NTD
//          ctx->ground.idle.r_t[1]   = 0.10000f; // ~25.3% N1 @ NTD
            ctx->ground.idle.r_t[0]   = 0.25000f;
            ctx->ground.idle.r_t[1]   = 0.25000f;
            ctx->ground.idle.minimums = 1; break;

        case ACF_TYP_B737_XG:
//          ctx->ground.idle.r_taxi   = 0.13333f; // ~33.3% N1 @ NTD
            ctx->ground.idle.r_taxi   = 0.37500f;
            ctx->ground.idle.minimums = 1; break;

        case ACF_TYP_B757_FF:
            if ((d_ref = XPLMFindDataRef("757Avionics/engine")))
            {
                switch (XPLMGetDatai(d_ref))
                {
                    case 0: // Pratt & Whitney
//                      ctx->ground.idle.r_taxi   = 0.09000f; // ~26.1% N1 @ NTD
                        ctx->ground.idle.r_taxi   = 0.12500f;
                        ctx->ground.idle.minimums = 1; break;
                    case 1: // Rolls-Royce
//                      ctx->ground.idle.r_taxi   = 0.15555f; // ~26.1% N1 @ NTD
                        ctx->ground.idle.r_taxi   = 0.25000f;
                        ctx->ground.idle.minimums = 1; break;
                    default:
                        ndt_log("navP [warning]: couldn't determine engine type for FF757\n");
                        break;
                }
                break;
            }
            ndt_log("navP [warning]: couldn't obtain engine type data reference for FF757\n");
            break;

        case ACF_TYP_B767_FF:
            if ((d_ref = XPLMFindDataRef("757Avionics/engine")))
            {
                switch (XPLMGetDatai(d_ref))
                {
                    case 0: // Pratt & Whitney
//                      ctx->ground.idle.r_taxi   = 0.16666f; // ~26.1% N1 @ NTD
                        ctx->ground.idle.r_taxi   = 0.25000f;
                        ctx->ground.idle.minimums = 1; break;
                    default:
                        ndt_log("navP [warning]: couldn't determine engine type for FF767\n");
                        break;
                }
                break;
            }
            ndt_log("navP [warning]: couldn't obtain engine type data reference for FF767\n");
            break;

        case ACF_TYP_B777_FF:
            if ((d_ref = XPLMFindDataRef("1-sim/engineType")))
            {
                switch (XPLMGetDatai(d_ref))
                {
                    case 1: // General Electric
//                      ctx->ground.idle.r_taxi   = 0.09765f; // ~28.1% N1 @ NTD
                        ctx->ground.idle.r_taxi   = 0.12500f;
                        ctx->ground.idle.minimums = 1; break;
                    default:
                        ndt_log("navP [warning]: couldn't determine engine type for FF777\n");
                        break;
                }
                break;
            }
            ndt_log("navP [warning]: couldn't obtain engine type data reference for FF777\n");
            break;

        default:
        {
            if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("com.simcoders.rep"))
            {
                // REP: 3.4+: custom engine model, use "simcoders/" rpm datarefs
                if (!STRN_CASECMP_AUTO(ctx->info->icaoid, "BE33") ||
                    !STRN_CASECMP_AUTO(ctx->info->icaoid, "BE35"))
                {
//                  ctx->ground.idle.r_idle   = 0.06845f; // prop 1,100rpm @ NTD
//                  ctx->ground.idle.r_taxi   = 0.15525f; // prop 1,400rpm @ NTD
                    ctx->ground.idle.r_idle   = 0.05000f;
                    ctx->ground.idle.r_taxi   = 0.15000f;
                    ctx->ground.idle.minimums = 2; break;
                }
                if (!STRN_CASECMP_AUTO(ctx->info->icaoid, "BE58"))
                {
//                  ctx->ground.idle.r_idle   = 0.08725f; // prop 1,000rpm @ NTD
//                  ctx->ground.idle.r_taxi   = 0.20375f; // prop 1,400rpm @ NTD
                    ctx->ground.idle.r_idle   = 0.07500f;
                    ctx->ground.idle.r_taxi   = 0.20000f;
                    ctx->ground.idle.minimums = 2; break;
                }
                if (!STRN_CASECMP_AUTO(ctx->info->icaoid, "T210"))
                {
//                  ctx->ground.idle.r_idle   = 0.06150f; // prop 1,100rpm @ NTD
//                  ctx->ground.idle.r_taxi   = 0.11025f; // prop 1,400rpm @ NTD
                    ctx->ground.idle.r_idle   = 0.05000f;
                    ctx->ground.idle.r_taxi   = 0.15000f;
                    ctx->ground.idle.minimums = 2; break;
                }
                break;
            }
            if (!STRN_CASECMP_AUTO(ctx->info->author, "Aerobask") ||
                !STRN_CASECMP_AUTO(ctx->info->author, "Stephane Buon"))
            {
                if (!STRN_CASECMP_AUTO(ctx->info->descrp, "Lancair Legacy FG"))
                {
//                  ctx->ground.idle.r_idle   = 0.03010f; // prop 1,100rpm @ NTD
//                  ctx->ground.idle.r_taxi   = 0.09650f; // prop 1,400rpm @ NTD
                    ctx->ground.idle.r_idle   = 0.02500f;
                    ctx->ground.idle.r_taxi   = 0.12500f;
                    ctx->ground.idle.minimums = 2; break;
                }
                if (!STRN_CASECMP_AUTO(ctx->info->descrp, "Pipistrel Panthera"))
                {
//                  ctx->ground.idle.r_idle   = 0.05585f; // prop 1,100rpm @ NTD
//                  ctx->ground.idle.r_taxi   = 0.13650f; // prop 1,400rpm @ NTD
                    ctx->ground.idle.r_idle   = 0.05000f;
                    ctx->ground.idle.r_taxi   = 0.12500f;
                    ctx->ground.idle.minimums = 2; break;
                }
                if (!STRN_CASECMP_AUTO(ctx->info->descrp, "Epic Victory"))
                {
//                  ctx->ground.idle.r_taxi   = 0.16666f; // ~45.0% N1 @ NTD
                    ctx->ground.idle.r_taxi   = 0.25000f;
                    ctx->ground.idle.minimums = 1; break;
                }
                if (!STRN_CASECMP_AUTO(ctx->info->descrp, "The Eclipse 550"))
                {
//                  ctx->ground.idle.r_taxi   = 0.23875f; // ~50.0% N1 @ NTD
                    ctx->ground.idle.r_taxi   = 0.37500f;
                    ctx->ground.idle.minimums = 1; break;
                }
                break;
            }
            if (!STRN_CASECMP_AUTO(ctx->info->author, "Alabeo") ||
                !STRN_CASECMP_AUTO(ctx->info->author, "Carenado"))
            {
                if (!STRN_CASECMP_AUTO(ctx->info->descrp, "Pilatus PC12"))
                {
                    ctx->ground.idle.r_taxi   = 0.37500f;
                    ctx->ground.idle.minimums = 1; break;
                }
                break;
            }
            break;
        }
    }
    if (ctx->ground.oatc.flc)
    {
        XPLMUnregisterFlightLoopCallback(ctx->ground.oatc.flc, &ctx);
    }
    if (ctx->ground.flc_g)
    {
        XPLMUnregisterFlightLoopCallback(ctx->ground.flc_g, &ctx->ground);
    }
    ctx->ground.every_ten_seconds = 0.0f;
    XPLMRegisterFlightLoopCallback((ctx->ground.flc_g = &gnd_stab_hdlr), 1, &ctx->ground);

    /* mixture and prop pitch command handlers */
    REGISTER_CHANDLER(ctx->throt.mi, chandler_mixdn, 1, ctx->throt.mixratio);
    REGISTER_CHANDLER(ctx->throt.rp, chandler_rpmdn, 1, ctx->throt.rpmratio);

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
        return -1;
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

//    /* Boost frame rates by disabling cloud drawing altogether */
//    if (ctx->menu_context)
//    {
//        nvp_menu_ckill(ctx->menu_context, xplm_Menu_Checked);
//    }
#endif

    return (ctx->first_fcall = 0);
}

static int aibus_fbw_init(refcon_qpacfbw *fbw)
{
    if (fbw && fbw->ready == 0)
    {
        if ((fbw->mwcb.command = XPLMFindCommand(   "toliss_airbus/iscs_open")) &&
            (fbw->popup_x      = XPLMFindDataRef("AirbusFBW/PopUpXCoordArray")) &&
            (fbw->popup_y      = XPLMFindDataRef("AirbusFBW/PopUpYCoordArray")))
        {
            REGISTER_CHANDLER(fbw->mwcb, chandler_31isc, 1/*before ToLiSS plugin*/, fbw);
        }
        if ((fbw->pkb_ref = XPLMFindDataRef("1-sim/parckBrake")))
        {
            /*
             * We're good to go!
             */
            (fbw->h_b_max = fbw->h_b_reg = NULL);
            (fbw->ready = 1); return 0;
        }
        if ((fbw->pkb_ref = XPLMFindDataRef("AirbusFBW/ParkBrake")) ||
            (fbw->pkb_ref = XPLMFindDataRef("com/petersaircraft/airbus/ParkBrake")))
        {
            if ((fbw->tolb[0] = XPLMFindDataRef("AirbusFBW/BrakePedalInputLeft"))  &&
                (fbw->tolb[1] = XPLMFindDataRef("AirbusFBW/BrakePedalInputRight")) &&
                (fbw->tolb[2] = XPLMFindDataRef("AirbusFBW/BrakePedalInputOverride")))
            {
                /*
                 * Permanently override braking input.
                 */
                XPLMSetDatai(fbw->tolb[2], 1);

                /*
                 * DEBUG: log stuff when alternate/fallback braking is used.
                 */
                fbw->toaltbr = 0;

                /*
                 * We're good to go!
                 */
                (fbw->ready = 1); return 0;
            }
            if ((fbw->h_b_max = XPLMFindCommand("sim/flight_controls/brakes_max"    )) &&
                (fbw->h_b_reg = XPLMFindCommand("sim/flight_controls/brakes_regular")))
            {
                /*
                 * We're good to go!
                 */
                (fbw->ready = 1); return 0;
            }
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

#undef REGISTER_CHANDLER
#undef UNREGSTR_CHANDLER
#undef CALLOUT_PARKBRAKE
#undef CALLOUT_SPEEDBRAK
#undef CALLOUT_FLAPLEVER
#undef CALLOUT_GEARLEVER
#undef A320T_CLMB
#undef A320T_HALF
#undef A320T_IDLE
#undef A320T_TAXI
#undef T_ZERO
#undef _DO
