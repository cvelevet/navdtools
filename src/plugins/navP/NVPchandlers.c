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
#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMDisplay.h"
#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMProcessing.h"
#include "XPLM/XPLMUtilities.h"

#include "common/common.h"

#include "NVPchandlers.h"

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
    XPLMDataRef p_b_rat;
    XPLMDataRef l_b_rat;
    XPLMDataRef r_b_rat;
} refcon_braking;

typedef struct
{
    int           ready;
    XPLMDataRef pkb_ref;
} refcon_ff_a350;

typedef struct
{
    int              ready;
    int            pkb_var;
    XPLMDataRef    pkb_tmp;
    XPLMDataRef    pkb_ref;
    XPLMCommandRef h_b_max;
    XPLMCommandRef h_b_reg;
} refcon_qpacfbw;

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
    int            acftyp;
    int            status;
    XPLMCommandRef cmd[2];
} refcon_cdu_pop;

typedef struct
{
    int initialized;
    int first_fcall;
    int kill_daniel;

    char icao[5]; // addon's ICAO aircraft type designator
    enum
    {
        NVP_ACF_GENERIC = 0x0000000,
        NVP_ACF_A320_JD = 0x0000001,
        NVP_ACF_A320_QP = 0x0000002,
        NVP_ACF_A330_JD = 0x0000004,
        NVP_ACF_A330_RW = 0x0000008,
        NVP_ACF_A350_FF = 0x0000010,
        NVP_ACF_A380_PH = 0x0000020,
        NVP_ACF_B727_FJ = 0x0000040,
        NVP_ACF_B737_EA = 0x0000080,
        NVP_ACF_B737_FJ = 0x0000100,
        NVP_ACF_B737_XG = 0x0000200,
        NVP_ACF_B757_FF = 0x0001000,
        NVP_ACF_B767_FF = 0x0002000,
        NVP_ACF_B777_FF = 0x0004000,
        NVP_ACF_DH8D_FJ = 0x0010000,
        NVP_ACF_EMBE_SS = 0x0020000,
        NVP_ACF_EMBE_XC = 0x0040000,
        NVP_ACF_ERJ1_4D = 0x0080000,
        NVP_ACF_SSJ1_RZ = 0x0100000,
    } atyp;
#define NVP_ACF_MASK_FFR  0x0107010 // all FlightFactor addons
#define NVP_ACF_MASK_FJS  0x0010140 // all of FlyJSim's addons
#define NVP_ACF_MASK_JDN  0x0000005 // all J.A.R.Design addons
#define NVP_ACF_MASK_QPC  0x000002A // all QPAC-powered addons
#define NVP_ACF_MASK_SSG  0x0020000 // all SSGroup/FJCC addons
#define NVP_ACF_MASK_XCR  0x0040000 // all X-Crafts/S.W addons
#define NVP_ACF_MASK_320  0x0000003 // all A320 series aircraft
#define NVP_ACF_MASK_330  0x000000C // all A330 series aircraft
#define NVP_ACF_MASK_350  0x0000010 // all A350 series aircraft
#define NVP_ACF_MASK_380  0x0000020 // all A380 series aircraft
#define NVP_ACF_MASK_727  0x0000040 // all B727 series aircraft
#define NVP_ACF_MASK_737  0x0000380 // all B737 series aircraft
#define NVP_ACF_MASK_757  0x0001000 // all B757 series aircraft
#define NVP_ACF_MASK_767  0x0002000 // all B767 series aircraft
#define NVP_ACF_MASK_777  0x0004000 // all B777 series aircraft
#define NVP_ACF_MASK_EMB  0x00E0000 // all EMB* series aircraft
    /*
     * Note to self: the QPAC plugin (at least A320) seems to overwrite radio
     * frequency datarefs with its own; I found the datarefs but they're not
     * writable. There are custom commands though:
     *
     * - AirbusFBW/RMP3FreqUpLrg
     * - AirbusFBW/RMP3FreqUpSml
     * - AirbusFBW/RMP3FreqDownLrg
     * - AirbusFBW/RMP3FreqDownSml
     * - AirbusFBW/RMP3Swap etc.
     *
     * A plugin could control radio frequencies manually via said commands.
     *
     * It's worth noting that RMP1 != COM1, RMP2 != COM2; each RMP may control
     * either of COM1 or 2; before changing frequencies via an RMP, we must set
     * the RMP to the COM radio we want to change frequency for using the
     * relevant custom command:
     *
     * - AirbusFBW/VHF1RMP3
     * - AirbusFBW/VHF2RMP3
     *
     * RMP3 should be the backup radio panel located overhead the pilots.
     */

    struct
    {
        int         var_park_brake;
        XPLMDataRef ref_park_brake;
        int         var_speedbrake;
        XPLMDataRef ref_speedbrake;
        int         var_flap_lever;
        XPLMDataRef ref_flap_lever;

        // named, plane-specific (hardcoded) flap callouts
        chandler_callback cb_flapu;
        chandler_callback cb_flapd;
        XPLMDataRef ref_flap_ratio;
        XPLMFlightLoop_f flc_flaps;
    } callouts;

    struct
    {
        refcon_ff_a350 a350;
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

        int         n_engines;
        XPLMDataRef acf_numng;
        XPLMDataRef prop_mode;
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
        struct
        {
            chandler_callback cb;
            chandler_command  cc;
        } disc;
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
} chandler_context;

/* Callout default values */
#define CALLOUT_PARKBRAKE 1
#define CALLOUT_SPEEDBRAK 1
#define CALLOUT_FLAPLEVER 1

/* Flap lever position name constants */
static       char  _flap_callout_st[11];
static const char* _flap_names_2POS[10] = { "up",  "half",  "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_A10W[10] = { "up",    "15",    "30",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_AIB1[10] = { "up",     "1",     "2",    "3", "full",   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_B52G[10] = { "up",    "10",    "20",   "30",   "40",   "55",   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_BD5J[10] = { "up",    "10",    "20",   "30",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_BNG1[10] = { "up",     "1",     "2",    "5",   "10",   "15",   "25",   "30",   "40",   NULL, };
static const char* _flap_names_BNG2[10] = { "up",     "1",     "5",   "15",   "20",   "25",   "30",   NULL,   NULL,   NULL, };
static const char* _flap_names_BOM1[10] = { "up",    "10",    "20",   "30",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_BOM2[10] = { "up",     "5",    "10",   "15",   "35",   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_C130[10] = { "up",     "1",     "2",    "3",    "4",    "5",    "6", "full",   NULL,   NULL, };
static const char* _flap_names_C172[10] = { "up",    "10",    "20", "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_DC10[10] = { "up",     "5",    "15",   "25",   "30",   "35",   "40",   NULL,   NULL,   NULL, };
static const char* _flap_names_EMB1[10] = { "up",     "9",    "18",   "22",   "45",   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_EMB2[10] = { "up",     "1",     "2",    "3",    "4",    "5", "full",   NULL,   NULL,   NULL, };
static const char* _flap_names_FA7X[10] = { "up",     "1",     "2", "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_PA32[10] = { "up",    "10",    "25",   "40",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_PC12[10] = { "up",    "15",    "30",   "40",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, };
static const char* _flap_names_TBM8[10] = { NULL,   "up",  "half",  "full",   NULL,   NULL,   NULL,   NULL,   NULL,   NULL, }; // because: Carenado :-(
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

static int  dataref_read_string(XPLMDataRef dataref, char *string_buffer,  size_t buffer_size);
static int  chandler_turna(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_p_max(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_p_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_b_max(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_b_reg(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_swtch(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_sp_ex(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_sp_re(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_pt_up(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_pt_dn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_at_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_at_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_rt_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_rt_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_r_fwd(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_r_rev(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_sview(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_qlprv(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_qlnxt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_flchg(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int  chandler_mcdup(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static float flc_flap_func(                                          float, float, int, void*);
static int  first_fcall_do(                                             chandler_context *ctx);
static int  aibus_350_init(                                               refcon_ff_a350 *ffa);
static int  aibus_fbw_init(                                               refcon_qpacfbw *fbw);
static int  boing_733_init(                                               refcon_ixeg733 *i33);
static int  boing_738_init(                                               refcon_eadt738 *x38);
static int  priv_getdata_i(                                       void *inRefcon             );
static void priv_setdata_i(                                       void *inRefcon, int inValue);

void* nvp_chandlers_init(void)
{
    chandler_context *ctx = calloc(1, sizeof(chandler_context));
    if (!ctx)
    {
        return NULL;
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
                                                            &priv_getdata_i,
                                                            &priv_setdata_i,
                                                            NULL, NULL, NULL, NULL, NULL,
                                                            NULL, NULL, NULL, NULL, NULL,
                                                            &ctx->callouts.var_park_brake,
                                                            &ctx->callouts.var_park_brake);
    ctx->callouts.var_speedbrake = CALLOUT_SPEEDBRAK;
    ctx->callouts.ref_speedbrake = XPLMRegisterDataAccessor("navP/callouts/speedbrake",
                                                            xplmType_Int, 1,
                                                            &priv_getdata_i,
                                                            &priv_setdata_i,
                                                            NULL, NULL, NULL, NULL, NULL,
                                                            NULL, NULL, NULL, NULL, NULL,
                                                            &ctx->callouts.var_speedbrake,
                                                            &ctx->callouts.var_speedbrake);
    ctx->callouts.var_flap_lever = CALLOUT_FLAPLEVER;
    ctx->callouts.ref_flap_lever = XPLMRegisterDataAccessor("navP/callouts/flap_lever",
                                                            xplmType_Int, 1,
                                                            &priv_getdata_i,
                                                            &priv_setdata_i,
                                                            NULL, NULL, NULL, NULL, NULL,
                                                            NULL, NULL, NULL, NULL, NULL,
                                                            &ctx->callouts.var_flap_lever,
                                                            &ctx->callouts.var_flap_lever);
    if (!ctx->callouts.ref_park_brake ||
        !ctx->callouts.ref_speedbrake ||
        !ctx->callouts.ref_flap_lever)
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
    ctx->bking.rc_brk.l_b_rat = XPLMFindDataRef  ("sim/cockpit2/controls/left_brake_ratio");
    ctx->bking.rc_brk.r_b_rat = XPLMFindDataRef  ("sim/cockpit2/controls/right_brake_ratio");
    if (!ctx->bking.tur.cb.command ||
        !ctx->bking.prk.cb.command || !ctx->bking.off.cb.command                      ||
        !ctx->bking.max.cb.command || !ctx->bking.reg.cb.command                      ||
        !ctx->bking.rc_brk.p_b_rat || !XPLMCanWriteDataRef(ctx->bking.rc_brk.l_b_rat) ||
        !ctx->bking.rc_brk.l_b_rat || !XPLMCanWriteDataRef(ctx->bking.rc_brk.l_b_rat) ||
        !ctx->bking.rc_brk.r_b_rat || !XPLMCanWriteDataRef(ctx->bking.rc_brk.r_b_rat))
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
    ctx->revrs.acf_numng      = XPLMFindDataRef  ("sim/aircraft/engine/acf_num_engines");
    ctx->revrs.prop_mode      = XPLMFindDataRef  ("sim/cockpit2/engine/actuators/prop_mode");
    if (!ctx->revrs.fwd.cb.command || !ctx->revrs.rev.cb.command ||
        !ctx->revrs.acf_numng      || !ctx->revrs.prop_mode)
    {
        goto fail;
    }
    else
    {
        REGISTER_CHANDLER(ctx->revrs.fwd.cb, chandler_r_fwd, 0, ctx);
        REGISTER_CHANDLER(ctx->revrs.rev.cb, chandler_r_rev, 0, ctx);
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
    ctx->otto.disc.cb.command = XPLMCreateCommand("navP/switches/ap_disc", "A/P disconnect");
    ctx->athr.disc.cb.command = XPLMCreateCommand("navP/switches/at_disc", "A/T disconnect");
    ctx->athr.toga.cb.command = XPLMCreateCommand("navP/switches/at_toga", "A/T takeoff/GA");
    if (!ctx->otto.disc.cb.command ||
        !ctx->athr.disc.cb.command ||
        !ctx->athr.toga.cb.command)
    {
        goto fail;
    }
    else
    {
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
    UNREGSTR_CHANDLER(ctx->otto.   disc.cb);
    UNREGSTR_CHANDLER(ctx->athr.   disc.cb);
    UNREGSTR_CHANDLER(ctx->athr.   toga.cb);
    UNREGSTR_CHANDLER(ctx->views.  prev.cb);
    UNREGSTR_CHANDLER(ctx->views.  next.cb);
    for (int i = 0; i < 10; i++)
    {
        UNREGSTR_CHANDLER(ctx->views.cbs[i]);
    }
    UNREGSTR_CHANDLER(ctx->callouts.cb_flapu);
    UNREGSTR_CHANDLER(ctx->callouts.cb_flapd);
    UNREGSTR_CHANDLER(ctx->mcdu.          cb);

    /* …and all datarefs */
    if (ctx->acfspec.qpac.pkb_tmp)
    {
        XPLMUnregisterDataAccessor(ctx->acfspec.qpac.pkb_tmp);
        ctx->acfspec.qpac.pkb_tmp = NULL;
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
    if (ref_ql_idx_get())
    {
        XPLMUnregisterDataAccessor(ref_ql_idx_get());
        ref_ql_idx_set            (NULL);
    }

    /* …and all callbacks */
    XPLMUnregisterFlightLoopCallback(ctx->callouts.flc_flaps, NULL);

    /* all good */
    free(ctx);
    return 0;
}

int nvp_chandlers_reset(void *inContext)
{
    chandler_context *ctx = inContext;
    if (!ctx)
    {
        return -1;
    }

    /* Reset the aircraft/addon type */
    ctx->atyp = NVP_ACF_GENERIC;

    /* Don't use 3rd-party commands/datarefs until we know the plane we're in */
    ctx->acfspec.a350.ready = 0;
    ctx->acfspec.qpac.ready = 0;
    ctx->acfspec.i733.ready = 0;
    ctx->acfspec.x738.ready = 0;
    ctx->athr.disc.cc.name  = NULL;
    ctx->athr.toga.cc.name  = NULL;
    ctx->otto.disc.cc.name  = NULL;

    /* Reset engine count */
    ctx->revrs.n_engines = -1;

    /* all good */
    ndt_log("navP [info]: nvp_chandlers_reset OK\n"); return (ctx->initialized = 0);
}

int nvp_chandlers_update(void *inContext)
{
    XPLMDataRef xdref_acf_ICAO = NULL;
    char xaircraft_icao_code[41] = "";
    char acf_file[257], acf_path[513];
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

    /* get the aircraft path and model information */
    if ((xdref_acf_ICAO = XPLMFindDataRef("sim/aircraft/view/acf_ICAO")))
    {
        dataref_read_string(xdref_acf_ICAO, xaircraft_icao_code, sizeof(xaircraft_icao_code));
    }
    XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, acf_file, acf_path);

    /* check enabled plugins to determine which plane we're flying */
    do // dummy loop we can break out of
    {
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("jardesign.sound3d"))
        {
            if (!strcasecmp(xaircraft_icao_code, "A320"))
            {
                ndt_log("navP [info]: plane is J.A.R. Design Airbus A320-200 CFM\n");
                ctx->atyp = NVP_ACF_A320_JD;
                break;
            }
            if (1) // TODO: add check for ICAO code when I know it
            {
                ndt_log("navP [info]: plane is J.A.R. Design Airbus A330-200 RR\n");
                ctx->atyp = NVP_ACF_A330_JD;
                break;
            }
            ndt_log("navP [warning]: no aircraft type match despite plugin (jardesign.sound3d)\n");
            break; // fall back to generic
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("QPAC.airbus.fbw"))
        {
            if (!strcasecmp(xaircraft_icao_code, "A320"))
            {
                ndt_log("navP [info]: plane is QPAC Airbus A320-200 IAE\n");
                ctx->atyp = NVP_ACF_A320_QP;
                break;
            }
            if (!strlen(xaircraft_icao_code))
            {
                if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("FFSTSmousehandler"))
                {
                    ndt_log("navP [info]: plane is FlightFactor-QPAC Airbus A350 XWB Advanced\n");
                    ctx->atyp = NVP_ACF_A350_FF;
                    break;
                }
                ndt_log("navP [info]: plane is RWDesigns-QPAC Airbus A330-300\n");
                ctx->atyp = NVP_ACF_A330_RW;
                break;
            }
            ndt_log("navP [warning]: no aircraft type match despite plugin (QPAC.airbus.fbw)\n");
            break; // fall back to generic
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("QPAC.A380.airbus.fbw"))
        {
            if (!strcasecmp(xaircraft_icao_code, "A388"))
            {
                ndt_log("navP [info]: plane is Peter Hager's Airbus A380-800 QPAC\n");
                ctx->atyp = NVP_ACF_A380_PH;
                break;
            }
            ndt_log("navP [warning]: no aircraft type match despite plugin (QPAC.A380.airbus.fbw)\n");
            ctx->atyp = NVP_ACF_A380_PH; // still an A380 variant
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("bs.x737.plugin"))
        {
            if (!strcasecmp(xaircraft_icao_code, "B737") ||
                !strcasecmp(xaircraft_icao_code, "B738"))
            {
                ndt_log("navP [info]: plane is EADT Boeing x737-800\n");
                ctx->atyp = NVP_ACF_B737_EA;
                break;
            }
            ndt_log("navP [warning]: no aircraft type match despite plugin (bs.x737.plugin)\n");
            break; // fall back to generic
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("de-ru.philippmuenzel-den_rain.757avionics") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("ru.flightfactor-steptosky.757767avionics"))
        {
            if (!strcasecmp(xaircraft_icao_code, "B763"))
            {
                ndt_log("navP [info]: plane is FlightFactor-StepToSky Boeing 767 Professional\n");
                ctx->atyp = NVP_ACF_B767_FF;
                break;
            }
            ndt_log("navP [warning]: no aircraft type match despite plugin (757767avionics)\n");
            ctx->atyp = NVP_ACF_B767_FF; // still a 75/76 variant
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("de.philippmuenzel.t7avionics"))
        {
            if (!strcasecmp(xaircraft_icao_code, "B77L"))
            {
                ndt_log("navP [info]: plane is FlightFactor Boeing 777 Worldliner Professional\n");
                ctx->atyp = NVP_ACF_B777_FF;
                break;
            }
            ndt_log("navP [warning]: no aircraft type match despite plugin (t7avionics)\n");
            ctx->atyp = NVP_ACF_B777_FF; // still a T7 variant
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("FJCC.SSGERJ"))
        {
            if (!strcasecmp(xaircraft_icao_code, "E170"))
            {
                ndt_log("navP [info]: plane is SSG Embraer E-Jet 170 Evolution\n");
                ctx->atyp = NVP_ACF_EMBE_SS;
                break;
            }
            ndt_log("navP [warning]: no aircraft type match despite plugin (SSGERJ)\n");
            ctx->atyp = NVP_ACF_EMBE_SS;
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("DreamFoil.DreamEngine"))
        {
            if (!strcasecmp(xaircraft_icao_code, "B732"))
            {
                ndt_log("navP [info]: plane is FlyJSim Boeing 737-200 Advanced Twinjet\n");
                ctx->atyp = NVP_ACF_B737_FJ;
                break;
            }
            // fall through (no generic fallback, plugin not developer-specific)
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("1-sim.sasl"))
        {
            if (!strcasecmp(xaircraft_icao_code, "DH8D"))
            {
                ndt_log("navP [info]: plane is FlyJSim Bombardier Dash 8 Q400\n");
                ctx->atyp = NVP_ACF_DH8D_FJ;
                break;
            }
            if (!strcasecmp(xaircraft_icao_code, "E135"))
            {
                ndt_log("navP [info]: plane is Dan Klaue's ERJ-140 Regional Jet\n");
                ctx->atyp = NVP_ACF_ERJ1_4D;
                break;
            }
            if (!strcasecmp(xaircraft_icao_code, "E175"))
            {
                ndt_log("navP [info]: plane is X-Crafts Embraer E-Jet E175\n");
                ctx->atyp = NVP_ACF_EMBE_XC;
                break;
            }
            if (!strcasecmp(xaircraft_icao_code, "E195"))
            {
                ndt_log("navP [info]: plane is X-Crafts Embraer E-Jet E195LR\n");
                ctx->atyp = NVP_ACF_EMBE_XC;
                break;
            }
            if (!strcasecmp(xaircraft_icao_code, "SU95") || !strcasecmp(xaircraft_icao_code, "S95"))
            {
                ndt_log("navP [info]: plane is Ramzzess Sukhoi Superjet 100-95LR\n");
                ctx->atyp = NVP_ACF_SSJ1_RZ;
                break;
            }
            // fall through (no generic fallback, plugin not developer-specific)
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("gizmo.x-plugins.com"))
        {
            if (!strcasecmp(xaircraft_icao_code, "B733"))
            {
                XPLMDataRef override_throttles =
                XPLMFindDataRef("sim/operation/override/override_throttles");
                if  (override_throttles && XPLMGetDatai(override_throttles))
                {
                    ndt_log("navP [info]: plane is IXEG Boeing 737-300 Classic\n");
                    ctx->atyp = NVP_ACF_B737_XG;
                    break;
                }
            }
            // fall through (no generic fallback, Gizmo running for all planes)
        }
    }
    while (0);

    /* ICAO type designator (for use by e.g. some of the flap callouts) */
    switch (ctx->atyp)
    {
        case NVP_ACF_A330_RW:
            snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "A333");
            break;

        case NVP_ACF_A350_FF:
            snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "A359");
            break;

        case NVP_ACF_SSJ1_RZ:
            snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "SU95");
            break;

        case NVP_ACF_GENERIC:
            if (!strcasecmp(xaircraft_icao_code, "VIPJ"))  // Aerobask Epic Victory (bug)
            {
                snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "EVIC");
                break;
            }
            if (!strnlen(xaircraft_icao_code, 1)) // XXX
            {
                if (!strcasecmp(acf_file, "787.acf"))
                {
                    snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "B787");
                    break;
                }
                if (!strcasecmp(acf_file, "A10.acf"))
                {
                    snprintf(ctx->icao, sizeof(ctx->icao), "%.4s",  "A10");
                    break;
                }
                if (!strcasecmp(acf_file, "akoya.acf"))
                {
                    snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "AKOY");
                    break;
                }
                if (!strcasecmp(acf_file, "avanti.acf"))
                {
                    snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "P180");
                    break;
                }
                if (!strcasecmp(acf_file, "B-52G NASA.acf"))
                {
                    snprintf(ctx->icao, sizeof(ctx->icao), "%.4s",  "B52");
                    break;
                }
                if (!strcasecmp(acf_file, "c4.acf"))
                {
                    snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "SF50");
                    break;
                }
                if (!strcasecmp(acf_file, "Car_TBM850.acf"))
                {
                    snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "TBM8");
                    break;
                }
                if (!strcasecmp(acf_file, "falcon7.acf"))
                {
                    snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "FA7X");
                    break;
                }
                if (!strcasecmp(acf_file, "KC-10.acf"))
                {
                    snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "DC10");
                    break;
                }
                if (!strcasecmp(acf_file, "KingAirC90B.acf"))
                {
                    snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", "BE9L");
                    break;
                }
            }
            // fall through
        default:
            snprintf(ctx->icao, sizeof(ctx->icao), "%.4s", xaircraft_icao_code);
            break;
    }

    /* plane-specific custom commands for automation disconnects, if any */
    switch (ctx->atyp)
    {
        case NVP_ACF_B737_EA:
            ctx->otto.disc.cc.name = "x737/yoke/capt_AP_DISENG_BTN";
            ctx->athr.disc.cc.name = "x737/mcp/ATHR_ARM_TOGGLE";
            ctx->athr.toga.cc.name = "x737/mcp/TOGA_TOGGLE";
            break;

        case NVP_ACF_B737_XG:
            ctx->otto.disc.cc.name = "ixeg/733/autopilot/AP_disengage";
            ctx->athr.disc.cc.name = "ixeg/733/autopilot/at_disengage";
            ctx->athr.toga.cc.name = "sim/engines/TOGA_power";
            break;

        case NVP_ACF_B767_FF:
            ctx->otto.disc.cc.name = "1-sim/comm/AP/ap_disc";
            ctx->athr.disc.cc.name = "1-sim/comm/AP/at_disc";
            ctx->athr.toga.cc.name = "1-sim/comm/AP/at_toga";
            break;

        case NVP_ACF_B777_FF:
            ctx->otto.disc.cc.name = "777/ap_disc";
            ctx->athr.disc.cc.name = "777/at_disc";
            ctx->athr.toga.cc.name = "777/at_toga";
            break;

        case NVP_ACF_EMBE_SS:
            ctx->otto.disc.cc.name = "SSG/UFMC/AP_discon_Button";
            ctx->athr.disc.cc.name = "SSG/UFMC/AP_ARM_AT_Switch";
            ctx->athr.toga.cc.name = "SSG/UFMC/TOGA_Button";
            break;

        case NVP_ACF_GENERIC:
        default:
            ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
            ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
            if (ctx->atyp == NVP_ACF_GENERIC)
            {
                ndt_log("navP [info]: plane is generic (ICAO: \"%s\")\n", xaircraft_icao_code);
            }
            break;
    }

    /*
     * reset MCDU pop-up handler status.
     *
     * pre-emptively disable X-FMC for all aircraft; if
     * required, it gets re-enabled later automatically.
     */
    XPLMPluginID xfmc = XPLMFindPluginBySignature("x-fmc.com");
    if (xfmc != XPLM_NO_PLUGIN_ID && XPLMIsPluginEnabled(xfmc))
    {
        XPLMDisablePlugin(xfmc);
    }
    ctx->mcdu.rc.acftyp = ctx->atyp;
    ctx->mcdu.rc.status = -1;

    /* all good */
    ndt_log("navP [info]: nvp_chandlers_update OK\n"); XPLMSpeakString("nav P configured"); return 0;
}

static int dataref_read_string(XPLMDataRef dataref, char *string_buffer, size_t buffer_size)
{
    if (dataref && string_buffer && buffer_size)
    {
        int len = XPLMGetDatab(dataref, string_buffer, 0, buffer_size - 1);
        if (len > 0)
        {
            string_buffer[buffer_size - 1] = '\0';
            return 0;
        }
        string_buffer[0] = '\0';
        return -1;
    }
    return ENOMEM;
}

/*
 * action: set an aircraft's turnaround state to a pilot-friendly cold & dark variant.
 */
static int chandler_turna(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    int speak = XPLMGetDatai(ctx->callouts.ref_park_brake);
    /*
     * Do any additional aircraft-specific stuff that can't be done earlier.
     */
    if (ctx->atyp == NVP_ACF_A350_FF)
    {
        if (ctx->acfspec.a350.ready == 0)
        {
            aibus_350_init(&ctx->acfspec.a350);
        }
    }
    if (ctx->atyp & NVP_ACF_MASK_QPC)
    {
        if (ctx->acfspec.qpac.ready == 0)
        {
            aibus_fbw_init(&ctx->acfspec.qpac);
        }
    }
    if (ctx->first_fcall && inPhase == xplm_CommandEnd)
    {
        if (first_fcall_do(ctx) == 0 && ctx->first_fcall == 0)
        {
            if ((ctx->atyp & NVP_ACF_MASK_JDN) == 0)
            {
                XPLMSpeakString("turn around set");
            }
            XPLMCommandOnce(ctx->views.cbs[1].command);
        }
        if (speak) XPLMSetDatai(ctx->callouts.ref_park_brake, 0);
        XPLMCommandOnce        (ctx->bking.prk.cb.      command);
        if (speak) XPLMSetDatai(ctx->callouts.ref_park_brake, 1);
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
    chandler_context *ctx = inRefcon;
    refcon_braking   *rcb = &ctx->bking.rc_brk;
    int speak = XPLMGetDatai(ctx->callouts.ref_park_brake);
//  if (ctx->kill_daniel)
//  {
//      // TODO: implement
//  }
    if (ctx->atyp & NVP_ACF_MASK_JDN)
    {
        speak = 0;
    }
    if (ctx->atyp == NVP_ACF_A350_FF)
    {
        if (ctx->acfspec.a350.ready == 0)
        {
            aibus_350_init(&ctx->acfspec.a350);
        }
        if (ctx->acfspec.a350.ready && inPhase == xplm_CommandEnd)
        {
            XPLMSetDatai(ctx->acfspec.a350.pkb_ref, 0); // inverted
            if (speak) XPLMSpeakString("park brake set");
        }
        return 0;
    }
    if (ctx->atyp & NVP_ACF_MASK_QPC)
    {
        if (ctx->acfspec.qpac.ready == 0)
        {
            aibus_fbw_init(&ctx->acfspec.qpac);
        }
        if (ctx->acfspec.qpac.ready && inPhase == xplm_CommandEnd)
        {
            XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, 1);
            XPLMSetDatai(ctx->acfspec.qpac.pkb_tmp, 1);
            if (speak) XPLMSpeakString("park brake set");
        }
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        XPLMSetDataf(rcb->p_b_rat, 1.0f);
        if (speak) XPLMSpeakString("park brake set");
    }
    return 0;
}

static int chandler_p_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    refcon_braking   *rcb = &ctx->bking.rc_brk;
    int speak = XPLMGetDatai(ctx->callouts.ref_park_brake);
    if (ctx->atyp & NVP_ACF_MASK_JDN)
    {
        speak = 0;
    }
    if (ctx->atyp == NVP_ACF_A350_FF)
    {
        if (ctx->acfspec.a350.ready == 0)
        {
            aibus_350_init(&ctx->acfspec.a350);
        }
        if (ctx->acfspec.a350.ready && inPhase == xplm_CommandEnd)
        {
            XPLMSetDatai(ctx->acfspec.a350.pkb_ref, 1); // inverted
            if (speak) XPLMSpeakString("park brake released");
        }
        return 0;
    }
    if (ctx->atyp & NVP_ACF_MASK_QPC)
    {
        if (ctx->acfspec.qpac.ready == 0)
        {
            aibus_fbw_init(&ctx->acfspec.qpac);
        }
        if (ctx->acfspec.qpac.ready && inPhase == xplm_CommandEnd)
        {
            XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, 0);
            XPLMSetDatai(ctx->acfspec.qpac.pkb_tmp, 0);
            if (speak) XPLMSpeakString("park brake released");
        }
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        XPLMSetDataf(rcb->p_b_rat, 0.0f);
        if (speak) XPLMSpeakString("park brake released");
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
    chandler_context *ctx = inRefcon;
    refcon_braking   *rcb = &ctx->bking.rc_brk;
    if (ctx->atyp & NVP_ACF_MASK_QPC)
    {
        if (ctx->acfspec.qpac.ready == 0)
        {
            aibus_fbw_init(&ctx->acfspec.qpac);
        }
        if (ctx->acfspec.qpac.ready)
        {
            switch (inPhase)
            {
                case xplm_CommandBegin:
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_tmp, XPLMGetDatai(ctx->acfspec.qpac.pkb_ref));
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, 1);
                    XPLMCommandBegin(ctx->acfspec.qpac.h_b_max);
                    break;
                case xplm_CommandEnd:
                    XPLMCommandEnd(ctx->acfspec.qpac.h_b_max);
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, XPLMGetDatai(ctx->acfspec.qpac.pkb_tmp));
                    break;
                default:
                    break;
            }
        }
        return 0;
    }
    /*
     * Using 90% of max. braking makes the command behave the same
     * as brakes_regular, i.e. it doesn't unset the parking brake :-)
     */
    switch (inPhase)
    {
        case xplm_CommandBegin:
            XPLMSetDataf(rcb->l_b_rat, 0.9f);
            XPLMSetDataf(rcb->r_b_rat, 0.9f);
            break;
        case xplm_CommandContinue:
            XPLMSetDataf(rcb->l_b_rat, 0.9f);
            XPLMSetDataf(rcb->r_b_rat, 0.9f);
            break;
        case xplm_CommandEnd:
        default:
            XPLMSetDataf(rcb->l_b_rat, 0.0f);
            XPLMSetDataf(rcb->r_b_rat, 0.0f);
            break;
    }
    return 0;
}

static int chandler_b_reg(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    refcon_braking   *rcb = &ctx->bking.rc_brk;
    if (ctx->atyp & NVP_ACF_MASK_QPC)
    {
        if (ctx->acfspec.qpac.ready == 0)
        {
            aibus_fbw_init(&ctx->acfspec.qpac);
        }
        if (ctx->acfspec.qpac.ready)
        {
            switch (inPhase)
            {
                case xplm_CommandBegin:
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_tmp, XPLMGetDatai(ctx->acfspec.qpac.pkb_ref));
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, 1);
                    XPLMCommandBegin(ctx->acfspec.qpac.h_b_reg);
                    break;
                case xplm_CommandEnd:
                    XPLMCommandEnd(ctx->acfspec.qpac.h_b_reg);
                    XPLMSetDatai(ctx->acfspec.qpac.pkb_ref, XPLMGetDatai(ctx->acfspec.qpac.pkb_tmp));
                    break;
                default:
                    break;
            }
        }
        return 0;
    }
    switch (inPhase)
    {
        case xplm_CommandBegin:
            XPLMSetDataf(rcb->l_b_rat, 0.3f);
            XPLMSetDataf(rcb->r_b_rat, 0.3f);
            break;
        case xplm_CommandContinue:
            XPLMSetDataf(rcb->l_b_rat, 0.3f);
            XPLMSetDataf(rcb->r_b_rat, 0.3f);
            break;
        case xplm_CommandEnd:
        default:
            XPLMSetDataf(rcb->l_b_rat, 0.0f);
            XPLMSetDataf(rcb->r_b_rat, 0.0f);
            break;
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
        refcon_ixeg733   *i33 = NULL;
        refcon_eadt738   *x38 = NULL;
        chandler_context *ctx = inRefcon;
        int speak = XPLMGetDatai(ctx->callouts.ref_speedbrake);
        switch (ctx->atyp)
        {
            case NVP_ACF_B737_EA:
                x38 = &ctx->acfspec.x738;
                if (x38->ready == 0)
                {
                    boing_738_init(x38);
                }
                break;

            case NVP_ACF_B737_XG:
                i33 = &ctx->acfspec.i733;
                if (i33->ready == 0)
                {
                    boing_733_init(i33);
                }
                break;

            case NVP_ACF_A320_JD:
            case NVP_ACF_A330_JD:
                speak = 0; // fall through
            default:
                XPLMCommandOnce (ctx->spbrk.sext);
                if (XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    if (speak) XPLMSpeakString("spoilers disarmed");
                    return 0;
                }
                if (XPLMGetDataf(ctx->spbrk.srat) > +.01f)
                {
                    if (speak) XPLMSpeakString("speedbrake");
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
                    if (speak) XPLMSpeakString("spoilers disarmed");
                    return 0;
                }
                XPLMSetDataf(i33->slat, 0.8f);        // extend: flight detent
                if (speak) XPLMSpeakString("speedbrake");
                return 0;
            }
            if (x38 && x38->ready)
            {
                if (ratio > .1f && ratio < .2f)
                {
                    XPLMCommandOnce(x38->spret);      // extend: disarm spoilers
                    if (speak) XPLMSpeakString("spoilers disarmed");
                    return 0;
                }
                XPLMCommandOnce(ctx->spbrk.sext);     // extend: one
                if (speak) XPLMSpeakString("speedbrake");
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
        refcon_ixeg733   *i33 = NULL;
        refcon_eadt738   *x38 = NULL;
        chandler_context *ctx = inRefcon;
        int speak = XPLMGetDatai(ctx->callouts.ref_speedbrake);
        switch (ctx->atyp)
        {
            case NVP_ACF_B737_EA:
                x38 = &ctx->acfspec.x738;
                if (x38->ready == 0)
                {
                    boing_738_init(x38);
                }
                break;

            case NVP_ACF_B737_XG:
                i33 = &ctx->acfspec.i733;
                if (i33->ready == 0)
                {
                    boing_733_init(i33);
                }
                break;

            case NVP_ACF_A320_JD:
            case NVP_ACF_A330_JD:
                speak = 0; // fall through
            default:
                XPLMCommandOnce (ctx->spbrk.sret);
                if (XPLMGetDataf(ctx->spbrk.srat) < -.01f)
                {
                    if (speak) XPLMSpeakString("spoilers armed");
                    return 0;
                }
                if (XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    if (speak) XPLMSpeakString("speedbrake retracted");
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
                    if (speak) XPLMSpeakString("spoilers armed");
                    return 0;
                }
                if (ratio > +.51f)
                {
                    XPLMSetDataf(i33->slat, 0.8f);    // retract: flight detent
                    return 0;
                }
                XPLMSetDataf(i33->slat, 0.0f);        // retract: fully
                if (speak) XPLMSpeakString("speedbrake retracted");
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
                    if (speak) XPLMSpeakString("spoilers armed");
                    return 0;
                }
                XPLMCommandOnce (ctx->spbrk.sret);    // retract: one
                if (XPLMGetDataf(ctx->spbrk.srat) < .01f)
                {
                    if (speak) XPLMSpeakString("speedbrake retracted");
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
    switch (ctx->atyp)
    {
        case NVP_ACF_B737_EA:
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
    switch (ctx->atyp)
    {
        case NVP_ACF_B737_EA:
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
    switch (ctx->atyp)
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
    switch (ctx->atyp)
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
    switch (ctx->atyp)
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
    switch (ctx->atyp)
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
    if (ctx->revrs.n_engines == -1)
    {
        ctx->revrs.n_engines = XPLMGetDatai(ctx->revrs.acf_numng);
        ctx->revrs.n_engines = ctx->revrs.n_engines > 8 ? 8 : ctx->revrs.n_engines;
        ctx->revrs.n_engines = ctx->revrs.n_engines < 0 ? 0 : ctx->revrs.n_engines;
    }
    if (ctx->revrs.n_engines >= 1)
    {
        XPLMSetDatavi(ctx->revrs.prop_mode, propmode_fwd_get(), 0, ctx->revrs.n_engines);
    }
    return 0;
}

static int chandler_r_rev(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    if (ctx->revrs.n_engines == -1)
    {
        ctx->revrs.n_engines = XPLMGetDatai(ctx->revrs.acf_numng);
        ctx->revrs.n_engines = ctx->revrs.n_engines > 8 ? 8 : ctx->revrs.n_engines;
        ctx->revrs.n_engines = ctx->revrs.n_engines < 0 ? 0 : ctx->revrs.n_engines;
    }
    if (ctx->revrs.n_engines >= 1)
    {
        XPLMSetDatavi(ctx->revrs.prop_mode, propmode_rev_get(), 0, ctx->revrs.n_engines);
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
    chandler_command *cc = inRefcon;
    if (cc->name)
    {
        if (cc->xpcr == NULL)
        {
            cc->xpcr = XPLMFindCommand(cc->name);
            if (NULL == cc->xpcr)
            {
                XPLMSpeakString("Failed to resolve command");
            }
        }
        if (cc->xpcr && inPhase == xplm_CommandEnd)
        {
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
        if (!XPLMGetDatai(ctx->callouts.ref_flap_lever))
        {
            return 1;
        }
        switch (ctx->atyp)
        {
            case NVP_ACF_A320_QP:
            case NVP_ACF_A330_RW:
            case NVP_ACF_A350_FF:
            case NVP_ACF_A380_PH:
            case NVP_ACF_SSJ1_RZ:
                flap_callout_setst(_flap_names_AIB1, lroundf(4.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
//          case NVP_ACF_B727_FJ:
            case NVP_ACF_B737_EA:
            case NVP_ACF_B737_FJ:
            case NVP_ACF_B737_XG:
                flap_callout_setst(_flap_names_BNG1, lroundf(8.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case NVP_ACF_B757_FF:
            case NVP_ACF_B767_FF:
            case NVP_ACF_B777_FF:
                flap_callout_setst(_flap_names_BNG2, lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case NVP_ACF_DH8D_FJ:
                flap_callout_setst(_flap_names_BOM2, lroundf(4.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case NVP_ACF_ERJ1_4D:
                flap_callout_setst(_flap_names_EMB1, lroundf(4.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            case NVP_ACF_EMBE_SS:
            case NVP_ACF_EMBE_XC:
                flap_callout_setst(_flap_names_EMB2, lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                break;
            default:
                if (!strcasecmp(ctx->icao, "A10"))
                {
                    flap_callout_setst(_flap_names_A10W, lroundf(2.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "AKOY") ||
                    !strcasecmp(ctx->icao, "BE58") ||
                    !strcasecmp(ctx->icao, "BE9L") ||
                    !strcasecmp(ctx->icao, "COL4") ||
                    !strcasecmp(ctx->icao, "EA50") ||
                    !strcasecmp(ctx->icao, "EPIC") ||
                    !strcasecmp(ctx->icao, "EVIC") ||
                    !strcasecmp(ctx->icao, "LEG2") ||
                    !strcasecmp(ctx->icao, "P180") ||
                    !strcasecmp(ctx->icao, "SF50"))
                {
                    flap_callout_setst(_flap_names_2POS, lroundf(2.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "B52"))
                {
                    flap_callout_setst(_flap_names_B52G, lroundf(5.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "B732") || // all Boeing 737 variants
                    !strcasecmp(ctx->icao, "B733") ||
                    !strcasecmp(ctx->icao, "B734") ||
                    !strcasecmp(ctx->icao, "B735") ||
                    !strcasecmp(ctx->icao, "B736") ||
                    !strcasecmp(ctx->icao, "B737") ||
                    !strcasecmp(ctx->icao, "B738") ||
                    !strcasecmp(ctx->icao, "B739") ||
                    !strcasecmp(ctx->icao, "E737") ||
                    !strcasecmp(ctx->icao, "P8"))
                {
                    flap_callout_setst(_flap_names_BNG1, lroundf(8.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "BSCA") || // all Boeing 747 variants
                    !strcasecmp(ctx->icao, "BLCF") ||
                    !strcasecmp(ctx->icao, "B74D") ||
                    !strcasecmp(ctx->icao, "B74F") ||
                    !strcasecmp(ctx->icao, "B74R") ||
                    !strcasecmp(ctx->icao, "B74S") ||
                    !strcasecmp(ctx->icao, "B741") ||
                    !strcasecmp(ctx->icao, "B742") ||
                    !strcasecmp(ctx->icao, "B743") ||
                    !strcasecmp(ctx->icao, "B744") ||
                    !strcasecmp(ctx->icao, "B747") ||
                    !strcasecmp(ctx->icao, "B748") ||
                    !strcasecmp(ctx->icao, "B752") || // all Boeing 757 variants
                    !strcasecmp(ctx->icao, "B753") ||
                    !strcasecmp(ctx->icao, "B757") ||
                    !strcasecmp(ctx->icao, "B762") || // all Boeing 767 variants
                    !strcasecmp(ctx->icao, "B763") ||
                    !strcasecmp(ctx->icao, "B764") ||
                    !strcasecmp(ctx->icao, "B767") ||
                    !strcasecmp(ctx->icao, "B77F") || // all Boeing 777 variants
                    !strcasecmp(ctx->icao, "B77L") ||
                    !strcasecmp(ctx->icao, "B77W") ||
                    !strcasecmp(ctx->icao, "B772") ||
                    !strcasecmp(ctx->icao, "B773") ||
                    !strcasecmp(ctx->icao, "B777") ||
                    !strcasecmp(ctx->icao, "B778") ||
                    !strcasecmp(ctx->icao, "B779") ||
                    !strcasecmp(ctx->icao, "B78X") || // all Boeing 787 variants
                    !strcasecmp(ctx->icao, "B787") ||
                    !strcasecmp(ctx->icao, "B788") ||
                    !strcasecmp(ctx->icao, "B789"))
                {
                    flap_callout_setst(_flap_names_BNG2, lroundf(6.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "BD5J"))
                {
                    flap_callout_setst(_flap_names_BD5J, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "C130"))
                {
                    flap_callout_setst(_flap_names_C130, lroundf(7.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "C172"))
                {
                    flap_callout_setst(_flap_names_C172, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "CL30"))
                {
                    flap_callout_setst(_flap_names_BOM1, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "DC10"))
                {
                    flap_callout_setst(_flap_names_DC10, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "FA7X"))
                {
                    flap_callout_setst(_flap_names_FA7X, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "PA32"))
                {
                    flap_callout_setst(_flap_names_PA32, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "PC12"))
                {
                    flap_callout_setst(_flap_names_PC12, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                if (!strcasecmp(ctx->icao, "TBM8"))
                {
                    flap_callout_setst(_flap_names_TBM8, lroundf(3.0f * XPLMGetDataf(ctx->callouts.ref_flap_ratio)));
                    break;
                }
                return 1;
        }
        XPLMSetFlightLoopCallbackInterval(ctx->callouts.flc_flaps, 1.0f, 1, NULL);
    }
    return 1;
}

static int chandler_mcdup(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    refcon_cdu_pop *cdu = inRefcon;
    if (cdu->status == -2)
    {
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (cdu->status == -1)
        {
            XPLMPluginID xfmc = XPLMFindPluginBySignature("x-fmc.com");
            switch (cdu->acftyp)
            {
                case NVP_ACF_A320_JD:
                case NVP_ACF_A330_JD:
                case NVP_ACF_B737_XG:
                case NVP_ACF_B757_FF:
                case NVP_ACF_B767_FF:
                case NVP_ACF_B777_FF:
                    cdu->status = -2; return 0; // custom FMS, but no popup/command

                case NVP_ACF_A320_QP:
                case NVP_ACF_A330_RW:
                case NVP_ACF_A350_FF:
                    cdu->cmd[0] = cdu->cmd[1] = XPLMFindCommand("AirbusFBW/UndockMCDU1");
                    cdu->status = 0; break;

                case NVP_ACF_EMBE_SS:
                {
                    for (int i = 0; i < XPLMCountHotKeys(); i++)
                    {
                        char outDescr[513];
                        XPLMPluginID outPl;
                        XPLMHotKeyID h_key = XPLMGetNthHotKey(i);
                        XPLMGetHotKeyInfo(h_key, NULL, NULL, outDescr, &outPl);
                        if (XPLMFindPluginBySignature("FJCC.SSGERJ") == outPl &&
                            strncasecmp(outDescr, "F8", 2) == 0)
                        {
                            /* TODO: remove this awful hack */
                            XPLMSetHotKeyCombination(h_key, XPLM_VK_X, xplm_DownFlag);
                            break;
                        }
                    }
                    cdu->status = -2; return 0;
                }

                default:
                    // X-FMC may automatically pop-up when it's enabled,
                    // no need to call the pop-up command right after init
                    if (xfmc != XPLM_NO_PLUGIN_ID && !XPLMIsPluginEnabled(xfmc))
                    {
                        XPLMEnablePlugin(xfmc);
                    }
                    cdu->cmd[0] = cdu->cmd[1] = XPLMFindCommand("xfmc/toggle");
                    cdu->status = 0; return 0;
            }
        }
        if (!cdu->cmd[0] || !cdu->cmd[1])
        {
            cdu->status = -2; return 0;
        }
        XPLMCommandOnce(cdu->cmd[!!cdu->status]);
        cdu->status  = !cdu->status;
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

#define _DO(_func, _val, _name) { if ((d_ref = XPLMFindDataRef(_name))) _func(d_ref, _val); }
static int first_fcall_do(chandler_context *ctx)
{
    XPLMCommandRef cr;
    XPLMDataRef d_ref;
    switch (ctx->atyp)
    {
        case NVP_ACF_A320_QP:
            if ((d_ref = XPLMFindDataRef("AirbusFBW/DUBrightness")))
            {
                float DUBrightness[1] = { 0.8f, };
                for  (int i = 0; i < 8; i++)
                {
                    XPLMSetDatavf(d_ref, &DUBrightness[0], i, 1);
                }
            }
            if ((d_ref = XPLMFindDataRef("AirbusFBW/OHPLightSwitches")))
            {
                int OHPLightSwitches[1] = { 1, };
                XPLMSetDatavi(d_ref, &OHPLightSwitches[0], 2, 1);               // nav&logo: system 1
                XPLMSetDatavi(d_ref, &OHPLightSwitches[0], 7, 1);               // strobes: automatic
            }
            break;

        case NVP_ACF_A350_FF:
            _DO(XPLMSetDatai,      1, "1-sim/fcu/altModeSwitch");               // FCU alt. sel. increm.  (1000ft)
            _DO(XPLMSetDataf,   3.0f, "1-sim/fcu/ndModeLeft/switch");           // ND m. sel. (cap. side) (arc)
            _DO(XPLMSetDataf,   2.0f, "1-sim/fcu/ndModeRight/switch");          // ND m. sel. (f/o. side) (nav)
            _DO(XPLMSetDataf,   1.0f, "1-sim/fcu/ndZoomLeft/switch");           // ND r. sel. (cap. side) (20)
            _DO(XPLMSetDataf,   3.0f, "1-sim/fcu/ndZoomRight/switch");          // ND r. sel. (f/o. side) (80)
            _DO(XPLMSetDatai,      1, "1-sim/radio/button/1/16");               // RMP1: microph. on VHF1 (on)
            _DO(XPLMSetDatai,      1, "1-sim/radio/push/1/1");                  // RMP1: receiver on VHF1 (on)
            _DO(XPLMSetDataf, 270.0f, "1-sim/radio/1/1/rotary");                // RMP1: receiver on VHF1 (volume)
            _DO(XPLMSetDatai,      1, "1-sim/radio/push/1/10");                 // RMP1: c. crew intercom (on)
            _DO(XPLMSetDataf, 270.0f, "1-sim/radio/1/10/rotary");               // RMP1: c. crew intercom (volume)
            _DO(XPLMSetDatai,      1, "1-sim/radio/push/1/11");                 // RMP1: p. announcements (on)
            _DO(XPLMSetDataf, 270.0f, "1-sim/radio/1/11/rotary");               // RMP1: p. announcements (volume)
            _DO(XPLMSetDatai,      1, "1-sim/1/switch");                        // Evac. panel: selector  (capt&purs)
            _DO(XPLMSetDatai,      1, "1-sim/2/switch");                        // Ext. lighting: strobe  (auto)
//          _DO(XPLMSetDatai,      1, "1-sim/3/switch");                        // Ext. lighting: beacon  (on)
            _DO(XPLMSetDatai,      1, "1-sim/4/switch");                        // Ext. lighting: navig.  (on)
            _DO(XPLMSetDatai,      1, "1-sim/5/switch");                        // Ext. lighting: logo    (auto)
            _DO(XPLMSetDatai,      1, "1-sim/20/switch");                       // Ext. lighting: emerg.  (auto)
            _DO(XPLMSetDatai,      1, "1-sim/37/switch");                       // Braking: anti-skid sw. (on)
//          _DO(XPLMSetDataf,   1.0f, "1-sim/3/cover");                         // Ground HFreq. datalink (lift cover)
//          _DO(XPLMSetDataf,   1.0f, "1-sim/8/button");                        // Ground HFreq. datalink (off)
            _DO(XPLMSetDataf,   1.0f, "1-sim/15/button");                       // Cabin panel: wireless  (auto)
//          _DO(XPLMSetDataf,   1.0f, "1-sim/7/cover");                         // Cabin panel: pas. data (lift cover)
            _DO(XPLMSetDataf,   1.0f, "1-sim/16/button");                       // Cabin panel: pas. data (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/17/button");                       // Cabin panel: sat. com. (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/18/button");                       // Cabin panel: lan. cam. (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/19/button");                       // Cabin panel: FAR4 sys. (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/20/button");                       // Cabin panel: mob. com. (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/21/button");                       // Cabin panel: IFEC sys. (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/22/button");                       // Cabin panel: DER  sys. (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/38/button");                       // Oxyg. panel: crew sup. (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/42/button");                       // Vent. panel: cooling   (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/44/button");                       // Vent. panel: c. fans 1 (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/45/button");                       // Vent. panel: c. fans 2 (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/53/button");                       // NSS: data to avionics  (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/54/button");                       // NSS: cabin data to NSS (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/230/button");                      // NSS: gatelink          (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/55/button");                       // Cargo: A/C I. v. (fwd) (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/56/button");                       // Cargo: A/C I. v. (aft) (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/57/button");                       // Cargo: A/C I. v. (blk) (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/59/button");                       // Cargo: A/C heating sw. (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/65/button");                       // Hydraulics: YELL. p. 1 (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/67/button");                       // Hydraulics: GREEN p. 1 (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/75/button");                       // Hydraulics: YELL. p. 2 (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/77/button");                       // Hydraulics: GREEN p. 2 (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/84/button");                       // Fuel: C tank feed mode (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/96/button");                       // Elec. panel: AC bus 1A (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/99/button");                       // Elec. panel: AC bus 1B (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/100/button");                      // Elec. panel: AC bus t. (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/101/button");                      // Elec. panel: APU (gen) (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/103/button");                      // Elec. panel: AC bus 2B (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/106/button");                      // Elec. panel: AC bus 2A (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/109/button");                      // Air panel: bleed en. 1 (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/110/button");                      // Air panel: heat. en. 1 (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/114/button");                      // Air panel: heat. en. 2 (auto)
            _DO(XPLMSetDataf,   1.0f, "1-sim/115/button");                      // Air panel: bleed en. 2 (auto)
            _DO(XPLMSetDatai,      1, "1-sim/air/airFlowSwitch");               // Air panel: flow selec. (norm)
            _DO(XPLMSetDatai,      1, "1-sim/air/crossBeedSwitch");             // Air panel: c.b. selec. (auto)
            _DO(XPLMSetDataf,   0.4f, "1-sim/air/ckptSettingRotery");           // Air panel: cock. temp. (purser)
            _DO(XPLMSetDataf,   0.4f, "1-sim/air/cabinSettingRotery");          // Air panel: cabin temp. (purser)
            break;

        case NVP_ACF_A380_PH:
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/instrument_brightness_ratio")))
            {
                float instrument_brightness_ratio[3] = { 0.8f, 0.4f, 0.0f, };
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0],  0, 1);   // Backlighting: switches, FCU
//              XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0],  1, 1);   // Controlled by plugin?
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0],  2, 1);   // Navigation display
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0],  3, 1);   // ECAM (upp. display)
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[2],  4, 1);   // OIS (unus. display)
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0],  5, 1);   // Primary f. display
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0],  6, 1);   // Control dis. units
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0],  7, 1);   // ECAM (low. display)
//              XPLMSetDatavf(d_ref, &instrument_brightness_ratio[2],  8, 1);   // ???
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[1],  9, 1);   // LED lam. 4 displays
//              XPLMSetDatavf(d_ref, &instrument_brightness_ratio[2], 10, 1);   // ???
//              XPLMSetDatavf(d_ref, &instrument_brightness_ratio[2], 11, 1);   // ???
//              XPLMSetDatavf(d_ref, &instrument_brightness_ratio[2], 12, 1);   // ???
//              XPLMSetDatavf(d_ref, &instrument_brightness_ratio[2], 13, 1);   // ???
//              XPLMSetDatavf(d_ref, &instrument_brightness_ratio[2], 14, 1);   // ???
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0], 15, 1);   // R.M. Panel displays, instrument outline lighting
            }
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/panel_brightness_ratio")))
            {
                float panel_brightness_ratio[1] = { 0.4f, };
                XPLMSetDatavf(d_ref, &panel_brightness_ratio[0], 0, 1);         // Cockpit/flood lights
            }
            _DO(XPLMSetDatai, 1, "sim/cockpit2/switches/navigation_lights_on");
            break;

        case NVP_ACF_B737_XG:
            _DO(XPLMSetDataf,   0.0f, "ixeg/733/aircond/aircond_cont_cabin_sel_act"); // Cont. cab. air temper. (normal)
            _DO(XPLMSetDataf,   0.0f, "ixeg/733/aircond/aircond_pass_cabin_sel_act"); // Pass. cab. air temper. (normal)
            _DO(XPLMSetDataf,   1.0f, "ixeg/733/bleedair/bleedair_recirc_fan_act");   // Bleed air recirc. fans (auto)
            _DO(XPLMSetDataf, -20.0f, "ixeg/733/ehsi/dh_pt_act");                     // ADI DH REF (cap. side) (reset)     // note: only works when power is on
            _DO(XPLMSetDataf, -20.0f, "ixeg/733/ehsi/dh_cpt_act");                    // ADI DH REF (f/o. side) (reset)     // note: only works when power is on
            _DO(XPLMSetDataf,   2.0f, "ixeg/733/ehsi/ehsi_mode_pt_act");              // HSI m.sel. (cap. side) (map)
            _DO(XPLMSetDataf,   1.0f, "ixeg/733/ehsi/ehsi_mode_cpt_act");             // HSI m.sel. (f/o. side) (exp)
            _DO(XPLMSetDataf,   0.0f, "ixeg/733/ehsi/ehsi_range_pt_act");             // HSI r.sel. (cap. side) (10)
            _DO(XPLMSetDataf,   1.0f, "ixeg/733/ehsi/ehsi_range_cpt_act");            // HSI r.sel. (f/o. side) (20)
            _DO(XPLMSetDataf,   1.0f, "ixeg/733/lighting/position_lt_act");           // Exte. lighting: posit. (on)
            _DO(XPLMSetDataf,   0.0f, "ixeg/733/rheostats/light_breakers_act");       // Circuit breakers light (off)
            _DO(XPLMSetDataf,   0.0f, "ixeg/733/rheostats/light_pedflood_act");       // Flood light (pedestal) (off)
            _DO(XPLMSetDataf,   0.8f, "ixeg/733/rheostats/light_pedpanel_act");       // Panel light (pedestal) (daylight)
            _DO(XPLMSetDataf,   0.8f, "ixeg/733/rheostats/light_overhead_act");       // Panel light (overhead) (daylight)
            _DO(XPLMSetDataf,   0.2f, "ixeg/733/rheostats/light_afds_act");           // Flood light (A.F.D.S.) (daylight)
            _DO(XPLMSetDataf,   0.8f, "ixeg/733/rheostats/light_fmc_pt_act");         // MCDU: lig. (cap. side) (daylight)
            _DO(XPLMSetDataf,   0.8f, "ixeg/733/rheostats/light_fmc_cpt_act");        // MCDU: lig. (f/o. side) (daylight)
            _DO(XPLMSetDataf,   0.0f, "ixeg/733/rheostats/light_mapbr_pt_act");       // Maps: lig. (cap. side) (off)
            _DO(XPLMSetDataf,   0.0f, "ixeg/733/rheostats/light_mapbr_cpt_act");      // Maps: lig. (f/o. side) (off)
            if ((d_ref = XPLMFindDataRef("sim/cockpit2/switches/instrument_brightness_ratio")))
            {
                float instrument_brightness_ratio[1] = { 0.8f, };
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0], 0, 1);    // Panel (cap. side): daylight
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0], 1, 1);    // Panel (f/o. side): daylight
                XPLMSetDatavf(d_ref, &instrument_brightness_ratio[0], 2, 1);    // Panel (backgrnd.): neon off
            }
            break;

        case NVP_ACF_B767_FF:
            // the following two are special, the buttons auto-revert to zero;
            // thankfully they always work (even without any electrical power)
            if ((d_ref = XPLMFindDataRef("1-sim/vor1/isAuto")) &&
                (0000 == XPLMGetDatai(d_ref)))
            {
                _DO(XPLMSetDatai,1, "anim/78/button"); // VOR 1: manual -> auto
            }
            if ((d_ref = XPLMFindDataRef("1-sim/vor2/isAuto")) &&
                (0000 == XPLMGetDatai(d_ref)))
            {
                _DO(XPLMSetDatai,1, "anim/79/button"); // VOR 2: manual -> auto
            }
//          _DO(XPLMSetDatai,    1, "anim/75/button");                          // Terrain override switch (on)
            _DO(XPLMSetDataf, 1.0f, "1-sim/gauges/terrOVRDcover");              // Terrain override switch (lift cover)
            _DO(XPLMSetDataf, 1.0f, "1-sim/electrical/batteryCover");           // Battery on/off selector (lift cover)
//          _DO(XPLMSetDataf, 1.0f, "1-sim/elecengcont/leftCover");             // Engine elec. contr. (L) (lift cover)
//          _DO(XPLMSetDataf, 1.0f, "1-sim/elecengcont/rightCover");            // Engine elec. contr. (R) (lift cover)
            _DO(XPLMSetDatai,    1, "anim/3/button");                           // Engine elec. contr. (L) (on)
            _DO(XPLMSetDatai,    1, "anim/4/button");                           // Engine elec. contr. (R) (on)
            _DO(XPLMSetDatai,    1, "anim/8/button");                           // Engine-dr. hy. pump (L) (on)
            _DO(XPLMSetDatai,    1, "anim/11/button");                          // Engine-dr. hy. pump (R) (on)
            _DO(XPLMSetDatai,    1, "anim/43/button");                          // Exter. lighting: posit. (on)
            _DO(XPLMSetDatai,    1, "anim/54/button");                          // Air: trim air valve sw. (on)
            _DO(XPLMSetDatai,    1, "anim/55/button");                          // Air: L recirc. fans sw. (on)
            _DO(XPLMSetDatai,    1, "anim/56/button");                          // Air: R recirc. fans sw. (on)
            _DO(XPLMSetDatai,    1, "anim/59/button");                          // Air: L bleed ISLN valve (on/open)
            _DO(XPLMSetDatai,    1, "anim/87/button");                          // Air: R bleed ISLN valve (on/open)
            _DO(XPLMSetDatai,    1, "anim/90/button");                          // Air: C bleed ISLN valve (on/open)
            _DO(XPLMSetDatai,    1, "anim/60/button");                          // Air: L e. bleed air sw. (on)
            _DO(XPLMSetDatai,    1, "anim/62/button");                          // Air: R e. bleed air sw. (on)
            _DO(XPLMSetDataf, 0.5f, "1-sim/cond/fwdTempControl");               // Temp. control (fwd ca.) (auto)
            _DO(XPLMSetDataf, 0.5f, "1-sim/cond/midTempControl");               // Temp. control (mid ca.) (auto)
            _DO(XPLMSetDataf, 0.5f, "1-sim/cond/aftTempControl");               // Temp. control (aft ca.) (auto)
            _DO(XPLMSetDataf, 0.5f, "1-sim/cond/fltdkTempControl");             // Temp. control (f. deck) (auto)
            break;

        case NVP_ACF_B777_FF:
//          _DO(XPLMSetDatai,    1, "anim/???/button"); // TODO: find dataref   // Terrain override switch (on)
//          _DO(XPLMSetDataf, 1.0f, "anim/33/cover");                           // Terrain override switch (lift cover)
//          _DO(XPLMSetDataf, 1.0f, "anim/14/cover");                           // Engine elec. contr. (L) (lift cover)
//          _DO(XPLMSetDataf, 1.0f, "anim/16/cover");                           // Engine elec. contr. (R) (lift cover)
            _DO(XPLMSetDatai,    1, "anim/108/button");                         // Engine-dr. hy. pump (L) (on)
            _DO(XPLMSetDatai,    1, "anim/111/button");                         // Engine-dr. hy. pump (R) (on)
            _DO(XPLMSetDatai,    1, "anim/116/button");                         // Engine elec. contr. (L) (norm)
            _DO(XPLMSetDatai,    1, "anim/117/button");                         // Engine elec. contr. (R) (norm)
            _DO(XPLMSetDatai,    1, "anim/154/button");                         // Engine autostart switch (on)
            _DO(XPLMSetDatai,    1, "anim/130/button");                         // Exter. lighting: navig. (on)
            _DO(XPLMSetDatai,    1, "anim/134/button");                         // Air: upper recirc. fans (on)
            _DO(XPLMSetDatai,    1, "anim/135/button");                         // Air: lower recirc. fans (on)
            _DO(XPLMSetDatai,    1, "anim/137/button");                         // Air: L t. air valve sw. (on)
            _DO(XPLMSetDatai,    1, "anim/138/button");                         // Air: R t. air valve sw. (on)
            _DO(XPLMSetDatai,    1, "anim/139/button");                         // Air: L bleed ISLN valve (auto)
            _DO(XPLMSetDatai,    1, "anim/140/button");                         // Air: C bleed ISLN valve (auto)
            _DO(XPLMSetDatai,    1, "anim/141/button");                         // Air: R bleed ISLN valve (auto)
            _DO(XPLMSetDatai,    1, "anim/143/button");                         // Air: APU  bleed air sw. (auto)
            _DO(XPLMSetDatai,    1, "anim/142/button");                         // Air: L e. bleed air sw. (on)
            _DO(XPLMSetDatai,    1, "anim/144/button");                         // Air: R e. bleed air sw. (on)
            _DO(XPLMSetDatai,    1, "anim/25/switch");                          // Cargo temp. cont. (aft) (low)
            _DO(XPLMSetDatai,    1, "anim/26/switch");                          // Cargo temp. cont. (blk) (low)
            _DO(XPLMSetDataf, 0.5f, "anim/5/rotery");                           // Temp. control (f. deck) (auto)
            _DO(XPLMSetDataf, 0.5f, "anim/6/rotery");                           // Temp. control (all ca.) (auto)
            break;

        case NVP_ACF_EMBE_SS:
            _DO(XPLMSetDatai, 1, "SSG/EJET/LIGHTS/nav_lights_sw");              // Exter. lighting: navig. (on)
            _DO(XPLMSetDatai, 1, "ssg/EJET/GND/rain_hide_sw");                  // Disable custom rain effects
            _DO(XPLMSetDatai, 0, "ssg/EJET/GND/stair1_ON");                     // Hide passenger stairs
            _DO(XPLMSetDatai, 0, "ssg/EJET/GND/seats_hide_sw");                 // Hide captain's seat
            _DO(XPLMSetDatai, 0, "ssg/EJET/GND/yokes_hide_sw");                 // Hide both yokes
            break;

        case NVP_ACF_SSJ1_RZ:
        {
            // check avionics state, enable and delay processing if required
            XPLMDataRef av_pwr_on = XPLMFindDataRef("sim/cockpit2/switches/avionics_power_on");
            if (NULL == av_pwr_on)
            {
                XPLMSpeakString("nav P first call failed");
                return (ctx->first_fcall = 0) - 1;
            }
            if (ctx->first_fcall > +0 && !XPLMGetDatai(av_pwr_on))
            {
                ctx->first_fcall = -2; // pass 1, enable pass 2
                XPLMSetDatai(av_pwr_on, 1); return 0;
            }

            // items that require avionics power to change state
            _DO(XPLMSetDataf, 0.0f, "but/temp/airL");
            _DO(XPLMSetDataf, 0.0f, "but/temp/airR");
            _DO(XPLMSetDataf, 0.0f, "but/temp/heater");
            _DO(XPLMSetDataf, 1.0f, "but/fuel/pumpLt");
            _DO(XPLMSetDataf, 1.0f, "but/fuel/pumpRt");
            _DO(XPLMSetDataf, 1.0f, "but/sim/cockpit2/fuel/fuel_tank_pump_on0");
            _DO(XPLMSetDataf, 1.0f, "but/sim/cockpit2/fuel/fuel_tank_pump_on1");
            _DO(XPLMSetDataf, 1.0f, "but/sim/cockpit2/fuel/fuel_tank_pump_on2");
            _DO(XPLMSetDataf, 1.0f, "but/sim/cockpit2/fuel/fuel_tank_pump_on3");
            _DO(XPLMSetDataf, 1.0f, "but/sim/cockpit2/fuel/fuel_tank_pump_on4");
            if (ctx->first_fcall == -2)
            {
                ctx->first_fcall += -1; return 0; // pass 2, enable pass 3
            }

            // everything else
            _DO(XPLMSetDataf, 1.0f, "door/lock");
            _DO(XPLMSetDataf, 1.0f, "elec/cap1");
            _DO(XPLMSetDataf, 1.0f, "elec/cap2");
            _DO(XPLMSetDataf, 1.0f, "elec/cap3");
            _DO(XPLMSetDataf, 1.0f, "elec/cap4");
            _DO(XPLMSetDataf, 1.0f, "elec/cap8");
            _DO(XPLMSetDataf, 0.0f, "prep/lock1");
            _DO(XPLMSetDataf, 0.0f, "prep/lock2");
            _DO(XPLMSetDataf, 0.0f, "prep/lock3");
            _DO(XPLMSetDataf, 0.0f, "prep/lock4");
            _DO(XPLMSetDataf, 0.0f, "prep/lock5");
            _DO(XPLMSetDataf, 0.0f, "prep/lock6");
            _DO(XPLMSetDataf, .75f, "lights/brt");
            _DO(XPLMSetDataf, 0.5f, "lights/flood");
            _DO(XPLMSetDataf, 0.5f, "but/lights/pilot");
            _DO(XPLMSetDatai,    1, "sim/cockpit2/switches/navigation_lights_on");
            if (ctx->first_fcall == -3)
            {
                XPLMSetDatai(av_pwr_on, 0); // pass 3, avionics back to off
            }
            if ((cr = XPLMFindCommand("shortcuts/hydraulic_system_auto_tgl")))
            {
                XPLMCommandOnce(cr);
            }
            break;
        }

        case NVP_ACF_GENERIC:
            // all Carenado addons handled here (dataref not found: no effect)
            _DO(XPLMSetDatai,     0, "thranda/views/InstRefl");                 // various aircraft
            _DO(XPLMSetDatai,     0, "thranda/views/WindowRefl");               // various aircraft
            _DO(XPLMSetDatai,     1, "thranda/cockpit/actuators/HideYokeL");    // various aircraft
            _DO(XPLMSetDatai,     1, "thranda/cockpit/actuators/HideYokeR");    // various aircraft
            _DO(XPLMSetDataf,  1.0f, "thranda/cockpit/actuators/VisorSwingL");  // TBM 850 & PC-12
            _DO(XPLMSetDataf,  1.0f, "thranda/cockpit/actuators/VisorSwingR");  // TBM 850 & PC-12
            _DO(XPLMSetDataf,  0.0f, "thranda/cockpit/actuators/VisorSlideL");  // TBM 850 & PC-12
            _DO(XPLMSetDataf,  0.0f, "thranda/cockpit/actuators/VisorSlideR");  // TBM 850 & PC-12
            _DO(XPLMSetDataf, -0.5f, "thranda/cockpit/actuators/VisorL");       // TBM 850 & PC-12
            _DO(XPLMSetDataf, -0.5f, "thranda/cockpit/actuators/VisorR");       // TBM 850 & PC-12
            _DO(XPLMSetDatai, 1, "sim/cockpit2/switches/navigation_lights_on"); // various aircraft
            break;

        default:
            break;
    }
    return (ctx->first_fcall = 0);
}
#undef _DO

static int aibus_350_init(refcon_ff_a350 *ffa)
{
    if (ffa && ffa->ready == 0)
    {
        if ((ffa->pkb_ref = XPLMFindDataRef("1-sim/parckBrake")))
        {
            (ffa->ready = 1); return 0;
        }
        return -1;
    }
    return 0;
}

static int aibus_fbw_init(refcon_qpacfbw *fbw)
{
    if (fbw && fbw->ready == 0)
    {
        if (fbw->pkb_tmp == NULL)
        {
            fbw->pkb_tmp = XPLMRegisterDataAccessor("navP/refcon_qpacfbw/pkb_tmp",
                                                    xplmType_Int, 1,
                                                    &priv_getdata_i,
                                                    &priv_setdata_i,
                                                    NULL, NULL, NULL, NULL, NULL,
                                                    NULL, NULL, NULL, NULL, NULL,
                                                    &fbw->pkb_var, &fbw->pkb_var);
            if (!fbw->pkb_tmp) return -1;
        }
        if ((fbw->h_b_max = XPLMFindCommand("sim/flight_controls/brakes_max"    )) &&
            (fbw->h_b_reg = XPLMFindCommand("sim/flight_controls/brakes_regular")))
        {
            if ((fbw->pkb_ref = XPLMFindDataRef("AirbusFBW/ParkBrake")))
            {
                (fbw->ready = 1); return 0;
            }
            if ((fbw->pkb_ref = XPLMFindDataRef("com/petersaircraft/airbus/ParkBrake")))
            {
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

#undef REGISTER_CHANDLER
#undef UNREGSTR_CHANDLER
#undef CALLOUT_PARKBRAKE
#undef CALLOUT_SPEEDBRAK
#undef CALLOUT_FLAPLEVER
