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
#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"
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
    int initialized;
    int first_fcall;
    int kill_daniel;

    enum
    {
        NVP_ACF_GENERIC = 0x0000000,
        NVP_ACF_A320_JD = 0x0000010,
        NVP_ACF_A332_JD = 0x0000020,
        NVP_ACF_A320_QP = 0x0000100,
        NVP_ACF_A333_RW = 0x0000200,
        NVP_ACF_A350_FF = 0x0001000,
        NVP_ACF_B733_XG = 0x0004000,
        NVP_ACF_B738_EA = 0x0008000,
        NVP_ACF_B752_FF = 0x0010000,
        NVP_ACF_B763_FF = 0x0100000,
        NVP_ACF_B77L_FF = 0x1000000,
    } atyp;
#define NVP_ACF_MASK_JDN  0x00000F0 // all J.A.R.Design addons
#define NVP_ACF_MASK_QPC  0x0000F00 // all QPAC-powered addons
#define NVP_ACF_MASK_FFR  0xFFFF000 // all FlightFactor addons
#define NVP_ACF_MASK_32x  0x0000110 // all A320 series aircraft
#define NVP_ACF_MASK_33x  0x0000220 // all A330 series aircraft
#define NVP_ACF_MASK_35x  0x0001000 // all A350 series aircraft
#define NVP_ACF_MASK_73x  0x000C000 // all B737 series aircraft
#define NVP_ACF_MASK_75x  0x00F0000 // all B757 series aircraft
#define NVP_ACF_MASK_76x  0x0F00000 // all B767 series aircraft
#define NVP_ACF_MASK_77x  0xF000000 // all B777 series aircraft
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
} chandler_context;

/* Callout default values */
#define CALLOUT_PARKBRAKE 1
#define CALLOUT_SPEEDBRAK 1

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
    if (!ctx->callouts.ref_park_brake || !ctx->callouts.ref_speedbrake)
    {
        goto fail;
    }

    /* Custom commands: braking */
    ctx->bking.prk.cb.command = XPLMCreateCommand("navP/brakes/parking", "apply max. park brake");
    ctx->bking.off.cb.command = XPLMCreateCommand("navP/brakes/release", "release parking brake");
    ctx->bking.max.cb.command = XPLMCreateCommand("navP/brakes/maximum", "maximum braking action");
    ctx->bking.reg.cb.command = XPLMCreateCommand("navP/brakes/regular", "regular braking action");
    ctx->bking.rc_brk.p_b_rat = XPLMFindDataRef  ("sim/cockpit2/controls/parking_brake_ratio");
    ctx->bking.rc_brk.l_b_rat = XPLMFindDataRef  ("sim/cockpit2/controls/left_brake_ratio");
    ctx->bking.rc_brk.r_b_rat = XPLMFindDataRef  ("sim/cockpit2/controls/right_brake_ratio");
    if (!ctx->bking.prk.cb.command || !ctx->bking.off.cb.command                      ||
        !ctx->bking.max.cb.command || !ctx->bking.reg.cb.command                      ||
        !ctx->bking.rc_brk.p_b_rat || !XPLMCanWriteDataRef(ctx->bking.rc_brk.l_b_rat) ||
        !ctx->bking.rc_brk.l_b_rat || !XPLMCanWriteDataRef(ctx->bking.rc_brk.l_b_rat) ||
        !ctx->bking.rc_brk.r_b_rat || !XPLMCanWriteDataRef(ctx->bking.rc_brk.r_b_rat))
    {
        goto fail;
    }
    else
    {
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
    if (ref_ql_idx_get())
    {
        XPLMUnregisterDataAccessor(ref_ql_idx_get());
        ref_ql_idx_set            (NULL);
    }

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
//  char acf_file[257], acf_path[513];
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
//  XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, acf_file, acf_path);

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
                ctx->atyp = NVP_ACF_A332_JD;
                break;
            }
            ndt_log("navP [warning]: no aircraft type match despite plugin (jardesign.sound3d)\n");
            break; // fall back to generic
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("QPAC.airbus.fbw"))
        {
            if (!strlen(xaircraft_icao_code))
            {
                if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("FFSTSmousehandler"))
                {
                    ndt_log("navP [info]: plane is FlightFactor-QPAC Airbus A350 XWB Advanced\n");
                    ctx->atyp = NVP_ACF_A350_FF;
                    break;
                }
                ndt_log("navP [info]: plane is RWDesigns-QPAC Airbus A330-300\n");
                ctx->atyp = NVP_ACF_A333_RW;
                break;
            }
            if (!strcasecmp(xaircraft_icao_code, "A320"))
            {
                ndt_log("navP [info]: plane is QPAC Airbus A320-200 IAE\n");
                ctx->atyp = NVP_ACF_A320_QP;
                break;
            }
            ndt_log("navP [warning]: no aircraft type match despite plugin (QPAC.airbus.fbw)\n");
            break; // fall back to generic
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
                    ctx->atyp = NVP_ACF_B733_XG;
                    break;
                }
            }
            // fall through (no generic fallback, Gizmo running for all planes)
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("bs.x737.plugin"))
        {
            if (!strcasecmp(xaircraft_icao_code, "B737") ||
                !strcasecmp(xaircraft_icao_code, "B738"))
            {
                ndt_log("navP [info]: plane is EADT Boeing x737-800\n");
                ctx->atyp = NVP_ACF_B738_EA;
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
                ctx->atyp = NVP_ACF_B763_FF;
                break;
            }
            // TODO: future FlightFactor 757 and 767 addons
            ndt_log("navP [warning]: no aircraft type match despite plugin (757767avionics)\n");
            break; // fall back to generic
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("de.philippmuenzel.t7avionics"))
        {
            if (!strcasecmp(xaircraft_icao_code, "B77L"))
            {
                ndt_log("navP [info]: plane is FlightFactor Boeing 777 Worldliner Professional\n");
                ctx->atyp = NVP_ACF_B77L_FF;
                break;
            }
            // TODO: other FlightFactor T7 addons
            ndt_log("navP [warning]: no aircraft type match despite plugin (t7avionics)\n");
            break; // fall back to generic
        }
    }
    while (0);

    /* all good */
    switch (ctx->atyp)
    {
        case NVP_ACF_A320_JD:
            ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
            ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
            break;

        case NVP_ACF_A320_QP:
            ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
            ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
            break;

        case NVP_ACF_A332_JD:
            ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
            ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
            break;

        case NVP_ACF_A333_RW:
            ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
            ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
            break;

        case NVP_ACF_A350_FF:
            ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
            ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
            break;

        case NVP_ACF_B733_XG:
            ctx->otto.disc.cc.name = "ixeg/733/autopilot/AP_disengage";
            ctx->athr.disc.cc.name = "ixeg/733/autopilot/at_disengage";
            ctx->athr.toga.cc.name = "sim/engines/TOGA_power";
            break;

        case NVP_ACF_B738_EA:
            ctx->otto.disc.cc.name = "x737/yoke/capt_AP_DISENG_BTN";
            ctx->athr.disc.cc.name = "x737/mcp/ATHR_ARM_TOGGLE";
            ctx->athr.toga.cc.name = "x737/mcp/TOGA_TOGGLE";
            break;

        case NVP_ACF_B763_FF:
            ctx->otto.disc.cc.name = "1-sim/comm/AP/ap_disc";
            ctx->athr.disc.cc.name = "1-sim/comm/AP/at_disc";
            ctx->athr.toga.cc.name = "1-sim/comm/AP/at_toga";
            break;

        case NVP_ACF_B77L_FF:
            ctx->otto.disc.cc.name = "777/ap_disc";
            ctx->athr.disc.cc.name = "777/at_disc";
            ctx->athr.toga.cc.name = "777/at_toga";
            break;

        case NVP_ACF_GENERIC:
        default:
            ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
            ctx->athr.disc.cc.name = "sim/autopilot/autothrottle_off";
            ndt_log("navP [info]: plane is generic (ICAO: \"%s\")\n", xaircraft_icao_code);
            break;
    }
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
 * action: set or unset parking brake.
 *
 * rationale: X-Plane only has a toggle for this :(
 */
static int chandler_p_max(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    refcon_braking   *rcb = &ctx->bking.rc_brk;
    int speak = XPLMGetDatai(ctx->callouts.ref_park_brake);
    /*
     * XXX: this function is basically guaranteed to be called early, so here we
     *      do any additional aircraft-specific stuff that can't be done earlier.
     */
    if (ctx->first_fcall && inPhase == xplm_CommandEnd)
    {
        first_fcall_do(ctx);
        speak = 0;
    }
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
            XPLMSetDataf(rcb->l_b_rat, 0.5f);
            XPLMSetDataf(rcb->r_b_rat, 0.5f);
            break;
        case xplm_CommandContinue:
            XPLMSetDataf(rcb->l_b_rat, 0.5f);
            XPLMSetDataf(rcb->r_b_rat, 0.5f);
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
            case NVP_ACF_B733_XG:
                i33 = &ctx->acfspec.i733;
                if (i33->ready == 0)
                {
                    boing_733_init(i33);
                }
                break;

            case NVP_ACF_B738_EA:
                x38 = &ctx->acfspec.x738;
                if (x38->ready == 0)
                {
                    boing_738_init(x38);
                }
                break;

            case NVP_ACF_A320_JD:
            case NVP_ACF_A332_JD:
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
            case NVP_ACF_B733_XG:
                i33 = &ctx->acfspec.i733;
                if (i33->ready == 0)
                {
                    boing_733_init(i33);
                }
                break;

            case NVP_ACF_B738_EA:
                x38 = &ctx->acfspec.x738;
                if (x38->ready == 0)
                {
                    boing_738_init(x38);
                }
                break;

            case NVP_ACF_A320_JD:
            case NVP_ACF_A332_JD:
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
        case NVP_ACF_B738_EA:
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
        case NVP_ACF_B738_EA:
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

#define _DO(_func, _val, _name) { if ((d_ref = XPLMFindDataRef(_name))) _func(d_ref, _val); }
static int first_fcall_do(chandler_context *ctx)
{
    XPLMDataRef d_ref;
    switch (ctx->atyp)
    {
        case NVP_ACF_A320_QP:
            if ((d_ref = XPLMFindDataRef("AirbusFBW/DUBrightness")))
            {
                float DUBrightness[8] =
                {
                    [0] = 0.9f, [1] = 0.9f, [2] = 0.9f, [3] = 0.9f,
                    [4] = 0.9f, [5] = 0.9f, [6] = 0.9f, [7] = 0.9f,
                };
                XPLMSetDatavf(d_ref, &DUBrightness[0], 0, 8);
            }
            if ((d_ref = XPLMFindDataRef("AirbusFBW/OHPLightSwitches")))
            {
                int OHPLightSwitches[16] =
                {
                    [2] = 1, // nav&logo: system 1
                    [7] = 1, // strobes: automatic
                };
                XPLMSetDatavi(d_ref, &OHPLightSwitches[2], 2, 1);
                XPLMSetDatavi(d_ref, &OHPLightSwitches[7], 7, 1);
            }
            break;

        case NVP_ACF_A350_FF:
            _DO(XPLMSetDatai,      1, "1-sim/radio/button/1/16");               // RMP1: microph. on VHF1 (on)
            _DO(XPLMSetDatai,      1, "1-sim/radio/push/1/1");                  // RMP1: receiver on VHF1 (on)
            _DO(XPLMSetDataf, 270.0f, "1-sim/radio/1/1/rotary");                // RMP1: receiver on VHF1 (volume)
            _DO(XPLMSetDatai,      1, "1-sim/radio/push/1/10");                 // RMP1: c. crew intercom (on)
            _DO(XPLMSetDataf, 270.0f, "1-sim/radio/1/10/rotary");               // RMP1: c. crew intercom (volume)
            _DO(XPLMSetDatai,      1, "1-sim/radio/push/1/11");                 // RMP1: p. announcements (on)
            _DO(XPLMSetDataf, 270.0f, "1-sim/radio/1/11/rotary");               // RMP1: p. announcements (volume)
            _DO(XPLMSetDatai,      1, "1-sim/1/switch");                        // Evac. panel: selector  (capt&purs)
            _DO(XPLMSetDatai,      1, "1-sim/2/switch");                        // Ext. lighting: strobe  (auto)
            _DO(XPLMSetDatai,      1, "1-sim/5/switch");                        // Ext. lighting: navig.  (on)
            _DO(XPLMSetDatai,      1, "1-sim/20/switch");                       // Ext. lighting: logo    (auto)
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

        case NVP_ACF_B733_XG:
            _DO(XPLMSetDataf, 1.0f, "ixeg/733/bleedair/bleedair_recirc_fan_act");   // Bleed air recirc. fans (auto)
            _DO(XPLMSetDataf, 0.0f, "ixeg/733/aircond/aircond_cont_cabin_sel_act"); // Cont. cab. air temper. (normal)
            _DO(XPLMSetDataf, 0.0f, "ixeg/733/aircond/aircond_pass_cabin_sel_act"); // Pass. cab. air temper. (normal)
            _DO(XPLMSetDataf, 1.0f, "ixeg/733/lighting/position_lt_act");           // Exte. lighting: posit. (on)
            break;

        case NVP_ACF_B763_FF:
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

        case NVP_ACF_B77L_FF:
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

        default:
            break;
    }
    if ((ctx->atyp & NVP_ACF_MASK_JDN) == 0)
    {
        XPLMSpeakString("nav P first call");
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
        if ((fbw->pkb_ref = XPLMFindDataRef("AirbusFBW/ParkBrake"               )) &&
            (fbw->h_b_max = XPLMFindCommand("sim/flight_controls/brakes_max"    )) &&
            (fbw->h_b_reg = XPLMFindCommand("sim/flight_controls/brakes_regular")))
        {
            (fbw->ready = 1); return 0;
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
