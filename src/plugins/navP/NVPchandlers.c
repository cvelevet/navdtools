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

    /* …and all datarefs */
    if (ctx->acfspec.qpac.pkb_tmp)
    {
        XPLMUnregisterDataAccessor(ctx->acfspec.qpac.pkb_tmp);
        ctx->acfspec.qpac.pkb_tmp = NULL;
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
    ctx->athr.disc.cc.name = NULL;
    ctx->athr.toga.cc.name = NULL;
    ctx->otto.disc.cc.name = NULL;

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
            if (!strcasecmp(xaircraft_icao_code, "B737"))
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
    /*
     * XXX: this function is basically guaranteed to be called early, so here we
     *      do any additional aircraft-specific stuff that can't be done earlier.
     */
    if (ctx->first_fcall)
    {
        first_fcall_do(ctx);
    }
//    if (ctx->kill_daniel)
//    {
//        // TODO: implement
//    }
    if (ctx->atyp == NVP_ACF_A350_FF)
    {
        if (ctx->acfspec.a350.ready == 0)
        {
            aibus_350_init(&ctx->acfspec.a350);
        }
        if (ctx->acfspec.a350.ready && inPhase == xplm_CommandEnd)
        {
            XPLMSetDatai(ctx->acfspec.a350.pkb_ref, 0); // inverted
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
        }
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        XPLMSetDataf(rcb->p_b_rat, 1.0f);
    }
    return 0;
}

static int chandler_p_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    chandler_context *ctx = inRefcon;
    refcon_braking   *rcb = &ctx->bking.rc_brk;
    if (ctx->atyp == NVP_ACF_A350_FF)
    {
        if (ctx->acfspec.a350.ready == 0)
        {
            aibus_350_init(&ctx->acfspec.a350);
        }
        if (ctx->acfspec.a350.ready && inPhase == xplm_CommandEnd)
        {
            XPLMSetDatai(ctx->acfspec.a350.pkb_ref, 1); // inverted
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
        }
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        XPLMSetDataf(rcb->p_b_rat, 0.0f);
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
        chandler_context *ctx = inRefcon;
        refcon_ixeg733   *i33 = NULL;
        refcon_eadt738   *x38 = NULL;
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

            default:
                XPLMCommandOnce (ctx->spbrk.sext);
                if (XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    XPLMSpeakString("spoilers disarmed");
                    return 0;
                }
                if (XPLMGetDataf(ctx->spbrk.srat) > +.01f)
                {
                    XPLMSpeakString("speedbrake");
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
                    XPLMSpeakString("spoilers disarmed");
                    XPLMSetDataf(i33->slat, 0.0f);    // armed: retract fully
                    return 0;
                }
                XPLMSpeakString("speedbrake");
                XPLMSetDataf(i33->slat, 0.8f);        // extend: flight detent
                return 0;
            }
            if (x38 && x38->ready)
            {
                if (ratio > .1f && ratio < .2f)
                {
                    XPLMSpeakString("spoilers disarmed");
                    XPLMCommandOnce(x38->spret);      // extend: disarm spoilers
                    return 0;
                }
                XPLMSpeakString("speedbrake");
                XPLMCommandOnce(ctx->spbrk.sext);     // extend: one
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
        chandler_context *ctx = inRefcon;
        refcon_ixeg733   *i33 = NULL;
        refcon_eadt738   *x38 = NULL;
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

            default:
                XPLMCommandOnce (ctx->spbrk.sret);
                if (XPLMGetDataf(ctx->spbrk.srat) < -.01f)
                {
                    XPLMSpeakString("spoilers armed");
                    return 0;
                }
                if (XPLMGetDataf(ctx->spbrk.srat) < +.01f)
                {
                    XPLMSpeakString("speedbrake retracted");
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
                    XPLMSpeakString("spoilers armed");
                    return 0;
                }
                if (ratio > +.51f)
                {
                    XPLMSetDataf(i33->slat, 0.8f);    // retract: flight detent
                    return 0;
                }
                XPLMSetDataf(i33->slat, 0.0f);        // retract: fully
                XPLMSpeakString("speedbrake retracted");
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
                    XPLMSpeakString("spoilers armed");
                    return 0;
                }
                XPLMCommandOnce (ctx->spbrk.sret);    // retract: one
                if (XPLMGetDataf(ctx->spbrk.srat) < .01f)
                {
                    XPLMSpeakString("speedbrake retracted");
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

static int first_fcall_do(chandler_context *ctx)
{
    XPLMDataRef d_ref;
    switch (ctx->atyp)
    {
        case NVP_ACF_A320_QP:
            if ((d_ref = XPLMFindDataRef("AirbusFBW/DUBrightness")))
            {
                float DUBrightness[8];
                int   valuesCopied = XPLMGetDatavf(d_ref, DUBrightness, 0, 8);
                for  (int i = 0; i < valuesCopied; i++)
                {
                    DUBrightness[i] = 0.9f;
                }
                XPLMSetDatavf(d_ref, DUBrightness, 0, valuesCopied);
            }
            break;

        case NVP_ACF_A350_FF:
            if ((d_ref = XPLMFindDataRef("1-sim/radio/button/1/16")))
            {
                XPLMSetDatai(d_ref, 1);      // RMP1 - on VHF1 radio, button: on
            }
            if ((d_ref = XPLMFindDataRef("1-sim/radio/push/1/1")))
            {
                XPLMSetDatai(d_ref, 1);      // RMP1 - on VHF1 radio, switch: on
            }
            if ((d_ref = XPLMFindDataRef("1-sim/radio/1/1/rotary")))
            {
                XPLMSetDataf(d_ref, 270.0f); // RMP1 - on VHF1 radio volume: max
            }
            if ((d_ref = XPLMFindDataRef("1-sim/radio/push/1/10")))
            {
                XPLMSetDatai(d_ref, 1);      // cabin crew intercomm, switch: on
            }
            if ((d_ref = XPLMFindDataRef("1-sim/radio/1/10/rotary")))
            {
                XPLMSetDataf(d_ref, 270.0f); // cabin crew intercomm volume: max
            }
            if ((d_ref = XPLMFindDataRef("1-sim/radio/push/1/11")))
            {
                XPLMSetDatai(d_ref, 1);      // pass. announcements', switch: on
            }
            if ((d_ref = XPLMFindDataRef("1-sim/radio/1/11/rotary")))
            {
                XPLMSetDataf(d_ref, 270.0f); // pass. announcements' volume: max
            }
            break;

        case NVP_ACF_B763_FF:
            if ((d_ref = XPLMFindDataRef("anim/75/button")))
            {
                XPLMSetDatai(d_ref, 1);    // terrain override switch/button: on
            }
            break;

        default:
            break;
    }
    XPLMSpeakString("nav P first call");
    return (ctx->first_fcall = 0);
}

static int aibus_350_init(refcon_ff_a350 *ffa)
{
    if (ffa && ffa->ready == 0)
    {
        if ((ffa->pkb_ref = XPLMFindDataRef("1-sim/parckBrake")))
        {
            (ffa->ready = 1);
        }
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
        }
        fbw->pkb_ref = XPLMFindDataRef("AirbusFBW/ParkBrake");
        fbw->h_b_max = XPLMFindCommand("sim/flight_controls/brakes_max");
        fbw->h_b_reg = XPLMFindCommand("sim/flight_controls/brakes_regular");
        if (fbw->pkb_tmp && fbw->pkb_ref && fbw->h_b_max && fbw->h_b_reg)
        {
            fbw->ready = 1;
        }
    }
    return 0;
}

static int boing_733_init(refcon_ixeg733 *i33)
{
    if (i33 && i33->ready == 0)
    {
        if ((i33->slat = XPLMFindDataRef("ixeg/733/hydraulics/speedbrake_act")))
        {
            (i33->ready = 1);
        }
    }
    return 0;
}

static int boing_738_init(refcon_eadt738 *x38)
{
    if (x38 && x38->ready == 0)
    {
        x38->sparm = XPLMFindCommand("x737/speedbrakes/SPEEDBRAKES_ARM");
        x38->spret = XPLMFindCommand("x737/speedbrakes/SPEEDBRAKES_DOWN");
        x38->pt_up = XPLMFindCommand("x737/trim/CAPT_STAB_TRIM_UP_ALL");
        x38->pt_dn = XPLMFindCommand("x737/trim/CAPT_STAB_TRIM_DOWN_ALL");
        if (x38->sparm && x38->spret && x38->pt_up && x38->pt_dn)
        {
            x38->ready = 1;
        }
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
