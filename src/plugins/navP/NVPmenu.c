/*
 * NVPmenu.c
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

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Widgets/XPStandardWidgets.h"
#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMDisplay.h"
#include "XPLM/XPLMMenus.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMProcessing.h"
#include "XPLM/XPLMUtilities.h"

#include "common/common.h"

#include "NVPmenu.h"
#include "YFSmain.h"

#define REFUEL_DIALG_CAPW 120
#define REFUEL_DIALG_CAPH  24
#define REFUEL_DIALG_CLBW 140
#define REFUEL_DIALG_CLBH  24
#define REFUEL_DIALG_DEFW 200
#define REFUEL_DIALG_DEFH 200
#define REFUEL_DIALG_TXTW  50
#define REFUEL_DIALG_TXTH  24
#define REFUEL_FUEL_KHSEC 34.0f // 34kg/0.5s: 40,800kg of fuel in 10m (600s)
#define REFUEL_PLOD_KHSEC 26.0f // 26kg/0.5s: 300 pax (x104kg) in 10m (600s)

typedef struct
{
    int id;
    enum
    {
        MENUITEM_NOTHING_TODO,
        MENUITEM_VOLUME_SM_IC,
        MENUITEM_VOLUME_PRST0,
        MENUITEM_VOLUME_PRST1,
        MENUITEM_VOLUME_PRST2,
        MENUITEM_VOLUME_PRST3,
        MENUITEM_VOLUME_PRST4,
        MENUITEM_FILDOV_SM_IC,
        MENUITEM_FILDOV_45DEG,
        MENUITEM_FILDOV_50DEG,
        MENUITEM_FILDOV_55DEG,
        MENUITEM_FILDOV_60DEG,
        MENUITEM_FILDOV_65DEG,
        MENUITEM_FILDOV_70DEG,
        MENUITEM_FILDOV_75DEG,
        MENUITEM_XFMC01_SM_IC,
        MENUITEM_XFMC01_NABLE,
        MENUITEM_XFMC01_DSBLE,
        MENUITEM_CALLOUTS_STS,
        MENUITEM_SPEEDBOOSTER,
        MENUITEM_CLOUD_KILLER,
        MENUITEM_REFUEL_DIALG,
        MENUITEM_SPEAKWEATHER,
        MENUITEM_YFMS_MAINWIN,
    } mivalue;
} item_context;

typedef struct
{
    XPLMMenuID id;
    int setupdone;

    struct
    {
        struct
        {
            XPLMMenuID      sm_id;
            item_context    sm_ic;
            item_context    prst0;
            item_context    prst1;
            item_context    prst2;
            item_context    prst3;
            item_context    prst4;
        } volume;
        struct
        {
            XPLMMenuID      sm_id;
            item_context    sm_ic;
            item_context    deg45;
            item_context    deg50;
            item_context    deg55;
            item_context    deg60;
            item_context    deg65;
            item_context    deg70;
            item_context    deg75;
        } fildov;
        struct
        {
            XPLMMenuID      sm_id;
            item_context    sm_ic;
            item_context    nable;
            item_context    dsble;
        } xfmc01;
        item_context callouts_sts;
        item_context speedbooster;
        item_context cloud_killer;
        item_context dummy_item_1;
        item_context refuel_dialg;
        item_context speakweather;
        item_context yfms_mainwin;
    } items;

    struct
    {
        struct
        {
            XPLMDataRef park_brake;
            XPLMDataRef speedbrake;
            XPLMDataRef flap_lever;
        } callouts_sts;

        struct
        {
            int      adjust_fuel;
            int      adjust_plod;
            int   num_fuel_tanks;
            float min_fuel_total;
            float max_fuel_total;
            float fuel_tank_r[9];
            float fuel_target_kg;
            float plod_target_kg;
            char  fueltot_str[9];
            char  payload_str[9];
            XPLMDataRef pay_load;
            XPLMDataRef fuel_max;
            XPLMDataRef fuel_rat;
            XPLMDataRef fuel_qty;
            XPLMDataRef fuel_tot;
            XPWidgetID dialog_id;
            XPWidgetID f_txtf_id;
            XPWidgetID p_txtf_id;
            XPWidgetID button_id;
            XPLMFlightLoop_f rfc;
        } refuel_dialg;

        struct
        {
            XPLMDataRef baro_sl;
            XPLMDataRef wind_dt;
            XPLMDataRef wind_sd;
            XPLMDataRef temp_dc;
            XPLMDataRef temp_dp;
        } speakweather;

        struct
        {
            // toggle drawing clouds altogether
            // note: sim/operation/override/override_clouds disables real weather, kudos to Laminar…
            XPLMDataRef dr_kill_2d; // sim/private/controls/clouds/kill_2d      (toggle: 0/1 (default 0))
            XPLMDataRef dr_kill_3d; // sim/private/controls/clouds/kill_3d      (toggle: 0/1 (default 0))
            XPLMDataRef dr_skpdraw; // sim/private/controls/clouds/skip_draw    (toggle: 0/1 (default 0))

            // cloud resolution & drawing parameters
            XPLMDataRef dr_frstr3d; // sim/private/controls/clouds/first_res_3d
            float       df_frstr3d;
            XPLMDataRef dr_lastr3d; // sim/private/controls/clouds/last_res_3d
            float       df_lastr3d;
            XPLMDataRef dr_plotrad; // sim/private/controls/clouds/plot_radius
            float       df_plotrad;
            XPLMDataRef dr_shadrad; // sim/private/controls/clouds/shad_radius
            float       df_shadrad;
            XPLMDataRef dr_limitfr; // sim/private/controls/clouds/limit_far
            float       df_limitfr;
            XPLMDataRef dr_difgain; // sim/private/controls/clouds/diffuse_gain
            float       df_difgain;
            XPLMDataRef dr_ovrdctl; // sim/private/controls/clouds/overdraw_control
            float       df_ovrdctl;

            // advanced features
            XPLMDataRef dr_use_csm; // sim/private/controls/caps/use_csm
            float       df_use_csm;
            XPLMDataRef dr_disprep; // sim/private/controls/perf/disable_shadow_prep
            float       df_disprep;
            XPLMDataRef dr_disrcam; // sim/private/controls/perf/disable_reflection_cam
            float       df_disrcam;
        } speedbooster;

        struct
        {
            XPLMDataRef dr_vol_eng;
            XPLMDataRef dr_vol_prs;
            XPLMDataRef dr_vol_grt;
            XPLMDataRef dr_vol_wer;
            XPLMDataRef dr_vol_was;
            XPLMDataRef dr_vol_coo;
            XPLMDataRef dr_vol_avs;
        } volume_prsts;

        struct
        {
            XPLMDataRef fildov_deg;
        } fildov_prsts;

        struct
        {
            XPLMPluginID plugin_id;
        } xfmc;

        struct
        {
            void *context;
        } yfms;
    } data;
} menu_context;

static void  mainw_center(int out[4],                  int in_w, int in_h);
static void  menu_rm_item(menu_context *ctx,                    int index);
static void  menu_handler(void *inMenuRef,                void *inItemRef);
static char* string4speak(char *string,       size_t alloc, int text_only);
static int   widget_hdlr1(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
static float refuel_hdlr1(                       float, float, int, void*);

static int create_menu(const char *name,
                       void       *data,
                       XPLMMenuID *_ptr,
                       XPLMMenuHandler_f func,
                       XPLMMenuID parent_menu,
                       int        parent_item)
{
    if (!name || !_ptr || !func)
    {
        return -1;
    }
    XPLMMenuID  id = XPLMCreateMenu(name, parent_menu, parent_item, func, data);
    if (id)
    {
        *_ptr = id; return 0;
    }
    return -1;
}

static int append_menu_item(const char *name, item_context *ctx, int mivalue, XPLMMenuID parent)
{
    if (!name || !ctx)
    {
        return -1;
    }
    ctx->mivalue = mivalue;
    return ((ctx->id = XPLMAppendMenuItem(parent, name, ctx, 0)) < 0);
}

static int get_dataref(XPLMDataRef *_ptr, const char *name)
{
    if (!_ptr || !name)
    {
        return -1;
    }
    XPLMDataRef dref = XPLMFindDataRef(name);
    if (dref)
    {
        *_ptr = dref; return 0;
    }
    return -1;
}

void* nvp_menu_init(void)
{
    menu_context *ctx = calloc(1, sizeof(menu_context));
    if (!ctx)
    {
        return NULL;
    }

    /* create a submenu in the plugins menu */
    if (create_menu("navP", ctx, &ctx->id, &menu_handler, NULL, 0))
    {
        goto fail;
    }

    /* volume sub-menu & its items */
    if (append_menu_item("Volume", &ctx->items.volume.sm_ic,
                         MENUITEM_VOLUME_SM_IC, ctx->id))
    {
        goto fail;
    }
    else
    {
        XPLMAppendMenuSeparator(ctx->id);
    }
    if (create_menu("Volume",      ctx,    &ctx->items.volume.sm_id,
                    &menu_handler, ctx->id, ctx->items.volume.sm_ic.id))
    {
        goto fail;
    }
    struct
    {
        const char *name;
        int item_mivalue;
        item_context *cx;
    }
    volume_items[] =
    {
        { "  0 %", MENUITEM_VOLUME_PRST0, &ctx->items.volume.prst0, },
        { " 25 %", MENUITEM_VOLUME_PRST1, &ctx->items.volume.prst1, },
        { " 50 %", MENUITEM_VOLUME_PRST2, &ctx->items.volume.prst2, },
        { " 75 %", MENUITEM_VOLUME_PRST3, &ctx->items.volume.prst3, },
        { "100 %", MENUITEM_VOLUME_PRST4, &ctx->items.volume.prst4, },
        {    NULL,                     0,                     NULL, },
    };
    for (int i = 0; volume_items[i].name; i++)
    {
        if (append_menu_item(volume_items[i].name,
                             volume_items[i].cx,
                             volume_items[i].item_mivalue,
                             ctx->items.volume.sm_id))
        {
            goto fail;
        }
    }
    if (get_dataref(&ctx->data.volume_prsts.dr_vol_eng, "sim/operation/sound/engine_volume_ratio" ) ||
        get_dataref(&ctx->data.volume_prsts.dr_vol_prs, "sim/operation/sound/prop_volume_ratio"   ) ||
        get_dataref(&ctx->data.volume_prsts.dr_vol_grt, "sim/operation/sound/ground_volume_ratio" ) ||
        get_dataref(&ctx->data.volume_prsts.dr_vol_wer, "sim/operation/sound/weather_volume_ratio") ||
        get_dataref(&ctx->data.volume_prsts.dr_vol_was, "sim/operation/sound/warning_volume_ratio") ||
        get_dataref(&ctx->data.volume_prsts.dr_vol_coo, "sim/operation/sound/radio_volume_ratio"  ) ||
        get_dataref(&ctx->data.volume_prsts.dr_vol_avs, "sim/operation/sound/fan_volume_ratio"    ))
    {
        goto fail;
    }

    /* field of view sub-menu & its items */
    if (append_menu_item("Field of view", &ctx->items.fildov.sm_ic,
                         MENUITEM_FILDOV_SM_IC, ctx->id))
    {
        goto fail;
    }
    if (create_menu("Field of view",  ctx, &ctx->items.fildov.sm_id,
                    &menu_handler, ctx->id, ctx->items.fildov.sm_ic.id))
    {
        goto fail;
    }
    else
    {
        XPLMAppendMenuSeparator(ctx->id);
    }
    struct
    {
        const char *name;
        int item_mivalue;
        item_context *cx;
    }
    fildov_items[] =
    {
        { "45 degr.", MENUITEM_FILDOV_45DEG, &ctx->items.fildov.deg45, },
        { "50 degr.", MENUITEM_FILDOV_50DEG, &ctx->items.fildov.deg50, },
        { "55 degr.", MENUITEM_FILDOV_55DEG, &ctx->items.fildov.deg55, },
        { "60 degr.", MENUITEM_FILDOV_60DEG, &ctx->items.fildov.deg60, },
        { "65 degr.", MENUITEM_FILDOV_65DEG, &ctx->items.fildov.deg65, },
        { "70 degr.", MENUITEM_FILDOV_70DEG, &ctx->items.fildov.deg70, },
        { "75 degr.", MENUITEM_FILDOV_75DEG, &ctx->items.fildov.deg75, },
        {       NULL,                     0,                     NULL, },
    };
    for (int i = 0; fildov_items[i].name; i++)
    {
        if (append_menu_item(fildov_items[i].name,
                             fildov_items[i].cx,
                             fildov_items[i].item_mivalue,
                             ctx->items.fildov.sm_id))
        {
            goto fail;
        }
    }
    if (get_dataref(&ctx->data.fildov_prsts.fildov_deg, "sim/graphics/view/field_of_view_deg"))
    {
        goto fail;
    }

    /* X-FMC sub-menu & its items */
    ctx->data.xfmc.plugin_id = XPLMFindPluginBySignature("x-fmc.com");
    if (XPLM_NO_PLUGIN_ID != ctx->data.xfmc.plugin_id)
    {
        if (append_menu_item("X-FMC", &ctx->items.xfmc01.sm_ic,
                             MENUITEM_XFMC01_SM_IC, ctx->id))
        {
            goto fail;
        }
        if (create_menu("X-FMC", ctx, &ctx->items.xfmc01.sm_id,
                        &menu_handler, ctx->id, ctx->items.xfmc01.sm_ic.id))
        {
            goto fail;
        }
        else
        {
            XPLMAppendMenuSeparator(ctx->id);
        }
        struct
        {
            const char *name;
            int item_mivalue;
            item_context *cx;
        }
        xfmc01_items[] =
        {
            {  "Enable", MENUITEM_XFMC01_NABLE, &ctx->items.xfmc01.nable, },
            { "Disable", MENUITEM_XFMC01_DSBLE, &ctx->items.xfmc01.dsble, },
            {      NULL,                     0,                     NULL, },
        };
        for (int i = 0; xfmc01_items[i].name; i++)
        {
            if (append_menu_item(xfmc01_items[i].name,
                                 xfmc01_items[i].cx,
                                 xfmc01_items[i].item_mivalue,
                                 ctx->items.xfmc01.sm_id))
            {
                goto fail;
            }
        }
    }

    /* toggle: callouts on/off */
    if (append_menu_item("navP custom callouts", &ctx->items.callouts_sts,
                         MENUITEM_CALLOUTS_STS,   ctx->id))
    {
        goto fail;
    }
    else
    {
        XPLMAppendMenuSeparator(ctx->id);
    }
    if (get_dataref(&ctx->data.callouts_sts.park_brake, "navP/callouts/park_brake") ||
        get_dataref(&ctx->data.callouts_sts.speedbrake, "navP/callouts/speedbrake") ||
        get_dataref(&ctx->data.callouts_sts.flap_lever, "navP/callouts/flap_lever"))
    {
        // Note: XPLMSetDatai doesn't work from XPluginEnable() either, so the
        // default dataref/checkbox values can't be set here (no else clause).
        goto fail;
    }

    /* toggle: speed boost on/off */
    if (append_menu_item("Tachyon Enhancement", &ctx->items.speedbooster,
                         MENUITEM_SPEEDBOOSTER,  ctx->id))
    {
        goto fail;
    }
    if (append_menu_item("Inoperative clouds", &ctx->items.cloud_killer,
                         MENUITEM_CLOUD_KILLER, ctx->id))
    {
        goto fail;
    }
    if (append_menu_item("", &ctx->items.dummy_item_1,
                         MENUITEM_NOTHING_TODO, ctx->id))
    {
        goto fail;
    }
    else
    {
        XPLMAppendMenuSeparator(ctx->id);
    }

    /* show payload & fuel dialog */
    if (append_menu_item("Fuel & Payload", &ctx->items.refuel_dialg,
                         MENUITEM_REFUEL_DIALG, ctx->id))
    {
        goto fail;
    }
    else
    {
        XPLMAppendMenuSeparator(ctx->id);
    }
    if (get_dataref(&ctx->data.refuel_dialg.fuel_max, "sim/aircraft/weight/acf_m_fuel_tot" ) ||
        get_dataref(&ctx->data.refuel_dialg.fuel_rat, "sim/aircraft/overflow/acf_tank_rat" ) ||
        get_dataref(&ctx->data.refuel_dialg.fuel_qty, "sim/flightmodel/weight/m_fuel"      ) ||
        get_dataref(&ctx->data.refuel_dialg.fuel_tot, "sim/flightmodel/weight/m_fuel_total") ||
        get_dataref(&ctx->data.refuel_dialg.pay_load, "sim/flightmodel/weight/m_fixed"     ))
    {
        goto fail;
    }

    /* speak local weather */
    if (append_menu_item("Speak weather", &ctx->items.speakweather,
                         MENUITEM_SPEAKWEATHER, ctx->id))
    {
        goto fail;
    }
    else
    {
        XPLMAppendMenuSeparator(ctx->id);
    }
    if (get_dataref(&ctx->data.speakweather.baro_sl, "sim/weather/barometer_sealevel_inhg") ||
        get_dataref(&ctx->data.speakweather.wind_dt, "sim/weather/wind_direction_degt[0]" ) ||
        get_dataref(&ctx->data.speakweather.wind_sd, "sim/weather/wind_speed_kt[0]"       ) ||
        get_dataref(&ctx->data.speakweather.temp_dc, "sim/weather/temperature_ambient_c"  ) ||
        get_dataref(&ctx->data.speakweather.temp_dp, "sim/weather/dewpoi_sealevel_c"      ))
    {
        goto fail;
    }

    /* YFMS main window */
    if (append_menu_item("YFMS toggle", &ctx->items.yfms_mainwin,
                         MENUITEM_YFMS_MAINWIN, ctx->id))
    {
        goto fail;
    }
    if ((ctx->data.yfms.context = yfs_main_init()) == NULL)
    {
        goto fail;
    }

    /* all good */
    return ctx;

fail:
    nvp_menu_close((void**)&ctx);
    return NULL;
}

int nvp_menu_setup(void *_menu_context)
{
    menu_context *ctx = _menu_context;
    if (ctx && !ctx->setupdone)
    {
        /*
         * Disable X-Plane 10 built-in ATC using "art" controls.
         *
         * In theory, having it enable doesn't hurt, but the ATC system may be
         * the cause of several X-Plane crashes I've experienced, so let's not
         * take any chances, kill it until it dies!
         */
        XPLMDataRef perf_kill_atc = XPLMFindDataRef("sim/private/controls/perf/kill_atc");
        if (perf_kill_atc)
        {
            XPLMSetDataf(perf_kill_atc, 1.0f);
            ndt_log("navP [debug]: killing evil ATC (%.1lf)\n", XPLMGetDataf(perf_kill_atc));
        }

        /*
         * Enable X-Plane real weather on startup (may be reset after a crash).
         *
         * TODO: detect "NOAA" plugin and adjust accordingly.
         */
        XPLMDataRef urw = XPLMFindDataRef("sim/weather/use_real_weather_bool");
        XPLMDataRef drw = XPLMFindDataRef("sim/weather/download_real_weather");
        if (urw && drw)
        {
            XPLMSetDatai(urw, 1); // enable
            XPLMSetDatai(drw, 1); // download
            ndt_log("navP [info]: enabling X-Plane real weather\n");
        }

        /*
         * Set defaults for dataref-backed variables,
         * since we can't do it in XPluginEnable().
         *
         * Future: read from a config file instead of hardcoding said defaults.
         */
        /*
         * Speed boost features.
         * Note: art controls aren't available in XPluginEnable(),
         *       so we must resolve private datarefs here instead.
         *
         * Defaults are loosely based on:
         * - Tom Knudsen's Clouds 2015 v1.1 script
         * - maydayc's "XP10 Performance tweak" forum post
         * - jörn-jören jörensön's 3jFPS-control 1.23 script
         *
         * Check whewther SkyMaxx Pro is installed & enabled
         * and disable all cloud-related features when it is.
         */
        XPLMPluginID skyMaxxPro;
        ctx->data.speedbooster.dr_kill_2d = XPLMFindDataRef("sim/private/controls/clouds/kill_2d"               );
        ctx->data.speedbooster.dr_kill_3d = XPLMFindDataRef("sim/private/controls/clouds/kill_3d"               );
        ctx->data.speedbooster.dr_skpdraw = XPLMFindDataRef("sim/private/controls/clouds/skip_draw"             );
        ctx->data.speedbooster.dr_frstr3d = XPLMFindDataRef("sim/private/controls/clouds/first_res_3d"          );
        ctx->data.speedbooster.dr_lastr3d = XPLMFindDataRef("sim/private/controls/clouds/last_res_3d"           );
        ctx->data.speedbooster.dr_plotrad = XPLMFindDataRef("sim/private/controls/clouds/plot_radius"           );
        ctx->data.speedbooster.dr_shadrad = XPLMFindDataRef("sim/private/controls/clouds/shad_radius"           );
        ctx->data.speedbooster.dr_limitfr = XPLMFindDataRef("sim/private/controls/clouds/limit_far"             );
        ctx->data.speedbooster.dr_difgain = XPLMFindDataRef("sim/private/controls/clouds/diffuse_gain"          );
        ctx->data.speedbooster.dr_ovrdctl = XPLMFindDataRef("sim/private/controls/clouds/overdraw_control"      );
        ctx->data.speedbooster.dr_use_csm = XPLMFindDataRef("sim/private/controls/caps/use_csm"                 );
        ctx->data.speedbooster.dr_disprep = XPLMFindDataRef("sim/private/controls/perf/disable_shadow_prep"     );
        ctx->data.speedbooster.dr_disrcam = XPLMFindDataRef("sim/private/controls/perf/disable_reflection_cam"  );
        if (XPLM_NO_PLUGIN_ID != (skyMaxxPro = XPLMFindPluginBySignature("SilverLiningV2.Clouds")) ||
            XPLM_NO_PLUGIN_ID != (skyMaxxPro = XPLMFindPluginBySignature("SilverLiningV3.Clouds")))
        {
            if (XPLMIsPluginEnabled(skyMaxxPro))
            {
                menu_rm_item(ctx,
                             ctx->items.cloud_killer.id);
                ctx->data.speedbooster.dr_kill_2d = NULL;
                ctx->data.speedbooster.dr_kill_3d = NULL;
                ctx->data.speedbooster.dr_skpdraw = NULL;
                ctx->data.speedbooster.dr_frstr3d = NULL;
                ctx->data.speedbooster.dr_lastr3d = NULL;
                ctx->data.speedbooster.dr_plotrad = NULL;
                ctx->data.speedbooster.dr_shadrad = NULL;
                ctx->data.speedbooster.dr_limitfr = NULL;
                ctx->data.speedbooster.dr_difgain = NULL;
                ctx->data.speedbooster.dr_ovrdctl = NULL;
            }
        }
        if (ctx->data.speedbooster.dr_kill_2d)
        {
            XPLMSetDataf(ctx->data.speedbooster.dr_kill_2d, 0.00f);
        }
        if (ctx->data.speedbooster.dr_kill_3d)
        {
            XPLMSetDataf(ctx->data.speedbooster.dr_kill_3d, 0.00f);
        }
        if (ctx->data.speedbooster.dr_skpdraw)
        {
            XPLMSetDataf(ctx->data.speedbooster.dr_skpdraw, 0.00f);
        }
        if (ctx->data.speedbooster.dr_frstr3d)
        {
            ctx->data.speedbooster.df_frstr3d = XPLMGetDataf(ctx->data.speedbooster.dr_frstr3d);
            XPLMSetDataf(ctx->data.speedbooster.dr_frstr3d, 3.00f);
        }
        if (ctx->data.speedbooster.dr_lastr3d)
        {
            ctx->data.speedbooster.df_lastr3d = XPLMGetDataf(ctx->data.speedbooster.dr_lastr3d);
            XPLMSetDataf(ctx->data.speedbooster.dr_lastr3d, 3.00f);
        }
        if (ctx->data.speedbooster.dr_plotrad)
        {
            ctx->data.speedbooster.df_plotrad = XPLMGetDataf(ctx->data.speedbooster.dr_plotrad);
            XPLMSetDataf(ctx->data.speedbooster.dr_plotrad, 0.60f);
        }
        if (ctx->data.speedbooster.dr_shadrad)
        {
            ctx->data.speedbooster.df_shadrad = XPLMGetDataf(ctx->data.speedbooster.dr_shadrad);
            XPLMSetDataf(ctx->data.speedbooster.dr_shadrad, 0.40f);
        }
        if (ctx->data.speedbooster.dr_limitfr)
        {
            ctx->data.speedbooster.df_limitfr = XPLMGetDataf(ctx->data.speedbooster.dr_limitfr);
            XPLMSetDataf(ctx->data.speedbooster.dr_limitfr, 0.35f);
        }
        if (ctx->data.speedbooster.dr_difgain)
        {
            ctx->data.speedbooster.df_difgain = XPLMGetDataf(ctx->data.speedbooster.dr_difgain);
            XPLMSetDataf(ctx->data.speedbooster.dr_difgain, 1.00f);
        }
        if (ctx->data.speedbooster.dr_ovrdctl)
        {
            ctx->data.speedbooster.df_ovrdctl = XPLMGetDataf(ctx->data.speedbooster.dr_ovrdctl);
            XPLMSetDataf(ctx->data.speedbooster.dr_ovrdctl, 1.00f);
        }
        if (ctx->data.speedbooster.dr_use_csm)
        {
            ctx->data.speedbooster.df_use_csm = XPLMGetDataf(ctx->data.speedbooster.dr_use_csm);
            XPLMSetDataf(ctx->data.speedbooster.dr_use_csm, 0.00f);
        }
        if (ctx->data.speedbooster.dr_disprep)
        {
            ctx->data.speedbooster.df_disprep = XPLMGetDataf(ctx->data.speedbooster.dr_disprep);
            XPLMSetDataf(ctx->data.speedbooster.dr_disprep, 1.00f);
        }
        if (ctx->data.speedbooster.dr_disrcam)
        {
            ctx->data.speedbooster.df_disrcam = XPLMGetDataf(ctx->data.speedbooster.dr_disrcam);
            XPLMSetDataf(ctx->data.speedbooster.dr_disrcam, 1.00f);
        }
        XPLMCheckMenuItem(ctx->id, ctx->items.cloud_killer.id, xplm_Menu_NoCheck);
        XPLMCheckMenuItem(ctx->id, ctx->items.speedbooster.id, xplm_Menu_Checked);
        ndt_log          ("navP [info]: enabling clouds & Tachyon Enhancement\n");

        /* custom brake brake and speedbrake callouts */
        XPLMCheckMenuItem(ctx->id, ctx->items.callouts_sts.id, xplm_Menu_Checked);
        XPLMSetDatai     (         ctx->data.callouts_sts.park_brake,          1);
        XPLMSetDatai     (         ctx->data.callouts_sts.speedbrake,          1);
        XPLMSetDatai     (         ctx->data.callouts_sts.flap_lever,          1);
        ndt_log          (             "navP [info]: enabling custom callouts\n");

        /*
         * Create and place the payload & fuel dialog's window and contents.
         * Do not show it until requested by the user (hide root by default).
         *
         * x and y axes start at bottom left (obviously, except for me).
         * We create widgets directly there: 0 -> (height - 1)
         *                                   0 -> (width  - 1)
         * We move the widgets to the display center just before showing them.
         * Thankfully the child widgets move with their parents :-)
         */
        /* Payload & fuel main window (root, container) */
        int inBM = 0;
        int inLT = 0;
        int inTP = REFUEL_DIALG_DEFH - 1;
        int inRT = REFUEL_DIALG_DEFW - 1;
        ctx->data.refuel_dialg.dialog_id = XPCreateWidget(inLT, inTP, inRT, inBM,
                                                          0, "Fuel & Payload", 1, NULL,
                                                          xpWidgetClass_MainWindow);
        if (!ctx->data.refuel_dialg.dialog_id)
        {
            ndt_log("navP [warning]: could not create payload & fuel dialog\n");
            menu_rm_item(ctx, ctx->items.refuel_dialg.id);
            ctx->setupdone = -1; return -1;
        }
        XPSetWidgetProperty(ctx->data.refuel_dialg.dialog_id,
                            xpProperty_MainWindowHasCloseBoxes, 1);
        XPSetWidgetProperty(ctx->data.refuel_dialg.dialog_id,
                            xpProperty_MainWindowType, xpMainWindowStyle_Translucent);
        XPAddWidgetCallback(ctx->data.refuel_dialg.dialog_id, &widget_hdlr1);
        /* push button to initialie re/defuel */
        inLT = (REFUEL_DIALG_DEFW - REFUEL_DIALG_CLBW) / 2;
        inTP = (REFUEL_DIALG_CLBH) - 1 + (inBM = 10);
        inRT = (REFUEL_DIALG_CLBW) - 1 + (inLT);
        ctx->data.refuel_dialg.button_id = XPCreateWidget(inLT, inTP, inRT, inBM,
                                                          1, "LOAD/UNLOAD", 0,
                                                          ctx->data.refuel_dialg.dialog_id,
                                                          xpWidgetClass_Button);
        if (!ctx->data.refuel_dialg.button_id)
        {
            ndt_log("navP [warning]: could not create fuel & payload button\n");
            menu_rm_item(ctx, ctx->items.refuel_dialg.id);
            ctx->setupdone = -1; return -1;
        }
        XPSetWidgetProperty(ctx->data.refuel_dialg.button_id,
                            xpProperty_ButtonType, xpPushButton);
        XPSetWidgetProperty(ctx->data.refuel_dialg.button_id,
                            xpProperty_Refcon, (intptr_t)ctx);
        /* caption & fuel entry text field */
        inLT = (((REFUEL_DIALG_DEFW - REFUEL_DIALG_CAPW - REFUEL_DIALG_TXTW) / 3) & (~1));
        inBM = (((REFUEL_DIALG_DEFH * 15 / 20) & (~1)));
        inTP = (((REFUEL_DIALG_CAPH) - 1 + (inBM)));
        inRT = (((REFUEL_DIALG_CAPW) - 1 + (inLT)));
        XPWidgetID f_txtx_ll = XPCreateWidget(inLT, inTP, inRT, inBM,
                                              1, "(KG x1000) FUEL", 0,
                                              ctx->data.refuel_dialg.dialog_id,
                                              xpWidgetClass_Caption);
        if (!f_txtx_ll)
        {
            ndt_log("navP [warning]: could not create label for fuel entry\n");
            menu_rm_item(ctx, ctx->items.refuel_dialg.id);
            ctx->setupdone = -1; return -1;
        }
        XPSetWidgetProperty(f_txtx_ll, xpProperty_CaptionLit, 1);
        inLT = (REFUEL_DIALG_CAPW + inLT * 2); // right next to the field caption
        inTP = (REFUEL_DIALG_TXTH) - 1 + (inBM = inBM - 2); // widget mismatch
        inRT = (REFUEL_DIALG_TXTW) - 1 + (inLT);
        ctx->data.refuel_dialg.f_txtf_id = XPCreateWidget(inLT, inTP, inRT, inBM,
                                                          1, "0.0", 0,
                                                          ctx->data.refuel_dialg.dialog_id,
                                                          xpWidgetClass_TextField);
        if (!ctx->data.refuel_dialg.f_txtf_id)
        {
            ndt_log("navP [warning]: could not create text field for fuel entry\n");
            menu_rm_item(ctx, ctx->items.refuel_dialg.id);
            ctx->setupdone = -1; return -1;
        }
        XPSetWidgetProperty(ctx->data.refuel_dialg.f_txtf_id,
                            xpProperty_TextFieldType, xpTextTranslucent);
        /* caption & payload entry text field */
        inLT = (((REFUEL_DIALG_DEFW - REFUEL_DIALG_CAPW - REFUEL_DIALG_TXTW) / 3) & (~1));
        inBM = (((REFUEL_DIALG_DEFH * 12 / 20) & (~1)));
        inTP = (((REFUEL_DIALG_CAPH) - 1 + (inBM)));
        inRT = (((REFUEL_DIALG_CAPW) - 1 + (inLT)));
        XPWidgetID p_txtx_ll = XPCreateWidget(inLT, inTP, inRT, inBM,
                                              1, "(KG x1000) PAYLOAD", 0,
                                              ctx->data.refuel_dialg.dialog_id,
                                              xpWidgetClass_Caption);
        if (!p_txtx_ll)
        {
            ndt_log("navP [warning]: could not create label for payload entry\n");
            menu_rm_item(ctx, ctx->items.refuel_dialg.id);
            ctx->setupdone = -1; return -1;
        }
        XPSetWidgetProperty(p_txtx_ll, xpProperty_CaptionLit, 1);
        inLT = (REFUEL_DIALG_CAPW + inLT * 2); // right next to the field caption
        inTP = (REFUEL_DIALG_TXTH) - 1 + (inBM = inBM - 2); // widget mismatch
        inRT = (REFUEL_DIALG_TXTW) - 1 + (inLT);
        ctx->data.refuel_dialg.p_txtf_id = XPCreateWidget(inLT, inTP, inRT, inBM,
                                                          1, "0.0", 0,
                                                          ctx->data.refuel_dialg.dialog_id,
                                                          xpWidgetClass_TextField);
        if (!ctx->data.refuel_dialg.p_txtf_id)
        {
            ndt_log("navP [warning]: could not create text field for payload entry\n");
            menu_rm_item(ctx, ctx->items.refuel_dialg.id);
            ctx->setupdone = -1; return -1;
        }
        XPSetWidgetProperty(ctx->data.refuel_dialg.p_txtf_id,
                            xpProperty_TextFieldType, xpTextTranslucent);

        /* all done, we're good to go! */
        ctx->setupdone = 1; return 0;
    }
    return ctx ? 0 : -1;
}

int nvp_menu_reset(void *_menu_context)
{
    menu_context *ctx = _menu_context;
    if (ctx && ctx->setupdone)
    {
        if (ctx->data.refuel_dialg.rfc)
        {
            XPLMUnregisterFlightLoopCallback(ctx->data.refuel_dialg.rfc, ctx);
            ctx->data.refuel_dialg.rfc = NULL;
        }
    }
    return ctx ? 0 : -1;
}

int nvp_menu_close(void **_menu_context)
{
    menu_context *ctx = *_menu_context;
    *_menu_context = NULL;
    if (!ctx)
    {
        return -1;
    }

    /* if the menu exists (has an ID), destroy it */
    if (ctx->id)
    {
        XPLMDestroyMenu(ctx->id);
    }

    /* if we still have a flight loop callback, unregister it */
    if (ctx->data.refuel_dialg.rfc)
    {
        XPLMUnregisterFlightLoopCallback(ctx->data.refuel_dialg.rfc, ctx);
    }

    /* if we nuke the menu, we nuke YFMS too */
    if (ctx->data.yfms.context)
    {
        yfs_main_close(&ctx->data.yfms.context);
    }

    /* all good */
    free(ctx);
    return 0;
}

static void mainw_center(int out[4], int in_w, int in_h)
{
    int scrn_w, scrn_h;
    XPLMGetScreenSize(&scrn_w, &scrn_h);
    out[3] = (scrn_h - in_h) / 2; // inBottom
    out[0] = (scrn_w - in_w) / 2; // inLeft
    out[1] = (out[3] + in_h) - 1; // inTop
    out[2] = (out[0] + in_w) - 1; // inRight
}

static void menu_rm_item(menu_context *ctx, int index)
{
#define MENUITEM_UNDEF_VAL(item_ctx) { if ((item_ctx).id == index) (item_ctx).mivalue = MENUITEM_NOTHING_TODO; }
#define MENUITEM_CHECK_IDX(item_idx) { if ((item_idx)     > index) (item_idx)--; }
    if (ctx && ctx->id)
    {
        XPLMRemoveMenuItem(ctx->id, index);
        MENUITEM_UNDEF_VAL(ctx->items.callouts_sts);
        MENUITEM_CHECK_IDX(ctx->items.callouts_sts.id);
        MENUITEM_UNDEF_VAL(ctx->items.speedbooster);
        MENUITEM_CHECK_IDX(ctx->items.speedbooster.id);
        MENUITEM_UNDEF_VAL(ctx->items.cloud_killer);
        MENUITEM_CHECK_IDX(ctx->items.cloud_killer.id);
        MENUITEM_UNDEF_VAL(ctx->items.dummy_item_1);
        MENUITEM_CHECK_IDX(ctx->items.dummy_item_1.id);
        MENUITEM_UNDEF_VAL(ctx->items.refuel_dialg);
        MENUITEM_CHECK_IDX(ctx->items.refuel_dialg.id);
        MENUITEM_UNDEF_VAL(ctx->items.speakweather);
        MENUITEM_CHECK_IDX(ctx->items.speakweather.id);
    }
#undef MENUITEM_CHECK_IDX
#undef MENUITEM_UNDEF_VAL
}

static void menu_handler(void *inMenuRef, void *inItemRef)
{
    menu_context *ctx = inMenuRef;
    item_context *itx = inItemRef;

    if (itx->mivalue == MENUITEM_NOTHING_TODO)
    {
        return;
    }

    if (itx->mivalue == MENUITEM_CALLOUTS_STS)
    {
        XPLMMenuCheck state = xplm_Menu_Checked;
        XPLMCheckMenuItemState(ctx->id, itx->id, &state);
        if (state == xplm_Menu_Checked)
        {
            XPLMCheckMenuItem(ctx->id, itx->id,  xplm_Menu_NoCheck);
            XPLMSetDatai     (ctx->data.callouts_sts.park_brake, 0);
            XPLMSetDatai     (ctx->data.callouts_sts.speedbrake, 0);
            XPLMSetDatai     (ctx->data.callouts_sts.flap_lever, 0);
            ndt_log    ("navP [info]: disabling custom callouts\n");
            return;
        }
        XPLMCheckMenuItem(ctx->id, itx->id,  xplm_Menu_Checked);
        XPLMSetDatai     (ctx->data.callouts_sts.park_brake, 1);
        XPLMSetDatai     (ctx->data.callouts_sts.speedbrake, 1);
        XPLMSetDatai     (ctx->data.callouts_sts.flap_lever, 1);
        ndt_log     ("navP [info]: enabling custom callouts\n");
        return;
    }

    if (itx->mivalue == MENUITEM_REFUEL_DIALG)
    {
        float fuel_rat[9];
        ctx->data.refuel_dialg.num_fuel_tanks = 0;
        ctx->data.refuel_dialg.max_fuel_total =
        floorf(XPLMGetDataf(ctx->data.refuel_dialg.fuel_max));
        ctx->data.refuel_dialg.min_fuel_total =
        floorf(XPLMGetDataf(ctx->data.refuel_dialg.fuel_max) / 6.242f); // 18726max -> 3000min (QPAC A320)
        XPLMGetDatavf(ctx->data.refuel_dialg.fuel_rat, fuel_rat, 0, 9);
        for (int i = 0; i < 9; i++)
        {
            if (fuel_rat[i] > .01f)
            {
                ctx->data.refuel_dialg.fuel_tank_r[i] = fuel_rat[i];
                ctx->data.refuel_dialg.num_fuel_tanks++;
            }
        }
#if 0
        {
            ndt_log("navP [info]: Fuel tanks: %d, ratio:", ctx->data.refuel_dialg.num_fuel_tanks);
            for (int i = 0; i < ctx->data.refuel_dialg.num_fuel_tanks; i++)
            {
                ndt_log(" %.3f", ctx->data.refuel_dialg.fuel_tank_r[i]);
            }
            ndt_log(", maximum: %.0f kgs (%.0f lbs)\n",
                    ctx->data.refuel_dialg.max_fuel_total,
                    floorf(XPLMGetDataf(ctx->data.refuel_dialg.fuel_max) / 0.45359f));
        }
#endif
        if (XPIsWidgetVisible(ctx->data.refuel_dialg.dialog_id) == 0)
        {
            int g[4];
            if (ctx->data.refuel_dialg.rfc)
            {
                XPLMUnregisterFlightLoopCallback(ctx->data.refuel_dialg.rfc, ctx);
                ctx->data.refuel_dialg.rfc = NULL;
            }
            snprintf(       ctx->data.refuel_dialg.fueltot_str,
                     sizeof(ctx->data.refuel_dialg.fueltot_str), "%.2f",
                     XPLMGetDataf(ctx->data.refuel_dialg.fuel_tot) / 1000.0f);
            XPSetWidgetDescriptor(ctx->data.refuel_dialg.f_txtf_id,
                                  ctx->data.refuel_dialg.fueltot_str);
            snprintf(       ctx->data.refuel_dialg.payload_str,
                     sizeof(ctx->data.refuel_dialg.payload_str), "%.2f",
                     XPLMGetDataf(ctx->data.refuel_dialg.pay_load) / 1000.0f);
            XPSetWidgetDescriptor(ctx->data.refuel_dialg.p_txtf_id,
                                  ctx->data.refuel_dialg.payload_str);
            mainw_center            (g, REFUEL_DIALG_DEFW, REFUEL_DIALG_DEFH);
            XPSetWidgetGeometry     (ctx->data.refuel_dialg.dialog_id, g[0], g[1], g[2], g[3]);
            XPShowWidget            (ctx->data.refuel_dialg.dialog_id);
            XPBringRootWidgetToFront(ctx->data.refuel_dialg.dialog_id);
        }
        return;
    }

    if (itx->mivalue == MENUITEM_SPEAKWEATHER)
    {
        char  baro[127], wind[127], temp[127], weather[255];
        float baro_sl = XPLMGetDataf(ctx->data.speakweather.baro_sl);
        float wind_dt = XPLMGetDataf(ctx->data.speakweather.wind_dt);
        float wind_sd = XPLMGetDataf(ctx->data.speakweather.wind_sd);
        float temp_dc = XPLMGetDataf(ctx->data.speakweather.temp_dc);
        float temp_dp = XPLMGetDataf(ctx->data.speakweather.temp_dp);
        snprintf(baro, sizeof(baro), "Altimeter %04d, area QNH %04d.",
                 (int)round(baro_sl * 100.), (int)round(baro_sl * 33.86389));
        if (wind_sd < 4.5f)
        {
            if (wind_sd < 1.5f)
            {
                snprintf(wind, sizeof(wind), "%s", "Wind calm.");
            }
            else
            {
                snprintf(wind, sizeof(wind), "%s", "Wind light and variable.");
            }
        }
        else
        {
            snprintf(wind, sizeof(wind), "Wind %03d at %d.",
                     (int)wind_dt, (int)round(wind_sd));
        }
        snprintf(temp, sizeof(temp), "Temperature %d, dew point %d.",
                 (int)round(temp_dc), (int)round(temp_dp));
        snprintf(weather, sizeof(weather), "%s %s %s",
                 baro, wind, temp);
        XPLMSpeakString(string4speak(weather, sizeof(weather), 0));
        return;
    }

#define SPEEDBOOSTER_DEFAULTV(_function, _dataref, _default) \
{ if (ctx->data.speedbooster._dataref) { _function(ctx->data.speedbooster._dataref, ctx->data.speedbooster._default); } }
#define SPEEDBOOSTER_SETVALUE(_function, _dataref, _value) \
{ if (ctx->data.speedbooster._dataref) { _function(ctx->data.speedbooster._dataref, (_value)); } }
    if (itx->mivalue == MENUITEM_SPEEDBOOSTER)
    {
        XPLMMenuCheck state = xplm_Menu_Checked;
        XPLMCheckMenuItemState(ctx->id, itx->id, &state);
        if (state == xplm_Menu_Checked)
        {
            XPLMCheckMenuItem(ctx->id, itx->id, xplm_Menu_NoCheck);
            SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_frstr3d, df_frstr3d);
            SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_lastr3d, df_lastr3d);
            SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_plotrad, df_plotrad);
            SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_shadrad, df_shadrad);
            SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_limitfr, df_limitfr);
            SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_difgain, df_difgain);
            SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_ovrdctl, df_ovrdctl);
            SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_use_csm, df_use_csm);
            SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_disprep, df_disprep);
            SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_disrcam, df_disrcam);
            ndt_log(    "navP [info]: disabling Tachyon Enhancement\n");
            return;
        }
        XPLMCheckMenuItem(ctx->id, itx->id, xplm_Menu_Checked);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_frstr3d, 3.00f);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_lastr3d, 3.00f);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_plotrad, 0.60f);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_shadrad, 0.40f);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_limitfr, 0.35f);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_difgain, 1.00f);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_ovrdctl, 1.00f);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_use_csm, 0.00f);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_disprep, 1.00f);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_disrcam, 1.00f);
        ndt_log("navP [info]: enabling Tachyon Enhancement\n");
        return;
    }
    if (itx->mivalue == MENUITEM_CLOUD_KILLER)
    {
        XPLMMenuCheck state = xplm_Menu_Checked;
        XPLMCheckMenuItemState(ctx->id, itx->id, &state);
        if (state == xplm_Menu_Checked)
        {
            XPLMCheckMenuItem(ctx->id, itx->id, xplm_Menu_NoCheck);
            SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_kill_2d, 0.00f);
            SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_kill_3d, 0.00f);
            SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_skpdraw, 0.00f);
            ndt_log(          "navP [info]: re-enabling clouds\n");
            return;
        }
        XPLMCheckMenuItem(ctx->id, itx->id, xplm_Menu_Checked);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_kill_2d, 1.00f);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_kill_3d, 1.00f);
        SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_skpdraw, 1.00f);
        ndt_log(     "navP [info]: disabling clouds (INOP)\n");
        return;
    }
#undef SPEEDBOOSTER_DEFAULTV
#undef SPEEDBOOSTER_SETVALUE

    if (itx->mivalue >= MENUITEM_VOLUME_PRST0 &&
        itx->mivalue <= MENUITEM_VOLUME_PRST4)
    {
        float   volume_ratio;
        switch (itx->mivalue)
        {
            case MENUITEM_VOLUME_PRST0:
                volume_ratio = 0.00f;
                break;
            case MENUITEM_VOLUME_PRST1:
                volume_ratio = 0.25f;
                break;
            case MENUITEM_VOLUME_PRST2:
                volume_ratio = 0.50f;
                break;
            case MENUITEM_VOLUME_PRST3:
                volume_ratio = 0.75f;
                break;
            case MENUITEM_VOLUME_PRST4:
                volume_ratio = 1.00f;
                break;
            default:
                return;
        }
        // TODO: different ratios for e.g. radios vs. propellers vs. eng etc.
        XPLMSetDataf(ctx->data.volume_prsts.dr_vol_eng, volume_ratio);
        XPLMSetDataf(ctx->data.volume_prsts.dr_vol_prs, volume_ratio);
        XPLMSetDataf(ctx->data.volume_prsts.dr_vol_grt, volume_ratio);
        XPLMSetDataf(ctx->data.volume_prsts.dr_vol_wer, volume_ratio);
        XPLMSetDataf(ctx->data.volume_prsts.dr_vol_was, volume_ratio);
        XPLMSetDataf(ctx->data.volume_prsts.dr_vol_coo, volume_ratio);
        XPLMSetDataf(ctx->data.volume_prsts.dr_vol_avs, volume_ratio);
        XPLMSpeakString("Volume set");
        return;
    }

    if (itx->mivalue >= MENUITEM_FILDOV_45DEG &&
        itx->mivalue <= MENUITEM_FILDOV_75DEG)
    {
        float  field_of_view;
        switch (itx->mivalue)
        {
            case MENUITEM_FILDOV_45DEG:
                field_of_view = 45.0f;
                break;
            case MENUITEM_FILDOV_50DEG:
                field_of_view = 50.0f;
                break;
            case MENUITEM_FILDOV_55DEG:
                field_of_view = 55.0f;
                break;
            case MENUITEM_FILDOV_60DEG:
                field_of_view = 60.0f;
                break;
            case MENUITEM_FILDOV_65DEG:
                field_of_view = 65.0f;
                break;
            case MENUITEM_FILDOV_70DEG:
                field_of_view = 70.0f;
                break;
            case MENUITEM_FILDOV_75DEG:
                field_of_view = 75.0f;
                break;
            default:
                return;
        }
        XPLMSetDataf(ctx->data.fildov_prsts.fildov_deg, field_of_view);
        XPLMSpeakString("F O V set");
        return;
    }

    if (itx->mivalue == MENUITEM_XFMC01_NABLE)
    {
        if (XPLMIsPluginEnabled(ctx->data.xfmc.plugin_id) == 0)
        {
            XPLMEnablePlugin   (ctx->data.xfmc.plugin_id);
            XPLMSpeakString    ("X FMC enabled");
        }
        return;
    }
    if (itx->mivalue == MENUITEM_XFMC01_DSBLE)
    {
        if (XPLMIsPluginEnabled(ctx->data.xfmc.plugin_id) != 0)
        {
            XPLMDisablePlugin  (ctx->data.xfmc.plugin_id);
            XPLMSpeakString    ("X FMC disabled");
        }
        return;
    }

    if (itx->mivalue == MENUITEM_YFMS_MAINWIN)
    {
        yfs_main_toggl(ctx->data.yfms.context);
        return;
    }
}

static char* string4speak(char *string, size_t alloc, int text_only)
{
    char   buffer[2048];
    int    isname = 0;
    size_t maxlen = alloc > sizeof(buffer) ? sizeof(buffer) : alloc;
    char  *out    = buffer;
    char  *in     = string;
    if   (!in)
    {
        return NULL;
    }

    while (*in != '\0' && alloc > 1)
    {
        if (in != string && ((*(in-1)  < '0' || *(in-1)  > '9') &&
                             (*(in-1) != ' ' && *(in-1) != ',' && *(in-1) != '.')))
        {
            // previous character neither a digit nor a space/comma/period
            // if this character is a digit, it's got to be part of a name
            if (*in >= '0' && *in <= '9')
            {
                isname = 1;
            }
        }
        else if (isname && (*in < '0' || *in > '9'))
        {
            // this character is not a digit, if we had a name, it ends here
            isname = 0;
        }
        switch (*in)
        {
            case '0':
                if (text_only && !isname)
                {
                    int ret = snprintf(out, alloc, "%s ", "zero");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
            case '1':
                if (text_only && !isname)
                {
                    int ret = snprintf(out, alloc, "%s ", "one");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
            case '2':
                if (text_only && !isname)
                {
                    int ret = snprintf(out, alloc, "%s ", "two");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
            case '3':
                if (text_only && !isname)
                {
                    int ret = snprintf(out, alloc, "%s ", "tree");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
            case '4':
                if (text_only && !isname)
                {
                    int ret = snprintf(out, alloc, "%s ", "four");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
            case '5':
                if (text_only && !isname)
                {
                    int ret = snprintf(out, alloc, "%s ", "fife");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
            case '6':
                if (text_only && !isname)
                {
                    int ret = snprintf(out, alloc, "%s ", "sixer");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
            case '7':
                if (text_only && !isname)
                {
                    int ret = snprintf(out, alloc, "%s ", "seven");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
            case '8':
                if (text_only && !isname)
                {
                    int ret = snprintf(out, alloc, "%s ", "eight");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
            case '9':
                if (text_only && !isname)
                {
                    int ret = snprintf(out, alloc, "%s ", "niner");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
                else if (!isname)
                {
                    int ret = snprintf(out, alloc, "%c ", *in);
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
                *out++ = *in++;
                alloc--;
                break;

            case '.':
                if (*(in+1) >= '0' && *(in+1) <= '9')
                {
                    int ret = snprintf(out, alloc, "%s ", "point");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
                *out++ = *in++;
                alloc--;
                break;

            case '-':
                if (*(in+1) >= '0' && *(in+1) <= '9')
                {
                    int ret = snprintf(out, alloc, "%s ", "minus");
                    if (ret < alloc && ret > 0)
                    {
                        alloc -= ret;
                        out   += ret;
                        in    += 1;
                        break;
                    }
                    alloc = 0; // no space left (out is terminated by snprintf)
                    break;
                }
                *out++ = *in++;
                alloc--;
                break;

            default:
                *out++ = *in++;
                alloc--;
                break;
        }
        if (alloc > 0)
        {
            if ((*(out-1) == ' ') && (*in == ' ' || *in == ',' || *in == '.'))
            {
                // remove space if next character is a space/comma/period
                alloc++;
                out--;
            }
            *out = '\0';
        }
    }
    return strncpy(string, buffer, maxlen);
}

static int widget_hdlr1(XPWidgetMessage inMessage,
                        XPWidgetID      inWidget,
                        intptr_t        inParam1,
                        intptr_t        inParam2)
{
    /*
     * the callback is only ever added to a specific
     * main window, with just a single custom button
     */
    if (inMessage == xpMessage_CloseButtonPushed)
    {
        XPHideWidget(inWidget);
        return 1;
    }
    if (inMessage == xpMsg_PushButtonPressed)
    {
        float temp1, temp2; char descr_buf[9]; descr_buf[8] = '\0';
        menu_context *ctx = (void*)XPGetWidgetProperty((void*)inParam1, xpProperty_Refcon, NULL);
        ctx->data.refuel_dialg.adjust_fuel = ctx->data.refuel_dialg.adjust_plod = 0;
        XPHideWidget         (ctx->data.refuel_dialg.dialog_id);
        XPGetWidgetDescriptor(ctx->data.refuel_dialg.f_txtf_id, descr_buf, sizeof(descr_buf) - 1);
        if (sscanf(descr_buf, "%f", &temp1) == 1)
        {
            temp2 = XPLMGetDataf(ctx->data.refuel_dialg.fuel_tot);
            ctx->data.refuel_dialg.fuel_target_kg = 1000.0f * temp1;
            if (fabsf(temp2 - ctx->data.refuel_dialg.fuel_target_kg) > 50.0f)
            {
                ctx->data.refuel_dialg.adjust_fuel = 1;
            }
            if (ctx->data.refuel_dialg.fuel_target_kg < ctx->data.refuel_dialg.min_fuel_total)
            {
                ctx->data.refuel_dialg.fuel_target_kg = ctx->data.refuel_dialg.min_fuel_total;
            }
            if (ctx->data.refuel_dialg.fuel_target_kg > ctx->data.refuel_dialg.max_fuel_total)
            {
                ctx->data.refuel_dialg.fuel_target_kg = ctx->data.refuel_dialg.max_fuel_total;
            }
        }
        XPGetWidgetDescriptor(ctx->data.refuel_dialg.p_txtf_id, descr_buf, sizeof(descr_buf) - 1);
        if (sscanf(descr_buf, "%f", &temp1) == 1)
        {
            temp2 = XPLMGetDataf(ctx->data.refuel_dialg.pay_load);
            ctx->data.refuel_dialg.plod_target_kg = 1000.0f * temp1;
            if (fabsf(temp2 - ctx->data.refuel_dialg.plod_target_kg) > 50.0f)
            {
                ctx->data.refuel_dialg.adjust_plod = 1;
            }
            if (ctx->data.refuel_dialg.fuel_target_kg < 50.0f)
            {
                ctx->data.refuel_dialg.fuel_target_kg = 0.0f;
            }
        }
        if (ctx->data.refuel_dialg.rfc)
        {
            XPLMUnregisterFlightLoopCallback(ctx->data.refuel_dialg.rfc, ctx);
        }
        if (ctx->data.refuel_dialg.adjust_fuel || ctx->data.refuel_dialg.adjust_plod)
        {
            ndt_log("navP [info]: loading/unloading:");
            if (ctx->data.refuel_dialg.adjust_fuel)
            {
                ndt_log(" fuel %.2f metric tons", ctx->data.refuel_dialg.fuel_target_kg / 1000.0f);
                ndt_log(ctx->data.refuel_dialg.adjust_plod ? "," : "\n");
            }
            if (ctx->data.refuel_dialg.adjust_plod)
            {
                ndt_log(" payload %.2f metric tons\n", ctx->data.refuel_dialg.plod_target_kg / 1000.0f);
            }
            XPLMRegisterFlightLoopCallback((ctx->data.refuel_dialg.rfc = &refuel_hdlr1), 1.5f, ctx);
        }
        return 1;
    }
    return 0;
}

static float refuel_hdlr1(float inElapsedSinceLastCall,
                          float inElapsedTimeSinceLastFlightLoop,
                          int   inCounter,
                          void *inRefcon)
{
    int disable_cllbk = 1, c = 0;
    menu_context *ctx = inRefcon;
    if (ctx->data.refuel_dialg.adjust_fuel)
    {
        float fuel_qty[9];
        float fuel_tot = XPLMGetDataf(ctx->data.refuel_dialg.fuel_tot);
        if   (fuel_tot < ctx->data.refuel_dialg.fuel_target_kg - 1.0f)
        {
            disable_cllbk = 0;
            XPLMGetDatavf(ctx->data.refuel_dialg.fuel_qty, fuel_qty, 0,
                          ctx->data.refuel_dialg.num_fuel_tanks);
            for (int i = 0; i < ctx->data.refuel_dialg.num_fuel_tanks; i++)
            {
                fuel_qty[i] += REFUEL_FUEL_KHSEC * ctx->data.refuel_dialg.fuel_tank_r[i];
            }
            XPLMSetDatavf(ctx->data.refuel_dialg.fuel_qty, fuel_qty, 0,
                          ctx->data.refuel_dialg.num_fuel_tanks);
        }
        else if (fuel_tot > ctx->data.refuel_dialg.fuel_target_kg + 1.0f + REFUEL_FUEL_KHSEC)
        {
            disable_cllbk = 0;
            XPLMGetDatavf(ctx->data.refuel_dialg.fuel_qty, fuel_qty, 0,
                          ctx->data.refuel_dialg.num_fuel_tanks);
            for (int i = 0; i < ctx->data.refuel_dialg.num_fuel_tanks; i++)
            {
                fuel_qty[i] -= REFUEL_FUEL_KHSEC * ctx->data.refuel_dialg.fuel_tank_r[i];
            }
            XPLMSetDatavf(ctx->data.refuel_dialg.fuel_qty, fuel_qty, 0,
                          ctx->data.refuel_dialg.num_fuel_tanks);
        }
        else
        {
            if (c == 0)
            {
                c += 1;
                XPLMSpeakString("Fueling/de-fueling done");
            }
            ctx->data.refuel_dialg.adjust_fuel = 0;
            ndt_log("navP [info]: refuel/defuel procedure completed\n");
        }
    }
    if (ctx->data.refuel_dialg.adjust_plod)
    {
        float pay_load = XPLMGetDataf(ctx->data.refuel_dialg.pay_load);
        if   (pay_load < ctx->data.refuel_dialg.plod_target_kg - 1.0f)
        {
            disable_cllbk = 0;
            XPLMSetDataf(ctx->data.refuel_dialg.pay_load, pay_load + REFUEL_PLOD_KHSEC);
        }
        else if (pay_load > ctx->data.refuel_dialg.plod_target_kg + 1.0f + REFUEL_PLOD_KHSEC)
        {
            disable_cllbk = 0;
            XPLMSetDataf(ctx->data.refuel_dialg.pay_load, pay_load - REFUEL_PLOD_KHSEC);
        }
        else
        {
            if (c == 0)
            {
                c += 1;
                XPLMSpeakString("Boarding/de-boarding done");
            }
            ctx->data.refuel_dialg.adjust_plod = 0;
            ndt_log("navP [info]: board/deboard procedure completed\n");
        }
    }
    if (disable_cllbk)
    {
        return 0;
    }
    return 0.5f;
}

#undef REFUEL_DIALG_CAPW
#undef REFUEL_DIALG_CAPH
#undef REFUEL_DIALG_CLBW
#undef REFUEL_DIALG_CLBH
#undef REFUEL_DIALG_DEFW
#undef REFUEL_DIALG_DEFH
#undef REFUEL_DIALG_TXTW
#undef REFUEL_DIALG_TXTH
#undef REFUEL_FUEL_KHSEC
#undef REFUEL_PLOD_KHSEC
