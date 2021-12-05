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

#include <errno.h>
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

#include "assert/includes.h"

#include "common/common.h"

#include "ACFtypes.h"
#include "ACFvolumes.h"
#include "NVPmenu.h"

#define REFUEL_DIALG_CAPW ( 80)
#define REFUEL_DIALG_CAPH ( 24)
#define REFUEL_DIALG_TXTW ( 60)
#define REFUEL_DIALG_TXTH ( 24)
#define REFUEL_DIALG_CLBW (100)
#define REFUEL_DIALG_CLBH ( 24)
#define REFUEL_DIALG_DEFW (170)
#define REFUEL_DIALG_DEFH (170)
#define REFUEL_DIALG_TBAR ( 20)
#define LOAD_MINIMUM_RATE (20.0f)
#define LOAD_MINIMUM_DIFF (38.5f) // half an FSEconomy human
#define AUTOSNPRINTF(string, format, ...) snprintf(string, sizeof(string), format, __VA_ARGS__)

enum
{
    NVP_MENU_DECR = -1,
    NVP_MENU_DONE = +0,
    NVP_MENU_INCR = +1,
};

typedef struct
{
    int id;
    enum
    {
        MENUITEM_NOTHING_TODO,
//      MENUITEM_WEATHR_SM_IC,
//      MENUITEM_WEATHR_REALW,
//      MENUITEM_WEATHR_CAVOK,
        MENUITEM_VOLUME_SM_IC,
        MENUITEM_VOLUME_PRST0,
        MENUITEM_VOLUME_PRST1,
        MENUITEM_VOLUME_PRST2,
        MENUITEM_VOLUME_PRST3,
        MENUITEM_VOLUME_PRST4,
        MENUITEM_VOLUME_PRST5,
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
        MENUITEM_XFLTS1_SM_IC,
        MENUITEM_XFLTS1_NABLE,
        MENUITEM_XFLTS1_DSBLE,
        MENUITEM_RESET_ALLSYS,
        MENUITEM_SPEEDBOOSTER,
        MENUITEM_CLOUD_KILLER,
        MENUITEM_REFUEL_DIALG,
        MENUITEM_SPEAKWEATHER,
    } mivalue;
} item_context;

typedef struct
{
    XPLMMenuID id;
    int setupdone;

    struct
    {
//      struct
//      {
//          XPLMMenuID      sm_id;
//          item_context    sm_ic;
//          item_context    realw;
//          item_context    cavok;
//      } weathr;
        struct
        {
            XPLMMenuID      sm_id;
            item_context    sm_ic;
            item_context    prst0;
            item_context    prst1;
            item_context    prst2;
            item_context    prst3;
            item_context    prst4;
            item_context    prst5;
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
        struct
        {
            XPLMMenuID      sm_id;
            item_context    sm_ic;
            item_context    nable;
            item_context    dsble;
        } xflts1;
        item_context reset_allsys;
        item_context callouts_sts;
        item_context dummy_item_0;
//      item_context speedbooster;
        item_context cloud_killer;
        item_context dummy_item_1;
        item_context refuel_dialg;
        item_context speakweather;
    } items;

    struct
    {
        struct
        {
            acf_info_context *ic;
            int      adjust_fuel;
            int      adjust_load;
            int   refuel_started;
            int   brding_started;
            int   payload_is_zfw;
            float fuel_target_kg;
            float load_target_kg;
            float fuel_rate_kg_s;
            float load_rate_kg_s;
            float last_fuel_amnt;
            float last_load_amnt;
            char  fueltot_str[9];
            char  payload_str[9];
            XPWidgetID dialog_id;
            XPWidgetID f_txtl_id;
            XPWidgetID p_txtl_id;
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
            XPLMDataRef dr_k3drain; // sim/private/controls/rain/kill_3d_rain   (toggle: 0/1 (default 0))

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
//          XPLMDataRef dr_use_csm; // sim/private/controls/caps/use_csm
//          float       df_use_csm;
            XPLMDataRef dr_disprep; // sim/private/controls/perf/disable_shadow_prep
            float       df_disprep;
            XPLMDataRef dr_disrcam; // sim/private/controls/perf/disable_reflection_cam
            float       df_disrcam;

            // X-Plane 11.41
            XPLMDataRef dr_x_i_ver; // sim/version/xplane_internal_version
            XPLMDataRef dr_use_bmp; // sim/private/controls/reno/use_bump_maps
            int         df_use_bmp;
            XPLMDataRef dr_det_tex; // sim/private/controls/reno/use_detail_textures
            int         df_det_tex;
        } speedbooster;

        struct
        {
            XPLMDataRef fildov_deg;
            XPLMDataRef fildov_non;
        } fildov_prsts;

        struct
        {
            XPLMPluginID plugin_id;
        } xfmc;

        struct
        {
            XPLMPluginID plugin_id;
        } xfsr;

        struct
        {
            XPLMCommandRef all_sys_op;
        } reset;

        struct
        {
            XPLMCommandRef weather_ld;
            XPLMCommandRef weather_rg;
            XPLMDataRef real_use_bool;
            XPLMDataRef real_download;
            XPLMDataRef cloud_type_00;
            XPLMDataRef cloud_type_01;
            XPLMDataRef cloud_type_02;
            XPLMDataRef visibility_me;
            XPLMDataRef precp_percent;
            XPLMDataRef thndr_percent;
            XPLMDataRef turbl_percent;
            XPLMDataRef runway_frictn;
            XPLMDataRef turbulence[3];
            XPLMDataRef windspd[3][2];
        } weather;
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

//  /* weather sub-menu & its items */
//  /* TODO: disable when/if any of NOAA, xEnviro or X-Plane 11 is present */
//  if (append_menu_item("Weather", &ctx->items.weathr.sm_ic,
//                       MENUITEM_WEATHR_SM_IC, ctx->id))
//  {
//      goto fail;
//  }
//  else
//  {
//      XPLMAppendMenuSeparator(ctx->id);
//  }
//  if (create_menu("Weather",     ctx,    &ctx->items.weathr.sm_id,
//                  &menu_handler, ctx->id, ctx->items.weathr.sm_ic.id))
//  {
//      goto fail;
//  }
//  struct
//  {
//      const char *name;
//      int item_mivalue;
//      item_context *cx;
//  }
//  weathr_items[] =
//  {
//      { "X-Plane Real Weather", MENUITEM_WEATHR_REALW, &ctx->items.weathr.realw, },
//      { "Custom (CAVOK-ish)"  , MENUITEM_WEATHR_CAVOK, &ctx->items.weathr.cavok, },
//      {                   NULL,                     0,                     NULL, },
//  };
//  for (int i = 0; weathr_items[i].name; i++)
//  {
//      if (append_menu_item(weathr_items[i].name,
//                           weathr_items[i].cx,
//                           weathr_items[i].item_mivalue,
//                           ctx->items.weathr.sm_id))
//      {
//          goto fail;
//      }
//  }
//  if ((ctx->data.weather.weather_rg = XPLMFindCommand("sim/operation/regen_weather"    )) == NULL ||
//      (ctx->data.weather.weather_ld = XPLMFindCommand("sim/operation/load_real_weather")) == NULL)
//  {
//      goto fail;
//  }
//  if (get_dataref(&ctx->data.weather.real_use_bool, "sim/weather/use_real_weather_bool"   ) ||
//      get_dataref(&ctx->data.weather.real_download, "sim/weather/download_real_weather"   ) ||
//      get_dataref(&ctx->data.weather.cloud_type_00, "sim/weather/cloud_type[0]"           ) ||
//      get_dataref(&ctx->data.weather.cloud_type_01, "sim/weather/cloud_type[1]"           ) ||
//      get_dataref(&ctx->data.weather.cloud_type_02, "sim/weather/cloud_type[2]"           ) ||
//      get_dataref(&ctx->data.weather.visibility_me, "sim/weather/visibility_reported_m"   ) ||
//      get_dataref(&ctx->data.weather.precp_percent, "sim/weather/rain_percent"            ) ||
//      get_dataref(&ctx->data.weather.thndr_percent, "sim/weather/thunderstorm_percent"    ) ||
//      get_dataref(&ctx->data.weather.turbl_percent, "sim/weather/wind_turbulence_percent" ) ||
//      get_dataref(&ctx->data.weather.runway_frictn, "sim/weather/runway_friction"         ) ||
//      get_dataref(&ctx->data.weather.turbulence[0], "sim/weather/turbulence[0]"           ) ||
//      get_dataref(&ctx->data.weather.turbulence[1], "sim/weather/turbulence[1]"           ) ||
//      get_dataref(&ctx->data.weather.turbulence[1], "sim/weather/turbulence[2]"           ) ||
//      get_dataref(&ctx->data.weather.windspd[0][0], "sim/weather/wind_speed_kt[0]"        ) ||
//      get_dataref(&ctx->data.weather.windspd[1][0], "sim/weather/wind_speed_kt[1]"        ) ||
//      get_dataref(&ctx->data.weather.windspd[2][0], "sim/weather/wind_speed_kt[2]"        ) ||
//      get_dataref(&ctx->data.weather.windspd[0][1], "sim/weather/shear_speed_kt[0]"       ) ||
//      get_dataref(&ctx->data.weather.windspd[1][1], "sim/weather/shear_speed_kt[1]"       ) ||
//      get_dataref(&ctx->data.weather.windspd[2][1], "sim/weather/shear_speed_kt[2]"       ))
//  {
//      goto fail;
//  }

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
        { " 0.00 %", MENUITEM_VOLUME_PRST0, &ctx->items.volume.prst0, },
        { " 6.25 %", MENUITEM_VOLUME_PRST1, &ctx->items.volume.prst1, },
        { " 25.0 %", MENUITEM_VOLUME_PRST2, &ctx->items.volume.prst2, },
        { " 50.0 %", MENUITEM_VOLUME_PRST3, &ctx->items.volume.prst3, },
        { " 75.0 %", MENUITEM_VOLUME_PRST4, &ctx->items.volume.prst4, },
        { "100.0 %", MENUITEM_VOLUME_PRST5, &ctx->items.volume.prst5, },
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
    if (get_dataref(&ctx->data.fildov_prsts.fildov_non, "sim/graphics/settings/non_proportional_vertical_FOV") ||
        get_dataref(&ctx->data.fildov_prsts.fildov_deg,                 "sim/graphics/view/field_of_view_deg"))
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

    /* X-FlightServer sub-menu & its items */
    ctx->data.xfsr.plugin_id = XPLMFindPluginBySignature("ivao.xivap");
    if (XPLM_NO_PLUGIN_ID != ctx->data.xfsr.plugin_id)
    {
        if (append_menu_item("X-IvAp", &ctx->items.xflts1.sm_ic,
                             MENUITEM_XFLTS1_SM_IC, ctx->id))
        {
            goto fail;
        }
        if (create_menu("IvAp", ctx, &ctx->items.xflts1.sm_id,
                        &menu_handler, ctx->id, ctx->items.xflts1.sm_ic.id))
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
        xflts1_items[] =
        {
            {  "Enable", MENUITEM_XFLTS1_NABLE, &ctx->items.xflts1.nable, },
            { "Disable", MENUITEM_XFLTS1_DSBLE, &ctx->items.xflts1.dsble, },
            {      NULL,                     0,                     NULL, },
        };
        for (int i = 0; xflts1_items[i].name; i++)
        {
            if (append_menu_item(xflts1_items[i].name,
                                 xflts1_items[i].cx,
                                 xflts1_items[i].item_mivalue,
                                 ctx->items.xflts1.sm_id))
            {
                goto fail;
            }
        }
    }

    /* failure reset helper */
    if (append_menu_item("R2-D2 does maintenance", &ctx->items.reset_allsys,
                         MENUITEM_RESET_ALLSYS, ctx->id))
    {
        goto fail;
    }
    else
    {
        XPLMAppendMenuSeparator(ctx->id);
    }
    if ((ctx->data.reset.all_sys_op = XPLMFindCommand("sim/operation/fix_all_systems")) == NULL)
    {
        goto fail;
    }
    else
    {
        XPLMAppendMenuSeparator(ctx->id);
    }

    /* toggle: speed boost on/off */
    if (append_menu_item("", &ctx->items.dummy_item_0,
                         MENUITEM_NOTHING_TODO, ctx->id))
    {
        goto fail;
    }
//  if (append_menu_item("Tachyon Enhancement", &ctx->items.speedbooster,
//                       MENUITEM_SPEEDBOOSTER,  ctx->id))
//  {
//      goto fail;
//  }
    if (append_menu_item("Disable cloud draw", &ctx->items.cloud_killer,
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

    /* speak local weather */
    if (append_menu_item("Speak weather", &ctx->items.speakweather,
                         MENUITEM_SPEAKWEATHER, ctx->id))
    {
        goto fail;
    }
    if (get_dataref(&ctx->data.speakweather.baro_sl, "sim/weather/barometer_sealevel_inhg") ||
        get_dataref(&ctx->data.speakweather.wind_dt, "sim/weather/wind_direction_degt[0]" ) ||
        get_dataref(&ctx->data.speakweather.wind_sd, "sim/weather/wind_speed_kt[0]"       ) ||
        get_dataref(&ctx->data.speakweather.temp_dc, "sim/weather/temperature_ambient_c"  ) ||
        get_dataref(&ctx->data.speakweather.temp_dp, "sim/weather/dewpoi_sealevel_c"      ))
    {
        goto fail;
    }

    /* all good */
    return ctx;

fail:
    nvp_menu_close((void**)&ctx);
    return NULL;
}

static inline XPLMDataRef get_dataref2(const char *name)
{
    XPLMDataRef ref = XPLMFindDataRef(name);
    if (ref == NULL)
    {
        ndt_log("navP [debug]: coulnd't find dataref \"%s\"\n", name);
    }
    return ref;
}

int nvp_menu_setup(void *_menu_context)
{
    menu_context *ctx = _menu_context;
    if (ctx && !ctx->setupdone)
    {
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
        ctx->data.speedbooster.dr_kill_2d = get_dataref2("sim/private/controls/clouds/kill_2d"             );
        ctx->data.speedbooster.dr_kill_3d = get_dataref2("sim/private/controls/clouds/kill_3d"             );
        ctx->data.speedbooster.dr_skpdraw = get_dataref2("sim/private/controls/clouds/skip_draw"           );
        ctx->data.speedbooster.dr_k3drain = get_dataref2("sim/private/controls/rain/kill_3d_rain"          );
        ctx->data.speedbooster.dr_frstr3d = get_dataref2("sim/private/controls/clouds/first_res_3d"        );
        ctx->data.speedbooster.dr_lastr3d = get_dataref2("sim/private/controls/clouds/last_res_3d"         );
        ctx->data.speedbooster.dr_plotrad = get_dataref2("sim/private/controls/clouds/plot_radius"         );
        ctx->data.speedbooster.dr_shadrad = get_dataref2("sim/private/controls/clouds/shad_radius"         );
        ctx->data.speedbooster.dr_limitfr = get_dataref2("sim/private/controls/clouds/limit_far"           );
        ctx->data.speedbooster.dr_difgain = get_dataref2("sim/private/controls/clouds/diffuse_gain"        );
        ctx->data.speedbooster.dr_ovrdctl = get_dataref2("sim/private/controls/clouds/overdraw_control"    );
//      ctx->data.speedbooster.dr_use_csm = get_dataref2("sim/private/controls/caps/use_csm"               );
        ctx->data.speedbooster.dr_disprep = get_dataref2("sim/private/controls/perf/disable_shadow_prep"   );
        ctx->data.speedbooster.dr_disrcam = get_dataref2("sim/private/controls/perf/disable_reflection_cam");
        ctx->data.speedbooster.dr_x_i_ver = get_dataref2("sim/version/xplane_internal_version"             );
        ctx->data.speedbooster.dr_use_bmp = get_dataref2("sim/private/controls/reno/use_bump_maps"         );
        ctx->data.speedbooster.dr_det_tex = get_dataref2("sim/private/controls/reno/use_detail_textures"   );
        if (XPLM_NO_PLUGIN_ID != (skyMaxxPro = XPLMFindPluginBySignature("SilverLiningV2.Clouds")) ||
            XPLM_NO_PLUGIN_ID != (skyMaxxPro = XPLMFindPluginBySignature("SilverLiningV3.Clouds")) ||
            XPLM_NO_PLUGIN_ID != (skyMaxxPro = XPLMFindPluginBySignature("SilverLiningV4.Clouds")))
        {
            if (XPLMIsPluginEnabled(skyMaxxPro))
            {
                menu_rm_item(ctx,
                             ctx->items.cloud_killer.id);
                ctx->data.speedbooster.dr_kill_2d = NULL;
                ctx->data.speedbooster.dr_kill_3d = NULL;
                ctx->data.speedbooster.dr_skpdraw = NULL;
                ctx->data.speedbooster.dr_k3drain = NULL;
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
        if (ctx->data.speedbooster.dr_k3drain)
        {
            XPLMSetDataf(ctx->data.speedbooster.dr_k3drain, 0.00f);
        }
        if (ctx->data.speedbooster.dr_frstr3d)
        {
            ctx->data.speedbooster.df_frstr3d = XPLMGetDataf(ctx->data.speedbooster.dr_frstr3d);
        }
        if (ctx->data.speedbooster.dr_lastr3d)
        {
            ctx->data.speedbooster.df_lastr3d = XPLMGetDataf(ctx->data.speedbooster.dr_lastr3d);
        }
        if (ctx->data.speedbooster.dr_plotrad)
        {
            ctx->data.speedbooster.df_plotrad = XPLMGetDataf(ctx->data.speedbooster.dr_plotrad);
        }
        if (ctx->data.speedbooster.dr_shadrad)
        {
            ctx->data.speedbooster.df_shadrad = XPLMGetDataf(ctx->data.speedbooster.dr_shadrad);
        }
        if (ctx->data.speedbooster.dr_limitfr)
        {
            ctx->data.speedbooster.df_limitfr = XPLMGetDataf(ctx->data.speedbooster.dr_limitfr);
        }
        if (ctx->data.speedbooster.dr_difgain)
        {
            ctx->data.speedbooster.df_difgain = XPLMGetDataf(ctx->data.speedbooster.dr_difgain);
        }
        if (ctx->data.speedbooster.dr_ovrdctl)
        {
            ctx->data.speedbooster.df_ovrdctl = XPLMGetDataf(ctx->data.speedbooster.dr_ovrdctl);
        }
//      if (ctx->data.speedbooster.dr_use_csm)
//      {
//          ctx->data.speedbooster.df_use_csm = XPLMGetDataf(ctx->data.speedbooster.dr_use_csm);
//      }
        if (ctx->data.speedbooster.dr_disprep)
        {
            ctx->data.speedbooster.df_disprep = XPLMGetDataf(ctx->data.speedbooster.dr_disprep);
        }
        if (ctx->data.speedbooster.dr_disrcam)
        {
            ctx->data.speedbooster.df_disrcam = XPLMGetDataf(ctx->data.speedbooster.dr_disrcam);
        }
        if (ctx->data.speedbooster.dr_use_bmp)
        {
            ctx->data.speedbooster.df_use_bmp = XPLMGetDatai(ctx->data.speedbooster.dr_use_bmp);
        }
        if (ctx->data.speedbooster.dr_det_tex)
        {
            ctx->data.speedbooster.df_det_tex = XPLMGetDatai(ctx->data.speedbooster.dr_det_tex);
        }
        XPLMCheckMenuItem(ctx->id, ctx->items.cloud_killer.id, xplm_Menu_NoCheck);
//      XPLMCheckMenuItem(ctx->id, ctx->items.speedbooster.id, xplm_Menu_NoCheck);
        ndt_log          (   "navP [info]: clouds on, Tachyon Enhancement off\n");

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
        ctx->data.refuel_dialg.dialog_id = XPCreateWidget(inLT, inTP, inRT, inBM, 0,
                                                          "Fuel & Payload", 1, NULL,
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
        inBM = (((REFUEL_DIALG_DEFH - REFUEL_DIALG_TBAR - REFUEL_DIALG_CAPH - 15)));
        inTP = (((REFUEL_DIALG_CAPH) - 1 + (inBM)));
        inRT = (((REFUEL_DIALG_CAPW) - 1 + (inLT)));
        ctx->data.refuel_dialg.f_txtl_id = XPCreateWidget(inLT, inTP, inRT, inBM, 1, "", 0,
                                                          ctx->data.refuel_dialg.dialog_id,
                                                          xpWidgetClass_Caption);
        if (!ctx->data.refuel_dialg.f_txtl_id)
        {
            ndt_log("navP [warning]: could not create label for fuel entry\n");
            menu_rm_item(ctx, ctx->items.refuel_dialg.id);
            ctx->setupdone = -1; return -1;
        }
        XPSetWidgetProperty(ctx->data.refuel_dialg.f_txtl_id, xpProperty_CaptionLit, 1);
        inLT = (REFUEL_DIALG_CAPW + inLT * 2); // right next to the field caption
        inTP = (REFUEL_DIALG_TXTH) - 1 + (inBM = inBM - 2); // widget mismatch
        inRT = (REFUEL_DIALG_TXTW) - 1 + (inLT);
        ctx->data.refuel_dialg.f_txtf_id = XPCreateWidget(inLT, inTP, inRT, inBM, 1, "", 0,
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
        inBM = (((REFUEL_DIALG_DEFH - REFUEL_DIALG_TBAR - REFUEL_DIALG_CAPH - 45)));
        inTP = (((REFUEL_DIALG_CAPH) - 1 + (inBM)));
        inRT = (((REFUEL_DIALG_CAPW) - 1 + (inLT)));
        ctx->data.refuel_dialg.p_txtl_id = XPCreateWidget(inLT, inTP, inRT, inBM, 1, "", 0,
                                                          ctx->data.refuel_dialg.dialog_id,
                                                          xpWidgetClass_Caption);
        if (!ctx->data.refuel_dialg.p_txtl_id)
        {
            ndt_log("navP [warning]: could not create label for payload entry\n");
            menu_rm_item(ctx, ctx->items.refuel_dialg.id);
            ctx->setupdone = -1; return -1;
        }
        XPSetWidgetProperty(ctx->data.refuel_dialg.p_txtl_id, xpProperty_CaptionLit, 1);
        inLT = (REFUEL_DIALG_CAPW + inLT * 2); // right next to the field caption
        inTP = (REFUEL_DIALG_TXTH) - 1 + (inBM = inBM - 2); // widget mismatch
        inRT = (REFUEL_DIALG_TXTW) - 1 + (inLT);
        ctx->data.refuel_dialg.p_txtf_id = XPCreateWidget(inLT, inTP, inRT, inBM, 1, "", 0,
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

#define SPEEDBOOSTER_DEFAULTV(_function, _dataref, _default) \
    { if (ctx->data.speedbooster._dataref) { _function(ctx->data.speedbooster._dataref, ctx->data.speedbooster._default); } }
#define SPEEDBOOSTER_SETVALUE(_function, _dataref, _value) \
    { if (ctx->data.speedbooster._dataref) { _function(ctx->data.speedbooster._dataref,                        (_value)); } }
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

    /*
     * reset default art control values when closing the menu so we'll get the
     * initial default values if we close and re-open the menu within the same
     * X-Plane session (e.g. when reloading plugins w/Tachyon Enhancement on)
     */
    SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_frstr3d, df_frstr3d);
    SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_lastr3d, df_lastr3d);
    SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_plotrad, df_plotrad);
    SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_shadrad, df_shadrad);
    SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_limitfr, df_limitfr);
    SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_difgain, df_difgain);
    SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_ovrdctl, df_ovrdctl);
//  SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_use_csm, df_use_csm);
    SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_disprep, df_disprep);
    SPEEDBOOSTER_DEFAULTV(XPLMSetDataf, dr_disrcam, df_disrcam);
    ndt_log(    "navP [info]: disabling Tachyon Enhancement\n");
    SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_kill_2d,      0.00f);
    SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_kill_3d,      0.00f);
    SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_skpdraw,      0.00f);
    SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_k3drain,      0.00f);
    ndt_log(          "navP [info]: enabling default clouds\n");
    if (NULL != ctx->data.speedbooster.dr_x_i_ver) // lazy XP11+ detection
    {
//      SPEEDBOOSTER_DEFAULTV(XPLMSetDatai, dr_use_bmp, df_use_bmp);
//      SPEEDBOOSTER_DEFAULTV(XPLMSetDatai, dr_det_tex, df_det_tex);
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
        MENUITEM_UNDEF_VAL(ctx->items.dummy_item_0);
        MENUITEM_CHECK_IDX(ctx->items.dummy_item_0.id);
//      MENUITEM_UNDEF_VAL(ctx->items.speedbooster);
//      MENUITEM_CHECK_IDX(ctx->items.speedbooster.id);
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

static void wb_handler(menu_context *ctx)
{
    if (XPIsWidgetVisible(ctx->data.refuel_dialg.dialog_id))
    {
        return;
    }
    if ((ctx->data.refuel_dialg.ic = acf_type_info_update()) == NULL)
    {
        ndt_log("navP [error]: wb_handler: acf_type_info_update() failed");
        XPLMSpeakString("fuel and load dialog failure");
        return;
    }
    float fmax, fuel, load, zfwt, oewt; int e;
    if ((e = acf_type_info_acf_ctx_init()))
    {
        if (e == EAGAIN)
        {
            XPLMSpeakString("please try again later");
            return;
        }
        ndt_log("navP [error]: wb_handler: acf_type_info_acf_ctx_init() failed (%d)", e);
        XPLMSpeakString("fuel and load dialog failure");
        return;
    }
    if ((e = acf_type_fmax_get(ctx->data.refuel_dialg.ic, &fmax)))
    {
        ndt_log("navP [error]: wb_handler: acf_type_fmax_get() failed (%d)", e);
        XPLMSpeakString("fuel and load dialog failure");
        return;
    }
    if ((e = acf_type_fuel_get(ctx->data.refuel_dialg.ic, &fuel)))
    {
        ndt_log("navP [error]: wb_handler: acf_type_fuel_get() failed (%d)", e);
        XPLMSpeakString("fuel and load dialog failure");
        return;
    }
    if ((e = acf_type_load_get(ctx->data.refuel_dialg.ic, &load)))
    {
        ndt_log("navP [error]: wb_handler: acf_type_load_get() failed (%d)", e);
        XPLMSpeakString("fuel and load dialog failure");
        return;
    }
    if ((e = acf_type_zfwt_get(ctx->data.refuel_dialg.ic, &zfwt)))
    {
        ndt_log("navP [error]: wb_handler: acf_type_zfwt_get() failed (%d)", e);
        XPLMSpeakString("fuel and load dialog failure");
        return;
    }
    if ((e = acf_type_oewt_get(ctx->data.refuel_dialg.ic, &oewt)))
    {
        ndt_log("navP [error]: wb_handler: acf_type_grwt_get() failed (%d)", e);
        XPLMSpeakString("fuel and load dialog failure");
        return;
    }
    int g[4]; mainw_center(g, REFUEL_DIALG_DEFW, REFUEL_DIALG_DEFH);
    {
        // set boarding and fueling rates based on resp. OEW and fuel capacity
        // so airliners can: fuel in ~12.5 minutes, deboard in about 5 minutes
        // 40T for 16,000kgs (~150pax) in ~5 minutes: 16000/40/300 = 1.33OEW/s
        // sanitize to a minimum to 20.0kgs per second so it also works for GA
        if ((ctx->data.refuel_dialg.load_rate_kg_s = (oewt *  4.0f / 3000.0f)) < LOAD_MINIMUM_RATE)
        {
            (ctx->data.refuel_dialg.load_rate_kg_s = LOAD_MINIMUM_RATE);
        }
        if ((ctx->data.refuel_dialg.fuel_rate_kg_s = (fmax / 750.000f)) < LOAD_MINIMUM_RATE)
        {
            (ctx->data.refuel_dialg.fuel_rate_kg_s = LOAD_MINIMUM_RATE);
        }
    }
    switch (ctx->data.refuel_dialg.ic->ac_type)
    {
        case ACF_TYP_A319_TL:
        case ACF_TYP_A321_TL:
        case ACF_TYP_A346_TL:
        case ACF_TYP_A350_FF:
        case ACF_TYP_A320_FF:
        case ACF_TYP_B737_XG:
            XPShowWidget(ctx->data.refuel_dialg.f_txtl_id);
            XPShowWidget(ctx->data.refuel_dialg.f_txtf_id);
            XPShowWidget(ctx->data.refuel_dialg.p_txtl_id);
            XPShowWidget(ctx->data.refuel_dialg.p_txtf_id);
            ctx->data.refuel_dialg.payload_is_zfw = 1;
            break;
        default: // 7,000kg: aircraft *starting* above ICAO category "L" are ZFW
            ctx->data.refuel_dialg.payload_is_zfw = (7000.0f < oewt);
            XPShowWidget(ctx->data.refuel_dialg.f_txtl_id);
            XPShowWidget(ctx->data.refuel_dialg.f_txtf_id);
            XPShowWidget(ctx->data.refuel_dialg.p_txtl_id);
            XPShowWidget(ctx->data.refuel_dialg.p_txtf_id);
            break;
    }
    if (ctx->data.refuel_dialg.rfc)
    {
        XPLMUnregisterFlightLoopCallback(ctx->data.refuel_dialg.rfc, ctx);
        ctx->data.refuel_dialg.rfc = NULL;
    }
    if (ctx->data.refuel_dialg.payload_is_zfw)
    {
        XPSetWidgetDescriptor(ctx->data.refuel_dialg.f_txtl_id, "(KG) FUEL");
        XPSetWidgetDescriptor(ctx->data.refuel_dialg.p_txtl_id, "(KG) ZFW");
        AUTOSNPRINTF(ctx->data.refuel_dialg.fueltot_str, "%.0f", fuel);
        AUTOSNPRINTF(ctx->data.refuel_dialg.payload_str, "%.0f", zfwt);
    }
    else
    {
        XPSetWidgetDescriptor(ctx->data.refuel_dialg.p_txtl_id, "(KG) PAYLOAD");
        XPSetWidgetDescriptor(ctx->data.refuel_dialg.f_txtl_id, "(KG) FUEL");
        AUTOSNPRINTF(ctx->data.refuel_dialg.fueltot_str, "%.0f", fuel);
        AUTOSNPRINTF(ctx->data.refuel_dialg.payload_str, "%.0f", load);
    }
    XPSetWidgetDescriptor(ctx->data.refuel_dialg.f_txtf_id, ctx->data.refuel_dialg.fueltot_str);
    XPSetWidgetDescriptor(ctx->data.refuel_dialg.p_txtf_id, ctx->data.refuel_dialg.payload_str);
    XPSetWidgetGeometry     (ctx->data.refuel_dialg.dialog_id, g[0], g[1], g[2], g[3]);
    XPShowWidget            (ctx->data.refuel_dialg.dialog_id);
    XPBringRootWidgetToFront(ctx->data.refuel_dialg.dialog_id);
    return;
}

static void menu_handler(void *inMenuRef, void *inItemRef)
{
    menu_context *ctx = inMenuRef;
    item_context *itx = inItemRef;

    if (itx->mivalue == MENUITEM_NOTHING_TODO)
    {
        return;
    }

    if (itx->mivalue == MENUITEM_RESET_ALLSYS)
    {
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("com.simcoders.rep") &&
            XPLMIsPluginEnabled(XPLMFindPluginBySignature("com.simcoders.rep")) &&
            XPLMFindCommand("simcoders/rep/systems/fix_all"))
        {
            XPLMCommandOnce(XPLMFindCommand("simcoders/rep/systems/fix_all"));
            XPLMSpeakString("REP failures fixed");
            return;
        }
        XPLMCommandOnce(ctx->data.reset.all_sys_op);
        XPLMSpeakString("default failures fixed");
        return;
    }

    if (itx->mivalue == MENUITEM_REFUEL_DIALG)
    {
        return wb_handler(ctx);
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
        if (3 <= (int)round(wind_sd))
        {
            snprintf(wind, sizeof(wind), "Wind %03d at %d.",
                     (int)wind_dt, (int)round(wind_sd));
        }
        else
        {
            /*
             * FAA ORDER JO 7110.65W
             * Describe the wind as calm when the
             * wind velocity is less than 3 knots.
             */
            snprintf(wind, sizeof(wind), "%s", "Wind calm.");
        }
        snprintf(temp, sizeof(temp), "Temperature %d, dew point %d.",
                 (int)round(temp_dc), (int)round(temp_dp));
        snprintf(weather, sizeof(weather), "%s %s %s",
                 baro, wind, temp);
        XPLMSpeakString(string4speak(weather, sizeof(weather), 0));
        return;
    }

    if (itx->mivalue == MENUITEM_SPEEDBOOSTER || itx->mivalue == MENUITEM_CLOUD_KILLER)
    {
        XPLMMenuCheck speedb = xplm_Menu_Checked, cloudk = xplm_Menu_Checked;
//      XPLMCheckMenuItemState(ctx->id, ctx->items.speedbooster.id, &speedb);
        XPLMCheckMenuItemState(ctx->id, ctx->items.cloud_killer.id, &cloudk);
        int speedbooster_enabled = speedb == xplm_Menu_Checked;
        int cloud_killer_enabled = cloudk == xplm_Menu_Checked;
        if (itx->mivalue == MENUITEM_SPEEDBOOSTER)
        {
            if ((speedbooster_enabled = !speedbooster_enabled))
            {
                (cloud_killer_enabled = 0); // toggle
            }
        }
        if (itx->mivalue == MENUITEM_CLOUD_KILLER)
        {
            if ((cloud_killer_enabled = !cloud_killer_enabled))
            {
                (speedbooster_enabled = 0); // toggle
            }
        }
        /*if (speedbooster_enabled)
        {
//          XPLMCheckMenuItem(ctx->id, ctx->items.speedbooster.id, xplm_Menu_Checked);
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
        }
        else
        {
//          XPLMCheckMenuItem(ctx->id, ctx->items.speedbooster.id, xplm_Menu_NoCheck);
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
        }*/
        if (cloud_killer_enabled)
        {
            XPLMCheckMenuItem(ctx->id, ctx->items.cloud_killer.id, xplm_Menu_Checked);
            SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_kill_2d, 1.00f);
            SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_kill_3d, 1.00f);
            SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_skpdraw, 1.00f);
            SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_k3drain, 1.00f);
            ndt_log(    "navP [info]: disabling default clouds\n");
            if (NULL != ctx->data.speedbooster.dr_x_i_ver) // lazy XP11+ detection
            {
//              SPEEDBOOSTER_SETVALUE(XPLMSetDatai, dr_use_bmp, 0);
//              SPEEDBOOSTER_SETVALUE(XPLMSetDatai, dr_det_tex, 0);
            }
        }
        else
        {
            XPLMCheckMenuItem(ctx->id, ctx->items.cloud_killer.id, xplm_Menu_NoCheck);
            SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_kill_2d, 0.00f);
            SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_kill_3d, 0.00f);
            SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_skpdraw, 0.00f);
            SPEEDBOOSTER_SETVALUE(XPLMSetDataf, dr_k3drain, 0.00f);
            ndt_log(     "navP [info]: enabling default clouds\n");
            if (NULL != ctx->data.speedbooster.dr_x_i_ver) // lazy XP11+ detection
            {
//              SPEEDBOOSTER_DEFAULTV(XPLMSetDatai, dr_use_bmp, df_use_bmp);
//              SPEEDBOOSTER_DEFAULTV(XPLMSetDatai, dr_det_tex, df_det_tex);
            }
        }
        return;
    }

    if (itx->mivalue >= MENUITEM_VOLUME_PRST0 &&
        itx->mivalue <= MENUITEM_VOLUME_PRST5)
    {
        acf_info_context *ic = acf_type_info_update();
        acf_volume_context *vc = acf_volume_ctx_get();
        if (ic == NULL)
        {
            XPLMSpeakString("Volume error 1");
            return;
        }
        if (vc == NULL)
        {
            XPLMSpeakString("Volume error 2");
            return;
        }
        switch (itx->mivalue)
        {
            case MENUITEM_VOLUME_PRST0:
                acf_volume_set(vc, 0.0000f, ic->ac_type);
                XPLMSpeakString("Volume muted");
                return;
            case MENUITEM_VOLUME_PRST1:
                acf_volume_set(vc, 0.0625f, ic->ac_type);
                XPLMSpeakString("Volume low");
                return;
            case MENUITEM_VOLUME_PRST2:
                acf_volume_set(vc, 0.2500f, ic->ac_type);
                XPLMSpeakString("Volume 25");
                return;
            case MENUITEM_VOLUME_PRST3:
                acf_volume_set(vc, 0.5000f, ic->ac_type);
                XPLMSpeakString("Volume 50");
                return;
            case MENUITEM_VOLUME_PRST4:
                acf_volume_set(vc, 0.7500f, ic->ac_type);
                XPLMSpeakString("Volume 75");
                return;
            case MENUITEM_VOLUME_PRST5:
                acf_volume_set(vc, 1.0000f, ic->ac_type);
                XPLMSpeakString("Volume 100");
                return;
            default:
                XPLMSpeakString("Volume error 3");
                return;
        }
        return;
    }

    if (itx->mivalue >= MENUITEM_FILDOV_45DEG &&
        itx->mivalue <= MENUITEM_FILDOV_75DEG)
    {
        if (XPLMGetDatai(ctx->data.fildov_prsts.fildov_non))
        {
            XPLMSpeakString("non proprotional F O V");
            return;
        }
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

    if (itx->mivalue == MENUITEM_XFLTS1_NABLE)
    {
        if (XPLMIsPluginEnabled(ctx->data.xfsr.plugin_id) == 0)
        {
            XPLMEnablePlugin   (ctx->data.xfsr.plugin_id);
            XPLMSpeakString    ("X IvAp enabled");
        }
        return;
    }
    if (itx->mivalue == MENUITEM_XFLTS1_DSBLE)
    {
        if (XPLMIsPluginEnabled(ctx->data.xfsr.plugin_id) != 0)
        {
            XPLMDisablePlugin  (ctx->data.xfsr.plugin_id);
            XPLMSpeakString    ("X IvAp disabled");
        }
        return;
    }

//  if (itx->mivalue >= MENUITEM_WEATHR_REALW &&
//      itx->mivalue <= MENUITEM_WEATHR_CAVOK)
//  {
//      switch (itx->mivalue)
//      {
//          case MENUITEM_WEATHR_REALW:
//              XPLMSetDatai   (ctx->data.weather.real_use_bool, 1);
//              XPLMSetDatai   (ctx->data.weather.real_download, 1);
//              XPLMCommandOnce(ctx->data.weather.weather_ld);
//              XPLMSpeakString("Real weather enabled\n");
//              break;
//          case MENUITEM_WEATHR_CAVOK:
//              XPLMSetDatai   (ctx->data.weather.real_use_bool,        0);
//              XPLMSetDatai   (ctx->data.weather.real_download,        0);
//              XPLMSetDatai   (ctx->data.weather.cloud_type_00,        0); // clear
//              XPLMSetDatai   (ctx->data.weather.cloud_type_01,        0); // clear
//              XPLMSetDatai   (ctx->data.weather.cloud_type_02,        0); // clear
//              XPLMSetDataf   (ctx->data.weather.visibility_me, 40000.0f); // 40km
//              XPLMSetDataf   (ctx->data.weather.precp_percent,     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.thndr_percent,     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.turbl_percent,     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.windspd[0][0],     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.windspd[1][0],     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.windspd[2][0],     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.windspd[0][1],     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.windspd[1][1],     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.windspd[2][1],     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.turbulence[0],     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.turbulence[1],     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.turbulence[2],     0.0f); // none
//              XPLMSetDataf   (ctx->data.weather.runway_frictn,     0.0f); // dry
//              XPLMCommandOnce(ctx->data.weather.weather_rg);
//              XPLMSpeakString("Real weather disabled\n");
//              break;
//          default:
//              return;
//      }
//      return;
//  }
}

void nvp_menu_ckill(void *_menu_context, int set_state)
{
    menu_context *ctx = _menu_context;
    if (ctx)
    {
        XPLMMenuCheck state = xplm_Menu_Unchecked;
        XPLMCheckMenuItemState(ctx->id, ctx->items.cloud_killer.id, &state);
        if (state != xplm_Menu_Unchecked && state != set_state)
        {
            return menu_handler(ctx, &ctx->items.cloud_killer);
        }
    }
    return;
}

void nvp_menu_tachy(void *_menu_context, int set_state)
{
    menu_context *ctx = _menu_context;
    if (ctx)
    {
        XPLMMenuCheck state = xplm_Menu_Checked;
//      XPLMCheckMenuItemState(ctx->id, ctx->items.speedbooster.id, &state);
        if (state != set_state)
        {
//          return menu_handler(ctx, &ctx->items.speedbooster);
        }
    }
    return;
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
        menu_context *ctx = (void*)XPGetWidgetProperty((void*)inParam1, xpProperty_Refcon, NULL);
        float target_fuel_weight, target_load_weight, target_zero_weight; char desc_buf[9] = "";
        XPGetWidgetDescriptor(ctx->data.refuel_dialg.f_txtf_id, desc_buf, sizeof(desc_buf) - 1);
        if (sscanf(desc_buf, "%f", &target_fuel_weight) == 1)
        {
            float current_fuel; acf_type_fuel_get(ctx->data.refuel_dialg.ic, &current_fuel);
            float maximum_fuel; acf_type_fmax_get(ctx->data.refuel_dialg.ic, &maximum_fuel);
            float minim_fuel = maximum_fuel / 8.0f; minim_fuel = fminf(minim_fuel, 4000.0f);
            if (fabsf(target_fuel_weight - current_fuel) > LOAD_MINIMUM_DIFF)
            {
                if (target_fuel_weight > current_fuel)
                {
                    ctx->data.refuel_dialg.refuel_started = 0;
                    ctx->data.refuel_dialg.adjust_fuel = NVP_MENU_INCR;
                }
                else
                {
                    ctx->data.refuel_dialg.refuel_started = 0;
                    ctx->data.refuel_dialg.adjust_fuel = NVP_MENU_DECR;
                }
                ctx->data.refuel_dialg.fuel_target_kg = target_fuel_weight;
            }
            else
            {
                ctx->data.refuel_dialg.adjust_fuel = NVP_MENU_DONE;
            }
            if (ctx->data.refuel_dialg.fuel_target_kg > maximum_fuel)
            {
                ctx->data.refuel_dialg.fuel_target_kg = maximum_fuel;
            }
            if (ctx->data.refuel_dialg.fuel_target_kg < minim_fuel)
            {
                ctx->data.refuel_dialg.fuel_target_kg = minim_fuel;
            }
        }
        else
        {
            XPLMSpeakString("unable to parse fuel weight");
            return 1;
        }
        XPGetWidgetDescriptor(ctx->data.refuel_dialg.p_txtf_id, desc_buf, sizeof(desc_buf) - 1);
        if (sscanf(desc_buf, "%f", &target_load_weight) == 1 &&
            sscanf(desc_buf, "%f", &target_zero_weight) == 1)
        {
            float current_load; acf_type_load_get(ctx->data.refuel_dialg.ic, &current_load);
            float current_zfwt; acf_type_zfwt_get(ctx->data.refuel_dialg.ic, &current_zfwt);
            if (ctx->data.refuel_dialg.ic->ac_type == ACF_TYP_A319_TL ||
                ctx->data.refuel_dialg.ic->ac_type == ACF_TYP_A321_TL ||
                ctx->data.refuel_dialg.ic->ac_type == ACF_TYP_A346_TL)
            {
                if (fabsf(target_zero_weight - current_zfwt) > LOAD_MINIMUM_DIFF)
                {
                    if (target_zero_weight > current_zfwt)
                    {
                        ctx->data.refuel_dialg.adjust_load = NVP_MENU_INCR;
                    }
                    else
                    {
                        ctx->data.refuel_dialg.adjust_load = NVP_MENU_DECR;
                    }
                    ctx->data.refuel_dialg.load_target_kg = target_zero_weight;
                }
                else
                {
                    ctx->data.refuel_dialg.adjust_load = NVP_MENU_DONE;
                }
            }
            else if (ctx->data.refuel_dialg.ic->ac_type == ACF_TYP_TBM9_HS)
            {
                if (fabsf(target_load_weight - current_load) > LOAD_MINIMUM_DIFF)
                {
                    ndt_log("navP [info]: use TBM-900's custom load manager\n");
                }
                ctx->data.refuel_dialg.load_target_kg = current_load;
                ctx->data.refuel_dialg.adjust_load = NVP_MENU_DONE;
            }
            else
            {
                if (ctx->data.refuel_dialg.payload_is_zfw)
                {
                    float diff = target_zero_weight - current_zfwt;
                    target_load_weight = current_load + diff;
                }
                if (fabsf(target_load_weight - current_load) > LOAD_MINIMUM_DIFF)
                {
                    if (target_load_weight > current_load)
                    {
                        ctx->data.refuel_dialg.brding_started = 0;
                        ctx->data.refuel_dialg.load_rate_kg_s /= 2.0f;
                        ctx->data.refuel_dialg.adjust_load = NVP_MENU_INCR;
                    }
                    else
                    {
                        ctx->data.refuel_dialg.brding_started = 0;
                        ctx->data.refuel_dialg.adjust_load = NVP_MENU_DECR;
                    }
                    ctx->data.refuel_dialg.load_target_kg = target_load_weight;
                }
                else
                {
                    ctx->data.refuel_dialg.adjust_load = NVP_MENU_DONE;
                }
                if (ctx->data.refuel_dialg.load_target_kg < LOAD_MINIMUM_DIFF)
                {
                    ctx->data.refuel_dialg.load_target_kg = 0.0f;
                }
            }
        }
        else
        {
            XPLMSpeakString("unable to parse load weight");
            return 1;
        }
        if (ctx->data.refuel_dialg.rfc)
        {
            XPLMUnregisterFlightLoopCallback(ctx->data.refuel_dialg.rfc, ctx);
        }
        if (ctx->data.refuel_dialg.adjust_fuel != NVP_MENU_DONE ||
            ctx->data.refuel_dialg.adjust_load != NVP_MENU_DONE)
        {
            if (acf_type_is_engine_running() != 0 &&
                ctx->data.refuel_dialg.adjust_load != NVP_MENU_DONE)
            {
                XPLMSpeakString("cannot set payload in flight");
                XPHideWidget(inWidget);
                return 1;
            }
            for (int ii = 1; ii <= ctx->data.refuel_dialg.ic->fuel.tanks.count; ii++)
            {
                if (ii == ctx->data.refuel_dialg.ic->fuel.tanks.count)
                {
                    ndt_log("%.3f, capacity: %.0fkgs (%.0flbs)\n",
                            ctx->data.refuel_dialg.ic->fuel.tanks.rat[ii-1],
                            ctx->data.refuel_dialg.ic->fuel.tanks.max_kg,
                            ctx->data.refuel_dialg.ic->fuel.tanks.max_lb);
                }
                else
                {
                    if (ii == 1)
                    {
                        ndt_log("navP [info]: fuel tank count: %d, ratios: ", ctx->data.refuel_dialg.ic->fuel.tanks.count);
                    }
                    ndt_log("%.3f ", ctx->data.refuel_dialg.ic->fuel.tanks.rat[ii-1]);
                }
            }
            if (ctx->data.refuel_dialg.ic->ac_type == ACF_TYP_A319_TL ||
                ctx->data.refuel_dialg.ic->ac_type == ACF_TYP_A321_TL ||
                ctx->data.refuel_dialg.ic->ac_type == ACF_TYP_A346_TL)
            {
                if (XPIsWidgetVisible(inWidget))
                {
                    XPHideWidget(inWidget);
                }
                if (ctx->data.refuel_dialg.adjust_load != NVP_MENU_DONE)
                {
                    if (acf_type_zfwt_set(ctx->data.refuel_dialg.ic, &ctx->data.refuel_dialg.load_target_kg))
                    {
                        ndt_log("navP [error]: failed to set ZFW\n");
                        XPLMSpeakString("boarding failed");
                        return 1;
                    }
                    else
                    {
                        ndt_log("navP [info]: set ZFW (%.0f)\n", ctx->data.refuel_dialg.load_target_kg);
                    }
                }
                if (ctx->data.refuel_dialg.adjust_fuel != NVP_MENU_DONE)
                {
                    if (acf_type_fuel_set(ctx->data.refuel_dialg.ic, &ctx->data.refuel_dialg.fuel_target_kg))
                    {
                        ndt_log("navP [error]: failed to set fuel\n");
                        XPLMSpeakString("re-fueling failed");
                        return 1;
                    }
                    ndt_log("navP [info]: set fuel (%.0f)\n", ctx->data.refuel_dialg.fuel_target_kg);
                }
                XPLMSpeakString("fuel and load set");
                return 1;
            }
            if (ctx->data.refuel_dialg.ic->ac_type == ACF_TYP_A320_FF)
            {
                if (XPIsWidgetVisible(inWidget))
                {
                    XPHideWidget(inWidget);
                }
                if (acf_type_is_engine_running() == 0) // cold & dark
                {
                    if (ctx->data.refuel_dialg.adjust_load == NVP_MENU_DONE)
                    {
                        acf_type_zfwt_get(ctx->data.refuel_dialg.ic, &ctx->data.refuel_dialg.load_target_kg);
                    }
                    else
                    {
                        ctx->data.refuel_dialg.load_target_kg = target_zero_weight;
                    }
                    if (ctx->data.refuel_dialg.adjust_fuel == NVP_MENU_DONE)
                    {
                        acf_type_fuel_get(ctx->data.refuel_dialg.ic, &ctx->data.refuel_dialg.fuel_target_kg);
                    }
                    assert_context *ac = &ctx->data.refuel_dialg.ic->assert;
                    float zero_fuel_center_z_axis; int32_t weight_boarding = 1;
                    float takeoff_fuel_target_kgs, takeoff_load_target_kgs, oewt;
                    int twb = ac->api.ValueIdByName("Aircraft.TakeoffWeightBoarding");
                    int tcd = ac->api.ValueIdByName("Aircraft.TakeoffCenterDry");
                    int twd = ac->api.ValueIdByName("Aircraft.TakeoffWeightDry");
                    int tbf = ac->api.ValueIdByName("Aircraft.TakeoffBlockFuel");
                    if (twb <= 0 || tcd <= 0 || twd <= 0 || tbf <= 0)
                    {
                        ndt_log("navP [error]: fuel and load data failure\n");
                        XPLMSpeakString("fuel and load data failure");
                        return 1;
                    }
                    else
                    {
                        /*
                         * Zero-fuel center of gravity (super-simplified):
                         * * +25.00% MAC while empty (payload == 00,000kgs)
                         * * +35.00% MAC when loaded (payload == 20,000kgs)
                         */
                        acf_type_oewt_get(ctx->data.refuel_dialg.ic, &oewt);
                        float rq_payload = ctx->data.refuel_dialg.load_target_kg - oewt;
                        takeoff_fuel_target_kgs = ctx->data.refuel_dialg.fuel_target_kg;
                        takeoff_load_target_kgs = ctx->data.refuel_dialg.load_target_kg;
                        zero_fuel_center_z_axis = (25.0f + (10.0f * (rq_payload / 20000.0f)));
                    }
                    ac->api.ValueSet(tcd, &zero_fuel_center_z_axis);
                    ac->api.ValueSet(twd, &takeoff_load_target_kgs);
                    ac->api.ValueSet(tbf, &takeoff_fuel_target_kgs);
                    ac->api.ValueSet(twb, &weight_boarding); XPLMSpeakString("fuel and load set");
                    ndt_log("navP [info]: set block fuel (%.0f) ZFW (%.0f) and ZFWCG (%.1f)\n", takeoff_fuel_target_kgs, takeoff_load_target_kgs, zero_fuel_center_z_axis);
                    return 1;
                }
                else if (acf_type_fuel_set(ctx->data.refuel_dialg.ic, &ctx->data.refuel_dialg.fuel_target_kg))
                {
                    ndt_log("navP [error]: failed to set fuel\n");
                    XPLMSpeakString("re-fueling failed");
                    return 1;
                }
                ndt_log("navP [info]: set fuel (%.0f)\n", ctx->data.refuel_dialg.fuel_target_kg);
                XPLMSpeakString("re-fueling done");
                return 1;
            }
            else
            {
                ndt_log("navP [info]: loading/unloading: ");
            }
            if (ctx->data.refuel_dialg.adjust_fuel != NVP_MENU_DONE)
            {
                ndt_log("fuel %.0fkgs (%.0fkg/s)", ctx->data.refuel_dialg.fuel_target_kg, ctx->data.refuel_dialg.fuel_rate_kg_s);
                ndt_log(ctx->data.refuel_dialg.adjust_load ? ", " : "\n");
            }
            if (ctx->data.refuel_dialg.adjust_load != NVP_MENU_DONE)
            {
                ndt_log("payload %.0fkgs (%.0fkg/s)\n", ctx->data.refuel_dialg.load_target_kg, ctx->data.refuel_dialg.load_rate_kg_s);
            }
            if (ctx->data.refuel_dialg.ic->ac_type == ACF_TYP_A350_FF)
            {
                if (ctx->data.refuel_dialg.adjust_fuel != NVP_MENU_DONE)
                {
                    if (acf_type_fuel_set(ctx->data.refuel_dialg.ic, &ctx->data.refuel_dialg.fuel_target_kg))
                    {
                        ndt_log("navP [error]: failed to set fuel\n");
                        XPLMSpeakString("fueling failed");
                        return 1;
                    }
                    else
                    {
                        ndt_log("navP [info]: set fuel (%.0f)\n", ctx->data.refuel_dialg.fuel_target_kg);
                    }
                }
                if (ctx->data.refuel_dialg.adjust_load != NVP_MENU_DONE)
                {
                    if (acf_type_load_set(ctx->data.refuel_dialg.ic, &ctx->data.refuel_dialg.load_target_kg))
                    {
                        ndt_log("navP [error]: failed to set load\n");
                        XPLMSpeakString("boarding failed");
                        return 1;
                    }
                    else
                    {
                        ndt_log("navP [info]: set load (%.0f)\n", ctx->data.refuel_dialg.load_target_kg);
                    }
                }
                XPLMSpeakString("fuel and load set");
                XPHideWidget(inWidget);
                return 1;
            }
            XPLMRegisterFlightLoopCallback((ctx->data.refuel_dialg.rfc = &refuel_hdlr1), 4.0f, ctx);
        }
        XPHideWidget(inWidget);
        return 1;
    }
    return 0;
}

static float refuel_hdlr1(float inElapsedSinceLastCall,
                          float inElapsedTimeSinceLastFlightLoop,
                          int   inCounter,
                          void *inRefcon)
{
    menu_context *ctx = inRefcon; float fuel, load;
    acf_type_fuel_get(ctx->data.refuel_dialg.ic, &fuel);
    acf_type_load_get(ctx->data.refuel_dialg.ic, &load);
    if (ctx->data.refuel_dialg.adjust_fuel == NVP_MENU_INCR)
    {
        if (ctx->data.refuel_dialg.refuel_started)
        {
            if (fuel <= ctx->data.refuel_dialg.last_fuel_amnt) // no increase
            {
                XPLMSpeakString("fueling failed");
                ndt_log("navP [error]: refuel failed\n");
                ctx->data.refuel_dialg.adjust_fuel = NVP_MENU_DONE;
                goto adjust_load;
            }
            ctx->data.refuel_dialg.last_fuel_amnt = fuel;
        }
        else
        {
            ctx->data.refuel_dialg.last_fuel_amnt = fuel;
            ctx->data.refuel_dialg.refuel_started = 1;
        }
        if (fuel <= ctx->data.refuel_dialg.fuel_target_kg - 1.0f)
        {
            fuel += ctx->data.refuel_dialg.fuel_rate_kg_s;
            if (acf_type_fuel_set(ctx->data.refuel_dialg.ic, &fuel))
            {
                XPLMSpeakString("fueling failed");
                ndt_log("navP [error]: refuel failed\n");
                ctx->data.refuel_dialg.adjust_fuel = NVP_MENU_DONE;
            }
        }
        else
        {
            XPLMSpeakString("fueling done");
            ctx->data.refuel_dialg.adjust_fuel = NVP_MENU_DONE;
            ndt_log("navP [info]: refueling completed (%f)\n", fuel);
        }
    }
    if (ctx->data.refuel_dialg.adjust_fuel == NVP_MENU_DECR)
    {
        if (ctx->data.refuel_dialg.refuel_started)
        {
            if (fuel >= ctx->data.refuel_dialg.last_fuel_amnt) // no deccrease
            {
                XPLMSpeakString("de-fueling failed");
                ndt_log("navP [error]: defueling failed\n");
                ctx->data.refuel_dialg.adjust_fuel = NVP_MENU_DONE;
                goto adjust_load;
            }
            ctx->data.refuel_dialg.last_fuel_amnt = fuel;
        }
        else
        {
            ctx->data.refuel_dialg.last_fuel_amnt = fuel;
            ctx->data.refuel_dialg.refuel_started = 1;
        }
        if (fuel >= ctx->data.refuel_dialg.fuel_target_kg + 1.0f)
        {
            fuel -= ctx->data.refuel_dialg.fuel_rate_kg_s;
            if (acf_type_fuel_set(ctx->data.refuel_dialg.ic, &fuel))
            {
                XPLMSpeakString("de-fueling failed");
                ndt_log("navP [error]: defueling failed\n");
                ctx->data.refuel_dialg.adjust_fuel = NVP_MENU_DONE;
            }
        }
        else
        {
            XPLMSpeakString("de-fueling done");
            ctx->data.refuel_dialg.adjust_fuel = NVP_MENU_DONE;
            ndt_log("navP [info]: defueling completed (%f)\n", fuel);
        }
    }
adjust_load:
    if (ctx->data.refuel_dialg.adjust_load == NVP_MENU_INCR)
    {
        if (ctx->data.refuel_dialg.brding_started)
        {
            if (load <= ctx->data.refuel_dialg.last_load_amnt) // no increase
            {
                XPLMSpeakString("boarding failed");
                ndt_log("navP [error]: boarding failed\n");
                ctx->data.refuel_dialg.adjust_load = NVP_MENU_DONE;
                goto adjust_done;
            }
            ctx->data.refuel_dialg.last_load_amnt = load;
        }
        else
        {
            ctx->data.refuel_dialg.last_load_amnt = load;
            ctx->data.refuel_dialg.brding_started = 1;
        }
        if (load <= ctx->data.refuel_dialg.load_target_kg - 1.0f)
        {
            load += ctx->data.refuel_dialg.load_rate_kg_s;
            if (acf_type_load_set(ctx->data.refuel_dialg.ic, &load))
            {
                XPLMSpeakString("boarding failed");
                ndt_log("navP [error]: boarding failed\n");
                ctx->data.refuel_dialg.adjust_load = NVP_MENU_DONE;
            }
        }
        else
        {
            XPLMSpeakString("boarding complete");
            ctx->data.refuel_dialg.adjust_load = NVP_MENU_DONE;
            ndt_log("navP [info]: boarding completed (%f)\n", load);
        }
    }
    if (ctx->data.refuel_dialg.adjust_load == NVP_MENU_DECR)
    {
        if (ctx->data.refuel_dialg.brding_started)
        {
            if (load >= ctx->data.refuel_dialg.last_load_amnt) // no deccrease
            {
                XPLMSpeakString("de-boarding failed");
                ndt_log("navP [error]: deboarding failed\n");
                ctx->data.refuel_dialg.adjust_load = NVP_MENU_DONE;
                goto adjust_done;
            }
            ctx->data.refuel_dialg.last_load_amnt = load;
        }
        else
        {
            ctx->data.refuel_dialg.last_load_amnt = load;
            ctx->data.refuel_dialg.brding_started = 1;
        }
        if (load >= ctx->data.refuel_dialg.load_target_kg + 1.0f)
        {
            load -= ctx->data.refuel_dialg.load_rate_kg_s;
            if (acf_type_load_set(ctx->data.refuel_dialg.ic, &load))
            {
                XPLMSpeakString("de-boarding failed");
                ndt_log("navP [error]: deboarding failed\n");
                ctx->data.refuel_dialg.adjust_load = NVP_MENU_DONE;
            }
        }
        else
        {
            XPLMSpeakString("de-boarding complete");
            ctx->data.refuel_dialg.adjust_load = NVP_MENU_DONE;
            ndt_log("navP [info]: deboarding completed (%f)\n", load);
        }
    }
adjust_done:
    if (ctx->data.refuel_dialg.adjust_fuel == NVP_MENU_DONE &&
        ctx->data.refuel_dialg.adjust_load == NVP_MENU_DONE)
    {
        float grwt; acf_type_grwt_get(ctx->data.refuel_dialg.ic, &grwt);
        float zfwt; acf_type_zfwt_get(ctx->data.refuel_dialg.ic, &zfwt);
        float lowt; acf_type_load_get(ctx->data.refuel_dialg.ic, &lowt);
        float fuwt; acf_type_fuel_get(ctx->data.refuel_dialg.ic, &fuwt);
        switch (ctx->data.refuel_dialg.ic->ac_type)
        {
            case ACF_TYP_B737_XG:
                {
                    /*
                     * Zero-fuel center of gravity (super-simplified):
                     * * +20.00% MAC while empty (payload == 00,000kgs)
                     * * +25.00% MAC when loaded (payload == 16,666kgs)
                     * shift +/- ~5.0%: final range +15.00% to +30.00%
                     *
                     * Translation to X-Plane's cgz_ref_to_default:
                     * * -16.0f (dataref) ~= -450.6% MAC
                     * * +00.0f (dataref) ~= +020.0% MAC
                     * * +16.0f (dataref) ~= +490.6% MAC
                     * Factor: 1.00% MAC ~= 16.0f/470.6f
                     */
                    srand(time(NULL)); // -0.17f to +0.17f (+/- ~5.0% MAC shift)
                    float zfw_cg_initial = (20.0f + (5.0f * (load / 16666.0f)));
                    float zfw_cgz_offset = (((rand() / (float)RAND_MAX) * 0.34f) - .17f);
                    ndt_log("navP [info]: IXEG B733 Classic: ZFWCG %+.1f\n", ((zfw_cg_initial + zfw_cgz_offset)));
                    XPLMSetDataf(ctx->data.refuel_dialg.ic->weight.zfwcgzm, (((zfw_cg_initial + zfw_cgz_offset) - 20.0f) / 29.4125f));
                }
                break;

            case ACF_TYP_LEGA_XC:
                {
                    srand(time(NULL)); // -0.25f to +0.25f (28.45 to 30.56% MAC)
                    float zfw_cgz = (((rand() / (float)RAND_MAX) * 0.5f) - .25f);
                    XPLMSetDataf(ctx->data.refuel_dialg.ic->weight.zfwcgzm, zfw_cgz);
                }
                break;

            default:
                acf_type_totalizr(ctx->data.refuel_dialg.ic);
                break;
        }
        ndt_log("navP [info]: aircraft GW %.0f ZFW %.0f load %.0f fuel %.0f cgz_ref_to_default %+.3f\n", grwt, zfwt, lowt, fuwt, XPLMGetDataf(ctx->data.refuel_dialg.ic->weight.zfwcgzm));
        return 0;
    }
    return 1.0f;
}

#undef AUTOSNPRINTF
#undef REFUEL_DIALG_CAPW
#undef REFUEL_DIALG_CAPH
#undef REFUEL_DIALG_CLBW
#undef REFUEL_DIALG_CLBH
#undef REFUEL_DIALG_DEFW
#undef REFUEL_DIALG_DEFH
#undef REFUEL_DIALG_TXTW
#undef REFUEL_DIALG_TXTH
#undef LOAD_MINIMUM_DIFF
#undef LOAD_MINIMUM_RATE
#undef SPEEDBOOSTER_DEFAULTV
#undef SPEEDBOOSTER_SETVALUE
