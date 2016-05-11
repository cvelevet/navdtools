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
#include "XPLM/XPLMProcessing.h"
#include "XPLM/XPLMUtilities.h"

#include "common/common.h"

#include "NVPmenu.h"

#define REFUEL_DIALG_CAPW 120
#define REFUEL_DIALG_CAPH  24
#define REFUEL_DIALG_CLBW 140
#define REFUEL_DIALG_CLBH  24
#define REFUEL_DIALG_DEFW 200
#define REFUEL_DIALG_DEFH 200
#define REFUEL_DIALG_TXTW  50
#define REFUEL_DIALG_TXTH  24
#define REFUEL_FUEL_KHSEC 35.0f // 35kg/0.5s: 42,000kg of fuel in 10m (600s)
#define REFUEL_PLOD_KHSEC 26.0f // 26kg/0.5s: 300 pax (x104kg) in 10m (600s)

typedef struct
{
    int id;
    enum
    {
        MENUITEM_NOTHING_TODO,
        MENUITEM_CALLOUTS_STS,
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
        item_context callouts_sts;
        item_context refuel_dialg;
        item_context speakweather;
    } items;

    struct
    {
        struct
        {
            XPLMDataRef park_brake;
            XPLMDataRef speedbrake;
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
    } data;
} menu_context;

static void  mainw_center(int out[4],                  int in_w, int in_h);
static void  menu_rm_item(menu_context *ctx,                    int index);
static void  menu_handler(void *inMenuRef,                void *inItemRef);
static char* string4speak(char *string,       size_t alloc, int text_only);
static int   widget_hdlr1(XPWidgetMessage, XPWidgetID, intptr_t, intptr_t);
static float refuel_hdlr1(                       float, float, int, void*);

void* nvp_menu_init(void)
{
    menu_context *ctx = calloc(1, sizeof(menu_context));
    if (!ctx)
    {
        return NULL;
    }

    /* create a submenu in the plugins menu */
    if ((ctx->id = XPLMCreateMenu("navP", NULL, 0, &menu_handler, ctx)) == NULL)
    {
        goto fail;
    }

    /* add desired items */
    /* toggle: callouts on/off */
    ctx->items.callouts_sts.mivalue = MENUITEM_CALLOUTS_STS;
    if ((ctx->items.callouts_sts.id = XPLMAppendMenuItem( ctx->id, "navP custom callouts",
                                                         &ctx->items.callouts_sts, 0)) < 0)
    {
        goto fail;
    }
    ctx->data.callouts_sts.park_brake = XPLMFindDataRef("navP/callouts/park_brake");
    ctx->data.callouts_sts.speedbrake = XPLMFindDataRef("navP/callouts/speedbrake");
    if (!ctx->data.callouts_sts.park_brake ||
        !ctx->data.callouts_sts.speedbrake)
    {
        goto fail;
    }
    else
    {
        // Note: XPLMSetDatai doesn't seem to work from XPluginEnable() either,
        //       so the default dataref and checkbox values can't be set here.
        XPLMAppendMenuSeparator(ctx->id);
    }

    /* show payload & fuel dialog */
    ctx->items.refuel_dialg.mivalue = MENUITEM_REFUEL_DIALG;
    if ((ctx->items.refuel_dialg.id = XPLMAppendMenuItem( ctx->id, "Fuel & Payload",
                                                         &ctx->items.refuel_dialg, 0)) < 0)
    {
        goto fail;
    }
    ctx->data.refuel_dialg.fuel_max = XPLMFindDataRef("sim/aircraft/weight/acf_m_fuel_tot" );
    ctx->data.refuel_dialg.fuel_rat = XPLMFindDataRef("sim/aircraft/overflow/acf_tank_rat" );
    ctx->data.refuel_dialg.fuel_qty = XPLMFindDataRef("sim/flightmodel/weight/m_fuel"      );
    ctx->data.refuel_dialg.fuel_tot = XPLMFindDataRef("sim/flightmodel/weight/m_fuel_total");
    ctx->data.refuel_dialg.pay_load = XPLMFindDataRef("sim/flightmodel/weight/m_fixed"     );
    if (!ctx->data.refuel_dialg.fuel_max ||
        !ctx->data.refuel_dialg.fuel_rat ||
        !ctx->data.refuel_dialg.fuel_qty ||
        !ctx->data.refuel_dialg.fuel_tot ||
        !ctx->data.refuel_dialg.pay_load)
    {
        goto fail;
    }
    else
    {
        XPLMAppendMenuSeparator(ctx->id);
    }

    /* speak local weather */
    ctx->items.speakweather.mivalue = MENUITEM_SPEAKWEATHER;
    if ((ctx->items.speakweather.id = XPLMAppendMenuItem( ctx->id, "Speak weather",
                                                         &ctx->items.speakweather, 0)) < 0)
    {
        goto fail;
    }
    ctx->data.speakweather.baro_sl = XPLMFindDataRef("sim/weather/barometer_sealevel_inhg");
    ctx->data.speakweather.wind_dt = XPLMFindDataRef("sim/weather/wind_direction_degt[0]" );
    ctx->data.speakweather.wind_sd = XPLMFindDataRef("sim/weather/wind_speed_kt[0]"       );
    ctx->data.speakweather.temp_dc = XPLMFindDataRef("sim/weather/temperature_ambient_c"  );
    ctx->data.speakweather.temp_dp = XPLMFindDataRef("sim/weather/dewpoi_sealevel_c"      );
    if (!ctx->data.speakweather.baro_sl ||
        !ctx->data.speakweather.wind_dt ||
        !ctx->data.speakweather.wind_sd ||
        !ctx->data.speakweather.temp_dc ||
        !ctx->data.speakweather.temp_dp)
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
         * Set defaults for dataref-backed variables,
         * since we can't do it in XPluginEnable().
         *
         * Future: read from a config file instead of hardcoding said defaults.
         */
        XPLMCheckMenuItem(ctx->id, ctx->items.callouts_sts.id, xplm_Menu_Checked);
        XPLMSetDatai     (         ctx->data.callouts_sts.park_brake,          1);
        XPLMSetDatai     (         ctx->data.callouts_sts.speedbrake,          1);

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
        int inLT = 0;
        int inTP = REFUEL_DIALG_DEFH - 1;
        int inRT = REFUEL_DIALG_DEFW - 1;
        int inBM = 0;
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

    /* all good */
    free(ctx);
    return 0;
}

static void mainw_center(int out[4], int in_w, int in_h)
{
    int scrn_w, scrn_h;
    XPLMGetScreenSize(&scrn_w, &scrn_h);
    out[0] = (scrn_w - in_w) / 2; // inLeft
    out[3] = (scrn_h - in_h) / 2; // inBottom
    out[2] = (out[0] + in_w) - 1; // inRight
    out[1] = (out[3] + in_h) - 1; // inTop
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

    if (itx->mivalue == MENUITEM_CALLOUTS_STS)
    {
        XPLMMenuCheck state = xplm_Menu_Checked;
        XPLMCheckMenuItemState(ctx->id, itx->id, &state);
        if (state == xplm_Menu_Checked)
        {
            XPLMCheckMenuItem(ctx->id, itx->id,  xplm_Menu_NoCheck);
            XPLMSetDatai     (ctx->data.callouts_sts.park_brake, 0);
            XPLMSetDatai     (ctx->data.callouts_sts.speedbrake, 0);
            return;
        }
        XPLMCheckMenuItem(ctx->id, itx->id,  xplm_Menu_Checked);
        XPLMSetDatai     (ctx->data.callouts_sts.park_brake, 1);
        XPLMSetDatai     (ctx->data.callouts_sts.speedbrake, 1);
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
        float temp; char descr_buf[9]; descr_buf[8] = '\0';
        menu_context *ctx = (void*)XPGetWidgetProperty((void*)inParam1, xpProperty_Refcon, NULL);
        XPHideWidget (ctx->data.refuel_dialg.dialog_id);
        XPGetWidgetDescriptor(ctx->data.refuel_dialg.f_txtf_id, descr_buf, sizeof(descr_buf) - 1);
        if (sscanf(descr_buf, "%f", &temp) != 1)
        {
            ctx->data.refuel_dialg.fuel_target_kg = ctx->data.refuel_dialg.min_fuel_total;
        }
        else if ((1000.0f * temp) < ctx->data.refuel_dialg.min_fuel_total)
        {
            ctx->data.refuel_dialg.fuel_target_kg = ctx->data.refuel_dialg.min_fuel_total;
        }
        else if ((1000.0f * temp) > ctx->data.refuel_dialg.max_fuel_total)
        {
            ctx->data.refuel_dialg.fuel_target_kg = ctx->data.refuel_dialg.max_fuel_total;
        }
        else
        {
            ctx->data.refuel_dialg.fuel_target_kg = 1000.0f * temp;
        }
        XPGetWidgetDescriptor(ctx->data.refuel_dialg.p_txtf_id, descr_buf, sizeof(descr_buf) - 1);
        if (sscanf(descr_buf, "%f", &temp) != 1 || temp < 0.2f)
        {
            ctx->data.refuel_dialg.plod_target_kg = 0.0f;
        }
        else
        {
            ctx->data.refuel_dialg.plod_target_kg = 1000.0f * temp;
        }
        float fuel_tot = XPLMGetDataf(ctx->data.refuel_dialg.fuel_tot);
        ctx->data.refuel_dialg.adjust_fuel = (fabsf(fuel_tot - ctx->data.refuel_dialg.fuel_target_kg) > 50.0f);
        float pay_load = XPLMGetDataf(ctx->data.refuel_dialg.pay_load);
        ctx->data.refuel_dialg.adjust_plod = (fabsf(pay_load - ctx->data.refuel_dialg.plod_target_kg) > 50.0f);
        if (ctx->data.refuel_dialg.rfc)
        {
            XPLMUnregisterFlightLoopCallback(ctx->data.refuel_dialg.rfc, ctx);
        }
        if (ctx->data.refuel_dialg.adjust_fuel || ctx->data.refuel_dialg.adjust_plod)
        {
            ndt_log("navP [info]: loading/unloading: fuel %.2f metric tons, payload %.2f metric tons\n",
                    ctx->data.refuel_dialg.fuel_target_kg / 1000.0f,
                    ctx->data.refuel_dialg.plod_target_kg / 1000.0f);
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
    int disable_cllbk = 1;
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
            ctx->data.refuel_dialg.adjust_fuel = 0;
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
            ctx->data.refuel_dialg.adjust_plod = 0;
        }
    }
    if (disable_cllbk)
    {
        ndt_log("navP [info]: refuel/defuel procedure completed\n");
        XPLMSpeakString("Loading/unloading done");
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
