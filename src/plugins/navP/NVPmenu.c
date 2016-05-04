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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMMenus.h"
#include "XPLM/XPLMUtilities.h"

#include "NVPmenu.h"

typedef struct
{
    int id;
    enum
    {
        MENUITEM_CALLOUTS_STS,
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
        item_context speakweather;
    } items;

    struct
    {
        XPLMDataRef park_brake;
        XPLMDataRef speedbrake;
    } data_callouts_sts;

    struct
    {
        int     initialized;
        XPLMDataRef baro_sl;
        XPLMDataRef wind_dt;
        XPLMDataRef wind_sd;
        XPLMDataRef temp_dc;
        XPLMDataRef temp_dp;
    } data_speakweather;
} menu_context;

static void  menu_handler(void *inMenuRef,          void *inItemRef);
static char* string4speak(char *string, size_t alloc, int text_only);

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
    ctx->items.callouts_sts.mivalue = MENUITEM_CALLOUTS_STS;
    if ((ctx->items.callouts_sts.id = XPLMAppendMenuItem( ctx->id, "navP custom callouts",
                                                         &ctx->items.callouts_sts, 0)) < 0)
    {
        goto fail;
    }
    ctx->data_callouts_sts.park_brake = XPLMFindDataRef("navP/callouts/park_brake");
    ctx->data_callouts_sts.speedbrake = XPLMFindDataRef("navP/callouts/speedbrake");
    if (!ctx->data_callouts_sts.park_brake ||
        !ctx->data_callouts_sts.speedbrake)
    {
        goto fail;
    }
    else
    {
        // Note: XPLMSetDatai doesn't seem to work from XPluginEnable() either,
        //       so the default dataref and checkbox values can't be set here.
        XPLMAppendMenuSeparator(ctx->id);
    }
    ctx->items.speakweather.mivalue = MENUITEM_SPEAKWEATHER;
    if ((ctx->items.speakweather.id = XPLMAppendMenuItem( ctx->id, "Speak weather",
                                                         &ctx->items.speakweather, 0)) < 0)
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
        XPLMSetDatai     (         ctx->data_callouts_sts.park_brake,          1);
        XPLMSetDatai     (         ctx->data_callouts_sts.speedbrake,          1);
        ctx->setupdone = 1; return 0;
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

    /* all good */
    free(ctx);
    return 0;
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
            XPLMSetDatai     (ctx->data_callouts_sts.park_brake, 0);
            XPLMSetDatai     (ctx->data_callouts_sts.speedbrake, 0);
            return;
        }
        XPLMCheckMenuItem(ctx->id, itx->id,  xplm_Menu_Checked);
        XPLMSetDatai     (ctx->data_callouts_sts.park_brake, 1);
        XPLMSetDatai     (ctx->data_callouts_sts.speedbrake, 1);
        return;
    }

    if (itx->mivalue == MENUITEM_SPEAKWEATHER)
    {
        if (ctx->data_speakweather.initialized == 0)
        {
            ctx->data_speakweather.baro_sl = XPLMFindDataRef("sim/weather/barometer_sealevel_inhg");
            ctx->data_speakweather.wind_dt = XPLMFindDataRef("sim/weather/wind_direction_degt[0]" );
            ctx->data_speakweather.wind_sd = XPLMFindDataRef("sim/weather/wind_speed_kt[0]"       );
            ctx->data_speakweather.temp_dc = XPLMFindDataRef("sim/weather/temperature_ambient_c"  );
            ctx->data_speakweather.temp_dp = XPLMFindDataRef("sim/weather/dewpoi_sealevel_c"      );
            ctx->data_speakweather.initialized = 1;
        }
        char  baro[127], wind[127], temp[127], weather[255];
        float baro_sl = XPLMGetDataf(ctx->data_speakweather.baro_sl);
        float wind_dt = XPLMGetDataf(ctx->data_speakweather.wind_dt);
        float wind_sd = XPLMGetDataf(ctx->data_speakweather.wind_sd);
        float temp_dc = XPLMGetDataf(ctx->data_speakweather.temp_dc);
        float temp_dp = XPLMGetDataf(ctx->data_speakweather.temp_dp);
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
