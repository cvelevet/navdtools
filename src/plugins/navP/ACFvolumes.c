/*
 * ACFvolumes.c
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2017 Timothy D. Walker and others.
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
#include <stdlib.h>

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMDefs.h"
#include "XPLM/XPLMPlugin.h"

#include "common/common.h"

#include "ACFvolumes.h"

static acf_volume_context *default_ctx = NULL;

acf_volume_context* acf_volume_ctx_get(void)
{
    if ((default_ctx) != NULL)
    {
        return default_ctx;
    }
    if ((default_ctx = malloc(sizeof(acf_volume_context))) == NULL)
    {
        goto fail;
    }

    /* default X-Plane datarefs */
    if (NULL == (default_ctx->sound.  nabled = XPLMFindDataRef("sim/operation/sound/sound_on"                       )) ||
        NULL == (default_ctx->radio.  atctxt = XPLMFindDataRef("sim/operation/prefs/text_out"                       )) ||
        NULL == (default_ctx->sound.  speech = XPLMFindDataRef("sim/operation/sound/speech_on"                      )) ||
        NULL == (default_ctx->radio.  volume = XPLMFindDataRef("sim/operation/sound/radio_volume_ratio"             )) ||
        NULL == (default_ctx->radio.vol.dme0 = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_volume_dme"     )) ||
        NULL == (default_ctx->radio.vol.adf1 = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_volume_adf1"    )) ||
        NULL == (default_ctx->radio.vol.adf2 = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_volume_adf2"    )) ||
        NULL == (default_ctx->radio.vol.com1 = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_volume_com1"    )) ||
        NULL == (default_ctx->radio.vol.com2 = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_volume_com2"    )) ||
        NULL == (default_ctx->radio.vol.nav1 = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_volume_nav1"    )) ||
        NULL == (default_ctx->radio.vol.nav2 = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_volume_nav2"    )) ||
        NULL == (default_ctx->radio.vol.mark = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_volume_mark"    )) ||
        NULL == (default_ctx->radio.tx. comm = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_com_selection"  )) ||
        NULL == (default_ctx->radio.rx. com1 = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_selection_com1" )) ||
        NULL == (default_ctx->radio.rx. com2 = XPLMFindDataRef("sim/cockpit2/radios/actuators/audio_selection_com2" )))
    {
        goto fail;
    }

    /* X-Plane 10 sound datarefs */
    if (1) // TODO check version
    {
        if (NULL == (default_ctx->sound.xp10.fan = XPLMFindDataRef("sim/operation/sound/fan_volume_ratio"           )) ||
            NULL == (default_ctx->sound.xp10.prp = XPLMFindDataRef("sim/operation/sound/prop_volume_ratio"          )) ||
            NULL == (default_ctx->sound.xp10.eng = XPLMFindDataRef("sim/operation/sound/engine_volume_ratio"        )) ||
            NULL == (default_ctx->sound.xp10.wxr = XPLMFindDataRef("sim/operation/sound/ground_volume_ratio"        )) ||
            NULL == (default_ctx->sound.xp10.grd = XPLMFindDataRef("sim/operation/sound/warning_volume_ratio"       )) ||
            NULL == (default_ctx->sound.xp10.wrn = XPLMFindDataRef("sim/operation/sound/weather_volume_ratio"       )))
        {
            goto fail;
        }
    }

    /* defaults that may change at runtime */
    default_ctx->custom.atc.pe.vol0 = M_SQRT1_2;
    default_ctx->custom.atc.pe.vol1 = M_SQRT1_2;

    /* success */
    return default_ctx;

fail:
    if ((default_ctx) != NULL)
    {
        free(default_ctx);
        default_ctx = NULL;
    }
    return NULL;
}

#define SETDR_CHECK(fnc, ref, val) { if (ref) { fnc(ref, val); } }
static int acf_radios_set(acf_volume_context *ctx, float volume)
{
    if (ctx)
    {
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("com.pilotedge.plugin.xplane"))
        {
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.ea50.v1, ctx->custom.atc.pe.vol1);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.ea50.v2, ctx->custom.atc.pe.vol1);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.pipa.v1, ctx->custom.atc.pe.vol1);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.pipa.v2, ctx->custom.atc.pe.vol1);
            XPLMSetDataf(ctx->radio.vol.com1,     ctx->custom.atc.pe.vol1);
            XPLMSetDataf(ctx->radio.vol.com2,     ctx->custom.atc.pe.vol1);
            XPLMSetDataf(ctx->radio.vol.nav1,     ctx->custom.atc.pe.vol1);
            XPLMSetDataf(ctx->radio.vol.nav2,     ctx->custom.atc.pe.vol1);
            XPLMSetDataf(ctx->radio.vol.adf1,     ctx->custom.atc.pe.vol1);
            XPLMSetDataf(ctx->radio.vol.adf2,     ctx->custom.atc.pe.vol1);
            XPLMSetDataf(ctx->radio.vol.mark,     ctx->custom.atc.pe.vol1);
            XPLMSetDataf(ctx->radio.vol.dme0,     ctx->custom.atc.pe.vol1);
            XPLMSetDataf(ctx->radio.volume,       ctx->custom.atc.pe.vol0);
            int tx = XPLMGetDatai(ctx->radio.tx.comm);
            if (tx < 6 || tx > 7)
            {
                XPLMSetDatai(ctx->radio.tx.comm, 6);
            }
            if (XPLMGetDatai(ctx->radio.rx.com1) <= 0 &&
                XPLMGetDatai(ctx->radio.rx.com2) <= 0)
            {
                XPLMSetDatai(ctx->radio.rx.com1, 1);
            }
            if (XPLMGetDatai(ctx->radio.atctxt) <= 0)
            {
                XPLMSetDatai(ctx->radio.atctxt, 1);
            }
            return 1;
        }
        else
        {
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.ea50.v1, 0.9f);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.ea50.v2, 0.9f);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.pipa.v1, 0.9f);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.pipa.v2, 0.9f);
            XPLMSetDataf(ctx->radio.vol.com1,     0.9f);
            XPLMSetDataf(ctx->radio.vol.com2,     0.9f);
            XPLMSetDataf(ctx->radio.vol.nav1,     0.9f);
            XPLMSetDataf(ctx->radio.vol.nav2,     0.9f);
            XPLMSetDataf(ctx->radio.vol.adf1,     0.9f);
            XPLMSetDataf(ctx->radio.vol.adf2,     0.9f);
            XPLMSetDataf(ctx->radio.vol.mark,     0.9f);
            XPLMSetDataf(ctx->radio.vol.dme0,     0.9f);
            XPLMSetDataf(ctx->radio.volume, sqrtf(volume));
            if (XPLMGetDatai(ctx->radio.atctxt) <= 0)
            {
                XPLMSetDatai(ctx->radio.atctxt, 1);
            }
            return (volume >= 0.01f);
        }
    }
    return 0;
}
#undef SETDR_CHECK

float acf_atcvol_adj(acf_volume_context *ctx, float offset)
{
    if (ctx)
    {
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("com.pilotedge.plugin.xplane"))
        {
            if (fabsf(offset) >= 0.01f)
            {
                ctx->custom.atc.pe.vol0 += offset;
            }
            if (ctx->custom.atc.pe.vol0 > 1.0f)
            {
                ctx->custom.atc.pe.vol0 = 1.0f;
            }
            if (ctx->custom.atc.pe.vol0 < 0.1f)
            {
                ctx->custom.atc.pe.vol0 = 0.0f;
            }
            if (acf_radios_set(ctx, ctx->custom.atc.pe.vol0) == 1)
            {
                return ctx->custom.atc.pe.vol0;
            }
        }
    }
    return -1;
}

void acf_volume_set(acf_volume_context *ctx, acf_type type, float volume)
{
    if (ctx)
    {
        /* dereference addon-specific datarefs */
        ctx->custom.fsts.volume = XPLMFindDataRef("volumeX");
        ctx->custom.fft7.ambien = XPLMFindDataRef("volume/ambient");
        ctx->custom.fft7.engine = XPLMFindDataRef("volume/engines");
        ctx->custom.fft7.volume = XPLMFindDataRef("volume/volumeX");
        ctx->custom.fft7.cllout = XPLMFindDataRef("volume/callouts");
        ctx->custom.ddnn.volume = XPLMFindDataRef("cl300/sound_vol");
        ctx->custom.ddnn.volmut = XPLMFindDataRef("cl300/sound_mute");
        ctx->custom.ddnn.volext = XPLMFindDataRef("cl300/ext_sound_vol");
        ctx->custom.a350.volume = XPLMFindDataRef("1-sim/options/Volume");
        ctx->custom.absk.mtrack = XPLMFindDataRef("aerobask/eclipse/m_trk");
        ctx->custom.tlss.master = XPLMFindDataRef("toliss_airbus/master_volume");
        ctx->custom.atc.ea50.v1 = XPLMFindDataRef("aerobask/eclipse/gtn750_vol");
        ctx->custom.atc.ea50.v2 = XPLMFindDataRef("aerobask/eclipse/gtn650_vol");
        ctx->custom.atc.pipa.v1 = XPLMFindDataRef("aerobask/panthera/gtn750_vol");
        ctx->custom.atc.pipa.v2 = XPLMFindDataRef("aerobask/panthera/gtn650_vol");
        ctx->custom.absk.volume = XPLMFindDataRef("aerobask/eclipse/custom_volume_ratio");

        /* sanitize volume */
        if (volume < 0.01f)
        {
            volume = 0.00f;
        }

        /* non-ATC */
        switch (type)
        {
            case ACF_TYP_A319_TL:
                // ToLiSS A319's defaults:
                // - engine volume     25%
                // - system volume     25%
                // - cockpit sounds    75%
                // - environmental     25%
                // - ground contact    25%
                // - aural alert       25%
                // - external ratio    50%
                // we calibrate the A329's master volume so its external
                // volume matches "volume" at our default setting of 25%
//              XPLMSetDataf(ctx->sound. xp10.   eng,          0.0f);
                XPLMSetDataf(ctx->sound. xp10.   prp,          0.0f);
//              XPLMSetDataf(ctx->sound. xp10.   wxr,          0.0f);
//              XPLMSetDataf(ctx->sound. xp10.   wrn,          0.0f);
//              XPLMSetDataf(ctx->sound. xp10.   grd,          0.0f);
//              XPLMSetDataf(ctx->sound. xp10.   fan,          0.0f);
                XPLMSetDataf(ctx->custom.tlss.master, sqrtf(volume));
                break;
            case ACF_TYP_A320_FF:
                XPLMSetDataf(ctx->sound.xp10.eng, volume / 1.25f);
                XPLMSetDataf(ctx->sound.xp10.prp, volume / 1.25f);
                XPLMSetDataf(ctx->sound.xp10.wxr, volume / 1.25f);
                XPLMSetDataf(ctx->sound.xp10.wrn, volume / 1.25f);
                XPLMSetDataf(ctx->sound.xp10.grd, volume / 1.25f);
                XPLMSetDataf(ctx->sound.xp10.fan, volume / 1.25f);
                break;
            case ACF_TYP_A350_FF:
                XPLMSetDataf(ctx->sound. xp10.   eng, volume);
                XPLMSetDataf(ctx->sound. xp10.   prp, volume);
                XPLMSetDataf(ctx->sound. xp10.   wxr, volume);
                XPLMSetDataf(ctx->sound. xp10.   wrn, volume);
                XPLMSetDataf(ctx->sound. xp10.   grd, volume);
                XPLMSetDataf(ctx->sound. xp10.   fan, volume);
                XPLMSetDataf(ctx->custom.a350.volume, volume);
                break;
            case ACF_TYP_B737_XG:
                break;
            case ACF_TYP_B757_FF:
            case ACF_TYP_B767_FF:
                XPLMSetDataf(ctx->sound. xp10.   eng, volume * 1.0f);
                XPLMSetDataf(ctx->sound. xp10.   prp, volume * 1.0f);
                XPLMSetDataf(ctx->sound. xp10.   wxr, volume * 1.0f);
                XPLMSetDataf(ctx->sound. xp10.   wrn, volume * 1.0f);
                XPLMSetDataf(ctx->sound. xp10.   grd, volume * 1.0f);
                XPLMSetDataf(ctx->sound. xp10.   fan, volume * 1.0f);
                XPLMSetDataf(ctx->custom.fsts.volume, volume * 0.5f);
                break;
            case ACF_TYP_B777_FF:
                XPLMSetDataf(ctx->sound. xp10.   eng, volume * 1.0f);
                XPLMSetDataf(ctx->sound. xp10.   prp, volume * 1.0f);
                XPLMSetDataf(ctx->sound. xp10.   wxr, volume * 1.0f);
                XPLMSetDataf(ctx->sound. xp10.   wrn, volume * 1.0f);
                XPLMSetDataf(ctx->sound. xp10.   grd, volume * 1.0f);
                XPLMSetDataf(ctx->sound. xp10.   fan, volume * 1.0f);
                XPLMSetDataf(ctx->custom.fft7.ambien, volume * 1.0f);
                XPLMSetDataf(ctx->custom.fft7.cllout, volume * 1.5f);
                XPLMSetDataf(ctx->custom.fft7.engine, volume * 1.0f);
                XPLMSetDataf(ctx->custom.fft7.volume, volume * 0.5f);
                break;
            case ACF_TYP_CL30_DD:
                XPLMSetDataf(ctx->sound.xp10.eng,      volume);
                XPLMSetDataf(ctx->sound.xp10.prp,      volume);
                XPLMSetDataf(ctx->sound.xp10.wxr,      volume);
                XPLMSetDataf(ctx->sound.xp10.wrn,      volume);
                XPLMSetDataf(ctx->sound.xp10.grd,      volume);
                XPLMSetDataf(ctx->sound.xp10.fan,      volume);
                XPLMSetDataf(ctx->custom.ddnn.volext, (0.25f));
                XPLMSetDatai(ctx->custom.ddnn.volmut, (0.01f > volume));
                XPLMSetDatai(ctx->custom.ddnn.volume, (int)roundf(2200.0f * sqrtf(volume)));
                break;
            default:
                XPLMSetDataf(ctx->sound.xp10.eng, volume);
                XPLMSetDataf(ctx->sound.xp10.prp, volume);
                XPLMSetDataf(ctx->sound.xp10.wxr, volume);
                XPLMSetDataf(ctx->sound.xp10.wrn, volume);
                XPLMSetDataf(ctx->sound.xp10.grd, volume);
                XPLMSetDataf(ctx->sound.xp10.fan, volume);
                if (ctx->custom.absk.volume && ctx->custom.absk.mtrack &&
                    fabsf(XPLMGetDataf(ctx->custom.absk.mtrack)) > 0.001f)
                {
                    XPLMSetDataf(ctx->sound.xp10.eng, volume / 2.0f);
                    XPLMSetDataf(ctx->custom.absk.volume, volume);
                }
                break;
        }

        /* ATC-related */
        int enabled = acf_radios_set(ctx, volume);

        /* global toggles */
        XPLMSetDatai(ctx->sound.nabled, !!enabled);
        XPLMSetDatai(ctx->sound.speech, !!enabled);
    }
}
