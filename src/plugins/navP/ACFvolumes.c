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

#define V0_DEFAULT_PE 0.25f
#define V0_DEFAULT_XB 0.25f
#define V1_DEFAULT_AL M_SQRT1_2
#define V1_DEFAULT_PE M_SQRT1_2
#define V1_DEFAULT_XB M_SQRT1_2
#define V2_DEFAULT_AL M_SQRT1_2
#define V2_DEFAULT_PE M_SQRT1_2
#define V2_DEFAULT_XB M_SQRT1_2

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

    /* are we X-Plane 11??? */
    default_ctx->x_plane_v11 = (NULL != XPLMFindDataRef("sim/version/xplane_internal_version"));

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

    /* Version-specific things */
    if (default_ctx->x_plane_v11)
    {
        if (NULL == (default_ctx->sound.xp11.uiv = XPLMFindDataRef("sim/operation/sound/ui_volume_ratio"            )) ||
            NULL == (default_ctx->sound.xp11.fan = XPLMFindDataRef("sim/operation/sound/fan_volume_ratio"           )) ||
            NULL == (default_ctx->sound.xp11.prp = XPLMFindDataRef("sim/operation/sound/prop_volume_ratio"          )) ||
            NULL == (default_ctx->sound.xp11.eng = XPLMFindDataRef("sim/operation/sound/engine_volume_ratio"        )) ||
            NULL == (default_ctx->sound.xp11.env = XPLMFindDataRef("sim/operation/sound/enviro_volume_ratio"        )) ||
            NULL == (default_ctx->sound.xp11.grd = XPLMFindDataRef("sim/operation/sound/ground_volume_ratio"        )) ||
            NULL == (default_ctx->sound.xp11.mas = XPLMFindDataRef("sim/operation/sound/master_volume_ratio"        )) ||
            NULL == (default_ctx->sound.xp11.cop = XPLMFindDataRef("sim/operation/sound/copilot_volume_ratio"       )) ||
            NULL == (default_ctx->sound.xp11.wrn = XPLMFindDataRef("sim/operation/sound/warning_volume_ratio"       )) ||
            NULL == (default_ctx->sound.xp11.wxr = XPLMFindDataRef("sim/operation/sound/weather_volume_ratio"       )) ||
            NULL == (default_ctx->sound.xp11.ext = XPLMFindDataRef("sim/operation/sound/exterior_volume_ratio"      )) ||
            NULL == (default_ctx->sound.xp11.inn = XPLMFindDataRef("sim/operation/sound/interior_volume_ratio"      )))
        {
            goto fail;
        }
    }
    else
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

    /*
     * defaults that may change at runtime
     */
    default_ctx->custom.atc.pe.vol0 = V0_DEFAULT_PE;
    default_ctx->custom.atc.pe.vol1 = V1_DEFAULT_PE;
    default_ctx->custom.atc.pe.vol2 = V2_DEFAULT_PE;
    default_ctx->custom.atc.xb.vol0 = V0_DEFAULT_XB;
    default_ctx->custom.atc.xb.vol1 = V1_DEFAULT_XB;
    default_ctx->custom.atc.xb.vol2 = V2_DEFAULT_XB;

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
            XPLMSetDataf(ctx->radio.vol.adf1,     ctx->custom.atc.pe.vol2);
            XPLMSetDataf(ctx->radio.vol.adf2,     ctx->custom.atc.pe.vol2);
            XPLMSetDataf(ctx->radio.vol.dme0,     ctx->custom.atc.pe.vol2);
            XPLMSetDataf(ctx->radio.vol.mark,     ctx->custom.atc.pe.vol2);
            XPLMSetDataf(ctx->radio.vol.nav1,     ctx->custom.atc.pe.vol2);
            XPLMSetDataf(ctx->radio.vol.nav2,     ctx->custom.atc.pe.vol2);
            XPLMSetDataf(ctx->radio.vol.com1,     ctx->custom.atc.pe.vol1);
            XPLMSetDataf(ctx->radio.vol.com2,     ctx->custom.atc.pe.vol1);
            XPLMSetDataf(ctx->radio.volume,       ctx->custom.atc.pe.vol0);
            XPLMSetDatai(ctx->radio.atctxt,                 volume > .01f); // Text-to-speech overlays
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
            return 1;
        }
        else if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("vatsim.protodev.clients.xsquawkbox"))
        {
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.ea50.v1, ctx->custom.atc.xb.vol1);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.ea50.v2, ctx->custom.atc.xb.vol1);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.pipa.v1, ctx->custom.atc.xb.vol1);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.pipa.v2, ctx->custom.atc.xb.vol1);
            XPLMSetDataf(ctx->radio.vol.adf1,     ctx->custom.atc.xb.vol2);
            XPLMSetDataf(ctx->radio.vol.adf2,     ctx->custom.atc.xb.vol2);
            XPLMSetDataf(ctx->radio.vol.dme0,     ctx->custom.atc.xb.vol2);
            XPLMSetDataf(ctx->radio.vol.mark,     ctx->custom.atc.xb.vol2);
            XPLMSetDataf(ctx->radio.vol.nav1,     ctx->custom.atc.xb.vol2);
            XPLMSetDataf(ctx->radio.vol.nav2,     ctx->custom.atc.xb.vol2);
            XPLMSetDataf(ctx->radio.vol.com1,     ctx->custom.atc.xb.vol1);
            XPLMSetDataf(ctx->radio.vol.com2,     ctx->custom.atc.xb.vol1);
            XPLMSetDataf(ctx->radio.volume,       ctx->custom.atc.xb.vol0);
            XPLMSetDatai(ctx->radio.atctxt,               (volume > .01f)); // Text-to-speech overlays
            int tx = XPLMGetDatai(ctx->radio.tx.comm);
            if (XPLMGetDatai(ctx->radio.rx.com1) == 1 &&
                XPLMGetDatai(ctx->radio.rx.com2) == 1 && tx == 0)
            {
                // XXX: XSquawkBox-specific, don't start
                // with both radios on for first connect
                // rx=both tx=none set in NVPchandlers.c
                XPLMSetDatai(ctx->radio.tx.comm, (tx = 6));
                XPLMSetDatai(ctx->radio.rx.com1, 1);
                XPLMSetDatai(ctx->radio.rx.com1, 0);
            }
            if (tx < 6 || tx > 7)
            {
                XPLMSetDatai(ctx->radio.tx.comm, 6);
            }
            if (XPLMGetDatai(ctx->radio.rx.com1) <= 0 &&
                XPLMGetDatai(ctx->radio.rx.com2) <= 0)
            {
                XPLMSetDatai(ctx->radio.rx.com1, 1);
            }
            return 1;
        }
        else
        {
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.ea50.v1, V1_DEFAULT_AL);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.ea50.v2, V1_DEFAULT_AL);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.pipa.v1, V1_DEFAULT_AL);
SETDR_CHECK(XPLMSetDataf,ctx->custom.atc.pipa.v2, V1_DEFAULT_AL);
            XPLMSetDataf(ctx->radio.vol.adf1,     V2_DEFAULT_AL);
            XPLMSetDataf(ctx->radio.vol.adf2,     V2_DEFAULT_AL);
            XPLMSetDataf(ctx->radio.vol.dme0,     V2_DEFAULT_AL);
            XPLMSetDataf(ctx->radio.vol.mark,     V2_DEFAULT_AL);
            XPLMSetDataf(ctx->radio.vol.nav1,     V2_DEFAULT_AL);
            XPLMSetDataf(ctx->radio.vol.nav2,     V2_DEFAULT_AL);
            XPLMSetDataf(ctx->radio.vol.com1,     V1_DEFAULT_AL);
            XPLMSetDataf(ctx->radio.vol.com2,     V1_DEFAULT_AL);
            XPLMSetDataf(ctx->radio.volume,       sqrtf(volume));
            XPLMSetDatai(ctx->radio.atctxt,       volume > .01f); // Text-to-speech overlays
            return (volume > .01f);
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
            if (fabsf(offset) > 0.5f) // offset is in "ticks" of 1/10th the default
            {
                ctx->custom.atc.pe.vol0 += roundf(offset) * V0_DEFAULT_PE * 0.1f;
            }
            if (fabsf((float)V0_DEFAULT_PE - ctx->custom.atc.pe.vol0) < 0.02f)
            {
                ctx->custom.atc.pe.vol0 = V0_DEFAULT_PE;
            }
            if (ctx->custom.atc.pe.vol0 > 1.0f)
            {
                ctx->custom.atc.pe.vol0 = 1.0f;
            }
            if (ctx->custom.atc.pe.vol0 < 0.1f)
            {
                ctx->custom.atc.pe.vol0 = 0.0f;
            }
            if (acf_radios_set(ctx, 0.0f/*ignored*/) == 1)
            {
                return ctx->custom.atc.pe.vol0;
            }
            return -1;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("vatsim.protodev.clients.xsquawkbox"))
        {
            if (fabsf(offset) > 0.5f) // offset is in "ticks" of 1/10th the default
            {
                ctx->custom.atc.xb.vol0 += roundf(offset) * V0_DEFAULT_XB * 0.1f;
            }
            if (fabsf((float)V0_DEFAULT_XB - ctx->custom.atc.xb.vol0) < 0.02f)
            {
                ctx->custom.atc.xb.vol0 = V0_DEFAULT_XB;
            }
            if (ctx->custom.atc.xb.vol0 > 1.0f)
            {
                ctx->custom.atc.xb.vol0 = 1.0f;
            }
            if (ctx->custom.atc.xb.vol0 < 0.1f)
            {
                ctx->custom.atc.xb.vol0 = 0.0f;
            }
            if (acf_radios_set(ctx, 0.0f/*ignored*/) == 1)
            {
                return ctx->custom.atc.xb.vol0;
            }
            return -1;
        }
        return -1;
    }
    return -1;
}

void acf_volume_reset(acf_volume_context *ctx, acf_type type)
{
    if (ctx)
    {
        return acf_volume_set(ctx, ctx->last_volume, type);
    }
}

void acf_volume_set(acf_volume_context *ctx, float volume, acf_type type)
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
        if (volume > 1.00f)
        {
            volume = 1.00f;
        }
        if (volume > 0.50f && ctx->x_plane_v11)
        {
            volume = 0.50f;
        }
        ctx->last_volume = volume;

        /* non-ATC */
        switch (type)
        {
            case ACF_TYP_A319_TL:
            case ACF_TYP_A321_TL:
                XPLMSetDataf(ctx->custom.tlss.master, sqrtf(volume));
                break;

            case ACF_TYP_A320_FF:
                if (ctx->x_plane_v11)
                {
                    //fixme
                }
                else
                {
                    XPLMSetDataf(ctx->sound.xp10.eng, volume / 1.25f);
                    XPLMSetDataf(ctx->sound.xp10.prp, volume / 1.25f);
                    XPLMSetDataf(ctx->sound.xp10.wxr, volume / 1.25f);
                    XPLMSetDataf(ctx->sound.xp10.wrn, volume / 1.25f);
                    XPLMSetDataf(ctx->sound.xp10.grd, volume / 1.25f);
                    XPLMSetDataf(ctx->sound.xp10.fan, volume / 1.25f);
                }
                break;

            case ACF_TYP_A350_FF:
                if (ctx->x_plane_v11)
                {
                    //fixme
                }
                else
                {
                    XPLMSetDataf(ctx->sound.xp10.eng, volume);
                    XPLMSetDataf(ctx->sound.xp10.prp, volume);
                    XPLMSetDataf(ctx->sound.xp10.wxr, volume);
                    XPLMSetDataf(ctx->sound.xp10.wrn, volume);
                    XPLMSetDataf(ctx->sound.xp10.grd, volume);
                    XPLMSetDataf(ctx->sound.xp10.fan, volume);
                }
                XPLMSetDataf(ctx->custom.a350.volume, volume);
                break;

            case ACF_TYP_B737_XG:
                break;

            case ACF_TYP_B757_FF:
            case ACF_TYP_B767_FF:
                if (ctx->x_plane_v11)
                {
                    //fixme
                }
                else
                {
                    XPLMSetDataf(ctx->sound.xp10.eng, volume);
                    XPLMSetDataf(ctx->sound.xp10.prp, volume);
                    XPLMSetDataf(ctx->sound.xp10.wxr, volume);
                    XPLMSetDataf(ctx->sound.xp10.wrn, volume);
                    XPLMSetDataf(ctx->sound.xp10.grd, volume);
                    XPLMSetDataf(ctx->sound.xp10.fan, volume);
                }
                XPLMSetDataf(ctx->custom.fsts.volume, volume * 0.5f);
                break;

            case ACF_TYP_B777_FF:
                if (ctx->x_plane_v11)
                {
                    //fixme
                }
                else
                {
                    XPLMSetDataf(ctx->sound.xp10.eng, volume);
                    XPLMSetDataf(ctx->sound.xp10.prp, volume);
                    XPLMSetDataf(ctx->sound.xp10.wxr, volume);
                    XPLMSetDataf(ctx->sound.xp10.wrn, volume);
                    XPLMSetDataf(ctx->sound.xp10.grd, volume);
                    XPLMSetDataf(ctx->sound.xp10.fan, volume);
                }
                XPLMSetDataf(ctx->custom.fft7.ambien, volume * 1.0f);
                XPLMSetDataf(ctx->custom.fft7.cllout, volume * 1.5f);
                XPLMSetDataf(ctx->custom.fft7.engine, volume * 1.0f);
                XPLMSetDataf(ctx->custom.fft7.volume, volume * 0.5f);
                break;

            case ACF_TYP_CL30_DD:
                if (ctx->x_plane_v11)
                {
                    //fixme
                }
                else
                {
                    XPLMSetDataf(ctx->sound.xp10.eng, volume);
                    XPLMSetDataf(ctx->sound.xp10.prp, volume);
                    XPLMSetDataf(ctx->sound.xp10.wxr, volume);
                    XPLMSetDataf(ctx->sound.xp10.wrn, volume);
                    XPLMSetDataf(ctx->sound.xp10.grd, volume);
                    XPLMSetDataf(ctx->sound.xp10.fan, volume);
                }
                XPLMSetDataf(ctx->custom.ddnn.volext, (0.25f));
                XPLMSetDatai(ctx->custom.ddnn.volmut, (0.01f > volume));
                XPLMSetDatai(ctx->custom.ddnn.volume, (int)roundf(2200.0f * sqrtf(volume)));
                break;

            default:
                if (ctx->x_plane_v11)
                {
                    //fixme
                    if (ctx->custom.absk.volume && ctx->custom.absk.mtrack &&
                        fabsf(XPLMGetDataf(ctx->custom.absk.mtrack)) > 0.001f)
                    {
//                        XPLMSetDataf(ctx->sound.xp11.eng, volume / 2.0f);
                        XPLMSetDataf(ctx->custom.absk.volume, volume);
                    }
                }
                else
                {
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

#undef V0_DEFAULT_PE
#undef V0_DEFAULT_XB
#undef V1_DEFAULT_AL
#undef V1_DEFAULT_PE
#undef V1_DEFAULT_XB
#undef V2_DEFAULT_AL
#undef V2_DEFAULT_PE
#undef V2_DEFAULT_XB
