/*
 * ACFtypes.c
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

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMUtilities.h"

#include "assert/includes.h"

#include "common/common.h"

#include "ACFtypes.h"

int uf_dref_string_read(XPLMDataRef dataref, char *string_buffer, size_t buffer_size)
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

int uf_dref_string_wrte(XPLMDataRef dataref, char *string_buffer, size_t buffer_size)
{
    if (dataref && string_buffer && buffer_size)
    {
        size_t str_len = strnlen(string_buffer, buffer_size - 2);
        XPLMSetDatab(dataref, string_buffer, 0, str_len);
        XPLMSetDatab(dataref, "", str_len, 1);
    }
    return ENOMEM;
}

const char* acf_type_get_name(acf_type type)
{
    for (int ii = 0; acf_type_kvps[ii].name != NULL; ii++)
    {
        if (acf_type_kvps[ii].acf_type_id == type)
        {
            return acf_type_kvps[ii].name;
        }
    }
    return acf_type_kvps[0].name;
}

int acf_type_init_cts(acf_info_context *info)
{
    if (info == NULL)
    {
        return ENOMEM;
    }
    if (info->ac_type == ACF_TYP_A320_FF && !info->assert.initialized)
    {
        XPLMSendMessageToPlugin(info->assert.plugin_id, XPLM_FF_MSG_GET_SHARED_INTERFACE, &info->assert.api);
        if (info->assert.api.DataVersion == NULL || info->assert.api.DataAddUpdate == NULL)
        {
            return EAGAIN;
        }
#if 0
        /* List all nodes and variables exposed by the provided SDK. */
        {
            const char *valueDescription;
            char tmp[2048], fullname[2048];
            int vID, valueID, parentValueID;
            unsigned int valueType, valueFlags;
            ndt_log("acf_type [debug] =======================\n");
            unsigned int valuesCount = info->assert.api.ValuesCount();
            ndt_log("acf_type [debug]: valuesCount: %u\n", valuesCount);
            for (unsigned int ii = 0; ii < valuesCount; ii++)
            {
                vID = valueID         = info->assert.api.ValueIdByIndex (ii);
                valueType             = info->assert.api.ValueType (valueID);
                valueFlags            = info->assert.api.ValueFlags(valueID);
                valueDescription      = info->assert.api.ValueDesc (valueID);
                sprintf(fullname, "%s", info->assert.api.ValueName (valueID));
                while (valueID && (parentValueID = info->assert.api.ValueParent(valueID)) >= 0)
                {
                    valueID = parentValueID;
                    sprintf(tmp, "%s", fullname);
                    sprintf(fullname, "%s.%s", info->assert.api.ValueName(valueID), tmp);
                }
                ndt_log("acf_type [debug]: ID: %d, name: \"%s\", desc: \"%s\", type: %u, flags: %u\n", vID, fullname, valueDescription, valueType, valueFlags);
            }
            ndt_log("acf_type [debug] =======================\n");
        }
#endif
        /* Initialize the aircraft's data references via the provided API */
        info->assert.dat.ldg_gears_lever         = XPLMFindDataRef               ("model/controls/gears_lever"                       );
        info->assert.dat.engine_lever_lt         = XPLMFindDataRef               ("model/controls/engine_lever1"                     );
        info->assert.dat.engine_reverse1         = XPLMFindDataRef               ("model/controls/engine_reverse1"                   );
        info->assert.dat.engine_reverse2         = XPLMFindDataRef               ("model/controls/engine_reverse2"                   );
        info->assert.dat.throttles_up            = XPLMFindCommand               ("sim/engines/throttle_up"                          );
        info->assert.dat.throttles_dn            = XPLMFindCommand               ("sim/engines/throttle_down"                        );
        info->assert.dat.h_brk_mximum            = XPLMFindCommand               ("sim/flight_controls/brakes_max"                   );
        info->assert.dat.h_brk_regulr            = XPLMFindCommand               ("sim/flight_controls/brakes_regular"               );
        info->assert.dat.p_brk_toggle            = XPLMFindCommand               ("sim/flight_controls/brakes_toggle_max"            );
        info->assert.dat.toggle_r_ng1            = XPLMFindCommand               ("sim/engines/thrust_reverse_toggle_1"              );
        info->assert.dat.toggle_r_ng2            = XPLMFindCommand               ("sim/engines/thrust_reverse_toggle_2"              );
        info->assert.dat.toggle_srvos            = XPLMFindCommand               ("sim/autopilot/servos_toggle"                      );
        info->assert.dat.id_s32_fmgs_fcu1_fl_lvl = info->assert.api.ValueIdByName("Aircraft.FMGS.FCU1.Altitude"                      );
        info->assert.dat.id_s32_light_autopilot1 = info->assert.api.ValueIdByName("Aircraft.FMGS.FCU1.AutoPilotLight1"               );
        info->assert.dat.id_s32_light_autopilot2 = info->assert.api.ValueIdByName("Aircraft.FMGS.FCU1.AutoPilotLight2"               );
        info->assert.dat.id_f32_p_spoilers_lever = info->assert.api.ValueIdByName("Aircraft.Cockpit.Pedestal.SpoilersLever"          );
        info->assert.dat.id_u32_efis_nav_mod_lft = info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_NavModeL.Target"      );
        info->assert.dat.id_u32_efis_nav_mod_rgt = info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_NavModeR.Target"      );
        info->assert.dat.id_u32_efis_nav_rng_lft = info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_NavRangeL.Target"     );
        info->assert.dat.id_u32_efis_nav_rng_rgt = info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_NavRangeR.Target"     );
        info->assert.dat.id_u32_fcu_tgt_alt_step = info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.FCU_AltitudeStep.Target"   );
        info->assert.dat.id_u32_emer_lights_mode = info->assert.api.ValueIdByName("Aircraft.Cockpit.Overhead.LightEmerMode.Target"   );
        info->assert.dat.id_s32_click_thr_disc_l = info->assert.api.ValueIdByName("Aircraft.Cockpit.Pedestal.EngineDisconnect1.Click");
        info->assert.dat.id_s32_click_ss_tkovr_l = info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.SidestickTakeoverL.Click"  );
        if (info->assert.dat.id_s32_fmgs_fcu1_fl_lvl <= 0 ||
            info->assert.dat.id_s32_light_autopilot1 <= 0 ||
            info->assert.dat.id_s32_light_autopilot2 <= 0 ||
            info->assert.dat.id_f32_p_spoilers_lever <= 0 ||
            info->assert.dat.id_u32_efis_nav_mod_lft <= 0 ||
            info->assert.dat.id_u32_efis_nav_mod_rgt <= 0 ||
            info->assert.dat.id_u32_efis_nav_rng_lft <= 0 ||
            info->assert.dat.id_u32_efis_nav_rng_rgt <= 0 ||
            info->assert.dat.id_u32_fcu_tgt_alt_step <= 0 ||
            info->assert.dat.id_u32_emer_lights_mode <= 0 ||
            info->assert.dat.id_s32_click_thr_disc_l <= 0 ||
            info->assert.dat.id_s32_click_ss_tkovr_l <= 0 ||
            info->assert.dat.ldg_gears_lever      == NULL ||
            info->assert.dat.engine_lever_lt      == NULL ||
            info->assert.dat.engine_reverse1      == NULL ||
            info->assert.dat.engine_reverse2      == NULL ||
            info->assert.dat.throttles_up         == NULL ||
            info->assert.dat.throttles_dn         == NULL ||
            info->assert.dat.h_brk_mximum         == NULL ||
            info->assert.dat.h_brk_regulr         == NULL ||
            info->assert.dat.p_brk_toggle         == NULL ||
            info->assert.dat.toggle_r_ng1         == NULL ||
            info->assert.dat.toggle_r_ng2         == NULL ||
            info->assert.dat.toggle_srvos         == NULL)
        {
            ndt_log("acf_type [debug]: acf_type_init_cts: can't find required data for ACF_TYP_A320_FF\n");
            return EINVAL;
        }
        info->assert.initialized = 1; return 0;
    }
    return 0;
}

int acf_type_resetall(acf_info_context *info)
{
    if (info)
    {
        info->engine_count = 0; info->engine_type1 = 0;
        info->afname[0] = '\0'; info->afpath[0] = '\0';
        info->author[0] = '\0'; info->descrp[0] = '\0';
        info->tailnb[0] = '\0'; info->icaoid[0] = '\0';
        info->ac_type = ACF_TYP_GENERIC;
        info->assert.initialized = 0;
    }
    return 0;
}

int acf_type_get_info(acf_info_context *info)
{
    if (info == NULL)
    {
        return -1;
    }

    /* re-initialize the passed structure */
    XPLMDataRef tmp = NULL; acf_type_resetall(info);

    /* get the aircraft path and model information */
    if ((tmp = XPLMFindDataRef("sim/aircraft/view/acf_author")))
    {
        uf_dref_string_read(tmp, info->author, sizeof(info->author));
    }
    if ((tmp = XPLMFindDataRef("sim/aircraft/view/acf_descrip")))
    {
        uf_dref_string_read(tmp, info->descrp, sizeof(info->descrp));
    }
    if ((tmp = XPLMFindDataRef("sim/aircraft/view/acf_tailnum")))
    {
        uf_dref_string_read(tmp, info->tailnb, sizeof(info->tailnb));
    }
    if ((info->dric = XPLMFindDataRef("sim/aircraft/view/acf_ICAO")))
    {
        uf_dref_string_read(info->dric, info->icaoid, sizeof(info->icaoid));
    }
    XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, info->afname, info->afpath);

    /* get engine count and type */
    if ((tmp = XPLMFindDataRef("sim/aircraft/engine/acf_num_engines")))
    {
        info->engine_count = XPLMGetDatai(tmp);
        if (info->engine_count > 8) info->engine_count = 8;
        if (info->engine_count < 0) info->engine_count = 0;
    }
    if ((tmp = XPLMFindDataRef("sim/aircraft/prop/acf_en_type")))
    {
        if (info->engine_count > 0)
        {
            int t[8]; XPLMGetDatavi(tmp, t, 0, info->engine_count);
            for (int i = 1; i < info->engine_count; i++)
            {
                if (t[i] != t[0])
                {
                    info->engine_count = i; break; // additional dummy engines, used by e.g. Carenado
                }
            }
            info->engine_type1 = t[0];
        }
    }

    /* check enabled plugins to determine which plane we're flying */
    do // dummy loop we can break out of
    {
        XPLMPluginID ff_pluginid = XPLMFindPluginBySignature(XPLM_FF_SIGNATURE);
        if (XPLM_NO_PLUGIN_ID != ff_pluginid)
        {
            info->assert.plugin_id = ff_pluginid;
            info->ac_type = ACF_TYP_A320_FF;
            info->assert.initialized = 0;
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("QPAC.airbus.fbw") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("QPAC.A380.airbus.fbw"))
        {
            if (!STRN_CASECMP_AUTO(info->author, "QualityPark"))
            {
                info->ac_type = ACF_TYP_A320_QP;
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "A330-300"))
            {
                info->ac_type = ACF_TYP_A330_RW;
                break;
            }
            if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("FFSTSmousehandler") ||   // 1.3.x or earlier
                XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("ru.ffsts.mousehandler")) // 1.4.x or later
            {
                info->ac_type = ACF_TYP_A350_FF;
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "Airbus A380"))
            {
                info->ac_type = ACF_TYP_A380_PH;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (QPAC.airbus.fbw)\n");
            info->ac_type = ACF_TYP_A320_QP;
            break; // still QPAC-based variant
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("bs.x737.plugin"))
        {
            if (!STRN_CASECMP_AUTO(info->descrp, "Boeing 737-800"))
            {
                info->ac_type = ACF_TYP_B737_EA;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (bs.x737.plugin)\n");
            info->ac_type = ACF_TYP_B737_EA;
            break; // still an x737 variant
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("de-ru.philippmuenzel-den_rain.757avionics") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature( "ru.flightfactor-steptosky.757767avionics") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature(                  "ru.stsff.757767avionics"))
        {
            if (!STRN_CASECMP_AUTO(info->descrp, "Boeing 757"))
            {
                info->ac_type = ACF_TYP_B757_FF;
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "Boeing 767"))
            {
                info->ac_type = ACF_TYP_B767_FF;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (757767avionics)\n");
            info->ac_type = ACF_TYP_B767_FF; // still a 757 or 767 variant
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("de.philippmuenzel.t7avionics"))
        {
            if (!STRN_CASECMP_AUTO(info->descrp, "Boeing 777"))
            {
                info->ac_type = ACF_TYP_B777_FF;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (t7avionics)\n");
            info->ac_type = ACF_TYP_B777_FF; // still a T7 variant
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("FJCC.SSGERJ"))
        {
            if (!STRN_CASECMP_AUTO(info->descrp, "E 170LR"))
            {
                info->ac_type = ACF_TYP_EMBE_SS;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (SSGERJ)\n");
            info->ac_type = ACF_TYP_EMBE_SS; // still an SSG E-Jet variant
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("Rotate.MD-80.Core"))
        {
            if (!STRN_CASECMP_AUTO(info->descrp, "Rotate MD-80"))
            {
                info->ac_type = ACF_TYP_MD80_RO;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (MD-80.Core)\n");
            info->ac_type = ACF_TYP_MD80_RO; // still a Rotate addon
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("1-sim.sasl"))
        {
            if (!STRN_CASECMP_AUTO(info->author, "JARDESIGN") &&
                !STRN_CASECMP_AUTO(info->descrp, "Airbus A320"))
            {
                info->ac_type = ACF_TYP_A320_JD;
                break;
            }
            if (!STRN_CASECMP_AUTO(info->author, "JARDESIGN") &&
                !STRN_CASECMP_AUTO(info->descrp, "Airbus A330"))
            {
                info->ac_type = ACF_TYP_A330_JD;
                break;
            }
            if (!STRN_CASECMP_AUTO(info->author, "FlyJsim") &&
                !strcasecmp       (info->icaoid, "B732"))
            {
                info->ac_type = ACF_TYP_B737_FJ;
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "Dash 8 Q400"))
            {
                info->ac_type = ACF_TYP_DH8D_FJ;
                break;
            }
            if (!STRN_CASECMP_AUTO(info->author, "Daniel Klaue") &&
                !STRN_CASECMP_AUTO(info->descrp, "ERJ-140"))
            {
                info->ac_type = ACF_TYP_ERJ1_4D;
                break;
            }
            if (!STRN_CASECMP_AUTO(info->author, "Marko Mamula"))
            {
                if (!STRN_CASECMP_AUTO(info->descrp, "Embraer E175") ||
                    !STRN_CASECMP_AUTO(info->descrp, "Embraer E195"))
                {
                    info->ac_type = ACF_TYP_EMBE_XC;
                    break;
                }
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "Sukhoy SuperJet"))
            {
                info->ac_type = ACF_TYP_SSJ1_RZ;
                break;
            }
            if (!STRN_CASECMP_AUTO(info->author, "Rob Wilson") &&
                !STRN_CASECMP_AUTO(info->descrp, "Hawker 4000"))
            {
                info->ac_type = ACF_TYP_HA4T_RW;
                break;
            }
            // fall through
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("gizmo.x-plugins.com"))
        {
            if (!STRN_CASECMP_AUTO(info->author, "IXEG") &&
                !STRN_CASECMP_AUTO(info->descrp, "Boeing 737-300"))
            {
                info->ac_type = ACF_TYP_B737_XG;
                break;
            }
            // fall through
        }
        info->ac_type = ACF_TYP_GENERIC;
        break;
    }
    while (0);

    /* sanitize ICAO aircraft type designator */
    char new_icao[sizeof(info->icaoid)] = "";
    switch (info->ac_type)
    {
        case ACF_TYP_A330_RW:
            sprintf(new_icao, "%.4s", "A333");
            break;
        case ACF_TYP_A350_FF:
            sprintf(new_icao, "%.4s", "A359");
            break;
        case ACF_TYP_HA4T_RW:
            sprintf(new_icao, "%.4s", "HA4T");
            break;
        case ACF_TYP_SSJ1_RZ:
            sprintf(new_icao, "%.4s", "SU95");
            break;
        case ACF_TYP_GENERIC:
            if (!STRN_CASECMP_AUTO(info->descrp, "A-10 Warthog"))
            {
                sprintf(new_icao, "%.4s", "A10");
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "Lisa Airplanes Akoya"))
            {
                sprintf(new_icao, "%.4s", "AKOY");
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "Mother Ship 1"))
            {
                sprintf(new_icao, "%.4s", "B52");
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "Boeing 787"))
            {
                sprintf(new_icao, "%.4s", "B788");
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "Bonanza V35B"))
            {
                sprintf(new_icao, "%.4s", "BE35");
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "King Air C90"))
            {
                sprintf(new_icao, "%.4s", "BE9L");
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "KC10"))
            {
                sprintf(new_icao, "%.4s", "DC10");
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "Epic Victory"))
            {
                sprintf(new_icao, "%.4s", "EVIC");
                break;
            }
            if (!STRN_CASECMP_AUTO(info->author, "After") && !strnlen(info->descrp, 1))
            {
                sprintf(new_icao, "%.4s", "FA7X");
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "Ferrari"))
            {
                sprintf(new_icao, "%.4s", "P180");
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "cirrus jet"))
            {
                sprintf(new_icao, "%.4s", "SF50");
                break;
            }
            if (!STRN_CASECMP_AUTO(info->descrp, "TBM 850"))
            {
                sprintf(new_icao, "%.4s", "TBM8");
                break;
            }
            // fall through
        default:
            snprintf(new_icao, sizeof(new_icao), "%s", info->icaoid);
            break;
    }
    if (strncmp(info->icaoid, new_icao, sizeof(new_icao)))
    {
        ndt_log("acf_type [info]: '%s' -> '%s'\n", info->icaoid, new_icao);
        snprintf(info->icaoid, sizeof(info->icaoid), "%s", new_icao);
        uf_dref_string_wrte(info->dric, new_icao, sizeof(new_icao));
    }

    return 0;
}
