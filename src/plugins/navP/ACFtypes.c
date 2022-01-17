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
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMUtilities.h"

#include "assert/includes.h"

#include "common/common.h"

#include "ACFtypes.h"

static acf_info_context *global_info = NULL;

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

void acf_type_totalizr(acf_info_context *info)
{
    if (info)
    {
        XPLMDataRef fuel_totalizer_sum_kg, fuel_totalizer_init_kg;
        if ((fuel_totalizer_sum_kg = XPLMFindDataRef("sim/cockpit2/fuel/fuel_totalizer_sum_kg")) &&
            (fuel_totalizer_init_kg = XPLMFindDataRef("sim/cockpit2/fuel/fuel_totalizer_init_kg")))
        {
            ndt_log("navP [info]: acf_type_totalizr: fuel totalizer reset\n");
            float fuel; acf_type_fuel_get(info, &fuel);
            XPLMSetDataf(fuel_totalizer_init_kg, fuel);
            XPLMSetDataf(fuel_totalizer_sum_kg, 0.0f);
            return;
        }
        return;
    }
    return;
}

acf_info_context* acf_type_info_get()
{
    if (global_info == NULL)
    {
        if ((global_info = malloc(sizeof(acf_info_context))) == NULL)
        {
            return NULL;
        }
        global_info->weight.zfwcgzm = XPLMFindDataRef("sim/flightmodel/misc/cgz_ref_to_default");
        global_info->weight.minimum = XPLMFindDataRef("sim/aircraft/weight/acf_m_empty");
        global_info->weight.payload = XPLMFindDataRef("sim/flightmodel/weight/m_fixed");
        global_info->weight.current = XPLMFindDataRef("sim/flightmodel/weight/m_total");
        global_info->weight.maximum = XPLMFindDataRef("sim/aircraft/weight/acf_m_max");
        global_info->fuel.pertank   = XPLMFindDataRef("sim/flightmodel/weight/m_fuel");
        global_info->fuel.tankrat   = XPLMFindDataRef("sim/aircraft/overflow/acf_tank_rat");
        global_info->fuel.maximum   = XPLMFindDataRef("sim/aircraft/weight/acf_m_fuel_tot");
        global_info->fuel.current   = XPLMFindDataRef("sim/flightmodel/weight/m_fuel_total");
        if (global_info->weight.zfwcgzm == NULL ||
            global_info->weight.minimum == NULL ||
            global_info->weight.payload == NULL ||
            global_info->weight.current == NULL ||
            global_info->weight.maximum == NULL ||
            global_info->fuel.  pertank == NULL ||
            global_info->fuel.  tankrat == NULL ||
            global_info->fuel.  maximum == NULL ||
            global_info->fuel.  current == NULL)
        {
            free(global_info);
            return NULL;
        }
        acf_type_info_reset();
    }
    return global_info;
}

static void toliss_info_reset(acf_info_context *info)
{
    if (info)
    {
        if ((info->ac_type & ACF_TYP_MASK_TOL))
        {
            info->toliss.initialized = 0;
            info->toliss.npax = NULL;
            info->toliss.paxd = NULL;
            info->toliss.fcgo = NULL;
            info->toliss.acgo = NULL;
            info->toliss.wfob = NULL;
            info->toliss.c = NULL;
            return;
        }
        return;
    }
    return;
}

static int toliss_info_init(acf_info_context *info)
{
    if (info)
    {
        if ((info->ac_type & ACF_TYP_MASK_TOL))
        {
            if (info->toliss.initialized != 1)
            {
                if (info->toliss.initialized == 0)
                {
                    info->toliss.npax = XPLMFindDataRef("AirbusFBW/NoPax");
                    info->toliss.wfob = XPLMFindDataRef("AirbusFBW/WriteFOB");
                    info->toliss.fcgo = XPLMFindDataRef("AirbusFBW/FwdCargo");
                    info->toliss.acgo = XPLMFindDataRef("AirbusFBW/AftCargo");
                    info->toliss.paxd = XPLMFindDataRef("AirbusFBW/PaxDistrib");
                    info->toliss.c = XPLMFindCommand("AirbusFBW/SetWeightAndCG");
                    if (!info->toliss.npax || !info->toliss.wfob ||
                        !info->toliss.fcgo || !info->toliss.acgo ||
                        !info->toliss.paxd || !info->toliss.c)
                    {
                        ndt_log("navP [error]: toliss_info_init: missing dataref or command\n");
                        info->toliss.initialized = -1;
                        return -1;
                    }
                    info->toliss.initialized = 1;
                    return 1;
                }
                return -1;
            }
            return 1;
        }
        return 0;
    }
    return -1;
}

int acf_type_info_reset()
{
    if (global_info)
    {
        global_info->engine_count = 0; global_info->engine_type1 = 0;
        global_info->afname[0] = '\0'; global_info->afpath[0] = '\0';
        global_info->author[0] = '\0'; global_info->descrp[0] = '\0';
        global_info->tailnb[0] = '\0'; global_info->icaoid[0] = '\0';
        toliss_info_reset(global_info); // call before reset ac_type
        global_info->ac_type = ACF_TYP_GENERIC;
        global_info->assert.initialized = 0;
        global_info->flap_detents = 0;
        global_info->up_to_date = 0;
    }
    return 0;
}

struct tank_sorter
{
    int index;
    float rat;
};

static int compare_tanks_by_volume(const void *t1, const void *t2)
{
    const struct tank_sorter *ts1 = t1;
    const struct tank_sorter *ts2 = t2;
    if (ts1->rat < .01f)
    {
        return 1; // empty tanks go to the end (we should never get them anyway)
    }
    if (ts1->rat < ts2->rat)
    {
        return -1; // tanks with lower ratio/volume refuel first
    }
    if (ts1->rat > ts2->rat)
    {
        return 1; // tanks with higher ratio/volume refuel later
    }
    return (ts1->index < ts2->index) ? -1 : 1; // ratios equal so order by index
}

acf_info_context* acf_type_info_update()
{
    if (global_info == NULL)
    {
        return acf_type_info_get();
    }
    if (global_info->up_to_date)
    {
        return global_info;
    }

    /* re-initialize the passed structure */
    XPLMDataRef tmp = NULL; acf_type_info_reset();

    /* get the aircraft path and model information */
    if ((tmp = XPLMFindDataRef("sim/aircraft/view/acf_author")))
    {
        uf_dref_string_read(tmp, global_info->author, sizeof(global_info->author));
    }
    if ((tmp = XPLMFindDataRef("sim/aircraft/view/acf_descrip")))
    {
        uf_dref_string_read(tmp, global_info->descrp, sizeof(global_info->descrp));
    }
    if ((tmp = XPLMFindDataRef("sim/aircraft/view/acf_tailnum")))
    {
        uf_dref_string_read(tmp, global_info->tailnb, sizeof(global_info->tailnb));
    }
    if ((global_info->dric = XPLMFindDataRef("sim/aircraft/view/acf_ICAO")))
    {
        uf_dref_string_read(global_info->dric, global_info->icaoid, sizeof(global_info->icaoid));
    }
    XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, global_info->afname, global_info->afpath);

    /* how many flap detents? */
    if ((tmp = XPLMFindDataRef("sim/aircraft/controls/acf_flap_detents")))
    {
        global_info->flap_detents = XPLMGetDatai(tmp);
    }

    /* get engine count and type */
    if ((tmp = XPLMFindDataRef("sim/aircraft/overflow/acf_tank_rat")))
    {
                float tank_ratio[9]; global_info->ftanks_count = 0;
                for (int i = 0, j = XPLMGetDatavf(tmp, tank_ratio, 0, 9); i < j; i++)
                {
                    if (tank_ratio[i] > .01f)
                    {
                        global_info->ftanks_count++;
                        continue;
                    }
                    continue;
                }
                if (global_info->ftanks_count == 4 && (fabsf(tank_ratio[0] - tank_ratio[1]) < .01f &&
                                                       fabsf(tank_ratio[2] - tank_ratio[3]) < .01f))
                {
                    global_info->ftanks_count = 2; // e.g. Bonanza V35B w/tip tanks
                }
    }
    if ((tmp = XPLMFindDataRef("sim/aircraft/engine/acf_num_engines")))
    {
        global_info->engine_count = XPLMGetDatai(tmp);
        if (global_info->engine_count > 8) global_info->engine_count = 8;
        if (global_info->engine_count < 0) global_info->engine_count = 0;
    }
    if ((tmp = XPLMFindDataRef("sim/aircraft/prop/acf_en_type")))
    {
        if (global_info->engine_count > 0)
        {
            int t[8]; XPLMGetDatavi(tmp, t, 0, global_info->engine_count);
            for (int i = 1; i < global_info->engine_count; i++)
            {
                if (t[i] != t[0])
                {
                    global_info->engine_count = i; break; // additional dummy engines, used by e.g. Carenado
                }
            }
            global_info->engine_type1 = t[0];
        }
    }
    switch (global_info->engine_type1)
    {
        case 4: case 5: // twin turbojet/turbofan
            if ((tmp = XPLMFindDataRef("sim/aircraft/prop/acf_revthrust_eq")))
            {
                global_info->has_rvrs_thr = XPLMGetDatai(tmp) != 0;
            }
            global_info->has_auto_thr = 0;
            global_info->has_beta_thr = 0;
            break;
        case 0: case 1: // piston engines
        case 2: case 8: // and turboprops
        case 9: // another turboprop (XP11+, not documented in Datarefs.txt)
            if ((tmp = XPLMFindDataRef("sim/aircraft/prop/acf_revthrust_eq")))
            {
                global_info->has_rvrs_thr = XPLMGetDatai(tmp) != 0;
            }
            if ((tmp = XPLMFindDataRef("sim/aircraft/overflow/acf_has_beta")))
            {
                global_info->has_beta_thr = XPLMGetDatai(tmp) != 0;
            }
            if (global_info->has_rvrs_thr == 0 && global_info->has_beta_thr == 0)
            {
                global_info->has_rvrs_thr = -1; // XXX: prop-driven w/out reverse
                global_info->has_beta_thr = -1; // XXX: prop-driven w/out reverse
            }
            global_info->has_auto_thr = 0; // TODO: case by case basis
            break;
        default:
            global_info->has_rvrs_thr = 0;
            global_info->has_beta_thr = 0;
            global_info->has_auto_thr = 0;
            break;
    }
    /*
     * XXX: disable globally; re-enable on a case-by-case basis until
     * I can get a better understanding of X-Plane's default behavior…
     */
    global_info->has_beta_thr = 0;

    /* update fuel tank characteristics */
    struct tank_sorter fuel_tank_for_sorting[9]; int lsingles[9];
    float rr[9], max_f = XPLMGetDataf(global_info->fuel.maximum);
    int iii = XPLMGetDatavf(global_info->fuel.tankrat, rr, 0, 9);
    for (int ii = 0; ii < 9 && ii < iii; ii++)
    {
        if (ii == 0)
        {
            global_info->fuel.tanks.count = 0;
        }
        if (rr[ii] > .01f)
        {
            global_info->fuel.tanks.max[ii] = rr[ii] * max_f;
            global_info->fuel.tanks.rat[ii] = rr[ii];
            fuel_tank_for_sorting[ii].rat   = rr[ii];
            fuel_tank_for_sorting[ii].index = ii;
            global_info->fuel.tanks.count++;
        }
        else
        {
            break; // we don't support holes in fuel tank configuration -- will probably never happen anyway
        }
    }
    qsort(fuel_tank_for_sorting, global_info->fuel.tanks.count, sizeof(struct tank_sorter), &compare_tanks_by_volume);
    global_info->fuel.tanks.max_kg = floorf(max_f); global_info->fuel.tanks.max_lb = floorf(max_f / 0.45359f); int lc;
    for (int ii = 0, jj = 1, kk = 0, ll = 0; ii < global_info->fuel.tanks.count;)
    {
        // all tanks are now volume-sorted, but we still need to move all single
        // tanks to the end, after any fuel tank "couples"; first, store couples
        if ((jj < global_info->fuel.tanks.count) && (fuel_tank_for_sorting[jj].rat ==
                                                     fuel_tank_for_sorting[ii].rat))
        {
            global_info->fuel.tanks.rfo[kk++] = fuel_tank_for_sorting[ii].index; ii += 2;
            global_info->fuel.tanks.rfo[kk++] = fuel_tank_for_sorting[jj].index; jj += 2;
            lc = kk; // first index after last fuel tank couple is sorted
        }
        else
        {
            lsingles[ll++] = fuel_tank_for_sorting[ii].index; ii += 1; jj+= 1;
        }
    }
    for (int ii = 0; ii < lc; ii++)
    {
        global_info->fuel.tanks.cpl[ii] = 1;
    }
    for (int ii = lc, jj = 0; ii < global_info->fuel.tanks.count; ii++, jj++)
    {
        // append single tanks to refueling order list
        global_info->fuel.tanks.rfo[ii] = lsingles[jj];
        global_info->fuel.tanks.cpl[ii] = 0;
    }
#if 0
    for (int ii = 0; ii < global_info->fuel.tanks.count; ii++)
    {
        if (ii == 0)
        {
            ndt_log("acf_types [debug]: fuel tanks (native order):\n");
        }
        ndt_log("acf_types [debug]: %d with ratio %.3f\n", ii, global_info->fuel.tanks.rat[ii]);
    }
    for (int ii = 0; ii < global_info->fuel.tanks.count; ii++)
    {
        int jj = global_info->fuel.tanks.rfo[ii];
        if (ii == 0)
        {
            ndt_log("acf_types [debug]: fuel tanks (refuel order):\n");
        }
        ndt_log("acf_types [debug]: %d with ratio %.3f\n", jj, global_info->fuel.tanks.rat[jj]);
    }
#endif

    /* check enabled plugins to determine which plane we're flying */
    do // dummy loop we can break out of
    {
        XPLMPluginID ff_pluginid = XPLMFindPluginBySignature(XPLM_FF_SIGNATURE);
        if (XPLM_NO_PLUGIN_ID != ff_pluginid)
        {
            global_info->assert.plugin_id = ff_pluginid;
            global_info->ac_type = ACF_TYP_A320_FF;
            global_info->assert.initialized = 0;
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("hotstart.cl650"))
        {
            global_info->ac_type = ACF_TYP_CL60_HS;
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature(           "QPAC.airbus.fbw") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature(      "QPAC.A380.airbus.fbw") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature(     "ToLiSs.Airbus.systems") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("XP11.ToLiss.Airbus.systems") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature(       "ToLiss.A319.systems") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature(  "XP10.ToLiss.A319.systems") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature(  "XP11.ToLiss.A319.systems") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature(  "XP10.ToLiss.A321.systems") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature(  "XP11.ToLiss.A321.systems"))
        {
            if (!STRN_CASECMP_AUTO(global_info->icaoid, "A319"))
            {
                global_info->ac_type = ACF_TYP_A319_TL;
                toliss_info_reset(global_info);
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->icaoid, "A321") ||
                !STRN_CASECMP_AUTO(global_info->icaoid, "A21N"))
            {
                global_info->ac_type = ACF_TYP_A321_TL;
                toliss_info_reset(global_info);
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->icaoid, "A346"))
            {
                global_info->ac_type = ACF_TYP_A346_TL;
                toliss_info_reset(global_info);
                break;
            }
            if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature(    "FFSTSmousehandler") || // 1.3.x or earlier
                XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("ru.ffsts.mousehandler") || // 1.4.x or later
                XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("ru.stsff.mousehandler"))   // 1.6.x or later
            {
                global_info->ac_type = ACF_TYP_A350_FF;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (AirbusFBW)\n");
            global_info->ac_type = ACF_TYP_GENERIC;
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("bs.x737.plugin"))
        {
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Boeing 737-800"))
            {
                global_info->ac_type = ACF_TYP_B737_EA;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (bs.x737.plugin)\n");
            global_info->ac_type = ACF_TYP_B737_EA;
            break; // still an x737 variant
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("de-ru.philippmuenzel-den_rain.757avionics") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature( "ru.flightfactor-steptosky.757767avionics") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature(                  "ru.stsff.757767avionics"))
        {
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Boeing 757"))
            {
                global_info->ac_type = ACF_TYP_B757_FF;
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Boeing 767"))
            {
                global_info->ac_type = ACF_TYP_B767_FF;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (757767avionics)\n");
            global_info->ac_type = ACF_TYP_B767_FF; // still a 757 or 767 variant
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("de.philippmuenzel.t7avionics"))
        {
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Boeing 777"))
            {
                global_info->ac_type = ACF_TYP_B777_FF;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (t7avionics)\n");
            global_info->ac_type = ACF_TYP_B777_FF; // still a T7 variant
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("FJCC.SSGERJ"))
        {
            if (!STRN_CASECMP_AUTO(global_info->descrp, "E 170LR"))
            {
                global_info->ac_type = ACF_TYP_EMBE_SS;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (SSGERJ)\n");
            global_info->ac_type = ACF_TYP_EMBE_SS; // still an SSG E-Jet variant
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("Rotate.MD-80.Core"))
        {
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Rotate MD-80"))
            {
                global_info->ac_type = ACF_TYP_MD80_RO;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (MD-80.Core)\n");
            global_info->ac_type = ACF_TYP_MD80_RO; // still a Rotate addon
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("ERJ_Functions") ||
            XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("Tekton_Functions"))
        {
            if (!STRN_CASECMP_AUTO(global_info->icaoid, "E175") ||
                !STRN_CASECMP_AUTO(global_info->icaoid, "E195"))
            {
                global_info->ac_type = ACF_TYP_EMBE_XC;
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->icaoid, "E35L"))
            {
                global_info->ac_type = ACF_TYP_LEGA_XC;
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Hawker 4000"))
            {
                global_info->ac_type = ACF_TYP_HA4T_RW;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (ERJ_Functions/Tekton_Functions)\n");
            global_info->ac_type = ACF_TYP_GENERIC;
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("1-sim Phenom_300"))
        {
            if (!STRN_CASECMP_AUTO(global_info->icaoid, "E55P"))
            {
                global_info->ac_type = ACF_TYP_E55P_AB;
                break;
            }
            ndt_log("acf_type [warning]: no aircraft type match despite plugin (1-sim Phenom_300)\n");
            global_info->ac_type = ACF_TYP_GENERIC;
            break;
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("1-sim.sasl"))
        {
            if (!STRN_CASECMP_AUTO(global_info->author, "Denis 'ddenn' Krupin") &&
                !STRN_CASECMP_AUTO(global_info->descrp, "Bombardier Challenger 300"))
            {
                global_info->ac_type = ACF_TYP_CL30_DD;
                break;
            }
            // fall through
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("gizmo.x-plugins.com"))
        {
            if (!STRN_CASECMP_AUTO(global_info->author, "IXEG") &&
                !STRN_CASECMP_AUTO(global_info->descrp, "Boeing 737-300"))
            {
                global_info->ac_type = ACF_TYP_B737_XG;
                break;
            }
            // fall through
        }
        if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("hotstart.tbm900"))
        {
            global_info->ac_type = ACF_TYP_TBM9_HS;
            break;
        }
        global_info->ac_type = ACF_TYP_GENERIC;
        break;
    }
    while (0);

    /* sanitize ICAO aircraft type designator */
    char new_icao[sizeof(global_info->icaoid)] = "";
    switch (global_info->ac_type)
    {
        case ACF_TYP_A350_FF:
            sprintf(new_icao, "%.4s", "A359");
            break;
        case ACF_TYP_HA4T_RW:
            sprintf(new_icao, "%.4s", "HA4T");
            break;
        case ACF_TYP_GENERIC:
            if (!STRN_CASECMP_AUTO(global_info->descrp, "A-10 Warthog"))
            {
                sprintf(new_icao, "%.4s", "A10");
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Lisa Airplanes Akoya"))
            {
                sprintf(new_icao, "%.4s", "AKOY");
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Mother Ship 1"))
            {
                sprintf(new_icao, "%.4s", "B52");
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Boeing 787"))
            {
                sprintf(new_icao, "%.4s", "B788");
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Bonanza V35B"))
            {
                sprintf(new_icao, "%.4s", "BE35");
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "King Air C90"))
            {
                sprintf(new_icao, "%.4s", "BE9L");
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "KC10"))
            {
                sprintf(new_icao, "%.4s", "DC10");
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Epic Victory"))
            {
                sprintf(new_icao, "%.4s", "EVIC");
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->author, "After") &&
                !STRN_CASECMP_AUTO(global_info->icaoid, "1945"))
            {
                sprintf(new_icao, "%.4s", "FA7X");
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "Ferrari"))
            {
                sprintf(new_icao, "%.4s", "P180");
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "cirrus jet"))
            {
                sprintf(new_icao, "%.4s", "SF50");
                break;
            }
            if (!STRN_CASECMP_AUTO(global_info->descrp, "TBM 850"))
            {
                sprintf(new_icao, "%.4s", "TBM8");
                break;
            }
            // fall through
        default:
            snprintf(new_icao, sizeof(new_icao), "%s", global_info->icaoid);
            break;
    }
    if (strncmp(global_info->icaoid, new_icao, sizeof(new_icao)))
    {
        ndt_log("acf_type [info]: '%s' -> '%s'\n", global_info->icaoid, new_icao);
        snprintf(global_info->icaoid, sizeof(global_info->icaoid), "%s", new_icao);
        uf_dref_string_wrte(global_info->dric, new_icao, sizeof(new_icao));
    }
    if (global_info->ac_type == ACF_TYP_GENERIC)
    {
        if (!STRN_CASECMP_AUTO(global_info->author, "Aerobask") || !STRN_CASECMP_AUTO(global_info->author, "Stephane Buon"))
        {
            if (!STRN_CASECMP_AUTO(global_info->icaoid, "DA62"))
            {
                global_info->thrust_presets = NVP_TP_DA62;
            }
            else if (!STRN_CASECMP_AUTO(global_info->icaoid, "EA50"))
            {
                global_info->thrust_presets = NVP_TP_EA50;
            }
            else if (!STRN_CASECMP_AUTO(global_info->icaoid, "EVIC"))
            {
                global_info->thrust_presets = NVP_TP_EVIC;
            }
            else
            {
                global_info->thrust_presets = NVP_TP_XPLM;
            }
        }
        else if (!STRN_CASECMP_AUTO(global_info->author, "Carenado"))
        {
            if (!STRN_CASECMP_AUTO(global_info->icaoid, "PC12"))
            {
                global_info->thrust_presets = NVP_TP_PC12;
            }
            else
            {
                global_info->thrust_presets = NVP_TP_XPLM;
            }
        }
        else
        {
            global_info->thrust_presets = NVP_TP_XPLM;
        }
    }
    else
    {
        global_info->thrust_presets = NVP_TP_XPLM;
    }
    switch (global_info->ac_type)
    {
        case ACF_TYP_A320_FF:
        case ACF_TYP_A319_TL:
        case ACF_TYP_A321_TL:
        case ACF_TYP_A346_TL:
        case ACF_TYP_A350_FF:
        case ACF_TYP_B737_EA:
        case ACF_TYP_B737_XG:
        case ACF_TYP_B757_FF:
        case ACF_TYP_B767_FF:
        case ACF_TYP_B777_FF:
        case ACF_TYP_EMBE_SS:
        case ACF_TYP_EMBE_XC:
        case ACF_TYP_HA4T_RW:
        case ACF_TYP_MD80_RO:
            global_info->has_auto_thr = 1;
            break;

        case ACF_TYP_CL60_HS: // apparently it has engine type 7 (tip rockets) :-D
            global_info->has_rvrs_thr = 1;
            global_info->has_beta_thr = 0;
            global_info->has_auto_thr = 1;
            break;

        default:
            switch (global_info->thrust_presets)
            {
                case NVP_TP_EA50:
                    global_info->has_auto_thr = 1;
                    break;

                default:
                    break;
            }
            break;
    }
    global_info->up_to_date = 1; return global_info;
}

int acf_type_info_acf_ctx_init()
{
    if (global_info == NULL)
    {
        return ENOMEM;
    }
    if (global_info->up_to_date == 0)
    {
        return EAGAIN;
    }
    if (global_info->ac_type == ACF_TYP_A320_FF && !global_info->assert.initialized)
    {
        XPLMSendMessageToPlugin(global_info->assert.plugin_id, XPLM_FF_MSG_GET_SHARED_INTERFACE, &global_info->assert.api);
        if (global_info->assert.api.DataVersion == NULL || global_info->assert.api.DataAddUpdate == NULL)
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
            unsigned int valuesCount = global_info->assert.api.ValuesCount();
            ndt_log("acf_type [debug]: valuesCount: %u\n", valuesCount);
            for (unsigned int ii = 0; ii < valuesCount; ii++)
            {
                vID = valueID         = global_info->assert.api.ValueIdByIndex (ii);
                valueType             = global_info->assert.api.ValueType (valueID);
                valueFlags            = global_info->assert.api.ValueFlags(valueID);
                valueDescription      = global_info->assert.api.ValueDesc (valueID);
                sprintf(fullname, "%s", global_info->assert.api.ValueName (valueID));
                while (valueID && (parentValueID = global_info->assert.api.ValueParent(valueID)) >= 0)
                {
                    valueID = parentValueID;
                    sprintf(tmp, "%s", fullname);
                    sprintf(fullname, "%s.%s", global_info->assert.api.ValueName(valueID), tmp);
                }
                ndt_log("acf_type [debug]: ID: %d, name: \"%s\", desc: \"%s\", type: %u, flags: %u\n", vID, fullname, valueDescription, valueType, valueFlags);
            }
            ndt_log("acf_type [debug] =======================\n");
        }
#endif
        /* Initialize the aircraft's data references via the provided API */
        global_info->assert.dat.ldg_gears_lever         = XPLMFindDataRef                      ("model/controls/gears_lever"                       );
        global_info->assert.dat.engine_lever_lt         = XPLMFindDataRef                      ("model/controls/engine_lever1"                     );
        global_info->assert.dat.engine_lever_rt         = XPLMFindDataRef                      ("model/controls/engine_lever2"                     );
        global_info->assert.dat.engine_reverse1         = XPLMFindDataRef                      ("model/controls/engine_reverse1"                   );
        global_info->assert.dat.engine_reverse2         = XPLMFindDataRef                      ("model/controls/engine_reverse2"                   );
        global_info->assert.dat.acf_brake_force         = XPLMFindDataRef                      ("sim/aircraft/overflow/acf_brake_co"               );
        global_info->assert.dat.throttles_up            = XPLMFindCommand                      ("sim/engines/throttle_up"                          );
        global_info->assert.dat.throttles_dn            = XPLMFindCommand                      ("sim/engines/throttle_down"                        );
        global_info->assert.dat.h_brk_mximum            = XPLMFindCommand                      ("sim/flight_controls/brakes_max"                   );
        global_info->assert.dat.h_brk_regulr            = XPLMFindCommand                      ("sim/flight_controls/brakes_regular"               );
        global_info->assert.dat.p_brk_toggle            = XPLMFindCommand                      ("sim/flight_controls/brakes_toggle_max"            );
        global_info->assert.dat.toggle_r_ng1            = XPLMFindCommand                      ("sim/engines/thrust_reverse_toggle_1"              );
        global_info->assert.dat.toggle_r_ng2            = XPLMFindCommand                      ("sim/engines/thrust_reverse_toggle_2"              );
        global_info->assert.dat.toggle_srvos            = XPLMFindCommand                      ("sim/autopilot/servos_toggle"                      );
        global_info->assert.dat.id_s32_acft_request_chk = global_info->assert.api.ValueIdByName("Aircraft.ShocksRequest"                           );
        global_info->assert.dat.id_s32_acft_request_gpu = global_info->assert.api.ValueIdByName("Aircraft.ExtPowerRequest"                         );
        global_info->assert.dat.id_f32_acft_dryweightkg = global_info->assert.api.ValueIdByName("Aircraft.DryOperationalWeight"                    );
        global_info->assert.dat.id_f32_acft_dryweightcg = global_info->assert.api.ValueIdByName("Aircraft.DryOperationalCenter"                    );
        global_info->assert.dat.id_f32_acft_payload_cgz = global_info->assert.api.ValueIdByName("Aircraft.PayloadCenterZ"                          );
        global_info->assert.dat.id_f32_acft_payload_kgs = global_info->assert.api.ValueIdByName("Aircraft.PayloadWeight"                           );
        global_info->assert.dat.id_f32_acft_fuel_outerl = global_info->assert.api.ValueIdByName("Aircraft.FuelOuterL"                              );
        global_info->assert.dat.id_f32_acft_fuel_outerr = global_info->assert.api.ValueIdByName("Aircraft.FuelOuterR"                              );
        global_info->assert.dat.id_f32_acft_fuel_innerl = global_info->assert.api.ValueIdByName("Aircraft.FuelInnerL"                              );
        global_info->assert.dat.id_f32_acft_fuel_innerr = global_info->assert.api.ValueIdByName("Aircraft.FuelInnerR"                              );
        global_info->assert.dat.id_f32_acft_fuel_center = global_info->assert.api.ValueIdByName("Aircraft.FuelCenter"                              );
        global_info->assert.dat.id_s32_light_autopilot1 = global_info->assert.api.ValueIdByName("Aircraft.FMGS.FCU1.AutoPilotLight1"               );
        global_info->assert.dat.id_s32_light_autopilot2 = global_info->assert.api.ValueIdByName("Aircraft.FMGS.FCU1.AutoPilotLight2"               );
        global_info->assert.dat.id_f32_p_engines_lever1 = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Pedestal.EngineLever1"           );
        global_info->assert.dat.id_f32_p_engines_lever2 = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Pedestal.EngineLever2"           );
        global_info->assert.dat.id_f32_p_spoilers_lever = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Pedestal.SpoilersLever"          );
        global_info->assert.dat.id_u32_efis_nav_mod_lft = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_NavModeL.Target"      );
        global_info->assert.dat.id_u32_efis_nav_mod_rgt = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_NavModeR.Target"      );
        global_info->assert.dat.id_u32_efis_nav_rng_lft = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_NavRangeL.Target"     );
        global_info->assert.dat.id_u32_efis_nav_rng_rgt = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.EFIS_NavRangeR.Target"     );
        global_info->assert.dat.id_u32_fcu_tgt_alt_step = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.FCU_AltitudeStep.Target"   );
        global_info->assert.dat.id_u32_light_mode_belts = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Overhead.LightBelts.Target"      );
        global_info->assert.dat.id_u32_light_mode_emerg = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Overhead.LightEmerMode.Target"   );
        global_info->assert.dat.id_u32_light_mode_smoke = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Overhead.LightSmoke.Target"      );
        global_info->assert.dat.id_u32_light_mode_strob = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Overhead.LightStrobe.Target"     );
        global_info->assert.dat.id_s32_click_ss_tkovr_l = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Panel.SidestickTakeoverL.Click"  );
        global_info->assert.dat.id_s32_click_thr_disc_l = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.Pedestal.EngineDisconnect1.Click");
        global_info->assert.dat.id_u32_overhead_rmp3pow = global_info->assert.api.ValueIdByName("Aircraft.Cockpit.RMP3.Power.Target"               );
        if (global_info->assert.dat.id_s32_acft_request_chk <= 0 ||
            global_info->assert.dat.id_s32_acft_request_gpu <= 0 ||
            global_info->assert.dat.id_f32_acft_dryweightkg <= 0 ||
            global_info->assert.dat.id_f32_acft_dryweightcg <= 0 ||
            global_info->assert.dat.id_f32_acft_payload_cgz <= 0 ||
            global_info->assert.dat.id_f32_acft_payload_kgs <= 0 ||
            global_info->assert.dat.id_f32_acft_fuel_outerl <= 0 ||
            global_info->assert.dat.id_f32_acft_fuel_outerr <= 0 ||
            global_info->assert.dat.id_f32_acft_fuel_innerl <= 0 ||
            global_info->assert.dat.id_f32_acft_fuel_innerr <= 0 ||
            global_info->assert.dat.id_f32_acft_fuel_center <= 0 ||
            global_info->assert.dat.id_s32_light_autopilot1 <= 0 ||
            global_info->assert.dat.id_s32_light_autopilot2 <= 0 ||
            global_info->assert.dat.id_f32_p_engines_lever1 <= 0 ||
            global_info->assert.dat.id_f32_p_engines_lever2 <= 0 ||
            global_info->assert.dat.id_f32_p_spoilers_lever <= 0 ||
            global_info->assert.dat.id_u32_efis_nav_mod_lft <= 0 ||
            global_info->assert.dat.id_u32_efis_nav_mod_rgt <= 0 ||
            global_info->assert.dat.id_u32_efis_nav_rng_lft <= 0 ||
            global_info->assert.dat.id_u32_efis_nav_rng_rgt <= 0 ||
            global_info->assert.dat.id_u32_fcu_tgt_alt_step <= 0 ||
            global_info->assert.dat.id_u32_light_mode_belts <= 0 ||
            global_info->assert.dat.id_u32_light_mode_emerg <= 0 ||
            global_info->assert.dat.id_u32_light_mode_smoke <= 0 ||
            global_info->assert.dat.id_u32_light_mode_strob <= 0 ||
            global_info->assert.dat.id_s32_click_ss_tkovr_l <= 0 ||
            global_info->assert.dat.id_s32_click_thr_disc_l <= 0 ||
            global_info->assert.dat.id_u32_overhead_rmp3pow <= 0 ||
            global_info->assert.dat.ldg_gears_lever      == NULL ||
            global_info->assert.dat.engine_lever_lt      == NULL ||
            global_info->assert.dat.engine_reverse1      == NULL ||
            global_info->assert.dat.engine_reverse2      == NULL ||
            global_info->assert.dat.acf_brake_force      == NULL ||
            global_info->assert.dat.throttles_up         == NULL ||
            global_info->assert.dat.throttles_dn         == NULL ||
            global_info->assert.dat.h_brk_mximum         == NULL ||
            global_info->assert.dat.h_brk_regulr         == NULL ||
            global_info->assert.dat.p_brk_toggle         == NULL ||
            global_info->assert.dat.toggle_r_ng1         == NULL ||
            global_info->assert.dat.toggle_r_ng2         == NULL ||
            global_info->assert.dat.toggle_srvos         == NULL)
        {
            ndt_log("acf_type [debug]: info_init_contexts: can't find required data for ACF_TYP_A320_FF\n");
            return EINVAL;
        }
        global_info->assert.dat.acf_brake_force_nomin = XPLMGetDataf(global_info->assert.dat.acf_brake_force);
        global_info->assert.initialized = 1; return 0;
    }
    return 0;
}

int acf_type_is_engine_running(void)
{
    if (global_info && global_info->up_to_date)
    {
        if (global_info->ac_type == ACF_TYP_CL60_HS)
        {
            XPLMDataRef ref;
            if ((ref = XPLMFindDataRef("CL650/pedestal/throttle/shutoff_L_value")))
            {
                if (XPLMGetDatai(ref) > 0)
                {
                    return 1;
                }
            }
            if ((ref = XPLMFindDataRef("CL650/pedestal/throttle/shutoff_R_value")))
            {
                if (XPLMGetDatai(ref) > 0)
                {
                    return 1;
                }
            }
        }
        else
        {
            XPLMDataRef ref;
            if ((ref = XPLMFindDataRef("sim/flightmodel/engine/ENGN_running")))
            {
                int nbe, run[9];
                if ((nbe = XPLMGetDatavi(ref, run, 0, global_info->engine_count)))
                {
                    for (int i = 0; i < nbe && i < global_info->engine_count; i++)
                    {
                        if (run[i])
                        {
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

int acf_type_load_get(acf_info_context *info, float *weight)
{
    if (info == NULL || weight == NULL)
    {
        return ENOMEM;
    }
    if (info->up_to_date == 0)
    {
        return EINVAL;
    }
    if (toliss_info_init(info) < 0)
    {
        return EINVAL;
    }
    if (info->ac_type == ACF_TYP_A320_FF)
    {
        if (global_info->assert.initialized == 0)
        {
            return EINVAL;
        }
        global_info->assert.api.ValueGet(global_info->assert.dat.id_f32_acft_payload_kgs, weight);
        return 0;
    }
    *weight = XPLMGetDataf(global_info->weight.payload);
    return 0;
}

int acf_type_load_set(acf_info_context *info, float *weight)
{
    if (info == NULL || weight == NULL)
    {
        return ENOMEM;
    }
    if (info->up_to_date == 0)
    {
        return EINVAL;
    }
    if (*weight < 0.0f)
    {
        return ERANGE;
    }
    if (toliss_info_init(info) < 0)
    {
        return EINVAL;
    }
    if (toliss_info_init(info) == 1)
    {
        float      load, pax_dist_zfcg;
        int   max_pax_count, pax_count;
        float max_cargo_fwd, cargo_fwd;
        float max_cargo_aft, cargo_aft;
        float max_cargo_all, cargo_all;
        if ((load = *weight) < 0.0f)
        {
            (load = 0.0f);
        }
        /*
         * We must distribute the load via the ISCS load manager.
         */
        switch (info->ac_type)
        {
            case ACF_TYP_A319_TL:
                /*
                 *  Pseudo-random value between 0.4375f (somewhat forward) and 0.6875f (somewhat aft).
                 */
                srand(time(NULL));
                pax_dist_zfcg = 0.4375f + (0.6875f - 0.4375f) * rand() / (float)RAND_MAX;
                max_cargo_fwd = 2268.0f;
                max_cargo_aft = 4518.0f;
                max_pax_count = 110; // ceil((58,500 - 2200 - 4500 - 40,800) / 100)
                break;
            case ACF_TYP_A321_TL:
                /*
                 *  Pseudo-random value between 0.4375f (somewhat forward) and 0.6875f (somewhat aft).
                 */
                srand(time(NULL));
                pax_dist_zfcg = 0.4375f + (0.6875f - 0.4375f) * rand() / (float)RAND_MAX;
                max_cargo_fwd = 5670.0f;
                max_cargo_aft = 7167.0f;
//              max_pax_count = 170; // ceil((79,200 - 5600 - 7100 - 49,500) / 100)
                max_pax_count = 174; // ceil((77,800 - 5600 - 7100 - 47,700) / 100)
                break;
            case ACF_TYP_A346_TL:
                /*
                 *  Pseudo-random value between 0.4375f (somewhat forward) and 0.6875f (somewhat aft).
                 */
                srand(time(NULL));
                pax_dist_zfcg = 0.4375f + (0.6875f - 0.4375f) * rand() / (float)RAND_MAX;
                max_cargo_fwd = 30482.0f;
                max_cargo_aft = 26329.0f;
                max_pax_count = 328; // ceil(((251,000 - 185,500) / 2) / 100)
                break;
            default:
                return EINVAL;
        }
        if (max_pax_count < (pax_count = (int)(load / 100.0f)))
        {
            pax_count = max_pax_count;
        }
        if ((max_cargo_all = max_cargo_fwd + max_cargo_aft) < (cargo_all = (load - ((float)pax_count * 100.0f))))
        {
            cargo_all = max_cargo_all;
        }
        ndt_log("navP [info]: ToLiSs: payload: %.0f, pax: %d, cargo: %.0f, discarded: %.0f\n",
                load, pax_count, cargo_all, load - ((float)pax_count * 100.0f) - cargo_all);
        XPLMSetDataf(info->toliss.fcgo, cargo_all * max_cargo_fwd / max_cargo_all);
        XPLMSetDataf(info->toliss.acgo, cargo_all * max_cargo_aft / max_cargo_all);
        *weight = cargo_all + ((float)pax_count * 100.0f);
        XPLMSetDataf(info->toliss.paxd, pax_dist_zfcg);
        XPLMSetDatai(info->toliss.npax, pax_count);
        XPLMCommandOnce(info->toliss.c);
        return 0;
    }
    if (info->ac_type == ACF_TYP_A320_FF)
    {
        if (global_info->assert.initialized == 0)
        {
            return EINVAL;
        }
        global_info->assert.api.ValueSet(global_info->assert.dat.id_f32_acft_payload_kgs, weight);
        // TODO: compute and set id_f32_acft_payload_cgz (offset in meters)
        return acf_type_load_get(info, weight);
    }
    XPLMSetDataf(global_info->weight.payload, *weight);
    return acf_type_load_get(info, weight);
}

int acf_type_zfwt_get(acf_info_context *info, float *weight)
{
    if (info == NULL || weight == NULL)
    {
        return ENOMEM;
    }
    if (info->up_to_date == 0)
    {
        return EINVAL;
    }
    if (toliss_info_init(info) < 0)
    {
        return EINVAL;
    }
    if (toliss_info_init(info) == 1)
    {
        *weight = XPLMGetDataf(info->weight.current) - XPLMGetDataf(info->fuel.current);
        return 0;
    }
    if (info->ac_type == ACF_TYP_A320_FF)
    {
        if (global_info->assert.initialized == 0)
        {
            return EINVAL;
        }
        assert_context *ac = &global_info->assert; float oew, p;
        ac->api.ValueGet(ac->dat.id_f32_acft_dryweightkg, &oew);
        ac->api.ValueGet(ac->dat.id_f32_acft_payload_kgs, &p);
        *weight = (oew + p);
        return 0;
    }
    float mptywt = XPLMGetDataf(global_info->weight.minimum);
    float loadwt = XPLMGetDataf(global_info->weight.payload);
    *weight = mptywt + loadwt;
    return 0;
}

int acf_type_zfwt_set(acf_info_context *info, float *weight)
{
    if (info == NULL || weight == NULL)
    {
        return ENOMEM;
    }
    if (info->up_to_date == 0)
    {
        return EINVAL;
    }
    if (*weight < 0.0f)
    {
        return ERANGE;
    }
    if (toliss_info_init(info) < 0)
    {
        return EINVAL;
    }
    if (toliss_info_init(info) == 1)
    {
        /*
         * We have a plugin-driven OEW, let's compute it.
         */
        XPLMSetDataf(info->toliss.fcgo, 0.0f);
        XPLMSetDataf(info->toliss.acgo, 0.0f);
        XPLMSetDatai(info->toliss.npax, 0);
        XPLMCommandOnce(info->toliss.c);
        float oew = XPLMGetDataf(info->weight.minimum) + XPLMGetDataf(info->weight.payload);
        ndt_log("navP [info]: ToLiSs: actual OEW %.0f\n", oew);
        /*
         * Now let's compute the actual payload to match requested ZFW.
         */
        float load = *weight - oew;
        /*
         * Delegate actual laoding procedure.
         */
        int ret = acf_type_load_set(info, &load);
        if (ret)
        {
            return ret;
        }
        *weight = oew + load;
        return 0;
    }
    float zfwt, load; int ret;
    if ((ret = acf_type_zfwt_get(info, &zfwt)) ||
        (ret = acf_type_load_get(info, &load)))
    {
        return ret;
    }
    else
    {
        *weight -= zfwt; load += *weight;
    }
    if (load < 0.0f)
    {
        load = 0.0f;
    }
    if ((ret = acf_type_load_set(info, &load)))
    {
        return ret;
    }
    return acf_type_zfwt_get(info, weight);
}

int acf_type_oewt_get(acf_info_context *info, float *weight)
{
    if (info == NULL || weight == NULL)
    {
        return ENOMEM;
    }
    if (info->up_to_date == 0)
    {
        return EINVAL;
    }
    if (toliss_info_init(info) < 0)
    {
        return EINVAL;
    }
    if (info->ac_type == ACF_TYP_A320_FF)
    {
        if (global_info->assert.initialized == 0)
        {
            return EINVAL;
        }
        assert_context *ac = &global_info->assert;
        ac->api.ValueGet(ac->dat.id_f32_acft_dryweightkg, weight);
        return 0;
    }
    *weight = XPLMGetDataf(global_info->weight.minimum);
    return 0;
}

int acf_type_grwt_get(acf_info_context *info, float *weight)
{
    if (info == NULL || weight == NULL)
    {
        return ENOMEM;
    }
    if (info->up_to_date == 0)
    {
        return EINVAL;
    }
    if (toliss_info_init(info) < 0)
    {
        return EINVAL;
    }
    if (toliss_info_init(info) == 1)
    {
        *weight = XPLMGetDataf(info->weight.current);
        return 0;
    }
    float oewt, load, fuel; int ret;
    if ((ret = acf_type_oewt_get(info, &oewt)) ||
        (ret = acf_type_load_get(info, &load)) ||
        (ret = acf_type_fuel_get(info, &fuel)))
    {
        return ret;
    }
    else
    {
        /*
         * must compute gross weight ourselves: X-Plane does not recompute
         * the relevant dataref immediately -- we may be adjusting several
         * applicable parameters before a flight loop gets to recompute GW
         */
        *weight = oewt + load + fuel;
    }
    return 0;
}

int acf_type_fmax_get(acf_info_context *info, float *weight)
{
    if (info == NULL || weight == NULL)
    {
        return ENOMEM;
    }
    if (info->up_to_date == 0)
    {
        return EINVAL;
    }
    if (toliss_info_init(info) < 0)
    {
        return EINVAL;
    }
    if (info->ac_type == ACF_TYP_A320_FF)
    {
        if (global_info->assert.initialized == 0)
        {
            return EINVAL;
        }
        *weight = XPLMGetDataf(global_info->fuel.maximum); // TODO: use API?
        return 0;
    }
    *weight = XPLMGetDataf(global_info->fuel.maximum);
    return 0;
}

static void refuel(acf_info_context *info, float by)
{
    float src[9], max[9], dst[9];
    int count = info->fuel.tanks.count, tank_idx = 0;
    XPLMGetDatavf(info->fuel.pertank, src, 0, count);
    for (int i = 0; i < count; i++)
    {
        float m = info->fuel.tanks.max[i] - src[i];
        if (0.0f > m) // should never happen
        {
            max[i] = 0.0f;
        }
        else
        {
            max[i] = m; // per-tank remaining capacity
        }
        dst[i] = 0.00f; // per-tank fuel *added*
    }
    if (!(global_info->engine_count % 3) && // 3 eng., 3 tanks etc.
        !(global_info->fuel.tanks.count % global_info->engine_count))
    {
        // special case: dumb re-fuel
        for (int i = 0; i < count; i++)
        {
            dst[i] = by * info->fuel.tanks.rat[i];
            if (dst[i] > max[i]) dst[i] = max[i];
        }
        goto do_refuel;
    }
    while (tank_idx < count && info->fuel.tanks.cpl[tank_idx] > 0)
    {
        if (by <= 0.0f)
        {
            goto do_refuel; // no more fuel to add
        }
        int index2 = info->fuel.tanks.rfo[tank_idx + 1];
        int index1 = info->fuel.tanks.rfo[tank_idx];
        float max2 = max[index1] + max[index2];
        if (((max2) <= 0.0f))
        {
            tank_idx += 2; continue; // already full
        }
        if (((max2 - by) <= 0.0f))
        {
            dst[index1] = max[index1]; by -= dst[index1];
            dst[index2] = max[index2]; by -= dst[index2];
            tank_idx += 2; continue; // tank couple full
        }
        if (src[index1] != src[index2] && fabsf(src[index1] - src[index2]) < 100.0f)
        {
            // reasonably small fuel imbalance: lets' fix it
            max[index1] = (max[index1] + max[index2]) / 2.0f;
            src[index1] = (src[index1] + src[index2]) / 2.0f;
            max[index2] = (max[index1]); src[index2] = (src[index1]);
        }
        dst[index1] += by / 2.0f; if (dst[index1] > max[index1]) dst[index1] = max[index1];
        dst[index2] += by / 2.0f; if (dst[index2] > max[index2]) dst[index2] = max[index2];
        goto do_refuel; // all fuel was assigned
    }
    while (tank_idx < count)
    {
        if (by <= 0.0f)
        {
            goto do_refuel; // no fuel remaining
        }
        int index = info->fuel.tanks.rfo[tank_idx];
        if ((max[index]) <= 0.0f)
        {
            tank_idx += 1; continue; // already full
        }
        if ((max[index] - by) <= 0.0f)
        {
            dst[index] = max[index]; by -= dst[index];
            tank_idx += 1; continue; // tank now full
        }
        dst[index] += by; if (dst[index] > max[index]) dst[index] = max[index];
        goto do_refuel; // all fuel was assigned
    }
do_refuel:
    for (int i = 0; i < count; i++)
    {
        dst[i] += src[i];
    }
    XPLMSetDatavf(info->fuel.pertank, dst, 0, count);
}

static void defuel(acf_info_context *info, float by)
{
    float fuelqt[9];
    int count = info->fuel.tanks.count, idx = count - 1;
    XPLMGetDatavf(info->fuel.pertank, fuelqt, 0, count);
    if (!(global_info->engine_count % 3) && // 3 eng., 3 tanks etc.
        !(global_info->fuel.tanks.count % global_info->engine_count))
    {
        // special case: dumb de-fuel
        for (int i = 0; i < count; i++)
        {
            fuelqt[i] -= by * info->fuel.tanks.rat[i];
            if (fuelqt[i] < 0.0f) fuelqt[i] = 0.0f;
        }
        goto do_defuel;
    }
    while (idx >= 0 && info->fuel.tanks.cpl[idx] <= 0)
    {
        if (by <= 0.0f)
        {
            goto do_defuel; // no more fuel to get
        }
        int index = info->fuel.tanks.rfo[idx];
        if ((fuelqt[index]) <= 0.0f)
        {
            idx -= 1; continue; // already empty
        }
        if ((fuelqt[index] - by) <= 0.0f)
        {
            by -= fuelqt[index]; fuelqt[index] = 0.0f;
            idx -= 1; continue; // this tank is empty
        }
        fuelqt[index] -= by; if (fuelqt[index] < 0.0f) fuelqt[index] = 0.0f;
        goto do_defuel; // all fuel was removed
    }
    while (idx >= 1)
    {
        if (by <= 0.0f)
        {
            goto do_defuel; // no more fuel to get
        }
        int index2 = info->fuel.tanks.rfo[idx];
        int index1 = info->fuel.tanks.rfo[idx - 1];
        float qty2 = fuelqt[index1] + fuelqt[index2];
        if (((qty2) <= 0.0f))
        {
            idx -= 2; continue; // already empty
        }
        if (((qty2 - by) <= 0.0f))
        {
            by -= fuelqt[index1]; fuelqt[index1] = 0.0f;
            by -= fuelqt[index2]; fuelqt[index2] = 0.0f;
            idx -= 2; continue; // both tanks now empty
        }
        if (fuelqt[index1] != fuelqt[index2] && fabsf(fuelqt[index1] - fuelqt[index2]) < 100.0f)
        {
            // reasonably small fuel imbalance: lets' fix it
            fuelqt[index1] = (fuelqt[index1] + fuelqt[index2]) / 2.0f;
            fuelqt[index2] = (fuelqt[index1]);
        }
        fuelqt[index1] -= by / 2.0f; if (fuelqt[index1] < 0.0f) fuelqt[index1] = 0.0f;
        fuelqt[index2] -= by / 2.0f; if (fuelqt[index2] < 0.0f) fuelqt[index2] = 0.0f;
        goto do_defuel; // all fuel was removed
    }
do_defuel:
    XPLMSetDatavf(info->fuel.pertank, fuelqt, 0, count);
}

int acf_type_fuel_get(acf_info_context *info, float *weight)
{
    if (info == NULL || weight == NULL)
    {
        return ENOMEM;
    }
    if (info->up_to_date == 0)
    {
        return EINVAL;
    }
    if (toliss_info_init(info) < 0)
    {
        return EINVAL;
    }
    if (info->ac_type == ACF_TYP_A320_FF)
    {
        if (global_info->assert.initialized == 0)
        {
            return EINVAL;
        }
        float tank[5]; assert_context *ac = &global_info->assert;
        ac->api.ValueGet(ac->dat.id_f32_acft_fuel_outerl, &tank[0]);
        ac->api.ValueGet(ac->dat.id_f32_acft_fuel_outerr, &tank[1]);
        ac->api.ValueGet(ac->dat.id_f32_acft_fuel_innerl, &tank[2]);
        ac->api.ValueGet(ac->dat.id_f32_acft_fuel_innerr, &tank[3]);
        ac->api.ValueGet(ac->dat.id_f32_acft_fuel_center, &tank[4]);
        *weight = (tank[0] + tank[1] + tank[2] + tank[3] + tank[4]);
        return 0;
    }
    *weight = XPLMGetDataf(global_info->fuel.current);
    return 0;
}

int acf_type_fuel_set(acf_info_context *info, float *weight)
{
    if (info == NULL || weight == NULL)
    {
        return ENOMEM;
    }
    if (info->up_to_date == 0)
    {
        return EINVAL;
    }
    if (*weight < 0.0f)
    {
        return ERANGE;
    }
    if (toliss_info_init(info) < 0)
    {
        return EINVAL;
    }
    if (toliss_info_init(info) == 1)
    {
        float fmax = XPLMGetDataf(global_info->fuel.maximum);
        if (*weight > (fmax - 40.0f))
        {
            *weight = (fmax - 40.0f);
        }
        XPLMSetDataf(info->toliss.wfob, *weight);
        return acf_type_fuel_get (info,  weight);
    }
    if (info->ac_type == ACF_TYP_A320_FF)
    {
        if (global_info->assert.initialized == 0)
        {
            return EINVAL;
        }
        float maxi[5], remg, tank[5]; assert_context *ac = &global_info->assert;
        global_info->fuel.tanks.max[1] -= 152.6; // compensate for ACF/API difference
        global_info->fuel.tanks.max[2] -= 152.6; // compensate for ACF/API difference
        maxi[0] = global_info->fuel.tanks.max[3] - (tank[0] = 60.0f); // minimum fuel
        maxi[1] = global_info->fuel.tanks.max[4] - (tank[1] = 60.0f); // minimum fuel
        maxi[2] = global_info->fuel.tanks.max[1] - (tank[2] = 60.0f); // minimum fuel
        maxi[3] = global_info->fuel.tanks.max[2] - (tank[3] = 60.0f); // minimum fuel
        maxi[4] = global_info->fuel.tanks.max[0] - (tank[4] = 60.0f); // minimum fuel
        if ((remg = *weight - (tank[0] + tank[1] + tank[2] + tank[3] + tank[4])) < 0)
        {
            goto assert_done;
        }
        if ((remg - maxi[0] - maxi[1]) < 0.0f)
        {
            tank[0] += remg / 2.0f;
            tank[1] += remg / 2.0f;
            goto assert_done;
        }
        else
        {
            tank[0] += maxi[0]; remg -= maxi[0];
            tank[1] += maxi[1]; remg -= maxi[1];
        }
        if ((remg - maxi[2] - maxi[3]) < 0.0f)
        {
            tank[2] += remg / 2.0f;
            tank[3] += remg / 2.0f;
            goto assert_done;
        }
        else
        {
            tank[2] += maxi[2]; remg -= maxi[2];
            tank[3] += maxi[3]; remg -= maxi[3];
        }
        if ((remg - maxi[4]) < 0.0f)
        {
            tank[4] += remg;
            goto assert_done;
        }
        else
        {
            tank[4] += maxi[4];
        }
    assert_done:
        ac->api.ValueSet(ac->dat.id_f32_acft_fuel_outerl, &tank[0]);
        ac->api.ValueSet(ac->dat.id_f32_acft_fuel_outerr, &tank[1]);
        ac->api.ValueSet(ac->dat.id_f32_acft_fuel_innerl, &tank[2]);
        ac->api.ValueSet(ac->dat.id_f32_acft_fuel_innerr, &tank[3]);
        ac->api.ValueSet(ac->dat.id_f32_acft_fuel_center, &tank[4]);
        return acf_type_fuel_get(info, weight);
    }
    float diff = *weight - XPLMGetDataf(global_info->fuel.current);
    if (0.0f < diff)
    {
        refuel(info, +diff);
    }
    if (0.0f > diff)
    {
        defuel(info, -diff);
    }
    return acf_type_fuel_get(info, weight);
}
