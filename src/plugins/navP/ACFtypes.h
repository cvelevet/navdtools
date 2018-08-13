/*
 * ACFtypes.h
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

#ifndef ACF_TYPES_H
#define ACF_TYPES_H

#include <string.h>

#include "XPLM/XPLMUtilities.h"

#include "assert/includes.h"

// some addons' datarefs have a trailing space for no reason :-(
#define STRN_CASECMP_AUTO(s1, s2) strncasecmp(s1, s2, strlen(s2))

typedef enum
{
        ACF_TYP_GENERIC = 0x0000000,
        ACF_TYP_A319_TL = 0x0000001,
        ACF_TYP_A320_FF = 0x0000002,
        ACF_TYP_A320_JD = 0x0000004,
        ACF_TYP_A320_QP = 0x0000008,
        ACF_TYP_A330_JD = 0x0000010,
        ACF_TYP_A330_RW = 0x0000020,
        ACF_TYP_A350_FF = 0x0000040,
        ACF_TYP_A380_PH = 0x0000080,
        ACF_TYP_B727_FJ = 0x0000100,
        ACF_TYP_B737_EA = 0x0000200,
        ACF_TYP_B737_FJ = 0x0000400,
        ACF_TYP_B737_XG = 0x0000800,
        ACF_TYP_B757_FF = 0x0001000,
        ACF_TYP_B767_FF = 0x0002000,
        ACF_TYP_B777_FF = 0x0004000,
        ACF_TYP_PHOLDR1 = 0x0008000,
        ACF_TYP_DH8D_FJ = 0x0010000,
        ACF_TYP_EMBE_SS = 0x0020000,
        ACF_TYP_EMBE_XC = 0x0040000,
        ACF_TYP_ERJ1_4D = 0x0080000,
        ACF_TYP_SSJ1_RZ = 0x0100000,
        ACF_TYP_HA4T_RW = 0x0200000,
        ACF_TYP_MD80_RO = 0x0400000,
        ACF_TYP_PHOLDR2 = 0x0808000,
}
acf_type;
#define ACF_TYP_MASK_JDN (ACF_TYP_A320_JD|ACF_TYP_A330_JD)
#define ACF_TYP_MASK_QPC (ACF_TYP_A319_TL|ACF_TYP_A320_QP|ACF_TYP_A330_RW|ACF_TYP_A350_FF|ACF_TYP_A380_PH)

static struct
{
    int  acf_type_id;
    const char* name;
}
const acf_type_kvps[] =
{
    { ACF_TYP_GENERIC, "ACF_TYP_GENERIC", },
    { ACF_TYP_A319_TL, "ACF_TYP_A319_TL", },
    { ACF_TYP_A320_FF, "ACF_TYP_A320_FF", },
    { ACF_TYP_A320_JD, "ACF_TYP_A320_JD", },
    { ACF_TYP_A320_QP, "ACF_TYP_A320_QP", },
    { ACF_TYP_A330_JD, "ACF_TYP_A330_JD", },
    { ACF_TYP_A330_RW, "ACF_TYP_A330_RW", },
    { ACF_TYP_A350_FF, "ACF_TYP_A350_FF", },
    { ACF_TYP_A380_PH, "ACF_TYP_A380_PH", },
    { ACF_TYP_B727_FJ, "ACF_TYP_B727_FJ", },
    { ACF_TYP_B737_EA, "ACF_TYP_B737_EA", },
    { ACF_TYP_B737_FJ, "ACF_TYP_B737_FJ", },
    { ACF_TYP_B737_XG, "ACF_TYP_B737_XG", },
    { ACF_TYP_B757_FF, "ACF_TYP_B757_FF", },
    { ACF_TYP_B767_FF, "ACF_TYP_B767_FF", },
    { ACF_TYP_B777_FF, "ACF_TYP_B777_FF", },
    { ACF_TYP_PHOLDR1, "ACF_TYP_PHOLDR1", },
    { ACF_TYP_DH8D_FJ, "ACF_TYP_DH8D_FJ", },
    { ACF_TYP_EMBE_SS, "ACF_TYP_EMBE_SS", },
    { ACF_TYP_EMBE_XC, "ACF_TYP_EMBE_XC", },
    { ACF_TYP_ERJ1_4D, "ACF_TYP_ERJ1_4D", },
    { ACF_TYP_SSJ1_RZ, "ACF_TYP_SSJ1_RZ", },
    { ACF_TYP_HA4T_RW, "ACF_TYP_HA4T_RW", },
    { ACF_TYP_MD80_RO, "ACF_TYP_MD80_RO", },
    { ACF_TYP_PHOLDR2, "ACF_TYP_PHOLDR2", },
    { 0,                            NULL, },
};

typedef struct
{
    SharedValuesInterface api;
    XPLMPluginID    plugin_id;
    int           initialized;
    struct
    {
        XPLMCommandRef throttles_up;
        XPLMCommandRef throttles_dn;
        XPLMCommandRef p_brk_toggle;
        XPLMCommandRef h_brk_regulr;
        XPLMCommandRef h_brk_mximum;
        XPLMCommandRef toggle_r_ng1;
        XPLMCommandRef toggle_r_ng2;
        XPLMCommandRef toggle_srvos;
        XPLMDataRef ldg_gears_lever;
        XPLMDataRef engine_lever_lt;
        XPLMDataRef engine_lever_rt;
        XPLMDataRef engine_reverse1;
        XPLMDataRef engine_reverse2;
        int id_s32_acft_request_chk;
        int id_s32_acft_request_gpu;
        int id_f32_acft_dryweightkg;
        int id_f32_acft_dryweightcg;
        int id_f32_acft_payload_cgz;
        int id_f32_acft_payload_kgs;
        int id_f32_acft_fuel_outerl;
        int id_f32_acft_fuel_outerr;
        int id_f32_acft_fuel_innerl;
        int id_f32_acft_fuel_innerr;
        int id_f32_acft_fuel_center;
        int id_s32_light_autopilot1;
        int id_s32_light_autopilot2;
        int id_f32_p_engines_lever1;
        int id_f32_p_engines_lever2;
        int id_f32_p_spoilers_lever;
        int id_s32_click_ss_tkovr_l;
        int id_s32_click_thr_disc_l;
        int id_u32_light_mode_belts;
        int id_u32_light_mode_emerg;
        int id_u32_light_mode_smoke;
        int id_u32_light_mode_strob;
        int id_u32_overhead_rmp3pow;
        int id_u32_efis_nav_rng_lft;
        int id_u32_efis_nav_rng_rgt;
        int id_u32_efis_nav_mod_lft;
        int id_u32_efis_nav_mod_rgt;
        int id_u32_fcu_tgt_alt_step;
    } dat;
} assert_context;

typedef struct
{
    acf_type ac_type;
    int   up_to_date;
    char afname[257];
    char afpath[513];
    char author[501];
    char descrp[261];
    char tailnb[ 41];
    char icaoid[ 41];
    XPLMDataRef dric;
    int engine_count;
    int engine_type1;
    int has_auto_thr;
    int has_rvrs_thr;
    struct
    {
        XPLMDataRef minimum;
        XPLMDataRef maximum;
        XPLMDataRef current;
        XPLMDataRef payload;
        XPLMDataRef gwcgz_m;
    }
    weight;
    struct
    {
        XPLMDataRef maximum;
        XPLMDataRef current;
        XPLMDataRef pertank;
        XPLMDataRef tankrat;
        struct
        {
            int    count;
            float max_kg;
            float max_lb;
            float cpl[9];
            float rat[9];
            float rfo[9];
            float max[9];
        }
        tanks;
    }
    fuel;
    union
    {
        assert_context assert;
    };
}
acf_info_context;

int uf_dref_string_read(XPLMDataRef dataref, char *string_buffer, size_t buffer_size);
int uf_dref_string_wrte(XPLMDataRef dataref, char *string_buffer, size_t buffer_size);
const char*                                 acf_type_get_name(acf_type aircraft_type);
int                                         acf_type_set_fuel(acf_info_context *info);
acf_info_context*                           acf_type_info_get                  (void);
int                                         acf_type_info_reset                (void);
acf_info_context*                           acf_type_info_update               (void);
int                                         acf_type_info_acf_ctx_init         (void);
int                                         acf_type_is_engine_running         (void);
int                          acf_type_load_get(acf_info_context *info, float *weight);
int                          acf_type_load_set(acf_info_context *info, float *weight);
int                          acf_type_zfwt_get(acf_info_context *info, float *weight);
int                          acf_type_zfwt_set(acf_info_context *info, float *weight);
int                          acf_type_grwt_get(acf_info_context *info, float *weight);
int                          acf_type_oewt_get(acf_info_context *info, float *weight);
int                          acf_type_fmax_get(acf_info_context *info, float *weight);
int                          acf_type_fuel_get(acf_info_context *info, float *weight);
int                          acf_type_fuel_set(acf_info_context *info, float *weight);

#endif /* ACF_TYPES_H */
