/*
 * ACFvolumes.h
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

#ifndef ACF_VOLUMES_H
#define ACF_VOLUMES_H

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMDefs.h"

#include "ACFtypes.h"

typedef struct
{
    int   x_plane_v11;
    float last_volume;
    struct
    {
        struct
        {
            XPLMDataRef com1;
            XPLMDataRef com2;
            XPLMDataRef nav1;
            XPLMDataRef nav2;
            XPLMDataRef adf1;
            XPLMDataRef adf2;
            XPLMDataRef mark;
            XPLMDataRef dme0;
        } vol;
        struct
        {
            XPLMDataRef com1;
            XPLMDataRef com2;
        } rx;
        struct
        {
            XPLMDataRef comm;
        } tx;
        XPLMDataRef atctxt;
        XPLMDataRef volume;
    } radio;
    struct
    {
        union
        {
            struct
            {
                XPLMDataRef eng;
                XPLMDataRef fan;
                XPLMDataRef grd;
                XPLMDataRef prp;
                XPLMDataRef wrn;
                XPLMDataRef wxr;
            } xp10;
            struct
            {
                XPLMDataRef mas;
                XPLMDataRef cop;
                XPLMDataRef eng;
                XPLMDataRef env;
                XPLMDataRef ext;
                XPLMDataRef fan;
                XPLMDataRef grd;
                XPLMDataRef inn;
                XPLMDataRef prp;
                XPLMDataRef uiv;
                XPLMDataRef wxr;
                XPLMDataRef wrn;
            } xp11;
        };
        XPLMDataRef nabled;
        XPLMDataRef speech;
    } sound;
    struct
    {
        struct
        {
            struct
            {
                float vol0;
                float vol1;
                float vol2;
            } pe;
            struct
            {
                float vol0;
                float vol1;
                float vol2;
            } xb;
            struct
            {
                XPLMDataRef v1;
                XPLMDataRef v2;
            } ea50;
            struct
            {
                XPLMDataRef v1;
                XPLMDataRef v2;
            } pipa;
        } atc;
        struct
        {
            XPLMDataRef volume;
        } a350;
        struct
        {
            XPLMDataRef mtrack;
            XPLMDataRef volume;
        } absk;
        struct
        {
            XPLMDataRef ambien;
            XPLMDataRef engine;
            XPLMDataRef cllout;
            XPLMDataRef volume;
        } fft7;
        struct
        {
            XPLMDataRef volume;
        } fsts;
        struct
        {
            XPLMDataRef master;
            XPLMDataRef animc1;
            XPLMDataRef animc2;
        } tlss;
        struct
        {
            XPLMDataRef volext;
            XPLMDataRef volmut;
            XPLMDataRef volume;
        } ddnn;
    } custom;
}
acf_volume_context;

acf_volume_context* acf_volume_ctx_get(void);
float acf_atcvol_adj  (acf_volume_context *ctx, float offset               );
void  acf_volume_set  (acf_volume_context *ctx, float volume, acf_type type);
void  acf_volume_reset(acf_volume_context *ctx,               acf_type type);

#endif /* ACF_VOLUMES_H */
