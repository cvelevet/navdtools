/*
 * YFSkeys.c
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2016 Timothy D. Walker and others.
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

#include "Widgets/XPWidgets.h"

#include "YFSkeys.h"
#include "YFSmain.h"
#include "YFSspad.h"

void yfs_keypressed(yfms_context *yfms, XPWidgetID key)
{
    if (!yfms || !key)
    {
        return; // no error
    }

    /* keys that write to the scratchpad */
    if (key == yfms->mwindow.keys.keyid_al_a)
    {
        yfs_spad_apndc(yfms, 'A', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_b)
    {
        yfs_spad_apndc(yfms, 'B', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_c)
    {
        yfs_spad_apndc(yfms, 'C', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_d)
    {
        yfs_spad_apndc(yfms, 'D', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_e)
    {
        yfs_spad_apndc(yfms, 'E', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_f)
    {
        yfs_spad_apndc(yfms, 'F', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_g)
    {
        yfs_spad_apndc(yfms, 'G', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_h)
    {
        yfs_spad_apndc(yfms, 'H', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_i)
    {
        yfs_spad_apndc(yfms, 'I', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_j)
    {
        yfs_spad_apndc(yfms, 'J', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_k)
    {
        yfs_spad_apndc(yfms, 'K', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_l)
    {
        yfs_spad_apndc(yfms, 'L', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_m)
    {
        yfs_spad_apndc(yfms, 'M', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_n)
    {
        yfs_spad_apndc(yfms, 'N', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_o)
    {
        yfs_spad_apndc(yfms, 'O', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_p)
    {
        yfs_spad_apndc(yfms, 'P', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_q)
    {
        yfs_spad_apndc(yfms, 'Q', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_r)
    {
        yfs_spad_apndc(yfms, 'R', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_s)
    {
        yfs_spad_apndc(yfms, 'S', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_t)
    {
        yfs_spad_apndc(yfms, 'T', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_u)
    {
        yfs_spad_apndc(yfms, 'U', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_v)
    {
        yfs_spad_apndc(yfms, 'V', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_w)
    {
        yfs_spad_apndc(yfms, 'W', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_x)
    {
        yfs_spad_apndc(yfms, 'X', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_y)
    {
        yfs_spad_apndc(yfms, 'Y', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_z)
    {
        yfs_spad_apndc(yfms, 'Z', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_slsh)
    {
        yfs_spad_apndc(yfms, '/', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_spce)
    {
        yfs_spad_apndc(yfms, '_', -1); return; // cf. YFSmain.c, draw_display()
    }
    if (key == yfms->mwindow.keys.keyid_ovfy)
    {
        yfs_spad_reset(yfms, "^", -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_clir)
    {
        yfs_spad_remvc(yfms); return;
    }
    if (key == yfms->mwindow.keys.keyid_num0)
    {
        yfs_spad_apndc(yfms, '0', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num1)
    {
        yfs_spad_apndc(yfms, '1', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num2)
    {
        yfs_spad_apndc(yfms, '2', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num3)
    {
        yfs_spad_apndc(yfms, '3', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num4)
    {
        yfs_spad_apndc(yfms, '4', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num5)
    {
        yfs_spad_apndc(yfms, '5', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num6)
    {
        yfs_spad_apndc(yfms, '6', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num7)
    {
        yfs_spad_apndc(yfms, '7', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num8)
    {
        yfs_spad_apndc(yfms, '8', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num9)
    {
        yfs_spad_apndc(yfms, '9', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_pird)
    {
        yfs_spad_apndc(yfms, '.', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_plus)
    {
        yfs_spad_apndc(yfms, '-', -1); return;
    }
}
