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

void yfs_keypressed(yfms_context *yfms, XPWidgetID key)
{
    if (!yfms || !key)
    {
        return; // no error
    }

    /* keys that write to the scratchpad */
    if (key == yfms->mwindow.keys.keyid_al_a)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_b)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_c)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_d)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_e)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_f)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_g)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_h)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_i)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_j)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_k)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_l)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_m)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_n)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_o)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_p)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_q)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_r)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_s)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_t)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_u)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_v)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_w)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_x)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_y)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_al_z)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_slsh)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_spce)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_ovfy)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_clir)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_num0)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_num1)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_num2)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_num3)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_num4)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_num5)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_num6)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_num7)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_num8)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_num9)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_pird)
    {
        return;
    }
    if (key == yfms->mwindow.keys.keyid_plus)
    {
        return;
    }
}
