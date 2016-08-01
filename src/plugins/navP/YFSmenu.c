/*
 * YFSmenu.c
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

#include <stdio.h>
#include <string.h>

#include "YFSmain.h"
#include "YFSmenu.h"
#include "YFSspad.h"

void yfs_menu_resetall(yfms_context *yfms)
{
    if (!yfms)
    {
        return; // no error
    }

    /* scratchpad */
    for (int i = 0; i < YFS_DISPLAY_NUMR; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }
    char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf);
    if  (strnlen(buf, 1))
    {
        yfs_spad_clear(yfms);
    }
    yfs_spad_reset(yfms, "YFMS INITIALIZED", COLR_IDX_ORANGE);

    /* all good */
    yfs_idnt_pageopen(yfms); return;
}

void yfs_idnt_pageopen(yfms_context *yfms)
{
    if (!yfms)
    {
        return; // no error
    }

    /* */
}//fixme
