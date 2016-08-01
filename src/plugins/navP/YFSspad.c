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

#include <stdio.h>
#include <string.h>

#include "YFSmain.h"
#include "YFSspad.h"

#define SPAD_IDX (YFS_DISPLAY_NUMR - 1)
#define SPAD_COL (COLR_IDX_WHITE)

void yfs_spad_dupto(yfms_context *yfms, char buf[YFS_DISPLAY_NUMC + 1])
{
    if (!yfms || !buf)
    {
        return; // no error
    }
    snprintf(buf, YFS_DISPLAY_NUMC + 1, "%s", yfms->mwindow.screen.text[SPAD_IDX]);
    return;
}

void yfs_spad_clear(yfms_context *yfms)
{
    if (!yfms)
    {
        return; // no error
    }
    for (int i = 0; i < YFS_DISPLAY_NUMC; i++)
    {
        yfms->mwindow.screen.colr[SPAD_IDX][i] = SPAD_COL;
    }
    sprintf(yfms->mwindow.screen.text[SPAD_IDX], "%s", "");
    return;
}

void yfs_spad_remvc(yfms_context *yfms)
{
    if (!yfms)
    {
        return; // no error
    }
    if (strnlen(yfms->mwindow.screen.text[SPAD_IDX], 2) <= 1)
    {
        yfs_spad_clear(yfms);
        return;
    }
    size_t l = strnlen(yfms->mwindow.screen.text[SPAD_IDX], YFS_DISPLAY_NUMC);
    yfms->mwindow.screen.text[SPAD_IDX][l - 1] = '\0';
    return;
}

void yfs_spad_apndc(yfms_context *yfms, char c, int color)
{
    if (!yfms)
    {
        return; // no error
    }
    size_t l = strlen(yfms->mwindow.screen.text[SPAD_IDX]);
    if (YFS_DISPLAY_NUMC - 1 < l)
    {
        l = YFS_DISPLAY_NUMC - 1; // TODO: shift text & colors by 1 left
    }
    if (color >= 0)
    {
        yfms->mwindow.screen.colr[SPAD_IDX][l] = color;
    }
    yfms->mwindow.screen.colr[SPAD_IDX][l + 1] = '\0';
    yfms->mwindow.screen.colr[SPAD_IDX][l] = c;
    return;
}

void yfs_spad_reset(yfms_context *yfms, char *s, int color)
{
    if (!yfms)
    {
        return; // no error
    }
    for (int i = 0; i < YFS_DISPLAY_NUMC; i++)
    {
        yfms->mwindow.screen.colr[SPAD_IDX][i] = color >= 0 ? color : SPAD_COL;
    }
    snprintf(yfms->mwindow.screen.text[SPAD_IDX], YFS_DISPLAY_NUMC + 1, "%s", s);
    return;
}

//fixme
