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

void yfs_spad_copy2(yfms_context *yfms, char buf[YFS_DISPLAY_NUMC + 1])
{
    if (!yfms || !buf)
    {
        return; // no error
    }
    size_t l = strnlen(yfms->mwindow.screen.text[SPAD_IDX], YFS_DISPLAY_NUMC);
    while (l >= 1 &&   yfms->mwindow.screen.text[SPAD_IDX][l - 1] == ' ')
    {
        yfms->mwindow.screen.text[SPAD_IDX][--l] = 0;
    }
    sprintf(buf, "%s", yfms->mwindow.screen.text[SPAD_IDX]);
    return;
}

void yfs_spad_clear(yfms_context *yfms)
{
    if (!yfms)
    {
        return; // no error
    }
    if (yfms->mwindow.screen.text[SPAD_IDX][0] == 0 ||
        yfms->mwindow.screen.text[SPAD_IDX][0] == ' ')
    {
        yfs_spad_reset(yfms, "CLR", -1); return;
    }
    for (int i = 0; i < YFS_DISPLAY_NUMC; i++)
    {
        yfms->mwindow.screen.colr[SPAD_IDX][i] = SPAD_COL;
    }
    yfms->mwindow.screen.text[SPAD_IDX][0] = 0;
    return;
}

void yfs_spad_remvc(yfms_context *yfms)
{
    if (!yfms)
    {
        return; // no error
    }
    char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf); size_t l = strlen(buf);
    if (l <= 1)
    {
        yfs_spad_clear(yfms); return; // one or fewer character left
    }
    buf[l - 1] = 0; sprintf(yfms->mwindow.screen.text[SPAD_IDX], "%s", buf);
    return;
}

void yfs_spad_apndc(yfms_context *yfms, char c, int color)
{
    if (!yfms)
    {
        return; // no error
    }
    char buf[YFS_DISPLAY_NUMC + 1]; yfs_spad_copy2(yfms, buf); size_t l = strlen(buf);
    if (c == '+' || c == '-')
    {
        if (buf[l - 1] == '+' || buf[l - 1] == '-')
        {
            buf[l - 1] = buf[l - 1] == '+' ? '-' : '+';
            sprintf(yfms->mwindow.screen.text[SPAD_IDX], "%s", buf);
            return;
        }
    }
    if (l >= YFS_DISPLAY_NUMC)
    {
        return; // scratchpad full
    }
    yfms->mwindow.screen.colr[SPAD_IDX][l] = color >= 0 ? color : SPAD_COL;
    sprintf(yfms->mwindow.screen.text[SPAD_IDX], "%s%c", buf, c);
    return;
}

/*
 * TODO: keep a backup before overwriting; e.g. when trying to line select a
 *       string into a field, and the format is wrong, we may simply write
 *       "FORMAT ERROR" to the scratchpad; in some circumstances, we may want
 *       to recover the previous scratchpad's contents (e.g. by pressing CLR
 *       after the error) (has_backup && clear -> recover, else see below).
 *
 *       Also, when pressing a key right after a reset, we may want to clear
 *       the scratchpad instead of appending (TODO: implement). We need a new
 *       variable (e.g. was_reset: 1 -> clear, else append) to keep track…
 */
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
    for (int i = strlen(s) - 1; i >= 0; i--)
    {
        if (YFS_DISPLAY_NUMC >= i && s[i] == ' ')
        {
            yfms->mwindow.screen.text[SPAD_IDX][i] = '_'; // cf. YFSmain.c, draw_display()
        }
    }
    return;
}
