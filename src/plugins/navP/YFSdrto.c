/*
 * YFSdrto.c
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

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>

#include "common/common.h"
//#include "lib/airport.h"
//#include "lib/flightplan.h"
//#include "lib/navdata.h"

#include "YFSdrto.h"
#include "YFSfpln.h"
#include "YFSmain.h"
#include "YFSspad.h"

void yfs_drto_pageopen(yfms_context *yfms)//fixme
{
    if (yfms == NULL)
    {
        return; // no error
    }
    if (yfs_main_newpg(yfms, PAGE_DRTO))
    {
        return;
    }
    //fixme
    yfs_drto_pageupdt(yfms); return;
}

void yfs_drto_pageupdt(yfms_context *yfms)//fixme
{
    /* reset lines, set page title */
    for (int i = 0; i < YFS_DISPLAY_NUMR - 1; i++)
    {
        yfs_main_rline(yfms, i, -1);
    }

    /* ensure we're still synced w/navdlib, reset if necessary */
    if (yfms->data.init.ialized)
    {
        //fixme
    }

    //fixme

    /* all good */
    return;
}
