/*
 * airway.c
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2014-2016 Timothy D. Walker and others.
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

#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "common/common.h"
#include "common/list.h"

#include "compat/compat.h"

#include "airway.h"

ndt_airway* ndt_airway_init()
{
    ndt_airway *awy = calloc(1, sizeof(ndt_airway));
    if (!awy)
    {
        goto end;
    }

end:
    return awy;
}

void ndt_airway_close(ndt_airway **_awy)
{
    if (_awy && *_awy)
    {
        ndt_airway     *awy = *_awy;
        ndt_airway_leg *leg = awy->leg;

        while (leg)
        {
            ndt_airway_leg *next = leg->next;
            free(leg);
            leg = next;
        }

        free(awy);

        *_awy = NULL;
    }
}

ndt_airway_leg* ndt_airway_startpoint(ndt_airway *awy, const char *wpt, ndt_position pos)
{
    if (!awy || !wpt)
    {
        goto end;
    }

    ndt_airway_leg *leg = awy->leg;

    while (leg)
    {
        if (!strcmp(wpt, leg->in.info.idnt) &&
            !ndt_distance_get(ndt_position_calcdistance(pos, leg->in.position), NDT_ALTUNIT_NA))
        {
            return leg;
        }

        leg = leg->next;
    }

end:
    return NULL;
}

ndt_airway_leg* ndt_airway_endpoint(ndt_airway_leg *in, const char *wpt, ndt_position pos)
{
    if (!in || !wpt)
    {
        goto end;
    }

    ndt_airway_leg *leg = in;

    while (leg)
    {
        if (!strcmp(wpt, leg->out.info.idnt) &&
            !ndt_distance_get(ndt_position_calcdistance(pos, leg->out.position), NDT_ALTUNIT_NA))
        {
            return leg;
        }

        leg = leg->next;
    }

end:
    return NULL;
}

ndt_airway_leg* ndt_airway_intersect(ndt_airway_leg *in, ndt_airway *awy)
{
    if (!in || !awy)
    {
        goto end;
    }

    /*
     * Find a valid   endpoint for the first  airway, starting at leg in, that's
     * also a valid startpoint for the second airway (awy).
     */
    ndt_airway_leg *leg = in;

    while (leg)
    {
        if (ndt_airway_startpoint(awy, leg->out.info.idnt, leg->out.position))
        {
            return leg;
        }

        leg = leg->next;
    }

end:
    return NULL;
}
