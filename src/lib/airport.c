/*
 * airport.c
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2014 Timothy D. Walker and others.
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

#include <inttypes.h>
#include <stdlib.h>
#include <strings.h>

#include "common/common.h"
#include "common/list.h"

#include "compat/compat.h"

#include "airport.h"
#include "flightplan.h"

ndt_airport* ndt_airport_init()
{
    ndt_airport *apt = calloc(1, sizeof(ndt_airport));
    if (!apt)
    {
        goto end;
    }

    apt->runways = ndt_list_init();
    if (!apt->runways)
    {
        ndt_airport_close(&apt);
        goto end;
    }
    apt->sids = ndt_list_init();
    if (!apt->sids)
    {
        ndt_airport_close(&apt);
        goto end;
    }
    apt->stars = ndt_list_init();
    if (!apt->stars)
    {
        ndt_airport_close(&apt);
        goto end;
    }

end:
    return apt;
}

void ndt_airport_close(ndt_airport **_apt)
{
    if (_apt && *_apt)
    {
        size_t i;
        ndt_airport *apt = *_apt;

        if (apt->runways)
        {
            while ((i = ndt_list_count(apt->runways)))
            {
                ndt_runway *rwy = ndt_list_item(apt->runways,  -1);
                ndt_list_rem                   (apt->runways, rwy);
                ndt_runway_close               (             &rwy);
            }
            ndt_list_close(&apt->runways);
        }
        if (apt->allprocs)
        {
            while ((i = ndt_list_count(apt->allprocs)))
            {
                ndt_procedure *proc = ndt_list_item(apt->allprocs,   -1);
                ndt_list_rem                       (apt->allprocs, proc);
                ndt_procedure_close                (              &proc);
            }
            ndt_list_close(&apt->allprocs);
        }
        if (apt->sids)
        {
            ndt_list_close(&apt->sids);
        }
        if (apt->stars)
        {
            ndt_list_close(&apt->stars);
        }

        free(apt);

        *_apt = NULL;
    }
}

ndt_runway* ndt_runway_init()
{
    ndt_runway *rwy = calloc(1, sizeof(ndt_runway));
    if (!rwy)
    {
        goto end;
    }

    rwy->sids = ndt_list_init();
    if (!rwy->sids)
    {
        ndt_runway_close(&rwy);
        goto end;
    }
    rwy->stars = ndt_list_init();
    if (!rwy->stars)
    {
        ndt_runway_close(&rwy);
        goto end;
    }
    rwy->approaches = ndt_list_init();
    if (!rwy->approaches)
    {
        ndt_runway_close(&rwy);
        goto end;
    }

    rwy->surface = NDT_RWYSURF_OTHER;

end:
    return rwy;
}

void ndt_runway_close(ndt_runway **_rwy)
{
    if (_rwy && *_rwy)
    {
        ndt_runway *rwy = *_rwy;

        if (rwy->sids)
        {
            ndt_list_close(&rwy->sids);
        }
        if (rwy->stars)
        {
            ndt_list_close(&rwy->stars);
        }
        if (rwy->approaches)
        {
            ndt_list_close(&rwy->approaches);
        }

        free(rwy);

        *_rwy = NULL;
    }
}

ndt_runway* ndt_runway_get(ndt_list *runways, const char *name)
{
    ndt_runway *rwy = NULL;

    if (!runways || !name)
    {
        goto end;
    }

    for (size_t i = 0; i < ndt_list_count(runways); i++)
    {
        rwy = ndt_list_item(runways, i);

        if (rwy && !strcmp(name, rwy->info.idnt))
        {
            goto end;
        }

        rwy = NULL;
    }

end:
    return rwy;
}
