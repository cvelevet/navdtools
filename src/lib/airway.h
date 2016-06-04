/*
 * airway.h
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

#ifndef NDT_AIRWAY_H
#define NDT_AIRWAY_H

#include "common/common.h"
#include "common/list.h"

/*
 * Ideally, airway legs would simply reference waypoints is the waypoint list;
 * however, there are about 150,000 waypoints in a typical database; duplicates
 * (same waypoint found in more than one airway) need to be looked up each time.
 *
 * Even at 5 milliseconds per waypoint lookup, this would still take a good 10
 * to 15 minutes, which we can't possibly do unless we store the results in our
 * own database for later use, which is not planned.
 *
 * So we have to store airway legs using a specific format and delay lookup
 * until we know which airways we need to do this for.
 */

typedef struct ndt_airway
{
    ndt_info              info; // identification information
    struct ndt_airway_leg *leg; // first leg in airway
} ndt_airway;

typedef struct ndt_airway_leg
{
    struct
    {
        ndt_info     info;
        ndt_position position;
    } in;

    struct
    {
        ndt_info     info;
        ndt_position position;
    } out;

    struct
    {
        int inbound;
        int outbound;
    } course;

    ndt_distance length;

    struct ndt_airway     *awy;
    struct ndt_airway_leg *next;
} ndt_airway_leg;

ndt_airway*     ndt_airway_init      (                                                                   );
void            ndt_airway_close     (ndt_airway   **_airway                                             );
ndt_airway_leg* ndt_airway_startpoint(ndt_airway     *airway, const char *waypoint, ndt_position position);
ndt_airway_leg* ndt_airway_endpoint  (ndt_airway_leg *inleg,  const char *waypoint, ndt_position position);
ndt_airway_leg* ndt_airway_intersect (ndt_airway_leg *inleg,  ndt_airway *airway                         );

#endif /* NDT_AIRWAY_H */
