/*
 * navdata.h
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

#ifndef NDT_NAVDATA_H
#define NDT_NAVDATA_H

#include <inttypes.h>

#include "common/common.h"
#include "common/list.h"

#include "airport.h"
#include "airway.h"
#include "waypoint.h"

typedef enum ndt_navdataformat
{
    NDT_NAVDFMT_OTHER, // unknown or unsupported format
    NDT_NAVDFMT_XPGNS, // X-Plane 10.30 GNS navdata
} ndt_navdataformat;

typedef struct ndt_navdatabase
{
    ndt_info  info;        // identification information
    ndt_list *airports;    // list of all airports  in database (struct ndt_airport)
    ndt_list *airways;     // list of all airways   in database (struct ndt_airway)
    ndt_list *waypoints;   // list of all waypoints in database (struct ndt_waypoint)

    void *wmm;             // World Magnetic Model library wrapper
} ndt_navdatabase;

ndt_navdatabase* ndt_navdatabase_init (const char            *root, ndt_navdataformat fmt);
void             ndt_navdatabase_close(      ndt_navdatabase **ptr                       );

ndt_airport*  ndt_navdata_get_airport (ndt_navdatabase *ndb, const char   *idt                                                                                                          );
ndt_airway*   ndt_navdata_get_airway  (ndt_navdatabase *ndb, const char   *idt, size_t     *idx                                                                                         );
ndt_waypoint* ndt_navdata_get_waypoint(ndt_navdatabase *ndb, const char   *idt, size_t     *idx                                                                                         );
ndt_waypoint* ndt_navdata_get_wptnear2(ndt_navdatabase *ndb, const char   *idt, size_t     *idx,    ndt_position  pos                                                                   );
ndt_waypoint* ndt_navdata_get_wpt4pos (ndt_navdatabase *ndb, const char   *idt, size_t     *idx,    ndt_position  pos                                                                   );
ndt_waypoint* ndt_navdata_get_wpt4aws (ndt_navdatabase *ndb, ndt_waypoint *src, const char *awy2id, const char   *awyidt, ndt_airway **_awy, ndt_airway_leg **_in, ndt_airway_leg **_out);
ndt_waypoint* ndt_navdata_get_wpt4awy (ndt_navdatabase *ndb, ndt_waypoint *src, const char *dstidt, const char   *awyidt, ndt_airway **_awy, ndt_airway_leg **_in, ndt_airway_leg **_out);

#endif /* NDT_NAVDATA_H */
