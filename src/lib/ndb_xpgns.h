/*
 * ndb_xpgns.h
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

#ifndef NDT_NDB_XPGNS_H
#define NDT_NDB_XPGNS_H

#include "common/common.h"

#include "airport.h"
#include "flightplan.h"
#include "navdata.h"

int            ndt_ndb_xpgns_navdatabase_init    (ndt_navdatabase *ndb                    );
ndt_airport*   ndt_ndb_xpgns_navdata_init_airport(ndt_navdatabase *ndb, ndt_airport   *apt);
ndt_procedure* ndt_ndb_xpgns_navdata_open_procdre(ndt_navdatabase *ndb, ndt_procedure *prc);

#endif /* NDT_NDB_XPGNS_H */
