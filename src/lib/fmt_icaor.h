/*
 * fmt_icaor.h
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

#ifndef NDT_FMT_ICAOR_H
#define NDT_FMT_ICAOR_H

#include "common/common.h"

#include "flightplan.h"
#include "navdata.h"

int ndt_fmt_icaor_flightplan_set_route(ndt_flightplan *flightplan, ndt_navdatabase *navdatabase, const char *route);
int ndt_fmt_icaor_flightplan_write    (ndt_flightplan *flightplan,                               FILE       *file );
int ndt_fmt_sbrif_flightplan_write    (ndt_flightplan *flightplan,                               FILE       *file );
int ndt_fmt_svect_flightplan_write    (ndt_flightplan *flightplan,                               FILE       *file );

#endif /* NDT_FMT_ICAOR_H */
