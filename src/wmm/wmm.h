/*
 * wmm.h
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

#ifndef NDT_WMM_H
#define NDT_WMM_H

void*  ndt_wmm_init          (void);
void   ndt_wmm_close         (void **_wmm);
double ndt_wmm_getbearing_mag(void *wmm, double tru_bearing, ndt_position position, ndt_date date);
double ndt_wmm_getbearing_tru(void *wmm, double mag_bearing, ndt_position position, ndt_date date);

#endif /* NDT_WMM_H */
