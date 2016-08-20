/*
 * YFSrdio.h
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

#ifndef YFS_RDIO_H
#define YFS_RDIO_H

#include "YFSmain.h"

void yfs_rad1_pageopen(yfms_context *yfms);
void yfs_rad2_pageopen(yfms_context *yfms);
void yfs_rdio_pageupdt(yfms_context *yfms);

#endif /* YFS_RDIO_H */
