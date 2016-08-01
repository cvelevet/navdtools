/*
 * YFSspad.h
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

#ifndef YFS_SPAD_H
#define YFS_SPAD_H

#include "YFSmain.h"

void yfs_spad_dupto(yfms_context *yfms, char buf[YFS_DISPLAY_NUMC + 1]);
void yfs_spad_clear(yfms_context *yfms                                );
void yfs_spad_remvc(yfms_context *yfms                                );
void yfs_spad_apndc(yfms_context *yfms, char c,              int color);
void yfs_spad_reset(yfms_context *yfms, char string[],       int color);

#endif /* YFS_SPAD_H */
