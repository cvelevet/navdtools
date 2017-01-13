/*
 * YFSfpln.h
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

#ifndef YFS_FPLN_H
#define YFS_FPLN_H

#include "YFSmain.h"

void yfs_fpln_pageopen(yfms_context *yfms                       );
void yfs_fpln_pageupdt(yfms_context *yfms                       );
void yfs_fpln_fplnsync(yfms_context *yfms                       );
void yfs_fpln_fplnupdt(yfms_context *yfms                       );
void yfs_fpln_directto(yfms_context *yfms, int index, int insert);

#endif /* YFS_FPLN_H */
