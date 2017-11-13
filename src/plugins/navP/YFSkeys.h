/*
 * YFSkeys.h
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

#ifndef YFS_KEYS_H
#define YFS_KEYS_H

#include "Widgets/XPWidgetDefs.h"
#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDefs.h"

#include "YFSmain.h"

void yfs_keypressed(yfms_context*,                       XPWidgetID);
int  yfs_keysniffer(char,          XPLMKeyFlags,     char,    void*);
int  yfs_mouseevent(yfms_context*, XPMouseState_t*, XPWidgetMessage);

#endif /* YFS_KEYS_H */
