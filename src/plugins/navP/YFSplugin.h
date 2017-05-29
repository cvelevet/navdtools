/*
 * YFSplugin.h
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2017 Timothy D. Walker and others.
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

#ifndef YFS_PLUGIN_H
#define YFS_PLUGIN_H

int  yfs_plugin_start  (char*,       char*, char*);
void yfs_plugin_stop   (void                     );
int  yfs_plugin_enable (void                     );
void yfs_plugin_disable(void                     );
void yfs_plugin_message(XPLMPluginID, long, void*);

#endif /* YFS_PLUGIN_H */
