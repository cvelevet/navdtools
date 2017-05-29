/*
 * NVPplugin.h
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

#ifndef NVP_PLUGIN_H
#define NVP_PLUGIN_H

int  nvp_plugin_start  (char*,       char*, char*);
void nvp_plugin_stop   (void                     );
int  nvp_plugin_enable (void                     );
void nvp_plugin_disable(void                     );
void nvp_plugin_message(XPLMPluginID, long, void*);

#endif /* NVP_PLUGIN_H */
