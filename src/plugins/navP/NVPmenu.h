/*
 * NVPmenu.h
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2015 Timothy D. Walker and others.
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

#ifndef NVP_MENU_H
#define NVP_MENU_H

void* nvp_menu_init (void                );
int   nvp_menu_setup(void   *menu_context);
int   nvp_menu_reset(void   *menu_context);
int   nvp_menu_close(void **_menu_context);

#endif /* NVP_MENU_H */
