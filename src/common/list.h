/*
 * list.h
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

#ifndef NDT_LIST_H
#define NDT_LIST_H

#include <inttypes.h>

typedef struct ndt_list ndt_list;

ndt_list* ndt_list_init  (                                            );
size_t    ndt_list_count (const ndt_list *list                        );
void*     ndt_list_item  (const ndt_list *list,             size_t idx);
void      ndt_list_insert(      ndt_list *list, void *item, size_t idx);
void      ndt_list_add   (      ndt_list *list, void *item            );
void      ndt_list_rem   (      ndt_list *list, void *item            );
void      ndt_list_close (      ndt_list **ptr                        );
void      ndt_list_sort  (      ndt_list *list,
                          size_t w, int (*c)(const void*, const void*));

#endif /* NDT_LIST_H */
