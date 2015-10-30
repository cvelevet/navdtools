/*
 * list.c
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

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "compat/compat.h"

#include "list.h"

#define NDT_LIST_DEFAULT_SIZE 10

struct ndt_list
{
    void **items;
    int    count;
    int    alloc;
};

ndt_list* ndt_list_init()
{
    ndt_list *l = calloc(1, sizeof(ndt_list));

    if (l)
    {
        l->items = malloc(NDT_LIST_DEFAULT_SIZE * sizeof(void*));

        if (l->items)
        {
            l->alloc = NDT_LIST_DEFAULT_SIZE;
            return l;
        }
    }

    ndt_list_close(&l);
    return NULL;
}

size_t ndt_list_count(const ndt_list *l)
{
    return l && l->count > 0 ? l->count : 0;
}

void* ndt_list_item(const ndt_list *l, int i)
{
    if (!l || l->count <= 0 || l->count <= i)
    {
        return NULL;
    }

    // negative index means start from the end
    if (i < 0)
    {
        if ((i += l->count) < 0)
        {
            return NULL;
        }
    }
    if (i > l->count - 1)
    {
        i = l->count - 1;
    }

    return l->items[i];
}

void ndt_list_insert(ndt_list *l, void *p, int i)
{
    if (!l || !p)
    {
        return;
    }

    if (i < 0)
    {
        i = 0;
    }
    if (i > l->count)
    {
        i = l->count;
    }

    if (l->alloc == l->count)
    {
        void *ptr = realloc(l->items,
                            sizeof(void*) * (l->alloc + NDT_LIST_DEFAULT_SIZE));
        if (!ptr)
        {
            /* l->items it untouched, but we can't add anything */
            return;
        }

        l->alloc += NDT_LIST_DEFAULT_SIZE;
        l->items  = ptr;
    }

    if (i != l->count)
    {
        memmove(&l->items[i + 1], &l->items[i], (l->count - i) * sizeof(void*));
    }


    l->items[i] = p;
    l->count++;
}

void ndt_list_add(ndt_list *l, void *p)
{
    ndt_list_insert(l, p, ndt_list_count(l));
}

void ndt_list_rem(ndt_list *l, void *p)
{
    if (!l || !p || !ndt_list_count(l))
    {
        return;
    }

    /* check if we can avoid using memmove for performance reasons */
    if (p == l->items[l->count - 1])
    {
        l->count--;
        return;
    }

    for (size_t i = 0; i < l->count - 1; i++)
    {
        if (p == l->items[i])
        {
            memmove(&l->items[i], &l->items[i + 1], (--l->count - i) * sizeof(void*));
            return;
        }
    }
}

void ndt_list_empty(ndt_list *l)
{
    size_t count = ndt_list_count(l);
    if (l)
    {
        while (ndt_list_count(l) && count-- > 0)
        {
            ndt_list_rem(l, ndt_list_item(l, -1));
        }
    }
}

void ndt_list_close(ndt_list **_l)
{
    if (_l && *_l)
    {
        ndt_list *l = *_l;

        free(l->items);
        free(l);

        *_l = NULL;
    }
}

void ndt_list_sort(ndt_list *l, size_t w, int (*c)(const void*, const void*))
{
    if (l && l->count && w && c)
    {
        qsort(l->items, l->count, w, c);
    }
}
