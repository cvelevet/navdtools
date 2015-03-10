/*
 * strndup.c
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

#include <stdlib.h>
#include <string.h>

char* strndup(const char *s1, size_t n)
{
    char  *dup;
    size_t len = strnlen(s1, n);

    if ((dup = malloc(len + 1)))
    {
        memcpy(dup, s1, len);
        dup[len] = '\0';
    }

    return dup;
}
