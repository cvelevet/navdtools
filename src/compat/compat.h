/*
 * compat.h
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

#ifndef NDT_COMPAT_H
#define NDT_COMPAT_H

#if COMPAT_STRCASESTR
char* strcasestr(const char *s1, const char *s2);
#endif

#if COMPAT_STRERROR_R
#include <string.h>
int strerror_r(int errnum, char *strerrbuf, size_t buflen);
#endif

#if COMPAT_STRNDUP
#include <string.h>
char* strndup(const char *s1, size_t n);
#endif

#if COMPAT_STRSEP
char* strsep(char **stringp, const char *delim);
#endif

#endif /* NDT_COMPAT_H */
