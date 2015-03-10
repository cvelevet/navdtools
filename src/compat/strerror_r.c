/*
 * strerror_r.c
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

#include <errno.h>
#include <string.h>

int strerror_r(int errnum, char *strerrbuf, size_t buflen)
{
    int   err = 0;
    char *str = strerror(errnum);

    if (!str)
    {
        err = ENOMEM;
        goto end;
    }

    size_t len = strnlen(str, buflen);

    if (len >= buflen)
    {
        err = ERANGE;
        len = buflen - 1;
    }

    memcpy(strerrbuf, str, len);
    strerrbuf[len] = '\0';

end:
    return err;
}
