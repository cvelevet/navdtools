/*
 * compat.c
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

#include "compat.h"

#if COMPAT_STRCASESTR
#include "strcasestr.c"
#endif

#if COMPAT_STRERROR_R
#include "strerror_r.c"
#endif

#if COMPAT_STRNDUP
#include "strndup.c"
#endif

#if COMPAT_STRSEP
#include "strsep.c"
#endif
