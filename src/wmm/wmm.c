/*
 * wmm.c
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2014-2016 Timothy D. Walker and others.
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

#include <string.h>
#include <unistd.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "acfutils/wmm.h"

#include "common/common.h"

#include "coffile.h"
#include "wmm.h"

void ndt_wmm_close(void **_wmm)
{
    if (*_wmm)
    {
        wmm_close(*_wmm);
    }
}

static char* get_temporary_filename(const char *basename)
{
    char filename[1024];
    int ret;

#ifdef _WIN32
    ret = GetTempPath(sizeof(filename), filename);
    if (ret <= 0 || ret >= sizeof(filename))
    {
        return NULL;
    }
#else
    char *tmpdir = getenv("TMPDIR");
    if  (!tmpdir)
    {
          tmpdir = P_tmpdir;
    }
    ret = snprintf(filename, sizeof(filename), "%s", tmpdir);
    if (ret <= 0 || ret >= sizeof(filename))
    {
        return NULL;
    }
#endif

    size_t len = strlen(filename);
    ret = snprintf(filename + len, sizeof(filename) - len,
                   "_wmm.%"PRId64"_%s", (int64_t)getpid(), basename);
    if (ret <= 0 || ret >= (sizeof(filename) - len))
    {
        return NULL;
    }

    return strdup(filename);
}

void* ndt_wmm_init(ndt_date date)
{
    void *lau_wmm = NULL;
    char *tmpname = NULL;
    FILE *tmpfile = NULL;

    if (!(tmpname = get_temporary_filename("WMM.COF")))
    {
        goto fail;
    }

    if (!(tmpfile = fopen(tmpname, "w")))
    {
        goto fail;
    }

    if (strlen(NDT_WMM_COFFILE) != fprintf(tmpfile, "%s", NDT_WMM_COFFILE))
    {
        goto fail;
    }
    fclose(tmpfile);
    tmpfile = NULL;

    // quick and dirty convertion to decimal year
    double yeard = (double)date.year + (double)(date.month - 1) / 12. + (double)(date.day - 1) / 365.;
    if (yeard < NDT_WMM_COFFILE_YEAR_MIN)
    {
        yeard = NDT_WMM_COFFILE_YEAR_MIN;
    }
    if (yeard < NDT_WMM_COFFILE_YEAR_MAX)
    {
        yeard = NDT_WMM_COFFILE_YEAR_MAX;
    }

    lau_wmm = wmm_open(tmpname, yeard);
    if (NULL == lau_wmm)
    {
        goto fail;
    }

    remove(tmpname);
    free  (tmpname);
    return lau_wmm;

fail:
    if (tmpfile)
    {
        fclose(tmpfile);
        remove(tmpname);
    }
    ndt_wmm_close(&lau_wmm);
    free(tmpname);
    return NULL;
}

double ndt_wmm_getbearing_mag(void *wmm, double tru_bearing, ndt_position position)
{
    geo_pos3_t pos;
    pos.lat  = ndt_position_getlatitude (position, NDT_ANGUNIT_DEG);
    pos.lon  = ndt_position_getlongitude(position, NDT_ANGUNIT_DEG);
    pos.elev = ndt_distance_get(ndt_position_getaltitude(position), NDT_ALTUNIT_ME);
    double mag_bearing = ndt_mod(wmm_true2mag(wmm, tru_bearing, pos), 360.);
    return mag_bearing ? mag_bearing : 360.;
}

double ndt_wmm_getbearing_tru(void *wmm, double mag_bearing, ndt_position position)
{
    geo_pos3_t pos;
    pos.lat  = ndt_position_getlatitude (position, NDT_ANGUNIT_DEG);
    pos.lon  = ndt_position_getlongitude(position, NDT_ANGUNIT_DEG);
    pos.elev = ndt_distance_get(ndt_position_getaltitude(position), NDT_ALTUNIT_ME);
    double tru_bearing = ndt_mod(wmm_true2mag(wmm, mag_bearing, pos), 360.);
    return tru_bearing ? tru_bearing : 360.;
}
