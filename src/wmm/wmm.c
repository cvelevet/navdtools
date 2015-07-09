/*
 * wmm.c
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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdlib.h>
#include <unistd.h>

#ifdef _WIN32
#include <windows.h>
/* GeomagnetismHeader.h redefines those and the GCC warning can't be disabled */
#ifdef FALSE
#undef FALSE
#endif
#ifdef TRUE
#undef TRUE
#endif
#endif // _WIN32

#include "common/common.h"

#include "GeomagnetismHeader.h"
#include "EGM9615.h"
#include "coffile.h"
#include "wmm.h"

typedef struct
{
    // model data
    MAGtype_MagneticModel *MagneticModels[1];
    MAGtype_MagneticModel *TimedMagneticModel;
    MAGtype_Ellipsoid      Ellip;
    MAGtype_Geoid          Geoid;

    // results we care about (position/date-dependent)
    MAGtype_GeoMagneticElements GeoMagneticElements;
} ndt_wmm;

static void recalculate(ndt_wmm *wmm, ndt_position position, ndt_date date)
{
    if (wmm != NULL)
    {
        char errbuf[255];
        MAGtype_CoordSpherical CoordSpherical;
        MAGtype_CoordGeodetic  CoordGeodetic;
        MAGtype_Date           UserDate;

        /* Coordinates */
        CoordGeodetic.phi    = ndt_position_getlatitude (position, NDT_ANGUNIT_DEG);
        CoordGeodetic.lambda = ndt_position_getlongitude(position, NDT_ANGUNIT_DEG);

        /* Altitude */
        wmm->Geoid.UseGeoid            = 1;
        CoordGeodetic.HeightAboveGeoid = ndt_distance_get(ndt_position_getaltitude(position), NDT_ALTUNIT_ME) / 1000;
        MAG_ConvertGeoidToEllipsoidHeight(&CoordGeodetic, &wmm->Geoid);

        /* Date */
        UserDate.Year  = date.year;
        UserDate.Month = date.month;
        UserDate.Day   = date.day;
        MAG_DateToYear(&UserDate, errbuf);
        if (UserDate.DecimalYear > wmm->MagneticModels[0]->CoefficientFileEndDate)
        {
            UserDate.DecimalYear = wmm->MagneticModels[0]->CoefficientFileEndDate;
        }
        if (UserDate.DecimalYear < wmm->MagneticModels[0]->epoch)
        {
            UserDate.DecimalYear = wmm->MagneticModels[0]->epoch;
        }

        /* Convert from geodetic to Spherical Equations: 17-18, WMM Technical report */
        MAG_GeodeticToSpherical(wmm->Ellip, CoordGeodetic, &CoordSpherical);
        /* Time adjust the coefficients, Equation 19, WMM Technical report */
        MAG_TimelyModifyMagneticModel(UserDate, wmm->MagneticModels[0], wmm->TimedMagneticModel);
        /* Computes the geoMagnetic field elements and their time change */
        MAG_Geomag(wmm->Ellip, CoordSpherical, CoordGeodetic, wmm->TimedMagneticModel, &wmm->GeoMagneticElements);
    }
}

static double get_magnetic_decl(ndt_wmm *wmm, ndt_position position, ndt_date date)
{
    if (wmm != NULL)
    {
        recalculate(wmm, position, date);
        return wmm->GeoMagneticElements.Decl;
    }
    return 0.;
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

void* ndt_wmm_init(void)
{
    char *tmpname = NULL;
    FILE *tmpfile = NULL;
    void *w = calloc(1, sizeof(ndt_wmm));
    ndt_wmm *wmm = w;
    if (wmm == NULL)
    {
        goto fail;
    }

    if (!(tmpname = get_temporary_filename("WMM.COF")))
    {
        goto fail;
    }

    tmpfile = fopen(tmpname, "w");
    if (!tmpfile)
    {
        goto fail;
    }

    if (fprintf(tmpfile, "%s", NDT_WMM_COFFILE) != strlen(NDT_WMM_COFFILE))
    {
        goto fail;
    }
    fclose(tmpfile);
    tmpfile = NULL;

    if (!MAG_robustReadMagModels(tmpname, &wmm->MagneticModels, 1))
    {
        goto fail;
    }

    if (wmm->MagneticModels[0] == NULL)
    {
        goto fail;
    }

    if (wmm->MagneticModels[0]->nMax <= 0)
    {
        goto fail;
    }

     /* For storing the time modified WMM Model parameters */
    wmm->TimedMagneticModel = MAG_AllocateModelMemory((wmm->MagneticModels[0]->nMax + 1) *
                                                      (wmm->MagneticModels[0]->nMax + 2) / 2);
    if (wmm->MagneticModels[0]  == NULL ||
        wmm->TimedMagneticModel == NULL)
    {
        goto fail;
    }

     /* Set default values and constants */
    MAG_SetDefaults(&wmm->Ellip, &wmm->Geoid);

    /* Set EGM96 Geoid parameters */
    wmm->Geoid.GeoidHeightBuffer = GeoidHeightBuffer;
    wmm->Geoid.Geoid_Initialized = 1;

    remove(tmpname);
    free  (tmpname);
    return w;

fail:
    if (tmpfile)
    {
        fclose(tmpfile);
        remove(tmpname);
    }
    ndt_wmm_close(&w);
    free(tmpname);
    return NULL;
}

void ndt_wmm_close(void **_wmm)
{
    ndt_wmm *wmm = *_wmm;

    if (wmm != NULL)
    {
        if (wmm->TimedMagneticModel != NULL)
        {
            MAG_FreeMagneticModelMemory(wmm->TimedMagneticModel);
        }
        if (wmm->MagneticModels[0] != NULL)
        {
            MAG_FreeMagneticModelMemory(wmm->MagneticModels[0]);
        }
    }

    *_wmm = NULL;
    free(wmm);
}

double ndt_wmm_getbearing_mag(void *wmm, double tru_bearing, ndt_position position, ndt_date date)
{
    double mag_bearing = ndt_mod(tru_bearing - get_magnetic_decl(wmm, position, date), 360.);
    return mag_bearing ? mag_bearing : 360.;
}

double ndt_wmm_getbearing_tru(void *wmm, double mag_bearing, ndt_position position, ndt_date date)
{
    double tru_bearing = ndt_mod(mag_bearing + get_magnetic_decl(wmm, position, date), 360.);
    return tru_bearing ? tru_bearing : 360.;
}
