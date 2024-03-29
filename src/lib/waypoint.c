/*
 * waypoint.c
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

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/common.h"

#include "compat/compat.h"

#include "wmm/wmm.h"

#include "waypoint.h"

ndt_waypoint* ndt_waypoint_init()
{
    ndt_waypoint *wpt = calloc(1, sizeof(ndt_waypoint));
    if (!wpt)
    {
        goto end;
    }

    // make it valid by default
    wpt->type     = NDT_WPTYPE_LLC;
    wpt->position = ndt_position_init(0., 0., ndt_distance_init(0, NDT_ALTUNIT_NA));

end:
    return wpt;
}

void ndt_waypoint_close(ndt_waypoint **_wpt)
{
    if (_wpt && *_wpt)
    {
        ndt_waypoint *wpt = *_wpt;

        free(wpt);

        *_wpt = NULL;
    }
}

ndt_waypoint* ndt_waypoint_posn(ndt_position position)
{
    ndt_waypoint *wpt = ndt_waypoint_init();
    if (wpt == NULL)
    {
        goto end;
    }

    double lat = ndt_position_getlatitude (position, NDT_ANGUNIT_DEG);
    double lon = ndt_position_getlongitude(position, NDT_ANGUNIT_DEG);

    /* Keep the identifier short */
    snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "%c%02d%c%03d",
             lat >= 0. ? 'N' : 'S', (int)round(fabs(lat)),
             lon >= 0. ? 'E' : 'W', (int)round(fabs(lon)));

    wpt->type     = NDT_WPTYPE_LLC;
    wpt->position = position;

end:
    return wpt;
}

ndt_waypoint* ndt_waypoint_llc(const char *fmt)
{
    if (!fmt)
    {
        goto end;
    }

    double lat, lon, latm, lonm, lats, lons;
    size_t fmtlen = strlen(fmt);
    char de1[2], de2[2];

    if ((fmtlen ==  7 && sscanf(fmt,  "%1[NSns]%2lf%1[EWew]%3lf", de1, &lat, de2, &lon) == 4) ||
        (fmtlen ==  8 && sscanf(fmt, "%1[NSns]%2lf/%1[EWew]%3lf", de1, &lat, de2, &lon) == 4) ||
        (fmtlen ==  7 && sscanf(fmt,  "%2lf%1[NSns]%3lf%1[EWew]", &lat, de1, &lon, de2) == 4) ||
        (fmtlen ==  8 && sscanf(fmt, "%2lf%1[NSns]/%3lf%1[EWew]", &lat, de1, &lon, de2) == 4))
    {
        /*
         * 7-character format: N44W066  (N 44° W 066°)
         * 8-character format: N44/W066 (N 44° W 066°)
         *
         * 7-character format: 44N066W  (N 44° W 066°)
         * 8-character format: 44N/066W (N 44° W 066°)
         */
        switch (*de1)
        {
            case 'N': // north latitude
            case 'n':
                break;
            case 'S': // south latitude
            case 's':
                lat = -lat;
                break;
            default:  // invalid value
                goto end;
        }
        switch (*de2)
        {
            case 'E': // east longitude
            case 'e':
                break;
            case 'W': // west longitude
            case 'w':
                lon = -lon;
                break;
            default:  // invalid value
                goto end;
        }
    }
    else if (((strstr(fmt, ".") == NULL)) &&
             ((fmtlen == 15 && sscanf(fmt,  "%1[NSns]%2lf%2lf%2lf%1[EWew]%3lf%2lf%2lf", de1, &lat, &latm, &lats, de2, &lon, &lonm, &lons) == 8) ||
              (fmtlen == 16 && sscanf(fmt, "%1[NSns]%2lf%2lf%2lf/%1[EWew]%3lf%2lf%2lf", de1, &lat, &latm, &lats, de2, &lon, &lonm, &lons) == 8) ||
              (fmtlen == 15 && sscanf(fmt,  "%2lf%2lf%2lf%1[NSns]%3lf%2lf%2lf%1[EWew]", &lat, &latm, &lats, de1, &lon, &lonm, &lons, de2) == 8) ||
              (fmtlen == 16 && sscanf(fmt, "%2lf%2lf%2lf%1[NSns]/%3lf%2lf%2lf%1[EWew]", &lat, &latm, &lats, de1, &lon, &lonm, &lons, de2) == 8)))
    {
        /*
         * 15-character format: N441154W0662206  (N 44° 11' 54" W 066° 22' 06")
         * 16-character format: N441154/W0662206 (N 44° 11' 54" W 066° 22' 06")
         *
         * 15-character format: 441154N0662206W  (N 44° 11' 54" W 066° 22' 06")
         * 16-character format: 441154N/0662206W (N 44° 11' 54" W 066° 22' 06")
         */
        lat = (lat + latm / 60. + lats / 3600.);
        lon = (lon + lonm / 60. + lons / 3600.);
        switch (*de1)
        {
            case 'N': // north latitude
            case 'n':
                break;
            case 'S': // south latitude
            case 's':
                lat = -lat;
                break;
            default:  // invalid value
                goto end;
        }
        switch (*de2)
        {
            case 'E': // east longitude
            case 'e':
                break;
            case 'W': // west longitude
            case 'w':
                lon = -lon;
                break;
            default:  // invalid value
                goto end;
        }
    }
    else if ((fmtlen == 11 && sscanf(fmt,  "%1[NSns]%2lf%2lf%1[EWew]%3lf%2lf", de1, &lat, &latm, de2, &lon, &lonm) == 6) ||
             (fmtlen == 12 && sscanf(fmt, "%1[NSns]%2lf%2lf/%1[EWew]%3lf%2lf", de1, &lat, &latm, de2, &lon, &lonm) == 6) ||
             (fmtlen == 11 && sscanf(fmt,  "%2lf%2lf%1[NSns]%3lf%2lf%1[EWew]", &lat, &latm, de1, &lon, &lonm, de2) == 6) ||
             (fmtlen == 12 && sscanf(fmt, "%2lf%2lf%1[NSns]/%3lf%2lf%1[EWew]", &lat, &latm, de1, &lon, &lonm, de2) == 6) ||
             (fmtlen == 15 && sscanf(fmt,  "%1[NSns]%2lf%4lf%1[EWew]%3lf%4lf", de1, &lat, &latm, de2, &lon, &lonm) == 6) ||
             (fmtlen == 16 && sscanf(fmt, "%1[NSns]%2lf%4lf/%1[EWew]%3lf%4lf", de1, &lat, &latm, de2, &lon, &lonm) == 6) ||
             (fmtlen == 15 && sscanf(fmt,  "%2lf%4lf%1[NSns]%3lf%4lf%1[EWew]", &lat, &latm, de1, &lon, &lonm, de2) == 6) ||
             (fmtlen == 16 && sscanf(fmt, "%2lf%4lf%1[NSns]/%3lf%4lf%1[EWew]", &lat, &latm, de1, &lon, &lonm, de2) == 6))
    {
        /*
         * 11-character format: N4411W06622  (N 44° 11' W 066° 22')
         * 12-character format: N4411/W06622 (N 44° 11' W 066° 22')
         *
         * 11-character format: 4411N06622W  (N 44° 11' W 066° 22')
         * 12-character format: 4411N/06622W (N 44° 11' W 066° 22')
         * ---> used by FlightAware as: 4411N 06622W
         *
         * 15-character format: N4411.9W06622.1  (N 44° 11.9' W 066° 22.1')
         * 16-character format: N4411.9/W06622.1 (N 44° 11.9' W 066° 22.1')
         *
         * 15-character format: 4411.9N06622.1W  (N 44° 11.9' W 066° 22.1')
         * 16-character format: 4411.9N/06622.1W (N 44° 11.9' W 066° 22.1')
         */
        lat = (lat + latm / 60.);
        lon = (lon + lonm / 60.);
        switch (*de1)
        {
            case 'N': // north latitude
            case 'n':
                break;
            case 'S': // south latitude
            case 's':
                lat = -lat;
                break;
            default:  // invalid value
                goto end;
        }
        switch (*de2)
        {
            case 'E': // east longitude
            case 'e':
                break;
            case 'W': // west longitude
            case 'w':
                lon = -lon;
                break;
            default:  // invalid value
                goto end;
        }
    }
    else if (fmtlen == 5 && sscanf(fmt,  "%2lf%2lf%1[NEWSnews]", &lat, &lon, de1) == 3)
    {
        /*
         * 5-character format: 4466N (N 44° W 066°)
         *
         * [latitude][longitude][designator]
         */
        switch (*de1)
        {
            case 'N': // north latitude, west longitude
            case 'n':
                lon = -lon;
                break;
            case 'E': // north latitude, east longitude
            case 'e':
                break;
            case 'W': // south latitude, west longitude
            case 'w':
                lat = -lat;
                lon = -lon;
                break;
            case 'S': // south latitude, east longitude
            case 's':
                lat = -lat;
                break;
            default:  // invalid value
                goto end;
        }
    }
    else if (fmtlen == 5 && sscanf(fmt,  "%2lf%1[NEWSnews]%2lf", &lat, de1, &lon) == 3)
    {
        /*
         * 5-character format: 44N66 (N 44° W 166°)
         *
         * [latitude][designator][longitude - 100]
         */
        lon += 100.;
        switch (*de1)
        {
            case 'N': // north latitude, west longitude
            case 'n':
                lon = -lon;
                break;
            case 'E': // north latitude, east longitude
            case 'e':
                break;
            case 'W': // south latitude, west longitude
            case 'w':
                lat = -lat;
                lon = -lon;
                break;
            case 'S': // south latitude, east longitude
            case 's':
                lat = -lat;
                break;
            default:  // invalid value
                goto end;
        }
    }
    else if (fmtlen >= 5 && fmtlen <= 22 && sscanf(fmt, "%9lf%1[NSns]/%10lf%1[EWew]", &lat, de1, &lon, de2) == 4)
    {
        /*
         * Easy match, check it late.
         *
         * variable length: 8N/3W                  (N 08.000000° W 003.000000°)
         * variable length: 44.444444S/111.111111E (S 44.444444° E 111.111111°)
         */
        switch (*de1)
        {
            case 'N': // north latitude
            case 'n':
                break;
            case 'S': // south latitude
            case 's':
                lat = -lat;
                break;
            default:  // invalid value
                goto end;
        }
        switch (*de2)
        {
            case 'E': // east longitude
            case 'e':
                break;
            case 'W': // west longitude
            case 'w':
                lon = -lon;
                break;
            default:  // invalid value
                goto end;
        }
    }
    else if (fmtlen >= 3 && fmtlen <= 22 && sscanf(fmt, "%10lf/%11lf", &lat, &lon) == 2)
    {
        /*
         * Easiest match, check it last.
         *
         * variable length: 8/3                    (N 08.000000° W 003.000000°)
         * variable length: -44.444444/-111.111111 (S 44.444444° E 111.111111°)
         *
         * Used e.g. in North Atlantic Track descriptions.
         */
    }
    else
    {
        /* Invalid or unsupported format */
        goto end;
    }

    if (round(fabs(lat)) > 90. || round(fabs(lon)) > 180.)
    {
        /* Invalid data */
        goto end;
    }

    return ndt_waypoint_posn(ndt_position_init(lat, lon, ndt_distance_init(0, NDT_ALTUNIT_NA)));

end:
    return NULL;
}

ndt_waypoint* ndt_waypoint_pbd(ndt_waypoint *plce, double magb, ndt_distance dist, void *wmm)
{
    ndt_waypoint *wpt = NULL;
    if (!plce || !wmm || !(wpt = ndt_waypoint_init()))
    {
        goto end;
    }
    wpt->pbd.distance = dist;
    wpt->pbd.bearing  = magb;
    wpt->pbd.place    = plce;
    wpt->type         = NDT_WPTYPE_PBD;
    wpt->position     = ndt_position_calcpos4pbd(plce->position,
                                                 ndt_wmm_getbearing_tru(wmm, magb, plce->position),
                                                 dist);
    snprintf(wpt->info.idnt, sizeof(wpt->info.idnt), "%s/%05.1lf/%05.1lf",
             plce->info.idnt, magb,
             ndt_distance_get(dist, NDT_ALTUNIT_ME) / 1852.);

end:
    return wpt;
}

ndt_waypoint* ndt_waypoint_pbpb(ndt_waypoint *src1, double mag1, ndt_waypoint *src2, double mag2, void *wmm)
{
    ndt_waypoint *wpt = NULL;
    if (!src1 || !src2 || !wmm || !(wpt = ndt_waypoint_init()))
    {
        goto end;
    }
    wpt->pbpb.wpt1 = src1;
    wpt->pbpb.brg1 = mag1;
    wpt->pbpb.wpt2 = src2;
    wpt->pbpb.brg2 = mag2;
    wpt->type      = NDT_WPTYPE_PBX;
    int rtval      = ndt_position_calcpos4pbpb(&wpt->position,
                                               src1->position,
                                               ndt_wmm_getbearing_tru(wmm, mag1, src1->position),
                                               src2->position,
                                               ndt_wmm_getbearing_tru(wmm, mag2, src2->position));
    switch (rtval)
    {
        case 0:
            break;
        case EDOM:
            ndt_log("%s %05.1lf, %s %05.1lf: infinity of intersections\n", src1->info.idnt, mag1, src2->info.idnt, mag2);
            ndt_waypoint_close(&wpt);
            goto end;
        case ERANGE:
            ndt_log("%s %05.1lf, %s %05.1lf: intersection(s) ambiguous\n", src1->info.idnt, mag1, src2->info.idnt, mag2);
            ndt_waypoint_close(&wpt);
            goto end;
        default:
            ndt_waypoint_close(&wpt);
            goto end;
    }
    snprintf( wpt->info.idnt, sizeof(wpt->info.idnt), "%s-%03.0lf/%s-%03.0lf",
             src1->info.idnt, mag1, src2->info.idnt, mag2);

end:
    return wpt;
}

ndt_waypoint* ndt_waypoint_pbpd(ndt_waypoint *src1, double magb, ndt_waypoint *src2, ndt_distance dist, void *wmm)
{
    ndt_waypoint *wpt = NULL;
    if (!src1 || !src2 || !wmm || !(wpt = ndt_waypoint_init()))
    {
        goto end;
    }
    wpt->pbpd.place    = src1;
    wpt->pbpd.bearing  = magb;
    wpt->pbpd.navaid   = src2;
    wpt->pbpd.distance = dist;
    wpt->type          = NDT_WPTYPE_INT;
    int rtval          = ndt_position_calcpos4pbpd(&wpt->position,
                                                   src1->position,
                                                   ndt_wmm_getbearing_tru(wmm, magb, src1->position),
                                                   src2->position, dist);
    switch (rtval)
    {
        case 0:
            break;
        case EDOM:
            ndt_log("%s %05.1lf, %s D%.1lf: not found\n", src1->info.idnt, magb,
                    src2->info.idnt, ndt_distance_get(dist, NDT_ALTUNIT_ME) / 1852.);
            ndt_waypoint_close(&wpt);
            goto end;
        default:
            ndt_waypoint_close(&wpt);
            goto end;
    }
    snprintf( wpt->info.idnt, sizeof(wpt->info.idnt), "%s-%03.0lf/%s-D%.1lf",
             src1->info.idnt, magb, src2->info.idnt, ndt_distance_get(dist, NDT_ALTUNIT_ME) / 1852.);

end:
    return wpt;
}
