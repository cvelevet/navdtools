/*
 * common.c
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
#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "compat/compat.h"

#include "common.h"

double ndt_mod(double y, double x)
{
    return  y - x * floor(y / x);
}

static ndt_log_callback *log_callback = NULL;

int ndt_log(const char *fmt, ...)
{
    int ret;
    va_list ap;
    va_start(ap, fmt);
    if (log_callback)
    {
        ret = log_callback(fmt, ap);
    }
    else
    {
        ret = vfprintf(stderr, fmt, ap);
    }
    va_end(ap);
    return ret;
}

int ndt_fprintf(FILE *fd, const char *fmt, ...)
{
    if (fd && fmt)
    {
        va_list ap;
        va_start(ap, fmt);
        int ret = vfprintf(fd, fmt, ap);
        va_end(ap);
        return ret < 0 ? errno : 0;
    }
    return !fd ? EBADF : EINVAL;
}

void ndt_log_set_callback(ndt_log_callback *callback)
{
    log_callback = callback;
}

char* ndt_file_slurp(const char *name, int *p)
{
    long  flen;
    int   ret  = 0;
    char *file = NULL;
    FILE *fdes = fopen(name ? name : "", "rb");

    if (!fdes)
    {
        ret = errno;
        goto end;
    }

    if (fseek(fdes, 0L, SEEK_END) < 0)
    {
        ret = errno;
        goto end;
    }

    flen = ftell(fdes);

    if (flen < 0)
    {
        ret = errno;
        goto end;
    }

    if (fseek(fdes, 0L, SEEK_SET) < 0)
    {
        ret = errno;
        goto end;
    }

    file = malloc(flen + 2);

    if (!file)
    {
        ret = ENOMEM;
        goto end;
    }

    if (fread(file, 1, flen, fdes) != flen)
    {
        ret = EIO;
        goto end;
    }

    if (file[flen - 1] != '\n')
    {
        file[flen++]    = '\n';
    }
    file[flen] = '\0';

end:
    if (ret)
    {
        free(file);
        file = NULL;
    }
    if (p) *p = ret;
    fclose(fdes);
    return file;
}

int ndt_file_getline(char **_l, int *_c, char **_p)
{
    if (!_l || !_c || !_p)
    {
        return -1;
    }

    char *l = *_l;
    int   c = *_c;
    char *p = *_p;
    char *n = strpbrk(p ? p : "", "\r\n");
    size_t len;

    if (!n)
    {
        return 0;
    }

    *_p = n + 1;
    len = n + 1 - p;

    if (!l || len >= c)
    {
        c = len + 1;
        l = realloc(l, c);
    }

    if (!l)
    {
        return -1;
    }

    strncpy(l, "",  2);
    strncat(l, p, len);

    *_l = l;
    *_c = c;
    return len;
}

int ndt_file_getpath(const char *b, const char *s, char **_p, int *_c)
{
    if ((!b && !s) || !_p || !_c)
    {
        return -1;
    }

    size_t bln = b ? strlen(b) : 0;
    size_t sln = s ? strlen(s) : 0;
    size_t len = bln + sln;
    char  *p   = *_p;
    int    c   = *_c;

    if (!p || len >= c)
    {
        c = len + 1;
        p = realloc(p, c);
    }

    if (!p)
    {
        return -1;
    }

    strncpy(p, "",  2);
    strncat(p, b, bln);
    strncat(p, s, sln);

    *_p = p;
    *_c = c;
    return 0;
}

ndt_airspeed ndt_airspeed_init(int64_t s, int u)
{
    ndt_airspeed airspeed;

    switch (u)
    {
        case NDT_SPDUNIT_FPS:
            airspeed.value = s * INT64_C(1097280);
            airspeed.unit  = NDT_SPDUNIT_NAT;
            break;

        case NDT_SPDUNIT_FPM:
            airspeed.value = s * INT64_C(18288);
            airspeed.unit  = NDT_SPDUNIT_NAT;
            break;

        case NDT_SPDUNIT_KPH:
            airspeed.value = s * INT64_C(1000000);
            airspeed.unit  = NDT_SPDUNIT_NAT;
            break;

        case NDT_SPDUNIT_KTS:
            airspeed.value = s * INT64_C(1852000);
            airspeed.unit  = NDT_SPDUNIT_NAT;
            break;

        case NDT_SPDUNIT_MACH:
            airspeed.value = s;
            airspeed.unit  = NDT_SPDUNIT_MACH;
            break;

        case NDT_SPDUNIT_MPH:
            airspeed.value = s * INT64_C(1609344);
            airspeed.unit  = NDT_SPDUNIT_NAT;
            break;

        case NDT_SPDUNIT_MTS:
            airspeed.value = s * INT64_C(3600000);
            airspeed.unit  = NDT_SPDUNIT_NAT;
            break;

        default:
            airspeed.value = s;
            airspeed.unit  = NDT_SPDUNIT_NAT;
            break;
    }

    return airspeed;
}

ndt_airspeed ndt_airspeed_mach(double oat)
{
    double speedofsound = (331.3 + (.6 * oat)) * 3600000.;
    return ndt_airspeed_init(speedofsound, NDT_SPDUNIT_NAT);
}

int64_t ndt_airspeed_get(ndt_airspeed s, int u, ndt_airspeed m)
{
    int64_t value;

    if (s.unit == u)
    {
        return s.value;
    }

    switch (s.unit)
    {
        case NDT_SPDUNIT_MACH:
            value = s.value * m.value / 1000;
            break;

        case NDT_SPDUNIT_NAT:
            value = s.value;
            break;

        default:
            return ndt_airspeed_get(ndt_airspeed_init(s.value, s.unit), u, m);
    }

    switch (u)
    {
        case NDT_SPDUNIT_FPS:
            return value / INT64_C(1097280);

        case NDT_SPDUNIT_FPM:
            return value / INT64_C(18288);

        case NDT_SPDUNIT_KPH:
            return value / INT64_C(1000000);

        case NDT_SPDUNIT_KTS:
            return value / INT64_C(1852000);

        case NDT_SPDUNIT_MPH:
            return value / INT64_C(1609344);

        case NDT_SPDUNIT_MTS:
            return value / INT64_C(3600000);

        case NDT_SPDUNIT_MACH:
            return value * INT64_C(1000) / m.value;

        default:
            return value;
    }
}

ndt_distance ndt_distance_init(int64_t d, int u)
{
    ndt_distance distance;

    switch (u)
    {
        case NDT_ALTUNIT_FL:
            distance.value = d * INT64_C(304800);
            break;

        case NDT_ALTUNIT_FT:
            distance.value = d * INT64_C(3048);
            break;

        case NDT_ALTUNIT_ME:
            distance.value = d * INT64_C(10000);
            break;

        case NDT_ALTUNIT_NM:
            distance.value = d * INT64_C(18520000);
            break;

        default:
            distance.value = d;
            break;
    }

    return distance;
}

int64_t ndt_distance_get(ndt_distance d, int u)
{
    switch (u)
    {
        case NDT_ALTUNIT_FL:
            return d.value / INT64_C(304800);

        case NDT_ALTUNIT_FT:
            return d.value / INT64_C(3048);

        case NDT_ALTUNIT_ME:
            return d.value / INT64_C(10000);

        case NDT_ALTUNIT_NM:
            return d.value / INT64_C(18520000);

        default:
            return d.value;
    }
}

ndt_distance ndt_distance_max(ndt_distance d1, ndt_distance d2)
{
    if (d1.value > d2.value)
    {
        return d1;
    }
    return d2;
}

ndt_distance ndt_distance_min(ndt_distance d1, ndt_distance d2)
{
    if (d1.value < d2.value)
    {
        return d1;
    }
    return d2;
}

ndt_distance ndt_distance_add(ndt_distance d1, ndt_distance d2)
{
    ndt_distance sum = { .value = d1.value + d2.value, };
    return       sum;
}

ndt_distance ndt_distance_rem(ndt_distance d1, ndt_distance d2)
{
    ndt_distance sub = { .value = d1.value - d2.value, };
    return       sub;
}

#define INTSEC (     1650)                     // ((INT_MAX / (361 * 3601)) - 1)
#define INTMIN (  60*1650)
#define INTDEG (3600*1650)
#define DECSEC ((double)INTSEC)
#define DECMIN ((double)INTMIN)
#define DECDEG ((double)INTDEG)
ndt_position ndt_position_init(double lat, double lon, ndt_distance alt)
{
    ndt_position position;

    position.latitude. equator  = ((lat * DECDEG) >= 0. ? 1 : -1);
    position.latitude. value    = ((lat * DECDEG) * position.latitude. equator);
    position.longitude.meridian = ((lon * DECDEG) >= 0. ? 1 : -1);
    position.longitude.value    = ((lon * DECDEG) * position.longitude.meridian);

    position.altitude     = alt;
    position.precision[0] = ndt_distance_init(1, NDT_ALTUNIT_FT);
    position.precision[1] = ndt_distance_init(1, NDT_ALTUNIT_FT);

    return position;
}

double ndt_position_getlatitude(ndt_position pos, ndt_angle_unit aut)
{
    if (aut == NDT_ANGUNIT_RAD)
    {
        return ndt_position_getlatitude(pos, NDT_ANGUNIT_DEG) * M_PI / 180.;
    }
    else
    {
        return ((double)(pos.latitude.equator * pos.latitude.value) / DECDEG);
    }
}

double ndt_position_getlongitude(ndt_position pos, ndt_angle_unit aut)
{
    if (aut == NDT_ANGUNIT_RAD)
    {
        return ndt_position_getlongitude(pos, NDT_ANGUNIT_DEG) * M_PI / 180.;
    }
    else
    {
        return ((double)(pos.longitude.meridian * pos.longitude.value) / DECDEG);
    }
}

ndt_distance ndt_position_getaltitude(ndt_position pos)
{
    return pos.altitude;
}

void ndt_position_getprecision(ndt_position pos, ndt_distance *p[2])
{
    if (p)
    {
        *p[0] = pos.precision[0];
        *p[1] = pos.precision[1];
    }
}

void ndt_position_setprecision(ndt_position *pos, ndt_distance p[2])
{
    if (pos)
    {
        pos->precision[0] = p[0];
        pos->precision[1] = p[1];
    }
}

double ndt_position_calcbearing(ndt_position from, ndt_position to)
{
    if (!memcmp(&from.latitude,  &to.latitude,  sizeof(to.latitude)) &&
        !memcmp(&from.longitude, &to.longitude, sizeof(to.longitude)))
    {
        return 0.;
    }

    /*
     * http://williams.best.vwh.net/avform.htm#Crs
     */
    double lat1 = ndt_position_getlatitude (from, NDT_ANGUNIT_RAD);
    double lon1 = ndt_position_getlongitude(from, NDT_ANGUNIT_RAD) * -1.;
    double lat2 = ndt_position_getlatitude (to,   NDT_ANGUNIT_RAD);
    double lon2 = ndt_position_getlongitude(to,   NDT_ANGUNIT_RAD) * -1.;
    double cr12 = ndt_mod(atan2(sin(lon1 - lon2) * cos(lat2),
                                cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon1 - lon2)),
                          2. * M_PI);

    double degr = cr12 * 180. / M_PI;
    return degr ? degr : 360.;
}

double ndt_position_bearing_angle(double bearing, double targetb)
{
    double brgdiff = ndt_mod(targetb, 360.) - ndt_mod(bearing, 360.);
    if (fabs(brgdiff) > 180.)
    {
        return ndt_position_angle_reverse(brgdiff);
    }
    return brgdiff == 180. ? -180. : brgdiff; // default left
}

double ndt_position_angle_reverse(double obverse)
{
    if (obverse > 0.)
    {
        double reverse = ndt_mod(obverse - 360., -360.); // e.g. +90. to -270.
        return reverse ? reverse : -360.;
    }
    if (obverse < 0.)
    {
        double reverse = ndt_mod(obverse + 360., +360.); // e.g. -90. to +270.
        return reverse ? reverse : +360.;
    }
    return 0.; // obverse == +0. || obverse == -0. (no angle)
}

/*
 * Earth radius: one minute == 1/60 degree == 1 nautical mile.
 *
 * Check: X-Plane 10.36, default 747, default FMC: LSGK -D-> YSSY
 * Note with default scenery, LSGK ramp 0.0nm from LSGK airport's waypoint.
 */
#define EARTHRAD (6366707.0195)

ndt_distance ndt_position_calcdistance(ndt_position from, ndt_position to)
{
    if (!memcmp(&from.latitude,  &to.latitude,  sizeof(to.latitude)) &&
        !memcmp(&from.longitude, &to.longitude, sizeof(to.longitude)))
    {
        return NDT_DISTANCE_ZERO;
    }

    /*
     * http://williams.best.vwh.net/avform.htm#Dist
     */
    double lat1 = ndt_position_getlatitude (from, NDT_ANGUNIT_RAD);
    double lon1 = ndt_position_getlongitude(from, NDT_ANGUNIT_RAD) * -1.;
    double lat2 = ndt_position_getlatitude (to,   NDT_ANGUNIT_RAD);
    double lon2 = ndt_position_getlongitude(to,   NDT_ANGUNIT_RAD) * -1.;
    double dis2 = 2. * asin(sqrt(pow(sin((lat1 - lat2) / 2.), 2.) +
                                 pow(sin((lon1 - lon2) / 2.), 2.) * cos(lat1) * cos(lat2)));

    return ndt_distance_init(EARTHRAD * dis2, NDT_ALTUNIT_ME);
}

int ndt_position_calcduration(ndt_position from, ndt_position to, ndt_airspeed at)
{
    return 0;
}

int ndt_position_calcintercept(ndt_position from, ndt_position to, ndt_position orig)
{
    return 0;
}

ndt_position ndt_position_calcpos4pbd(ndt_position from, double trub, ndt_distance dist)
{
    double cr12 = trub / 180. * M_PI;
    double dis2 = ndt_distance_get(dist, NDT_ALTUNIT_ME) / EARTHRAD;
    double lat1 = ndt_position_getlatitude (from, NDT_ANGUNIT_RAD);
    double lon1 = ndt_position_getlongitude(from, NDT_ANGUNIT_RAD) * -1.;

    /*
     * http://williams.best.vwh.net/avform.htm#LL
     */
    double lat2 = asin (sin(lat1) * cos(dis2) +
                        cos(lat1) * sin(dis2) * cos(cr12));
    double dlon = atan2(sin(cr12) * sin(dis2) * cos(lat1),
                        cos(dis2) - sin(lat1) * sin(lat2));
    double lon2 = ndt_mod(lon1 - dlon + M_PI, 2. * M_PI) - M_PI;

    /* Don't forget to convert radians to decimal degrees */
    return ndt_position_init(lat2 * 180. / M_PI,
                             lon2 * 180. / M_PI * -1., NDT_DISTANCE_ZERO);
}

int ndt_position_calcpos4pbpb(ndt_position *out, ndt_position pos1, double tru1, ndt_position pos2, double tru2)
{
    double lat1 = ndt_position_getlatitude (pos1, NDT_ANGUNIT_RAD);
    double lon1 = ndt_position_getlongitude(pos1, NDT_ANGUNIT_RAD) * -1.;
    double lat2 = ndt_position_getlatitude (pos2, NDT_ANGUNIT_RAD);
    double lon2 = ndt_position_getlongitude(pos2, NDT_ANGUNIT_RAD) * -1.;

    /*
     * http://williams.best.vwh.net/avform.htm#Dist
     */
    double dis2 = 2. * asin(sqrt(pow(sin((lat1 - lat2) / 2.), 2.) +
                                 pow(sin((lon1 - lon2) / 2.), 2.) * cos(lat1) * cos(lat2)));

    /*
     * http://williams.best.vwh.net/avform.htm#Crs
     */
    double cr12 = ndt_mod(atan2(sin(lon1 - lon2) * cos(lat2),
                                cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon1 - lon2)),
                          2. * M_PI);
    double cr21 = ndt_mod(atan2(sin(lon2 - lon1) * cos(lat1),
                                cos(lat2) * sin(lat1) - sin(lat2) * cos(lat1) * cos(lon2 - lon1)),
                          2. * M_PI);

    /*
     * http://williams.best.vwh.net/avform.htm#Intersection
     */
    double cr13 = tru1 / 180. * M_PI;
    double cr23 = tru2 / 180. * M_PI;
    double a1a3 = ndt_mod(cr13 - cr12 + M_PI, 2. * M_PI) - M_PI;
    double a2a3 = ndt_mod(cr21 - cr23 + M_PI, 2. * M_PI) - M_PI;

    if (!sin(a1a3) && !sin(a2a3))
    {
        return EDOM; // "infinity of intersections"
    }
    if (sin(a1a3) * sin(a2a3) < 0.)
    {
        return ERANGE; // "intersection ambiguous"
    }

    double ang1 = fabs (a1a3);
    double ang2 = fabs (a2a3);
    double ang3 = acos (cos(ang1) * cos(ang2) * -1.    +    sin(ang1) * sin(ang2) * cos(dis2));
    double dis3 = atan2(sin(dis2) * sin(ang1) * sin(ang2),  cos(ang2) + cos(ang1) * cos(ang3));
    double lat3 = asin (sin(lat1) * cos(dis2) + cos(lat1) * sin(dis3) * cos(cr13));
    double dlon = atan2(sin(cr13) * sin(dis3) * cos(lat1),  cos(dis3) - sin(lat1) * sin(lat3));
    double lon3 = ndt_mod(lon1 - dlon + M_PI, 2. * M_PI) - M_PI;
    ndt_position pos3 = ndt_position_init(lat3 * 180. / M_PI, lon3 * 180. / M_PI * -1., NDT_DISTANCE_ZERO);
    if (out) *out = pos3;
    return 0;
}

int ndt_position_calcpos4pbpd(ndt_position *out, ndt_position pos1, double trub, ndt_position pos2, ndt_distance dist)
{
    ndt_position posn;

    /*
     * If pos1 == pos2, math is really easy :)
     */
    if (ndt_distance_get(ndt_position_calcdistance(pos1, pos2), NDT_ALTUNIT_FT) < INT64_C(66))
    {
        posn = ndt_position_calcpos4pbd(pos1, trub, dist);
        goto end;
    }

    /*
     * Special cases where we're outbound or inbound pos2, math is straightforward.
     */
    double br12 = ndt_position_calcbearing(pos1, pos2);
    if (fabs(ndt_position_bearing_angle   (br12, trub)) < 2.)
    {
        // we're headed straight for pos2
        ndt_distance p1p2 = ndt_position_calcdistance(pos1, pos2);
        if (ndt_distance_get(p1p2, NDT_ALTUNIT_NA) >
            ndt_distance_get(dist, NDT_ALTUNIT_NA))
        {
            // we stop before reaching pos2
            posn = ndt_position_calcpos4pbd(pos1, br12, ndt_distance_rem(p1p2, dist));
        }
        else
        {
            // we reach pos2 and continue past it
            posn = ndt_position_calcpos4pbd(pos1, br12, ndt_distance_add(p1p2, dist));
        }
        goto end;
    }
    double br21 = ndt_position_calcbearing(pos2, pos1);
    if (fabs(ndt_position_bearing_angle   (br21, trub)) < 2.)
    {
        // pos1 -> (pos2 dist) is direct continuation of pos2 -> pos1
        posn = ndt_position_calcpos4pbd(pos2, br21, dist);
        goto end;
    }

    /*
     * Most naive implementation imaginable :(
     */
    ndt_position tpos;
    double       diff;
    int  found = 0;
    for (int i = 0; i < 360; i++)
    {
        if (!ndt_position_calcpos4pbpb(&tpos, pos1, trub, pos2, (double)i))
        {
            ndt_distance  p2tp = ndt_position_calcdistance(pos2, tpos);
            double        tdis = ndt_distance_get(ndt_distance_rem(dist, p2tp), NDT_ALTUNIT_FT);
            double        tdif = fabs(tdis);
            if (!found || tdif < diff)
            {
                diff   =  tdif;
                posn   =  tpos;
                found  = i + 1;
            }
        }
    }
    if (found) // refine to reduce diff
    {
        double rmin = ndt_mod((found - 1) - 1., 360.);
        double rmax = ndt_mod((found - 1) + 1., 360.);
        for (double i = rmin; i < rmax; i += 2./360.)
        {
            if (!ndt_position_calcpos4pbpb(&tpos, pos1, trub, pos2, i))
            {
                ndt_distance p2tp = ndt_position_calcdistance(pos2, tpos);
                double       tdis = ndt_distance_get(ndt_distance_rem(dist, p2tp), NDT_ALTUNIT_FT);
                double       tdif = fabs(tdis);
                if (diff  >  tdif)
                {
                    diff  =  tdif;
                    posn  =  tpos;
                }
            }
        }
    }
    if (!found || diff > 660.) // off by max. 1 furlong (660 feet, ~200m)
    {
        return EDOM;
    }
end:
    if (out) *out = posn;
    return 0;
}

int ndt_position_sprintllc(ndt_position pos, ndt_llcfmt fmt, char *buf, size_t len)
{
    if (!buf)
    {
        return -1;
    }

    int    ret;
    char   card[3];
    int    ilatd = ((pos.latitude. value)                                   / INTDEG);
    int    ilond = ((pos.longitude.value)                                   / INTDEG);
    int    ilatm = ((pos.latitude. value - ilatd * INTDEG)                  / INTMIN);
    int    ilonm = ((pos.longitude.value - ilond * INTDEG)                  / INTMIN);
    int    ilats = ((pos.latitude. value - ilatd * INTDEG - ilatm * INTMIN) / INTSEC);
    int    ilons = ((pos.longitude.value - ilond * INTDEG - ilonm * INTMIN) / INTSEC);
    double dlatd = ((pos.latitude. value)                                   / DECDEG);
    double dlond = ((pos.longitude.value)                                   / DECDEG);
    double dlatm = ((pos.latitude. value - ilatd * INTDEG)                  / DECMIN);
    double dlonm = ((pos.longitude.value - ilond * INTDEG)                  / DECMIN);
    double dlats = ((pos.latitude. value - ilatd * INTDEG - ilatm * INTMIN) / DECSEC);
    double dlons = ((pos.longitude.value - ilond * INTDEG - ilonm * INTMIN) / DECSEC);

    switch (pos.latitude.equator)
    {
        case 1:
            card[0] =                                     'N';
            card[1] = pos.longitude.meridian == 1 ? 'E' : 'W';
            card[2] = pos.longitude.meridian == 1 ? 'E' : 'N';
            break;
        case -1:
        default:
            card[0] =                                     'S';
            card[1] = pos.longitude.meridian == 1 ? 'E' : 'W';
            card[2] = pos.longitude.meridian == 1 ? 'S' : 'W';
            break;
    }

    switch (fmt)
    {
        case NDT_LLCFMT_AIBUS:
        {
            // full form, e.g. 4600.0N/05000.0E
            ret = snprintf(buf, len, "%02d%04.1lf%c/%03d%04.1lf%c",
                           ilatd, dlatm, card[0], ilond, dlonm, card[1]);
            break;
        }

        case NDT_LLCFMT_BOING:
        {
            if (ilatm == 0 && ilonm == 0 && ilats < 6 && ilons < 6)
            {
                // 7-letter form, e.g. N46E050
                ret = snprintf(buf, len, "%c%02d%c%03d",
                               card[0], ilatd, card[1], ilond);
                break;
            }
            // full form, e.g. N4600.0E05000.0
            ret = snprintf(buf, len, "%c%02d%04.1lf%c%03d%04.1lf",
                           card[0], ilatd, dlatm, card[1], ilond, dlonm);
            break;
        }

        case NDT_LLCFMT_CEEVA:
        {
            // full form, e.g. N 46°00.0' E 050°00.0'
            ret = snprintf(buf, len, "%c %02d°%04.1lf' %c %03d°%04.1lf'",
                           card[0], ilatd, dlatm, card[1], ilond, dlonm);
            break;
        }

        case NDT_LLCFMT_ICAOR:
        {
            if (ilatm == 0 && ilonm == 0 && ilats < 30 && ilons < 30)
            {
                // 7-letter form, e.g. 46N050E
                ret = snprintf(buf, len, "%02d%c%03d%c",
                               ilatd, card[0], ilond, card[1]);
                break;
            }
            // full form, e.g. 4600N05000E
            ret = snprintf(buf, len, "%02d%02d%c%03d%02d%c",
                           ilatd, (int)(dlatm + .5), card[0],
                           ilond, (int)(dlonm + .5), card[1]);
            break;
        }

        case NDT_LLCFMT_RECAP:
        {
            // full form, e.g. N46°00.0'  E050°00.0'
            ret = snprintf(buf, len, "%c%02d°%04.1lf'  %c%03d°%04.1lf'",
                           card[0], ilatd, dlatm, card[1], ilond, dlonm);
            break;
        }

        case NDT_LLCFMT_SBRIF:
        {
            if (ilatm == 0 && ilonm == 0 && ilats < 30 && ilons < 30)
            {
                // 5-letter form, e.g. 4650N
                if (ilond >= 100)
                {
                    ret = snprintf(buf, len, "%02d%c%02d",
                                   ilatd, card[2], ilond % 100);
                }
                else
                {
                    ret = snprintf(buf, len, "%02d%02d%c",
                                   ilatd, ilond, card[2]);
                }
                break;
            }
            // FlightAware-style, e.g. 4600N 05000E
            ret = snprintf(buf, len, "%02d%02d%c %03d%02d%c",
                           ilatd, (int)(dlatm + .5), card[0],
                           ilond, (int)(dlonm + .5), card[1]);
            break;
        }

        case NDT_LLCFMT_SVECT:
        {
            if (ilatm == 0 && ilonm == 0 && ilats == 0 && ilons == 0)
            {
                // 5-letter form, e.g. 4650N
                if (ilond >= 100)
                {
                    ret = snprintf(buf, len, "%02d%c%02d",
                                   ilatd, card[2], ilond % 100);
                }
                else
                {
                    ret = snprintf(buf, len, "%02d%02d%c",
                                   ilatd, ilond, card[2]);
                }
                break;
            }
            if (ilats == 0 && ilons == 0)
            {
                // 9-letter form, e.g. 4600N05000E
                ret = snprintf(buf, len, "%02d%02d%c%03d%02d%c",
                               ilatd, ilatm, card[0],
                               ilond, ilonm, card[1]);
                break;
            }
            // full form, e.g. 460000N0500000E
            ret = snprintf(buf, len, "%02d%02d%02d%c%03d%02d%02d%c",
                           ilatd, ilatm, ilats, card[0],
                           ilond, ilonm, ilons, card[1]);
            break;
        }

        case NDT_LLCFMT_DEFS5:
            // short format: 5-letter form, e.g. 4650N
            if ((int)(dlond + .5) >= 100)
            {
                ret = snprintf(buf, len, "%02d%c%02d",
                               (int)(dlatd + .5), card[2],
                               (int)(dlond + .5) % 100);
            }
            else
            {
                ret = snprintf(buf, len, "%02d%02d%c",
                               (int)(dlatd + .5),
                               (int)(dlond + .5), card[2]);
            }
            break;

        case NDT_LLCFMT_DEFLT:
        default:
            // default format: 7-letter form, e.g. N46E050
            ret = snprintf(buf, len, "%c%02d%c%03d",
                           card[0], (int)(dlatd + .5),
                           card[1], (int)(dlond + .5));
            break;
    }

    return ret >= len ? -1 : ret;
}

int ndt_position_fprintllc(ndt_position pos, ndt_llcfmt fmt, FILE *fd)
{
    char buf[25];
    int  ret = ndt_position_sprintllc(pos, fmt, buf, sizeof(buf));

    if (ret < 0)
    {
        return EIO;
    }

    return ndt_fprintf(fd, "%s", buf);
}

ndt_frequency ndt_frequency_init(double f)
{
    ndt_frequency frequency;

    frequency.value = f * 120;

    return frequency;
}

double ndt_frequency_get(ndt_frequency f)
{
    return f.value / 120.;
}

ndt_date ndt_date_init(time_t time)
{
    ndt_date date = { 0 };
    struct tm zulu;

#ifdef _WIN32
    if (gmtime_s(&zulu, &time) == 0)
#else
    if (gmtime_r(&time, &zulu) != NULL)
#endif
    {
        date.year    = zulu.tm_year + 1900;
        date.month   = zulu.tm_mon  + 1;
        date.day     = zulu.tm_mday;
        date.hours   = zulu.tm_hour;
        date.minutes = zulu.tm_min;
        date.seconds = zulu.tm_sec;
    }

    return date;
}

time_t ndt_date_get(ndt_date date)
{
    struct tm zulu = { 0 };

    zulu.tm_year = date.year  - 1900;
    zulu.tm_mon  = date.month - 1;
    zulu.tm_mday = date.day;
    zulu.tm_hour = date.hours;
    zulu.tm_min  = date.minutes;
    zulu.tm_sec  = date.seconds;

    return mktime(&zulu);
}

ndt_date ndt_date_now(void)
{
    return ndt_date_init(time(NULL));
}
