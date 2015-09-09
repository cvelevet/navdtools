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
            distance.unit  = NDT_ALTUNIT_NA;
            break;

        case NDT_ALTUNIT_FT:
            distance.value = d * INT64_C(3048);
            distance.unit  = NDT_ALTUNIT_NA;
            break;

        case NDT_ALTUNIT_ME:
            distance.value = d * INT64_C(10000);
            distance.unit  = NDT_ALTUNIT_NA;
            break;

        case NDT_ALTUNIT_NM:
            distance.value = d * INT64_C(18520000);
            distance.unit  = NDT_ALTUNIT_NA;
            break;

        default:
            distance.value = d;
            distance.unit  = NDT_ALTUNIT_NA;
            break;
    }

    return distance;
}

int64_t ndt_distance_get(ndt_distance d, int u)
{
    if (d.unit == u)
    {
        return d.value;
    }

    switch (d.unit)
    {
        case NDT_ALTUNIT_NA:
            break;

        default:
            return ndt_distance_get(ndt_distance_init(d.value, d.unit), u);
    }

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

ndt_position ndt_position_init(double lat, double lon, ndt_distance alt)
{
    ndt_position position;

    position.latitude. equator  = ((lat * 21600.) >= 0. ? 1 : -1);
    position.latitude. thirds   = ((lat * 21600.) * position.latitude. equator);
    position.longitude.meridian = ((lon * 21600.) >= 0. ? 1 : -1);
    position.longitude.thirds   = ((lon * 21600.) * position.longitude.meridian);

    position.altitude     = alt;
    position.precision[0] = ndt_distance_init(1, alt.unit);
    position.precision[1] = ndt_distance_init(1, alt.unit);

    return position;
}

double ndt_position_getlatitude(ndt_position pos, ndt_angle_unit aut)
{
    if (aut == NDT_ANGUNIT_RAD)
    {
        return ((double)(pos.latitude.equator * pos.latitude.thirds) * M_PI / 3888000.);
    }
    else
    {
        return ((double)(pos.latitude.equator * pos.latitude.thirds) / 21600.);
    }
}

double ndt_position_getlongitude(ndt_position pos, ndt_angle_unit aut)
{
    if (aut == NDT_ANGUNIT_RAD)
    {
        return ((double)(pos.longitude.meridian * pos.longitude.thirds) * M_PI / 3888000.);
    }
    else
    {
        return ((double)(pos.longitude.meridian * pos.longitude.thirds) / 21600.);
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
    double lat2 = ndt_position_getlatitude (to,   NDT_ANGUNIT_RAD);
    double lon1 = ndt_position_getlongitude(from, NDT_ANGUNIT_RAD);
    double lon2 = ndt_position_getlongitude(to,   NDT_ANGUNIT_RAD);
    double brng = 2 * M_PI - ndt_mod(atan2(sin(lon1 - lon2) * cos(lat2),
                                           cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon1 - lon2)),
                                     2 * M_PI);

    double degr = brng * 180. / M_PI;
    return degr ? degr : 360.;
}

ndt_distance ndt_position_calcdistance(ndt_position from, ndt_position to)
{
    if (!memcmp(&from.latitude,  &to.latitude,  sizeof(to.latitude)) &&
        !memcmp(&from.longitude, &to.longitude, sizeof(to.longitude)))
    {
        return ndt_distance_init(0, NDT_ALTUNIT_NA);
    }

    /*
     * http://williams.best.vwh.net/avform.htm#Dist
     *
     * Earth radius (ellipsoidal quadratic mean): approx. 6,372,800 meters
     */
    double  lat1 = ndt_position_getlatitude (from, NDT_ANGUNIT_RAD);
    double  lat2 = ndt_position_getlatitude (to,   NDT_ANGUNIT_RAD);
    double  lon1 = ndt_position_getlongitude(from, NDT_ANGUNIT_RAD);
    double  lon2 = ndt_position_getlongitude(to,   NDT_ANGUNIT_RAD);
    int64_t dist = 12745600. * asin(sqrt(pow(sin((lat1 - lat2) / 2.), 2.) +
                                         pow(sin((lon1 - lon2) / 2.), 2.) * cos(lat1) * cos(lat2)));

    return ndt_distance_init(dist, NDT_ALTUNIT_ME);
}

int ndt_position_calcduration(ndt_position from, ndt_position to, ndt_airspeed at)
{
    /* TODO: implement */
    return 0;
}

int ndt_position_calcintercept(ndt_position from, ndt_position to, ndt_position orig)
{
    /* TODO: implement */
    return 0;
}

ndt_position ndt_position_calcpos4pbd(ndt_position from, double trub, ndt_distance dist)
{
    /*
     * Earth radius (ellipsoidal quadratic mean): approx. 6,372,800 meters
     */
    double brg0 = (360. - trub) / 180. * M_PI;
    double dis0 = (double)ndt_distance_get (dist, NDT_ALTUNIT_ME) / 6372800.;
    double lat1 = ndt_position_getlatitude (from, NDT_ANGUNIT_RAD);
    double lon1 = ndt_position_getlongitude(from, NDT_ANGUNIT_RAD);

    /*
     * http://williams.best.vwh.net/avform.htm#LL
     */
    double lat2 = asin (sin(lat1) * cos(dis0) +
                        cos(lat1) * sin(dis0) * cos(brg0));
    double dlon = atan2(sin(brg0) * sin(dis0) * cos(lat1),
                        cos(dis0) - sin(lat1) * sin(lat2));
    double lon2 = ndt_mod(lon1 - dlon + M_PI, 2. * M_PI) - M_PI;

    /* Don't forget to convert radians to decimal degrees */
    return ndt_position_init(lat2 * 180. / M_PI,
                             lon2 * 180. / M_PI,
                             ndt_distance_init(0, NDT_ALTUNIT_NA));
}

int ndt_position_sprintllc(ndt_position pos, ndt_llcfmt fmt, char *buf, size_t len)
{
    if (!buf)
    {
        return len + 1;
    }

    char   card[3];
    int    ilatd = ((pos.latitude. thirds)                               / 21600);
    int    ilond = ((pos.longitude.thirds)                               / 21600);
    int    ilatm = ((pos.latitude. thirds - ilatd * 21600)               /   360);
    int    ilonm = ((pos.longitude.thirds - ilond * 21600)               /   360);
    int    ilats = ((pos.latitude. thirds - ilatd * 21600 - ilatm * 360) /    60);
    int    ilons = ((pos.longitude.thirds - ilond * 21600 - ilonm * 360) /    60);
    double dlatd = ((pos.latitude. thirds)                               / 21600.);
    double dlond = ((pos.longitude.thirds)                               / 21600.);
    double dlatm = ((pos.latitude. thirds - ilatd * 21600)               /   360.);
    double dlonm = ((pos.longitude.thirds - ilond * 21600)               /   360.);
    double dlats = ((pos.latitude. thirds - ilatd * 21600 - ilatm * 360) /    60.);
    double dlons = ((pos.longitude.thirds - ilond * 21600 - ilonm * 360) /    60.);

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
            return snprintf(buf, len, "%02d%04.1lf%c/%03d%04.1lf%c",
                            ilatd, dlatm, card[0], ilond, dlonm, card[1]);
        }

        case NDT_LLCFMT_BOING:
        {
            if (ilatm == 0 && ilonm == 0 && ilats < 6 && ilons < 6)
            {
                // 7-letter form, e.g. N46E050
                return snprintf(buf, len, "%c%02d%c%03d",
                                card[0], ilatd, card[1], ilond);
            }
            // full form, e.g. N4600.0E05000.0
            return snprintf(buf, len, "%c%02d%04.1lf%c%03d%04.1lf",
                            card[0], ilatd, dlatm, card[1], ilond, dlonm);
        }

        case NDT_LLCFMT_ICAOR:
        {
            if (ilatm == 0 && ilonm == 0 && ilats < 30 && ilons < 30)
            {
                // 7-letter form, e.g. 46N050E
                return snprintf(buf, len, "%02d%c%03d%c",
                                ilatd, card[0], ilond, card[1]);
            }
            // full form, e.g. 4600N05000E
            return snprintf(buf, len, "%02d%02d%c%03d%02d%c",
                            ilatd, (int)(dlatm + .5), card[0],
                            ilond, (int)(dlonm + .5), card[1]);
        }

        case NDT_LLCFMT_SBRIF:
        {
            if (ilatm == 0 && ilonm == 0 && ilats < 30 && ilons < 30)
            {
                // 5-letter form, e.g. 4650N
                if (ilond >= 100)
                {
                    return snprintf(buf, len, "%02d%c%02d",
                                    ilatd, card[2], ilond % 100);
                }
                else
                {
                    return snprintf(buf, len, "%02d%02d%c",
                                    ilatd, ilond, card[2]);
                }
            }
            // FlightAware-style, e.g. 4600N 05000E
            return snprintf(buf, len, "%02d%02d%c %03d%02d%c",
                            ilatd, (int)(dlatm + .5), card[0],
                            ilond, (int)(dlonm + .5), card[1]);
        }

        case NDT_LLCFMT_SVECT:
        {
            if (ilatm == 0 && ilonm == 0 && ilats == 0 && ilons == 0)
            {
                // 5-letter form, e.g. 4650N
                if (ilond >= 100)
                {
                    return snprintf(buf, len, "%02d%c%02d",
                                    ilatd, card[2], ilond % 100);
                }
                else
                {
                    return snprintf(buf, len, "%02d%02d%c",
                                    ilatd, ilond, card[2]);
                }
            }
            if (ilats == 0 && ilons == 0)
            {
                // 9-letter form, e.g. 4600N05000E
                return snprintf(buf, len, "%02d%02d%c%03d%02d%c",
                                ilatd, ilatm, card[0],
                                ilond, ilonm, card[1]);
            }
            // full form, e.g. 460000N0500000E
            return snprintf(buf, len, "%02d%02d%02d%c%03d%02d%02d%c",
                            ilatd, ilatm, ilats, card[0],
                            ilond, ilonm, ilons, card[1]);
        }

        case NDT_LLCFMT_DEFLT:
        default:
            break;
    }

    // default format: 7-letter form, e.g. N46E050
    return snprintf(buf, len, "%c%02d%c%03d",
                    card[0], (int)(dlatd + .5),
                    card[1], (int)(dlond + .5));
}

int ndt_position_fprintllc(ndt_position pos, ndt_llcfmt fmt, FILE *fd)
{
    char buf[21];
    int  ret = ndt_position_sprintllc(pos, fmt, buf, sizeof(buf));

    if (ret < 0)
    {
        return EIO;
    }
    if (ret >= sizeof(buf))
    {
        return ENOMEM;
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
