/*
 * common.h
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

#ifndef NDT_COMMON_H
#define NDT_COMMON_H

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>

/* Remainder of a floored division (sign follows the divisor) */
double ndt_mod(double y, double x);

typedef int (ndt_log_callback)(const char *format, va_list ap);
int  ndt_log    (              const char *format, ...);
int  ndt_fprintf(FILE *stream, const char *format, ...);
void ndt_log_set_callback  (ndt_log_callback *callback);

char* ndt_file_slurp  (const char *name, int *ret);
int   ndt_file_getline(      char **buf, int *cap, char **pos);
int   ndt_file_getpath(const char *base, const char *suffix, char **buf, int *cap);

typedef struct ndt_info
{
    char idnt [32]; // identifier (alphanumeric characters only)
    char misc[128]; // specific to each data structure
    char desc[512]; // text description
} ndt_info;

typedef struct ndt_airspeed
{
    enum
    {
        NDT_SPDUNIT_FPS,  // ft/s
        NDT_SPDUNIT_FPM,  // ft/m
        NDT_SPDUNIT_KPH,  // km/h
        NDT_SPDUNIT_KTS,  // knot
        NDT_SPDUNIT_MACH, // MACH (x1000)
        NDT_SPDUNIT_MPH,  // mile/h
        NDT_SPDUNIT_MTS,  // meter/s
        NDT_SPDUNIT_NAT,  // native representation
    } unit;

    int64_t value;        // unit: millimeters per hour
} ndt_airspeed;

/* TODO: true vs. indicated vs. calibrated airspeed */
ndt_airspeed ndt_airspeed_init(int64_t      speed, int unit                   );
ndt_airspeed ndt_airspeed_mach(double       oatmp                             );
int64_t      ndt_airspeed_get (ndt_airspeed speed, int unit, ndt_airspeed mach);

typedef struct ndt_distance
{
    enum
    {
        NDT_ALTUNIT_FL, // flight level
        NDT_ALTUNIT_FT, // foot
        NDT_ALTUNIT_ME, // meter
        NDT_ALTUNIT_NM, // nautical mile
        NDT_ALTUNIT_NA, // native representation
    } unit;

    int64_t value;      // unit: 0.10 millimeter ticks
} ndt_distance;

ndt_distance ndt_distance_init(int64_t      distance, int unit);
int64_t      ndt_distance_get (ndt_distance distance, int unit);

typedef enum ndt_angle_unit
{
    NDT_ANGUNIT_DEG,
    NDT_ANGUNIT_RAD,
} ndt_angle_unit;

typedef struct ndt_position
{
    struct
    {
        enum
        {
            NDT_LATITUDE_NORTH =  1,
            NDT_LATITUDE_SOUTH = -1,
        } equator;

        int value;
    } latitude;

    struct
    {
        enum
        {
            NDT_LONGITUDE_WEST =  1,
            NDT_LONGITUDE_EAST = -1,
        } meridian;

        int value;
    } longitude;

    ndt_distance altitude;     // altitude (relative to Mean Sea Level)
    ndt_distance precision[2]; // precision (0: lateral -- 1: vertical)
} ndt_position;

typedef enum
{
    NDT_LLCFMT_DEFLT,   // default: 7-letter, e.g. N46E050
    NDT_LLCFMT_AIBUS,   // Airbus CDU format, e.g. 4600.0N/05000.0E
    NDT_LLCFMT_BOING,   // Boeing CDU format, e.g. N4600.0E05000.0, N46E050
    NDT_LLCFMT_ICAOR,   // FAA ORDER JO 7110.10X, Appendix A. ICAO FLIGHT PLANS
    NDT_LLCFMT_SBRIF,   // optimized for SimBrief
    NDT_LLCFMT_SVECT,   // optimized for SkyVector
} ndt_llcfmt;

ndt_position ndt_position_init         (double        latitude, double         longitude,   ndt_distance altitude);
ndt_distance ndt_position_getaltitude  (ndt_position  position                                                   );
double       ndt_position_getlatitude  (ndt_position  position, ndt_angle_unit unit                              );
double       ndt_position_getlongitude (ndt_position  position, ndt_angle_unit unit                              );
void         ndt_position_getprecision (ndt_position  position, ndt_distance  *precision[2]                      );
void         ndt_position_setprecision (ndt_position *position, ndt_distance   precision[2]                      );
double       ndt_position_calcbearing  (ndt_position  from,     ndt_position   to                                );
ndt_distance ndt_position_calcdistance (ndt_position  from,     ndt_position   to                                );
int          ndt_position_calcduration (ndt_position  from,     ndt_position   to,          ndt_airspeed at      );
int          ndt_position_calcintercept(ndt_position  from,     ndt_position   to,          ndt_position orig    );
ndt_position ndt_position_calcpos4pbd  (ndt_position  from,     double trubearing,          ndt_distance dist    );
int          ndt_position_sprintllc    (ndt_position position,  ndt_llcfmt format, char *buffer,  size_t size    );
int          ndt_position_fprintllc    (ndt_position position,  ndt_llcfmt format, FILE *fd                      );

typedef struct ndt_frequency
{
    int value; // unit: 8.33 kHz ticks
} ndt_frequency;

ndt_frequency ndt_frequency_init(double        frequency);
double        ndt_frequency_get (ndt_frequency frequency);

typedef struct ndt_date
{
    int     year;
    uint8_t month;      // 1-12
    uint8_t day;        // 1-31

    uint8_t hours;      // 0-23
    uint8_t minutes;    // 0-59
    uint8_t seconds;    // 0-60
} ndt_date;

ndt_date ndt_date_init(time_t time);
time_t   ndt_date_get (ndt_date date);
ndt_date ndt_date_now (void);

#endif /* NDT_COMMON_H */
