/*
 * navdconv.c
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
#include <getopt.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

#include "common/common.h"
#include "compat/compat.h"
#include "lib/flightplan.h"
#include "lib/navdata.h"

// executable name and version
#ifndef NDCONV_EXE
#define NDCONV_EXE  "unnamed"
#endif
#ifndef NDT_VERSION
#define NDT_VERSION "unknown"
#endif

#define OPT_HELP 256
#define OPT_XMPL 257
#define OPT_VRSN 258
#define OPT_DTBS 259
#define OPT_XPLN 260
#define OPT_INPT 261
#define OPT_IFMT 262
#define OPT_OTPT 263
#define OPT_OFMT 264
#define OPT_DAPT 265
#define OPT_DRWY 266
#define OPT_DPRO 267
#define OPT_DTRA 268
#define OPT_AAPT 269
#define OPT_ARWY 270
#define OPT_APRO 271
#define OPT_ATRA 272
#define OPT_IRTE 273

static struct option navdconv_opts[] =
{
    { "h",             no_argument,       NULL, OPT_HELP, },
    { "help",          no_argument,       NULL, OPT_HELP, },
    { "x",             no_argument,       NULL, OPT_XMPL, },
    { "examples",      no_argument,       NULL, OPT_XMPL, },
    { "v",             no_argument,       NULL, OPT_VRSN, },
    { "version",       no_argument,       NULL, OPT_VRSN, },

    // navigation data
    { "db",            required_argument, NULL, OPT_DTBS, },
    { "xplane",        required_argument, NULL, OPT_XPLN, },

    // file input/output
    { "i",             required_argument, NULL, OPT_INPT, },
    { "ifmt",          required_argument, NULL, OPT_IFMT, },
    { "o",             required_argument, NULL, OPT_OTPT, },
    { "ofmt",          required_argument, NULL, OPT_OFMT, },

    // departure, arrival, route
    { "dep",           required_argument, NULL, OPT_DAPT, },
    { "dep-rwy",       required_argument, NULL, OPT_DRWY, },
    { "sid",           required_argument, NULL, OPT_DPRO, },
    { "sid-trans",     required_argument, NULL, OPT_DTRA, },
    { "arr",           required_argument, NULL, OPT_AAPT, },
    { "arr-rwy",       required_argument, NULL, OPT_ARWY, },
    { "star",          required_argument, NULL, OPT_APRO, },
    { "star-trans",    required_argument, NULL, OPT_ATRA, },
    { "rte",           required_argument, NULL, OPT_IRTE, },

    // that's all folks!
    { NULL,            0,                 NULL,  0,  },
};

// navigation data
static char *path_navdat = NULL;
static char *path_xplane = NULL;

// file input/output
static char *path_in     = NULL;
static int format_in     =   -1;
static char *path_out    = NULL;
static int format_out    =   -1;

// departure, arrival, route
static char *dep_apt     = NULL;
static char *dep_rwy     = NULL;
static char *sid_name    = NULL;
static char *sid_trans   = NULL;
static char *arr_apt     = NULL;
static char *arr_rwy     = NULL;
static char *star_name   = NULL;
static char *star_trans  = NULL;
static char *icao_route  = NULL;

static int execute_task    (void);
static int parse_options   (int argc, char **argv);
static int validate_options(void);
static int print_help      (void);
static int print_examples  (void);
static int print_version   (void);

int main(int argc, char **argv)
{
    int ret;

    if (argc <= 1)
    {
        ret = print_help();
        goto end;
    }

    if ((ret = parse_options(argc, argv)))
    {
        goto end;
    }

    if ((ret = validate_options()))
    {
        goto end;
    }

    ret = execute_task();

end:
    if (ret)
    {
        fprintf(stderr, "Error: %s\n", strerror(ret));
    }
    return ret;
}

static int execute_task(void)
{
    int              ret     = 0;
    ndt_navdatabase *navdata = NULL;
    ndt_flightplan  *fltplan = NULL;
    char            *flp_rte = NULL;
    FILE            *outfile = NULL;

    if (!(navdata = ndt_navdatabase_init(path_navdat, NDT_NAVDFMT_XPGNS)))
    {
        ret = EINVAL;
        goto end;
    }

    if (!(fltplan = ndt_flightplan_init()))
    {
        ret = ENOMEM;
        goto end;
    }

    if (dep_apt && (ret = ndt_flightplan_set_departure(fltplan, navdata, dep_apt, dep_rwy)))
    {
        goto end;
    }
    if (arr_apt && (ret = ndt_flightplan_set_arrival  (fltplan, navdata, arr_apt, arr_rwy)))
    {
        goto end;
    }

    if (path_in)
    {
        flp_rte = ndt_file_slurp(path_in, &ret);
        if (ret)
        {
            goto end;
        }
    }
    else
    {
        flp_rte   = icao_route;
        format_in = NDT_FLTPFMT_ICAOR;
    }

    if (flp_rte && (ret = ndt_flightplan_set_route(fltplan, navdata,
                                                   flp_rte, format_in)))
    {
        goto end;
    }

    if (path_out)
    {
        outfile = fopen(path_out, "w");
        if (!outfile)
        {
            ret = errno;
            goto end;
        }
    }
    else
    {
        outfile = stdout;
    }

    if ((ret = ndt_flightplan_write(fltplan, outfile, format_out)))
    {
        goto end;
    }

end:
    if (flp_rte != icao_route)
    {
        free(flp_rte);
    }
    if (outfile && outfile != stdout)
    {
        fclose(outfile);
    }
    ndt_navdatabase_close(&navdata);
    ndt_flightplan_close (&fltplan);
    return ret;
}

static int parse_options(int argc, char **argv)
{
    int opt;

    while ((opt = getopt_long_only(argc, argv, "", navdconv_opts, NULL)) >= 0)
    {
        switch (opt)
        {
            case 0:
                break;

            case OPT_HELP:
                exit(print_help());

            case OPT_XMPL:
                exit(print_examples());

            case OPT_VRSN:
                exit(print_version());

            case OPT_DTBS:
                path_navdat = strdup(optarg);
                break;

            case OPT_XPLN:
                path_xplane = strdup(optarg);
                break;

            case OPT_INPT:
                free(icao_route);
                icao_route = NULL;
                path_in    = strdup(optarg);
                break;

            case OPT_IFMT:
                if (!strcasecmp(optarg, "airbusx"))
                {
                    format_in = NDT_FLTPFMT_AIBXT;
                    break;
                }
                if (!strcasecmp(optarg, "icao"))
                {
                    format_in = NDT_FLTPFMT_ICAOR;
                    break;
                }
                if (!strcasecmp(optarg, "xplane"))
                {
                    format_in = NDT_FLTPFMT_XPFMS;
                    break;
                }
                fprintf(stderr, "Unsupported input format: '%s'\n", optarg);
                return EINVAL;

            case OPT_OTPT:
                path_out = strdup(optarg);
                break;

            case OPT_OFMT:
                if (!strcasecmp(optarg, "airbusx"))
                {
                    format_out = NDT_FLTPFMT_AIBXT;
                    break;
                }
                if (!strcasecmp(optarg, "icao"))
                {
                    format_out = NDT_FLTPFMT_ICAOR;
                    break;
                }
                if (!strcasecmp(optarg, "simbrief"))
                {
                    format_out = NDT_FLTPFMT_SBRIF;
                    break;
                }
                if (!strcasecmp(optarg, "skyvector"))
                {
                    format_out = NDT_FLTPFMT_SVECT;
                    break;
                }
                if (!strcasecmp(optarg, "xplane"))
                {
                    format_out = NDT_FLTPFMT_XPFMS;
                    break;
                }
                fprintf(stderr, "Unsupported output format: '%s'\n", optarg);
                return EINVAL;

            case OPT_DAPT:
                dep_apt = strdup(optarg);
                break;

            case OPT_DRWY:
                dep_rwy = strdup(optarg);
                break;

            case OPT_DPRO:
                sid_name = strdup(optarg);
                break;

            case OPT_DTRA:
                sid_trans = strdup(optarg);
                break;

            case OPT_AAPT:
                arr_apt = strdup(optarg);
                break;

            case OPT_ARWY:
                arr_rwy = strdup(optarg);
                break;

            case OPT_APRO:
                star_name = strdup(optarg);
                break;

            case OPT_ATRA:
                star_trans = strdup(optarg);
                break;

            case OPT_IRTE:
                free(path_in);
                path_in    = NULL;
                icao_route = strdup(optarg);
                break;

            default:
                return opt;
        }
    }

    return 0;
}

static int validate_options(void)
{
    char  *path         = NULL;
    int    pathlen, ret = 0;
    struct stat stats;
    char   error[64];

    if (path_xplane)
    {
        if (stat(path_xplane, &stats))
        {
            ret = errno;
        }
        else if (!S_ISDIR(stats.st_mode))
        {
            ret = ENOTDIR;
        }
        else if ((ret = ndt_file_getpath(path_xplane, "/Resources/GNS430/navdata", &path, &pathlen)))
        {
            // ret is set
        }
        else if (stat(path, &stats) || !S_ISDIR(stats.st_mode))
        {
            ret = EINVAL;
        }

        if (ret)
        {
            strerror_r(ret, error, sizeof(error));
            fprintf(stderr, "Invalid X-Plane directory: '%s' (%s)\n", path_xplane, error);
            goto end;
        }
        else if (!path_navdat)
        {
            if (!(path_navdat = strdup(path)))
            {
                ret = ENOMEM;
                goto end;
            }

            // check if updated Custom Data is available
            if ((ret = ndt_file_getpath(path_xplane, "/Custom Data/GNS430/navdata", &path, &pathlen)))
            {
                free(path);
                goto end;
            }
            else if (!stat(path, &stats) && S_ISDIR(stats.st_mode))
            {
                free(path_navdat);
                if ((path_navdat = strdup(path)) == NULL)
                {
                    ret = ENOMEM;
                    goto end;
                }
            }
        }
    }

    if (path_navdat)
    {
        if (stat(path_navdat, &stats))
        {
            ret = errno;
        }
        else if (!S_ISDIR(stats.st_mode))
        {
            ret = ENOTDIR;
        }
        else if ((ret = ndt_file_getpath(path_navdat, "/ATS.txt", &path, &pathlen)))
        {
            // ret is set
        }
        else if (stat(path, &stats) || !S_ISREG(stats.st_mode))
        {
            ret = EINVAL;
        }

        if (ret)
        {
            strerror_r(ret, error, sizeof(error));
            fprintf(stderr, "Invalid navdata directory: '%s' (%s)\n", path_navdat, error);
            goto end;
        }
    }
    else
    {
        fprintf(stderr, "No navdata directory provided\n");
        ret = EINVAL;
        goto end;
    }

    if (!path_in && !icao_route && !dep_apt && !arr_apt)
    {
        fprintf(stderr, "No input file or route provided\n");
        ret = EINVAL;
        goto end;
    }
    else if (path_in && access(path_in, R_OK))
    {
        strerror_r((ret = errno), error, sizeof(error));
        fprintf(stderr, "Bad input file: '%s' (%s)\n", path_in, error);
        goto end;
    }

    if (path_out && strnlen(path_out, 1))
    {
        char  *dir, *sep;
        size_t dirlen;

        if ((sep = strrchr(path_out, '/')) ||
            (sep = strrchr(path_out, '\\')))
        {
            dirlen = sep - path_out + 1;

            if (strnlen(path_out, dirlen + 1) <= dirlen)
            {
                strerror_r((ret = EISDIR), error, sizeof(error));
                fprintf(stderr, "Bad output file: '%s' (%s)\n", path_out, error);
                goto end;
            }

            if (!(dir = strndup(path_out, dirlen)))
            {
                ret = ENOMEM;
                goto end;
            }

            if (access(dir, W_OK))
            {
                strerror_r((ret = errno), error, sizeof(error));
                fprintf(stderr, "Bad output directory: '%s' (%s)\n", dir, error);
                free(dir);
                goto end;
            }
        }
    }
    else
    {
        free(path_out);
        path_out = NULL;
    }

    // default formats
    if (format_in  == -1)
    {
        format_in  = NDT_FLTPFMT_ICAOR;
    }
    if (format_out == -1)
    {
        format_out = NDT_FLTPFMT_XPFMS;
    }

end:
    free(path);
    return ret;
}

static int print_help(void)
{
    print_version();
    fprintf(stderr,
            "                                                                   \n"
            "### Documentation       -------------------------------------------\n"
            "  --help                This help ;)                               \n"
            "  --examples            A few examples to get started.             \n"
            "  --version             Version information.                       \n"
            "                                                                   \n"
            "### Navigation data     -------------------------------------------\n"
            "  --xplane     <string> Set the path to the root of a valid X-Plane\n"
            "                        installation (version 10.30 or later). Used\n"
            "                        to determine the location of the navdata.  \n"
            "  --db         <string> Set the path to the root of a valid navdata\n"
            "                        folder (in X-Plane 10.30 GNS430 format).   \n"
            "                        Required if the path to X-Plane is not set.\n"
            "                                                                   \n"
            "### Input and output    -------------------------------------------\n"
            "  -i           <string> Path to the file containing the route for  \n"
            "                        the flight plan. Required unless the full  \n"
            "                        route is specified via individual options. \n"
            "  --ifmt       <string> Input file format:                         \n"
            "                            airbusx     Airbus X Extended CoRte    \n"
            "                            icao        ICAO flight plan route     \n"
            "                            xplane      X-Plane .fms flight plan   \n"
            "                                        with QPAC enhancements for \n"
            "                                        altitude constraints and   \n"
            "                                        fly-over waypoints         \n"
            "                        Default: icao                              \n"
            "                                                                   \n"
            "  -o           <string> Path to the file where the flight plan will\n"
            "                        be written to. The parent directory must   \n"
            "                        already exist and be writable. The file has\n"
            "                        to be writable too (if it already exists). \n"
            "                        If omitted, output will be written to the  \n"
            "                        standard output stream (stdout) instead.   \n"
            "  --ofmt       <string> Output format:                             \n"
            "                            airbusx     Airbus X Extended CoRte    \n"
            "                            icao        ICAO flight plan route,    \n"
            "                                        without specific SID/STAR  \n"
            "                            skyvector   Decoded route, all legs on \n"
            "                                        a single line              \n"
            "                            xplane      X-Plane .fms flight plan   \n"
            "                                        with QPAC enhancements for \n"
            "                                        altitude constraints and   \n"
            "                                        fly-over waypoints         \n"
            "                        Default: xplane                            \n"
            "                                                                   \n"
            "### Flight planning     -------------------------------------------\n"
            "  --dep        <string> Set departure airport (4-letter ICAO code).\n"
            "                        May be omitted if the departure is present \n"
            "                        in the route as well.                      \n"
            "                                                                   \n"
#if 0
            "  --dep-rwy    <string> Set departure runway. Can be omitted unless\n"
            "                        a SID procedure is also specified.         \n"
            "  --sid        <string> Select a SID procedure (not implemented).  \n"
            "  --sid-trans  <string> Set the SID transition (not implemented).  \n"
            "                                                                   \n"
#endif
            "  --arr        <string> Set arrival airport (4-letter ICAO code).  \n"
            "                        May be omitted if the arrival is present   \n"
            "                        in the route as well.                      \n"
            "                                                                   \n"
#if 0
            "  --star       <string> Select a STAR procedure (not implemented). \n"
            "  --star-trans <string> Set the STAR transition (not implemented). \n"
            "  --arr-rwy    <string> Set arrival runway. Can be omitted.        \n"
            "                                                                   \n"
#endif
            "  --rte        <string> Route in ICAO flight plan format. Should   \n"
            "                        include the departure and arrival airports,\n"
            "                        if not set via the --dep and --arr options.\n");
    return 0;
}

static int print_examples(void)
{
    print_version();
    fprintf(stderr,
            "                                                                   \n"
            "### Documentation -------------------------------------------------\n"
            "  --help                Built-in help text.                        \n"
            "  --examples            You're reading it ;)                       \n"
            "  --version             Version information.                       \n"
            "                                                                   \n"
            "### Example commands ----------------------------------------------\n"
            "                                                                   \n"
            "Note: it is assumed that the user knows how to type file paths in  \n"
            "his Terminal or Command Prompt. $XPLANE is the path to the root    \n"
            "folder of the X-Plane installation, containing the X-Plane and     \n"
            "X-Plane 32-bit applications.                                       \n"
            "                                                                   \n"
            " Create a flight plan in the X-Plane FMS format, between LSGG and  \n"
            " LSZH, with the route \"MOLUS UN871 BERSU\", and write it to the   \n"
            " file \"LSGG-LSZH-01.fms\" in the current directory:               \n"
            "                                                                   \n"
            "     %s --xplane $XPLANE --dep LSGG --arr LSZH --rte \"MOLUS UN871 BERSU\" -o LSGG-LSZH-01.fms\n"
            "                                                                   \n"
            " Convert an ICAO route with airways to a list of waypoints for use \n"
            " on e.g. SkyVector, and write it to the terminal:                  \n"
            "                                                                   \n"
            "     %s --xplane $XPLANE --rte \"LSGG MOLUS UN871 BERSU LSZH\" --ofmt skyvector\n"
            "                                                                   \n"
            " Sanitize an Airbus X Extended company route, converting it from   \n"
            " the old format (DCT as airway) to the new (DCT with coordinates): \n"
            "     %s --xplane $XPLANE --ifmt airbusx --ofmt airbusx -i <input> -o <output>\n"
            "                                                                   \n"
            "### Example formats -----------------------------------------------\n"
            "                                                                   \n"
            " \"icao\" (input and output):                                      \n"
            "     LSGG SID MOLUS UN871 BERSU STAR LSZH                          \n"
            "     EIDW SID SUROX 56N020W 56N030W 55N040W 53N050W YAY J580 YQY SCUPP STAR KBOS\n"
            "                                                                   \n"
            " \"simbrief\" (output only):                                       \n"
            "     MOLUS UN871 BERSU                                             \n"
            "     SUROX 5620N 5630N 5540N 5350N YAY J580 YQY SCUPP              \n"
            "                                                                   \n"
            " \"skyvector\" (output only):                                      \n"
            "     LSGG MOLUS SOSAL TELNO KORED KONOL BERSU LSZH                 \n"
            "     EIDW SUROX +56.000000/-020.000000 +56.000000/-030.000000 +55.000000/-040.000000 +53.000000/-050.000000 YAY YJT YQY SCUPP KBOS\n"
            "                                                                   \n"
            " \"airbusx\" (input and output):                                   \n"
            "     [CoRte]                                                       \n"
            "     ArptDep=LSGG                                                  \n"
            "     ArptArr=LSZH                                                  \n"
            "     DctWpt1=MOLUS                                                 \n"
            "     DctWpt1Coordinates=46.443889,6.679444                         \n"
            "     Airway2=UN871                                                 \n"
            "     Airway2FROM=MOLUS                                             \n"
            "     Airway2TO=BERSU                                               \n"
            "     ---------------------------------------                       \n"
            "     [CoRte]                                                       \n"
            "     ArptDep=EIDW                                                  \n"
            "     ArptArr=KBOS                                                  \n"
            "     DctWpt1=SUROX                                                 \n"
            "     DctWpt1Coordinates=53.996667,-6.993333                        \n"
            "     DctWpt2=5620N                                                 \n"
            "     DctWpt2Coordinates=56.000000,-20.000000                       \n"
            "     DctWpt3=5630N                                                 \n"
            "     DctWpt3Coordinates=56.000000,-30.000000                       \n"
            "     DctWpt4=5540N                                                 \n"
            "     DctWpt4Coordinates=55.000000,-40.000000                       \n"
            "     DctWpt5=5350N                                                 \n"
            "     DctWpt5Coordinates=53.000000,-50.000000                       \n"
            "     DctWpt6=YAY                                                   \n"
            "     DctWpt6Coordinates=51.393889,-56.083611                       \n"
            "     Airway7=J580                                                  \n"
            "     Airway7FROM=YAY                                               \n"
            "     Airway7TO=YQY                                                 \n"
            "     DctWpt8=SCUPP                                                 \n"
            "     DctWpt8Coordinates=42.603056,-70.230278                       \n"
            "                                                                   \n"
            " \"xplane\" (input and output):                                    \n"
            "     I                                                             \n"
            "     3 version                                                     \n"
            "     1                                                             \n"
            "     7                                                             \n"
            "     1 LSGG 1411 46.238056 6.109167                                \n"
            "     11 MOLUS 0 46.443889 6.679444                                 \n"
            "     11 SOSAL 0 46.558056 6.884444                                 \n"
            "     11 TELNO 0 46.771944 7.270556                                 \n"
            "     11 KORED 0 46.850556 7.414167                                 \n"
            "     11 KONOL 0 46.995278 7.680556                                 \n"
            "     11 BERSU 0 47.135278 7.941111                                 \n"
            "     1 LSZH 1417 47.458056 8.548056                                \n"
            "     0 ---- 0 0.000000 0.000000                                    \n"
            "     ---------------------------------------                       \n"
            "     I                                                             \n"
            "     3 version                                                     \n"
            "     1                                                             \n"
            "     10                                                            \n"
            "     1 EIDW 242 53.421389 -6.270000                                \n"
            "     11 SUROX 0 53.996667 -6.993333                                \n"
            "     28 +56.00_-020.00 0 56.000000 -20.000000                      \n"
            "     28 +56.00_-050.00 0 56.000000 -30.000000                      \n"
            "     28 +55.00_-040.00 0 55.000000 -40.000000                      \n"
            "     28 +53.00_-050.00 0 53.000000 -50.000000                      \n"
            "     3 YAY 0 51.393889 -56.083611                                  \n"
            "     3 YJT 0 48.582500 -58.669167                                  \n"
            "     3 YQY 0 46.153333 -60.055556                                  \n"
            "     11 SCUPP 0 42.603056 -70.230278                               \n"
            "     1 KBOS 19 42.362778 -71.006389                                \n"
            "     0 ---- 0 0.000000 0.000000                                    \n",
            NDCONV_EXE, NDCONV_EXE, NDCONV_EXE);
    return 0;
}

static int print_version(void)
{
    fprintf(stderr,
            "%s version %s, Copyright (c) 2014-2015 Timothy D. Walker           \n"
            "                                                                   \n"
            "Syntax: %s [options] -i <file> [-o <file>]                         \n",
            NDCONV_EXE, NDT_VERSION, NDCONV_EXE);
    return 0;
}
