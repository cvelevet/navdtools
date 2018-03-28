/*
 * navdconv.c
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

#include <ctype.h>
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
#include "lib/fmt_icaor.h"
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
#define OPT_DSID 267
#define OPT_DSTR 268
#define OPT_AAPT 269
#define OPT_ARWY 270
#define OPT_ASTA 271
#define OPT_ASTR 272
#define OPT_ATRS 273
#define OPT_AFIN 274
#define OPT_IRTE 275
#define OPT_ANFO 276
#define OPT_QPAC 277
#define OPT_MTRC 278

// navigation data
static char *info_aptidt = NULL;
static char *path_navdat = NULL;
static char *path_xplane = NULL;
static char *qpac_aptids = NULL;
static int rwu = NDT_ALTUNIT_FT;

// file input/output
static char *path_in     = NULL;
static int format_in     =   -1;
static char *path_out    = NULL;
static int format_out    =   -1;
static int fprintairac   =    0;

// departure, arrival, route
static char *dep_apt     = NULL;
static char *dep_rwy     = NULL;
static char *sid_name    = NULL;
static char *sid_trans   = NULL;
static char *arr_apt     = NULL;
static char *arr_rwy     = NULL;
static char *star_name   = NULL;
static char *star_trans  = NULL;
static char *appr_trans  = NULL;
static char *final_appr  = NULL;
static char *icao_route  = NULL;

static struct option navdconv_opts[] =
{
    { "h",             no_argument,       NULL, OPT_HELP, },
    { "help",          no_argument,       NULL, OPT_HELP, },
    { "x",             no_argument,       NULL, OPT_XMPL, },
    { "examples",      no_argument,       NULL, OPT_XMPL, },
    { "v",             no_argument,       NULL, OPT_VRSN, },
    { "version",       no_argument,       NULL, OPT_VRSN, },
    { "metric",        no_argument,       NULL, OPT_MTRC, },

    // navigation data
    { "db",            required_argument, NULL, OPT_DTBS, },
    { "xplane",        required_argument, NULL, OPT_XPLN, },
    { "info",          required_argument, NULL, OPT_ANFO, },
    { "qpac",          required_argument, NULL, OPT_QPAC, },

    // file input/output
    { "i",             required_argument, NULL, OPT_INPT, },
    { "ifmt",          required_argument, NULL, OPT_IFMT, },
    { "o",             required_argument, NULL, OPT_OTPT, },
    { "ofmt",          required_argument, NULL, OPT_OFMT, },
    { "airac",         no_argument,      &fprintairac, 1, },

    // departure, arrival, route
    { "dep",           required_argument, NULL, OPT_DAPT, },
    { "drwy",          required_argument, NULL, OPT_DRWY, },
    { "dep-rwy",       required_argument, NULL, OPT_DRWY, },
    { "sid",           required_argument, NULL, OPT_DSID, },
    { "sidtr",         required_argument, NULL, OPT_DSTR, },
    { "arr",           required_argument, NULL, OPT_AAPT, },
    { "arwy",          required_argument, NULL, OPT_ARWY, },
    { "arr-rwy",       required_argument, NULL, OPT_ARWY, },
    { "star",          required_argument, NULL, OPT_ASTA, },
    { "startr",        required_argument, NULL, OPT_ASTR, },
    { "apptr",         required_argument, NULL, OPT_ATRS, },
    { "final",         required_argument, NULL, OPT_AFIN, },
    { "rte",           required_argument, NULL, OPT_IRTE, },

    // that's all folks!
    { NULL,            0,                 NULL,        0, },
};

static int sidstar_task    (void);
static int execute_task    (void);
static int parse_options   (int argc, char **argv);
static int validate_options(void);
static int print_airportnfo(void);
static int print_airac     (ndt_navdatabase  *ndb);
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

    if (info_aptidt)
    {
        ret = print_airportnfo();
        goto end;
    }
    if (qpac_aptids)
    {
        ret = sidstar_task();
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

static int print_airportnfo(void)
{
    ndt_navdatabase *navdata = ndt_navdatabase_init(path_navdat, NDT_NAVDFMT_XPGNS);
    if (!navdata)
    {
        return EINVAL;
    }
    return ndt_fmt_icaor_print_airportnfo(navdata, info_aptidt, rwu);
}

static int sidstar_procedure(ndt_navdatabase *ndb, int type,
                             ndt_procedure  *proc, ndt_procedure *trans)
{
    ndt_flightplan *flp = NULL;
    FILE            *fd = NULL;
    char          *path = NULL;
    char        *outdir = NULL;
    int         pathlen = 0;
    int             ret = 0;

    /*
     * Initialize empty flight plan.
     */
    if (!proc || !(flp = ndt_flightplan_init(ndb)))
    {
        ret = ENOMEM;
        goto end;
    }

    /*
     * Setup the requested procedure (1: SID, 2: STAR, 3: approach).
     * We set the same departure and arrival airport; combined with the lack of
     * enroute legs and the presence of a SID/STAR/approach procedure, we tell
     * the XPFMS flightplan writer to skip writing the departure airport for
     * SID procedures, resp. the arrival airport for STAR/approach procedures.
     */
    switch (type)
    {
        case 1:
            if ((ret = ndt_flightplan_set_departure(flp, dep_apt, dep_rwy)) ||
                (ret = ndt_flightplan_set_arrival  (flp, dep_apt, NULL)))
            {
                goto end;
            }
            ret = ndt_flightplan_set_departsid(flp, proc->info.idnt, trans ? trans->info.misc : NULL);
            break;
        case 2:
            if ((ret = ndt_flightplan_set_departure(flp, arr_apt, NULL)) ||
                (ret = ndt_flightplan_set_arrival  (flp, arr_apt, arr_rwy)))
            {
                goto end;
            }
            ret = ndt_flightplan_set_arrivstar(flp, proc->info.idnt, trans ? trans->info.misc : NULL);
            break;
        case 3:
            if ((ret = ndt_flightplan_set_departure(flp, arr_apt, NULL)) ||
                (ret = ndt_flightplan_set_arrival  (flp, arr_apt, arr_rwy)))
            {
                goto end;
            }
            ret = ndt_flightplan_set_arrivapch(flp, proc->info.idnt, trans ? trans->info.misc : NULL);
            break;
        default:
            fprintf(stderr, "Unknown procedure generic type '%d' for %s.%s\n",
                    type, proc->info.idnt, trans ? trans->info.idnt : "");
            ret  =  EINVAL;
            goto end;
    }
    if (ret)
    {
        goto end;
    }

    /*
     * Determine an appropriate output file name.
     * Note: to sort near the top,    use ' '
     *       to sort near the bottom, use '_' instead
     *       letters sort nearer the bottom than both of the above.
     */
    const char *transition;
    char filename[6+7+9+6+4+1];// "/STAR_" "ICAO06L" "_VORDME-Y" ".WAYPT" ".fms" "\n"
    ndt_route_leg *fleg = ndt_list_item(flp->legs,  0);
    ndt_route_leg *lleg = ndt_list_item(flp->legs, -1);
    if (fleg && fleg->type == NDT_LEGTYPE_ZZ)
    {
        fleg = ndt_list_item(flp->legs, 1);
    }
    if (!fleg || !lleg)
    {
        fprintf(stderr, "Flightplan is empty! %d: %s.%s\n",
                type, proc->info.idnt, trans ? trans->info.idnt : "");
        ret  =  EINVAL;
        goto end;
    }
    switch (type)
    {
        case 1:
            transition = lleg->dst ? lleg->dst->info.idnt : NULL;
            transition = trans     ? trans->    info.misc : transition;
            snprintf(filename, sizeof(filename), "/SID_%s%s %s%s%s.fms",
                     dep_apt, dep_rwy, proc->info.idnt,
                     transition == NULL ? "" : ".",
                     transition == NULL ? "" : transition);
            break;
        case 2:
            transition = fleg->dst ? fleg->dst->info.idnt : NULL;
            transition = trans     ? trans->    info.misc : transition;
            snprintf(filename, sizeof(filename), "/STAR_%s%s %s%s%s.fms",
                     arr_apt, arr_rwy, proc->info.idnt,
                     transition == NULL ? "" : ".",
                     transition == NULL ? "" : transition);
            break;
        case 3:
            transition = fleg->dst ? fleg->dst->info.idnt : NULL;
            transition = trans     ? trans->    info.misc : transition;
            snprintf(filename, sizeof(filename), "/STAR_%s%s_%s%s%s.fms",
                     arr_apt, arr_rwy, proc->approach.short_name,
                     transition == NULL ? "" : ".",
                     transition == NULL ? "" : transition);
            break;
        default:
            fprintf(stderr, "Unknown procedure generic type '%d' for %s.%s\n",
                    type, proc->info.idnt, trans ? trans->info.idnt : "");
            ret  =  EINVAL;
            goto end;
    }

    /*
     * If a subdirectory using the airport's ICAO code exists,
     * use said subfolder instead of the output folder's root.
     *
     * Also check for a directory named QPAC :)
     */
    struct stat stats;
    char subdir[1+4+1];// "/" "ICAO" "\n"
    snprintf(subdir, sizeof(subdir), "/%4s", dep_apt);
    if (!ndt_file_getpath(path_out, subdir, &path, &pathlen) &&
        !stat(path, &stats) && !!S_ISDIR(stats.st_mode) && !access(path, W_OK))
    {
        outdir = strdup(path);
    }
    else if (!ndt_file_getpath(path_out, "/QPAC", &path, &pathlen) &&
             !stat(path, &stats) && !!S_ISDIR(stats.st_mode) && !access(path, W_OK))
    {
        outdir = strdup(path);
    }
    else
    {
        outdir = strdup(path_out);
    }
    if ((ret = ndt_file_getpath(outdir, filename, &path, &pathlen)))
    {
        goto end;
    }

    /*
     * Finally, let's write the (partial) flightplan to a file.
     */
    if  (!(fd = fopen(path, "w")))
    {
        ret = errno;
        goto end;
    }
    if ((ret = ndt_flightplan_write(flp, fd, NDT_FLTPFMT_XPFMS)))
    {
        goto end;
    }

end:
    if (flp)
    {
        ndt_flightplan_close(&flp);
    }
    if (fd)
    {
        fclose(fd);
    }
    if (ret)
    {
        if (type == 1)
        {
            ndt_log("failed to create file for %s%s%s: %s%s%s (%s)\n", dep_apt,
                    dep_rwy == NULL     ? "" : "/",
                    dep_rwy == NULL     ? "" : dep_rwy,
                    proc  ? proc-> info.idnt : NULL,
                    trans   == NULL     ? "" : ".",
                    trans   == NULL     ? "" : trans->info.misc, strerror(ret));
        }
        else
        {
            ndt_log("failed to create file for %s%s%s: %s%s%s (%s)\n", arr_apt,
                    arr_rwy == NULL     ? "" : "/",
                    arr_rwy == NULL     ? "" : arr_rwy,
                    proc  ? proc-> info.idnt : NULL,
                    trans   == NULL     ? "" : ".",
                    trans   == NULL     ? "" : trans->info.misc, strerror(ret));
        }
    }
    free(outdir);
    free  (path);
    return     0;
}

static int sidstar_task(void)
{
    ndt_navdatabase   *navdata = NULL;
    int                   rval = 0;
    ndt_runway           *rnwy;
    ndt_route_leg        *rleg;
    ndt_airport   *apt1, *apt2;
    ndt_procedure *proc, *tran;

    /*
     * Free some global variables we'll be re-using shortly.
     */
    free(dep_rwy);
    free(arr_rwy);

    /*
     * Initialize navigation data and airport procedures.
     */
    if (!(navdata = ndt_navdatabase_init(path_navdat, NDT_NAVDFMT_XPGNS)))
    {
        rval = EINVAL;
        goto end;
    }
    if (!(apt1 = ndt_navdata_get_airport(navdata, dep_apt)))
    {
        fprintf(stderr, "Airport %s not found\n", dep_apt);
        rval = EINVAL;
        goto end;
    }
    if (!(apt2 = ndt_navdata_get_airport(navdata, arr_apt)))
    {
        fprintf(stderr, "Airport %s not found\n", arr_apt);
        rval = EINVAL;
        goto end;
    }
    if (!(ndt_navdata_init_airport(navdata, apt1)) ||
        !(ndt_navdata_init_airport(navdata, apt2)))
    {
        rval = EINVAL;
        goto end;
    }

    /*
     * First airport, all departures:
     * - one file per runway for each SID procedure;
     * - one file per runway for each transition (prepending its parent SID).
     */
    for (size_t i = 0; i < ndt_list_count(apt1->runways); i++)
    {
        if (!(rnwy = ndt_list_item(apt1->runways, i)))
        {
            rval = ENOMEM;
            goto end;
        }
        else
        {
            dep_rwy = rnwy->info.idnt;
            arr_rwy = NULL;
        }
        for (size_t j = 0; j < ndt_list_count(rnwy->sids); j++)
        {
            if (!(proc = ndt_list_item(rnwy->sids, j)))
            {
                rval = ENOMEM;
                goto end;
            }
            if ((rval = sidstar_procedure(navdata, 1, proc, NULL)))
            {
                goto end;
            }
            if (!proc->transition.enroute)
            {
                continue;
            }
            for (size_t k = 0; k < ndt_list_count(proc->transition.enroute); k++)
            {
                if (!(tran = ndt_list_item(proc->transition.enroute, k)))
                {
                    rval = ENOMEM;
                    goto end;
                }
                if ((rval = sidstar_procedure(navdata, 1, proc, tran)))
                {
                    goto end;
                }
            }
        }
    }

    /*
     * Second airport, all departures:
     * - one file per runway for each STAR procedure;
     * - one file per runway for each transition (appending its parent STAR);
     * - one file per runway for each final approach procedure;
     * - one file per runway for each transition (appending the final approach).
     */
    for (size_t i = 0; i < ndt_list_count(apt2->runways); i++)
    {
        if (!(rnwy = ndt_list_item(apt2->runways, i)))
        {
            rval = ENOMEM;
            goto end;
        }
        else
        {
            dep_rwy = NULL;
            arr_rwy = rnwy->info.idnt;
        }
        for (size_t j = 0; j < ndt_list_count(rnwy->stars); j++)
        {
            if (!(proc = ndt_list_item(rnwy->stars, j)))
            {
                rval = ENOMEM;
                goto end;
            }
            if ((rval = sidstar_procedure(navdata, 2, proc, NULL)))
            {
                goto end;
            }
            if (!proc->transition.enroute)
            {
                continue;
            }
            for (size_t k = 0; k < ndt_list_count(proc->transition.enroute); k++)
            {
                if (!(tran = ndt_list_item(proc->transition.enroute, k)))
                {
                    rval = ENOMEM;
                    goto end;
                }
                if ((rval = sidstar_procedure(navdata, 2, proc, tran)))
                {
                    goto end;
                }
            }
        }
        for (size_t j = 0; j < ndt_list_count(rnwy->approaches); j++)
        {
            if (!(proc = ndt_list_item(rnwy->approaches, j)))
            {
                rval = ENOMEM;
                goto end;
            }
            if ((rval = sidstar_procedure(navdata, 3, proc, NULL)))
            {
                goto end;
            }
            if (!proc->transition.approach)
            {
                continue;
            }
            for (size_t k = 0; k < ndt_list_count(proc->transition.approach); k++)
            {
                if (!(tran = ndt_list_item(proc->transition.approach, k)))
                {
                    rval = ENOMEM;
                    goto end;
                }
                if ((rval = sidstar_procedure(navdata, 3, proc, tran)))
                {
                    goto end;
                }
            }
        }
    }

end:
    if (navdata)
    {
        ndt_navdatabase_close(&navdata);
    }
    dep_rwy = NULL;
    arr_rwy = NULL;
    return    rval;
}

static int execute_task(void)
{
    int                  ret = 0;
    ndt_navdatabase *navdata = NULL;
    ndt_flightplan  *fltplan = NULL;
    char            *flp_rte = NULL;
    FILE            *outfile = NULL;

    if (!(navdata = ndt_navdatabase_init(path_navdat, NDT_NAVDFMT_XPGNS)))
    {
        ret = EINVAL;
        goto end;
    }

    if (fprintairac)
    {
        print_airac(navdata);
    }

    if (!(fltplan = ndt_flightplan_init(navdata)))
    {
        ret = ENOMEM;
        goto end;
    }

    // departure airport/runway, SID and arrival airport/runway must
    // be set first for sequencing and filtering of duplicate waypoints
    if (dep_apt)
    {
        if ((ret = ndt_flightplan_set_departure(fltplan, dep_apt, dep_rwy)))
        {
            goto end;
        }
        if (sid_name && (ret = ndt_flightplan_set_departsid(fltplan, sid_name, sid_trans)))
        {
            goto end;
        }
    }
    if (arr_apt)
    {
        if ((ret = ndt_flightplan_set_arrival(fltplan, arr_apt, arr_rwy)))
        {
            goto end;
        }
    }

    if (path_in && !icao_route)
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

    // we can set the flight route now
    if (flp_rte && (ret = ndt_flightplan_set_route(fltplan, flp_rte, format_in)))
    {
        goto end;
    }

    // STAR and approach must be set after the flight route for proper sequencing
    if (star_name)
    {
        if (fltplan->arr.star.proc) // STAR might be present in the flight route
        {
            if (star_trans)
            {
                fprintf(stderr, "warning: ignoring STAR '%s.%s'\n", star_name, star_trans);
            }
            else
            {
                fprintf(stderr, "warning: ignoring STAR '%s'\n", star_name);
            }
        }
        else if ((ret = ndt_flightplan_set_arrivstar(fltplan, star_name, star_trans)))
        {
            goto end;
        }
    }
    if (fltplan->arr.rwy && appr_trans && !strcasecmp(appr_trans, "auto"))
    {
        ndt_route_leg *leg = ndt_list_item(fltplan->legs, -1);
        ndt_waypoint  *src = leg ? leg->dst : NULL;
        free(appr_trans); appr_trans = NULL; // reset
        if (src)
        {
            ndt_procedure *final = ndt_procedure_get(fltplan->arr.rwy->approaches, final_appr, NULL);
            if (final)
            {
                for (size_t i = 0; i < ndt_list_count(final->transition.approach); i++)
                {
                    ndt_procedure *apptr = ndt_list_item(final->transition.approach, i);
                    if (apptr)
                    {
                        ndt_route_leg *leg = ndt_list_item(apptr->proclegs, 0);
                        if (leg)
                        {
                            if (leg->type == NDT_LEGTYPE_IF && leg->dst == src)
                            {
                                appr_trans = strdup(apptr->info.misc);
                                break;
                            }
                            if (leg->src == src)
                            {
                                appr_trans = strdup(apptr->info.misc);
                                break;
                            }
                        }
                        for (size_t j = i; j < ndt_list_count(apptr->proclegs); j++)
                        {
                            leg = ndt_list_item(apptr->proclegs, j);
                            if (leg && leg->type == NDT_LEGTYPE_IF && leg->dst == src)
                            {
                                appr_trans = strdup(apptr->info.misc);
                                break;
                            }
                        }
                        if (appr_trans)
                        {
                            break;
                        }
                    }
                }
            }
        }
        if (!appr_trans)
        {
            fprintf(stderr, "warning: no valid approach transition found\n");
        }
    }
    if ((final_appr && arr_rwy) &&
        (ret = ndt_flightplan_set_arrivapch(fltplan, final_appr, appr_trans)))
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

/* See NOTE in parse_options(). */
static void string_split4(char *arg, const char *delim, char **fields[6])
{
    if (!arg || !delim || !fields)
    {
        return;
    }
    char *dup = strdup(arg);
    char *buf = dup;
    if  (!buf)
    {
        return;
    }
    for (int i = 0; i < 6; i++)
    {
        char *elem = strsep(&buf, delim);
        if  (!elem)
        {
            break;
        }
        if (strnlen(elem, 1) && fields[i])
        {
            char **ptr = fields[i];
            char *prev = *ptr;
            if   (prev)
            {
                free(prev);
            }
            if (strnlen(elem, 5) <= 4 && !strncasecmp("none", elem, strlen(elem)))
            {
                *ptr = NULL;
            }
            else
            {
                *ptr = strdup(elem);
            }
        }
    }
    free(dup);
    return;
}

static int parse_options(int argc, char **argv)
{
    int     opt;
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

            case OPT_MTRC:
                rwu = NDT_ALTUNIT_ME;
                break;

            case OPT_ANFO:
                free(info_aptidt);
                info_aptidt = strdup(optarg);
                break;

            case OPT_DTBS:
                free(path_navdat);
                free(path_xplane);
                path_xplane = NULL;
                path_navdat = strdup(optarg);
                break;

            case OPT_XPLN:
                free(path_xplane);
                free(path_navdat);
                path_navdat = NULL;
                path_xplane = strdup(optarg);
                break;

            case OPT_QPAC:
                free(qpac_aptids);
                qpac_aptids = strdup(optarg);
                break;

            case OPT_INPT:
                free(path_in);
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
                free(path_out);
                if (!strcasecmp(optarg, "stdout"))
                    path_out = NULL;
                else
                    path_out = strdup(optarg);
                break;

            case OPT_OFMT:
                if (!strcasecmp(optarg, "airbusx"))
                {
                    format_out = NDT_FLTPFMT_AIBXT;
                    break;
                }
                if (!strcasecmp(optarg, "civa"))
                {
                    format_out = NDT_FLTPFMT_XPCVA;
                    break;
                }
                if (!strcasecmp(optarg, "decoded") ||
                    !strcasecmp(optarg, "skyvector")) // legacy name for flat, decoded route
                {
                    format_out = NDT_FLTPFMT_DCDED;
                    break;
                }
                if (!strcasecmp(optarg, "test"))
                {
                    format_out = NDT_FLTPFMT_DTEST; // purposefully undocumented
                    break;
                }
                if (!strcasecmp(optarg, "helper"))
                {
                    format_out = NDT_FLTPFMT_XPHLP;
                    break;
                }
                if (!strcasecmp(optarg, "icao"))
                {
                    format_out = NDT_FLTPFMT_ICAOR;
                    break;
                }
                if (!strcasecmp(optarg, "ixeg"))
                {
                    format_out = NDT_FLTPFMT_ICAOX;
                    break;
                }
                if (!strcasecmp(optarg, "mcdu"))
                {
                    format_out = NDT_FLTPFMT_XPCDU;
                    break;
                }
                if (!strcasecmp(optarg, "recap"))
                {
                    format_out = NDT_FLTPFMT_IRECP;
                    break;
                }
                if (!strcasecmp(optarg, "simbrief"))
                {
                    format_out = NDT_FLTPFMT_SBRIF;
                    break;
                }
                if (!strcasecmp(optarg, "xplane"))
                {
                    format_out = NDT_FLTPFMT_XPFMS;
                    break;
                }
                fprintf(stderr, "Unsupported output format: '%s'\n", optarg);
                return EINVAL;

            /*
             * NOTE: undocumented feature.
             *
             * Basically, --dep and --arr accept more than one argument,
             * separated by either of '/' or '.' -- so you can go with:
             *
             * --dep icao/runway/sid.sid_trans
             * --arr icao/runway/final/appr_trans/star.star_trans
             *
             * Trailing items can be omitted, whereas intermediate items can be
             * NULL-ified simply by specifying any of "n", "no", "non", "none".
             */
            case OPT_DAPT:
                {
                    char **fields[6] = { &dep_apt, &dep_rwy, &sid_name, &sid_trans, NULL, NULL, };
                    string_split4(optarg, "/.", fields);
                }
                break;

            case OPT_DRWY:
                {
                    char **fields[6] = { &dep_rwy, &sid_name, &sid_trans, NULL, NULL, NULL, };
                    string_split4(optarg, "/.", fields);
                }
                break;

            case OPT_DSID:
                {
                    char **fields[6] = { &sid_name, &sid_trans, NULL, NULL, NULL, NULL, };
                    string_split4(optarg, "/.", fields);
                }
                break;

            case OPT_DSTR:
                {
                    char **fields[6] = { &sid_trans, NULL, NULL, NULL, NULL, NULL, };
                    string_split4(optarg, "/.", fields);
                }
                break;

            case OPT_AAPT:
                {
                    char **fields[6] = { &arr_apt, &arr_rwy, &final_appr, &appr_trans, &star_name, &star_trans, };
                    string_split4(optarg, "/.", fields);
                }
                break;

            case OPT_ARWY:
                {
                    char **fields[6] = { &arr_rwy, &final_appr, &appr_trans, &star_name, &star_trans, NULL, };
                    string_split4(optarg, "/.", fields);
                }
                break;

            case OPT_ASTA:
                {
                    char **fields[6] = { &star_name, &star_trans, NULL, NULL, NULL, NULL, };
                    string_split4(optarg, "/.", fields);
                }
                break;

            case OPT_ASTR:
                {
                    char **fields[6] = { &star_trans, NULL, NULL, NULL, NULL, NULL, };
                    string_split4(optarg, "/.", fields);
                }
                break;

            case OPT_ATRS:
                {
                    char **fields[6] = { &appr_trans, NULL, NULL, NULL, NULL, NULL, };
                    string_split4(optarg, "/.", fields);
                }
                break;

            case OPT_AFIN:
                {
                    char **fields[6] = { &final_appr, &appr_trans, NULL, NULL, NULL, NULL, };
                    string_split4(optarg, "/.", fields);
                }
                break;

            case OPT_IRTE:
                free(path_in);
                free(icao_route);
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

    // convert all IDs and flight route to uppercase
    if (info_aptidt)
    {
        for (size_t i = 0; info_aptidt[i] != '\0'; i++)
        {
            info_aptidt[i] = toupper(info_aptidt[i]);
        }
    }
    if (qpac_aptids)
    {
        for (size_t i = 0; qpac_aptids[i] != '\0'; i++)
        {
            qpac_aptids[i] = toupper(qpac_aptids[i]);
        }
    }
    if (dep_apt)
    {
        for (size_t i = 0; dep_apt[i] != '\0'; i++)
        {
            dep_apt[i] = toupper(dep_apt[i]);
        }
    }
    if (dep_rwy)
    {
        for (size_t i = 0; dep_rwy[i] != '\0'; i++)
        {
            dep_rwy[i] = toupper(dep_rwy[i]);
        }
    }
    if (sid_name)
    {
        for (size_t i = 0; sid_name[i] != '\0'; i++)
        {
            sid_name[i] = toupper(sid_name[i]);
        }
    }
    if (sid_trans)
    {
        for (size_t i = 0; sid_trans[i] != '\0'; i++)
        {
            sid_trans[i] = toupper(sid_trans[i]);
        }
    }
    if (arr_apt)
    {
        for (size_t i = 0; arr_apt[i] != '\0'; i++)
        {
            arr_apt[i] = toupper(arr_apt[i]);
        }
    }
    if (arr_rwy)
    {
        for (size_t i = 0; arr_rwy[i] != '\0'; i++)
        {
            arr_rwy[i] = toupper(arr_rwy[i]);
        }
    }
    if (star_name)
    {
        for (size_t i = 0; star_name[i] != '\0'; i++)
        {
            star_name[i] = toupper(star_name[i]);
        }
    }
    if (star_trans)
    {
        for (size_t i = 0; star_trans[i] != '\0'; i++)
        {
            star_trans[i] = toupper(star_trans[i]);
        }
    }
    if (final_appr)
    {
        for (size_t i = 0; final_appr[i] != '\0'; i++)
        {
            final_appr[i] = toupper(final_appr[i]);
        }
    }
    if (appr_trans)
    {
        for (size_t i = 0; appr_trans[i] != '\0'; i++)
        {
            appr_trans[i] = toupper(appr_trans[i]);
        }
    }
    if (icao_route)
    {
        for (size_t i = 0; icao_route[i] != '\0'; i++)
        {
            icao_route[i] = toupper(icao_route[i]);
        }
    }

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
        if (!path_navdat)
        {
            if (!(path_navdat = strdup(path)))
            {
                ret = ENOMEM;
                goto end;
            }

            // check if updated Custom Data is available
            if ((ret = ndt_file_getpath(path_xplane, "/Custom Data/GNS430/navdata/ATS.txt", &path, &pathlen)))
            {
                goto end;
            }
            if (!stat(path, &stats) && S_ISREG(stats.st_mode))
            {
                if ((ret = ndt_file_getpath(path_xplane, "/Custom Data/GNS430/navdata", &path, &pathlen)))
                {
                    goto end;
                }
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

    if (qpac_aptids)
    {
        if (!path_out)
        {
            if (!path_xplane)
            {
                fprintf(stderr, "No X-Plane or output directory provided\n");
                ret = EINVAL;
                goto end;
            }
            if ((ret = ndt_file_getpath(path_xplane, "/Output/FMS plans", &path, &pathlen)))
            {
                goto end;
            }
            if (!(path_out = strdup(path)))
            {
                ret = ENOMEM;
                goto end;
            }
        }
        if (stat(path_out, &stats))
        {
            strerror_r((ret = errno), error, sizeof(error));
            fprintf(stderr, "Bad output directory: '%s' (%s)\n", path_out, error);
            goto end;
        }
        if (!S_ISDIR(stats.st_mode))
        {
            strerror_r((ret = ENOTDIR), error, sizeof(error));
            fprintf(stderr, "Bad output directory: '%s' (%s)\n", path_out, error);
            goto end;
        }
        if (access(path_out, W_OK))
        {
            strerror_r((ret = errno), error, sizeof(error));
            fprintf(stderr, "Bad output directory: '%s' (%s)\n", path_out, error);
            goto end;
        }
        free(arr_apt); arr_apt = strdup(qpac_aptids);
        free(dep_apt); dep_apt = strsep(&arr_apt, "/\\-.");
        if (!dep_apt || !strlen(dep_apt))
        {
            fprintf(stderr, "Failed to split \"%s\"\n", qpac_aptids);
            ret = EINVAL;
            goto end;
        }
        if (!arr_apt && !(arr_apt = strdup(dep_apt)))
        {
            ret = ENOMEM;
            goto end;
        }
        goto end; // no other data needed
    }
    if (info_aptidt)
    {
        goto end; // no other data needed
    }
    if (!path_in && !icao_route && !dep_apt && !arr_apt)
    {
        fprintf(stderr, "No input file or route provided\n");
        ret = EINVAL;
        goto end;
    }
    if (path_in && access(path_in, R_OK))
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

static int print_airac(ndt_navdatabase *ndb)
{
    fprintf(stderr, "Navdata -> %s\n\n", ndb->info.desc);
    return 0;
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
            "  --airac               Print navadata description to stderr.      \n"
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
            "                            civa        Decoded route, one leg per \n"
            "                                        line, for use w/CIVA INS   \n"
            "                            decoded     Decoded route (wpt IDs),   \n"
            "                                        all legs on a single line  \n"
            "                            helper      Decoded route, one leg per \n"
            "                                        line, for use w/X-Plane FMS\n"
            "                            icao        ICAO flight plan route,    \n"
            "                                        without specific SID/STAR  \n"
            "                            ixeg        ICAO flight plan, for use  \n"
            "                                        as company route w/IXEG 733\n"
            "                            mcdu        Decoded route, one leg per \n"
            "                                        line, for use w/QPAC MCDU  \n"
            "                            recap       Generic route recap        \n"
            "                            xplane      X-Plane .fms flight plan   \n"
            "                                        with QPAC enhancements for \n"
            "                                        altitude constraints and   \n"
            "                                        fly-over waypoints         \n"
            "                        Default: xplane                            \n"
            "                                                                   \n"
            "  --info       <string> Print airport information to stdout for the\n"
            "                        specified ICAO identifier, including all   \n"
            "                        known procedure and transition names.      \n"
            "                                                                   \n"
            "  --qpac <icao>/<icao>  Write all SID procedures for the first ICAO\n"
            "                        identifier, and all STAR and final approach\n"
            "                        procedures for the second ICAO identifier, \n"
            "                        to files in X-Plane .fms format (including \n"
            "                        altitude and speed constraints using QPAC- \n"
            "                        specific enhancements). Files are written  \n"
            "                        to X-Plane's /Output/FMS plans/ folder by  \n"
            "                        default, unless a different output folder  \n"
            "                        is specified via option --o                \n"
            "                                                                   \n"
            "### Flight planning     -------------------------------------------\n"
            "  --dep        <string> Set departure airport (4-letter ICAO code).\n"
            "                        May be omitted if the departure is present \n"
            "                        in the route as well.                      \n"
            "  --drwy       <string> Set departure runway. Can be omitted.      \n"
            "                                                                   \n"
            "  --sid        <string> Select a SID procedure.                    \n"
            "  --sidtr      <string> Select enroute transition for the SID.     \n"
            "                                                                   \n"
            "  --arr        <string> Set arrival airport (4-letter ICAO code).  \n"
            "                        May be omitted if the arrival is present   \n"
            "                        in the route as well.                      \n"
            "  --arwy       <string> Set arrival runway. Can be omitted.        \n"
            "                                                                   \n"
            "  --star       <string> Select a STAR procedure.                   \n"
            "  --startr     <string> Select enroute transition for the STAR.    \n"
            "  --final      <string> Select a final approach procedure.         \n"
            "  --apptr      <string> Select a transition for the approach.      \n"
            "                        Setting it to \"auto\" will attempt to pick\n"
            "                        a valid approach transition automatically. \n"
            "                                                                   \n"
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
            " LSZH, with the route \"MOLUS N871 BERSU\", and write it to the    \n"
            " file \"LSGG-LSZH-01.fms\" in the current directory:               \n"
            "                                                                   \n"
            "     %s --xplane $XPLANE --dep LSGG --arr LSZH --rte \"MOLUS N871 BERSU\" -o LSGG-LSZH-01.fms\n"
            "                                                                   \n"
            " Convert an ICAO route with airways to a list of waypoints for use \n"
            " on e.g. SkyVector, and write it to the terminal:                  \n"
            "                                                                   \n"
            "     %s --xplane $XPLANE --rte \"LSGG MOLUS N871 BERSU LSZH\" --ofmt decoded\n"
            "                                                                   \n"
            " Sanitize an Airbus X Extended company route, converting it from   \n"
            " the old format (DCT as airway) to the new (DCT with coordinates): \n"
            "     %s --xplane $XPLANE --ifmt airbusx --ofmt airbusx -i <input> -o <output>\n"
            "                                                                   \n"
            "### Example formats -----------------------------------------------\n"
            "                                                                   \n"
            " \"icao\" (input and output):                                      \n"
            "     LSGG SID MOLUS N871 BERSU STAR LSZH                           \n"
            "     EIDW SID SUROX 56N020W 56N030W 55N040W 53N050W YAY J580 YQY SCUPP STAR KBOS\n"
            "                                                                   \n"
            " \"simbrief\" (output only):                                       \n"
            "     MOLUS N871 BERSU                                              \n"
            "     SUROX 5620N 5630N 5540N 5350N YAY J580 YQY SCUPP              \n"
            "                                                                   \n"
            " \"decoded\" (output only):                                        \n"
            "     MOLUS SOSAL TELNO KORED KONOL BERSU                           \n"
            "     SUROX 5620N 5630N 5540N 5350N YAY YJT YQY SCUPP               \n"
            "                                                                   \n"
            " \"civa\" (output only):                                           \n"
            "     ---------------------------------------------------------     \n"
            "     0  LSGG              0  N 46°14.3' E°006 06.6'                \n"
            "                                                                   \n"
            "     1  MOLUS             1  N 46°26.6' E°006 40.8'                \n"
            "     2  SOSAL             2  N 46°33.5' E°006 53.1'                \n"
            "     3  TELNO             3  N 46°46.3' E°007 16.2'                \n"
            "     4  KORED             4  N 46°51.0' E°007 24.9'                \n"
            "     5  KONOL             5  N 46°59.7' E°007 40.8'                \n"
            "     6  BERSU             6  N 47°08.1' E°007 56.5'                \n"
            "     7  LSZH              7  N 47°27.5' E°008 32.9'                \n"
            "     ---------------------------------------------------------     \n"
            "     0  EIDW              0  N 53°25.3' W°006 16.2'                \n"
            "                                                                   \n"
            "     1  SUROX             1  N 53°59.8' W°006 59.6'                \n"
            "     2  N56W020           2  N 56°00.0' W°020 00.0'                \n"
            "     3  N56W030           3  N 56°00.0' W°030 00.0'                \n"
            "     4  N55W040           4  N 55°00.0' W°040 00.0'                \n"
            "     5  N53W050           5  N 53°00.0' W°050 00.0'                \n"
            "     6  YAY               6  N 51°23.6' W°056 05.0'                \n"
            "     7  YJT               7  N 48°35.0' W°058 40.2'                \n"
            "     8  YQY               8  N 46°09.2' W°060 03.3'                \n"
            "     9  SCUPP             9  N 42°36.2' W°070 13.8'                \n"
            "                                                                   \n"
            "     1  KBOS              1  N 42°21.8' W°071 00.4'                \n"
            "     ---------------------------------------------------------     \n"
            "                                                                   \n"
            " \"helper\" (output only):                                         \n"
            "     ---------------------------------------------------------     \n"
            "      0  APT  LSGG               0  +46.238  +006.109              \n"
            "                                                                   \n"
            "      1  fix  MOLUS              1  +46.444  +006.679              \n"
            "      2  fix  SOSAL              2  +46.558  +006.884              \n"
            "      3  fix  TELNO              3  +46.772  +007.271              \n"
            "      4  fix  KORED              4  +46.851  +007.414              \n"
            "      5  fix  KONOL              5  +46.995  +007.681              \n"
            "      6  fix  BERSU              6  +47.136  +007.941              \n"
            "                                                                   \n"
            "      7  APT  LSZH               7  +47.458  +008.548              \n"
            "     ---------------------------------------------------------     \n"
            "      0  APT  EIDW               0  +53.421  -006.270              \n"
            "                                                                   \n"
            "      1  fix  SUROX              1  +53.997  -006.993              \n"
            "      2  ---  N56W020            3  +56.000  -020.000              \n"
            "      3  ---  N56W030            4  +56.000  -030.000              \n"
            "      4  ---  N55W040            5  +55.000  -040.000              \n"
            "      5  ---  N53W050            6  +53.000  -050.000              \n"
            "      6  VOR  YAY                6  +51.394  -056.084              \n"
            "      7  VOR  YJT                7  +48.583  -058.669              \n"
            "      8  VOR  YQY                8  +46.153  -060.056              \n"
            "      9  fix  SCUPP              9  +42.603  -070.230              \n"
            "                                                                   \n"
            "     10  APT  KBOS              10  +42.363  -071.006              \n"
            "     ---------------------------------------------------------     \n"
            "                                                                   \n"
            " \"mcdu\" (output only):                                           \n"
            "     ---------------------------------------------------------     \n"
            "      0  LSGG                  0  4614.3N/00606.6E                 \n"
            "                                                                   \n"
            "      1  MOLUS                 1  4626.6N/00640.8E                 \n"
            "      2  SOSAL                 2  4633.5N/00653.1E                 \n"
            "      3  TELNO                 3  4646.3N/00716.2E                 \n"
            "      4  KORED                 4  4651.0N/00724.9E                 \n"
            "      5  KONOL                 5  4659.7N/00740.8E                 \n"
            "      6  BERSU                 6  4708.1N/00756.5E                 \n"
            "                                                                   \n"
            "      7  LSZH                  7  4727.5N/00832.9E                 \n"
            "     ---------------------------------------------------------     \n"
            "      0  EIDW                  0  5325.3N/00616.2W                 \n"
            "                                                                   \n"
            "      1  SUROX                 1  5359.8N/00659.6W                 \n"
            "      2  N56W020               2  5600.0N/02000.0W                 \n"
            "      3  N56W030               3  5600.0N/03000.0W                 \n"
            "      4  N55W040               4  5500.0N/04000.0W                 \n"
            "      5  N53W050               5  5300.0N/05000.0W                 \n"
            "      6  YAY                   6  5123.6N/05605.0W                 \n"
            "      7  YJT                   7  4835.0N/05840.2W                 \n"
            "      8  YQY                   8  4609.2N/06003.3W                 \n"
            "      9  SCUPP                 9  4236.2N/07013.8W                 \n"
            "                                                                   \n"
            "     10  KBOS                 10  4221.8N/07100.4W                 \n"
            "                                                                   \n"
            " \"recap\" (output only):                                          \n"
            "     ---------------------------------------------------------     \n"
            "     Departure: LSGG (GENEVA), elevation (ft): 1411, transition (ft): 7000/ATC\n"
            "     Enroute:   MOLUS N871 BERSU                                   \n"
            "     Arrival:   LSZH (ZURICH), elevation (ft): 1417, transition (ft): 7000/ATC\n"
            "                                                                   \n"
            "     Flight route:                                                 \n"
            "                                                                   \n"
            "     DCT               060.6° (062.2°T)  26.6 nm                   \n"
            "     MOLUS                  N46°26.6'  E006°40.8'                  \n"
            "                                                                   \n"
            "     N871              049.2° (050.9°T)  10.9 nm                   \n"
            "     SOSAL                  N46°33.5'  E006°53.1'                  \n"
            "                                                                   \n"
            "     N871              049.2° (051.0°T)  20.4 nm                   \n"
            "     TELNO                  N46°46.3'  E007°16.2'                  \n"
            "                                                                   \n"
            "     N871              049.4° (051.3°T)   7.5 nm                   \n"
            "     KORED                  N46°51.0'  E007°24.9'                  \n"
            "                                                                   \n"
            "     N871              049.5° (051.4°T)  14.0 nm                   \n"
            "     KONOL                  N46°59.7'  E007°40.8'                  \n"
            "                                                                   \n"
            "     N871              049.6° (051.6°T)  13.6 nm                   \n"
            "     BERSU                  N47°08.1'  E007°56.5'                  \n"
            "                                                                   \n"
            "     DCT               049.7° (051.7°T)  31.4 nm                   \n"
            "     LSZH                   N47°27.5'  E008°32.9'                  \n"
            "     ---------------------------------------------------------     \n"
            "     Departure: EIDW (DUBLIN INTL), elevation (ft): 242, transition (ft): 5000/ATC\n"
            "     Enroute:   SUROX 56N020W 56N030W 55N040W 53N050W YAY J580 YQY SCUPP\n"
            "     Arrival:   KBOS (BOSTON/GENERAL EDWAR), elevation (ft): 19, transition (ft): 18000/FL180\n"
            "                                                                   \n"
            "     Flight route:                                                 \n"
            "                                                                   \n"
            "     DCT               326.9° (323.6°T)  43.0 nm                   \n"
            "     SUROX                  N53°59.8'  W006°59.6'                  \n"
            "                                                                   \n"
            "     DCT               294.0° (290.3°T) 462.7 nm                   \n"
            "     5620N                  N56°00.0'  W020°00.0'                  \n"
            "                                                                   \n"
            "     DCT               284.1° (274.1°T) 335.2 nm                   \n"
            "     5630N                  N56°00.0'  W030°00.0'                  \n"
            "                                                                   \n"
            "     DCT               278.5° (264.1°T) 344.8 nm                   \n"
            "     5540N                  N55°00.0'  W040°00.0'                  \n"
            "                                                                   \n"
            "     DCT               273.3° (255.3°T) 372.1 nm                   \n"
            "     5350N                  N53°00.0'  W050°00.0'                  \n"
            "                                                                   \n"
            "     DCT               269.1° (249.1°T) 243.5 nm                   \n"
            "     ST ANTHONY VOR/DME                                            \n"
            "     YAY                    N51°23.6'  W056°05.0'                  \n"
            "                                                                   \n"
            "     J580              231.8° (211.6°T) 196.0 nm                   \n"
            "     STEPHENVILLE VOR/DME                                          \n"
            "     YJT                    N48°34.9'  W058°40.2'                  \n"
            "                                                                   \n"
            "     J580              220.8° (201.6°T) 156.2 nm                   \n"
            "     SYDNEY VOR/DME                                                \n"
            "     YQY                    N46°09.2'  W060°03.3'                  \n"
            "                                                                   \n"
            "     DCT               265.9° (247.6°T) 485.1 nm                   \n"
            "     SCUPP                  N42°36.2'  W070°13.8'                  \n"
            "                                                                   \n"
            "     DCT               262.6° (247.5°T)  37.2 nm                   \n"
            "     KBOS                   N42°21.8'  W071°00.4'                  \n"
            "     ---------------------------------------------------------     \n"
            "                                                                   \n"
            " \"airbusx\" (input and output):                                   \n"
            "     ---------------------------------------------------------     \n"
            "     [CoRte]                                                       \n"
            "     ArptDep=LSGG                                                  \n"
            "     ArptArr=LSZH                                                  \n"
            "     DctWpt1=MOLUS                                                 \n"
            "     DctWpt1Coordinates=46.443889,6.679444                         \n"
            "     Airway2=N871                                                 \n"
            "     Airway2FROM=MOLUS                                             \n"
            "     Airway2TO=BERSU                                               \n"
            "     ---------------------------------------------------------     \n"
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
            "     ---------------------------------------------------------     \n"
            "                                                                   \n"
            " \"xplane\" (input and output):                                    \n"
            "     ---------------------------------------------------------     \n"
            "     I                                                             \n"
            "     3 version                                                     \n"
            "     1                                                             \n"
            "     7                                                             \n"
            "     1 LSGG 1411 46.238056 6.109167                                \n"
            "     11 MOLUS 0 46.443889 6.679444 0.000000                        \n"
            "     11 SOSAL 0 46.558056 6.884444 0.000000                        \n"
            "     11 TELNO 0 46.771944 7.270556 0.000000                        \n"
            "     11 KORED 0 46.850556 7.414167 0.000000                        \n"
            "     11 KONOL 0 46.995278 7.680556 0.000000                        \n"
            "     11 BERSU 0 47.135278 7.941111 0.000000                        \n"
            "     1 LSZH 1417 47.458056 8.548056                                \n"
            "     0 ---- 0 0.000000 0.000000                                    \n"
            "     0 ---- 0 0.000000 0.000000                                    \n"
            "     ---------------------------------------------------------     \n"
            "     I                                                             \n"
            "     3 version                                                     \n"
            "     1                                                             \n"
            "     10                                                            \n"
            "     1 EIDW 242 53.421389 -6.270000                                \n"
            "     11 SUROX 0 53.996667 -6.993333 0.000000                       \n"
            "     28 +56.00_-020.00 0 56.000000 -20.000000 0.000000             \n"
            "     28 +56.00_-050.00 0 56.000000 -30.000000 0.000000             \n"
            "     28 +55.00_-040.00 0 55.000000 -40.000000 0.000000             \n"
            "     28 +53.00_-050.00 0 53.000000 -50.000000 0.000000             \n"
            "     3 YAY 0 51.393889 -56.083611 0.000000                         \n"
            "     3 YJT 0 48.582500 -58.669167 0.000000                         \n"
            "     3 YQY 0 46.153333 -60.055556 0.000000                         \n"
            "     11 SCUPP 0 42.603056 -70.230278 0.000000                      \n"
            "     1 KBOS 19 42.362778 -71.006389                                \n"
            "     0 ---- 0 0.000000 0.000000                                    \n"
            "     0 ---- 0 0.000000 0.000000                                    \n"
            "     ---------------------------------------------------------     \n",
            NDCONV_EXE, NDCONV_EXE, NDCONV_EXE);
    return 0;
}

static int print_version(void)
{
    fprintf(stderr,
            "%s version %s, Copyright (c) 2014-2016 Timothy D. Walker           \n"
            "                                                                   \n"
            "Syntax: %s [options] -i <file> [-o <file>]                         \n",
            NDCONV_EXE, NDT_VERSION, NDCONV_EXE);
    return 0;
}
