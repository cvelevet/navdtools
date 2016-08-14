/*
 * YFSkeys.c
 *
 * This file is part of the navdtools source code.
 *
 * (C) Copyright 2016 Timothy D. Walker and others.
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

#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMDefs.h"
#include "XPLM/XPLMDisplay.h"
#include "XPLM/XPLMUtilities.h"

#include "YFSkeys.h"
#include "YFSmain.h"
#include "YFSspad.h"

void yfs_keypressed(yfms_context *yfms, XPWidgetID key)
{
    if (!yfms || !key)
    {
        return; // no error
    }

    /* first key pressed since reset */
    if (yfms->xpl.atyp == YFS_ATYP_NSET)
    {
        do
        {
            /* check all possible datarefs and commands */
            yfms->xpl.ixeg.xpdr_mode_act   = XPLMFindDataRef("ixeg/733/xpdr/xpdr_mode_act"  );
            yfms->xpl.ixeg.xpdr_stby_act   = XPLMFindDataRef("ixeg/733/xpdr/xpdr_stby_act"  );
            yfms->xpl.qpac.XPDRPower       = XPLMFindDataRef("AirbusFBW/XPDRPower"          );
            yfms->xpl.qpac.XPDRAltitude    = XPLMFindDataRef("AirbusFBW/XPDRAltitude"       );
            yfms->xpl.qpac.BaroStdCapt     = XPLMFindDataRef("AirbusFBW/BaroStdCapt"        );
            yfms->xpl.qpac.BaroStdFO       = XPLMFindDataRef("AirbusFBW/BaroStdFO"          );
            yfms->xpl.qpac.RMPSwapCapt     = XPLMFindCommand("AirbusFBW/RMPSwapCapt"        );
            yfms->xpl.qpac.RMPSwapCo       = XPLMFindCommand("AirbusFBW/RMPSwapCo"          );
            yfms->xpl.qpac.VHF1Capt        = XPLMFindCommand("AirbusFBW/VHF1Capt"           );
            yfms->xpl.qpac.VHF2Co          = XPLMFindCommand("AirbusFBW/VHF2Co"             );
            yfms->xpl.qpac.RMP1FreqUpLrg   = XPLMFindCommand("AirbusFBW/RMP1FreqUpLrg"      );
            yfms->xpl.qpac.RMP1FreqUpSml   = XPLMFindCommand("AirbusFBW/RMP1FreqUpSml"      );
            yfms->xpl.qpac.RMP1FreqDownLrg = XPLMFindCommand("AirbusFBW/RMP1FreqDownLrg"    );
            yfms->xpl.qpac.RMP1FreqDownSml = XPLMFindCommand("AirbusFBW/RMP1FreqDownSml"    );
            yfms->xpl.qpac.RMP2FreqUpLrg   = XPLMFindCommand("AirbusFBW/RMP2FreqUpLrg"      );
            yfms->xpl.qpac.RMP2FreqUpSml   = XPLMFindCommand("AirbusFBW/RMP2FreqUpSml"      );
            yfms->xpl.qpac.RMP2FreqDownLrg = XPLMFindCommand("AirbusFBW/RMP2FreqDownLrg"    );
            yfms->xpl.qpac.RMP2FreqDownSml = XPLMFindCommand("AirbusFBW/RMP2FreqDownSml"    );
            yfms->xpl.fb76.systemMode      = XPLMFindDataRef("1-sim/transponder/systemMode" );

            /* use them to determine the custom aircraft type, if any */
            if (yfms->xpl.ixeg.xpdr_mode_act && yfms->xpl.ixeg.xpdr_stby_act)
            {
                yfms->xpl.atyp = YFS_ATYP_IXEG; break;
            }
            if (yfms->xpl.qpac.XPDRPower       && yfms->xpl.qpac.XPDRAltitude    &&
                yfms->xpl.qpac.BaroStdCapt     && yfms->xpl.qpac.BaroStdFO       &&
                yfms->xpl.qpac.RMPSwapCapt     && yfms->xpl.qpac.RMPSwapCo       &&
                yfms->xpl.qpac.VHF1Capt        && yfms->xpl.qpac.VHF2Co          &&
                yfms->xpl.qpac.RMP1FreqUpLrg   && yfms->xpl.qpac.RMP1FreqUpSml   &&
                yfms->xpl.qpac.RMP1FreqDownLrg && yfms->xpl.qpac.RMP1FreqDownSml &&
                yfms->xpl.qpac.RMP2FreqUpLrg   && yfms->xpl.qpac.RMP2FreqUpSml   &&
                yfms->xpl.qpac.RMP2FreqDownLrg && yfms->xpl.qpac.RMP2FreqDownSml)
            {
                yfms->xpl.atyp = YFS_ATYP_QPAC; break;
            }
            if (yfms->xpl.fb76.systemMode)
            {
                yfms->xpl.atyp = YFS_ATYP_FB76; break;
            }

            /* no custom type founmd, all default X-Plane systems used */
            yfms->xpl.atyp = YFS_ATYP_XPLN; break;
        }
        while (0);
        ndt_log("YFMS [info]: first key press, determined type %d\n", yfms->xpl.atyp);
    }

    /* keys that write to the scratchpad */
    if (key == yfms->mwindow.keys.keyid_al_a)
    {
        yfs_spad_apndc(yfms, 'A', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_b)
    {
        yfs_spad_apndc(yfms, 'B', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_c)
    {
        yfs_spad_apndc(yfms, 'C', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_d)
    {
        yfs_spad_apndc(yfms, 'D', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_e)
    {
        yfs_spad_apndc(yfms, 'E', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_f)
    {
        yfs_spad_apndc(yfms, 'F', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_g)
    {
        yfs_spad_apndc(yfms, 'G', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_h)
    {
        yfs_spad_apndc(yfms, 'H', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_i)
    {
        yfs_spad_apndc(yfms, 'I', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_j)
    {
        yfs_spad_apndc(yfms, 'J', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_k)
    {
        yfs_spad_apndc(yfms, 'K', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_l)
    {
        yfs_spad_apndc(yfms, 'L', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_m)
    {
        yfs_spad_apndc(yfms, 'M', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_n)
    {
        yfs_spad_apndc(yfms, 'N', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_o)
    {
        yfs_spad_apndc(yfms, 'O', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_p)
    {
        yfs_spad_apndc(yfms, 'P', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_q)
    {
        yfs_spad_apndc(yfms, 'Q', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_r)
    {
        yfs_spad_apndc(yfms, 'R', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_s)
    {
        yfs_spad_apndc(yfms, 'S', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_t)
    {
        yfs_spad_apndc(yfms, 'T', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_u)
    {
        yfs_spad_apndc(yfms, 'U', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_v)
    {
        yfs_spad_apndc(yfms, 'V', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_w)
    {
        yfs_spad_apndc(yfms, 'W', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_x)
    {
        yfs_spad_apndc(yfms, 'X', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_y)
    {
        yfs_spad_apndc(yfms, 'Y', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_al_z)
    {
        yfs_spad_apndc(yfms, 'Z', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_slsh)
    {
        yfs_spad_apndc(yfms, '/', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_spce)
    {
        yfs_spad_apndc(yfms, '_', -1); return; // cf. YFSmain.c, draw_display()
    }
    if (key == yfms->mwindow.keys.keyid_ovfy)
    {
        yfs_spad_reset(yfms, "^", -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_clir)
    {
        yfs_spad_remvc(yfms); return;
    }
    if (key == yfms->mwindow.keys.keyid_num0)
    {
        yfs_spad_apndc(yfms, '0', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num1)
    {
        yfs_spad_apndc(yfms, '1', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num2)
    {
        yfs_spad_apndc(yfms, '2', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num3)
    {
        yfs_spad_apndc(yfms, '3', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num4)
    {
        yfs_spad_apndc(yfms, '4', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num5)
    {
        yfs_spad_apndc(yfms, '5', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num6)
    {
        yfs_spad_apndc(yfms, '6', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num7)
    {
        yfs_spad_apndc(yfms, '7', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num8)
    {
        yfs_spad_apndc(yfms, '8', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_num9)
    {
        yfs_spad_apndc(yfms, '9', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_pird)
    {
        yfs_spad_apndc(yfms, '.', -1); return;
    }
    if (key == yfms->mwindow.keys.keyid_plus)
    {
        yfs_spad_apndc(yfms, '-', -1); return;
    }

    /* line select keys */
    for (int i = 0; i < (YFS_DISPLAY_NUMR - 1) / 2; i++)
    {
        int idx0[2] = { 0, i, };
        int idx1[2] = { 1, i, };
        if (yfms->mwindow.keys.keyid_lsk[i] == key &&
            yfms->lsks[0][i].cback)
        {
            yfms->lsks[0][i].cback(yfms, idx0, yfms->lsks[0][i].refcon);
        }
        if (yfms->mwindow.keys.keyid_rsk[i] == key &&
            yfms->lsks[1][i].cback)
        {
            yfms->lsks[1][i].cback(yfms, idx1, yfms->lsks[1][i].refcon);
        }
    }

    /* keys meant to open a specific page */
    if (yfms->mwindow.keys.keyid_menu == key &&
        yfms->spcs.cback_menu)
    {
        yfms->spcs.cback_menu(yfms);
    }
    if (yfms->mwindow.keys.keyid_radn == key &&
        yfms->spcs.cback_radn)
    {
        yfms->spcs.cback_radn(yfms);
    }
}

int yfs_keysniffer(char inChar, XPLMKeyFlags inFlags, char inVirtualKey, void *inRefcon)
{
    yfms_context *yfms = inRefcon; int x, xmin, xmax, y, ymin, ymax;
    if (!yfms)
    {
        return 1; // pass through
    }
    if ((inFlags & (xplm_ShiftFlag|xplm_OptionAltFlag|xplm_ControlFlag)) != 0)
    {
        return 1; // pass through
    }
    if ((inFlags & (xplm_DownFlag)) == 0)
    {
        return 1; // pass through
    }
    if (yfms->mwindow.ks_mode == YFS_KSM_OFF)
    {
        return 1; // pass through
    }
    if (XPIsWidgetVisible(yfms->mwindow.id) == 0)
    {
        return 1; // pass through
    }
    switch (yfms->mwindow.ks_mode)
    {
        case YFS_KSM_DSP: // mouse within main display only
            XPLMGetMouseLocation(&x, &y); XPGetWidgetGeometry(yfms->mwindow.screen.subw_id, &xmin, &ymax, &xmax, &ymin); break;
        case YFS_KSM_WIN: // mouse within main window boundaries
            XPLMGetMouseLocation(&x, &y); XPGetWidgetGeometry(yfms->mwindow.id,             &xmin, &ymax, &xmax, &ymin); break;
        case YFS_KSM_ALL: // anywhere
            XPLMGetMouseLocation(&x, &y);                                              xmin = xmax = x; ymin = ymax = y; break;
        default:
            return 1; // pass through
    }
    if (x < xmin || x > xmax || y < ymin || y > ymax)
    {
        return 1; // pass through
    }
    switch ((unsigned char)inVirtualKey)
    {
        case XPLM_VK_ADD:
            yfs_spad_apndc(yfms, '+', -1);
            return 0;
        case XPLM_VK_MINUS:
        case XPLM_VK_SUBTRACT:
            yfs_spad_apndc(yfms, '-', -1);
            return 0;
        case XPLM_VK_BACK:
            yfs_spad_remvc(yfms);
            return 0;
        case XPLM_VK_DECIMAL:
        case XPLM_VK_PERIOD:
            yfs_spad_apndc(yfms, '.', -1);
            return 0;
        case XPLM_VK_DELETE:
            yfs_spad_clear(yfms);
            return 0;
        case XPLM_VK_SLASH:
        case XPLM_VK_DIVIDE:
            yfs_spad_apndc(yfms, '/', -1);
            return 0;
        case XPLM_VK_SPACE:
            yfs_spad_apndc(yfms, ' ', -1);
            return 0;
        case XPLM_VK_UP:
        case XPLM_VK_DOWN:
        case XPLM_VK_LEFT:
        case XPLM_VK_RIGHT:
            return 1; // TODO (lnup, lndn, left, rigt)
        case XPLM_VK_0:
        case XPLM_VK_1:
        case XPLM_VK_2:
        case XPLM_VK_3:
        case XPLM_VK_4:
        case XPLM_VK_5:
        case XPLM_VK_6:
        case XPLM_VK_7:
        case XPLM_VK_8:
        case XPLM_VK_9:
        case XPLM_VK_NUMPAD0:
        case XPLM_VK_NUMPAD1:
        case XPLM_VK_NUMPAD2:
        case XPLM_VK_NUMPAD3:
        case XPLM_VK_NUMPAD4:
        case XPLM_VK_NUMPAD5:
        case XPLM_VK_NUMPAD6:
        case XPLM_VK_NUMPAD7:
        case XPLM_VK_NUMPAD8:
        case XPLM_VK_NUMPAD9:
            yfs_spad_apndc(yfms, inChar, -1);
            return 0;
        case XPLM_VK_A:
        case XPLM_VK_B:
        case XPLM_VK_C:
        case XPLM_VK_D:
        case XPLM_VK_E:
        case XPLM_VK_F:
        case XPLM_VK_G:
        case XPLM_VK_H:
        case XPLM_VK_I:
        case XPLM_VK_J:
        case XPLM_VK_K:
        case XPLM_VK_L:
        case XPLM_VK_M:
        case XPLM_VK_N:
        case XPLM_VK_O:
        case XPLM_VK_P:
        case XPLM_VK_Q:
        case XPLM_VK_R:
        case XPLM_VK_S:
        case XPLM_VK_T:
        case XPLM_VK_U:
        case XPLM_VK_V:
        case XPLM_VK_W:
        case XPLM_VK_X:
        case XPLM_VK_Y:
        case XPLM_VK_Z:
            yfs_spad_apndc(yfms, toupper(inChar), -1);
            return 0;
        default:
            return 1; // pass through
    }
}
