/*
 * File:           G:\cloud\manager\Электроника\ЦАП\DA-07\V2\DSP\96k\da07v2_96_IC96_PARAM.h
 *
 * Created:        Thursday, March 26, 2020 10:42:45 PM
 * Description:    da07v2_96:IC96 parameter RAM definitions.
 *
 * This software is distributed in the hope that it will be useful,
 * but is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * This software may only be used to program products purchased from
 * Analog Devices for incorporation by you into audio products that
 * are intended for resale to audio product end users. This software
 * may not be distributed whole or in any part to third parties.
 *
 * Copyright ©2020 Analog Devices, Inc. All rights reserved.
 */
#ifndef __DA07V2_96_IC96_PARAM_H__
#define __DA07V2_96_IC96_PARAM_H__


/* Module VOL_R_96 - Single Volume*/
#define MOD_VOL_R_96_COUNT                             1
#define MOD_VOL_R_96_DEVICE                            "IC96"
#define MOD_VOL_R_96_GAIN1940ALGNS1_ADDR               0
#define MOD_VOL_R_96_GAIN1940ALGNS1_FIXPT              0x00287A26
#define MOD_VOL_R_96_GAIN1940ALGNS1_VALUE              SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0.316227766016838)
#define MOD_VOL_R_96_GAIN1940ALGNS1_TYPE               SIGMASTUDIOTYPE_FIXPOINT

/* Module VOL_L_96 - Single Volume*/
#define MOD_VOL_L_96_COUNT                             1
#define MOD_VOL_L_96_DEVICE                            "IC96"
#define MOD_VOL_L_96_GAIN1940ALGNS2_ADDR               1
#define MOD_VOL_L_96_GAIN1940ALGNS2_FIXPT              0x00287A26
#define MOD_VOL_L_96_GAIN1940ALGNS2_VALUE              SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0.316227766016838)
#define MOD_VOL_L_96_GAIN1940ALGNS2_TYPE               SIGMASTUDIOTYPE_FIXPOINT

/* Module Phat-Stereo1 - Phat-Stereo*/
#define MOD_PHAT_STEREO1_COUNT                         2
#define MOD_PHAT_STEREO1_DEVICE                        "IC96"
#define MOD_PHAT_STEREO1_ALG0_PHATSTEREOALG19401SPREADLEVEL_ADDR 2
#define MOD_PHAT_STEREO1_ALG0_PHATSTEREOALG19401SPREADLEVEL_FIXPT 0x0050C335
#define MOD_PHAT_STEREO1_ALG0_PHATSTEREOALG19401SPREADLEVEL_VALUE SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0.630957344480193)
#define MOD_PHAT_STEREO1_ALG0_PHATSTEREOALG19401SPREADLEVEL_TYPE SIGMASTUDIOTYPE_FIXPOINT
#define MOD_PHAT_STEREO1_ALG0_PHATSTEREOALG19401ALPHASPREAD_ADDR 3
#define MOD_PHAT_STEREO1_ALG0_PHATSTEREOALG19401ALPHASPREAD_FIXPT 0x000F9924
#define MOD_PHAT_STEREO1_ALG0_PHATSTEREOALG19401ALPHASPREAD_VALUE SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0.121861049856833)
#define MOD_PHAT_STEREO1_ALG0_PHATSTEREOALG19401ALPHASPREAD_TYPE SIGMASTUDIOTYPE_FIXPOINT

#endif
