/*
********************************************************************************
* COPYRIGHT(c) ЗАО «ЧИП и ДИП», 2019
* 
* Программное обеспечение предоставляется на условиях «как есть» (as is).
* При распространении указание автора обязательно.
********************************************************************************
*/




#ifndef __AUDIO_CONFIGURATION_H
#define __AUDIO_CONFIGURATION_H


#include "board.h"
#include "audio_node.h"
#include "usb_audio_constants.h"
#include "audio_speaker_node.h"


#define   USE_USB_AUDIO_CLASS_10                 1

#define   ALTERNATE_SETTING_16_BIT               1
#define   ALTERNATE_SETTING_24_BIT               2

#define   CONFIG_RES_BIT_16                      16
#define   CONFIG_RES_BYTE_16                     2

#define   CONFIG_RES_BIT_24                      24
#define   CONFIG_RES_BYTE_24                     3

#define   CONFIG_RES_BIT_32                      32
#define   CONFIG_RES_BYTE_32                     4


#define   CONFIG_2_0_STEREO_CHANNEL_COUNT        2
#define   CONFIG_2_0_STEREO_CHANNEL_MAP          0x03
#define   CONFIG_2_0_STEREO_16_BIT_FREQ_COUNT    4
#define   CONFIG_2_0_STEREO_24_BIT_FREQ_COUNT    4
#define   CONFIG_2_0_STEREO_32_BIT_FREQ_COUNT    1
#define   CONFIG_2_0_STEREO_16_BIT_MAX_PACKET    ((192 + 2) * CONFIG_2_0_STEREO_CHANNEL_COUNT * CONFIG_RES_BYTE_16)
#define   CONFIG_2_0_STEREO_24_BIT_MAX_PACKET    ((96 + 2) * CONFIG_2_0_STEREO_CHANNEL_COUNT * CONFIG_RES_BYTE_24)
#define   CONFIG_2_0_STEREO_32_BIT_MAX_PACKET    ((96 + 2) * CONFIG_2_0_STEREO_CHANNEL_COUNT * CONFIG_RES_BYTE_32)
#define   CONFIG_2_0_SAI_COUNT                   1
#define   CONFIG_2_0_FREQUENCY_DEFAULT           USB_AUDIO_CONFIG_FREQ_96_K
#define   CONFIG_2_0_32BIT_FREQUENCY_DEFAULT     USB_AUDIO_CONFIG_FREQ_96_K

#define   CONFIG_3_1_CHANNEL_COUNT               4
#define   CONFIG_3_1_CHANNEL_MAP                 0x0F
#define   CONFIG_3_1_16_BIT_FREQ_COUNT           3
#define   CONFIG_3_1_24_BIT_FREQ_COUNT           2
#define   CONFIG_3_1_16_BIT_MAX_PACKET           ((96 + 2) * CONFIG_3_1_CHANNEL_COUNT * CONFIG_RES_BYTE_16)
#define   CONFIG_3_1_24_BIT_MAX_PACKET           ((48 + 2) * CONFIG_3_1_CHANNEL_COUNT * CONFIG_RES_BYTE_24)

#define   CONFIG_4_0_QUADRO_CHANNEL_COUNT        4
#define   CONFIG_4_0_QUADRO_CHANNEL_MAP          0x33
#define   CONFIG_4_0_QUADRO_16_BIT_FREQ_COUNT    3
#define   CONFIG_4_0_QUADRO_24_BIT_FREQ_COUNT    2
#define   CONFIG_4_0_QUADRO_16_BIT_MAX_PACKET    ((96 + 2) * CONFIG_4_0_QUADRO_CHANNEL_COUNT * CONFIG_RES_BYTE_16)
#define   CONFIG_4_0_QUADRO_24_BIT_MAX_PACKET    ((48 + 2) * CONFIG_4_0_QUADRO_CHANNEL_COUNT * CONFIG_RES_BYTE_24)
#define   CONFIG_4_0_SAI_COUNT                   2

#define   DEVICE_ID1                             ((uint32_t)0x1FFF7A10)
#define   DEVICE_ID2                             ((uint32_t)0x1FFF7A14)
#define   DEVICE_ID3                             ((uint32_t)0x1FFF7A18)

#define   USB_AUDIO_CONFIG_SAI_MAX_COUNT         CONFIG_4_0_SAI_COUNT
#define   USB_AUDIO_CONFIG_PLAY_BUFFER_SIZE      ((1024 * 10) * USB_AUDIO_CONFIG_SAI_MAX_COUNT)

#define   AUDIO_CONFIG_2_0_STEREO_32_BIT         0
#define   AUDIO_CONFIG_2_0_STEREO                1
#define   AUDIO_CONFIG_3_1                       2
#define   AUDIO_CONFIG_4_0_QUADRO                3

#define   EXTERNAL_SYNC                          1


void SetAudioConfigDependedFuncs(AUDIO_SpeakerNode_t *speaker);

void Play_SAIMaster(uint16_t *Data, uint16_t Size, uint8_t ResByte);

void Play_SAIMasterAndSlave(uint16_t *Data, uint16_t Size, uint8_t ResByte);

void PrepareData_4_Chnls(uint8_t* AudioData, uint16_t Size, uint8_t ResInBytes);

void AudioChangeFrequency(uint32_t AudioFrequency);

void ExtSyncSelectSource(uint32_t AudioFrequency);

void AudioChangeResolution(uint8_t AudioResolution);

void AudioOutMute(uint8_t MuteFlag);

void AudioOutInit(uint32_t AudioFrequency, uint8_t AudioResolution);

void PlayDescriptionInit(AUDIO_Description_t *Description);

uint8_t GetAudioConfiguration(void);

void MakeSerialNumber(uint32_t *Buffer);

void AudioConfig_Init(void);

void OUTClk_Init(void);

void ExtPowerDisable(void);

void ConfigGPIOs_Init(void);

void InitDelay(void);




#endif // __AUDIO_CONFIGURATION_H

