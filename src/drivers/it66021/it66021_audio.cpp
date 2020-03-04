/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ite_type.h"
#include "it66021.h"
#include "ite_reg.h"


#ifdef _FIX_ID_028_
//xxxxx 2014-0417
//FIX_ID_028 xxxxx //For Debug Audio error with S2
#define AUDIO_READY_TIMEOUT                 	MS_TimeOut(0)	// change 100ms to 0 for speed up audio on
//FIX_ID_028 xxxxx
//xxxxx 2014-0417
#else
//FIX_ID_023 xxxxx		//Fixed for Audio Channel Status Error with invalid HDMI source
#define AUDIO_READY_TIMEOUT                 	MS_TimeOut(200)
//FIX_ID_023 xxxxx
#endif


#define AUDIO_MONITOR_TIMEOUT               MS_TimeOut(150)


const char  *AStateStr[] = {
	"ASTATE_AudioOff",
	"ASTATE_RequestAudio",
	"ASTATE_ResetAudio",
	"ASTATE_WaitForReady",
	"ASTATE_AudioOn",
	"ASTATE_Reserved"
};


void IT66021::getHDMIRXInputAudio(AUDIO_CAPS *pAudioCaps)
{
	unsigned char uc;

	uc = hdmirxrd(REG_RX_0AE); // REG_RX_AUD_CHSTAT3
	pAudioCaps->SampleFreq = uc & M_FS;

	uc = hdmirxrd(REG_RX_0AA); //REG_RX_AUDIO_CH_STAT
	pAudioCaps->AudioFlag = uc & 0xF0;
	pAudioCaps->AudSrcEnable = uc & M_AUDIO_CH;
	pAudioCaps->AudSrcEnable |= hdmirxrd(REG_RX_0AA) & M_AUDIO_CH;

	if ((uc & (B_HBRAUDIO | B_DSDAUDIO)) == 0) {
		uc = hdmirxrd(REG_RX_0AB); //REG_RX_AUD_CHSTAT0

		if ((uc & B_NLPCM) == 0) {
			pAudioCaps->AudioFlag |= B_CAP_LPCM;
		}
	}

#ifdef EnableCalFs

	if (hdmirxrd(REG_RX_074) & 0x40) {
		AudioFsCal();
	}

#endif
}

#ifdef _FIX_ID_028_
// ---------------------------------------------------------------------------
//FIX_ID_028 xxxxx //For Debug Audio error with S2
void IT66021::IT6602AudioOutputEnable(unsigned char bEnable)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	m_bAudioWaiting = FALSE;

	hdmirxset(REG_RX_HWMuteCtrl, (B_HWMuteClr), (B_HWMuteClr));
	hdmirxset(REG_RX_HWMuteCtrl, (B_HWAudMuteClrMode), (B_HWAudMuteClrMode));
	hdmirxset(REG_RX_HWMuteCtrl, (B_HWMuteClr), (0));
	hdmirxset(REG_RX_HWMuteCtrl, (B_HWAudMuteClrMode), (0));
	aud_fiforst();

	if (bEnable == TRUE) {
		hdmirxset(REG_RX_052, (B_TriI2SIO | B_TriSPDIF), 0x00);
		it6602data->m_AState = ASTATE_AudioOn;

		IT_INFO(" === IT6602AudioOutputEnable 11111111111 ==== \r\n");

	} else {
		hdmirxset(REG_RX_052, (B_TriI2SIO | B_TriSPDIF), (B_TriI2SIO | B_TriSPDIF));
		it6602data->m_AState = ASTATE_AudioOff;

		IT_INFO(" === IT6602AudioOutputEnable 00000000000 ==== \r\n");
	}
}
//FIX_ID_028 xxxxx

#else

// ---------------------------------------------------------------------------
void IT66021::IT6602AudioOutputEnable(unsigned char bEnable)
{
	if (bEnable == true) {
		hdmirxset(REG_RX_052, (B_TriI2SIO | B_TriSPDIF), 0x00);

	} else {
#ifdef EnableCalFs
		//FIX_ID_023 xxxxx		//Fixed for Audio Channel Status Error with invalid HDMI source
		m_u16TMDSCLK = 0;
		m_AudioChannelStatusErrorCount = 0;
		hdmirxset(REG_RX_074, 0x40, 0x00); // reg74[6]=0 disable Force FS mode
		//FIX_ID_023 xxxxx
#endif

		hdmirxset(REG_RX_052, (B_TriI2SIO | B_TriSPDIF), (B_TriI2SIO | B_TriSPDIF));
	}
}

#endif


void IT66021::IT6602SwitchAudioState(struct it6602_dev_data *it6602, Audio_State_Type state)
{
	//	unsigned char uc;

	if (it6602->m_AState == state) {
		return;
	}

	IT_INFO("+++ %s\n", AStateStr[(unsigned char)state]);

	it6602->m_AState = state;
	//AssignAudioVirtualTime();

	switch (it6602->m_AState) {
	case ASTATE_AudioOff:
		hdmirxset(REG_RX_RST_CTRL, B_AUDRST, B_AUDRST);
		IT6602AudioOutputEnable(false);

		break;

	case ASTATE_RequestAudio:
		IT6602AudioOutputEnable(false);

		break;

	case ASTATE_WaitForReady:
		hdmirx_SetHWMuteClr();
		hdmirx_ClearHWMuteClr();
		it6602->m_AudioCountingTimer = AUDIO_READY_TIMEOUT;
		break;

	case ASTATE_AudioOn:
#ifdef EnableCalFs
//FIX_ID_023 xxxxx		//Fixed for Audio Channel Status Error with invalid HDMI source
// AudioFsCal();
//FIX_ID_023 xxxxx
#endif
		IT6602AudioOutputEnable(true);

		IT_INFO("Cat6023 Audio--> Audio flag=%02X,Ch No=%02X,Fs=%02X ... \n",
			(int)it6602->m_RxAudioCaps.AudioFlag,
			(int)it6602->m_RxAudioCaps.AudSrcEnable,
			(int)it6602->m_RxAudioCaps.SampleFreq);
		break;

	default:
		break;
	}
}

#ifdef _FIX_ID_028_
//FIX_ID_028 xxxxx //For Debug Audio error with S2
//remove --> static void IT6602AudioHandler(struct it6602_dev_data *it6602)
//remove --> {
//remove --> }
//FIX_ID_028 xxxxx
#else
void IT66021::IT6602AudioHandler(struct it6602_dev_data *it6602)
{
	//    unsigned char uc;

	if (it6602->m_AudioCountingTimer > MS_LOOP) {
		it6602->m_AudioCountingTimer -= MS_LOOP;

	} else {
		it6602->m_AudioCountingTimer = 0;
	}

	if (it6602->m_RxHDCPState == RxHDCP_ModeCheck) {
		return;
	}

	switch (it6602->m_AState) {
	case ASTATE_RequestAudio:

		getHDMIRXInputAudio(&it6602->m_RxAudioCaps);

		if (it6602->m_RxAudioCaps.AudioFlag & B_CAP_AUDIO_ON) {

			hdmirxset(REG_RX_MCLK_CTRL, M_MCLKSel, B_256FS);

			if (it6602->m_RxAudioCaps.AudioFlag & B_CAP_HBR_AUDIO) {
				IT_INFO("+++++++++++ B_CAP_HBR_AUDIO +++++++++++++++++\n");

				hdmirxset(REG_RX_MCLK_CTRL, M_MCLKSel, B_128FS); // MCLK = 128fs only for HBR audio

				hdmirx_SetHWMuteClrMode();
				hdmirx_ResetAudio();

			} else if (it6602->m_RxAudioCaps.AudioFlag & B_CAP_DSD_AUDIO) {

				hdmirx_SetHWMuteClrMode();
				hdmirx_ResetAudio();

			} else {

				hdmirxset(REG_RX_HWMuteCtrl, B_HWMuteClr, 0x00);
				hdmirx_SetHWMuteClrMode();
				hdmirx_ResetAudio();
			}

			IT6602SwitchAudioState(it6602, ASTATE_WaitForReady);
		}

		break;

	case ASTATE_WaitForReady:

		//if(AudioTimeOutCheck(AUDIO_READY_TIMEOUT))
#ifdef EnableCalFs
		//FIX_ID_023 xxxxx		//Fixed for Audio Channel Status Error with invalid HDMI source
		TMDSGet();
//FIX_ID_023 xxxxx
#endif

		if (it6602->m_AudioCountingTimer == 0) {
			IT6602SwitchAudioState(it6602, ASTATE_AudioOn);
		}

		break;

	case ASTATE_AudioOn:

		//if(AudioTimeOutCheck(AUDIO_MONITOR_TIMEOUT)==TRUE)
		if (it6602->m_AudioCountingTimer == 0) {
			AUDIO_CAPS CurAudioCaps;
			//it6602->m_AudioCountingTimer = GetCurrentVirtualTime();
			//AssignAudioTimerTimeout(AUDIO_MONITOR_TIMEOUT);
			it6602->m_AudioCountingTimer = AUDIO_MONITOR_TIMEOUT;

			getHDMIRXInputAudio(&CurAudioCaps);

			if (it6602->m_RxAudioCaps.AudioFlag != CurAudioCaps.AudioFlag
			    || it6602->m_RxAudioCaps.AudSrcEnable != CurAudioCaps.AudSrcEnable
			    || it6602->m_RxAudioCaps.SampleFreq != CurAudioCaps.SampleFreq) {
				//it6602->m_ucHDMIAudioErrorCount=0;
				IT6602SwitchAudioState(it6602, ASTATE_RequestAudio);
			}
		}

		break;

	default:
		break;
	}
}


// ***************************************************************************
// Audio function
// ***************************************************************************
void IT66021::aud_fiforst(void)
{
	unsigned char uc;

#ifndef _FIX_ID_028_
	hdmirxset(REG_RX_074, 0x0c, 0x0c); // enable Mute i2s and ws	and s/pdif
	//IT_Delay(100);
	hdmirxset(REG_RX_074, 0x0c, 0x00); // disable Mute i2s and ws	and s/pdif
#endif

	hdmirxset(REG_RX_010, 0x02, 0x02);
	hdmirxset(REG_RX_010, 0x02, 0x00);

	uc = hdmirxrd(REG_RX_07B);
	hdmirxwr(REG_RX_07B, uc);
	hdmirxwr(REG_RX_07B, uc);
	hdmirxwr(REG_RX_07B, uc);
	hdmirxwr(REG_RX_07B, uc); // HOPE said, after FIFO reset, four valid update will active the AUDIO FIFO.
}


void IT66021::hdmirx_ResetAudio(void)
{
	unsigned char uc;
	hdmirxset(REG_RX_RST_CTRL, B_AUDRST, B_AUDRST);
	hdmirxset(REG_RX_RST_CTRL, B_AUDRST, 0x00);

	uc = hdmirxrd(REG_RX_07B);
	hdmirxwr(REG_RX_07B, uc);
	hdmirxwr(REG_RX_07B, uc);
	hdmirxwr(REG_RX_07B, uc);
	hdmirxwr(REG_RX_07B, uc); // HOPE said, after FIFO reset, four valid update will active the AUDIO FIFO.
}

void IT66021::hdmirx_SetHWMuteClrMode(void)
{
	hdmirxset(REG_RX_HWMuteCtrl, (B_HWAudMuteClrMode), (B_HWAudMuteClrMode));
}

void IT66021::hdmirx_SetHWMuteClr(void)
{
	hdmirxset(REG_RX_HWMuteCtrl, (B_HWMuteClr), (B_HWMuteClr));
}

void IT66021::hdmirx_ClearHWMuteClr(void)
{
	hdmirxset(REG_RX_HWMuteCtrl, (B_HWMuteClr), 0);
}

#endif

