/**************************
* Created by:     XMAN
* Created date:   2016-06-06
* Version:        V1.0
* Descriptions:   9370 config
**************************/
/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "vsw.h"
#include "ad9370.h"
#include "t_mykonos.h"
#include "mykonos.h"
#include "ad9370_config.h"
#include "mykonos_macros.h"
#include "device.h"
#include "demoinit.h"

#include "common.h" //ybq
#include "ARM_binary.h"

#define AD9370_ARMCODE "/demosw/config/Mykonos_M3.bin"


VSW_VOID Ad9370Reset(VSW_VOID )
{
	PRINTF_INFO("Reset AD9370 .......\n\r");
   	DevFpgaWrite(AD9370_RST_REG, 1);
//	usleep(100);
	_Wait(1);
       DevFpgaWrite(AD9370_RST_REG, 0);
//	usleep(100);
	_Wait(1);
       DevFpgaWrite(AD9370_RST_REG, 1);
}

VSW_RET Ad9370SpiWrite( VSW_USWORD16 addr, VSW_BYTE data)
{
	DevAd9370Write(addr, data);

    return VSW_RET_SUCC;
}

VSW_BYTE Ad9370SpiRead(VSW_USWORD16 addr)
{
	return (DevAd9370Read(addr));		

}

VSW_VOID ATTSet(VSW_BYTE chn, VSW_FLOAT att)
{
	VSW_USWORD16 reg,lereg;
				
	if((chn>=2) ||(att<0.0||att>31.5))
	{	
		PRINTF_ERROR("input para error!\n\r");
		return ;
	}

	reg = (0==chn)?CHN1_ATT_REG:CHN2_ATT_REG;
	lereg = (0==chn)?CHN1_ATT_LE_REG:CHN2_ATT_LE_REG;
	DevFpgaWrite(lereg, 0);
	DevFpgaWrite(reg, (VSW_USWORD16) (att*2));
	DevFpgaWrite(lereg, 1);
	usleep(1);
	DevFpgaWrite(lereg, 0);
	
}

VSW_RET AD9370Config1(VSW_VOID)
{
	VSW_BYTE mcsStatus = 0;
	VSW_BYTE pllLockStatus = 0;
//	VSW_BYTE binary[98304] = {0}; /*** < contains ARM binary file as array - set by user > ***/
//	VSW_USWORD32 count = sizeof(binary); ybq
	VSW_BYTE errorFlag = 0;
	VSW_BYTE errorCode = 0;
	VSW_USWORD32 initCalsCompleted = 0;
	VSW_USWORD16 errorWord = 0;
	VSW_USWORD16 statusWord = 0;
	VSW_BYTE status = 0;
	mykonosInitCalStatus_t initCalStatus = {0};
	//VSW_BYTE i;
	//VSW_BYTE gainIndex;
	//mykonosObsRxChannels_t obs_rx_source = 5;
	//VSW_BYTE ADC_XBAR;
	//VSW_USWORD16 obsRxDecPower_mdBFS;

	VSW_USWORD32 ullo,dllo;
	VSW_BYTE temp1,temp2,temp3;
	VSW_BYTE retVal;
	VSW_USWORD32 len_armcode;
	VSW_BYTE deframerStatus = 0;
	VSW_BYTE obsFramerStatus = 0;
	VSW_BYTE framerStatus = 0;
	VSW_USWORD32 initCalMask = TX_BB_FILTER | ADC_TUNER | TIA_3DB_CORNER | DC_OFFSET |
	TX_ATTENUATION_DELAY | RX_GAIN_DELAY | FLASH_CAL |
	PATH_DELAY | TX_LO_LEAKAGE_INTERNAL | TX_QEC_INIT | 
	LOOPBACK_RX_LO_DELAY | LOOPBACK_RX_RX_QEC_INIT |
	RX_LO_DELAY | RX_QEC_INIT;

	VSW_USWORD32 trackingCalMask = 0;

	mykonosErr_t mykError = MYKONOS_ERR_OK;
	PRINTF_INFO("Start to config Ad9370 ......\n\r");
	DevFpgaWrite(0xc0, 0);   //reg_TX_CHANNEL_SWITCH,tx0_jesd_datain  <= 0;tx1_jesd_datain  <= 0;

	DevFpgaWrite(0x60, 0x00);
	DevFpgaWrite(0x60, 0x03);//reset jesd204b soft_reset_tx_in,soft_reset_rx_in
	usleep(100);
	DevFpgaWrite(0x60, 0x00);

	DevFpgaWrite(0xc5, 0x01); //reg_jesd204b_rx_rst
	DevFpgaWrite(0xc6, 0x01); //reg_jesd204b_tx_rst
	DevFpgaWrite(0xc7, 0x01); //reg_jesd204b_fb_rst
	usleep(100);
	DevFpgaWrite(0xc5, 0x0);
	DevFpgaWrite(0xc6, 0x0);
	DevFpgaWrite(0xc7, 0x0);

	/**********************************************************/
	/**********************************************************/
	/************ Mykonos Initialization Sequence *************/
	/**********************************************************/
	/**********************************************************/

	/*** < Insert User System Clock(s) Initialization Code Here > ***/

	/*** < Insert User BBIC JESD204B Initialization Code Here > ***/

	/*******************************/
	/****初始化 ***/
	/*******************************/
	PRINTF_INFO("Ad9370: Start to reset Device......\n\r");
	mykError = MYKONOS_resetDevice(&mykDevice); //通过SPI1的几根GPIO给FPGA的RESET_9370的寄存器写入复位操作

	/* MYKONOS_initialize() loads the Mykonos device data structure
	 * settings for the Rx/Tx/ORx/Sniffer profiles, digital
	 * filter enables, calibrates the CLKPLL, and loads the user provided Rx
	 * gain tables.
	 */
	PRINTF_INFO("Ad9370: Start to initialize Device......\n\r");

	mykError = MYKONOS_initialize(&mykDevice);
	if (mykError != MYKONOS_ERR_OK)
	{
		/*** < User log error return value to debug why init failed > ***/
		PRINTF_ERROR("MYKONOS_initialize failed!\n\r");
	}
	


	/*******************************/
	/***** CLKPLL Status Check *****/
	/*******************************/
	PRINTF_INFO("Ad9370: Start to Check CLKPLL Status ......\n\r");
	mykError = MYKONOS_checkPllsLockStatus(&mykDevice, &pllLockStatus);

	if (pllLockStatus & 0x01)
	{
		/*** < CLKPLL locked - user code here > ***/
		PRINTF_ERROR("CLKPLL is locked\n\r");
	}
	else
	{
		/*** < CLKPLL not locked - ensure lock before proceeding - user code here > ***/
		PRINTF_ERROR("CLKPLL is unlocked\n\r");
	}
      
	/*******************************************************/
	/**** Perform MultiChip Sync (MCS) on Mykonos Device ***/
	/*******************************************************/
	PRINTF_INFO("Ad9370: Start to enable Multichip Sync ......\n\r");
	mykError = MYKONOS_enableMultichipSync(&mykDevice, 1, &mcsStatus);

	/*** < Request minimum 3 SYSREF pulses from Clock Device - user code here > ***/

	/*******************/
	/**** Verify MCS ***/
	/*******************/
	PRINTF_INFO("Ad9370: Start to unenable Multichip Sync ......\n\r");
	mykError = MYKONOS_enableMultichipSync(&mykDevice, 0, &mcsStatus);
	if ((mcsStatus & 0x0B) == 0x0B)
	{
		/*** < MCS successful - user code here > ***/
		PRINTF_INFO("MCS  set success!\n\r");
	}
	else
	{
		/*** < MCS failed - ensure MCS before proceeding - user code here > ***/
		PRINTF_ERROR("MCS set fail!\n\r");
	}
#if 1  //YBQ 171203
	/*************************/
	/**** Load  ARM ***/
	/*************************/
	PRINTF_INFO("Ad9370: Start to Load  ARM ......\n\r");
	if (pllLockStatus & 0x01)
	{
		mykError = MYKONOS_initArm(&mykDevice);
             PRINTF_DEBUG("" COLOR_RED "start to load arm code DONE\n\r" COLOR_NORMAL);

		/*** < user code here: user must load ARM binary byte array into variable binary[98304] before calling next command > ***/
#if 0
		retVal = ReadFile(AD9370_ARMCODE, 98304, binary, &len_armcode);
		if(retVal !=0 )
		{
			PRINTF_ERROR("Get arm code error!\n");
			return;
		}
#endif
		mykError = MYKONOS_loadArmFromBinary(&mykDevice, &binary[0], 98304);
		if (mykError != MYKONOS_ERR_OK)
		{
			/*** < ARM did not load properly - check binary and settings - user code here > ***/
			PRINTF_ERROR(" Load  ARM  code  failed!\n\r");

		}
		else
		{
			PRINTF_INFO("Load  ARM  code   successful!\n\r");
		}

	}
	else
	{
		/*** < check settings for proper CLKPLL lock - user code here > ***/
	}

	//DevFpgaWrite(0x60, 1);
	//usleep(100);
	//DevFpgaWrite(0x60, 0);
	
#endif
	/*******************************/
	/****设置 PLL  ***/
	/*******************************/
	PRINTF_INFO("Ad9370: Start to set  PLL ......\n\r");
	//ullo = DevEepromWordRead(EEPROM_FREQ_FREQSTART);
	ullo=1000000;
	if(0xffff != ullo)
	{
		mykDevice.rx->rxPllLoFrequency_Hz = (VSW_USWORD64)ullo*1000;
	}
	//dllo = DevEepromWordRead(EEPROM_FREQ_FREQSTOP);
	dllo=1000000;
	if(0xffff != dllo)
	{
		mykDevice.tx->txPllLoFrequency_Hz = (VSW_USWORD64)dllo*1000;
	}
	PRINTF_INFO("mykDevice.rx->rxPllLoFrequency_Hz==%llu\n\r", mykDevice.rx->rxPllLoFrequency_Hz);
	
	mykError = MYKONOS_setRfPllFrequency(&mykDevice, RX_PLL, mykDevice.rx->rxPllLoFrequency_Hz);
	PRINTF_INFO("mykDevice.tx->txPllLoFrequency_Hz==%llu\n\r", mykDevice.tx->txPllLoFrequency_Hz);
	mykError = MYKONOS_setRfPllFrequency(&mykDevice, TX_PLL, mykDevice.tx->txPllLoFrequency_Hz);
	PRINTF_INFO("mykDevice.obsRx->snifferPllLoFrequency_Hz==%llu\n\r", mykDevice.obsRx->snifferPllLoFrequency_Hz);
	mykError = MYKONOS_setRfPllFrequency(&mykDevice, SNIFFER_PLL, mykDevice.obsRx->snifferPllLoFrequency_Hz);

	/*** 延时 200ms 等待 PLL 锁定 ***/
	//usleep(200*1000); ybq
	_Wait(200); //ybq

	PRINTF_INFO("Ad9370: Start to check PLL Lock Statu......\n\r");
	mykError = MYKONOS_checkPllsLockStatus(&mykDevice, &pllLockStatus);
	if ((pllLockStatus & 0x0F) == 0x0F)
	{
		/*** < All PLLs locked - user code here > ***/
		PRINTF_INFO("All PLLs locked\n\r");
	}
	else
	{
		/*** < PLLs not locked - ensure lock before proceeding - user code here > ***/
		PRINTF_ERROR("All PLLs not locked\n\r");

	}
	


	/*****************************************************/
	/*** Mykonos ARM Initialization Calibrations       ***/
	/*****************************************************/
	PRINTF_INFO("Ad9370: Mykonos ARM Initialization Calibrations......\n\r");
	mykError = MYKONOS_runInitCals(&mykDevice, initCalMask);
	mykError = MYKONOS_waitInitCals(&mykDevice, 60000, &errorFlag, &errorCode);
	
	_Wait(2000);
	mykError = MYKONOS_runInitCals(&mykDevice, initCalMask);
	mykError = MYKONOS_waitInitCals(&mykDevice, 60000, &errorFlag, &errorCode);	

	if ((errorFlag != 0) || (errorCode != 0)) 
	{                     
		PRINTF_INFO("initial cals error!\n\r");
		mykError = MYKONOS_getInitCalStatus(&mykDevice, &initCalStatus);
		if(mykError)
		{
			/*** < User log error return value to debug why init failed > ***/
		}
         
		//abort init calibrations      
		mykError = MYKONOS_abortInitCals(&mykDevice, &initCalsCompleted);
		if(mykError)
		{
			/*** < User log error return value to debug why init failed > ***/

		}
		if(initCalsCompleted)
		{
			/*** < which calls had completed, per the mask > ***/ 
		}
		
		mykError = MYKONOS_readArmCmdStatus(&mykDevice, &errorWord, &statusWord);
		if(mykError)
		{
			/*** < User log error return value to debug why init failed > ***/
		}

		mykError = MYKONOS_readArmCmdStatusByte(&mykDevice, 2, &status);
		if(status!=0)
		{
			/*** < Arm Mailbox Status Error errorWord > ***/
			/*** < Pending Flag per opcode statusWord, this follows the mask > ***/
		}
            
		if(mykError)
		{
			/*** < User log error return value to debug why init failed > ***/
		}
	}
	else
	{
		PRINTF_INFO("initial cals successfully!\n\r");
		/*** < Calibrations completed successfully - user code here > ***/
	}
	
	/*************************************************/
	/**** Enable SYSREF to Mykonos JESD204B Framers ***/
	/*************************************************/
	/*** < User: Make sure SYSREF is stopped/disabled > ***/
	/*** < User: make sure BBIC JESD is reset and ready to recieve CGS chars> ***/
	
	
	PRINTF_INFO("Ad9370: Enable SYSREF to Mykonos JESD204B Framers......\n\r");

	DevFpgaWrite(0x60, 0x02);	
	//usleep(100); ybq
	_Wait(1);
	DevFpgaWrite(0x60, 0x00);	
	
	DevFpgaWrite(0xc5, 0x01);
	DevFpgaWrite(0xc7, 0x01);
		
	mykError = MYKONOS_enableSysrefToRxFramer(&mykDevice, 1);
	if (mykError)
	{
		/*** < Read error message to determine what went wrong  - user code here > ***/
	}
	/*** < User: Mykonos is actively transmitting CGS from the RxFramer> ***/

	DevFpgaWrite(0xc5, 0x0);
	DevFpgaWrite(0xc7, 0x0);

	mykError = MYKONOS_enableSysrefToObsRxFramer(&mykDevice, 1);
	if (mykError)
	{
		/*** < Read error message to determine what went wrong  - user code here > ***/
	}
	/*** < User: Mykonos is actively transmitting CGS from the ObsRxFramer> ***/

	DevFpgaWrite(0x60, 0x01);	
	//usleep(100);//ybq
	_Wait(300);//ybq
	DevFpgaWrite(0xc6, 0x01);
	DevFpgaWrite(0x60, 0x00);


	/***************************************************/
	/**** Enable SYSREF to Mykonos JESD204B Deframer ***/
	/***************************************************/
	/*** < User: Make sure SYSREF is stopped/disabled > ***/
	PRINTF_INFO("Ad9370: Enable SYSREF to Mykonos JESD204B Deframer......\n\r");
	mykError = MYKONOS_enableSysrefToDeframer(&mykDevice, 0);
	mykError = MYKONOS_resetDeframer(&mykDevice);

	/*** < User: make sure BBIC JESD framer is actively transmitting CGS> ***/
	DevFpgaWrite(0xc6, 0x00);
	
	mykError = MYKONOS_enableSysrefToDeframer(&mykDevice, 1);

	/*** < User Sends SYSREF Here > ***/

	/*** <  User: Insert User BBIC JESD Sync Verification Code Here > ***/

	/************************************/
	/**** Check Mykonos Framer Status ***/
	/************************************/
	PRINTF_INFO("Ad9370: Check Mykonos Framer Status......\n\r");
	mykError = MYKONOS_readRxFramerStatus(&mykDevice, &framerStatus);
	mykError = MYKONOS_readOrxFramerStatus(&mykDevice, &obsFramerStatus);

	/**************************************/
	/**** Check Mykonos Deframer Status ***/
	/**************************************/
	PRINTF_INFO("Ad9370: Check Mykonos Deframer Status......\n\r");
	mykError = MYKONOS_readDeframerStatus(&mykDevice, &deframerStatus);
	/*if((deframerStatus&0x0f) != 0x08)
	{
		PRINTF_INFO("Ad9370: JESD ERROR PREPARE TO REBOOT......\n\r");
		system("reboot");
	}*/
	/*** < User: When links have been verified, proceed > ***/

	/* Allow Rx1/2 QEC tracking and Tx1/2 QEC tracking to run when in the radioOn state         */
	/* Tx calibrations will only run if radioOn and the obsRx path is set to OBS_INTERNAL_CALS  */

	mykError = MYKONOS_runInitCals(&mykDevice, initCalMask);
	mykError = MYKONOS_waitInitCals(&mykDevice, 60000, &errorFlag, &errorCode);
	
	_Wait(2000);
	mykError = MYKONOS_runInitCals(&mykDevice, initCalMask);
	mykError = MYKONOS_waitInitCals(&mykDevice, 60000, &errorFlag, &errorCode);	


	
	
	PRINTF_INFO("Ad9370: Allow Rx1/2 QEC tracking and Tx1/2 QEC tracking to run when in the radioOn state......\n\r");
	mykError = MYKONOS_enableTrackingCals(&mykDevice, trackingCalMask);
  // mykError = MYKONOS_enableTrackingCals(&mykDevice, 0);
	/* Function to turn radio on, Enables transmitters and receivers */
	/* that were setup during MYKONOS_initialize() */
   	PRINTF_INFO("Ad9370: Function to turn radio on, Enables transmitters and receivers......\n\r");
	mykError = MYKONOS_radioOn(&mykDevice);

	/* Allow TxQEC to run when user is not actively using ORx receive path */
   	PRINTF_INFO("Ad9370: Allow TxQEC to run when user is not actively using ORx receive path......\n\r");
	mykError = MYKONOS_setObsRxPathSource(&mykDevice, OBS_INTERNALCALS);
	mykError = MYKONOS_setObsRxPathSource(&mykDevice, OBS_RX1_TXLO);
	
//	MYKONOS_setupRxAgc(&mykDevice);
//	mykError = MYKONOS_setRxGainControlMode(&mykDevice, AGC);
//	if(mykError == MYKONOS_ERR_OK)
//	{
//		printf("OKOKOK\n\r");
//	}
//	else
//	{
//		printf("GREAT ERROR\n\r");
//	}
//	if(mykDevice.rx->rxGainCtrl->gainMode == AGC)
//	{
//		printf("OKOKOK\n\r");
//	}
//	else
//	{
//		printf("GREAT ERROR\n\r");
//	}

	
	DevFpgaWrite(0x60,0x03);
	//usleep(100);ybq]
	_Wait(1);
	DevFpgaWrite(0x60,0x00);
	//sleep(2); ybq
	_Wait(2000);
	/*DevFpgaWrite(0xc5,0x1);
	DevFpgaWrite(0xc7,0x1);
	usleep(100);
	DevFpgaWrite(0xc5,0x00);
	DevFpgaWrite(0xc7,0x00);
      	DevFpgaWrite(0xc6,0x1);
	usleep(100);
	DevFpgaWrite(0xc6,0x00);*/

	//DevFpgaWrite(0xc0, 1);

	//RX framer states
	Ad9370SpiWrite(0x068,0x1); 
	//sleep(1); ybq
	_Wait(1000);
	Ad9370SpiWrite(0x068,0x0);
	//sleep(1);
	temp1 = Ad9370SpiRead(0x069);
	//ORX framer states
	Ad9370SpiWrite(0xDCF,0x1);
	sleep(1);
	Ad9370SpiWrite(0xDCF,0x0);
	sleep(1);
	temp2 = Ad9370SpiRead(0xDD0);
	//TX deframer states
	Ad9370SpiWrite(0x08A,0x1);
	sleep(1);
	Ad9370SpiWrite(0x08A,0x0);
	sleep(1);
	temp3 = Ad9370SpiRead(0x08B);
	if((0x3e!= (0x3e&temp1))||(0x3e!= (0x3e&temp2))||(0x28!= (0x28&temp3)))
	{
		PRINTF_INFO("ad9370 reg value 0x069:0x%x,0xDD0:0x%x,0x08A:0x%x.\n\r", temp1, temp2, temp3);
		DevFpgaWrite(0x60,0x3);
		usleep(100);
		DevFpgaWrite(0x60,0x0);
		sleep(1);

		//RX framer states
		Ad9370SpiWrite(0x068,0x1); 
		sleep(1);
		Ad9370SpiWrite(0x068,0x0);
		sleep(1);
		temp1 = Ad9370SpiRead(0x069);
		//ORX framer states
		Ad9370SpiWrite(0xDCF,0x1);
		sleep(1);
		Ad9370SpiWrite(0xDCF,0x0);
		sleep(1);
		temp2 = Ad9370SpiRead(0xDD0);
		//TX deframer states
		Ad9370SpiWrite(0x08A,0x1);
		sleep(1);
		Ad9370SpiWrite(0x08A,0x0);
		sleep(1);
		temp3 = Ad9370SpiRead(0x08B);
		PRINTF_INFO("ad9370 reg value 0x069:0x%x,0xDD0:0x%x,0x08A:0x%x.\n\r", temp1, temp2, temp3);
	} 
	 
	return VSW_RET_SUCC;
}


//AD9370状态检测
VSW_VOID bspAD9370StateCheck(VSW_VOID)
{
	mykonosErr_t mykError = MYKONOS_ERR_OK;
	VSW_BYTE deframerStatus = 0;

	mykError = MYKONOS_readDeframerStatus(&mykDevice, &deframerStatus);
	if((deframerStatus&0x0f) != 0x08)
	{
		PRINTF_INFO(COLOR_GREEN"------(bspAD9370StateCheck)deframerStatus = 0x%x \n\r"COLOR_NORMAL,deframerStatus);
		PRINTF_INFO("bspAD9370StateCheck: JESD ERROR PREPARE TO REBOOT......\n\r");
		//system("reboot");
	}

}

VSW_VOID fabulous_check(VSW_VOID)
{
	mykonosErr_t mykError = MYKONOS_ERR_OK;
	
	VSW_BYTE pllLockStatus = 0;
	mykError = MYKONOS_checkPllsLockStatus(&mykDevice, &pllLockStatus);
	if(mykError != MYKONOS_ERR_OK)
	{
		printf("ERROR! \n\r");
	}
	else
	{
		printf("OK! \n\r");
	}
	printf("pllLockStatus = %x\n\r",pllLockStatus );

	VSW_BYTE errorFlagStatus = 0;
	mykError = MYKONOS_getPaProtectErrorFlagStatus(&mykDevice, &errorFlagStatus);
	if(mykError != MYKONOS_ERR_OK)
	{
		printf("ERROR! \n\r");
	}
	else
	{
		printf("OK! \n\r");
	}
	printf("errorFlagStatus = %x\n\r",errorFlagStatus );

	VSW_USWORD32 radioStatus = 0;
	mykError = MYKONOS_getRadioState(&mykDevice, &radioStatus);
	if(mykError != MYKONOS_ERR_OK)
	{
		printf("ERROR! \n\r");
	}
	else
	{
		printf("OK! \n\r");
	}
	printf("radioStatus = %lx\n\r",radioStatus );

	VSW_USWORD32  pendingCalMask  = 0;
	mykError = MYKONOS_getPendingTrackingCals(&mykDevice, &pendingCalMask);
	if(mykError != MYKONOS_ERR_OK)
	{
		printf("ERROR! \n\r");
	}
	else
	{
		printf("OK! \n\r");
	}
	printf("pendingCalMask = %lx\n\r",pendingCalMask );

	VSW_BYTE txFilterStatus = 0;
	mykError = MYKONOS_getTxFilterOverRangeStatus(&mykDevice, &txFilterStatus);
	if(mykError != MYKONOS_ERR_OK)
	{
		printf("ERROR! \n\r");
	}
	else
	{
		printf("OK! \n\r");
	}
	printf("txFilterStatus = %x\n\r",txFilterStatus );

	VSW_BYTE deframerStatus = 0;
	mykError = MYKONOS_readDeframerStatus(&mykDevice, &deframerStatus);
	if(mykError != MYKONOS_ERR_OK)
	{
		printf("ERROR! \n\r");
	}
	else
	{
		printf("OK! \n\r");
	}
	printf("deframerStatus = %x\n\r",deframerStatus );

	VSW_BYTE obsFramerStatus = 0;
	mykError = MYKONOS_readOrxFramerStatus(&mykDevice, &obsFramerStatus);
	if(mykError != MYKONOS_ERR_OK)
	{
		printf("ERROR! \n\r");
	}
	else
	{
		printf("OK! \n\r");
	}
	printf("obsFramerStatus = %x\n\r",obsFramerStatus );

	VSW_BYTE framerStatus = 0;
	mykError = MYKONOS_readRxFramerStatus(&mykDevice, &framerStatus);
	if(mykError != MYKONOS_ERR_OK)
	{
		printf("ERROR! \n\r");
	}
	else
	{
		printf("OK! \n\r");
	}
	printf("framerStatus = %x\n\r",framerStatus );
	
	VSW_BYTE rx1GainIndex = 0;
	mykError = MYKONOS_getRx1Gain(&mykDevice, &rx1GainIndex);
	if(mykError != MYKONOS_ERR_OK)
	{
		printf("ERROR! \n\r");
	}
	else
	{
		printf("OK! \n\r");
	}
	printf("rx1GainIndex = %x\n\r",rx1GainIndex );
	_Wait(8000);
}


VSW_VOID TXRX_switch(VSW_VOID)
{
	
	while(1)
	{	
		_Wait(10000);
		printf("turn off\n\r");
		CMB_SPIWriteByte(mykDevice.spiSettings, MYKONOS_ADDR_CONFIGURATION_CONTROL_2, 0x00);
    CMB_SPIWriteByte(mykDevice.spiSettings, MYKONOS_ADDR_CONFIGURATION_CONTROL_4, 0x04);
		
		_Wait(10000);
		printf("turn on\n\r");
		CMB_SPIWriteByte(mykDevice.spiSettings, MYKONOS_ADDR_CONFIGURATION_CONTROL_2, 0xd2);
    CMB_SPIWriteByte(mykDevice.spiSettings, MYKONOS_ADDR_CONFIGURATION_CONTROL_4, 0xc6);
		
	}
}


VSW_VOID initial_fzr(VSW_VOID)
{
	
	VSW_USWORD32 trackingCalMask = 	TRACK_RX1_QEC |
	TRACK_RX2_QEC |
	TRACK_TX1_QEC |
	TRACK_TX2_QEC |
	TRACK_ORX1_QEC|
	TRACK_ORX2_QEC;
	
	while(1)
	{	
		
		_Wait(55000);
		MYKONOS_radioOff(&mykDevice);
		_Wait(5000);
		MYKONOS_enableTrackingCals(&mykDevice, trackingCalMask);
		MYKONOS_radioOn(&mykDevice);
		
		_Wait(55000);
		MYKONOS_radioOff(&mykDevice);
		MYKONOS_enableTrackingCals(&mykDevice, 0);
		_Wait(5000);
		MYKONOS_radioOn(&mykDevice);
	}
}