/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body; on-grid PV micro-inverter battery controller, HW v01/v02
  ******************************************************************************
  * @attention
  * design, idea and implementation of HW and FW
  * **** WTB Wojciech Błędziński ****
  * bendziol@o2.pl
  * originally designed on STM32L431RCT6
  *
  *http://www.bepat.de/2020/12/02/stm32f103c8-uart-with-dma-buffer-and-idle-detection/
  *https://www.digikey.lt/en/maker/projects/getting-started-with-stm32-working-with-adc-and-dma/f5009db3a3ed4370acaf545a3370c30c
  *https://www.freertos.org/FreeRTOS-for-STM32F4xx-Cortex-M4F-IAR.html
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define DEBUG_EN		1	//comment to get production functionality
//#define TESTING_VALUES	1	//comment to get production values
#define FLASHSTATSAVE_PERIOD	0	//0- stats saved every hour; 1-stats saved only when fully ch/discharged
#define HW_VER		01		//1st rev 01; 2nd rev 02; etc.
#ifndef TESTING_VALUES		//********* below are PRODUCTION VALUES ************
#define TICKS_ONESECOND	0	//counter ticks for one second (default 0)
#define TICKS_ONEMINUTE	59	//counter ticks for one minute (default 59)
#define TICKS_ONEHOUR	59	//counter ticks for one hour (default 59)
#define TICKS_ONEDAY	23	//counter ticks for one day (default 23)
#define RX_BFR_SIZE 127		//uart 1
#define TX_BFR_SIZE 1023		//uart 1
#define NO_FLASH_PAGES	52	//number of flash pages for storing statistics to flash
#define MOSFET_MAX_TEMP		80	//Mosfet max operating temperature *C
#define PV_CURRENT_MIN		290	//in 0.001A, min PV curent to assume that it is daylight - for alghorithm. after HAL_ADC_Calibration values for current circuit are of bigger values - instead 120mA it returns 350mA
								//current>PV_CURRENT_MIN to enter DAYTIME; (PV_CURRENT_MIN - PV_CURRENT_HYST) to keep "DAYTIME"; current<(PV_CURRENT_MIN - PV_CURRENT_HYST) to enter nightime; current<PV_CURRENT_MIN to keep nightime
								//this value sets minimal level of current that can make inverter working without excessive reactive power at mains side(AC current)
#define PV_CURRENT_HYST		70	//in 0.001A, PV curent hysteresis, to prevent frequent change chg/dschg during dusk and dawn
#define INV_CURRENT_MIN		290	//in 0.001A, min inverter current to assume that inv transfers energy to mains. usually same as PV_CURRENT_MIN,
#define INV_CURRENT_MAX		12500	//in 0.001A, maximum inverter current during batttery discharging. Helps prevent overheating
#define INV_CURR_SC			18000	//in 0.001A, more than that is considered as short circuit, all will be shut down without delay (in 1 second)
#define PV_OCV_VOLGATE		430	//in 0.1V, OCV voltage for PV panel without load (minimal PV OCV voltage for hot, cloudy day)
#define BATT_CRITICAL_MIN_VOLTAGE	240	//in 0.1V, min limit for charging
#define BATT_MIN_VOLTAGE	349	//in 0.1V, consider min limit for discharging with "-hysteresisMIN"
#define BATT_MAX_VOLTAGE	401	//in 0.1V, consider max limit for charging with "+hysteresisMAX"
#define BATT_VOLTAGE_MAXHYSTERESIS	20	//in 0.1V, to prevent frequent switching chg/dschg at max voltage
#define BATT_VOLTAGE_MINHYSTERESIS	22	//in 0.1V, to prevent frequent switching chg/dschg at min voltage
#define SECONDS_HOUR		3600	//to calculate Watt-hours from Watt-seconds
#define LEDSTATUS_TIMER_LONG		9		//10 seconds cycle
#define LEDSTATUS_TIMER_MED			3		//4 seconds cycle
#define LEDSTATUS_TIMER_SHORT		1		//2 seconds cycle
#define TIME2RESET_INV				2500	//30min/1800s, 42m/2500s; if INV not working for longer than ... seconds perform reset procedure
#define TIME2OVLD_INV				35		//maximum time of overload,seconds, during battery discharge
#define TOUT_BATTRECHARGE			600		//600s/10m time limit for battery recharge
#define DUSK_TIME1					60*60	//60 minutes	delayed INV ON after dusk. in seconds
#define DUSK_TIME2					2*60*60	//2*60 minutes
#define DUSK_TIME3					3*60*60	//3h
#define DUSK_TIME4					4*60*60	//4h
#define DUSK_TIME5					5*60*60	//5h
#define DUSK_TIME6					6*60*60	//6h
#else						//TESTING VALUES

#define TICKS_ONESECOND	0	//counter ticks for one second (default 0)
#define TICKS_ONEMINUTE	10	//counter ticks for one minute (default 59)
#define TICKS_ONEHOUR	2	//counter ticks for one hour (default 59)
#define TICKS_ONEDAY	2	//counter ticks for one day (default 23)
#define RX_BFR_SIZE 127		//uart 1
#define TX_BFR_SIZE 1023		//uart 1
#define NO_FLASH_PAGES	48	//number of flash pages for storing statistics to flash
#define MOSFET_MAX_TEMP		30	//Mosfet max operating temperature *C
#define PV_CURRENT_MIN		300	//in 0.001A, min PV curent to assume that it is daylight
#define INV_CURRENT_MIN		300	//in 0.001A, min inverter current to assume that inv transfers energy to mains
#define INV_CURRENT_MAX		13000	//in 0.001A, maximum inverter current during batttery discharging. Helps prevent overheating
#define INV_CURR_SC			18000	//in 0.001A, more than that is considered as short circuit, all will be shut down without delay (in 1 second)
#define PV_OCV_VOLGATE		230	//in 0.1V, OCV voltage for PV panel without load (minimal voltage for hot day)
#define BATT_CRITICAL_MIN_VOLTAGE	90	//in 0.1V, min limit for charging
#define BATT_MIN_VOLTAGE	120	//in 0.1V, consider min limit for discharging with - hysteresis
#define BATT_MAX_VOLTAGE	225	//in 0.1V, consider max limit for charging with + hysteresis
#define BATT_VOLTAGE_MAXHYSTERESIS	05	//in 0.1V, to prevent frequent switching chg/dschg at max voltage
#define BATT_VOLTAGE_MINHYSTERESIS	05	//in 0.1V, to prevent frequent switching chg/dschg at min voltage
#define SECONDS_HOUR		1	//to calculate Watt-hours from Watt-seconds; to calculate hours from seconds
#define LEDSTATUS_TIMER_LONG		3		//10 seconds cycle
#define LEDSTATUS_TIMER_MED			2		//4 seconds cycle
#define LEDSTATUS_TIMER_SHORT		1		//2 seconds cycle
#define TIME2RESET_INV				20		//30sec, if INV not working for longer than ... seconds do reset procedure
#define TIME2OVLD_INV				15		//10 seconds of overload, during battery discharge
#define TOUT_BATTRECHARGE			10		//600s/10m time limit for battery recharge
#define DUSK_TIME1					30		//30 minutes	delayed INV ON after dusk. in seconds
#define DUSK_TIME2					2*30	//2*30 minutes
#define DUSK_TIME3					3*30	//1,5h
#define DUSK_TIME4					4*30	//2h
#define DUSK_TIME5					5*30	//2,5h
#define DUSK_TIME6					6*30	//3h
#endif
#define CONFIG_MAINS_0DELAY			0		//state-mashine steps
#define CONFIG_MAINS_1DELAY			1
#define CONFIG_MAINS_2DELAY			2
#define CONFIG_MAINS_3DELAY			3
#define CONFIG_MAINS_4DELAY			4
#define CONFIG_MAINS_5DELAY			5
#define CONFIG_MAINS_6DELAY			6
#define CONFIG_BATT_0DELAY			7
#define CONFIG_BATT_1DELAY			8
#define CONFIG_BATT_2DELAY			9
#define CONFIG_BATT_3DELAY			10
#define CONFIG_BATT_4DELAY			11
#define CONFIG_BATT_5DELAY			12
#define CONFIG_BATT_6DELAY			13
#define CONFIG_MAINS_NOBATTDSCHG	14

__attribute__((__section__(".user_data"))) const char Stats_savedInFLASH[NO_FLASH_PAGES][FLASH_PAGE_SIZE];	//matrix in flash for statistics storage
__attribute__((__section__(".user_data"))) const char Cal_savedInFLASH[FLASH_PAGE_SIZE];	//data in flash for calibration values storage, size one full page
//compiling with GCC STM32CubeIDE: only without optimization makes flash reading function working properly
//any level of optimization makes flash reading function operate improperly (reading 0xFF)
//flash writing works good even with strong optimization
void __attribute__((optimize("O0"))) RestoreStatisticsFromFLASH();
void __attribute__((optimize("O0"))) RestoreCalValuesFromFLASH();
void __attribute__((optimize("O0"))) StoreStatistics2FLASH();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};
/* USER CODE BEGIN PV */
uint8_t RxRollover = 0;
uint8_t RxCounter = 0;
uint16_t RxBfrPos = 0;
uint8_t TxCounter = 0;
char TxBuffer[TX_BFR_SIZE];
uint8_t RxBuffer[RX_BFR_SIZE];
uint8_t ConfigReg;
uint32_t RecentPage_pointer;	//pointer to actual page adress (needs shifting to get direct adress)
uint32_t count_second=TICKS_ONESECOND, count_minutes=TICKS_ONEMINUTE, count_hours=TICKS_ONEHOUR, count_days=TICKS_ONEDAY;
uint32_t FlagBatteryMOS=0;		//flag indicating Mosfet status: ON 1 or OFF 0
uint32_t FlagInverterMOS=0;		//flag indicating Mosfet status: ON 1 or OFF 0
uint32_t FlagBackupMOS=0;		//flag indicating backup power mosfet status
uint32_t FlagExt_I=0;			//flag showing External input (reset microinverter) reading
uint32_t VoltHysteresisChg=0, VoltHysteresisDsChg=0; PVCurrentHysteresis=0;
uint32_t LedStatus, LedStatusTimer;
uint32_t Flag_StoreStatistics=0;
uint32_t Flag_ShowStats=0;
uint32_t FlagRunMainLoop=0;			//flag to synhronize main loop with software timer procedure
uint32_t FlagResetInverter, TimeToResetInv=TIME2RESET_INV, StateResetInv;		//variables for resetInv procedure
typedef struct
{
	uint32_t Inv_current;
	uint32_t PV_current;
	uint32_t PV_voltage;
	uint32_t Batt_voltage;
	uint32_t NTC1_PCB;
	uint32_t NTC2_Inverter_mos;
	uint32_t NTC3_Battery_mos;
	uint32_t VrefInt;
}AdcConvertVals;

typedef struct		//should have even number of uint32_t due to 64bit word save flash function
{
	uint32_t FlashPageCounter;
	uint32_t Wh_BattIn;				//1.variable count energy stored in battery, battery storage mode only
	uint32_t Wh_BattNoInv;			//2.variable count energy stored in battery (should be INV), battery mains mode only
	uint32_t Wh_Inverter;			//3.variable count energy transferred to inverter from PV
	uint32_t Wh_BattOut;			//variable counts energy transferred to INV from battery (during discharge)
	uint32_t Wh_BattRecharge;		//variable counts energy used to keep battery above minimal voltage
	uint32_t Time_NoInv;			//variable counts time when INV doesn't consume energy from PV
	uint32_t Time_NightTime;		//total nighttime
	uint32_t Time_NoBattery2Chg;	//variable counts time when energy should be stored in battery but battery is full and can't. in Mains mode all that energy is wasted -> inv doesn't work
	uint32_t Dschg_cycle_count;		//counter, partial of full discharges. if this counter is significantly higher than Chg_cycle_count battery is probably too big in capacity
	uint32_t Chg_cycle_count;		//counter, partial or full charge cycles (to max. set voltage level), if this counter is close to Dschg_cycle_count, battery is probably too small in capacity
	uint32_t InvResetCntr;			//counter, number of reset procedures performed (more than 30minutes without conversion during daytime); includes InvExtResetCnt value
	uint32_t InvFaultCntr;			//counter, number of occurences for Inverter Fault condition (no conversion from PV)
	uint32_t MaxTempInvMos;			//max recorded Inverter mosfet temp
	uint32_t MaxTempInvMosCntr;		//counter, number of occurences, for long ones its time in seconds
	uint32_t MaxTempBatMos;			//max recorded battery mosfet temp
	uint32_t MaxTempBatMosCntr;		//counter, number of occurences, for long ones its time in seconds
	uint32_t MaxInvCurrent;			//max recorded inverter current
	uint32_t MaxInvCurrentCntr;		//counter, number of occurences, for long ones its time in seconds
	uint32_t MaxPVCurrent;			//max recorded PV current
	uint32_t MaxPVCurrentCntr;		//counter, number of occurences, for long ones its time in seconds
	uint32_t MaxBatVoltage;			//18. max recorded battery voltage
	uint32_t MaxBatVoltageCntr;		//counter, number of occurences, for long ones its time in seconds
	uint32_t MinBatVoltage;			//19.min recorded battery voltage
	uint32_t MinBatVoltageCntr;		//counter, number of occurences, for long ones its time in seconds
	uint32_t InvOvcCounter;			//20. counter, number of resets due to Inverter over current; during battery discharge
	uint32_t InvExtResetCnt;		//21. counter, number of resets due to Ext reset signal
	uint32_t Dschg_Ah_lastFull;		//discharge mAh during end of last discharge, today
	uint32_t Dschg_Ah_lastFull_1;		//discharge mAh during end of last discharge, yesterday
	uint32_t Dschg_Ah_lastFull_2;		//discharge mAh during end of last discharge, yesterday
	uint32_t Dschg_Ah_lastFull_3;		//discharge mAh during end of last discharge, yesterday
	uint32_t Dschg_Ah_current;		//current dsch mAh
	uint32_t Chg_Ah_lastFull;		//charge mAh during end of last charge
	uint32_t Chg_Ah_last;			//charge mAh at end of the day/beginig of the night (stored in 900 second of the dusktime)
	uint32_t Chg_Ah_1;				//charge mAh at end of the yesterday/beginig of the night (stored in 900 second of the dusktime)
	uint32_t Chg_Ah_2;				//charge mAh at end of the -2day/beginig of the night (stored in 900 second of the dusktime)
	uint32_t Chg_Ah_3;				//charge mAh at end of the -3day/beginig of the night (stored in 900 second of the dusktime)
	uint32_t Chg_Ah_current;		//current charge mAh
	uint32_t Dschg_Volt_lastFull;	//battery voltage in 0.1V at the end of discharge
	uint32_t Chg_Volt_lastFull;		//battery voltage in 0.1V at the end of charge
	uint32_t DayDuration_current;	//current day duration
	uint32_t DayDuration_1;			//yesterday day duration
	uint32_t DayDuration_2;			//day before yesterday day duration
	uint32_t DayDuration_3;			//2 days before yesterday day duration
	//uint32_t nousedvar;		//add something here to make number of variables even
}_WhStatistics;

_WhStatistics StatCurrentWh, Stat_Flash;

typedef struct
{
	uint32_t Ws_BattIn;				//variable count energy stored in battery, battery storage mode only
	uint32_t Ws_BattNoInv;			//variable count energy stored in battery (should be INV), battery mains mode only
	uint32_t Ws_Inverter;			//variable count energy transferred to inverter from PV
	uint32_t Ws_BattOut;			//variable counts energy transferred to INV from battery (during discharge)
	uint32_t Ws_BattRecharge;		//variable counts energy used to keep battery above minimal voltage
	uint32_t Time_BattRecharge;		//used only as a flag
	uint32_t Time_NoInv;			//variable counts time when INV doesn't consume energy from PV
	uint32_t Time_NightTime;		//total nighttime
	uint32_t Time_NoBattery2Chg;	//variable counts time when energy should be stored in battery but battery is full and can't. in Mains mode all that energy is wasted -> inv doesn't work
	uint32_t Dschg_cycle_count;		//used only as a flag
	uint32_t Chg_cycle_count;		//used only as a flag
	uint32_t Dschg_cycle_c2;		//used only as a flag
	uint32_t Chg_cycle_c2;			//used only as a flag
	uint32_t InvFault;				//used only as a flag
	uint32_t InvOutShorted;			//used only as a flag
	uint32_t Time_Daytime;			//used only as a flag
	uint32_t ChgStatSaved;			//used only as a flag
	uint32_t DchgStatSaved;			//used only as a flag
	uint32_t EmptyBattStatsSaved;	//used only as a flag
	uint32_t Time_DuskTime;			//time passed from dusk (not stored in statsFlash, just current night)
	uint32_t ChgAs;					//charge ampere-seconds, in 0,1As
	uint32_t DschgAs;				//discharge ampere-seconds, in 0,1As
}_WSecondsStatistics;

_WSecondsStatistics StatCurrentWs, StatCountFlagsWs;

typedef struct
{
	int Indicator;					//indicator, if 0xff then cal values arent present - erased flash
	int Inv_current_off;			//Inverter current offset cal value
	int PV_current_off;				//PV current offset cal value
	int PV_voltage;					//
	int Batt_voltage;				//
	int NTC1_PCB;					//
	int NTC2_Inverter_mos;			//
	int NTC3_Battery_mos;			//
	int VrefInt;					//
	int nousedvar;					//add something here to make number of variables even
}_CalValues;

_CalValues CalibrationValues;

typedef struct
{
	uint32_t seconds;
	uint32_t minutes;
	uint32_t hours;
	uint32_t days;
}uptime_;
uptime_ Uptime;
AdcConvertVals Adc1RawReadings, Adc1Measurements;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
void StartDefaultTask(void *argument);
void Callback01(void *argument);

/* USER CODE BEGIN PFP */
static void setPWM_TIM2(TIM_HandleTypeDef, uint32_t, uint16_t, uint16_t);
void BatteryMOS_ON(void);
void BatteryMOS_OFF(void);
void InverterMOS_OFF(void);
void InverterMOS_ON(void);
void ExtOut_InvResetStart(void);
void ExtOut_InvResetStop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DelayedOn_Wait(void)
{
	//is batt OK to charge? (and discharge as well)
	if ((Adc1Measurements.Batt_voltage) < (BATT_MAX_VOLTAGE+VoltHysteresisChg)
			&& Adc1Measurements.Batt_voltage > BATT_CRITICAL_MIN_VOLTAGE)
	{//yes, ok to charge
		InverterMOS_OFF();	//INV blocked,
		BatteryMOS_ON();	//battery to power controller, if some PV still is available (at dusk), it will charge BATT (currents below ~200mA)
	}
	else
	{//not ok to charge battery
#if HW_VER > 01		//charging/discharging blocked, PV might go only to INV
		BatteryMOS_OFF();
		InverterMOS_ON();
#else
		InverterMOS_OFF();	//INV blocked, battery to power controller, if some PV still is it will charge BATT (no other choice in HW01), needed to power controller
		BatteryMOS_ON();
#endif
	}
}

void DelayedOn_On(void)
{
	InverterMOS_ON();
	BatteryMOS_ON();
}
/*function switches on inverter mosfet, according to actual config in an instant or with delay*/
void DelayedInvMosOn(void)
{
	switch (ConfigReg)
	{
	case CONFIG_MAINS_0DELAY:
		DelayedOn_On();
		break;
	case CONFIG_MAINS_1DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME1)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
		break;
	case CONFIG_MAINS_2DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME2)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
		break;
	case CONFIG_MAINS_3DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME3)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
		break;
	case CONFIG_MAINS_4DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME4)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
		break;
	case CONFIG_MAINS_5DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME5)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
		break;
	case CONFIG_MAINS_6DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME6)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
	case CONFIG_BATT_0DELAY:
		DelayedOn_On();
		break;
	case CONFIG_BATT_1DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME1)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
		break;
	case CONFIG_BATT_2DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME2)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
		break;
	case CONFIG_BATT_3DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME3)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
		break;
	case CONFIG_BATT_4DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME4)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
		break;
	case CONFIG_BATT_5DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME5)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
		break;
	case CONFIG_BATT_6DELAY:
		if (StatCurrentWs.Time_DuskTime > DUSK_TIME6)
		{
			DelayedOn_On();
		}
		else
		{
			DelayedOn_Wait();
		}
		break;
	default:
		InverterMOS_ON();
		break;
	}
}

/*reset Inverter procedure
 * reset is needed due to issue with starting Inverter from low voltage at dawn (e.g. when battery is charging)
 * to reset MPPT algorithm at least few second long voltage spike (OCV) is needed
 * to do that procedure will disconnect INV, Battery, wait few seconds and start Inverter again
 */
void ResetInverterDay(void)
{
	switch (StateResetInv)
	{
	case 0:
#if HW_VER > 01
		if (Adc1Measurements.Inv_current > INV_CURRENT_MIN && FlagExt_I == 0)
#else
		if (Adc1Measurements.Inv_current > INV_CURRENT_MIN )
#endif
		{
			TimeToResetInv = TIME2RESET_INV;	//reset time to 30 minutes if inv is working
		}
#if HW_VER > 01
		else if (FlagExt_I == 1)
		{
			TimeToResetInv = 1;
			StatCurrentWh.InvExtResetCnt++;
		}
#endif
		if (TimeToResetInv)	TimeToResetInv--;
		if (!TimeToResetInv) StateResetInv=1;	//start Inv Reset procedure
		break;
	case 1:	//30 minutes without inv currrent. set flag that procedure is ON,
		FlagResetInverter = 1;	//procedure is on
		BatteryMOS_OFF();		//switch off all loads causing OCV
		InverterMOS_OFF();
		StateResetInv = 2;
		StatCurrentWh.InvResetCntr++;
		ExtOut_InvResetStart();
		break;
	case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9: case 10:
	case 11: case 12: case 13: case 14: case 15: case 16:
		//wait 15 seconds
		StateResetInv++;
		break;
	case 17: case 18: case 19: case 20: case 21: case 22: case 23: case 24: case 25: case 26:
		//ON inverter MOS and wait 10 seconds
		InverterMOS_ON();
		StateResetInv++;
		ExtOut_InvResetStop();
		break;
	case 27:	//now return to regular operation of controller
		StateResetInv=0;
		FlagResetInverter = 0;	//procedure is off
		TimeToResetInv = TIME2RESET_INV;	//reset timer for 30 minutes
		break;
	default:
		StateResetInv=0;
		FlagResetInverter = 0;	//procedure is off
		break;
	}
}

/*reset Inverter procedure
 * reset is needed due to excessive current consumption (when batt voltage < MPP but battery can supply higher current than PV)
 * its due to mppt alg in inverter. Not needed when battery has operating voltage higher than MPP of inverter or PV panel
 * procedure will disconnect INV, wait few seconds and start Inverter again
 */
void ResetInverterNight(void)
{
	switch (StateResetInv)
	{
	case 0:
		//if (Adc1Measurements.Inv_current > INV_CURRENT_MIN &&
		if ( Adc1Measurements.Inv_current < INV_CURRENT_MAX)
		{
			TimeToResetInv = TIME2OVLD_INV;	//set overload timer for x seconds
		}
		else if (Adc1Measurements.Inv_current > INV_CURR_SC)	//considered as short-circuit
		{
			TimeToResetInv = 0;
			StatCountFlagsWs.InvOutShorted = 1;
		}
#if HW_VER > 01
		if (FlagExt_I == 1)
		{
			TimeToResetInv = 1;
			StatCurrentWh.InvExtResetCnt++;
		}
#endif
		if (TimeToResetInv)	TimeToResetInv--;
		if (!TimeToResetInv) StateResetInv=1;	//start Inv Reset procedure
		break;
	case 1:	//10 sec ovld (or no current) inv currrent. set flag that procedure is ON,
		FlagResetInverter = 1;	//procedure is on

#if HW_VER > 01
		BatteryMOS_OFF();			//disconnect load (HW02>)and..
#endif
		InverterMOS_OFF();			//disconnect load
		StateResetInv = 17;			//for regular overload wait 10 seconds
		if (StatCountFlagsWs.InvOutShorted) StateResetInv = 2;	//for short circuit wait 25 seconds
		//StatCurrentWh.InvResetCntr++;
		StatCurrentWh.InvOvcCounter++;
		ExtOut_InvResetStart();
		break;
	case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9: case 10:
	case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18:
	case 19: case 20: case 21: case 22: case 23: case 24: case 25: case 26:
		//wait 25 seconds
		StateResetInv++;
		break;
	case 27: case 28: case 29: case 30: case 31: case 32: case 33: case 34: case 35: case 36:
		//ON inverter MOS and wait 10 seconds
		InverterMOS_ON();
		BatteryMOS_ON();
		StateResetInv++;
		ExtOut_InvResetStop();
		if (Adc1Measurements.Inv_current > INV_CURR_SC) StateResetInv = 1;	//if SC occurs launch again reset procedure, instantly
		break;
	case 37:	//now return to regular operation of controller
		StatCountFlagsWs.InvOutShorted = 0;
		StateResetInv=0;
		FlagResetInverter = 0;	//procedure is off
		TimeToResetInv = TIME2OVLD_INV;	//reset timer for 30 minutes
		break;
	default:
		StateResetInv=0;
		FlagResetInverter = 0;	//procedure is off
		break;
	}
}

void setPWM_TIM2(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
{
 //HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
 TIM_OC_InitTypeDef sConfigOC;
 //timer.Init.Period = period; // set the period duration
 //HAL_TIM_PWM_Init(&timer); // re-inititialise with new period value
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = pulse; // set the pulse duration
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
 HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
}

void setPWM_TIM16(TIM_HandleTypeDef timer, uint32_t channel, uint16_t pulse)
{
 TIM_OC_InitTypeDef sConfigOC;
 sConfigOC.Pulse = pulse; // set the pulse duration
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
 sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
 HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
 /* Set the Capture Compare Register value */
 //*timer->CCR1 = pulse;
 HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
}

/*function called once a 1 second to show mashine-state status thru LEDs*/
void LedStatusShow(void)
{
	if (LedStatusTimer)
	{
		LedStatusTimer--;
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
	}
	else //once a period 1-4-10seconds (LEDSTATUS_TIMER)
	{//highest priority to show
		if (Adc1Measurements.NTC3_Battery_mos>MOSFET_MAX_TEMP || Adc1Measurements.NTC2_Inverter_mos>MOSFET_MAX_TEMP)	//powerMOSFET OVT 0
		{
			if (Adc1Measurements.NTC3_Battery_mos>MOSFET_MAX_TEMP) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			if (Adc1Measurements.NTC2_Inverter_mos>MOSFET_MAX_TEMP) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
			LedStatusTimer = LEDSTATUS_TIMER_SHORT;
		}
		else if (StatCountFlagsWs.Time_NoBattery2Chg)	//cant charge battery (its full) 1 high priority
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			LedStatusTimer = LEDSTATUS_TIMER_SHORT;
		}
		else if (StatCountFlagsWs.Ws_BattRecharge)	//battery charging (mains mode) 2
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			LedStatusTimer = LEDSTATUS_TIMER_SHORT;
		}
		else if (StatCountFlagsWs.Ws_BattOut)	//battery discharging 3
		{
			if (FlagInverterMOS && FlagBatteryMOS)
			{//battery discharging 3, inv working if both are ON
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				LedStatusTimer = LEDSTATUS_TIMER_SHORT;
			}
			else
			{//battery discharging 3, inv disconnected
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				LedStatusTimer = LEDSTATUS_TIMER_MED;
			}

		}
		else if (StatCountFlagsWs.Ws_Inverter)	//inverter operational (mains mode) 3
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			LedStatusTimer = LEDSTATUS_TIMER_LONG;
		}
		else if (StatCountFlagsWs.Ws_BattNoInv)	//battery charging, no inverter (mains mode) 4
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			LedStatusTimer = LEDSTATUS_TIMER_SHORT;
		}
		else if (StatCountFlagsWs.Ws_BattIn)	//battery charging (battery mode) 5
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			LedStatusTimer = LEDSTATUS_TIMER_SHORT;
		}
		else if (StatCountFlagsWs.Time_NightTime)	//night time, lowest priority to show 6
		{//lowest priority to show
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			LedStatusTimer = LEDSTATUS_TIMER_LONG;
		}
	}
}

/*Function restoring  saved cal data from FLASH memory */
void RestoreCalValuesFromFLASH(void)
{
int i;

	for(i = 0; i < sizeof(CalibrationValues); i++)
	{
		((uint8_t *) &CalibrationValues)[i] = Cal_savedInFLASH[i];
	}
	if (CalibrationValues.Indicator == 0xffffffff)
	{
		for(i = 0; i < sizeof(CalibrationValues); i++)
		{
			((uint8_t *) &CalibrationValues)[i] = 0;
		}
	}

}

void StoreCalData2FLASH(void)
{
	uint32_t temp, sofar=0, PageAddress=0;
	FLASH_EraseInitTypeDef flash_conf;

	osTimerStop(myTimer01Handle);
	StatCurrentWh.FlashPageCounter++;
	flash_conf.TypeErase = FLASH_TYPEERASE_PAGES;
	flash_conf.NbPages = 1;
	flash_conf.Page = NO_FLASH_PAGES+64;
	//flash_conf.Page = (uint32_t)&Stats_savedInFLASH[RecentPage_pointer][0];
	flash_conf.Banks = FLASH_BANK_1;
#ifndef DEBUG_EN
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
	HAL_FLASHEx_Erase(&flash_conf, &temp);// FLASH_Erase_Sector(&Stats_savedInFLASH+RecentPage_pointer, VOLTAGE_RANGE_3);
	//HAL_FLASH_Lock();
	//HAL_FLASH_Unlock();
	//__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );

	 while (sofar<((sizeof(CalibrationValues)/(4*2))))	//should divided by number of bytes@word wrote at once
	 {
		 if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)&Cal_savedInFLASH[PageAddress], ((uint64_t *) &CalibrationValues)[sofar]) == HAL_OK)
		 {
			 PageAddress += 8;  // use StartPageAddress += 2 for half word and 8 for double word
			 sofar++;
		 }
		 else
		 {
		   /* Error occurred while writing data in Flash memory*/
			 osTimerStart(myTimer01Handle, 100);
			 return HAL_FLASH_GetError ();
		 }
	}
#endif
	HAL_FLASH_Lock();
	osTimerStart(myTimer01Handle, 100);
}

/*Function restoring last saved statistics from FLASH memory */
void RestoreStatisticsFromFLASH(void)
{
	uint32_t i, curr_val=0, Highest_val=0;
	//Stats_savedInFLASH[NO_FLASH_PAGES][FLASH_PAGE_SIZE]
	RecentPage_pointer = 0;
	for (i = 0; i < (NO_FLASH_PAGES); i++)
		{
			for(uint32_t j=4;j>0;)
			{
				j--;
				curr_val = (curr_val<<8) | Stats_savedInFLASH[i][j];
			}
			if (curr_val == 0xffffffff)
			{
				curr_val = 0;
			}
			if (curr_val > Highest_val)
			{
				RecentPage_pointer = i;
				Highest_val = curr_val;
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			}
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		}

	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);

	for(i = 0; i < sizeof(Stat_Flash); i++)
	{
		((uint8_t *) &Stat_Flash)[i] = Stats_savedInFLASH[RecentPage_pointer][i];
	}
	if (Stat_Flash.Time_NightTime == 0xffffffff)
	{
		for(i = 0; i < sizeof(Stat_Flash); i++) {
			    ((uint8_t *) &StatCurrentWh)[i] = 0;
			    ((uint8_t *) &Stat_Flash)[i] = 0;
			}
	}
	else StatCurrentWh = Stat_Flash;
}

/*Function storing last  statistics to FLASH memory */
void StoreStatistics2FLASH(void)
{
	uint32_t temp, sofar=0, StartPageAddress=0;
	FLASH_EraseInitTypeDef flash_conf;

	osTimerStop(myTimer01Handle);
	Stat_Flash = StatCurrentWh;
	if (RecentPage_pointer < NO_FLASH_PAGES-1) RecentPage_pointer++;
	else RecentPage_pointer = 0;
	StatCurrentWh.FlashPageCounter++;
//	return;
	flash_conf.TypeErase = FLASH_TYPEERASE_PAGES;
	flash_conf.NbPages = 1;
	flash_conf.Page = RecentPage_pointer + 64;
	//flash_conf.Page = (uint32_t)&Stats_savedInFLASH[RecentPage_pointer][0];
	flash_conf.Banks = FLASH_BANK_1;
#ifndef DEBUG_EN
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
	HAL_FLASHEx_Erase(&flash_conf, &temp);// FLASH_Erase_Sector(&Stats_savedInFLASH+RecentPage_pointer, VOLTAGE_RANGE_3);
	//HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, (uint32_t)&Stats_savedInFLASH[RecentPage_pointer][0], ((uint8_t *) &Stat_Flash)[i]);
	HAL_FLASH_Lock();
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );

	 while (sofar<((sizeof(Stat_Flash)/(4*2))))	//should divided by number of bytes@word wrote at once
		   {
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)&Stats_savedInFLASH[RecentPage_pointer][StartPageAddress], ((uint64_t *) &Stat_Flash)[sofar]) == HAL_OK)
		     {
		    	 StartPageAddress += 8;  // use StartPageAddress += 2 for half word and 8 for double word
		    	 sofar++;
		     }
		     else
		     {
		       /* Error occurred while writing data in Flash memory*/
		    	 osTimerStart(myTimer01Handle, 100);
		    	 return HAL_FLASH_GetError ();
		     }
		   }
#endif
	HAL_FLASH_Lock();
	osTimerStart(myTimer01Handle, 100);
}
/*Function storing last  statistics to FLASH memory */
void DeleteStatistics2FLASH(void)
{
	uint32_t temp, i;
	FLASH_EraseInitTypeDef flash_conf;

	osTimerStop(myTimer01Handle);
	for(i = 0; i < sizeof(Stat_Flash); i++)
	{
		((uint8_t *) &StatCurrentWh)[i] = 0;
		((uint8_t *) &Stat_Flash)[i] = 0;
	}
	RecentPage_pointer = 0;
	flash_conf.TypeErase = FLASH_TYPEERASE_PAGES;
	flash_conf.NbPages = 1;
	flash_conf.Page = RecentPage_pointer + 64;
	flash_conf.Banks = FLASH_BANK_1;
#ifndef DEBUG_EN
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
	while (RecentPage_pointer < NO_FLASH_PAGES-1)
	{
		HAL_FLASHEx_Erase(&flash_conf, &temp);
		RecentPage_pointer++;
		flash_conf.Page = RecentPage_pointer + 64;
	}

#endif
	HAL_FLASH_Lock();
	RecentPage_pointer = 0;
	osTimerStart(myTimer01Handle, 100);
}
/*function launched every second to calculate Watt-seconds for given machine state on basis of flags */
void Calculate_WattSeconds(void)
{
#ifndef TESTING_VALUES
	if (StatCountFlagsWs.Ws_BattIn)		//Watt-seconds when battery is charging
	{
		StatCountFlagsWs.Ws_BattIn = 0;
		StatCurrentWs.Ws_BattIn += (Adc1Measurements.Batt_voltage*Adc1Measurements.PV_current)/10000;
	}
	if (StatCountFlagsWs.Ws_BattNoInv)		//Watt-seconds when battery is charging but energy should go into mains (but cant)
	{
		StatCountFlagsWs.Ws_BattNoInv = 0;
		StatCurrentWs.Ws_BattNoInv += (Adc1Measurements.Batt_voltage*Adc1Measurements.PV_current)/10000;
		//StatCurrentWs.Time_NoInv++;	//double count
	}
	if (StatCountFlagsWs.Ws_Inverter)		//Watt-seconds for inverter
	{
		StatCountFlagsWs.Ws_Inverter = 0;
		StatCurrentWs.Ws_Inverter += (Adc1Measurements.PV_voltage*Adc1Measurements.Inv_current)/10000;
	}
	if (StatCountFlagsWs.Ws_BattOut)		//Watt-seconds when battery is discharging
	{
		StatCountFlagsWs.Ws_BattOut = 0;
		StatCurrentWs.Ws_BattOut += (Adc1Measurements.Batt_voltage*Adc1Measurements.Inv_current)/10000;
	}
	if (StatCountFlagsWs.Ws_BattRecharge)		//Watt-seconds when battery is recharging
	{
		StatCountFlagsWs.Ws_BattRecharge = 0;
		StatCurrentWs.Ws_BattRecharge += (Adc1Measurements.Batt_voltage*Adc1Measurements.PV_current)/10000;
	}
	if (StatCountFlagsWs.Time_NightTime)		//darkness time
	{
		StatCountFlagsWs.Time_NightTime = 0;
		StatCurrentWs.Time_NightTime++;
	}
	if (StatCountFlagsWs.Time_NoBattery2Chg)		//time when battery is not ready to be charged
	{
		StatCountFlagsWs.Time_NoBattery2Chg = 0;
		StatCurrentWs.Time_NoBattery2Chg++;
	}
	if (StatCountFlagsWs.Time_NoInv)		//time when INV is not working
	{
		StatCountFlagsWs.Time_NoInv = 0;
		StatCurrentWs.Time_NoInv++;
	}
	if (StatCountFlagsWs.Time_DuskTime)		//time after dusk
	{
		StatCountFlagsWs.Time_DuskTime = 0;
		StatCurrentWs.Time_DuskTime++;
	}
	if (StatCountFlagsWs.Time_Daytime)		//daytime
	{
		StatCountFlagsWs.Time_Daytime=0;
		StatCurrentWh.DayDuration_current++;
	}
	if (StatCountFlagsWs.ChgAs)		//count mAs
	{
		StatCountFlagsWs.ChgAs = 0;
		StatCurrentWs.ChgAs += (Adc1Measurements.PV_current);
	}
	if (StatCountFlagsWs.DschgAs)		//count mAs
	{
		StatCountFlagsWs.DschgAs = 0;
		StatCurrentWs.DschgAs += (Adc1Measurements.Inv_current);
	}
#else		//calculate really big numbers to overload printf
	if (StatCountFlagsWs.Ws_BattIn)		//Watt-seconds when battery is charging
	{
		StatCountFlagsWs.Ws_BattIn = 0;
		StatCurrentWs.Ws_BattIn += (Adc1Measurements.Batt_voltage*Adc1Measurements.PV_current);
	}
	if (StatCountFlagsWs.Ws_BattNoInv)		//Watt-seconds when battery is charging but energy should go into mains (but cant)
	{
		StatCountFlagsWs.Ws_BattNoInv = 0;
		StatCurrentWs.Ws_BattNoInv += (Adc1Measurements.Batt_voltage*Adc1Measurements.PV_current);
		//StatCurrentWs.Time_NoInv++;	//double count
	}
	if (StatCountFlagsWs.Ws_Inverter)		//Watt-seconds for inverter
	{
		StatCountFlagsWs.Ws_Inverter = 0;
		StatCurrentWs.Ws_Inverter += (Adc1Measurements.PV_voltage*Adc1Measurements.Inv_current);
	}
	if (StatCountFlagsWs.Ws_BattOut)		//Watt-seconds when battery is discharging
	{
		StatCountFlagsWs.Ws_BattOut = 0;
		StatCurrentWs.Ws_BattOut += (Adc1Measurements.Batt_voltage*Adc1Measurements.Inv_current);
	}
	if (StatCountFlagsWs.Ws_BattRecharge)		//Watt-seconds when battery is recharging
	{
		StatCountFlagsWs.Ws_BattRecharge = 0;
		StatCurrentWs.Ws_BattRecharge += (Adc1Measurements.Batt_voltage*Adc1Measurements.PV_current);
	}
	if (StatCountFlagsWs.Time_NightTime)		//darkness time
	{
		StatCountFlagsWs.Time_NightTime = 0;
		StatCurrentWs.Time_NightTime+=1000;
	}
	if (StatCountFlagsWs.Time_NoBattery2Chg)		//time when battery is not ready to be charged
	{
		StatCountFlagsWs.Time_NoBattery2Chg = 0;
		StatCurrentWs.Time_NoBattery2Chg+=1000;
	}
	if (StatCountFlagsWs.Time_NoInv)		//time when INV is not working
	{
		StatCountFlagsWs.Time_NoInv = 0;
		StatCurrentWs.Time_NoInv+=1000;
	}
	if (StatCountFlagsWs.Time_DuskTime)		//time after dusk
	{
		StatCountFlagsWs.Time_DuskTime = 0;
		StatCurrentWs.Time_DuskTime++;
	}
	if (StatCountFlagsWs.ChgAs)		//count mAs
	{
		StatCountFlagsWs.ChgAs = 0;
		StatCurrentWs.ChgAs += (Adc1Measurements.PV_current);
	}
	if (StatCountFlagsWs.DschgAs)		//count mAs
	{
		StatCountFlagsWs.DschgAs = 0;
		StatCurrentWs.DschgAs += (Adc1Measurements.PV_current)
	}
#endif
	if (Adc1Measurements.NTC2_Inverter_mos >= StatCurrentWh.MaxTempInvMos)
	{
		StatCurrentWh.MaxTempInvMos = Adc1Measurements.NTC2_Inverter_mos;
		StatCurrentWh.MaxTempInvMosCntr++;
	}
	if (Adc1Measurements.NTC3_Battery_mos >= StatCurrentWh.MaxTempBatMos)
	{
		StatCurrentWh.MaxTempBatMos = Adc1Measurements.NTC3_Battery_mos;
		StatCurrentWh.MaxTempBatMosCntr++;
	}
	if (Adc1Measurements.Inv_current >= StatCurrentWh.MaxInvCurrent)
	{
		StatCurrentWh.MaxInvCurrent = Adc1Measurements.Inv_current;
		StatCurrentWh.MaxInvCurrentCntr++;
	}
	if (Adc1Measurements.PV_current >= StatCurrentWh.MaxPVCurrent)
	{
		StatCurrentWh.MaxPVCurrent = Adc1Measurements.PV_current;
		StatCurrentWh.MaxPVCurrentCntr++;
	}
	if (Adc1Measurements.Batt_voltage >= StatCurrentWh.MaxBatVoltage)
	{
		StatCurrentWh.MaxBatVoltage = Adc1Measurements.Batt_voltage;
		StatCurrentWh.MaxBatVoltageCntr++;
	}
	if (Adc1Measurements.Batt_voltage > BATT_CRITICAL_MIN_VOLTAGE)
	{
		if (StatCurrentWh.MinBatVoltage <= BATT_CRITICAL_MIN_VOLTAGE)	//first launch value is "0", this 'if' is to cover that option
		{
			StatCurrentWh.MinBatVoltage = Adc1Measurements.Batt_voltage;
			StatCurrentWh.MinBatVoltageCntr++;
		}
		else if (Adc1Measurements.Batt_voltage <= StatCurrentWh.MinBatVoltage)
		{
			StatCurrentWh.MinBatVoltage = Adc1Measurements.Batt_voltage;
			StatCurrentWh.MinBatVoltageCntr++;
		}
	}
}

/*function launched every hour to calculate Watt-hours watt-seconds hourly statistics */
void Calculate_WattHours(void)
{
	StatCurrentWh.Wh_BattIn=+StatCurrentWs.Ws_BattIn/SECONDS_HOUR;
	StatCurrentWh.Wh_BattNoInv+=StatCurrentWs.Ws_BattNoInv/SECONDS_HOUR;
	StatCurrentWh.Time_NoInv+=StatCurrentWs.Time_NoInv;
	StatCurrentWh.Wh_Inverter+=StatCurrentWs.Ws_Inverter/SECONDS_HOUR;
	StatCurrentWh.Wh_BattOut+=StatCurrentWs.Ws_BattOut/SECONDS_HOUR;
	StatCurrentWh.Wh_BattRecharge+=StatCurrentWs.Ws_BattRecharge/SECONDS_HOUR;
	StatCurrentWh.Chg_Ah_current+=StatCurrentWs.ChgAs/SECONDS_HOUR;
	//StatCurrentWh.Chg_Ah_current=StatCurrentWh.Chg_Ah_current/1000;	//because current is stored in 0,001A
	StatCurrentWh.Dschg_Ah_current+=StatCurrentWs.DschgAs/SECONDS_HOUR;
	//StatCurrentWh.Dschg_Ah_current=StatCurrentWh.Dschg_Ah_current/1000;	//because current is stored in 0,001A
	StatCurrentWh.Time_NightTime+=StatCurrentWs.Time_NightTime;
	StatCurrentWh.Time_NoBattery2Chg+=StatCurrentWs.Time_NoBattery2Chg;
	StatCurrentWs.ChgAs=0;
	StatCurrentWs.DschgAs=0;
	StatCurrentWs.Ws_BattIn=0;
	StatCurrentWs.Ws_BattNoInv=0;
	StatCurrentWs.Time_NoInv=0;
	StatCurrentWs.Ws_Inverter=0;
	StatCurrentWs.Ws_BattOut=0;
	StatCurrentWs.Ws_BattRecharge=0;
	StatCurrentWs.Time_NightTime=0;
	StatCurrentWs.Time_NoBattery2Chg=0;
}
void ReadConfig(void)
{
#if HW_VER > 01
	ConfigReg = HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin)<<0;	//LSB, config for EECO TDR-16
	ConfigReg += HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)<<1;
	ConfigReg += HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)<<2;
	ConfigReg += HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)<<3;	//MSB
	ConfigReg = ~ConfigReg;
	ConfigReg = ConfigReg &0x0f;
#else
	ConfigReg = HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin)<<0;	//LSB, config for EECO 330041GS
	ConfigReg += HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)<<1;
	ConfigReg += HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)<<2;
	ConfigReg += HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)<<3;	//MSB
#endif
}
#if HW_VER > 01
void BackupPowerON(void)
{
uint32_t TxSize;
	if (FlagBackupMOS) return;
	FlagBackupMOS = 1;
	HAL_GPIO_WritePin(BATT_BCKP_DRV_GPIO_Port, BATT_BCKP_DRV_Pin, 1);	//turn on battery backup power: mosfet ON
    sprintf(TxBuffer, "BackupPwrON\r\n");
    TxSize = strlen(TxBuffer);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, TxSize);
    osDelay(10);	//just to empty usart buffer
}
void BackupPowerOFF(void)
{
uint32_t TxSize;
	if (!FlagBackupMOS) return;
    FlagBackupMOS = 0;
    HAL_GPIO_WritePin(BATT_BCKP_DRV_GPIO_Port, BATT_BCKP_DRV_Pin, 0);	//turn off battery backup power: mosfet ON
    sprintf(TxBuffer, "*** BackupPwrOFF ***\r\n");
	TxSize = strlen(TxBuffer);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, TxSize);
	osDelay(100);
}
#endif
//generate signal to inform other controllers: "INV reset procedure is in progress"
void ExtOut_InvResetStart(void)
{
	setPWM_TIM16(htim16, TIM_CHANNEL_1, 200);		//500 ->50% @ 4kHz, 50 @ 33khz, 100 @ 16khz, 200 @ 8khz
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = EXT_I_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;			//prevent excessive voltage on EXT_I by pulldown
	  HAL_GPIO_Init(EXT_I_GPIO_Port, &GPIO_InitStruct);
}

void ExtOut_InvResetStop(void)
{
	setPWM_TIM16(htim16, TIM_CHANNEL_1, 0);		//0% @ 4kHz
	HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = EXT_I_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
#if HW_VER > 01
	  GPIO_InitStruct.Pull = GPIO_NOPULL;			//disable pulldown
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;			//apply pulldown - in humid environment it oftyen it sets this input without real signal coming in. for testing purposes
#else
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;			//prevent excessive voltage on EXT_I by pulldown
#endif
	  HAL_GPIO_Init(EXT_I_GPIO_Port, &GPIO_InitStruct);
}

void BatteryMOS_ON(void)
{

	if (Adc1Measurements.NTC3_Battery_mos>MOSFET_MAX_TEMP)
	{
		BatteryMOS_OFF();
	}
	else
	{
		if (FlagBatteryMOS) return;
		HAL_GPIO_WritePin(BAT_SWITCH_OFF_GPIO_Port, BAT_SWITCH_OFF_Pin, 0);	//disable powerMosfet pulldown
		setPWM_TIM2(htim2, TIM_CHANNEL_1, 254, 500);		//500->50% @ 4kHz
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		FlagBatteryMOS = 1;
	}
}

void BatteryMOS_OFF(void)
{
	setPWM_TIM2(htim2, TIM_CHANNEL_1, 254, 0);
	HAL_GPIO_WritePin(BAT_SWITCH_OFF_GPIO_Port, BAT_SWITCH_OFF_Pin, 1);	//enable powerMosfet pulldown
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	FlagBatteryMOS = 0;
}

void InverterMOS_ON(void)
{
	if (Adc1Measurements.NTC2_Inverter_mos > MOSFET_MAX_TEMP)
	{
		InverterMOS_OFF();
	}
	else
	{
		HAL_GPIO_WritePin(INV_SWITCH_DRV_GPIO_Port, INV_SWITCH_DRV_Pin, 0);	//disable powerMosfet pulldown
		FlagInverterMOS = 1;
	}
}

void InverterMOS_OFF(void)
{
	HAL_GPIO_WritePin(INV_SWITCH_DRV_GPIO_Port, INV_SWITCH_DRV_Pin, 1);	//enable powerMosfet pulldown
	FlagInverterMOS = 0;
}

uint32_t ConvertNTCvalue(uint32_t RawReading)
{
double i, y;
uint32_t t;
	      i = (double) RawReading;	//
	      //y = -3*pow(10,-15)*pow(i,5) + 3*pow(10,-11)*pow(i,4)-pow(10,-7)*pow(i,3)+0.0002*pow(i,2)-0.2204*i+138.71;
	      y = -7*pow(10,-9)*pow(i,3) + 5*pow(10,-5)*pow(i,2)-0.1154*i+123.68;
	      t = (uint32_t) y;
	      t = (t - 13);				// some offset removal
	      return t;	//result in *C
}

uint32_t ConvertVValue(uint32_t RawReading)
{
    double i;
    i = (((float)RawReading))*100/4095;
    i = i*(float)3.29;					//ref voltage value
    i = i*(float)(470+16)/(float)16; //resistor divider R1+R2/R1
    //i *= (10.5/10.0)/10;					//coefficient due to tolerances, //without adc internal calibration
    i *= (10.0/10.0)/10;					//coefficient due to tolerances, //witht adc internal calibration
    return (uint32_t) i;			//result in 100mV
}

uint32_t ConvertIValue(uint32_t RawReading)
{	//xls equation from 120 gain and 2mR shunt: y = 0,2954x - 36,931 -> x=10000/2954*y + 36.931
    double i;
    //simple way
//    i = (((float)RawReading));
//    i *= (43/87.0)*10;			//43 is 430mA current, 87 is ADC value
#if HW_VER > 01
//without adc internal calibration
    //from test, 2,6A over ADC reading
//    i = ((2650/654)*(float)RawReading);
//    i = i + 50.1;
//with ADC internal calibration
    i = (double)(3824)*(double)RawReading;
    i = i/1000;
    i = i + 25;
#else
    //from XLS trendline
    i = ((10000/2510)*(float)RawReading);
    i = i + 38.1;
#endif
    if (i<0) i=0;
    return (uint32_t) i;	//result in 100mA
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
#if HW_VER > 01
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = EXT_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT_I_GPIO_Port, &GPIO_InitStruct);
#endif
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 500;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 254;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BATT_BCKP_DRV_Pin|BAT_SWITCH_OFF_Pin|INV_SWITCH_DRV_Pin|MEAS_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BATT_BCKP_DRV_Pin BAT_SWITCH_OFF_Pin INV_SWITCH_DRV_Pin MEAS_PWR_Pin */
  GPIO_InitStruct.Pin = BATT_BCKP_DRV_Pin|BAT_SWITCH_OFF_Pin|INV_SWITCH_DRV_Pin|MEAS_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C3_Pin C1_Pin */
  GPIO_InitStruct.Pin = C3_Pin|C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : C4_Pin C2_Pin */
  GPIO_InitStruct.Pin = C4_Pin|C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CD_Pin EXT_I_Pin */
  GPIO_InitStruct.Pin = CD_Pin|EXT_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// UART Rx Complete Callback;
	// Rx Complete is called by: DMA (automatically), if it rolls over
	// and when an IDLE Interrupt occurs
	// DMA Interrupt allays occurs BEFORE the idle interrupt can be fired because
	// idle detection needs at least one UART clock to detect the bus is idle. So
	// in the case, that the transmission length is one full buffer length
	// and the start buffer pointer is at 0, it will be also 0 at the end of the
	// transmission. In this case the DMA rollover will increment the RxRollover
	// variable first and len will not be zero.
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {									// Check if it is an "Idle Interrupt"
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);												// clear the interrupt
		RxCounter++;																	// increment the Rx Counter
		uint8_t TxSize = 0;
		uint16_t start = RxBfrPos;														// Rx bytes start position (=last buffer position)
		RxBfrPos = RX_BFR_SIZE - (uint16_t)huart->hdmarx->Instance->CNDTR;				// determine actual buffer position
		uint16_t len = RX_BFR_SIZE;														// init len with max. size

		if(RxRollover < 2)  {
			if(RxRollover) {															// rolled over once
				if(RxBfrPos <= start) len = RxBfrPos + RX_BFR_SIZE - start;				// no bytes overwritten
				else len = RX_BFR_SIZE + 1;												// bytes overwritten error
			} else {
				len = RxBfrPos - start;													// no bytes overwritten
			}
		} else {
			len = RX_BFR_SIZE + 2;														// dual rollover error
		}

		if(len && (len <= RX_BFR_SIZE)) {
			// create response message
//			sprintf(TxBuffer, "ACK RxC:%d S:%d L:%d RO:%d RXp:%d >>", RxCounter, start, len, RxRollover, RxBfrPos);
//			TxSize = strlen(TxBuffer);
			// add received bytes to TxBuffer
//			uint8_t i;
//			for(i = 0; i < len; i++) *(TxBuffer + TxSize + i) = *(RxBuffer + ((start + i) % RX_BFR_SIZE));
//			TxSize += i;
			if (RxBuffer[start] == 'f' || RxBuffer[start] == 'F') Flag_ShowStats = 4;	//show statistics from Flash
			if (RxBuffer[start] == 'c' || RxBuffer[start] == 'C') Flag_ShowStats = 5;	//show current statistics
			if (RxBuffer[start] == 'w' || RxBuffer[start] == 'W') Flag_ShowStats = 3;	//show WattHours statistics
			if (RxBuffer[start] == 'e' || RxBuffer[start] == 'E')						//erase statistics in flash - only newest
			{
				uint32_t i, temp;
				temp = StatCurrentWh.FlashPageCounter;
				for(i = 0; i < sizeof(Stat_Flash); i++)		//clear statistics that will be stored in flash at top of the hour
				{
					((uint8_t *) &StatCurrentWh)[i] = 0;
					((uint8_t *) &Stat_Flash)[i] = 0;
				}
				StatCurrentWh.FlashPageCounter = temp;
				Stat_Flash.FlashPageCounter = temp;
				Flag_ShowStats = 4;
			}
			if (RxBuffer[start] == 'd' || RxBuffer[start] == 'D')						//delete current and historical statistics stored in flash
			{
				DeleteStatistics2FLASH();
				Flag_ShowStats = 4;
			}
			if (RxBuffer[start] == 'p' || RxBuffer[start] == 'P')						//show calibration data
			{
				Flag_ShowStats = 62;
			}
			if (RxBuffer[start] == 'o' || RxBuffer[start] == 'O')						//read calibration data from flash
			{
				Flag_ShowStats = 62;
				RestoreCalValuesFromFLASH();
			}
			if (RxBuffer[start] == 'l' || RxBuffer[start] == 'L')						//save calibration data
			{
				StoreCalData2FLASH();
				Flag_ShowStats = 62;
			}
			if (RxBuffer[start] == 'k' || RxBuffer[start] == 'K')						//save calibration data
			{
				Flag_ShowStats = 62;
				for(int i = 0; i < sizeof(CalibrationValues); i++)
				{
					((uint8_t *) &CalibrationValues)[i] = 0;
				}
			}
			if (RxBuffer[start] == '0')													//calibrate inverter "0" current
			{
				Flag_ShowStats = 62;
				uint32_t temp;
				temp = 	ConvertIValue(Adc1RawReadings.Inv_current);
				if (temp < 500)
				{
					CalibrationValues.Inv_current_off = (int)temp;
					sprintf(TxBuffer, "\r\nInverter current offset: %i\r\n",( int)temp);
				}
				else
				{
					sprintf(TxBuffer, "\r\nInverter Zero current too high: %i\r\n",( int)Adc1Measurements.Inv_current);
				}
			}
			if (RxBuffer[start] == '1')													//calibrate PV "0" current
			{
				Flag_ShowStats = 62;
				uint32_t temp;
				temp = 	ConvertIValue(Adc1RawReadings.PV_current);
				if (temp < 500)
				{
					CalibrationValues.PV_current_off = (int)temp;
					sprintf(TxBuffer, "\r\nPV current offset: %i\r\n",( int)temp);
				}
				else
				{
					sprintf(TxBuffer, "\r\nPV Zero current too high: %i\r\n",( int)Adc1Measurements.PV_current);
				}
			}
			if (RxBuffer[start] == 'h' || RxBuffer[start] == 'H' || RxBuffer[start] == '?')
			{
				Flag_ShowStats = 60;
				sprintf(TxBuffer, "\r\nhelp: \r\n"
						"H - show this help\r\n"
						"C - show configuration and firmware version\r\n"
						"F - show last statistics stored in Flash mem\r\n"
						"E - erase current stat values stored in flash, will be saved at top of the hour. FlashPageCounter will be preserved, historical data preserved\r\n"
						"D - Delete all stat data saved in flash, current and historical, instantly, !irreversible!\r\n"
						"W - show Watt-Hours stat\r\n"
						"0 - calibrate 'zero' current level for Inverter\r\n"
						"1 - calibrate 'zero' current level for PV\r\n"
						"P - show calibration values\r\n"
						"O - read calibration values from Flash\r\n"
						"K - clear calibration values (not saved)\r\n"
						"L - save calibration values\r\n"
						"\r\n"
						);
			}

		} else {
			// buffer overflow error:
//			sprintf(TxBuffer, "NAK RX BUFFER OVERFLOW ERROR %d\r\n", (len - RX_BFR_SIZE));
//			TxSize = strlen(TxBuffer);
			if(HAL_UART_GetError(&huart1)) {	//clear RX errors if occured
			        HAL_UART_DMAStop(&huart1);                          // STOP Uart
			        MX_USART1_UART_Init();                              // INIT Uart
			        HAL_UART_Receive_DMA(&huart1, RxBuffer, RX_BFR_SIZE);  // START Uart DMA
			        __HAL_UART_CLEAR_IDLEFLAG(&huart1);                 // Clear Idle IT-Flag
			        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);        // Enable Idle Interrupt
			   }
		}

//		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, TxSize);						// send a response

		RxRollover = 0;																	// reset the Rollover variable
	} else {
		// no idle flag? --> DMA rollover occurred
		RxRollover++;		// increment Rollover Counter
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	// UART Tx Complete Callback;
	TxCounter++;
}

void InverterOn_batteryAsBackup(void)
{
	if (StatCurrentWh.DayDuration_current > 900)	//15 minutes in the day/after dawn
	{
		StatCurrentWs.Time_DuskTime = 0;		//clear "after dusk timer" - it's day
		StatCountFlagsWs.ChgStatSaved = 0;		//enable saving daytime stats at dusk
	}
	StatCountFlagsWs.EmptyBattStatsSaved = 0;	//clear flag to enable saving stats at the end of discharge at night
					//charge battery slightly?
					if (((Adc1Measurements.Batt_voltage) < (BATT_MIN_VOLTAGE-BATT_VOLTAGE_MINHYSTERESIS+VoltHysteresisDsChg))  &&
							((Adc1Measurements.Batt_voltage) > BATT_CRITICAL_MIN_VOLTAGE) &&
							StatCurrentWs.Ws_BattRecharge < TOUT_BATTRECHARGE)	//prevent too long recharging (be carefoul, cleared every hour)
					{//yes, re-charge battery
						BatteryMOS_ON();
						VoltHysteresisDsChg = BATT_VOLTAGE_MINHYSTERESIS;	//hysteresis for min operation batery voltage
						InverterMOS_OFF();	//tbd
						StatCountFlagsWs.Ws_BattRecharge=1;	//enable to count energy supplied for recharging, in 1Sectimer; will be cleared automatically
						StatCountFlagsWs.Time_BattRecharge=1;	//set flag to prevent INV mos ON
						StatCountFlagsWs.ChgAs=1;		//enable counting ampere-seconds
					}
					else
					{//no need to recharge battery, check what else you can do
						//prevent BatMosOff when no inv operation; MosOff only once when exiting from batt recharge
						if (StatCountFlagsWs.Time_BattRecharge) BatteryMOS_OFF();
						VoltHysteresisDsChg = 0;
						StatCountFlagsWs.Time_BattRecharge=0;	//reset flag to enable INV mos ON and disable counting time recharge

						//switch INV ON only if not battery recharge AND Inverter reset procedure isnt in progress
						//if (!StatCountFlagsWs.Time_BattRecharge
						//		&& !FlagResetInverter) InverterMOS_ON();
						InverterMOS_ON();
						//is inverter working?
						if (Adc1Measurements.Inv_current > INV_CURRENT_MIN)
						{//yes, inverter working
							StatCountFlagsWs.Ws_Inverter=1;	//inverter working, enable to count energy, in 1Sectimer
							BatteryMOS_OFF();		//it's day, inv working-> switch off battery mos
							StatCountFlagsWs.InvFault = 0;	//flag to clear invfault occurence
						}
						else
						{//it's day, inv is not working
							//count once inverter fault condition
							if (!StatCountFlagsWs.InvFault)
								{
									StatCurrentWh.InvFaultCntr++;
									StatCountFlagsWs.InvFault = 1;
								}
							StatCountFlagsWs.Time_NoInv=1;	//enable to count time when inv is not working, in 1Sectimer
							ResetInverterDay();	//try reset inv
							if (!FlagResetInverter ) 			//if INV Reset procedure is not launched
							{
								//is batt OK to charge?
								if ((Adc1Measurements.Batt_voltage) < (BATT_MAX_VOLTAGE+VoltHysteresisChg)
										&& Adc1Measurements.Batt_voltage > BATT_CRITICAL_MIN_VOLTAGE)
								{//yes, ok to charge
									BatteryMOS_ON();
									StatCountFlagsWs.Ws_BattNoInv=1;	//enable to count energy to battery when INV isnt working, in 1Sectimer
									StatCountFlagsWs.ChgAs=1;		//enable counting ampere-seconds
									VoltHysteresisChg = BATT_VOLTAGE_MAXHYSTERESIS;
									StatCountFlagsWs.Chg_cycle_count = 0;	//when battery charging unlock flag enabling counting dschg cycles
								}
								else
								{//no, batt not OK to charge
									BatteryMOS_OFF();
									StatCountFlagsWs.Time_NoBattery2Chg=1;	//enable to count time without possibility to charge battery, in 1Sectimer
									VoltHysteresisChg = 0;
									if ((Adc1Measurements.Batt_voltage) > (BATT_MAX_VOLTAGE+VoltHysteresisChg))
									{//if battery fully charged, check if it was moment ago.
										StatCurrentWh.Chg_Ah_lastFull=StatCurrentWh.Chg_Ah_current;	//store fully chg Ah
										if (!StatCountFlagsWs.Chg_cycle_c2 && StatCountFlagsWs.Chg_cycle_count)
										{
											StatCurrentWh.Chg_Volt_lastFull = Adc1Measurements.Batt_voltage;	//store lastvoltage
											StatCountFlagsWs.Chg_cycle_c2 = 1;
										}
										if (!StatCountFlagsWs.Chg_cycle_count)
										{
											StatCountFlagsWs.Chg_cycle_count = 1; //if not, set flag 'fully charged' to count/set only once

											StatCurrentWh.Dschg_Ah_current=0;	//clear discharge Ah
	#if FLASHSTATSAVE_PERIOD == 1	//FLASHSTATSAVE_PERIOD 1-only when batt fully charged and discharged
									Flag_StoreStatistics = 1;
	#endif
										}
									}//end of battery fully charged
								}//end of batt not OK to charge
							}//end of inv reset  procedure not launched
						}//end of its day, inv not working
					}//end of no need to recharge battery
}

void DischargeProcedure(void)
{
	//VoltHysteresisChg = 0;		//leave Chg hysteresis elevated for transition period during dusk
	StatCountFlagsWs.Time_BattRecharge=0;	//you cant recharge during night, reset procedure
	StatCountFlagsWs.Time_NightTime=1;	//its nighttime, enable to count nightime in 1Sectimer
	StatCountFlagsWs.Time_DuskTime=1;	//its nightitme, enable flag to count time passed from recent dusk;
	if (StatCurrentWs.Time_DuskTime > 900 && !StatCountFlagsWs.ChgStatSaved)	//15 minutes in the night/after dusk store Chg_Ah_last
	{
		StatCountFlagsWs.ChgStatSaved = 1;			//allow saving daytime stats at dusk only once
		StatCurrentWh.Chg_Ah_3 = StatCurrentWh.Chg_Ah_2;
		StatCurrentWh.Chg_Ah_2 = StatCurrentWh.Chg_Ah_1;
		StatCurrentWh.Chg_Ah_1 = StatCurrentWh.Chg_Ah_last;
		StatCurrentWh.Chg_Ah_last = StatCurrentWh.Chg_Ah_current;
		StatCurrentWh.DayDuration_3 = StatCurrentWh.DayDuration_2;
		StatCurrentWh.DayDuration_2 = StatCurrentWh.DayDuration_1;
		StatCurrentWh.DayDuration_1 = StatCurrentWh.DayDuration_current;
		StatCurrentWh.DayDuration_current = 0;
	}
	//batt OK to discharge?
	if (Adc1Measurements.Batt_voltage > (BATT_MIN_VOLTAGE-VoltHysteresisDsChg))
	{//yes, OK to discharge
		StatCountFlagsWs.Dschg_cycle_c2 = 0;		//flag enabling storage dsch stat when batt depleted
		StatCountFlagsWs.Dschg_cycle_count = 0;		//clear flag to enable dschg counter when batt empty
		if (!FlagResetInverter)
		{//if reset procedure isn't in progress, turn on batt mos and inv mos
			//BatteryMOS_ON();
			DelayedInvMosOn();
		}
		ResetInverterNight();
		VoltHysteresisDsChg = BATT_VOLTAGE_MINHYSTERESIS;	//hysteresis for discharge
		if (FlagInverterMOS)
		{
			StatCountFlagsWs.Ws_BattOut=1;	//enable to count energy taken from battery in 1Sectimer
			StatCountFlagsWs.DschgAs=1;		//enable counting ampere-seconds
		}
		if (Adc1Measurements.Inv_current > INV_CURRENT_MIN)
			if (StatCountFlagsWs.Chg_cycle_count)
			{	//if battery was fully charged before night, increment counter of charge cycles as night starts
				StatCountFlagsWs.Chg_cycle_count=0;
				StatCountFlagsWs.Chg_cycle_c2=0;
				StatCurrentWh.Chg_cycle_count++;
			}
	}
	else
	{//its night and not OK to discharge (battery fully discharged)
		VoltHysteresisDsChg = 0;
		VoltHysteresisChg = 0;		//most likely will be cleared by uP reset anyway
		if (!StatCountFlagsWs.Dschg_cycle_c2 && StatCountFlagsWs.Dschg_cycle_count)
		{	//perform that only once at first enter to that procedure
			StatCurrentWh.Dschg_Ah_lastFull_3 = StatCurrentWh.Dschg_Ah_lastFull_2;
			StatCurrentWh.Dschg_Ah_lastFull_2 = StatCurrentWh.Dschg_Ah_lastFull_1;
			StatCurrentWh.Dschg_Ah_lastFull_1 = StatCurrentWh.Dschg_Ah_lastFull;
			StatCurrentWh.Dschg_Ah_lastFull = StatCurrentWh.Dschg_Ah_current;	//store fully dschg Ah
			StatCurrentWh.Dschg_Volt_lastFull = Adc1Measurements.Batt_voltage;
			StatCountFlagsWs.Dschg_cycle_c2 = 1;
			StatCountFlagsWs.Chg_cycle_c2 = 0;
#if FLASHSTATSAVE_PERIOD == 1	//FLASHSTATSAVE_PERIOD 1-when batt fully charged and discharged
					Flag_StoreStatistics = 1;
#endif
		}
		if (!StatCountFlagsWs.Dschg_cycle_count)	//if battery wasn't fully discharged before, but now it is; do that only once at full discharge
		{	//increase full dschrg counter and set flag "dscharged"
			StatCountFlagsWs.Dschg_cycle_count = 1;	//to count only once
			StatCurrentWh.Dschg_cycle_count++;	//count only once
			StatCountFlagsWs.Chg_cycle_count = 0;	//most likely will be cleared by system reset
			StatCurrentWh.Chg_Ah_current=0;	//clear charge Ah, likely ill be cleared by reset
//#if HW_VER > 01
	//		BatteryMOS_OFF();		//disconnect battery from load (HW02>),
//#else
			InverterMOS_OFF();		//disconnect load (HW 01)
//#endif
			BatteryMOS_ON();		//battery sctually can be left ON - sometimes on cloudy evenings when battery depleted there is small current left in PV. below 100mA it can,t be consumed by inverter anyway.
		}
		if (StatCountFlagsWs.Dschg_cycle_count && StatCountFlagsWs.Dschg_cycle_c2)
		{//if stats updated, wait until it's saved
			if (Uptime.minutes == 0	//and wait for statistics to be saved and then...
					&& Uptime.seconds == 2)
			{
#if HW_VER > 01
			BackupPowerOFF();		//shut off controller completely
#endif
			InverterMOS_ON();		//can be left on, just in any case
			BatteryMOS_OFF();		//shut off controller completely
			}
		}
	}
}

void PrintConfig2TxBuffer(void)
{
	sprintf(TxBuffer,  "\r\nBattery controller for On-Grid PV microinverter\r\n"
				"HW ver %u\r\n"
				"FW Built "__DATE__", "__TIME__" \r\n**** WTB Wojciech Błędziński **** bendziol@o2.pl\r\n"
				"Config\r\n"
				"NO_FLASH_PAGES %u, "
				"MOSFET_MAX_TEMP %u, "
				"PV_CURRENT_MIN %u, "
				"INV_CURRENT_MIN %u, "
				"PV_OCV_VOLGATE %u, "
				"BATT_MIN_VOLTAGE %u, "
				"BATT_CRITICAL_MIN_VOLTAGE %u, "
				"BATT_MAX_VOLTAGE %u, "
				"BATT_VOLTAGE_MAXHYSTERESIS %u, "
				"BATT_VOLTAGE_MINHYSTERESIS %u, "
				"TIME2RESET_INV %u, "
				"TIME2OVLD_INV %u, "
				"INV_CURRENT_MAX %u, "
				"INV_CURR_SC %u, "
				"\r\nStart\n\r"
				,HW_VER,NO_FLASH_PAGES,MOSFET_MAX_TEMP,PV_CURRENT_MIN,INV_CURRENT_MIN,PV_OCV_VOLGATE
				,BATT_MIN_VOLTAGE,BATT_CRITICAL_MIN_VOLTAGE,BATT_MAX_VOLTAGE,BATT_VOLTAGE_MAXHYSTERESIS
				,BATT_VOLTAGE_MINHYSTERESIS,TIME2RESET_INV, TIME2OVLD_INV, INV_CURRENT_MAX, INV_CURR_SC
				);
}
void PrintFlashStats2TxBuffer(void)
{
	sprintf(TxBuffer, "Stat_Flash:FlashPCount %u, TNightTime %u,TNoBat2Chg %u,TNoInv %u, "
	    	    		"WhBattIn %u; "
	    	    		"WhBattNoInv %u; WhBattOut %u, "
	    	    		"WhBattRech %u; WhInv %u, "
	    				"Dschgcy %u; Chgcy %u, "
	    	    		"InvFaultCntr %u; InvResetCntr %u\r\n"
	    	    		"MaxTempBatMos %u; MaxTempInvMos %u, "
						"MaxTempInvMosCntr %u; MaxTempBatMosCntr %u, "
	    	    		"MaxInvCurrent %u, MaxPVCurrent %u, "
						"MaxInvCurrentCntr %u; MaxPVCurrentCntr %u, "
	    	    		"MaxBatVolt %u, MinBatVolt %u, "
						"MaxBatVoltageCntr %u, MinBatVoltageCntr %u, "
	    	    		"InvOvcCntr %u, InvExtRstCnt %u, "
	    	    		"ChmAhLastF %u, DchmAhLastF %u, "
	    	    		"DschVlastF %u, ChVlastF %u, "
	    	    		"ChgAhlast %u, "
	    	    		"\r\n"
	    	    ,(unsigned int)StatCurrentWh.FlashPageCounter, (unsigned int )StatCurrentWh.Time_NightTime, (unsigned int )StatCurrentWh.Time_NoBattery2Chg, (unsigned int )StatCurrentWh.Time_NoInv
	    		,(unsigned int)StatCurrentWh.Wh_BattIn
	    	    ,(unsigned int)StatCurrentWh.Wh_BattNoInv, (unsigned int )StatCurrentWh.Wh_BattOut
	    		,(unsigned int)StatCurrentWh.Wh_BattRecharge, (unsigned int )StatCurrentWh.Wh_Inverter
	    		,(unsigned int)StatCurrentWh.Dschg_cycle_count, (unsigned int )StatCurrentWh.Chg_cycle_count
	    		,(unsigned int)StatCurrentWh.InvFaultCntr, (unsigned int)StatCurrentWh.InvResetCntr
	    		,(unsigned int)StatCurrentWh.MaxTempBatMos, (unsigned int)StatCurrentWh.MaxTempInvMos
				,(unsigned int)StatCurrentWh.MaxTempInvMosCntr, (unsigned int)StatCurrentWh.MaxTempBatMosCntr
	    		,(unsigned int)StatCurrentWh.MaxInvCurrent, (unsigned int)StatCurrentWh.MaxPVCurrent
				,(unsigned int)StatCurrentWh.MaxInvCurrentCntr, (unsigned int)StatCurrentWh.MaxPVCurrentCntr
	    		,(unsigned int)StatCurrentWh.MaxBatVoltage, (unsigned int)StatCurrentWh.MinBatVoltage
				,(unsigned int)StatCurrentWh.MaxBatVoltageCntr, (unsigned int)StatCurrentWh.MinBatVoltageCntr
	    		,(unsigned int)StatCurrentWh.InvOvcCounter, (unsigned int)StatCurrentWh.InvExtResetCnt
	    		,(unsigned int)StatCurrentWh.Chg_Ah_lastFull, (unsigned int)StatCurrentWh.Dschg_Ah_lastFull
	    		,(unsigned int)StatCurrentWh.Dschg_Volt_lastFull, (unsigned int)StatCurrentWh.Chg_Volt_lastFull
	    		,(unsigned int)StatCurrentWh.Chg_Ah_last
	    	    );
}

void ShowWhStats(void)
{
	sprintf(TxBuffer, "Wh.FlashPage: %u, TNightTime %u,TNoBatt2Chg %u,TNoInv %u, "
    		"WhBattIn %u; "
    		"WhBattNoInv %u; WhBattOut %u, "
    		"WhBattRech %u; WhInv %u, "
			"DschgC %u; ChgC %u, "
			"InvFaultCntr %u; InvResetCntr %u, "
			"ChmAhcur %u, DchmAhCur %u, "
			"DchmAh-1 %u, DchmAh-2 %u, DchmAh-3 %u, "
			"ChmAhLF %u, DchmAhLF %u, "
			"ChgmAhL %u; ChgmAh-1 %u; ChgmAh-2 %u; ChgmAh-3 %u;"
			"DayCurr %u, Day-1 %u, "
			"Day-2 %u, Day-3 %u, "
    		"\r\n"
    ,(unsigned int )StatCurrentWh.FlashPageCounter, (unsigned int )StatCurrentWh.Time_NightTime, (unsigned int )StatCurrentWh.Time_NoBattery2Chg, (unsigned int )StatCurrentWh.Time_NoInv
	,(unsigned int )StatCurrentWh.Wh_BattIn
    ,(unsigned int )StatCurrentWh.Wh_BattNoInv, (unsigned int )StatCurrentWh.Wh_BattOut
	,(unsigned int )StatCurrentWh.Wh_BattRecharge, (unsigned int )StatCurrentWh.Wh_Inverter
	,(unsigned int )StatCurrentWh.Dschg_cycle_count, (unsigned int )StatCurrentWh.Chg_cycle_count
	,(unsigned int)StatCurrentWh.InvFaultCntr, (unsigned int)StatCurrentWh.InvResetCntr
	,(unsigned int)StatCurrentWh.Chg_Ah_current, (unsigned int)StatCurrentWh.Dschg_Ah_current
	,(unsigned int)StatCurrentWh.Chg_Ah_lastFull, (unsigned int)StatCurrentWh.Dschg_Ah_lastFull
	,(unsigned int)StatCurrentWh.Dschg_Ah_lastFull_1, (unsigned int)StatCurrentWh.Dschg_Ah_lastFull_2, (unsigned int)StatCurrentWh.Dschg_Ah_lastFull_3
	,(unsigned int)StatCurrentWh.Chg_Ah_last, (unsigned int)StatCurrentWh.Chg_Ah_1, (unsigned int)StatCurrentWh.Chg_Ah_2, (unsigned int)StatCurrentWh.Chg_Ah_3
	,(unsigned int)StatCurrentWh.DayDuration_current, (unsigned int)StatCurrentWh.DayDuration_1
	,(unsigned int)StatCurrentWh.DayDuration_2, (unsigned int)StatCurrentWh.DayDuration_3
    );
}

void PrintfCalData(void)
{
	sprintf(TxBuffer, "Cal Data: Indicator %u:   "
	        		"Inv_I_offsetmA %u; PV_I_offsetmA %u; "
	        		"PVV_off %u; BattV_off %u, "
	        		"NTC_PCB_off %u; NTC_INV_off %u, "
	    			"NTC_Batt_off %u, Vref_off %u "
	        		"\r\n"
	        ,(int )CalibrationValues.Indicator
	    	,(int )CalibrationValues.Inv_current_off, (int )CalibrationValues.PV_current_off
	    	,(int )CalibrationValues.PV_voltage, (int )CalibrationValues.Batt_voltage
	    	,(int )CalibrationValues.NTC1_PCB, (int )CalibrationValues.NTC2_Inverter_mos
	    	,(int )CalibrationValues.NTC3_Battery_mos, (int )CalibrationValues.VrefInt
	        );
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint16_t TxSize = 0;

	InverterMOS_ON();
	HAL_GPIO_WritePin(MEAS_PWR_GPIO_Port, MEAS_PWR_Pin, 0);		//turn ON power for op amp and other stuff
	RestoreStatisticsFromFLASH();
	RestoreCalValuesFromFLASH();
	HAL_UART_MspInit(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, RxBuffer, RX_BFR_SIZE);
	  //HAL_UART_MspDeInit();
#if HW_VER > 01
	BackupPowerON();
#endif
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	osDelay(50);
	if (HAL_ADC_Start_DMA(&hadc1, &Adc1RawReadings.Inv_current , sizeof(Adc1RawReadings)/sizeof(uint32_t)) != HAL_OK) return 0;

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
	PrintConfig2TxBuffer();
	TxSize = strlen(TxBuffer);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, TxSize);
	//BatteryMOS_ON();
	//BatteryMOS_OFF();
	//InverterMOS_OFF();
    //InverterMOS_ON();
	osDelay(5);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
	ReadConfig();
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, ConfigReg & 0b00000001);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ConfigReg & 0b00000010);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ConfigReg & 0b00000100);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, ConfigReg & 0b00001000);
	osDelay(50);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
    PrintFlashStats2TxBuffer();
    TxSize = strlen(TxBuffer);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, TxSize);
	osDelay(20);
    PrintfCalData();
    TxSize = strlen(TxBuffer);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, TxSize);
    osTimerStart(myTimer01Handle, 100);		//start timer
    StatCurrentWh.Chg_Ah_current=0;
    StatCurrentWh.Dschg_Ah_current=0;
    StatCountFlagsWs.Dschg_cycle_c2 = 1; 	//block saving dsch data before charging battery (in case of discharged battery during cloudy evenings with multiple possible controller restarts)
    StatCountFlagsWs.Dschg_cycle_count = 1;	//block saving dsch stats before even starting battery charge
    StatCountFlagsWs.ChgStatSaved = 1; 		//chg stats update only after actual charge; prevents any stats update during evening restarts
  /* Infinite loop */
  for(;;)
  {

	osDelay(90);		//one second delay
    HAL_GPIO_WritePin(MEAS_PWR_GPIO_Port, MEAS_PWR_Pin, 0);		//turn ON power for op amp and other stuff; just before soft timer starts
    while (!FlagRunMainLoop) {osDelay(2);};		//synchronizing timer with main loop
    //diabling pwr for op amp must be synchronized with ADC. for now permanently ON.
    //HAL_GPIO_WritePin(MEAS_PWR_GPIO_Port, MEAS_PWR_Pin, 1);		//turn OFF power for op amp and other stuff; when procedure is complete
    if (Flag_ShowStats == 3)
    {
        Flag_ShowStats = 2;
        ShowWhStats();
    }
    else if (Flag_ShowStats == 2)
    {
    	Flag_ShowStats = 1;
    	sprintf(TxBuffer, "Ws.TNightTime %u,TNoBat2Chg %u,TNoInv %u, "
        		"WsBattIn %u; "
        		"WsBattNoInv %u; Ws_BattOut %u, "
        		"WsBattRech %u; Ws_Inv %u, "
    			"DuskTime %u, "
    			"ChgmAs %u, DchgmAs %u, "
        		"\r\n"
        ,(unsigned int )StatCurrentWs.Time_NightTime, (unsigned int )StatCurrentWs.Time_NoBattery2Chg, (unsigned int )StatCurrentWs.Time_NoInv
    	,(unsigned int )StatCurrentWs.Ws_BattIn
        ,(unsigned int )StatCurrentWs.Ws_BattNoInv, (unsigned int )StatCurrentWs.Ws_BattOut
    	,(unsigned int )StatCurrentWs.Ws_BattRecharge, (unsigned int )StatCurrentWs.Ws_Inverter
		,(unsigned int)StatCurrentWs.Time_DuskTime
		,(unsigned int)StatCurrentWs.ChgAs, (unsigned int)StatCurrentWs.DschgAs
        );
    }
    else if (Flag_ShowStats == 4)	//show flash stats
    {
    	PrintFlashStats2TxBuffer();
    	Flag_ShowStats = 0;
    }
    else if (Flag_ShowStats == 5)		//show config
    {
    	PrintConfig2TxBuffer();
    	Flag_ShowStats = 0;
    }
    else if (Flag_ShowStats == 1)		//show current Values
	{
    	Flag_ShowStats = 0;
		sprintf(TxBuffer, "%u d %u h %u m %u s "
				"conf %u, "
				"stat: %u invMOS; %u batMOS, %u bckpMOS, "
				"temp PCB %u; invMOS %u; batMOS %u, "
				" mAInv %u; mAPV %u; "
				"VPV %u; VBat %u; "
				"Ext_I %u, "
				"\r\n"
		,(unsigned int )Uptime.days, (unsigned int )Uptime.hours, (unsigned int )Uptime.minutes, (unsigned int )Uptime.seconds
		,(unsigned int )ConfigReg
		,(unsigned int )FlagInverterMOS, (unsigned int )FlagBatteryMOS, (unsigned int)FlagBackupMOS
		,(unsigned int )Adc1Measurements.NTC1_PCB, (unsigned int )Adc1Measurements.NTC2_Inverter_mos, (unsigned int )Adc1Measurements.NTC3_Battery_mos
		,(unsigned int)Adc1Measurements.Inv_current
		,(unsigned int)Adc1Measurements.PV_current
		,(unsigned int)Adc1Measurements.PV_voltage
		,(unsigned int)Adc1Measurements.Batt_voltage	// (unsigned int)Adc1Measurements.Batt_voltage%100
		,(unsigned int)FlagExt_I
		);
	}
    else if (Flag_ShowStats == 62)	//show calibration data
    {
    	PrintfCalData();
    	if (Flag_ShowStats) Flag_ShowStats--;
    }
    else if (Flag_ShowStats == 60)		//show help
    {
    	if (Flag_ShowStats) Flag_ShowStats--;
    }
    else
    {
    	if (Flag_ShowStats) Flag_ShowStats--;
    	TxBuffer[0]=0;
    }
    TxSize = strlen(TxBuffer);
    if (TxSize>TX_BFR_SIZE) TxSize=TX_BFR_SIZE;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, TxSize);

    if (Flag_StoreStatistics)
    	{
    		StoreStatistics2FLASH();
    		Flag_StoreStatistics=0;
    	}

    /*main algorithm*/
    //**************************************CONFIG ENERGY TO MAINS************************************
    if (ConfigReg < CONFIG_BATT_0DELAY)
    {
    	//is OK to switch on INV mosfet?
    	if (Adc1Measurements.NTC2_Inverter_mos > MOSFET_MAX_TEMP)
    	{//no, its too hot
    		ConfigReg = CONFIG_BATT_0DELAY;	//config changed 'energy to battery'
			InverterMOS_OFF();		//OVT for InvMosfet usually happens during battery discharge
    	}
    	else
    	{//yes, you can switch on INV mosfet
    		//is it day?
    		//caution: too high PV_CURRENT_MIN causes troublesome starting at dawn, controllers disables BATMOSON (batt discharged) but PV current is too weak to keep INVerter operational
    		//too low PV_CURRENT_MIN causes troublesome change operation mode at dusk - inverter causes restarts of controller overloading weak PV current source
			if (Adc1Measurements.PV_current > (PV_CURRENT_MIN - PVCurrentHysteresis) ||
					Adc1Measurements.PV_voltage > PV_OCV_VOLGATE)
			{//yes, its day
				PVCurrentHysteresis = PV_CURRENT_HYST;	//to prevent multiple switching day/night at dawn and dusk
				InverterOn_batteryAsBackup();
				StatCountFlagsWs.Time_Daytime=1;		//enable to count daytime
			}//end of its day
    		else
    		{//no, its night
    			PVCurrentHysteresis = 0;	//to prevent multiple switching day/night at dawn and dusk
    			DischargeProcedure();
    		}//closing "its night"
    	}//closing "can switch INV ON"
    }//closing MAINS config
    //************************************CONFIG ENERGY TO BATTERY*********************************
    else if (ConfigReg > CONFIG_MAINS_6DELAY &&
    		ConfigReg < CONFIG_MAINS_NOBATTDSCHG)
    {
		//is OK to switch on battery mosfet?
		if (Adc1Measurements.NTC3_Battery_mos > MOSFET_MAX_TEMP)
		{//no, mosfet's too hot
			ConfigReg = CONFIG_MAINS_0DELAY;	//change config energy to mains
		}
		else
		{//yes, you can switch on battery mosfet
			//is it a day?
			if (Adc1Measurements.PV_current > (PV_CURRENT_MIN - PVCurrentHysteresis) || Adc1Measurements.PV_voltage > PV_OCV_VOLGATE)
			{//yes, its day
				PVCurrentHysteresis = PV_CURRENT_HYST;	//to prevent multiple switching day/night at dawn and dusk
				StatCountFlagsWs.Time_Daytime=1;		//enable to count daytime
				//is batt OK to charge
				if (Adc1Measurements.Batt_voltage < (BATT_MAX_VOLTAGE+VoltHysteresisChg)  &&
						Adc1Measurements.Batt_voltage > BATT_CRITICAL_MIN_VOLTAGE)
				{//yes, ok to charge
					BatteryMOS_ON();
					InverterMOS_OFF();
					VoltHysteresisChg = BATT_VOLTAGE_MAXHYSTERESIS;
					StatCountFlagsWs.Ws_BattIn=1;	//enable to count energy stored in battery, in 1Sectimer
					StatCountFlagsWs.Chg_cycle_count = 0;	//when battery charging unlock flag enabling counting dschg cycles
					StatCountFlagsWs.ChgAs=1;		//enable counting ampere-seconds
				}
				else
				{//no, batt not OK to charge
					InverterMOS_ON();
					BatteryMOS_OFF();
					StatCountFlagsWs.Time_NoBattery2Chg=1;	//enable to count time without possibility to charge battery, in 1Sectimer
					VoltHysteresisChg = 0;
					if ((Adc1Measurements.Batt_voltage) > (BATT_MAX_VOLTAGE+VoltHysteresisChg))
					{//if battery fully charged, check if it was moment ago.
						StatCurrentWh.Chg_Ah_lastFull=StatCurrentWh.Chg_Ah_current;	//store fully chg Ah
						if (!StatCountFlagsWs.Chg_cycle_c2 && StatCountFlagsWs.Chg_cycle_count)
						{
							StatCurrentWh.Chg_Volt_lastFull = Adc1Measurements.Batt_voltage;	//store lastvoltage
							StatCountFlagsWs.Chg_cycle_c2 = 1;
							StatCountFlagsWs.Dschg_cycle_c2 = 0;
						}
						if (!StatCountFlagsWs.Chg_cycle_count)
						{
							StatCountFlagsWs.Chg_cycle_count = 1; //if not, set flag 'fully charged' to count/set only once
							StatCountFlagsWs.Dschg_cycle_count = 0;	//clear flag to enable dschg counter when batt empty
							StatCurrentWh.Dschg_Ah_current=0;	//clear discharge Ah
#if FLASHSTATSAVE_PERIOD == 1	//FLASHSTATSAVE_PERIOD 1-only when batt fully charged and discharged
					Flag_StoreStatistics = 1;
#endif
						}
					}
				}
			}
			else
			{//no, its night
				PVCurrentHysteresis = 0;	//to prevent multiple switching day/night at dawn and dusk
				DischargeProcedure();
			}//closing "its night"
		}//closing "you can switch on batt mosfet"
    }//closing "config battery"
    //************************************CONFIG Spare ENERGY TO BATTERY, but no battery discharge*********************************
    else if (ConfigReg == CONFIG_MAINS_NOBATTDSCHG)
    {//is OK to switch on INV mosfet?
    	if (Adc1Measurements.NTC2_Inverter_mos > MOSFET_MAX_TEMP)
    	{//no, its too hot
    		ConfigReg = CONFIG_BATT_0DELAY;	//config changed 'energy to battery'
			InverterMOS_OFF();		//OVT for InvMosfet usually happens during battery discharge
    	}
    	else
    	{//yes, you can switch on INV mosfet
    		InverterOn_batteryAsBackup();
    	}//closing "ok to switch ON INV mosfet"
    }//closing config "spare energy to battery"
  }//closing main loop
  /* USER CODE END 5 */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */
	if (count_second) count_second--;
					else
					{
						count_second = TICKS_ONESECOND;	//1 for 1sec
						Uptime.seconds++;
						LedStatusShow();
						if (!Flag_ShowStats) Flag_ShowStats=2;				//once a second show secondy stat alternatively with measurements
						if (count_minutes) count_minutes--;
						else
						{
							count_minutes = TICKS_ONEMINUTE;	//59 for 1 minute
							Uptime.minutes++;
							Uptime.seconds=0;
							ReadConfig();
							Flag_ShowStats=3;				//once a minute show hourly stat
							if (count_hours) count_hours--;	//60 minutes for one hour
							else
							{
								Calculate_WattHours();
								count_hours = TICKS_ONEHOUR;
								Uptime.hours++;
								Uptime.minutes=0;
#if FLASHSTATSAVE_PERIOD == 0	//FLASHSTATSAVE_PERIOD 0-every hour,
								Flag_StoreStatistics = 1;
#endif
								if (count_days) count_days--;
								else
								{
									count_days = TICKS_ONEDAY;
									Uptime.days++;
									Uptime.hours=0;
								}
							}
						}
					}
		Calculate_WattSeconds();
		HAL_ADC_Start(&hadc1);
#ifdef DEBUG_EN
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
#endif
		uint32_t temp;
		//Adc1Measurements.Inv_current = ConvertIValue(Adc1RawReadings.Inv_current);
		temp = 	ConvertIValue(Adc1RawReadings.Inv_current);
		if ( temp > CalibrationValues.Inv_current_off) Adc1Measurements.Inv_current = temp-CalibrationValues.Inv_current_off;
		else Adc1Measurements.Inv_current = 0;
		//Adc1Measurements.PV_current = ConvertIValue(Adc1RawReadings.PV_current) ;
		temp = 	ConvertIValue(Adc1RawReadings.PV_current);
		if (temp > CalibrationValues.PV_current_off) Adc1Measurements.PV_current = temp-CalibrationValues.PV_current_off;
		else Adc1Measurements.PV_current = 0;
			Adc1Measurements.PV_voltage = ConvertVValue(Adc1RawReadings.PV_voltage) ;
			Adc1Measurements.Batt_voltage = ConvertVValue(Adc1RawReadings.Batt_voltage) ;
			Adc1Measurements.NTC1_PCB = ConvertNTCvalue(Adc1RawReadings.NTC1_PCB) ;
			Adc1Measurements.NTC2_Inverter_mos = ConvertNTCvalue(Adc1RawReadings.NTC2_Inverter_mos) ;
			Adc1Measurements.NTC3_Battery_mos = ConvertNTCvalue(Adc1RawReadings.NTC3_Battery_mos) ;
			FlagExt_I = HAL_GPIO_ReadPin(EXT_I_GPIO_Port, EXT_I_Pin);
		FlagRunMainLoop = 1;
  /* USER CODE END Callback01 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
