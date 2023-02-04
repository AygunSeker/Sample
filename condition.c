/* Includes ------------------------------------------------------------------*/
#include "conditions.h"

// Global Condition Handler Struct
extern Condition_Handler_t hCondition;
extern TIM_HandleTypeDef htim2;		//buzzer control
extern RTC_HandleTypeDef hrtc;		//buzzer control
// Timer variables
uint32_t countDownSec = 0, countDownSecRes = 0, \
		countDownSec_BRC1Active = 0, countDownSec_BRC1InActive = 0, \
		countDownSec_BRC2Active = 0, countDownSec_BRC2InActive = 0, \
		countDownSec_KRCActive = 0, countDownSec_KRCInActive = 0, \
		countDownSec_OSGUnLock = 0, countDownSec_OSGLock = 0, \
		countDownSec_ML1ML2ShortCircuit = 0;
// Buzzer variables
static uint16_t timerErrorBuzzer = 0, errorBuzzer = 0, flagErrorBuzzer = 0;
// Global variables
uint16_t moveUpDownCounter, ML1ML2Counter, timer140P = 0 , port130Counter = 0;
static uint8_t setPer = 0;	//this variable is used to record permanent error number and set to eeprom, in order to use during system restart

ErrorState liftError = noErrorState; 
// Global Screen Print Request Pointer
// This pointer equal to parameterMain which defines in Handler functions
//ParameterEntry *curParameter = parameterMain;

/** @addtogroup Liftern_Driver
  * @{
  */

/** @addtogroup Conditions
  * @{
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup Operation Condition Functions
 *  @brief    Check for system errors functions
 *
@verbatim
  ==============================================================================
              ##### Check for system errors functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Operation_Conditions                       : Check system errors.
@endverbatim
  * @{
  */

/**
  * @brief  This function checks every virtual inputs and sets or resets the spesific flags.
  * @param  Condition_Handler_t pointer which have parameters about conditions and flags.
  * @retval none
  */

void Operation_Conditions(uint16_t timer1msDiff, DoorControl *doors) // ibrahim: we need to disable door pointer from this function
{
	// Initialize Virtual Inputs - This is important as variable values can change all the time.
	Condition_VirtualInputsOutputs_Init(&hCondition);
	
	// Check all permanent flags - If flag is different than 0, it does not check operation conditions
	uint16_t permanentFlag = Permanent_Flag_Check(&hCondition);
	
	// Check if technician delete permanent error by parameter setting
	Is_Technician_Clear_Permanent_Error(&hCondition, &permanentFlag);
	
	// Check condition counters
	Condition_Counter_Check(&hCondition, &permanentFlag, timer1msDiff);
	
	// Check recall mode
	ReCall_Mode(&hCondition);


	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	if(((hCondition.VirtualInput.ithermistor == VIRTUAL_INPUT_LOW) && \
			(permanentFlag == PERMANENT_STATUS_LOW) && \
			(hCondition.Monitoring.mo_thermistor == VIRTUAL_INPUT_HIGH)) || \
			(hCondition.MovementFlags.flag_thermistor == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_thermistor == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.States.blockstatus_thermistor == STATE_SET)){
		// Call handler_Thermistor
		Handler_Thermistor(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((((hCondition.VirtualInput.ibrc1 == VIRTUAL_INPUT_LOW) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) && \
			(OD_RAM.x6430_targetVelocity == nospeed) && \
			(hCondition.Monitoring.mo_brc1 == VIRTUAL_INPUT_HIGH)) || \
			(hCondition.MovementFlags.flag_brc1active == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_brc1active == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_brc1active == PERMANENT_FLAG_HIGH)) && \
			(hCondition.States.conditionhandlercallstatus_brc1active == STATE_SET)){
		// Call Handler_BrcActive
		Handler_Brc1Active(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((((hCondition.VirtualInput.ibrc1 == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) && \
			(OD_RAM.x6430_targetVelocity != nospeed) && \
			(hCondition.Monitoring.mo_brc1 == VIRTUAL_INPUT_HIGH)) || \
			(hCondition.MovementFlags.flag_brc1inactive == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_brc1inactive == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_brc1inactive == PERMANENT_FLAG_HIGH)) && \
			(hCondition.States.conditionhandlercallstatus_brc1inactive == STATE_SET)){
		// Call Handler_BrcInActive
		Handler_Brc1InActive(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((((hCondition.VirtualInput.ibrc2 == VIRTUAL_INPUT_LOW) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) && \
			(OD_RAM.x6430_targetVelocity == nospeed) && \
			(hCondition.Monitoring.mo_brc2 == VIRTUAL_INPUT_HIGH)) || \
			(hCondition.MovementFlags.flag_brc2active == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_brc2active == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_brc2active == PERMANENT_FLAG_HIGH)) && \
			(hCondition.States.conditionhandlercallstatus_brc2active == STATE_SET)){
		// Call Handler_BrcActive
		Handler_Brc2Active(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((((hCondition.VirtualInput.ibrc2 == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) && \
			(OD_RAM.x6430_targetVelocity != nospeed) && \
			(hCondition.Monitoring.mo_brc2 == VIRTUAL_INPUT_HIGH)) || \
			(hCondition.MovementFlags.flag_brc2inactive == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_brc2inactive == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_brc2inactive == PERMANENT_FLAG_HIGH)) && \
			(hCondition.States.conditionhandlercallstatus_brc2inactive == STATE_SET)){
		// Call Handler_BrcInActive
		Handler_Brc2InActive(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((((hCondition.VirtualInput.ikrc == VIRTUAL_INPUT_LOW) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) && \
			(hCondition.VirtualOutput.omaincontactor == VIRTUAL_OUTPUT_LOW)) || \
			(hCondition.MovementFlags.flag_krcactive == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_krcactive == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_krcactive == PERMANENT_FLAG_HIGH)) && \
			(hCondition.States.conditionhandlercallstatus_krcactive == STATE_SET)){
		// Call Handler_BrcActive
		Handler_KrcActive(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((((hCondition.VirtualInput.ikrc == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) && \
			(hCondition.VirtualOutput.omaincontactor == VIRTUAL_OUTPUT_HIGH)) || \
			(hCondition.MovementFlags.flag_krcinactive == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_krcinactive == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_krcinactive == PERMANENT_FLAG_HIGH)) && \
			(hCondition.States.conditionhandlercallstatus_krcinactive == STATE_SET)){
		// Call Handler_BrcActive
		Handler_KrcInActive(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((((hCondition.VirtualInput.iosg == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) && \
			(hCondition.VirtualInput.idrun == VIRTUAL_INPUT_HIGH) && \
			(hCondition.Monitoring.mo_osg == VIRTUAL_INPUT_HIGH)) || \
			(hCondition.MovementFlags.flag_osgunlock == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_osgunlock == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_osgunlock == PERMANENT_FLAG_HIGH)) && \
			(hCondition.States.conditionhandlercallstatus_osgunlock == STATE_SET)){
		// Call Handler_OSGUnLock
		Handler_OSGUnLock(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((((hCondition.VirtualInput.iosg == VIRTUAL_INPUT_LOW) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) && \
			(hCondition.VirtualOutput.omaincontactor == VIRTUAL_OUTPUT_LOW) && \
			(hCondition.VirtualOutput.oapre == VIRTUAL_OUTPUT_LOW) && \
			(hCondition.Monitoring.mo_osg == VIRTUAL_INPUT_HIGH)) || \
			(hCondition.MovementFlags.flag_osglock == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_osglock == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_osglock == PERMANENT_FLAG_HIGH)) && \
			(hCondition.States.conditionhandlercallstatus_osglock == STATE_SET)){
		// Call Handler_OSGUnLock
		Handler_OSGLock(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.ixer1 == VIRTUAL_INPUT_HIGH) && (permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_xer1 == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_xer1 == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_OSGUnLock
		Handler_Xer1(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.ixer2 == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW) && \
			(OD_RAM.x6430_targetVelocity == nospeed)) || \
			(hCondition.MovementFlags.flag_xer2 == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_xer2 == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_OSGUnLock
		Handler_Xer2(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.ixbl1 == VIRTUAL_INPUT_HIGH) && (permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_xbl1 == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_xbl1 == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_OSGUnLock
		Handler_Xbl1(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.ixbl2 == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW) && \
			(OD_RAM.x6430_targetVelocity == nospeed)) || \
			(hCondition.MovementFlags.flag_xbl2 == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_xbl2 == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_OSGUnLock
		Handler_Xbl2(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.i818 == VIRTUAL_INPUT_LOW) && \
			(hCondition.VirtualInput.iml1 == VIRTUAL_INPUT_HIGH) && \
			(hCondition.VirtualInput.iml2 == VIRTUAL_INPUT_LOW) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_cartopexceed == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_cartopexceed == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_cartopexceed == PERMANENT_FLAG_HIGH)){
		// Call Handler_CarTopExceed
		Handler_CarTopExceed(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.i817 == VIRTUAL_INPUT_LOW) && \
			(hCondition.VirtualInput.iml1 == VIRTUAL_INPUT_LOW) && \
			(hCondition.VirtualInput.iml2 == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_carbottomexceed == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_carbottomexceed == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_carbottomexceed == PERMANENT_FLAG_HIGH)){
		// Call Handler_CarBottomExceed
		Handler_CarBottomExceed(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
/*	else if((((moveUpDownCounter / 1000) >= OD_MAXIMUMTIMETRAVEL) && \
			(permanentFlag == PERMANENT_STATUS_LOW) && \
			(OD_RAM.x6430_targetVelocity != nospeed)) || \
			(hCondition.MovementFlags.flag_traveltimeexceeded == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_traveltimeexceeded == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_traveltimeexceeded == PERMANENT_FLAG_HIGH)){
		// Call Handler_TimeTravelExceed
		Handler_TimeTravelExceed(&hCondition);
	}*/
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
/*	else if(((hCondition.VirtualInput.iml1ml2shortcircuit == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_ml1ml2shortcircuit == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_ml1ml2shortcircuit == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_ml1ml2shortcircuit == PERMANENT_FLAG_HIGH)){
		// Call Handler_Ml1Ml2ShortCircuit
		Handler_Ml1Ml2ShortCircuit(&hCondition);
	}*/
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
/*	else if((hCondition.VirtualInput.iml1ml2wrongorderstate == VIRTUAL_INPUT_HIGH && \
			(permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_ml1ml2wrongorderstate == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_ml1ml2wrongorderstate == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_ml1ml2wrongorderstate == PERMANENT_FLAG_HIGH)){
		// Call Handler_Ml1Ml2WrongOrderState
		Handler_Ml1Ml2WrongOrderState(&hCondition);
	}*/
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.i140 == VIRTUAL_INPUT_HIGH) && \
			(hCondition.VirtualInput.i130 == VIRTUAL_INPUT_LOW) && \
			(permanentFlag == PERMANENT_STATUS_LOW) && \
			(port130Counter >= 50)) || \
			(hCondition.MovementFlags.flag_130Off140On == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_130Off140On == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_130Off140On == PERMANENT_FLAG_HIGH)){
		// Call Handler_130Off140On
		Handler_130Off140On(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.iearthquake == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_earthquake == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_earthquake == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR) || \
			(hCondition.PermanentErrorFlags.permanentFlag_earthquake == PERMANENT_FLAG_HIGH)){
		// Call Handler_Earthquake
		Handler_Earthquake(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.i140 == VIRTUAL_INPUT_LOW) && \
			(permanentFlag == PERMANENT_STATUS_LOW) && \
			(hCondition.VirtualInput.idrun == VIRTUAL_INPUT_HIGH) && \
			(timer140P >= 50)) || \
			(hCondition.MovementFlags.flag_140interrupted == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_140interrupted == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_140Interrupted
		Handler_140Interrupted(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if((((hCondition.VirtualInput.iwrongdirectionup == VIRTUAL_INPUT_HIGH) || \
			(hCondition.VirtualInput.iwrongdirectiondown == VIRTUAL_INPUT_HIGH)) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_wrongdirection == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_wrongdirection == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_WrongDirection
		Handler_WrongDirection(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	/*else if(((hCondition.VirtualInput.iphasemissing == VIRTUAL_INPUT_HIGH) && (permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_phasemissing == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_phasemissing == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_PhaseMissing
		Handler_PhaseMissing(&hCondition);
	}*/
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
/*	else if(((hCondition.VirtualInput.iphaseseqwrong == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW) && \
			(hCondition.Monitoring.mo_phaseorder == VIRTUAL_INPUT_HIGH)) || \
			(hCondition.MovementFlags.flag_phaseseqwrong == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_phaseseqwrong == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_PhaseOrderWrong
		Handler_PhaseOrderWrong(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.i120 == VIRTUAL_INPUT_LOW) && \
			(permanentFlag == PERMANENT_STATUS_LOW) && \
			(hCondition.VirtualInput.i868 == VIRTUAL_INPUT_HIGH) && \
			(hCondition.VirtualInput.i869 == VIRTUAL_INPUT_HIGH) && \
			(hCondition.VirtualInput.i870 == VIRTUAL_INPUT_HIGH) && \
			((hCondition.Monitoring.mo_when120isoff == WHEN120ON_BLOCKELEVATOR) || \
			(hCondition.Monitoring.mo_when120isoff == WHEN120ON_BLOCKUNTILACARCALL))) || \
			(hCondition.MovementFlags.flag_120 == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_120 == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_120
		Handler_120(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.i817 == VIRTUAL_INPUT_LOW) && \
			(hCondition.VirtualInput.i818 == VIRTUAL_INPUT_LOW) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_817818 == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_817818 == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_817818
		Handler_817818(&hCondition);
	}*/
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.idoorsnotclosed == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_doorsnotclosed == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_doorsnotclosed == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_DoorsNotClosed
		Handler_DoorsNotClosed(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.idoorscontacts == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_doorscontacts == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_doorscontacts == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_DoorsContacts
		Handler_DoorsContacts(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.idoorAlimit == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_doorAlimit == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_doorAlimit == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_DoorALimit
		Handler_DoorALimit(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.idoorBlimit == VIRTUAL_INPUT_HIGH) && \
			(permanentFlag == PERMANENT_STATUS_LOW)) || \
			(hCondition.MovementFlags.flag_doorBlimit == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_doorBlimit == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_DoorBLimit
		Handler_DoorBLimit(&hCondition);
	}
	// if ithermistor is not high and liftError is noErrorState or thermistor flag is no move rev. and no move nor. or thermistor flag is move rev. and no move nor. or liftError is motorOverHeatState
	else if(((hCondition.VirtualInput.ispeedlowerthan03 == VIRTUAL_INPUT_LOW) && \
			(permanentFlag == PERMANENT_STATUS_LOW) && \
			(hCondition.Monitoring.mo_speedlowerthan03m_sn == VIRTUAL_INPUT_HIGH) && \
			(OD_PERSIST_APP_AUTO.x2002_driverType == driverVvvfParallel)) || \
			(hCondition.MovementFlags.flag_speedlowerthan03 == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) || \
			(hCondition.MovementFlags.flag_speedlowerthan03 == MOVEMENT_FLAG_MOV_REV_NOMOV_NOR)){
		// Call Handler_SpeedLowerThan03
		Handler_SpeedLowerThan03(&hCondition, doors);
	}
	if(errorBuzzer > 0){
		if(flagErrorBuzzer == 0){
			flagErrorBuzzer = 1;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		}
		timerErrorBuzzer += timer1msDiff;
		if(timerErrorBuzzer > 200){
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			timerErrorBuzzer = 0;
			errorBuzzer = 0;
			flagErrorBuzzer = 0;
		}
	}
	else{
		flagErrorBuzzer = 0;
		timerErrorBuzzer = 0;
	}
	// Update PSU Status - ibrahim: According to the status of operation conditions, this function sets the PSU_Status. This function should NOT
	// be used from any other place of program.
	PSU_Status();
}

/**
  * @}
  */

/* Private functions --------------------------------------------------------*/

/** @defgroup Initialize Functions
 *  @brief    Virtual Inputs/Outputs and Flags Initialize functions
 *
@verbatim
  ==============================================================================
              ##### Initialize functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Condition_VirtualInputsOutputs_Init        : Initialize virtual inputs and outputs.
      (+) Condition_Flags_Init              		 : Initialize flags.
@endverbatim
  * @{
  */

/**
  * @brief  This function initialize virtual inputs and outputs
  * @param  Condition_Handler_t pointer which have parameters about conditions and flags.
  * @retval none
  */

void Condition_VirtualInputsOutputs_Init(Condition_Handler_t *hCondition){
	// Initialize Thermistor Virtual Input
	hCondition->VirtualInput.ithermistor = VI_THERMISTOR;
	// Initialize brc1
	hCondition->VirtualInput.ibrc1 = VI_BREAKCONTROL1;
	// Initialize brc2
	hCondition->VirtualInput.ibrc2 = VI_BREAKCONTROL2;
	// Initialize osg
	hCondition->VirtualOutput.omaincontactor = VO_MAINCONTACTOR;
	// Initialize krc
	hCondition->VirtualInput.ikrc = VI_KRC;
	// Initialize osg
	hCondition->VirtualInput.iosg = VI_OVERSPEEDGOVERNER;
	// Initialize drun
	hCondition->VirtualInput.idrun = VI_DRIVERRUN;
	// Initialize osg
	hCondition->VirtualOutput.oapre = VO_APRE;
	// Initialize 868
	hCondition->VirtualInput.i868 = VI_868;
	// Initialize 869
	hCondition->VirtualInput.i869 = VI_869;
	// Initialize 870
	hCondition->VirtualInput.i870 = VI_870;
	// Initialize xer1
	hCondition->VirtualInput.ixer1 = VI_XER1;
	// Initialize xer2
	hCondition->VirtualInput.ixer2 = VI_XER2;
	// Initialize xbl1
	hCondition->VirtualInput.ixbl1 = VI_XBL1;
	// Initialize xbl2
	hCondition->VirtualInput.ixbl2 = VI_XBL2;
	// Initialize 817
	hCondition->VirtualInput.i817 = VI_817;
	// Initialize 818
	hCondition->VirtualInput.i818 = VI_818;
	// Initialize ml1
	hCondition->VirtualInput.iml1 = VI_ML1;
	// Initialize ml2
	hCondition->VirtualInput.iml2 = VI_ML2;
	// Initialize 140
	hCondition->VirtualInput.i140 = VI_140;
	// Initialize 130
	hCondition->VirtualInput.i130 = VI_130;
	// Initialize earthquake
	hCondition->VirtualInput.iearthquake = VI_EARTHQUAKE;
	// Initialize ml1 ml2 wrong order state
	hCondition->VirtualInput.iml1ml2shortcircuit = ECS_Ml1Ml2ShortCircuit(hCondition);
	// Initialize ml1 ml2 wrong order state
	hCondition->VirtualInput.iml1ml2wrongorderstate = ECS_Ml1Ml2WrongOrderState(hCondition);
	// Initialize Phase Missing
	hCondition->VirtualInput.iphasemissing = VI_PHASEMISSING;
	// Initialize Phase order wrong
	hCondition->VirtualInput.iphaseseqwrong = VI_PHASESEQWRONG;
	// Initialize BRC1 Monitoring
	hCondition->Monitoring.mo_brc1 = MO_BRC1;
	// Initialize BRC2 Monitoring
	hCondition->Monitoring.mo_brc2 = MO_BRC2;
	// Initialize OSG Monitoring
	hCondition->Monitoring.mo_osg = MO_OSG;
	// Initialize Thermistor Monitoring
	hCondition->Monitoring.mo_thermistor = MO_THERMISTOR;
	// Initialize Phase Order Monitoring
	hCondition->Monitoring.mo_phaseorder = MO_PHASEORDER;
	// Initialize 130
	hCondition->VirtualInput.i120 = VI_120;
	// Initialize When 120 off Monitoring
	hCondition->Monitoring.mo_when120isoff = MO_WHEN120OFF;
	// Initialize Wrong Direction Up
	hCondition->VirtualInput.iwrongdirectionup = ECS_WrongDirectionUp(hCondition);
	// Initialize Wrong Direction Down
	hCondition->VirtualInput.iwrongdirectiondown = ECS_WrongDirectionDown(hCondition);
	// Initialize 141
	hCondition->VirtualInput.i141 = VI_141;
	// Initialize 142
	hCondition->VirtualInput.i142 = VI_142;
	// Initialize 135
	hCondition->VirtualInput.i135 = VI_135;
	// Initialize Doors Not Closed
	hCondition->VirtualInput.idoorsnotclosed = VI_DOORSNOTCLOSED;
	// Initialize Doors Contacts
	hCondition->VirtualInput.idoorscontacts = VI_DOORSCONTACTS;
	// Initialize Door A Limit
	hCondition->VirtualInput.idoorAlimit = VI_DOORALIMIT;
	// Initialize Door B Limit
	hCondition->VirtualInput.idoorBlimit = VI_DOORBLIMIT;
	// Initialize Speed Lower Than 03 m/sn
	hCondition->VirtualInput.ispeedlowerthan03 = VI_SPEEDLOWERTHAN03;
}

/**
  * @brief  This function initialize flags during system reboot
  * @param  Condition_Handler_t pointer which have parameters about conditions and flags.
  * @retval none
  */

void Condition_Init(Condition_Handler_t *hCondition){
	// If flag_init is 0 (this means program is start just now)
	if(hCondition->States.state_init == STATE_INIT_LOW){
		// set all MovementFlags to "11" and set flag_init to "1"
		hCondition->MovementFlags.flag_thermistor = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_brc1active = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_brc1inactive = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_brc2active = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_brc2inactive = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_krcactive = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_krcinactive = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_osgunlock = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_osglock = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_xer1 = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_xer2 = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_xbl1 = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_xbl2 = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_cartopexceed = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_carbottomexceed = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_traveltimeexceeded = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_ml1ml2shortcircuit = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_ml1ml2wrongorderstate = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_130Off140On = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_earthquake = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_140interrupted = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_wrongdirection = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_phasemissing = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_phaseseqwrong = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_120 = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_817818 = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_doorsnotclosed = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_doorscontacts = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_doorAlimit = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_doorBlimit = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->MovementFlags.flag_speedlowerthan03 = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		hCondition->States.state_init = STATE_INIT_HIGH;

		//if there is a permanent error, below is used
		if(OD_PERSIST_APP_AUTO.x2105_permanentError != PERMANENT_STATUS_LOW){
			switch(OD_PERSIST_APP_AUTO.x2105_permanentError){
				case 1:
					hCondition->PermanentErrorFlags.permanentFlag_brc1active = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_brc1active = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					hCondition->States.conditioncounterstatus_brc1active = STATE_SET;
					liftError = brakeContReleaseUnDetectedState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 2:
					hCondition->PermanentErrorFlags.permanentFlag_brc1inactive = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_brc1inactive = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					hCondition->States.conditioncounterstatus_brc1inactive = STATE_SET;
					liftError = brakeContGripUnDetectedState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 3:
					hCondition->PermanentErrorFlags.permanentFlag_krcactive = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_krcactive = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					liftError = mainContGripUnDetectedState;
					hCondition->States.conditioncounterstatus_krcactive = STATE_SET;
					hCondition->States.fatalerrorstatus_krcactive = STATE_SET;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 4:
					hCondition->PermanentErrorFlags.permanentFlag_krcinactive = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_krcinactive = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					hCondition->States.conditioncounterstatus_krcinactive = STATE_SET;
					hCondition->States.fatalerrorstatus_krcinactive = STATE_SET;
					liftError = mainContReleaseUnDetectedState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 5:
					hCondition->PermanentErrorFlags.permanentFlag_osgunlock = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_osgunlock = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					liftError = errorOSGCFailedToOpenState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					hCondition->States.conditioncounterstatus_osgunlock = STATE_SET;
					break;
				case 6:
					hCondition->PermanentErrorFlags.permanentFlag_osglock = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_osglock = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					liftError = errorOSGCLockState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					hCondition->States.conditioncounterstatus_osglock = STATE_SET;
					break;
				case 7:
					hCondition->PermanentErrorFlags.permanentFlag_cartopexceed = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_cartopexceed = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					liftError = carOverTopDoorState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 8:
					hCondition->PermanentErrorFlags.permanentFlag_carbottomexceed = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_carbottomexceed = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					liftError = carUnderBottomDoorState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 9:
					hCondition->PermanentErrorFlags.permanentFlag_traveltimeexceeded = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_traveltimeexceeded = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					liftError = travelTimeExceededState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 10:
					hCondition->PermanentErrorFlags.permanentFlag_ml1ml2shortcircuit = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_ml1ml2shortcircuit = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					liftError = ML1ML2ShortCircuitState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 11:
					hCondition->PermanentErrorFlags.permanentFlag_ml1ml2wrongorderstate = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_ml1ml2wrongorderstate = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					liftError = ML1ML2WrongOrderState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 12:
					hCondition->PermanentErrorFlags.permanentFlag_130Off140On = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_130Off140On = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					liftError = doorContact140ON130OFFState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 13:
					hCondition->PermanentErrorFlags.permanentFlag_earthquake = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_earthquake = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					liftError = earthQuakeDetectedState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 15:
					hCondition->PermanentErrorFlags.permanentFlag_brc2active = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_brc2active = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					hCondition->States.conditioncounterstatus_brc2active = STATE_SET;
					liftError = brakeContReleaseUnDetectedState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
				case 16:
					hCondition->PermanentErrorFlags.permanentFlag_brc2inactive = PERMANENT_FLAG_HIGH;
					hCondition->MovementFlags.flag_brc2inactive = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
					hCondition->States.conditioncounterstatus_brc2inactive = STATE_SET;
					liftError = brakeContGripUnDetectedState;
					parameterMain[0].text1 = blockPermanentlyTxt;
					/* An audible signal is emitted */
					errorBuzzer = 1;
//					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					break;
			}
		}
	}
}

/**
  * @}
  */


/** @defgroup Handler Functions
 *  @brief    Errors handler functions
 *
@verbatim
  ==============================================================================
              ##### Handler functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Handler_Thermistor                         : Define Thermistor modes.
      (+) Handler_BrcActive		                     : Define BRC Active modes.
      (+) Handler_BrcInActive		                 : Define BRC InActive modes.
      (+) Handler_KrcActive		                     : Define KRC Active modes.
      (+) Handler_KrcInActive		                 : Define KRC InActive modes.
      (+) Handler_OSGUnLock		                 	 : Define OSG Unlock modes.
      (+) Handler_OSGLock		                 	 : Define OSG Lock modes.
      (+) Handler_Xer1		                 		 : Define Xer1 modes.
      (+) Handler_Xer2		                 		 : Define Xer2 modes.
      (+) Handler_Xbl1		                 		 : Define Xbl1 modes.
      (+) Handler_Xbl2		                 		 : Define Xbl2 modes.
      (+) Handler_CarTopExceed		                 : Define Car Top Exceed modes.
      (+) Handler_CarBottomExceed		             : Define Car Bottom Exceed modes.
	  (+) Handler_TimeTravelExceed		             : Define Time Travel Exceed modes.
	  (+) Handler_Ml1Ml2ShortCircuit		         : Define Ml1 Ml2 Short Circuit modes.
	  (+) Handler_Ml1Ml2WrongOrderState		         : Define Ml1 Ml2 Wrong Order State modes.
	  (+) Handler_130Off140On				         : Define 130 Off 140 On modes.
	  (+) Handler_Earthquake				         : Define Earthquake modes.
	  (+) Handler_140Interrupted				     : Define 140 Interrupted modes.
	  (+) Handler_WrongDirection				     : Define 140 Interrupted modes.
	  (+) Handler_PhaseMissing				    	 : Define Phase Missing modes.
	  (+) Handler_PhaseOrderWrong				     : Define Phase Order Wrong modes.
@endverbatim
  * @{
  */

/**
  * @brief  This function sets or resets the thermistor flag by their modes
  *         and displays error messages.
  * @param  Condition_Handler_t pointer which have parameters about conditions and flags.
  * @retval none
  */

void Handler_Thermistor(Condition_Handler_t *hCondition)
{

	/* Engage Mode */
	// if virtual input thermistor is high and thermistor flag is move rev. move nor.
	// this block(mode) set thermistor flag no move rev. no move nor. , displays error message and update liftError
	if((hCondition->VirtualInput.ithermistor == VIRTUAL_INPUT_LOW) && \
			(hCondition->MovementFlags.flag_thermistor == MOVEMENT_FLAG_MOV_REV_MOV_NOR)){
		if(OD_RAM.x6430_targetVelocity == nospeed){
			/* An audible signal is emitted */
			errorBuzzer = 1;
//			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
			// update liftError status
			liftError = motorOverHeatState;
			// display error message
			parameterMain[0].text1 = blockElevatorTxt;
			hCondition->States.blockstatus_thermistor = STATE_SET;
			// Set Thermistor flag no move rev. no move nor.
			hCondition->MovementFlags.flag_thermistor = MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR;
			// Log Error
			ErrorLog(&liftError);
		}
		else if(OD_RAM.x6430_targetVelocity != nospeed){
			/* An audible signal is emitted */
//			errorBuzzer = 1;
//			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
			// update liftError status
			liftError = motorOverHeatState;
			// display error message
			parameterMain[0].text1 = blockElevatorTxt;
			// Set Thermistor flag no move rev. no move nor.
			hCondition->States.actionflag_gotoparametertarget = ACTION_FLAG_HIGH;
			// Log Error
//			ErrorLog(&liftError);
		}
	}

	/* Keep Mode */
	// else if virtual input thermistor is not high and thermistor flag is no move rev. no move nor. and thermistor state is not set
	// this block(mode) display error message
	else if((hCondition->VirtualInput.ithermistor == VIRTUAL_INPUT_LOW) && \
			(hCondition->MovementFlags.flag_thermistor == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR) && \
			(hCondition->States.recallstatus_recall == STATE_RESET)){
		// sistem restore modundaki sayım sırasında iken hata tekrar gelirse, sayıma tekrar girsin diye
		if(hCondition->States.counterstatus_thermistor == STATE_SET){
			// Reset state thermistor
			hCondition->States.counterstatus_thermistor = STATE_RESET;
		}
		// update liftError status
		liftError = motorOverHeatState;
		// display error message
		parameterMain[0].text1 = blockElevatorTxt;
	}


	/* Restore Mode */
  // if virtual input thermistor is not high and thermistor flag is no move rev. no move nor. and state thermistor is not set
  // this block(mode) set countDownSec 180, display countDownSec and set state thermistor variable.
  // else if virtual input thermistor is not high and thermistor flag is no move rev. no move nor. and state thermistor is set and countDownSec is not Timer_Reset
  // this block(mode) set thermistor flag is move rev. move nor. and reset state thermistor variable.
  // else if virtual input thermistor is high and countDownSec is 0
  // this block(mode) set PSU state
  // else if virtual input thermistor is not high and thermistor flag is no move rev. no move nor. and state thermistor is set
  // this block(mode) display error message.
	else if((hCondition->VirtualInput.ithermistor == VIRTUAL_INPUT_HIGH) && \
			(hCondition->MovementFlags.flag_thermistor == MOVEMENT_FLAG_NOMOV_REV_NOMOV_NOR)){
	  // If state is not set
	  if(hCondition->States.counterstatus_thermistor == STATE_RESET){
		  /* An audible signal is emitted */
		  errorBuzzer = 1;
		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		  // Count value
		  countDownSec = (TIMER_VALUES_THERMISTOR*200);
		  // Display second counter value
		  parameterMain[0].value = (countDownSec/200);
		  // Display second text
		  parameterMain[0].text1 = secondTxt;
		  // Set state thermistor
		  hCondition->States.counterstatus_thermistor = STATE_SET;
	  }
	  //If state is set and virtual thermistor input is high and countDownSec is set
	  else if((hCondition->States.counterstatus_thermistor == STATE_SET) && (countDownSec != TIMER_RESET)){
		  // Update Counter
		  countDownSec--;
		  // Display second counter value
		  parameterMain[0].value = (countDownSec/200);
		  // Display second text
		  parameterMain[0].text1 = secondTxt;
	  }
	  else if((countDownSec == TIMER_RESET) && (hCondition->States.counterstatus_thermistor == STATE_SET)){
		  // liftError status noErrorState
		  liftError = noErrorState;
		  // Set thermistor flag move rev. move nor.
		  hCondition->MovementFlags.flag_thermistor = MOVEMENT_FLAG_MOV_REV_MOV_NOR;
		  hCondition->States.blockstatus_thermistor = STATE_RESET;
		  // Reset state thermistor
		  hCondition->States.counterstatus_thermistor = STATE_RESET;
	  }
	}
}

/**
  * @brief  This function sets or resets the break control1 flag by their modes
  *         and displays error messages.
  * @param  Condition_Handler_t pointer which have parameters about conditions and flags.
  * @retval none
  */

void Handler_Brc1Active(Condition_Handler_t *hCondition)
{
