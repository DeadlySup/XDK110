/* module includes ********************************************************** */
#include "XDKAppInfo.h"
#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_ACCEL_OVER_BLE
#define MINUTES(x) ((portTickType) (x * 60000 / portTICK_RATE_MS))
#define MS(x) ((portTickType) x / portTICK_RATE_MS)

/* own header files */
#include "SendAccelerometerDataOverBle.h"
#include "XdkSensorHandle.h"
#include "XdkUsbResetUtility.h"

/* system header files */
#include <stdio.h>
#include "BCDS_Basics.h"
#include <string.h>
#include <math.h>

/* additional interface header files */
#include "BCDS_Assert.h"
#include "BCDS_BidirectionalService.h"
#include "BCDS_Ble.h"
#include "BCDS_BlePeripheral.h"
#include "BCDS_CmdProcessor.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

/* constant definitions ***************************************************** */

/* local variables ********************************************************** */

/* global variables ********************************************************* */
CmdProcessor_T *AppCmdProcessorHandle;
static uint8_t bleTransmitStatus = NUMBER_ZERO; /**< Validate the repeated start flag */
static SemaphoreHandle_t BleStartSyncSemphr = NULL;
static SemaphoreHandle_t BleWakeUpSyncSemphr = NULL;
static SemaphoreHandle_t SendCompleteSync = NULL;
extern Accelerometer_HandlePtr_T xdkAccelerometers_BMA280_Handle;
extern Environmental_HandlePtr_T xdkEnvironmental_BME280_Handle;
static xTimerHandle gTimerHandle;
static xTimerHandle EnvironmentalTimerHandle;
static int32_t *Data = 0;
/**
 * Index Declaration: 0-143: Temperature in mC
 * 144-287: Pressure in Pa
 * 288-431: Humidity in %RelH
 * 432: holds Low-G-Violations
 * 433: holds High-G-Violations (High-G-Interrupt)
 * 434: holds Low-Temperature Violations (Low-T-Interrupt)
 * 435: holds High-Temperature Violations (High-T-Interrupt)
 * 436: holds Low-Pressure Violations (Low-P-Interrupt)
 * 437: holds High-Pressure Violations (High-P-Interrupt)
 * 438: holds Low-Humidity Violations (Low-RH-Interrupt)
 * 439: holds High-Humidity Violations (High-RH-Interrupt
 */
static int16_t i = 0; /* index variable for arrays */
static int16_t a = 0; /* index variable for data transmission */
static int8_t Buffer[BUFFERSIZE]; /* Buffer for Data transmission */

/* Overall size of Global Variables (user declared): 4 Bytes x 440 + 1 = 1761 Bytes */

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

/**
 * @brief function to initialize the BME280
 *
 * @param [out] data : Retcode of the function, only possible value "ok", otherwise infinite loop
 */
void arraydef(void)
{
	Data = (long int*) realloc(Data, DataArrayLength * sizeof(long int));
}
Retcode_T EnvironmentalSensorInit(void)
 {
	Retcode_T retval = RETCODE_FAILURE;
	printf("reached E sensor init \n\r");
	arraydef();
	retval = Environmental_init(xdkEnvironmental_BME280_Handle);
	 if (retval != RETCODE_OK)
		 {
		 	printf("E_init failed. \r\n");
		 	return retval;
			 assert(0); /* assert(0) always fails -> application will never reach this point */
		 }
	retval = Environmental_setPowerMode(xdkEnvironmental_BME280_Handle, ENVIRONMENTAL_BME280_POWERMODE_NORMAL);
	 if (retval != RETCODE_OK)
		 { /* Handling the powermode will be performed in a different task for the measurement */
			 printf("E_setPowerMode failed. \r\n");
			 return retval;
		 }
	retval = Environmental_setFilterCoefficient(xdkEnvironmental_BME280_Handle,1);
	 if (retval != RETCODE_OK)
		 {
		 	printf("E_setFilterCoefficient failed. \r\n");
		 	return retval;
		 }
	retval  = Environmental_setOverSamplingPressure(xdkEnvironmental_BME280_Handle,3);
	 if (retval != RETCODE_OK)
		 {
		 	printf("setOversamplingTemeprature failed. \r\n");
		 	return retval;
			 assert(0);
		 }
	retval = Environmental_setOverSamplingTemperature(xdkEnvironmental_BME280_Handle,2);
	if (retval != RETCODE_OK)
		 {
			printf("setOversamplingTemperature failed. \r\n");
			return retval;
		 }
	 retval = Environmental_setOverSamplingHumidity(xdkEnvironmental_BME280_Handle, 2);
	 if (retval != RETCODE_OK)
		 {
			 printf("setOversamplingHumidity failed. \r\n");
			 return retval;
		 }
	 return retval;
 }

/**
 * @brief function to initialize the BMA280 Accelerometer
 *
 * @param [out]  data : Retcode of the function
 */
Retcode_T AccelerometerSensorInit(void)
{
	Retcode_T retval = RETCODE_FAILURE;
	printf("reached A init \r\n");
	retval = Accelerometer_init(xdkAccelerometers_BMA280_Handle);
	if (retval != RETCODE_OK)
		{
			printf("Accelerometer initialization failed. \r\n");
			return retval; /* Mithilfe der return-Anweisung wird die Ausführung einer Funktion beendet, und die Steuerung wird an die aufrufende Funktion zurückgegeben */
		}
	retval = Accelerometer_setRange(xdkAccelerometers_BMA280_Handle, ACCELEROMETER_BMA280_RANGE_4G);
	if (retval != RETCODE_OK)
		{
			printf("Accelerometer setRange failed. \r\n");
			return retval;
		}
	retval = Accelerometer_setMode(xdkAccelerometers_BMA280_Handle, ACCELEROMETER_BMA280_POWERMODE_NORMAL);
	if (retval != RETCODE_OK)
		{
			printf("Accelerometer setMode failed. \r\n");
			return retval;
		}
	return retval;
}


/**
 * @brief function for the g measurement routine
 */
static void AccelerometerMeasure(xTimerHandle xTimer)
{
	(void) xTimer;
	Retcode_T ReturnValue = RETCODE_FAILURE;
	Accelerometer_XyzData_T bma280 = {INT32_C(0), INT32_C(0), INT32_C(0)};
	ReturnValue = Accelerometer_readXyzGValue(xdkAccelerometers_BMA280_Handle, &bma280);
			if (ReturnValue == RETCODE_OK)
			{

				/**if (abs(bma280.xAxisData - xOffset) >= 500 || abs(bma280.yAxisData - yOffset) >= 500 || abs(bma280.zAxisData - zOffset) >= 500) checks for G-Violation after Offset-Clearance
	*			{ */
					if (sqrt(sqr(bma280.xAxisData) + sqr(bma280.yAxisData) + sqr(bma280.zAxisData)) > 1210 || sqrt(sqr(bma280.xAxisData) + sqr(bma280.yAxisData) + sqr(bma280.zAxisData)) < 810) /* checks if the measured overall acceleration exceeds 1g (borders by experience values */
					{
					Data[432]++;
					}

			}
			else
			{
				printf("measurement failed.\r\n");
			}
			/* interrupt handling for HighGViolation_Counter tbd */
}
/* inline functions ********************************************************* */

/* local functions ********************************************************** */

/**
 * @brief Callback function called on BLE event
 *
 * @param [in]  event : event to be send by BLE during communication.
 *
 * @param [in]  data : void pointer pointing to a data type based on event.
 *                     currently reserved for future use
 *
 * event                                    |   data type                   |
 * -----------------------------------------|-------------------------------|
 * BLE_PERIPHERAL_STARTED                   |   Retcode_T                   |
 * BLE_PERIPHERAL_SERVICES_REGISTERED       |   unused                      |
 * BLE_PERIPHERAL_SLEEP_SUCCEEDED           |   Retcode_T                   |
 * BLE_PERIPHERAL_WAKEUP_SUCCEEDED          |   Retcode_T                   |
 * BLE_PERIPHERAL_CONNECTED                 |   Ble_RemoteDeviceAddress_T   |
 * BLE_PERIPHERAL_DISCONNECTED              |   Ble_RemoteDeviceAddress_T   |
 * BLE_PERIPHERAL_ERROR                     |   Retcode_T                   |
 */
static void BleEventCallBack(BlePeripheral_Event_T event, void * test)
{

    BlePeripheral_Event_T Event = event;
    BCDS_UNUSED(test);
    switch (Event)
    {
    case BLE_PERIPHERAL_SERVICES_REGISTERED:
        break;
    case BLE_PERIPHERAL_STARTED:
        printf("BLE powered ON successfully \r\n");
        if ( xSemaphoreGive( BleStartSyncSemphr ) != pdTRUE)
        {
            /* We would not expect this call to fail because we must have obtained the semaphore to get here.*/
            Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
        }
        break;

    case BLE_PERIPHERAL_SLEEP_SUCCEEDED:
        printf("BLE successfully entered into sleep mode \r\n");
        break;

    case BLE_PERIPHERAL_WAKEUP_SUCCEEDED:
        printf("Device Wake up succceded  : \r\n");
        if ( xSemaphoreGive( BleWakeUpSyncSemphr ) != pdTRUE)
        {
            /*We would not expect this call to fail because we must have obtained the semaphore to get here.*/
            Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
        }
        break;

    case BLE_PERIPHERAL_CONNECTED:
        printf("Device connected  : \r\n");
        break;

    case BLE_PERIPHERAL_DISCONNECTED:
        printf("Device Disconnected   : \r\n");
        break;
    case BLE_PERIPHERAL_ERROR:
        printf("BLE Error Event  : \r\n");
        break;

    default:
        /* assertion reason : invalid status of Bluetooth Device */
        assert(false);
        break;
    }
}

/**
 * @brief Callback function called on BLE send completion
 *
 * @param [in]  sendStatus : event to be send by BLE during communication.
 *
 */
static void BleAccelDataSentCallback(Retcode_T sendStatus)
{
	Retcode_T bleRetval = RETCODE_FAILURE;
	memset(Buffer, 0, BUFFERSIZE);
	if (RETCODE_OK != sendStatus)
    {
        printf("Error in transmitting the Accel Data over BLE. ERROR Code %ui  : \r\n", (unsigned int) sendStatus);
    }
    if ( xSemaphoreGive( SendCompleteSync ) != pdTRUE)
    {
        /*We would not expect this call to fail because we must have obtained the semaphore to get here.*/
        Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
    }
    else
    {
    	printf("Data Block successfully transmitted \r\n");
    	a = a + 4;
    	if (a < DataArrayLength)
    	{
    		sprintf((char*) Buffer, "%ld %ld %ld %ld", (long int) Data[a], (long int) Data[a+1], (long int) Data[a+2], (long int) Data[a+3]); /* keep transmitting data in blocks of 4 until array is cmpletely transmitted */
    		bleRetval = BidirectionalService_SendData((uint8_t*) Buffer, (uint8_t) BUFFERSIZE);
    		if (bleRetval == RETCODE_OK)
    		{
    			printf("Sending data was successful. \r\n");
    		}
    		else
			{
				printf("Sending data failed. \r\n");
			}
    	}
    	else
    	{
    		 /* reset all transfered data */
			 /*printf("All Data transmitted. \r\n");*/
			 memset(Data, 0, DataArrayLength);
			 i = 0;
			 a = 0;
    	}
    }
}

/** The function to send start or stop message to Bidirectional DataExchange service
 *  @param [in]  param1 : Unused, Reserved for future use
 *  @param [in]  param2 : Differentiates start and stop command
 */

/**
 * @brief Callback function called on data reception over BLE
 *
 * @param [in]  rxBuffer : Buffer in which received data to be stored.
 *
 * @param [in]  rxDataLength : Length of received data.
 */

static void BleDataReceivedCallBack(uint8_t *rxBuffer, uint8_t rxDataLength)
{
	uint8_t bleReceiveBuff[BLE_RECEIVE_BUFFER_SIZE];
	if (rxDataLength >= BLE_RECEIVE_BUFFER_SIZE)
	{
		printf("Data length received is invalid \n");
	}
	else
	{
		memset(bleReceiveBuff, 0, sizeof(bleReceiveBuff));
		memcpy(bleReceiveBuff, rxBuffer, rxDataLength);
		/* make sure that the received string is null-terminated */
		bleReceiveBuff[rxDataLength] = '\0';
	}

}

/**
 *  The function to register the bidirectional service
 */

static Retcode_T BiDirectionalServiceRegistryCallback(void)
{
    Retcode_T retval = RETCODE_OK;
    retval = BidirectionalService_Init(BleDataReceivedCallBack, BleAccelDataSentCallback);
    if (RETCODE_OK == retval)
    {
        retval = BidirectionalService_Register();
    }
    return (retval);
}

/** The function to get and transfer the accel data using BLE alpwise DataExchange service
 *
 * @brief        Gets the raw data from BMA280 Accel and transfer through the alphwise DataExchange service on BLE
 *
 * @param[in]   *param1: a generic pointer to any context data structure which will be passed to the function when it is invoked by the command processor.
 *
 * @param[in]    param2: a generic 32 bit value  which will be passed to the function when it is invoked by the command processor.
 */
static void BleAccelDataTransmit(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);
    Retcode_T bleRetval = RETCODE_OK;
    /* Return value for software timer */
    int8_t ReturnVal;
    memset(Buffer, 0, BUFFERSIZE);
/* buffer for data receive function */
	ReturnVal = sprintf((char*) Buffer, "%ld %ld %ld %ld", (long int) Data[a], (long int) Data[a+1], (long int) Data[a+2], (long int) Data[a+3]);
	printf("A total of %d characters were printed. \r\n", (int) ReturnVal);
	printf("%ld %ld %ld %ld\r\n", (long int) Data[a], (long int) Data[a+1], (long int) Data[a+2], (long int) Data[a+3]);
	bleRetval = BidirectionalService_SendData((uint8_t*) Buffer, (uint8_t) BUFFERSIZE);
	if (RETCODE_OK != bleRetval)
	/*Device Disconnect and data are discarded by Alphwise DataExchange Service */
	{
		/* clearing the flag */
		bleTransmitStatus = NUMBER_ZERO;

		printf("Not able to send accel data over BLE. \r\n");

	}
	switch (bleRetval)
	{
		case RETCODE_DATA_SEND_FAIL: printf("Failed to send data.\r\n");
			break;
		case RETCODE_NO_DEVICE_PAIRED: printf("No paired Device found.\r\n");
			break;
		case RETCODE_BLE_SEND_QUEUE_TIMEOUT: printf("Send queue timed out \r\n");
			break;
		case RETCODE_STACK_NO_RESOURCES: printf("The Stack has insufficient resources. \r\n");
			break;
		case RETCODE_BLETRANSPORT_UNKNOWN_ERROR: printf("An unknown error occured. \r\n");
			break;
		case RETCODE_OK: printf("Everything's gucci. \r\n");
			 break;
		default: printf("another known Error occured.");
	}
}


static void EnvironmentalMeasure(xTimerHandle xTimer)
{
	(void) xTimer;
	Retcode_T retval = RETCODE_OK;
	/*retval = Environmental_setPowerMode(xdkEnvironmental_BME280_Handle, ENVIRONMENTAL_BME280_POWERMODE_NORMAL);*/
	Environmental_Data_T DataStr = { INT32_C(0), UINT32_C(0), UINT32_C(0) };
	if (retval == RETCODE_OK)
		{
			if (Environmental_readData(xdkEnvironmental_BME280_Handle, &DataStr) == RETCODE_OK)
			{
				/*printf("Momentaner Durchlauf: %ld\r\n", (long int) i);*/
				Data[i] = DataStr.temperature;
				Data[i+144] = DataStr.pressure;
				Data[i+288] = DataStr.humidity;
				/* Interrupt-Handling tbd*/
				printf("Temperatur: %ld, Druck: %ld, relative Luftfeuchte: %ld\r\n", (long int) Data[i], (long int) Data[i+144], (long int) Data[i+288]);
				i++;
				if (i == SendInterval)
				{
					Retcode_T retVal = CmdProcessor_enqueue(AppCmdProcessorHandle, BleAccelDataTransmit, NULL, UINT32_C(0));
					    if (RETCODE_OK != retVal)
					    {
					        printf("Failed to Enqueue BleAccelDataTransmit to Application Command Processor \r\n");
					    }
					    else
					    {
					    	bleTransmitStatus = NUMBER_ONE;
					    	printf("BleAccelDataTransmit was enqueued. \r\n");
					    }
				}


			}
		}
	/*retval = Environmental_setPowerMode(xdkEnvironmental_BME280_Handle, ENVIRONMENTAL_BME280_POWERMODE_SLEEP);*/
}

/* global functions ********************************************************* */


/**
 * @brief To initializes SendDataOverBLE application for its proper operation
 */
Retcode_T init(void)
{
    Retcode_T retval = RETCODE_OK;
    Retcode_T retval2 = RETCODE_FAILURE;
    static_assert((portTICK_RATE_MS != 0), "Tick rate MS is zero");
    BleStartSyncSemphr = xSemaphoreCreateBinary();
    if (NULL == BleStartSyncSemphr)
    {
        return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
    }
    BleWakeUpSyncSemphr = xSemaphoreCreateBinary();
    if (NULL == BleWakeUpSyncSemphr)
    {
        return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
    }
    SendCompleteSync = xSemaphoreCreateBinary();
    if (NULL == SendCompleteSync)
    {
        return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
    }

    /*initialize accel sensor*/
    retval = AccelerometerSensorInit();
    retval2 = EnvironmentalSensorInit();

    if (RETCODE_OK == retval && retval2 == RETCODE_OK)
    {
    	gTimerHandle = xTimerCreate((char * const) "gTimerHandle", MS(4), pdTRUE, NULL, AccelerometerMeasure);
    	if (gTimerHandle == NULL)
    	{
    		printf("Creating gTimerHandle failed");
    		retval = RETCODE_FAILURE;
    	}
    	else
    	{
			EnvironmentalTimerHandle = xTimerCreate((char * const) "eTimerHandle", 1000, pdTRUE, NULL, EnvironmentalMeasure); /* should be ten minutes */
			if (EnvironmentalTimerHandle == NULL)
			{
				printf("Creating EnvironmentalTimerHandle failed.");
				retval = RETCODE_FAILURE;
			}
			else
			{
				retval = BlePeripheral_Initialize(BleEventCallBack, BiDirectionalServiceRegistryCallback);
				if (RETCODE_OK == retval)
				{
					retval = BlePeripheral_SetDeviceName((uint8_t*) XDK_BLE_DEVICE_NAME);
				}
				/* Powering on BLE module*/
				if (RETCODE_OK == retval)
				{
					retval = BlePeripheral_Start();
				}
				if (RETCODE_OK == retval)
				{
					if (pdTRUE != xSemaphoreTake(BleStartSyncSemphr, BLE_START_SYNC_TIMEOUT))
					{
						printf("Failed to Start BLE before timeout, Ble Initialization failed \n");
						retval = RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_TIME_OUT_ERROR);
					}
				}
				if (RETCODE_OK == retval)
				{
					retval = BlePeripheral_Wakeup();
				}
				/* Wake up BLE module*/
				if (RETCODE_OK == retval)
				{
					if (pdTRUE != xSemaphoreTake(BleWakeUpSyncSemphr, BLE_WAKEUP_SYNC_TIMEOUT))
					{
						printf("Failed to Wake up BLE before timeout, Ble Initialization failed \n");
						retval = RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_TIME_OUT_ERROR);
					}
				}
				if (RETCODE_OK == retval)
				{
					printf(" Ble Initialization succeded \n");
				}
				else
				{
					printf("Ble Initialization Failed \r\n");
				}
			}
		}
    }
    return retval;
}

/**
 * @brief This is a template function where the user can write his custom application.
 *
 */
void appInitSystem(void * CmdProcessorHandle, uint32_t param2)
{
	printf("appinitystem begin.\r\n");
    if (CmdProcessorHandle == NULL)
    {
        printf("Command processor handle is null \n\r");
        assert(false);
    }
    BCDS_UNUSED(param2);
    AppCmdProcessorHandle = (CmdProcessor_T *) CmdProcessorHandle;
    Retcode_T retVal = RETCODE_OK;
    retVal = init();
    if (RETCODE_OK == retVal)
    {
        printf("SendAccelerometerDataOverBle App Initialization completed successfully \r\n ");
    }
    else
    {
        printf("SendAccelerometerDataOverBle App Initialization failed \r\n ");
    }
    if(pdPASS != xTimerStart(EnvironmentalTimerHandle, 0))
    {
    	printf("check 1 failed\r\n");
    }
    else
    {
    	printf("check 1 successful\r\n");
    }
    if (pdPASS != xTimerStart(gTimerHandle, 0))
    {printf("check 2 failed\r\n");}
    else
    	{printf("check 2 successful.\r\n");}
    printf("All necessary timers were started successfully.\r\n");

}
/**@} */

/** ************************************************************************* */

