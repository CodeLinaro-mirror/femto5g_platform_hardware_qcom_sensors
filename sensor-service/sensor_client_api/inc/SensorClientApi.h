/* Copyright (c) 2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SENSORCLIENTAPI_H
#define SENSORCLIENTAPI_H

#include <vector>
#include <string>
#include <functional>
#include <memory>
#include <sensors.h>
#include <SensorList.h>

using std::string;

namespace sensor_client
{

typedef enum {
        /** On Success **/
        SENSOR_RESPONSE_SUCCESS=0,
        /** Client is not registered to SHD **/
        SENSOR_ERROR_CLIENT_REGISTER_FAILED=-1,
        /** Client is not generated while registering the client **/
        SENSOR_ERROR_INVALID_CLIENT=-2,
        /** Invalid input parameteres from respective AP **/
        SENSOR_ERROR_INVALID_INPUT_PARAMETER=-3,
        /** Callback is null in respective API **/
        SENSOR_ERROR_CALLBACK_MISSING=-4,
        /** Not supported feature of sensor **/
        SENSOR_ERROR_NOT_SUPPORTED=-5,
        /** Physical Sensor Enable/Disable failed **/
        SENSOR_ERROR_CONTROL_FAILED=-6,
        /** Physical Sensor Config failed **/
        SENSOR_ERROR_CONFIG_FAILED=-7,
        /** Socket communication failed b/w SHD and client lib **/
        SENSOR_ERROR_IPC_FAILED=-8,
        /** No sensors supported in h/w **/
        SENSOR_ERROR_NO_SENSORS_FOUND=-9,
        /**No snesor is activated and configured**/
        SENSOR_ERROR_TRACKING_FAILED=-10,
        /** Unknown error **/
        SENSOR_ERROR_UNKNOWN=-11,
        /** Buffer is not supported by sensor**/
        SENSOR_ERROR_BUFFER_NOT_SUPPORTED=-12,
        /** Buffer is deleted**/
        SENSOR_ERROR_BUFFER_DELETED=-13,
        /** MLC Event Enable failed**/
        SENSOR_ERROR_MLC_EVENT_ENABLE_FAILED=-14,
        /** NO MLC case found**/
        SENSOR_ERROR_NO_MLC_CASE_FOUND=-15,
	/**Sensor No response from SHD timeout happens*/
	SENSOR_ERROR_NO_RESPONSE_FROM_SHD_TIMEOUT = -16,
}SensorRet;

typedef enum {
    /*Disable the sensor*/
     SENSOR_DISABLE = 0,
    /*Enable the sensor*/
     SENSOR_ENABLE,
    /*Low power mode*/
     SENSOR_LPM,
    /*High power mode*/
     SENSOR_HPM,
}sensor_state;

/** @brief Provides the capabilities of the system. <br/>

    @param capsMask: SensorCapabilitiesMask. <br/>
*/
typedef std::function<void(
    SensorCapabilitiesMask capsMask
)> CapabilitiesCb;

/** @brief
    BatchingCb is for notifying the client with updated sampling
    rate and batch count with sensor identifier.

    @param[out] sensor_id: sensor identifier.
    @param[out] sampling_rate: The reported sampling rate will be nearest supported sampling rate
                (sensor_list->sampling_rate) to requested sampling rate by sensor_config api).
    @param[out] batch_count: The reported batch count is multiplication of minBatchCount and
                should be nearest to or equal to requested batch count in sensor_config.
*/
typedef std::function<void(
    int sensor_id, float sampling_rate, int batch_count
)> BatchingCb;

/** @brief
    SensorDataReadCb is for receiveing sensor data.

    This callback is invoked for every sensor when batch count is
    reached to the requested batch count by that sensor.

    @param[out] sensor_id: sensor identifier.
    @param[out] sensors_event_t: the sensor events.
    @param[out] count: the total number of events.
*/
typedef std::function<void(
int sensor_id, const sensors_event_t *events, uint32_t count
)> SensorDataReadCb;


/** @brief
    SensormFifoReadCb is for receiveing mlc fifo sensor data.

    This callback is invoked when device resumes and data stored in sensor
    h/w fifo will be sent usings this callback

    @param[out] sensor_id: sensor identifier.
    @param[out] sensors_event_t: the sensor events.
    @param[out] count: the total number of events.
*/
typedef std::function<void(
int sensor_id, const sensors_event_t *events, uint32_t count
)> SensormFifoReadCb;

/** @brief
    SensorMLCEventCb is for receiveing sensor mlc case event.

    @param[out] case_name: the mlc case name.
    @param[out] event : the event generated by mlc case case_name.
*/
typedef std::function<void(
const char *case_name, struct mlc_event_data *event
)> SensorMLCEventCb;


/** @brief
    SensorTempReadCb is for receiveing temperature data.

    @param[out] temperature: temperature of sensor in celcius.
*/
typedef std::function<void(
float temperature
)> SensorTempReadCb;

/** @brief
    SensorBufferDataReadCb is for receiveing sensor buffer data.

    @param[out] sensors_event_t: the sensor events.
    @param[out] count: the total number of events.
*/
typedef std::function<void(
const sensors_event_t *events, uint32_t count
)> SensorBufferDataReadCb;

class SensorClientImpl;
class SensorClient
{
public:
    /** @brief
        Creates an instance of SensorClient object. <br/>
       @param
        capsCallback: If this callback is not null,
                      sensor_client::capsMask will
                      be reported via this callback after the
                      construction of the object.  <br/>
                      This callback is allowed to be be null. <br/>
		      This callback is used to notify about SHD status and also system power
		      status. <b/r>
    */
    SensorClient(CapabilitiesCb capsCallback);

    /** @brief Default destructor */
    virtual ~SensorClient();

    /*================================== Get Sensor List ================================== */
    /** @brief get the sensor list  <br/>

	It will returns the list of sensors supported by SHD, see “sensor_list” for details on
	how the sensors are defined. The “sensor_count” represesnt number of sensors in the list,
	basically sensor_count represensts array size of pointer to sensor_list, to access each
	sensor, need to loop the sensor_list from 0 to “sensor_count” times.
	ex: if sensor_count is 2 then
	for(int i=0; i< sensor_count; i++)  where “sensor_list[i]” represent represent each sensor
    **/

    /** Definition and Data Structures:

    struct sensor_list {

	char *name: A user-defined string in configuration file that represents the sensor.

	char *vendor: Vendor of the hardware part ex: “STMicroelectronics”, “BOSCH”

	int sensor_id: sensor_id that identifies this sensor. This sensor_id is used to reference
		     this sensor throughout the Client API.The sensor_id is derived from type of sensor,
		     where each type of sensor can have multiple sensor_idrs depends on sensors supported by HAL.
		     Ex: #define SENSOR_ID_STANDARD_START (0U)
		     SENSOR_ID_ACCELEROMETER = SENSOR_ID_STANDARD_START + SENSOR_TYPE_ACCELEROMETER
		     SENSOR_ID_ACCELEROMETER_WAKEUP =SENSOR_ID_ACCELEROMETER +SENSOR_TYPE_ACCELEROMETER

        int type: The type of the sensor. Ex: SENSOR_TYPE_ACCELEROMETER,SENSOR_TYPE_GYROSCOPE

	int range: sensors scalar config; accel in g units; for gyro in dps;

        int minBatchCount: Min batch count supported by this sensor, client can’t get the samples less than this count.

        int maxBatchCount: Max batch count supported by this sensor, client can’t get the samples more than this count.

        int maxSamplingRate: Max sampling rate supported by this sensor, client can’t the samples more than this rate.

        float odr[]: List of sampling rate supported by the sensor
    }
    **/
    /**SENSOR_TYPES:
		Sensor Types: Possible Sensor Types representeby “sensor_list->type”.
     * SENSOR_TYPE_ACCELEROMETER
     * reporting-mode: continuous
     *
     *  All values are in SI units (m/s^2) and measure the acceleration of the
     *  device minus the force of gravity.
     *
     #define SENSOR_TYPE_ACCELEROMETER                    (1)
     * SENSOR_TYPE_GYROSCOPE
     * reporting-mode: continuous
     *
     *  All values are in radians/second and measure the rate of rotation
     *  around the X, Y and Z axis.
     *
     #define SENSOR_TYPE_GYROSCOPE                        (4)
    **/
    /**
    @param[out] sensor_list: contains sensor specific information of available sensor.
    @param[out] sensor_count: total number of sensor supported.
    @returns:
	SENSOR_RESPONSE_SUCCESS                  : Success
	SENSOR_ERROR_CLIENT_REGISTER_FAILED      : client is not registered
	SENSOR_ERROR_INVALID_CLIENT              : client id is not generated
	SENSOR_ERROR_NO_SENSORS_FOUND            : no sensors supported by h/w
    **/
    int get_sensor_list(struct sensor_list **s, int *sensor_count);

    /*================================== Sensor Config ================================== */
    /** @brief Configure sensor for interested sampling rate and batch count <br/>

	Configure the sensor identified by sensor_id with given parameters.
	The sampling rate should be sampling rate supported by sensor_list and should be
	less than or equal to maxSampling rate reported by sensor list.
	The batching count should not exceed maxBatchCount provided by sensor_list.
	The OnBatchingCb will be invoked to notify client about sensor configured sampling rate and
	batch count.

        @param[in]  SampingRate: sampling rate of sensors. <br/>
        @param[in]  BatchCount: batch count of sensor events. <br/>
        @param[out] BatchingCb: callback to notify the updated sampling rate and batch count. <br/>

        @return:
		SENSOR_RESPONSE_SUCCESS               : Success
		SENSOR_ERROR_CLIENT_REGISTER_FAILED   : client is not registered
		SENSOR_ERROR_INVALID_CLIENT           : client id is not generated
		SENSOR_ERROR_IPC_FAILED               : failed socket communication b/w SHD
							and client lib.
		SENSOR_ERROR_NO_SENSORS_FOUND         : no sensors supported by h/w
		SENSOR_ERROR_INVALID_INPUT_PARAMETERS : invalid input of sensor_id or
						        Sampling rate or batch count.
		SENSOR_ERROR_CALLBACK_MISSING 	      : BatchingCb is null
		SENSOR_ERROR_CONFIG_FAILED            : physical sensor configuration fail.

    **/
    int sensor_config(int sensor_id, float SampingRate, int BatchCount, BatchingCb batchingCallback);

    /*================================== Sensor Control - Activate/Deactivate ================================== */
    /** @brief Activates or deactivates a sensor <b/r>

	Activates or deactivates the sensor, sensor’s sensor_id is defined by the sensor_id field
	of sensor_list structure used to activate and deactivate the respective sensor.
	state is set to SENSOR_ENABLE to activate the sensor or set to SENSOR_DISABLE to deactivate the sensor.
	Activate/Deactivate the sensor identified by sensor_id, in order to get the sensor data,
	enabling the sensor is not enough, the client must configure sensor by calling sensor_config API

        This API can also be used to put the sensor in low power mode and high power mode by setting
	sensor state to SENSOR_LPM and SENSOR_HPM respectively. The Swithcing sensor to different
	power is mode allowed only when physical sensor is disabled.
	The SENSOR by default will be in High Power Mode.

        @param[in] sensor_id: sensor identifier. <br/>
        @param[in] state: SENSOR_ENABLE to activate and SENSOR_DISABLE to deactivate the sensor. <br/>
			  SENSOR_LPM to put sensor in low power mode, SENSOR_HPM to put sensor in high
			  power mode. <br/>
        @return:
		SENSOR_RESPONSE_SUCCESS               : Success
		SENSOR_ERROR_CLIENT_REGISTER_FAILED   : client is not registered
		SENSOR_ERROR_INVALID_CLIENT           : client id is not generated
		SENSOR_ERROR_IPC_FAILED               : failed socket communication b/w SHD
					  		and client lib.
		SENSOR_ERROR_NO_SENSORS_FOUND         : no sensors supported by h/w
		SENSOR_ERROR_INVALID_INPUT_PARAMETERS : invalid input of sensor_id or enable.
		SENSOR_ERROR_ENABLE_FAILED            : Sensor enable or disable failed.
    **/
    int sensor_control(int sensor_id, sensor_state state);

    /*================================== Sensor Read Events ================================== */
    /** @brief Start reading sensor events <br/>

	register a callback to collect data from SHD and deliver to client. The data is passed to client
	by invoking the onSensorReadEventsCb, this callback is invoked for every sensor when batch
	count is reached requested batch count by that sensor.
	The client will get data only if respective sensor is configuread and activated
	successfully by client.
	This API need to call only once after configuring and activating sensor,
	the data is delivered through SensorDataReadCb callback.

        @param[in]  sensor_id: sensor identified to which callback need to registerd. <br/>
        @param[out] SensorDataReadCb: callback method invoked to deliver the sensor data. <br/>

        @return:
		SENSOR_RESPONSE_SUCCESS               : Success
		SENSOR_ERROR_CLIENT_REGISTER_FAILED   : client is not registered
		SENSOR_ERROR_INVALID_CLIENT           : client id is not generated
		SENSOR_ERROR_IPC_FAILED               : failed socket communication b/w SHD
							and client lib.
		SENSOR_ERROR_CALLBACK_MISSING 	      : SensorDataReadCb is null
		SENSOR_ERROR_TRACKING_FAILED          : no sensor is configured and activated
    **/
    int sensor_read_events(int sensor_id, SensorDataReadCb sensorReadCallback);

    /*================================== Sensor Request MLC Cases================================== */
    /** @brief Sensor MLC request to get mlc cases supported<br/>

        This API is used to get the mlc case supported on hardware.

        It will returns the list of mlc cases supported by hardware. The “mlc_case_count”
        represesnt number of mlc cases in the list, basically mlc_case_count represensts array size
        of pointer to mlc_case_list, to access each mlc case name, need to loop the mlc_case_list from
        0 to “mlc_case_count” times.

        struct sensor_mlc_case_list {
             // Name of mlc case.
             char *name;
        }

        @param[out] sensor_mlc_case_list: contains sensor mlc case specific information of available cases.
        @param[out] mlc_case_count: total number of mlc cases supported.

       @returns:
        SENSOR_RESPONSE_SUCCESS                  : Success
        SENSOR_ERROR_CLIENT_REGISTER_FAILED      : client is not registered
        SENSOR_ERROR_INVALID_CLIENT              : client id is not generated
        SENSOR_ERROR_NO_MLC_CASE_FOUND           : no mlc case supported by h/w
    **/
    int sensor_request_mlc_case(struct sensor_mlc_case_list **s, int *mlc_case_count);

    /*================================== Sensor MLC event enable/Disable ================================== */
    /** @brief Sensor mlc case enable/disable and register callback <br/>

        This API used to enable or disable event of mlc use case identified by mlc_case_name.
        mlc_case_name can get by mlc_case_list->name.

        @param[in]  mlc_case_name    : name of mlc use case need to enable/disable. <br/>
        @param[in]  enable           : 1 to activate the use case, 0 to deactivate use case. <br/>
        @param[out] SensorMLCEventCb : callback method invoked to deliver the mlc event data. <br/>

        @return:
                SENSOR_RESPONSE_SUCCESS               : Success
                SENSOR_ERROR_CLIENT_REGISTER_FAILED   : client is not registered
                SENSOR_ERROR_INVALID_CLIENT           : client id is not generated
                SENSOR_ERROR_IPC_FAILED               : failed socket communication b/w SHD
                                                        and client lib.
                SENSOR_ERROR_CALLBACK_MISSING         : SensorMLCEventCb is null
                SENSOR_ERROR_NO_MLC_CASE_FOUND        : no mlc case supported by h/w
    **/
    int sensor_mlc_event_enable(char *mlc_case_name, bool enable, SensorMLCEventCb sensorMlcEventCallback,
		    SensormFifoReadCb sensorMfifoReadCallback);


    /*================================== Sensor Buffer Read Events =================================*/
    /** @brief Start reading sensor buffer events <br/>

        @param[out] SensorBufferDataReadCb: Callback to receive sensor buffer events. <br/>
        @param[in]  enable: 1 to read the buffer data, 0 to delete the buffer data. <br/>

        @return:
                SENSOR_RESPONSE_SUCCESS               : Success
                SENSOR_ERROR_CLIENT_REGISTER_FAILED   : client is not registered
                SENSOR_ERROR_INVALID_CLIENT           : client id is not generated
                SENSOR_ERROR_IPC_FAILED               : failed socket communication b/w SHD
                                                        and client lib.
                SENSOR_ERROR_CALLBACK_MISSING         : SensorBufferDataReadCb is null

		SENSOR_ERROR_BUFFER_NOT_SUPPORTED     : if buffer not supported
                SENSOR_ERROR_BUFFER_DELETED           : if request is to read the bufffer data
							but its deleted
    **/
    int sensor_read_buffer_data(bool enable, SensorBufferDataReadCb sensorBufferReadCallback);

    /*================================== Sensor Temperature Read ==================================*/
    /** @brief read sensor temperature <br/>

	The sensor_read_temperature need to call whenever application want to read temperature.
	The temperaure is notified with SensorTempReadCb Callback.

        @param[out] SensorTempReadCb: Callback to receive sensor temperature. <br/>

        @return:
                SENSOR_RESPONSE_SUCCESS               : Success
                SENSOR_ERROR_CLIENT_REGISTER_FAILED   : client is not registered
                SENSOR_ERROR_INVALID_CLIENT           : client id is not generated
                SENSOR_ERROR_IPC_FAILED               : failed socket communication b/w SHD
                                                        and client lib.
                SENSOR_ERROR_CALLBACK_MISSING         : SensorTempReadCb is null
    **/
    int sensor_read_temperature(SensorTempReadCb sensorTempReadCallback);
private:
    /** Internal implementation for SensorClient */
    SensorClientImpl* mApiImpl;
};
} // namespace sensor_client
#endif /* SENSORCLIENTAPI_H */
