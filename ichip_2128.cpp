/*
 * ichip_2128.cpp
 *
 * Class to interface with the ichip 2128 based wifi adapter we're using on our board
 *
 Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice shall be included
 in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

#include "ichip_2128.h"

/*
 * Constructor. Assign serial interface to use for ichip communication
 */
ICHIPWIFI::ICHIPWIFI() {
	prefsHandler = new PrefHandler(ICHIP2128);

	uint8_t sys_type;
	sysPrefs->read(EESYS_SYSTEM_TYPE, &sys_type);
	if (sys_type == 3 || sys_type == 4)
		serialInterface = &Serial2;
	else //older hardware used this instead
		serialInterface = &Serial3; 

	commonName = "WIFI (iChip2128)";
}

/*
 * Constructor. Pass serial interface to use for ichip communication
 */
ICHIPWIFI::ICHIPWIFI(USARTClass *which) {
	prefsHandler = new PrefHandler(ICHIP2128);
	serialInterface = which;
}

/*
 * Initialization of hardware and parameters
 */
void ICHIPWIFI::setup() {

	Logger::info("add device: iChip 2128 WiFi (id: %X, %X)", ICHIP2128, this);

	TickHandler::getInstance()->detach(this);

	//MSEL pin
	pinMode(18, OUTPUT);
	digitalWrite(18, HIGH);

	//RESET pin
	pinMode(42, OUTPUT);
	digitalWrite(42, HIGH);

	tickCounter = 0;
	ibWritePtr = 0;
	psWritePtr = 0;
	psReadPtr = 0;
	listeningSocket = 0;

	lastSentTime = millis();
	lastSentState = IDLE;
	lastSentCmd = String("");

	activeSockets[0] = -1;
	activeSockets[1] = -1;
	activeSockets[2] = -1;
	activeSockets[3] = -1;

	state = IDLE;

	didParamLoad = false;
	didTCPListener = false;

	serialInterface->begin(115200);

	

	TickHandler::getInstance()->attach(this, CFG_TICK_INTERVAL_WIFI);

}

//A version of sendCmd that defaults to SET_PARAM which is what most of the code used to assume.
void ICHIPWIFI::sendCmd(String cmd) {
	sendCmd(cmd, SET_PARAM);
}

/*
 * Send a command to ichip. The "AT+i" part will be added.
 * If the comm channel is busy it buffers the command
 */
void ICHIPWIFI::sendCmd(String cmd, ICHIP_COMM_STATE cmdstate) {
	if (state != IDLE) { //if the comm is tied up then buffer this parameter for sending later
		sendingBuffer[psWritePtr].cmd = cmd;
		sendingBuffer[psWritePtr].state = cmdstate;
		psWritePtr = (psWritePtr + 1) & 63;
		if (Logger::isDebug()) {
			String temp = "Buffer cmd: " + cmd;
			Logger::debug(ICHIP2128, (char *)temp.c_str());
		}
	}
	else { //otherwise, go ahead and blast away
		serialInterface->write(Constants::ichipCommandPrefix);
		serialInterface->print(cmd);
		serialInterface->write(13);
		state = cmdstate;
		lastSentTime = millis();
		lastSentCmd = String(cmd);
		lastSentState = cmdstate;

		if (Logger::isDebug()) {
			String temp = "Send to ichip cmd: " + cmd;
			Logger::debug(ICHIP2128, (char *)temp.c_str());
		}
	}
}

void ICHIPWIFI::sendToSocket(int socket, String data) {
	char buff[6];
	sprintf(buff, "%03i", socket);
	String temp = "SSND%%:" + String(buff);
	sprintf(buff, ",%i:", data.length());
	temp = temp + String(buff) + data;
	sendCmd(temp, SEND_SOCKET);
}

/*
 * Periodic updates of parameters to ichip RAM.
 * Also query for changed parameters of the config page.
 */
//TODO: See the processing function below for a more detailed explanation - can't send so many setParam commands in a row
void ICHIPWIFI::handleTick() {
 
	
}

/*
 * Calculate the runtime in hh:mm:ss
   This runtime calculation is good for about 50 days of uptime.
   Of course, the sprintf is only good to 99 hours so that's a bit less time.
 */
char *ICHIPWIFI::getTimeRunning() {
	uint32_t ms = millis();
	int seconds = (int) (ms / 1000) % 60;
	int minutes = (int) ((ms / (1000 * 60)) % 60);
	int hours = (int) ((ms / (1000 * 3600)) % 24);
	sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
	return buffer;
}

/*
 * Handle a message sent by the DeviceManager.
 * Currently MSG_SET_PARAM is supported. The message should be a two element char pointer array 
 * containing the addresses of a two element char array. char *paramPtr[2] = { &param[0][0], &param[1][0] };
 * Element 0 of the base array (char param [2][20]; )should contain the name of the parameter to be changed
 * Element 1 of the base array should contain the new value to be set.   
 *
 *  sendMessage(DEVICE_WIFI, ICHIP2128, MSG_SET_PARAM,  paramPtr);	
 *    
 */
void ICHIPWIFI::handleMessage(uint32_t messageType, void* message) {
	Device::handleMessage(messageType, message);  //Only matters if message is MSG_STARTUP

	switch (messageType) {
  
	case MSG_SET_PARAM:{   //Sets a single parameter to a single value
  	        char **params = (char **)message;  //recast message as a two element array (params)		
              // Logger::console("Received Device: %s value %s",params[0], params[1]);
		setParam((char *)params[0], (char *)params[1]);
		break;
	}
	case MSG_CONFIG_CHANGE:{  //Loads all parameters to web site
		loadParameters();
		break;
        }
	case MSG_COMMAND:  //Sends a message to the WiReach module in the form of AT+imessage
		sendCmd((char *)message);
		break;
        }
    
}

/*
 * Determine if a parameter has changed
 * The result will be processed in loop() -> processParameterChange()
 */
void ICHIPWIFI::getNextParam() {
	sendCmd("WNXT", GET_PARAM); //send command to get next changed parameter
}

/*
 * Try to retrieve the value of the given parameter.
 */
void ICHIPWIFI::getParamById(String paramName) {
	sendCmd(paramName + "?", GET_PARAM);
}

/*
 * Set a parameter to the given string value
 */
void ICHIPWIFI::setParam(String paramName, String value) {
      sendCmd(paramName + "=\"" + value + "\"", SET_PARAM);
}

/*
 * Set a parameter to the given int32 value
 */
void ICHIPWIFI::setParam(String paramName, int32_t value) {
	sprintf(buffer, "%l", value);
	setParam(paramName, buffer);
}

/*
 * Set a parameter to the given uint32 value
 */
void ICHIPWIFI::setParam(String paramName, uint32_t value) {
	sprintf(buffer, "%lu", value);
	setParam(paramName, buffer);
}

/*
 * Set a parameter to the given sint16 value
 */
void ICHIPWIFI::setParam(String paramName, int16_t value) {
	sprintf(buffer, "%d", value);
	setParam(paramName, buffer);
}

/*
 * Set a parameter to the given uint16 value
 */
void ICHIPWIFI::setParam(String paramName, uint16_t value) {
	sprintf(buffer, "%d", value);
	setParam(paramName, buffer);
}

/*
 * Set a parameter to the given uint8 value
 */
void ICHIPWIFI::setParam(String paramName, uint8_t value) {
	sprintf(buffer, "%d", value);
	setParam(paramName, buffer);
}
/*
 * Set a parameter to the given float value
 */
void ICHIPWIFI::setParam(String paramName, float value, int precision) {
	char format[10];
	sprintf(format, "%%.%df", precision);
	sprintf(buffer, format, value);
	setParam(paramName, buffer);
}

/*
 * Called in the main loop (hopefully) in order to process serial input waiting for us
 * from the wifi module. It should always terminate its answers with 13 so buffer
 * until we get 13 (CR) and then process it.
 * 
 */

void ICHIPWIFI::loop() {
	
}

/*
 * Process the parameter update from ichip we received as a response to AT+iWNXT.
 * The response usually looks like this : key="value", so the key can be isolated
 * by looking for the '=' sign and the leading/trailing '"' have to be ignored.
 */
void ICHIPWIFI::processParameterChange(char *key) {
	
	
}

/*
 * Get parameters from devices and forward them to ichip.
 * This is required to initially set-up the ichip
 */
void ICHIPWIFI::loadParameters() {
	
}

DeviceType ICHIPWIFI::getType() {
	return DEVICE_WIFI;
}

DeviceId ICHIPWIFI::getId() {
	return (ICHIP2128);
}

void ICHIPWIFI::loadConfiguration() {
	WifiConfiguration *config = (WifiConfiguration *)getConfiguration();

	if (prefsHandler->checksumValid()) { //checksum is good, read in the values stored in EEPROM
		Logger::debug(ICHIP2128, "Valid checksum so using stored wifi config values");
		//TODO: implement processing of config params for WIFI
//		prefsHandler->read(EESYS_WIFI0_SSID, &config->ssid);
	}
}

void ICHIPWIFI::saveConfiguration() {
	WifiConfiguration *config = (WifiConfiguration *) getConfiguration();

	//TODO: implement processing of config params for WIFI
//	prefsHandler->write(EESYS_WIFI0_SSID, config->ssid);
//	prefsHandler->saveChecksum();
}
