/*
 * SerialConsole.cpp
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

#include "SerialConsole.h"

extern PrefHandler *sysPrefs;

SerialConsole::SerialConsole(MemCache* memCache) :
		memCache(memCache), heartbeat(NULL) {
	init();
}

SerialConsole::SerialConsole(MemCache* memCache, Heartbeat* heartbeat) :
		memCache(memCache), heartbeat(heartbeat) {
	init();
}

void SerialConsole::init() {
	handlingEvent = false;

	//State variables for serial console
	ptrBuffer = 0;
	state = STATE_ROOT_MENU;
        loopcount=0;
        cancel=false;
      
}

void SerialConsole::loop() {
  if(!cancel)
    {
      if(loopcount++==350000){
        //DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_CONFIG_CHANGE, NULL);
        DeviceManager::getInstance()->updateWifi();
        cancel=true;  
        }
    } 
  if (handlingEvent == false) {
	if (SerialUSB.available()) {
        	serialEvent();
		}
	}
}

void SerialConsole::printMenu() {
	
	
	ICHIPWIFI *wifi = (ICHIPWIFI*) DeviceManager::getInstance()->getDeviceByType(DEVICE_WIFI);

	//Show build # here as well in case people are using the native port and don't get to see the start up messages
	SerialUSB.print("Build number: ");
	SerialUSB.println(CFG_BUILD_NUM);

	SerialUSB.println("System Menu:");
	SerialUSB.println();
	SerialUSB.println("Enable line endings of some sort (LF, CR, CRLF)");
	SerialUSB.println();
	SerialUSB.println("Short Commands:");
	SerialUSB.println("h = help (displays this message)");
	if (heartbeat != NULL) {
		SerialUSB.println("L = show raw analog/digital input/output values (toggle)");
	}
	SerialUSB.println("K = set all outputs high");
	SerialUSB.println("J = set all outputs low");
	//SerialUSB.println("U,I = test EEPROM routines");
	SerialUSB.println("E = dump system eeprom values");
	SerialUSB.println("z = detect throttle min/max, num throttles and subtype");
	SerialUSB.println("Z = save throttle values");
	SerialUSB.println("b = detect brake min/max");
	SerialUSB.println("B = save brake values");
	SerialUSB.println("p = enable wifi passthrough (reboot required to resume normal operation)");
	SerialUSB.println("S = show possible device IDs");
	SerialUSB.println("w = GEVCU 4.2 reset wifi to factory defaults, setup GEVCU ad-hoc network");
	SerialUSB.println("W = GEVCU 5.2 reset wifi to factory defaults, setup GEVCU as 10.0.0.1 Access Point");
	SerialUSB.println("s = Scan WiFi for nearby access points");
	SerialUSB.println();
	SerialUSB.println("Config Commands (enter command=newvalue). Current values shown in parenthesis:");
    SerialUSB.println();
    Logger::console("LOGLEVEL=%i - set log level (0=debug, 1=info, 2=warn, 3=error, 4=off)", Logger::getLogLevel());
   
	uint8_t systype;
	sysPrefs->read(EESYS_SYSTEM_TYPE, &systype);
	Logger::console("SYSTYPE=%i - Set board revision (Dued=2, GEVCU3=3, GEVCU4=4)", systype);

	DeviceManager::getInstance()->printDeviceList();




	

	

	

	
}

/*	There is a help menu (press H or h or ?)

 This is no longer going to be a simple single character console.
 Now the system can handle up to 80 input characters. Commands are submitted
 by sending line ending (LF, CR, or both)
 */
void SerialConsole::serialEvent() {
	int incoming;
	incoming = SerialUSB.read();
	if (incoming == -1) { //false alarm....
		return;
	}

	if (incoming == 10 || incoming == 13) { //command done. Parse it.
		handleConsoleCmd();
		ptrBuffer = 0; //reset line counter once the line has been processed
	} else {
		cmdBuffer[ptrBuffer++] = (unsigned char) incoming;
		if (ptrBuffer > 79)
			ptrBuffer = 79;
	}
}

void SerialConsole::handleConsoleCmd() {
	handlingEvent = true;

	if (state == STATE_ROOT_MENU) {
		if (ptrBuffer == 1) { //command is a single ascii character
			handleShortCmd();
		} else { //if cmd over 1 char then assume (for now) that it is a config line
			handleConfigCmd();
		}
	}
	handlingEvent = false;
}

/*For simplicity the configuration setting code uses four characters for each configuration choice. This makes things easier for
 comparison purposes.
 */
void SerialConsole::handleConfigCmd() {
	
	int i;
	int newValue;
	bool updateWifi = true;

	//Logger::debug("Cmd size: %i", ptrBuffer);
	if (ptrBuffer < 6)
		return; //4 digit command, =, value is at least 6 characters
	cmdBuffer[ptrBuffer] = 0; //make sure to null terminate
	String cmdString = String();
	unsigned char whichEntry = '0';
	i = 0;

	while (cmdBuffer[i] != '=' && i < ptrBuffer) {
	 cmdString.concat(String(cmdBuffer[i++]));
	}
	i++; //skip the =
	if (i >= ptrBuffer)
	{
		Logger::console("Command needs a value..ie TORQ=3000");
		Logger::console("");
		return; //or, we could use this to display the parameter instead of setting
	}

	
	// strtol() is able to parse also hex values (e.g. a string "0xCAFE"), useful for enable/disable by device id
	newValue = strtol((char *) (cmdBuffer + i), NULL, 0);

	cmdString.toUpperCase();
	
  if (cmdString == String("LOGLEVEL")) {
		switch (newValue) {
		case 0:
			Logger::setLoglevel(Logger::Debug);
			Logger::console("setting loglevel to 'debug'");
			break;
		case 1:
			Logger::setLoglevel(Logger::Info);
			Logger::console("setting loglevel to 'info'");
			break;
		case 2:
			Logger::console("setting loglevel to 'warning'");
			Logger::setLoglevel(Logger::Warn);
			break;
		case 3:
			Logger::console("setting loglevel to 'error'");
			Logger::setLoglevel(Logger::Error);
			break;
		case 4:
			Logger::console("setting loglevel to 'off'");
			Logger::setLoglevel(Logger::Off);
			break;
		}
		sysPrefs->write(EESYS_LOG_LEVEL, (uint8_t)newValue);
		sysPrefs->saveChecksum();

	} else if (cmdString == String("WIREACH")) {
		DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)(cmdBuffer + i));
		Logger::info("sent \"AT+i%s\" to WiReach wireless LAN device", (cmdBuffer + i));
                DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"DOWN");	
		updateWifi = false;
	} else if (cmdString == String("SSID")) {
		String cmdString = String();
    	        cmdString.concat("WLSI");
   		cmdString.concat('=');
		cmdString.concat((char *)(cmdBuffer + i));
                Logger::info("Sent \"%s\" to WiReach wireless LAN device", (cmdString.c_str()));
       		DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)cmdString.c_str());
                DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"DOWN");	
		updateWifi = false;
        } else if (cmdString == String("IP")) {
		String cmdString = String();
    	        cmdString.concat("DIP");
   		cmdString.concat('=');
		cmdString.concat((char *)(cmdBuffer + i));
                Logger::info("Sent \"%s\" to WiReach wireless LAN device", (cmdString.c_str()));
       		DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)cmdString.c_str());
                DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"DOWN");	
		updateWifi = false;
 } else if (cmdString == String("CHANNEL")) {
		String cmdString = String();
    	        cmdString.concat("WLCH");
   		cmdString.concat('=');
		cmdString.concat((char *)(cmdBuffer + i));
                Logger::info("Sent \"%s\" to WiReach wireless LAN device", (cmdString.c_str()));
       		DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)cmdString.c_str());
                DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"DOWN");	
		updateWifi = false;
	} else if (cmdString == String("SECURITY")) {
		String cmdString = String();
    	        cmdString.concat("WLPP");
   		cmdString.concat('=');
		cmdString.concat((char *)(cmdBuffer + i));
                Logger::info("Sent \"%s\" to WiReach wireless LAN device", (cmdString.c_str()));
       		DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)cmdString.c_str());
                DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"DOWN");	
		updateWifi = false;
	
   } else if (cmdString == String("PWD")) {
		String cmdString = String();
    	        cmdString.concat("WPWD");
   		cmdString.concat('=');
		cmdString.concat((char *)(cmdBuffer + i));
                Logger::info("Sent \"%s\" to WiReach wireless LAN device", (cmdString.c_str()));
       		DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)cmdString.c_str());
                DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"DOWN");	
		updateWifi = false;
	
	} else if (cmdString == String("OUTPUT") && newValue<8) {
                int outie = getOutput(newValue);
                Logger::console("DOUT%d,  STATE: %d",newValue, outie);
                if(outie)
                  {
                    setOutput(newValue,0);
                   // motorController->statusBitfield1 &= ~(1 << newValue);//Clear
                  }
                   else
                     {
                       setOutput(newValue,1);
                       // motorController->statusBitfield1 |=1 << newValue;//setbit to Turn on annunciator
		      }
                  
             
        Logger::console("DOUT0:%d, DOUT1:%d, DOUT2:%d, DOUT3:%d, DOUT4:%d, DOUT5:%d, DOUT6:%d, DOUT7:%d", getOutput(0), getOutput(1), getOutput(2), getOutput(3), getOutput(4), getOutput(5), getOutput(6), getOutput(7));
	} else if (cmdString == String("NUKE")) {
		if (newValue == 1) 
		{   //write zero to the checksum location of every device in the table.
			//Logger::console("Start of EEPROM Nuke");
			uint8_t zeroVal = 0;
			for (int j = 0; j < 64; j++) 
			{
				memCache->Write(EE_DEVICES_BASE + (EE_DEVICE_SIZE * j), zeroVal);
				memCache->FlushAllPages();
			}			
			Logger::console("Device settings have been nuked. Reboot to reload default settings");
		}
	} else {
		Logger::console("Unknown command");
		updateWifi = false;
	}
	// send updates to ichip wifi
	if (updateWifi)
		DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_CONFIG_CHANGE, NULL);
}

void SerialConsole::handleShortCmd() {
	uint8_t val;

	DeviceManager *deviceManager = DeviceManager::getInstance();

	switch (cmdBuffer[0]) {
	case 'h':
	case '?':
	case 'H':
		printMenu();
		break;
	case 'L':
		
		break;
	case 'U':
		Logger::console("Adding a sequence of values from 0 to 255 into eeprom");
		for (int i = 0; i < 256; i++) {
			memCache->Write(1000 + i, (uint8_t) i);
		}
		Logger::info("Flushing cache");
		memCache->FlushAllPages(); //write everything to eeprom
		memCache->InvalidateAll(); //remove all data from cache
		Logger::console("Operation complete.");
		break;
	case 'I':
		Logger::console("Retrieving data previously saved");
		for (int i = 0; i < 256; i++) {
			memCache->Read(1000 + i, &val);
			Logger::console("%d: %d", i, val);
		}
		break;
	case 'E':
		Logger::console("Reading System EEPROM values");
		for (int i = 0; i < 256; i++) {
			memCache->Read(EE_SYSTEM_START + i, &val);
			Logger::console("%d: %d", i, val);
		}
		break;
	case 'K': //set all outputs high
		for (int tout = 0; tout < NUM_OUTPUT; tout++) setOutput(tout, true);
		Logger::console("all outputs: ON");
		break;
	case 'J': //set the four outputs low
		for (int tout = 0; tout < NUM_OUTPUT; tout++) setOutput(tout, false);
		Logger::console("all outputs: OFF");
		break;
	
	
	
	case 'p':
		Logger::console("PASSTHROUGH MODE - All traffic Serial3 <-> SerialUSB");
		//this never stops so basically everything dies. you will have to reboot.
		int inSerialUSB, inSerial3;
		while (1 == 1) {
			inSerialUSB = SerialUSB.read();
			inSerial3 = Serial3.read();
			if (inSerialUSB > -1) {
				Serial3.write((char) inSerialUSB);
			}
			if (inSerial3 > -1) {
				SerialUSB.write((char) inSerial3);
			}
		}
		break;

	case 'S':
		//there is not really any good way (currently) to auto generate this list
		//the information just isn't stored anywhere in code. Perhaps we might
		//think to change that. Otherwise you must remember to update here or
		//nobody will know your device exists. Additionally, these values are
		//decoded into decimal from their hex specification in DeviceTypes.h
	
		Logger::console("WIFI (iChip2128) = %X", ICHIP2128);
		
		break;
	case 's':
		Logger::console("Finding and listing all nearby WiFi access points");
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"RP20");
		break;
	case 'W':
		Logger::console("Setting Wifi Adapter to WPS mode (make sure you press the WPS button on your router)");
		// restore factory defaults and give it some time
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"AWPS");
		break;
	case 'w':
		Logger::console("Resetting wifi to factory defaults and setting up GEVCU4.2 Ad Hoc network");
		// restore factory defaults and give it some time
        // pinMode(43,OUTPUT);
        //  digitalWrite(43, LOW); //Pin 43 held low for 5 seconds puts Version 4.2 in Recovery mode
        //  delay(6000);
        //  digitalWrite(43, HIGH);
       // delay(3000);
                deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"FD");//Reset
		  delay(2000);
                deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"HIF=1");  //Set for RS-232 serial.
		  delay(1000);
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"BDRA");//Auto baud rate selection
		  delay(1000);
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"WLCH=9"); //use whichever channel an AP wants to use
		  delay(1000);
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"WLSI=!GEVCU"); //set for GEVCU aS AP.
		  delay(1000);
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"DIP=192.168.3.10"); //enable IP 
		  delay(1000);
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"DPSZ=8"); //set DHCP server for 8
		  delay(1000);
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"RPG=secret"); // set the configuration password for /ichip
		  delay(1000);
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"WPWD=secret"); // set the password to update config params
		  delay(1000);
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"AWS=1"); //turn on web server 
		  delay(1000);
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_COMMAND, (void *)"DOWN"); //cause a reset to allow it to come up with the settings
		  delay(5000); // a 5 second delay is required for the chip to come back up ! Otherwise commands will be lost
  
		deviceManager->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_CONFIG_CHANGE, NULL); // reload configuration params as they were lost
		Logger::console("Wifi 4.2 initialized");
		break;
        
	case 'X':
		setup(); //this is probably a bad idea. Do not do this while connected to anything you care about - only for debugging in safety!
		break;
	}
}
