#include <SoftwareSerial.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <MemoryFree.h>
#include <SPI.h>

#define F_CPU 20000000L

#define MGMT_IFACE_DATA_RATE 500000
#define PANEL_DATA_RATE 9600

#define PANEL_0_DISCONNECTION_THRESHOLD (1000/125)

#define DLTC_CHECK_INTERVAL 500
#define PANEL_SCAN_INTERVAL (1000/8)

#define READER_PORT_0_RX_PIN A0
#define READER_PORT_0_TX_PIN A1

#define ETHERNET_RESET A2
#define NC0 A3

#define LED_0_PIN 3
#define LED_1_PIN 5
#define LED_2_PIN 6

#define EXT_0_PIN 8 //main entry door motion detector
#define EXT_1_PIN 7 //side entry door motion detector
#define EXT_2_PIN 4 //panel 0 controlled, main entry door control
#define EXT_3_PIN 2 //side entry door control

#define EEPROM_SIZE 128000
#define EEPROM_PAGE_SIZE 128

#define READ_LOG_COMMAND 'R'
#define READ_LOG_PART 'B'
#define ERASE_LOG 'Q'
#define WRITE_LIST_COMMAND 'L'
#define WRITE_LIST_PART 'S'
#define READ_LIST_PART 'K'
#define WRITE_RTC_COMMAND 'C'
#define READ_TIME_COMMAND 'T'
#define NULL_LIST_SEGMENT 'N'
#define GET_DOOR_CONTROL_OUTPUTS 'Z'
#define SET_DOOR_CONTROL_OUTPUTS 'A'

#define UNLOCKED LOW 
#define LOCKED HIGH

#define MOTION_DETECTED LOW
#define NO_MOTION HIGH

const uint8_t rtc_addr = 0b1101000;
const uint8_t id_eeprom_ctrl = 0b1010000;

const uint8_t mgmt_iface_entry_cmd = '?';

const uint8_t master_ready = 'M';
const uint8_t master_acknowledge = 'A';

const uint8_t skip_panel_command = 'B';

const uint8_t reader_auth_codes[] = {'D', 'E', 'F'}; 

const uint16_t read_timeout = 4000; //in milliseconds
const uint16_t panel_scan_timeout = 500; //in milliseconds
const uint16_t programming_operation_timeout = 2000; //in milliseconds

uint32_t ext_0_hold_duration = 0;
uint32_t ext_0_hold_timer = 0;
uint8_t ext_0_hold_type = 0;
uint16_t ext_0_pulse_interval = 0;
uint32_t ext_0_pulse_timer = 0;

uint8_t panel_0_disconnection_counter = 0;
uint8_t panel_0_disconnection_flag = false; 

uint32_t override_pulse_timer = 0;

uint8_t output_pulse_duration = 125;

SoftwareSerial reader_port_0 = SoftwareSerial(READER_PORT_0_RX_PIN, READER_PORT_0_TX_PIN);

uint32_t dltc_check_timer = 0;
uint32_t panel_scan_timer = 0;

uint8_t door_0_force_unlock = false;
uint8_t door_1_force_unlock = false;

uint32_t scan_timer;

void setup()
{	
	pinMode(9, INPUT_PULLUP); //the only unused pin
    pinMode(NC0, INPUT_PULLUP);
	
	pinMode(LED_0_PIN, OUTPUT);
	pinMode(LED_1_PIN, OUTPUT);
	pinMode(LED_2_PIN, OUTPUT);
	
	pinMode(EXT_0_PIN, INPUT_PULLUP);
	pinMode(EXT_1_PIN, INPUT_PULLUP);
	pinMode(EXT_2_PIN, OUTPUT);
	pinMode(EXT_3_PIN, OUTPUT);
	
	digitalWrite(EXT_2_PIN, LOCKED); //doors are locked by default
	digitalWrite(EXT_3_PIN, LOCKED); //doors are locked by default
	
	Serial.begin(MGMT_IFACE_DATA_RATE);

      

    reader_port_0.begin(PANEL_DATA_RATE);
    reader_port_0.listen();
	
	Wire.begin();
	
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	
	digitalWrite(SS, LOW);
	delay(10);
	digitalWrite(SS, HIGH);
	
	checkOSF();

    resetNetworkAdapter();
} 

void loop()
{
    idle_m(0);
    
    if((uint32_t)((long)millis()-dltc_check_timer) >= DLTC_CHECK_INTERVAL)
    {
        dltc_check_timer = millis();

        if(checkDoorLockTimeConditions())
        {
	        //both doors remain unlocked
            
            digitalWrite(EXT_2_PIN, UNLOCKED);
            digitalWrite(EXT_3_PIN, UNLOCKED);
        }
	    else
        {
		    //both doors default to locked
            if(door_0_force_unlock)
                digitalWrite(EXT_2_PIN, UNLOCKED);
            else
                if(digitalRead(EXT_0_PIN) != MOTION_DETECTED)
                    if(ext_0_hold_duration > 0)
                    {
                        if((uint32_t)((long)millis()-ext_0_hold_timer) >= ext_0_hold_duration) //if timer 0 has been active for ext_0_hold_duration then turn off the relay, clear the led, and set ext_0_hold_duration to 0
                        {
                            digitalWrite(EXT_2_PIN, LOCKED);
                            digitalWrite(LED_1_PIN, LOW);
                        
                            ext_0_hold_duration = 0;
                        }
                    }
                    else
                        digitalWrite(EXT_2_PIN, LOCKED);

            if(door_1_force_unlock)
                digitalWrite(EXT_3_PIN, UNLOCKED);
            else
                if(digitalRead(EXT_1_PIN) != MOTION_DETECTED)
                    digitalWrite(EXT_3_PIN, LOCKED);
        }
    }
			
	//check to see if we need to enter the management interface
	if(Serial.available())
	{
		digitalWrite(LED_0_PIN, HIGH);
		
		if(Serial.read() == mgmt_iface_entry_cmd) // question mark
		{
            if(sysMgmtMode() < 1)
            {
			    resetNetworkAdapter();
			    while(Serial.available() && Serial.peek() != mgmt_iface_entry_cmd) //clear the RX buffer
			        Serial.read();
            }
		}
		else
        {
			resetNetworkAdapter();
           while(Serial.available() && Serial.peek() != mgmt_iface_entry_cmd) //clear the RX buffer
           Serial.read();
        }
		
		digitalWrite(LED_0_PIN, LOW);
	}
	
    if(!door_0_force_unlock)
    {
        if((uint32_t)((long)millis()-panel_scan_timer) >= PANEL_SCAN_INTERVAL)
        {
	        panel_scan_timer = millis();
            
        
            if(panel_0_disconnection_flag && panel_0_disconnection_counter > 0)
                panel_0_disconnection_counter--;
            else
            {
                //reader_port_0.begin(PANEL_DATA_RATE);
                //reader_port_0.listen();

                while(reader_port_0.available())
                    reader_port_0.read();

                reader_port_0.write(master_ready);
        	    
                scan_timer = millis();
	            while(!reader_port_0.available())
		            if((uint32_t)((long)millis()-scan_timer) >= panel_scan_timeout)
                        break;
                    else
			            idle_u(208); //softwareserial seems to work better with this delay

                if(reader_port_0.available())
                {
                    panel_0_disconnection_flag = false;
                    panel_0_disconnection_counter = 0;
                
                    if(reader_port_0.peek() != skip_panel_command)
                        handleRequest(reader_port_0, reader_port_0.read());
                    else
                        reader_port_0.read();
                }
                else
                    if(panel_0_disconnection_flag)
                        panel_0_disconnection_counter = PANEL_0_DISCONNECTION_THRESHOLD;
                    else
                        if(++panel_0_disconnection_counter == PANEL_0_DISCONNECTION_THRESHOLD)
                            panel_0_disconnection_flag = true;
            }
        }
    }
}

uint32_t idle_timer = 0;

void idle_m(uint16_t idle_time)
{
    idle_timer = millis();
    
    if(digitalRead(EXT_0_PIN) == MOTION_DETECTED)
        digitalWrite(EXT_2_PIN, UNLOCKED);

    if(digitalRead(EXT_1_PIN) == MOTION_DETECTED)
        digitalWrite(EXT_3_PIN, UNLOCKED);

    if(idle_time)
    {        
        while((uint32_t)((long)millis()-idle_timer) < idle_time)
        {
            if(digitalRead(EXT_0_PIN) == MOTION_DETECTED)
                digitalWrite(EXT_2_PIN, UNLOCKED);

            if(digitalRead(EXT_1_PIN) == MOTION_DETECTED)
                digitalWrite(EXT_3_PIN, UNLOCKED);
        }
    }
}

void idle_u(uint16_t idle_time)
{
    idle_timer = micros();

    if(digitalRead(EXT_0_PIN) == MOTION_DETECTED)
        digitalWrite(EXT_2_PIN, UNLOCKED);

    if(digitalRead(EXT_1_PIN) == MOTION_DETECTED)
        digitalWrite(EXT_3_PIN, UNLOCKED);

    if(idle_time)
    {
        while((uint32_t)((long)micros()-idle_timer) < idle_time)
        {
            if(digitalRead(EXT_0_PIN) == MOTION_DETECTED)
                digitalWrite(EXT_2_PIN, UNLOCKED);

            if(digitalRead(EXT_1_PIN) == MOTION_DETECTED)
                digitalWrite(EXT_3_PIN, UNLOCKED);
        }
    }
}

uint8_t checkDoorLockTimeConditions()
{
  //the front doors should be open from 7:45am to 5:45pm
    uint8_t current_hour = getHour();
    uint8_t current_minute = getMinute();
    uint8_t current_day = getDay();

    if(current_day > 1 && current_day < 7)
        if(current_hour >= 7 && current_hour <= 17)
            if(current_hour > 7 || current_minute >= 45)
                if(current_hour < 17 || current_minute <= 45)
                    return 1;

    return 0;
}

uint8_t handleRequest(SoftwareSerial &panel_port, uint8_t reader_id)
{
	uint8_t uid_length, auth_code;
	uint32_t timeout_timer;
								   
	digitalWrite(LED_1_PIN, HIGH);
						   
	panel_port.write(master_acknowledge); //send master acknowledge
	
	//wait to receive the uid length
	timeout_timer = millis();

	while(!panel_port.available())
		if((uint32_t)((long)millis()-timeout_timer) >= read_timeout)
        {
			digitalWrite(LED_1_PIN, LOW);
            return 0;
        }
		else
			idle_u(208);
			
	uid_length = panel_port.read(); //read the length of the uid
	
	panel_port.write(master_acknowledge); //send master acknowledge
	
	uint8_t card_uid[uid_length];
	
	timeout_timer = millis();
	while(panel_port.available() < uid_length)
		if((uint32_t)((long)millis()-timeout_timer) >= read_timeout)
		{
    		digitalWrite(LED_1_PIN, LOW);
    		return 0;
		}
		else
			idle_u(833);
			
	for(uint8_t index_counter = 0; index_counter < uid_length; index_counter++)
		card_uid[index_counter] = panel_port.read();
		
	auth_code = requestEntry(card_uid, uid_length, reader_id); //search id list and determine if access will be granted for card holder
	
    panel_port.write(reader_auth_codes[auth_code]);
	
	if(auth_code == 1)
    {
        ext_0_hold_timer = millis();    
        digitalWrite(EXT_2_PIN, UNLOCKED);
    }
	
	writeLogEntry(card_uid, uid_length, reader_id, auth_code);

    digitalWrite(LED_1_PIN, LOW);

    return 1;
}

int8_t sysMgmtMode()
{
	uint32_t timeout_timer;
	
	timeout_timer = millis();
	while(!Serial.available())
	{
		if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
			return -1; //leave system management mode
		else
			idle_m(0);
	}

	uint8_t mgmt_command = Serial.read();
	
	if(mgmt_command == READ_LOG_COMMAND)
	{
		idle_m(500); //make sure the pc is ready
		
		uint32_t log_address = 80;
		
		digitalWrite(SS, LOW);
		SPI.transfer(0b00000011);
		SPI.transfer((log_address >> 16) & 0xFF);
		SPI.transfer((log_address >> 8) & 0xFF);
		SPI.transfer(log_address & 0xFF);

        uint8_t * log_buffer = (uint8_t *)malloc(16);
		
		//page size = 128 eeprom size = 128000
		for(uint16_t log_counter = 0; log_counter < 7995; log_counter++)
        {
			for(uint8_t byte_counter = 0; byte_counter < 16; byte_counter++)
				log_buffer[byte_counter] = SPI.transfer(0);
            
            Serial.write(log_buffer, 16);
            Serial.flush();

            idle_m(0);
        }

        free(log_buffer);
		
		digitalWrite(SS, HIGH);
		
		//write the extra bytes for compatibility
		for(uint8_t exbctr = 0; exbctr < 80; exbctr++)
			Serial.write(0);

        return 0;
	}
	else
		if(mgmt_command == WRITE_LIST_COMMAND)
		{
			Serial.print("?");
            
            for(uint32_t address_counter = 0; address_counter < EEPROM_SIZE; address_counter+=EEPROM_PAGE_SIZE)
			{
				timeout_timer = millis();
				while(Serial.available() < 128)
				{
					if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
						return -1; //leave system management mode
					else
						idle_m(0);
				}
				
				Wire.beginTransmission((address_counter > 0xFFFF ? id_eeprom_ctrl|1 : id_eeprom_ctrl));
				Wire.write( (address_counter > 0xFFFF ? address_counter-0xFFFF : address_counter)>>8 );
				Wire.write( (address_counter > 0xFFFF ? address_counter-0xFFFF : address_counter)&0xFF );
				
				for(uint8_t byte_counter = 0; byte_counter < 128; byte_counter++)
					Wire.write(Serial.read());

				Wire.endTransmission();

                Serial.print("?");
				
				idle_m(5); //wait for the eeprom to finish writing its buffer
			}

            return 0;
		}
		else
			if(mgmt_command == WRITE_RTC_COMMAND)
			{
				timeout_timer = millis();
				while(Serial.available() < 7)
				{
					if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
						return -1; //leave system management mode
					else
						idle_m(0);
				}
				
				Wire.beginTransmission(rtc_addr);
				Wire.write(0);
				
				for(uint8_t byte_counter = 0; byte_counter < 7; byte_counter++)
					Wire.write(Serial.read());
				
				Wire.endTransmission();
			}
            else
                if(mgmt_command == WRITE_LIST_PART)
                {
                    uint8_t * rx_buffer = (uint8_t*)malloc(128);
                    
                    Serial.print("??");

                    timeout_timer = millis();
                    while(Serial.available() < 2)
                    {
                        if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
                        {
                            free(rx_buffer);
                            return -1; //leave system management mode
                        }
                        else
                            idle_m(0);
                    }

                    uint32_t address_counter = (((uint32_t)Serial.read() << 8) | Serial.read())*EEPROM_PAGE_SIZE;

                    if(address_counter >= EEPROM_SIZE)
                    {
                        free(rx_buffer);
                        return -1;
                    }

                    while(Serial.available() < 129)
                    {
                        if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
                        {
                            free(rx_buffer);
                            return -1; //leave system management mode
                        }
                        else
                        idle_m(0);
                    }
 
                    Serial.readBytes((char*)rx_buffer, 128);
                    
                    uint8_t crc = Serial.read();

                    if(CRC8(rx_buffer, 128) != crc)
                       {
                           free(rx_buffer);
                           return -1;
                       }
                    
                    Wire.beginTransmission((address_counter > 0xFFFF ? id_eeprom_ctrl|1 : id_eeprom_ctrl));
                    Wire.write( (address_counter > 0xFFFF ? address_counter-0xFFFF : address_counter)>>8 );
                    Wire.write( (address_counter > 0xFFFF ? address_counter-0xFFFF : address_counter)&0xFF );
                    
                    Wire.write(rx_buffer, 128);

                    Wire.endTransmission();
                    
                    free(rx_buffer);

                    idle_m(6); //wait for the eeprom to finish writing its buffer

                    //verify that the page was stored correctly

                     Wire.beginTransmission(address_counter > 0xFFFF ? id_eeprom_ctrl|1 : id_eeprom_ctrl);
                     Wire.write((address_counter > 0xFFFF ? address_counter-0xFFFF : address_counter) >> 8);
                     Wire.write((address_counter > 0xFFFF ? address_counter-0xFFFF : address_counter) & 0xFF);
                     Wire.endTransmission(0);
                     
                     Wire.requestFrom(address_counter > 0xFFFF ? id_eeprom_ctrl|1 : id_eeprom_ctrl, EEPROM_PAGE_SIZE); //request 1 page of memory

                    if(CRC8(Wire.rxBuffer, 128) != crc)
                           return -1;

                    Wire.rxBufferIndex = 0;
                    //

                    Serial.print("!");

                    return 1;
                }
                else
                    if(mgmt_command == READ_LOG_PART)
                    {
                        Serial.print("?");
                        
                        timeout_timer = millis();
                        while(Serial.available() < 2)
                        {
                            if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
                                return -1; //leave system management mode
                            else
                                idle_m(0);
                        }

                        uint32_t log_address = ((((uint32_t)Serial.read() << 8) | Serial.read())*16) + 144;
                        
                        digitalWrite(SS, LOW);
                        SPI.transfer(0b00000011);
                        SPI.transfer((log_address >> 16) & 0xFF);
                        SPI.transfer((log_address >> 8) & 0xFF);
                        SPI.transfer(log_address & 0xFF);

                        uint8_t * log_buffer = (uint8_t *)malloc(16);
                        
                        for(uint8_t byte_counter = 0; byte_counter < 16; byte_counter++)
                            log_buffer[byte_counter] = SPI.transfer(0);
                        
                        Serial.write(log_buffer, 16);

                        Serial.write(CRC8(log_buffer, 16));

                        digitalWrite(SS, HIGH);

                        Serial.flush();

                        free(log_buffer);
                        
                        

                        timeout_timer = millis();
                        while(!Serial.available())
                        {
                            if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
                            return -1; //leave system management mode
                            else
                            idle_m(0);
                        }

                        if(Serial.read() == '!')
                        return 1;
                        else
                        return -1;
                    }
                    else
                        if(mgmt_command == READ_LIST_PART)
                        {
                            Serial.print("?");

                            timeout_timer = millis();
                            while(Serial.available() < 2)
                            {
                                if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
                                    return -1; //leave system management mode
                                else
                                    idle_m(0);
                            }

                            uint32_t address_counter = (((uint32_t)Serial.read() << 8) | Serial.read())*EEPROM_PAGE_SIZE;

                            Wire.beginTransmission(address_counter > 0xFFFF ? id_eeprom_ctrl|1 : id_eeprom_ctrl);
                            Wire.write((address_counter > 0xFFFF ? address_counter-0xFFFF : address_counter) >> 8);
                            Wire.write((address_counter > 0xFFFF ? address_counter-0xFFFF : address_counter) & 0xFF);
                            Wire.endTransmission(0);
                            
                            Wire.requestFrom(address_counter > 0xFFFF ? id_eeprom_ctrl|1 : id_eeprom_ctrl, EEPROM_PAGE_SIZE); //request 1 page of memory

                            Serial.write(Wire.rxBuffer, 128);

                            Serial.write(CRC8(Wire.rxBuffer, 128));

                            Wire.rxBufferIndex = 0;

                            Serial.print("?");

                            Serial.flush();

                            timeout_timer = millis();
                            while(!Serial.available())
                            {
                                if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
                                return -1; //leave system management mode
                                else
                                idle_m(0);
                            }

                            if(Serial.read() == '!')
                            return 1;
                            else
                            return -1;
                        }
                        else
                            if(mgmt_command == READ_TIME_COMMAND)
                            {
                                uint8_t  rtc_registers[7];
                                
                                Wire.beginTransmission(rtc_addr);
                                Wire.write(0);
                                Wire.endTransmission(0);
                                
                                Wire.requestFrom((uint8_t)rtc_addr, (uint8_t)7);
                                
                                for(int buffer_counter = 0; buffer_counter < 7; buffer_counter++)
                                    rtc_registers[buffer_counter] = Wire.read();

                                Serial.write(rtc_registers, 7);
                                Serial.write(CRC8(rtc_registers, 7));

                                Serial.print("?");

                                timeout_timer = millis();
                                while(!Serial.available())
                                {
                                    if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
                                        return -1; //leave system management mode
                                    else
                                        idle_m(0);
                                }

                                if(Serial.read() == '!')
                                    return 1;
                                else
                                    return -1;
                            }
                            else
                                if(mgmt_command == ERASE_LOG)
                                {
                                    uint32_t address_counter = 144;
                                    
                                    setLogPosition(0);
                                    
                                    digitalWrite(SS, LOW);
                                    
                                    SPI.transfer(0b00000010);
                                    SPI.transfer((address_counter >> 16) & 0xFF);
                                    SPI.transfer((address_counter >> 8) & 0xFF);
                                    SPI.transfer(address_counter & 0xFF);

                                    for(; address_counter < EEPROM_SIZE; address_counter++)
                                        SPI.transfer(0);

                                    digitalWrite(SS, HIGH);

                                    Serial.print("!");

                                    return 1;
                                }
                                else
                                    if(mgmt_command == NULL_LIST_SEGMENT)
                                    {
                                        Serial.print("?");

                                        timeout_timer = millis();
                                        while(Serial.available() < 4)
                                        {
                                            if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
                                                return -1; //leave system management mode
                                            else
                                                idle_m(0);
                                        }
                                    
                                        uint16_t start_page = (((uint16_t)Serial.read() << 8) | Serial.read());
                                        uint16_t end_page = (((uint16_t)Serial.read() << 8) | Serial.read());

                                        for(uint16_t page_counter = start_page; page_counter <= end_page; page_counter++)
                                        {
                                            uint32_t page_start_address = (uint32_t)page_counter*EEPROM_PAGE_SIZE;
                                            
                                            Wire.beginTransmission((page_start_address > 0xFFFF ? id_eeprom_ctrl|1 : id_eeprom_ctrl));
                                            Wire.write( (page_start_address > 0xFFFF ? page_start_address-0xFFFF : page_start_address)>>8 );
                                            Wire.write( (page_start_address > 0xFFFF ? page_start_address-0xFFFF : page_start_address)&0xFF );
                                    
                                            for(uint8_t byte_counter = 0; byte_counter < EEPROM_PAGE_SIZE; byte_counter++)
                                                Wire.write(0);

                                            Wire.endTransmission();
                                    
                                            idle_m(6); //wait for the eeprom to finish writing its buffer
                                        }

                                        Serial.print("!");

                                        return 1;
                                    }
                                    else
                                    if(mgmt_command == GET_DOOR_CONTROL_OUTPUTS)
                                    {
                                        uint8_t compressed_values = 0;

                                        compressed_values |= digitalRead(EXT_0_PIN);
                                        compressed_values |= digitalRead(EXT_1_PIN) << 1;
                                        compressed_values |= door_0_force_unlock << 2;
                                        compressed_values |= door_1_force_unlock << 3;

                                        Serial.write(compressed_values);
                                        
                                        return 1;
                                    }
                                    else
                                    if(mgmt_command == SET_DOOR_CONTROL_OUTPUTS)
                                    {
                                        Serial.print("?");

                                        timeout_timer = millis();
                                        while(!Serial.available())
                                        {
                                            if((uint32_t)((long)millis()-timeout_timer) >= programming_operation_timeout)
                                            return -1; //leave system management mode
                                            else
                                            idle_m(0);
                                        }

                                        uint8_t compressed_values = Serial.read();

                                        door_0_force_unlock = compressed_values & 1;
                                        door_1_force_unlock = (compressed_values >> 1) & 1;

                                        Serial.print("!");
                                        
                                        return 1;
                                    }
   return -1;
}

//Search The ID EEPROM And Determine If Card Holder Will Be Granted Access
//
//returns 0 for entry denied
//returns 1 for entry granted on secure floor. as of now there is only one control input for the elevator system so permission to enter one
//			restricted floor will also grant access to the others
uint8_t requestEntry(uint8_t * ID, uint8_t ID_length, uint8_t reader_id) //scan the ID list eeprom
{
	//get the current time and day
	uint8_t current_minute = getMinute();
	uint8_t current_hour = getHour();
	uint8_t current_day = getDay();
	
	for(uint32_t address_counter = 0; address_counter < EEPROM_SIZE; address_counter+=EEPROM_PAGE_SIZE)
	{
		//set eeprom address pointer to 0
		Wire.beginTransmission(address_counter > 0xFFFF ? id_eeprom_ctrl|1 : id_eeprom_ctrl);
		Wire.write((address_counter > 0xFFFF ? address_counter-0xFFFF : address_counter) >> 8);
		Wire.write((address_counter > 0xFFFF ? address_counter-0xFFFF : address_counter) & 0xFF);
		Wire.endTransmission(0);
		
		Wire.requestFrom(address_counter > 0xFFFF ? id_eeprom_ctrl|1 : id_eeprom_ctrl, EEPROM_PAGE_SIZE); //request 1 page of memory
		
		for(uint8_t page_counter = 0; page_counter < EEPROM_PAGE_SIZE; page_counter+=32) //loop required because each page has 4-32 Byte entries
		{
			uint8_t match_counter = 8-ID_length;
			
			//if the ID is less than 8 bytes then skip the excess
			Wire.rxBufferIndex += match_counter;
			
			for(uint8_t byte_counter = 8-ID_length; byte_counter < 32; byte_counter++)
			{
				if(match_counter < byte_counter)
				{
					Wire.rxBufferIndex += (32-byte_counter);
					break;
				}
				else
					if(match_counter == 8)
					{						
						//oh yay! a match :)
						//now check the following (in order) then skip remaining bytes
						//start time minute, start time hour, end time minute, end time hour, day of the week, is entry permitted on associated floors
						
						uint8_t start_time_minute =			   Wire.read(),
							    start_time_hour   =			   Wire.read(),
						        end_time_minute   =			   Wire.read(),
							    end_time_hour     =			   Wire.read(),
							    permitted_days    =			   Wire.read();
						uint16_t hold_config   = ((uint16_t)Wire.read() << 8) | Wire.read(); //5 flag bits, 9 bits for output hold duration resolution: 5s
																							   //msb = trigger type (0 = static, 1 = pulsed), next four bits are number of seconds between pulses
						
						//reset the Wire library internal counters because we'll be returning without reading anymore
						Wire.rxBufferIndex  =  0;
						Wire.rxBufferLength = 0;
						
						start_time_minute = (start_time_minute >> 4)      * 10 + (start_time_minute & 0xF);
						start_time_hour =  ((start_time_hour   >> 4) & 3) * 10 + (start_time_hour   & 0xF);
						
						end_time_minute = (end_time_minute >> 4)      * 10 + (end_time_minute & 0xF);
						end_time_hour =  ((end_time_hour   >> 4) & 3) * 10 + (end_time_hour   & 0xF);
						
						if(((((current_hour == start_time_hour) && (current_minute >= start_time_minute)) || ((current_hour == end_time_hour) && current_minute <= end_time_minute))) ||
							 ((current_hour > start_time_hour)  && (current_hour < end_time_hour)))
						{
							if(((0x80 >> current_day) & permitted_days) > 0)
							{	
								uint32_t hold_duration = ((uint32_t)hold_config & 0x1FF) * 5000;
								uint8_t hold_type = 0; //hardcode static hold type
								uint32_t pulse_interval = ((hold_config >> 9) & 0xF) * 1000; //pulse duration is 0.25s
								
								if(ext_0_hold_duration == hold_duration) //if the approved card has the same hold duration as the active hold then clear the hold
									ext_0_hold_duration = 0;
								else
									if((hold_duration > ext_0_hold_duration) || ((uint32_t)((long)millis() - ext_0_hold_duration) < hold_duration))
									{
										ext_0_hold_duration = hold_duration;
										ext_0_hold_type = hold_type;
										ext_0_pulse_interval = pulse_interval;
									}
									else
										return 2;
								
								return 1;
							}
						}
						return 0;			   
					}
					else
						if(Wire.read() == ID[byte_counter-(8-ID_length)])
							match_counter++;
			}
		}
	}
	return 0;
}

void writeLogEntry(uint8_t * ID, uint8_t ID_length, uint8_t reader_id, uint8_t auth_code)
{
	uint16_t log_position = getLogPosition();
		
	uint32_t log_address = log_position*16 + 144;
	uint8_t  rtc_registers[7];
	
	Wire.beginTransmission(rtc_addr);
	Wire.write(0);
	Wire.endTransmission(0);
	
	Wire.requestFrom((uint8_t)rtc_addr, (uint8_t)7);
	
	for(int buffer_counter = 0; buffer_counter < 7; buffer_counter++)
		rtc_registers[buffer_counter] = Wire.read();
	
	digitalWrite(SS, LOW);
	
	SPI.transfer(0b00000010);
	SPI.transfer((log_address >> 16) & 0xFF);
	SPI.transfer((log_address >> 8) & 0xFF);
	SPI.transfer(log_address & 0xFF);
	
	//write id to log. any id that is less than 8 bytes is zero padded
	for(uint8_t id_counter = 0; id_counter < 8; id_counter++)
		if(id_counter < (8-ID_length))
			SPI.transfer(0);
		else
			SPI.transfer(ID[id_counter-(8-ID_length)]);
	
	for(int buffer_counter = 0; buffer_counter < 7; buffer_counter++)
		SPI.transfer(rtc_registers[buffer_counter]);
		
	SPI.transfer((reader_id << 4) | auth_code); // bits 7 - 4 are the reader id. bits 3 - 0 are the authorization code
	
	digitalWrite(SS, HIGH);
	
 	if(++log_position > 7990) //increment and see if we need to roll over the log position
 		log_position = 0;
	
	setLogPosition(log_position);
}

void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	Wire.beginTransmission(rtc_addr);
	
	Wire.write((uint8_t)0);
	
	Wire.write((((seconds /10) <<4) | (seconds %10)));
	Wire.write((((minutes /10) <<4) | (minutes %10)));
	Wire.write((((hours   /10) <<4) |   (hours %10)));
	
	Wire.endTransmission(1);
}

void setTimeFormat(uint8_t time_format_flag)
{
	Wire.beginTransmission(rtc_addr);
	
	Wire.write((uint8_t)2);
	
	Wire.endTransmission(0);
	
	Wire.requestFrom(rtc_addr,(uint8_t)1);
	uint8_t hours_register = Wire.read();
	
	if(time_format_flag)
		hours_register|=0x40;
	else
		hours_register&=0xBF;
		
	Wire.beginTransmission(rtc_addr);
	
	Wire.write((uint8_t)2);
	Wire.write(hours_register);
	
	Wire.endTransmission(1);
}

void setDate(uint8_t year, uint8_t month, uint8_t date)
{
	Wire.beginTransmission(rtc_addr);
	
	Wire.write((uint8_t)4);
	Wire.write((((date  / 10) << 4) | (date  %10)));
	Wire.write((((month / 10) << 4) | (month %10)));
	Wire.write((((year  / 10) << 4) | (year  %10)));
	
	Wire.endTransmission(1);
}

uint8_t getMinute()
{
	Wire.beginTransmission(rtc_addr);
	Wire.write(1);
	Wire.endTransmission(0);
	
	Wire.requestFrom(rtc_addr, (uint8_t)1);
	
	uint8_t minute = Wire.read();
	
	return (minute >> 4) * 10 + (minute & 0xF);
}

uint8_t getHour()
{
	Wire.beginTransmission(rtc_addr);
	Wire.write(2);
	Wire.endTransmission(0);
	
	Wire.requestFrom(rtc_addr, (uint8_t)1);
	
	uint8_t hour = Wire.read() & 0x3F;
	
	return (hour >> 5) * 20 + ((hour >> 4) & 1) * 10 + (hour & 0xF);
}

uint8_t getDay()
{
	Wire.beginTransmission(rtc_addr);
	Wire.write(3);
	Wire.endTransmission(0);
	
	Wire.requestFrom(rtc_addr, (uint8_t)1);
	
	return Wire.read();
}

uint16_t getLogPosition()
{
	uint16_t log_position;
	
	digitalWrite(SS, LOW);
	
	SPI.transfer(0b00000011);
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(0);
	
	log_position = ((uint16_t)SPI.transfer(0x0) << 8) | SPI.transfer(0x0);
	
	digitalWrite(SS, HIGH);
	
	return log_position;
}

void setLogPosition(uint16_t log_pos)
{
	digitalWrite(SS, LOW);
	
	SPI.transfer(0b00000010);
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(0);
	
	SPI.transfer(log_pos >> 8);
	SPI.transfer(log_pos);
	
	digitalWrite(SS, HIGH);
}

//zero out the sram
void eraseSRAM()
{
	digitalWrite(LED_2_PIN, HIGH);
	
	digitalWrite(SS, LOW);
	
	SPI.transfer(0b00000010);
	SPI.transfer(0x0);
	SPI.transfer(0x0);
	SPI.transfer(0x0);
	
	for(uint32_t memory_address = 0; memory_address < 128000; memory_address++)
		SPI.transfer(0x0);
	
	digitalWrite(SS, HIGH);
	digitalWrite(LED_2_PIN, LOW);
}

//check for a power failure (main power and backup battery) and then erase the sram
uint8_t checkOSF()
{
	Wire.beginTransmission(rtc_addr);
	Wire.write(0x0F);
	Wire.endTransmission(0);
	
	Wire.requestFrom(rtc_addr, (uint8_t)1);
	
	uint8_t status_register = Wire.read();
	
	if((status_register >> 7) > 0)
	{
		Wire.beginTransmission(rtc_addr);
		
		Wire.write((uint8_t)0xF);
		Wire.write(status_register & 0b01111111);
		
		Wire.endTransmission(1);
		
		eraseSRAM();
		
		return 1;
	}
	else
		return 0;
}

void resetNetworkAdapter()
{
    pinMode(ETHERNET_RESET, OUTPUT);
    idle_m(10);
    pinMode(ETHERNET_RESET, INPUT);
}

//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
byte CRC8(const byte *data, byte len)
{
    byte crc = 0x00;
    while (len--)
    {
        byte extract = *data++;
        for (byte tempI = 8; tempI; tempI--)
        {
            byte sum = (crc ^ extract) & 0x01;

            crc >>= 1;

            if (sum)
            crc ^= 0x8C;

            extract >>= 1;
        }
    }
    
    return crc;
}
