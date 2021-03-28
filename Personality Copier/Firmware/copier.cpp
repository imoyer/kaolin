// KAOLIN PERSONALITY COPIER
//
// ILAN E. MOYER
// March 26th, 2021
//
// This Attiny84-based Kaolin personality copies the entire memory, fuse, and EEPROM contents of a source attiny44/84 into a target
// attiny44/84. It is heavily based on the Universal Personality Programmer Console.
//
// Note: Kaolin is being developed around the attiny84 exclusively, but because the additional overhead is so low, the code here is being
// 		 written to support the attiny 44 and 84.


//--INCLUDES--
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//-- DATA STRUCTURES --
typedef struct { //stores specific chip programming interfaces. These are simply pin numbers like PA0, all assumed on PORT A
  uint8_t resetPin;
  uint8_t mosiPin;
  uint8_t misoPin;
  uint8_t sckPin;
} chipInterface;

typedef struct { //stores parameters related to the chip on a particular interface
  uint32_t signature;
  uint8_t pageSize; //number of words per page
  uint8_t pageCount; //number of pages in the program memory
  uint16_t eepromSize; //number of bytes in EEPROM
} chipParameters;

typedef struct{ //stores all chip information, including the programming pins and the parameters
	chipInterface interface;
	chipParameters parameters;
} chip;


//--FUNCTION HEADERS--

//  Configuration
void resetChipInterface();
void switchFocus(chip *activeChip);
uint8_t detectChip(chip *activeChip);
void assignSourceAndTarget();

//  High-Level Copying
uint8_t copyEEPROM();
uint8_t copyFuses();
uint8_t copyProgramMemory();

//  Low-Level Programming
uint8_t spiExchange(uint8_t dataOut);
void progTransaction();
uint8_t initProgrammer();
uint32_t readSignature();
uint8_t readEEPROM(uint8_t address);
void writeEEPROM(uint8_t address, uint8_t value);
uint8_t readFuse(uint8_t fuseIndex);
void writeFuse(uint8_t fuseIndex, uint8_t fuseValue);
void eraseChip();
void writePage(uint16_t pageAddressWord);
uint16_t readMemoryWord(uint16_t wordAddress);
void readMemoryPage(uint16_t pageAddress, uint8_t pageSize);
void loadMemoryWord(uint16_t wordAddress, uint16_t wordValue);
void loadMemoryPage(uint16_t pageAddress, uint8_t pageSize);

//  Utility Functions
uint8_t checkTrigger();
void latchTrigger();
void releaseTrigger();
void resetState();
void greenLEDOn();
void greenLEDOff();
void greenLEDToggle();
void redLEDOn();
void redLEDOff();
void redLEDToggle();

// Timer Functions
void enableTimer(uint16_t duration);
void disableTimer();

//-- STATE VARIABLES --
volatile uint8_t state; //0: searching for chip in socket, 1: valid chip detected in socket, 2: actively copying, 3: error

chip socketChip; //stores the parameters of the chip in the socket
chip busChip; //stores the parameters of the chip in on the bus

chip *sourceChip;	//a pointer to the source chip
chip *targetChip; //a pointer to the target chip

uint8_t activePageCount; //the lowest common denominator memory page count between the socket and bus targets
uint8_t activePageSize; //the current page size being used. Both the bus and socket targets should have the same page size, for now.
uint16_t activeEEPROMSize; //the lowest common denominator EEPROM size between the socket and bus targets


// Chip Signatures
#define signature_attiny44		0x07921E
#define signature_attiny84		0x0C931E

// Timer Delay Values
#define delay_checkSocketChip	8000	//this is approximately a 1-sec delay at 8Mhz with a prescalar of 1/1024
#define delay_blinkGreenLED		2000	//approx 0.25 sec @ 8Mhz/1024

//--PROGRAMMER MEMORY--
uint8_t progTxBuffer[4];
uint8_t progRxBuffer[4];
#define progBufferBitSize	6 //the size of prog buffer is 1 << this value. 64-byte pages for the attiny84
const uint8_t progBufferSize = 1 << progBufferBitSize;
uint8_t progBuffer[progBufferSize];	//temporarily stores data to be written to external device.
uint8_t chipSignature[4]; //stores the most recently read signature


//--SPI INTERFACE--
#define SPIHalfPeriod  4  //uS per half-wave, 4uS = clock/128 period

//--PIN AND PORT DEFINITIONS--
#define greenLEDPORT  PORTB
#define greenLEDDDR   DDRB
#define greenLEDPIN	  PINB
#define greenLEDPin   PB0

#define redLEDPORT    PORTB
#define redLEDDDR     DDRB
#define redLEDPIN	  PINB
#define redLEDPin     PB1

#define triggerPORT	  PORTB //this is the bus signal that triggers programming
#define triggerDDR	  DDRB
#define triggerPIN	  PINB
#define triggerPin	  PB2

#define sckPORT 	PORTA
#define sckDDR		DDRA
volatile uint8_t sckPinMask; //Note that the pin on the port is dynamic, depending on whether the focus in on the socket or bus

#define misoPIN 	PINA  //note ref to PIN, not PORT, since this is an input
#define misoDDR		DDRA
volatile uint8_t misoPinMask;

#define mosiPORT 	PORTA
#define mosiDDR 	DDRA
volatile uint8_t mosiPinMask;

#define resetPORT	PORTA
#define resetDDR 	DDRA
volatile uint8_t resetPinMask;



// ---- SETUP ----
void setup(){
  // Configure output LED Pins
  greenLEDDDR |= (1<<greenLEDPin);
  greenLEDPORT |= (1<<greenLEDPin);
  redLEDDDR |= (1<<redLEDPin);
  redLEDPORT |= (1<<redLEDPin);
  triggerDDR &= ~(1<<triggerPin);

  // Assign pins to socket and bus interfaces
  socketChip.interface.resetPin = PA0;
  socketChip.interface.mosiPin = PA1;
  socketChip.interface.misoPin = PA2;
  socketChip.interface.sckPin = PA3;
  busChip.interface.resetPin = PA4;
  busChip.interface.mosiPin = PA5;
  busChip.interface.misoPin = PA6;
  busChip.interface.sckPin = PA7;

  resetState(); //resets the system state

  sei(); //enable global interrupts
};


//---- LOOP ----
void loop(){
  switch(state){

  	case 0: //searching for chip in socket
  	  if(detectChip(&socketChip)){ //chip has been detected in socket
  		state = 1;
    	enableTimer(delay_checkSocketChip); //check again (i.e. reset state to 0) in ~1 sec.
  	  }else{ //No chip in socket
  	  	greenLEDOff();
  	  }
  	  break;

  	case 1: //socket chip is detected
  	  greenLEDOn(); //turn on the green LED
  	  if(checkTrigger()){ //the trigger bus line has been brought LOW
  		  state = 2; //begin programming
  	  }
  	  break;

  	case 2: //beginning the programming task.
  	  latchTrigger(); //latches the trigger line
  	  _delay_ms(10); //give time for the latch to propagate and for the bus MCUs to react.
  	  assignSourceAndTarget(); // assigns the source and target chips based on the reverse switch. By default, the bus is the source and the
  	  	  	  	  	  	  	   // socket is the target.
  	  enableTimer(delay_blinkGreenLED); //start blinking the LED

  	  //Load parameters of target chip
  	  if(!detectChip(targetChip)){ //unable to load target chip
  		  state = 3; //error state
  		  break;
  	  }

  	  //Load parameters of source chip
  	  if(!detectChip(sourceChip)){ //unable to load
  		  state = 3; //error state
  		  break;
  	  }

  	  //Determine programming parameters based on the lowest common denominator
  	  // 	PAGE COUNT
  	  if((*targetChip).parameters.pageCount<(*sourceChip).parameters.pageCount){
  		  activePageCount = (*targetChip).parameters.pageCount;
  	  }else{
  		  activePageCount = (*sourceChip).parameters.pageCount;
  	  }
  	  // 	EEPROM SIZE
  	  if((*targetChip).parameters.eepromSize<(*sourceChip).parameters.eepromSize){
  		  activeEEPROMSize = (*targetChip).parameters.eepromSize;
  	  }else{
  		  activeEEPROMSize = (*sourceChip).parameters.eepromSize;
  	  }
  	  // 	PAGE SIZE
  	  activePageSize = (*sourceChip).parameters.pageSize; //just use bus target page size for now, as these should be the same


  	  // Copy Program Memory
  	  if(!copyProgramMemory()){
  		  state = 3; //error state, blink red for a while.
  		  break;
  	  }

  	  // Copy Fuses
  	  if(!copyFuses()){
  		  state = 3; //error state, blink red for a while.
  		  break;
  	  }


  	  // Copy EEPROM
  	  if(!copyEEPROM()){
  		  state = 3; //error state, blink red for a while.
  		  break;
  	  }

  	  resetState(); //Done! Return to searching for chip in socket

  	  break;

  	case 3: //error state
  	  _delay_ms(5000); //blink red LED for 5 sec
  	  resetState();
  	  break;
  };

};

//---- MAIN ----
int main(){
	setup();
	while(1){
		loop();
	}
};


//---- COPYING FUNCTIONS ----
uint8_t copyEEPROM(){
	// copies EEPROM from the bus target to the socket target.
    // returns 1 if EEPROM validates after copy, or 0 if not.

	// calculate how many times we need to fill the program buffer.
	uint8_t cycles = (uint8_t)(activeEEPROMSize >> progBufferBitSize); //number of times need to fill the program buffer
	if(cycles == 0){ //fractional fills of the program buffer still need to cycle once.
		cycles = 1;
	}

	uint16_t memoryOffset = 0; //tracks the memory address offset based on the cycle count

	for(uint8_t cycleCount = 0; cycleCount<cycles; cycleCount++){ //Cycle thru read/write cycles in increments of the programming buffer size
		//read in EEPROM data
		switchFocus(sourceChip); //switch focus to the source chip
		initProgrammer(); //initialize for programming
		for(uint8_t bufferIndex = 0; bufferIndex<progBufferSize; bufferIndex++){
			//Load EEPROM data into the programming buffer
			progBuffer[bufferIndex] = readEEPROM((uint8_t)(memoryOffset + bufferIndex));
		}

		//write out EEPROM data
		switchFocus(targetChip); //switch focus to the target chip
		initProgrammer();
		for(uint8_t bufferIndex = 0; bufferIndex<progBufferSize; bufferIndex++){
			//Load EEPROM data into the programming buffer
			writeEEPROM((uint8_t)(memoryOffset + bufferIndex), progBuffer[bufferIndex]);
		}

		//validate EEPROM data
		for(uint8_t bufferIndex = 0; bufferIndex<progBufferSize; bufferIndex++){
			//Load EEPROM data into the programming buffer
			if(readEEPROM((uint8_t)(memoryOffset + bufferIndex)) == progBuffer[bufferIndex]){
				continue;
			}else{
				return 0; //didn't validate
			}
		}

		//increment memory offset
		memoryOffset += progBufferSize;
	}
	return 1; //if we got this far, EEPROM copy should have been successful.
}

uint8_t copyFuses(){
	//read in fuses
	switchFocus(sourceChip);
	initProgrammer();
	uint8_t lfuse = readFuse(0);
	uint8_t hfuse = readFuse(1);
	uint8_t efuse = readFuse(2);

	//write fuses
	switchFocus(targetChip);
	initProgrammer();
	writeFuse(0, lfuse);
	writeFuse(1, hfuse);
	writeFuse(2, efuse);

	//verify fuses
	if(readFuse(0) == lfuse && readFuse(1) == hfuse && readFuse(2) == efuse){
		return 1;
	}else{
		return 0;
	}
}

uint8_t copyProgramMemory(){
	// copies program memory from the bus to the socket chip
	// Returns 1 if the copy verifies, or 0 if not.

	//erase target chip
	switchFocus(targetChip);
	eraseChip();

	//prepare to begin transfer
	switchFocus(sourceChip);
	initProgrammer();

	//NOTE: I'm going to try not re-initializing the programmer after each target switch.

	uint16_t currentPageAddress = 0;
	for(uint8_t pageIndex = 0; pageIndex<activePageCount; pageIndex++){ //iterate thru all pages

		//copy
		switchFocus(sourceChip);
		initProgrammer();
		readMemoryPage(currentPageAddress, activePageSize); //read memory page into progBuffer
		switchFocus(targetChip);
		initProgrammer();
		loadMemoryPage(currentPageAddress, activePageSize); //load memory page
		writePage(currentPageAddress);

		//verify
		for(uint8_t wordOffset = 0; wordOffset<activePageSize; wordOffset++){
			uint16_t wordValue = readMemoryWord(currentPageAddress + wordOffset); //read word value at offset
			if((uint8_t)(wordValue & 0x00FF) != progBuffer[wordOffset<<1]){
				return 0; //verify failed
			}
			if((uint8_t)((wordValue & 0xFF00)>>8) != progBuffer[(wordOffset<<1) +1]){
				return 0; //verify failed
			}
		}

		currentPageAddress += activePageSize; //increment current page address
	}
	return 1;


}

//---- PROGRAMMING FUNCTIONS ----

uint8_t spiExchange(uint8_t dataOut){
  //transmits in mode 0 (sample on rising, setup on falling), MSB first
  uint8_t dataIn = 0;  //stores received data
  uint8_t dataMask = 0b10000000;  //initialized to MSB
  for (uint8_t i=0; i<8; i++){
    sckPORT &= ~sckPinMask;  //drive SCK low. On first bit this will already be low. This is the setup edge.
    if(dataOut & dataMask){
       mosiPORT |= mosiPinMask;  //setup a 1
    }else{
       mosiPORT &= ~mosiPinMask; //setup a 0
    }
    _delay_us(SPIHalfPeriod);  //wait
    sckPORT |= sckPinMask;  //drive SCK high. This is the sample edge.
    if(misoPIN & misoPinMask){  //read MISO pin
      dataIn |= dataMask;  //update dataIn
    }
   _delay_us(SPIHalfPeriod);
    dataMask = dataMask >> 1;
  }

  sckPORT &= ~sckPinMask; //finish by dropping sck low
  _delay_us(20);  //
  return dataIn;
}

void progTransaction(){
  //programmer transaction
  for(uint8_t i = 0; i<4; i++){
    progRxBuffer[i] = spiExchange(progTxBuffer[i]);
  }
}

uint8_t initProgrammer(){

  resetChipInterface(); //reset the chip interface to a known state

  sckPORT &= ~sckPinMask;

  resetPORT &= ~resetPinMask;  //strobe reset line
  _delay_us(10);
  resetPORT |= resetPinMask;
  _delay_us(10);
  resetPORT &= ~resetPinMask;

  _delay_ms(20);

  progTxBuffer[0] = 0xAC;	//Programming Enable
  progTxBuffer[1] = 0x53;
  progTxBuffer[2] = 0x00;
  progTxBuffer[3] = 0x00;
  progTransaction();

  uint8_t response = progRxBuffer[2];  //read response, this should be 0x53

  progTxBuffer[0] = 0x4D;	//Load Extended Byte as Zero
  progTxBuffer[1] = 0x00;
  progTxBuffer[2] = 0x00;
  progTxBuffer[3] = 0x00;
  progTransaction();

  return response;

}

uint32_t readSignature(){
	//reads the signature of the device, loads into lastKnownSignature, and also returns the concatenated value.
  initProgrammer();

  progTxBuffer[0] = 0x30;  //command to read signature bytes
  progTxBuffer[1] = 0x00;
  progTxBuffer[3] = 0x00;

  for(uint8_t j = 0; j<4; j++){
	progTxBuffer[2] = j;  //signature byte address
	progTransaction();
	chipSignature[j] = progRxBuffer[3]; //stores most recent signature read from chip
  }

  return (uint32_t)chipSignature[0]+
		  (((uint32_t)chipSignature[1])<<8) +
		  (((uint32_t)chipSignature[2])<<16); //do not return last byte

}

uint8_t readEEPROM(uint8_t address){
  // Reads a byte from the EEPROM
  // address -- an 8-bit address in EEPROM. Note that this function is limited to reading from the first 256 bytes of EEPROM.
  // returns the EEPROM value at the requested address
  // NOTE: THIS FUNCTION ASSUMES THAT THE PROGRAMMER HAS ALREADY BEEN INITIALIZED.

  progTxBuffer[0] = 0xA0;  //command to read EEPROM
  progTxBuffer[1] = 0x00;
  progTxBuffer[2] = address;  //EEPROM address
  progTxBuffer[3] = 0x00;  //dummy byte
  progTransaction();
  return progRxBuffer[3];  //read data in
}

void writeEEPROM(uint8_t address, uint8_t value){
  // Writes a byte to the EEPROM.
  // address -- an 8-bit address in EEPROM. Note that this function is limited to writing to the first 256 bytes of EEPROM.
  // value -- the byte value to write to the EEPROM address.
  // NOTE: THIS FUNCTION ASSUMES THAT THE PROGRAMMER HAS ALREADY BEEN INITIALIZED.

  progTxBuffer[0] = 0xC0;  //command to write EEPROM
  progTxBuffer[1] = 0x00;
  progTxBuffer[2] = address;  //EEPROM address
  progTxBuffer[3] = value;  //EEPROM data
  progTransaction();
  _delay_ms(4); //minimum specified delay for writing EEPROM
}

uint8_t readFuse(uint8_t fuseIndex){
  // Reads a fuse at fuseIndex.
  // fuseIndex -- 0: lfuse, 1: hfuse, 2: efuse
  // NOTE: This function DOES NOT initialize the programmer in advance. This is left to the user.
  progTxBuffer[2] = 0x00;
  progTxBuffer[3] = 0x00;

  switch(fuseIndex){
	case 0:
	  progTxBuffer[0] = 0x50; //lfuse
	  progTxBuffer[1] = 0x00;
	  break;
	case 1:
	  progTxBuffer[0] = 0x58; //hfuse
	  progTxBuffer[1] = 0x08;
	  break;
	case 2:
	  progTxBuffer[0] = 0x50; //efuse
	  progTxBuffer[1] = 0x08;
	  break;
  }

  progTransaction();
  return progRxBuffer[3];
}

void writeFuse(uint8_t fuseIndex, uint8_t fuseValue){
  // writes a fuse at fuseIndex.
  // fuseIndex -- 0: lfuse, 1: hfuse, 2: efuse
  progTxBuffer[0] = 0xAC;
  progTxBuffer[2] = 0x00;

  switch(fuseIndex){
	case 0:
	  progTxBuffer[1] = 0xA0; //lfuse
	  break;
	case 1:
	  progTxBuffer[1] = 0xA8; //hfuse
	  break;
	case 2:
	  progTxBuffer[1] = 0xA4; //efuse
	  break;
	}

  progTxBuffer[3] = fuseValue;
  progTransaction();
  _delay_ms(5); //5ms delay per datasheet twd_fuse
}

void eraseChip(){
  //erases the target chip.
  initProgrammer();

  progTxBuffer[0] = 0xAC;  //Chip Erase
  progTxBuffer[1] = 0x80;
  progTxBuffer[2] = 0x00;
  progTxBuffer[3] = 0x00;
  progTransaction();

  _delay_ms(10);  //10ms delay per datasheet twd_erase
}

void writePage(uint16_t pageAddressWord){
  //writes a loaded page to the corresponding page address
  progTxBuffer[0] = 0x4C;
  progTxBuffer[1] = (uint8_t)((pageAddressWord&0xFF00)>>8);
  progTxBuffer[2] = (uint8_t)(pageAddressWord&0x00FF);
  progTxBuffer[3] = 0x00;
  progTransaction();

  _delay_ms(5);   //5ms delay per datasheet, twd_flash
}

uint16_t readMemoryWord(uint16_t wordAddress){
  // Reads a word from memory.
  // wordAddress -- the address of the word in program memory
  // NOTE: This assumes that the programmer has been initialized

  //read low byte
  progTxBuffer[0] = 0x20;
  progTxBuffer[1] = (uint8_t)((wordAddress & 0xFF00)>>8);
  progTxBuffer[2] = (uint8_t)(wordAddress & 0x00FF);
  progTxBuffer[3] = 0x00;
  progTransaction();

  uint16_t dataWord = progRxBuffer[3]; //store the low byte

  //read high byte
  progTxBuffer[0] = 0x28;
  progTxBuffer[1] = (uint8_t)((wordAddress & 0xFF00)>>8);
  progTxBuffer[2] = (uint8_t)(wordAddress & 0x00FF);
  progTxBuffer[3] = 0x00;
  progTransaction();

  dataWord += ((uint16_t)progRxBuffer[3])<<8; //store the high byte
  return dataWord;

}

void readMemoryPage(uint16_t pageAddress, uint8_t pageSize){
  // reads a page of memory into progBuffer
  // pageAddress -- the base WORD address of the page
  // pageSize -- the size of the page, IN WORDS

  for(uint8_t i=0; i<pageSize; i++){
	uint16_t dataWord = readMemoryWord(pageAddress + i);

	progBuffer[i<<1] = (uint8_t)(dataWord & 0x00FF);  //store low byte to progBuffer
	progBuffer[(i<<1)+1] = (uint8_t)((dataWord & 0xFF00)>>8); //store high byte to progBuffer
  }

}

void loadMemoryWord(uint16_t wordAddress, uint16_t wordValue){
  // Reads a word from memory.
  // wordAddress -- the address of the word in program memory
  // NOTE: This assumes that the programmer has been initialized
  progTxBuffer[0] = 0x40; //write low byte
  progTxBuffer[1] = 0x00;
  progTxBuffer[2] = (uint8_t)(wordAddress&0x00FF);  //base byte address offset by i, converted to word
  progTxBuffer[3] = (uint8_t)(wordValue & 0x00FF);
  progTransaction();

  progTxBuffer[0] = 0x48;  //write high byte
  progTxBuffer[1] = 0x00;
  progTxBuffer[2] = (uint8_t)(wordAddress&0x00FF);  //base byte address offset by i, converted to word
  progTxBuffer[3] = (uint8_t)((wordValue & 0xFF00)>>8);
  progTransaction();
}

void loadMemoryPage(uint16_t pageAddress, uint8_t pageSize){
  // Loads a page of memory into progBuffer
  // pageAddress -- the base WORD address of the page
  // pageSize -- the size of the page, IN WORDS
  uint16_t currentWordValue;
  for(uint8_t i=0; i<pageSize; i++){
	currentWordValue = (uint16_t)progBuffer[i<<1] + (((uint16_t)progBuffer[(i<<1)+1])<<8);
	loadMemoryWord(pageAddress + i, currentWordValue);
  }
}

//---- CONFIGURATION FUNCTIONS ----
void resetChipInterface(){
  //sets the chip interface to a known starting state

  sckDDR |= sckPinMask;  //set sck pin to output
  sckPORT &= ~sckPinMask;  //set sck pin low

  misoDDR &= ~misoPinMask; //set miso pin to input
  mosiDDR |= mosiPinMask; //set mosi pin to output

  resetDDR |= resetPinMask;  //set reset pin to output
  resetPORT |= resetPinMask; //set reset to high
}

void switchFocus(chip *activeChip){
  sckPinMask = 1 << (*activeChip).interface.sckPin;
  misoPinMask = 1 << (*activeChip).interface.misoPin;
  mosiPinMask = 1 << (*activeChip).interface.mosiPin;
  resetPinMask = 1 << (*activeChip).interface.resetPin;

  resetChipInterface(); //update the pin hardware config.
}

uint8_t detectChip(chip *activeChip){
	// This function will write to activeChip with the relevant info on the installed chip.
	// Returns 1 if the chip was identified, otherwise 0.
	switchFocus(activeChip); //switches the programming interface focus to the active chip

	uint32_t activeChipSignature = readSignature();
	switch(activeChipSignature){
	  case signature_attiny84: //attiny84
		(*activeChip).parameters.signature = activeChipSignature;
		(*activeChip).parameters.pageSize = 32; //words
		(*activeChip).parameters.pageCount = 128; //pages 128
		(*activeChip).parameters.eepromSize = 256; //EEPROM bytes. NOTE: This chip has 512 bytes, but we only access 256 with the current programming method.
		return 1;
	  case signature_attiny44: //attiny44
		(*activeChip).parameters.signature = activeChipSignature;
		(*activeChip).parameters.pageSize = 32; //words
		(*activeChip).parameters.pageCount = 64; //pages
		(*activeChip).parameters.eepromSize = 256; //EEPROM bytes
		return 1;

	  default: //no chip identified
	    return 0;
	}
}

void assignSourceAndTarget(){
	// assigns the source and target pointers to either the socket or bus.
	// this allows programming to work in reverse, to "suck" a program off the chip in the socket and into the bus node's chip.

	// For now, until a reverse button is added, the socket is always the target, and the bus is always the source.
	sourceChip = &busChip;
	targetChip = &socketChip;
}

//---- UTILITY FUNCTIONS ----

uint8_t checkTrigger(){
	//checks whether the trigger line has been pulled low
	return ~triggerPIN & (1<<triggerPin);
}

void latchTrigger(){
	//latches the trigger line low. This indicates to all bus nodes that programming is about to start
	triggerDDR |= (1<<triggerPin);
	triggerPORT &= ~(1<<triggerPin);
}

void releaseTrigger(){
	//puts the trigger pin into high-impedance, effectively releasing the trigger line.
	triggerDDR &= ~(1<<triggerPin);
}

void resetState(){
	//resets the state of the copier
	state = 0; //searching for a chip in the socket
	redLEDOff();
	greenLEDOff();
	releaseTrigger();
}

void greenLEDOn(){
	// Turns on the green LED
	greenLEDPORT |= (1<<greenLEDPin);
}

void greenLEDOff(){
	// Turns off the green LED
	greenLEDPORT &= ~(1<<greenLEDPin);
}

void greenLEDToggle(){
	// Toggles the green LED
	greenLEDPIN = (1<<greenLEDPin);
}

void redLEDOn(){
	// Turns on the red LED
	redLEDPORT |= (1<<redLEDPin);
}

void redLEDOff(){
	// Turns off the red LED
	redLEDPORT &= ~(1<<redLEDPin);
}

void redLEDToggle(){
	//Toggles the red LED
	redLEDPIN = (1<<redLEDPin);
}


//---- TIMER FUNCTIONS ----
void enableTimer(uint16_t duration){
	//enables the watchdog timer. If triggered, it will restore the node to the ready (0) state.
	TCCR1A = 0;	//CTC on OCR1A
	TCCR1B = (1<<WGM12)|(1<<CS12)|(1<<CS10); //CTC on OCR1A, CLK/1024
	TCCR1C = 0;

	TCNT1 = 0; //reset the timer/counter
	OCR1A = duration; //set TOP to provided duration, in units of 0.128ms (1024 / 8MHz)
	TIMSK1 = (1<<OCIE1A); //enable interrupts
}

void disableTimer(){
	//disables the watchdog timer
	TCNT1 = 0; //reset the timer/counter
	TIMSK1 = 0; //disable timer interrupts
	TCCR1B = (1<<WGM12); //CTC on OCR1A, disable clock
}

ISR(TIM1_COMPA_vect)	//transmitter interrupt routine
  {
	  disableTimer();
	  switch(state){
		case 0: //no chip is detected
		  break;
		case 1: //chip is detected
		  state = 0; //bump back to no chip detected state
		  break;
		case 2: //programming -- here the timer is used to blink the programming led, as control is entirely with this mcu
		  greenLEDToggle();
		  enableTimer(delay_blinkGreenLED);
		  break;
		case 3: //error state - toggle the red LED
		  redLEDToggle();
		  enableTimer(delay_blinkGreenLED);
		  break;
	  };
  }
