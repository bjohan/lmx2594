
#include <EEPROM.h>

bool macro=false;
uint8_t macroPosition=0;



#define SYSREF 5
#define CSB 6
#define MUX 7
#define SDI_DATA_OUT 8
#define SCK 9
#define RAMPCLK 10
#define RAMPDIR 11
#define CE 12
#define SYNC 13

#define REGISTER_R0 0b0010010000010000
#define REGISTER_R1 0b0000100000001000
#define REGISTER_R2 0b0000010100000000
#define REGISTER_R3 0b0000011001000010
#define REGISTER_R4 0b0000000001000011
#define REGISTER_R5 0b0000000011001000
#define REGISTER_R6 0b1100100000000010
#define REGISTER_R7 0b0000000010110010
#define REGISTER_R8 0b0010000000000000
#define REGISTER_R9 0b0000011000000100
#define REGISTER_R10 0b0001000001011000
#define REGISTER_R11 0b0000000000001000
#define REGISTER_R12 0b0101000000000000
#define REGISTER_R13 0b0100000000000000
#define REGISTER_R14 0b0001111000000000
#define REGISTER_R15 0b0000011001001111
#define REGISTER_R16 0b0000000000000000
#define REGISTER_R17 0b0000000000000000
#define REGISTER_R18 0b0000000001100100
#define REGISTER_R19 0b0010011100000000
#define REGISTER_R20 0b1100000001001000
#define REGISTER_R21 0b0000010000000001
#define REGISTER_R22 0b0000000000000001
#define REGISTER_R23 0b0000000001111100
#define REGISTER_R24 0b0000011100011010
#define REGISTER_R25 0b0000110000101011
#define REGISTER_R26 0b0000110110110000
#define REGISTER_R27 0b0000000000000010
#define REGISTER_R28 0b0000010010001000
#define REGISTER_R29 0b0011000110001100
#define REGISTER_R30 0b0011000110001100
#define REGISTER_R31 0b0000001111101100
#define REGISTER_R32 0b0000001110010011
#define REGISTER_R33 0b0001111000100001
#define REGISTER_R34 0b0000000000000000
#define REGISTER_R35 0b0000000000000100
#define REGISTER_R36 0b0000000000000000
#define REGISTER_R37 0b0000000000000100
#define REGISTER_R38 0b0000000000000000
#define REGISTER_R39 0b0000000000000000
#define REGISTER_R40 0b0000000000000000
#define REGISTER_R41 0b0000000000000000
#define REGISTER_R42 0b0000000000000000
#define REGISTER_R43 0b0000000000000000
#define REGISTER_R44 0b0000000000000000
#define REGISTER_R45 0b1100000011000000
#define REGISTER_R46 0b0000011111111100
#define REGISTER_R47 0b0000001100000000
#define REGISTER_R48 0b0000001100000000
#define REGISTER_R49 0b0100000110000000
#define REGISTER_R50 0b0000000000000000
#define REGISTER_R51 0b0000000010000000
#define REGISTER_R52 0b0000100000100000
#define REGISTER_R53 0b0000000000000000
#define REGISTER_R54 0b0000000000000000
#define REGISTER_R55 0b0000000000000000
#define REGISTER_R56 0b0000000000000000
#define REGISTER_R57 0b0000000000100000
#define REGISTER_R58 0b0000000000000001
#define REGISTER_R59 0b0000000000000000
#define REGISTER_R60 0b0000000000000000
#define REGISTER_R61 0b0000000010101000
#define REGISTER_R62 0b0000001100100010
#define REGISTER_R63 0b0000000000000000
#define REGISTER_R64 0b0001001110001000
#define REGISTER_R65 0b0000000000000000
#define REGISTER_R66 0b0000000111110100
#define REGISTER_R67 0b0000000000000000
#define REGISTER_R68 0b0000001111101000
#define REGISTER_R69 0b0000000000000000
#define REGISTER_R70 0b0000000000000000
#define REGISTER_R71 0b0000000000000001
#define REGISTER_R72 0b0000000000000000
#define REGISTER_R73 0b0000000000000000
#define REGISTER_R74 0b0000000000000000
#define REGISTER_R75 0b0000100000000000
#define REGISTER_R76 0b0000000000001100
#define REGISTER_R77 0b0000000000000000
#define REGISTER_R78 0b0000000000000001
#define REGISTER_R79 0b0000000000000000
#define REGISTER_R80 0b0000000000000000
#define REGISTER_R81 0b0000000000000000
#define REGISTER_R82 0b0000000000000000
#define REGISTER_R83 0b0000000000000000
#define REGISTER_R84 0b0000000000000000
#define REGISTER_R85 0b0000000000000000
#define REGISTER_R86 0b0000000000000000
#define REGISTER_R87 0b0000000000000000
#define REGISTER_R88 0b0000000000000000
#define REGISTER_R89 0b0000000000000000
#define REGISTER_R90 0b0000000000000000
#define REGISTER_R91 0b0000000000000000
#define REGISTER_R92 0b0000000000000000
#define REGISTER_R93 0b0000000000000000
#define REGISTER_R94 0b0000000000000000
#define REGISTER_R95 0b0000000000000000
#define REGISTER_R96 0b0000000000000000
#define REGISTER_R97 0b0000100000000000
#define REGISTER_R98 0b0000000000000000
#define REGISTER_R99 0b0000000000000000
#define REGISTER_R100 0b0000000000000000
#define REGISTER_R101 0b0000000000000000
#define REGISTER_R102 0b0000000000000000
#define REGISTER_R103 0b0000000000000000
#define REGISTER_R104 0b0000000000000000
#define REGISTER_R105 0b0000000000000000
#define REGISTER_R106 0b0000000000000000
#define REGISTER_R107 0b0000000000000000
#define REGISTER_R108 0b0000000000000000
#define REGISTER_R109 0b0000000000000000
#define REGISTER_R110 0b0000000000000000
#define REGISTER_R111 0b0000000000000000
#define REGISTER_R112 0b0000000000000000



const PROGMEM uint16_t default_regmap[] {
	REGISTER_R0, REGISTER_R1, REGISTER_R2, REGISTER_R3, REGISTER_R4, REGISTER_R5, REGISTER_R6, REGISTER_R7, 
	REGISTER_R8, REGISTER_R9, REGISTER_R10, REGISTER_R11, REGISTER_R12, REGISTER_R13, REGISTER_R14, REGISTER_R15,
	REGISTER_R16, REGISTER_R17, REGISTER_R18 ,REGISTER_R19, REGISTER_R20, REGISTER_R21, REGISTER_R22, REGISTER_R23,
	REGISTER_R24, REGISTER_R25, REGISTER_R26, REGISTER_R27, REGISTER_R28, REGISTER_R29, REGISTER_R30, REGISTER_R31,
	REGISTER_R32, REGISTER_R33, REGISTER_R34, REGISTER_R35, REGISTER_R36, REGISTER_R37, REGISTER_R38, REGISTER_R39, 
	REGISTER_R40, REGISTER_R41, REGISTER_R42, REGISTER_R43, REGISTER_R44, REGISTER_R45, REGISTER_R46, REGISTER_R47, 
	REGISTER_R48, REGISTER_R49, REGISTER_R50, REGISTER_R51, REGISTER_R52, REGISTER_R53, REGISTER_R54, REGISTER_R55, 
	REGISTER_R56, REGISTER_R57, REGISTER_R58, REGISTER_R59, REGISTER_R60, REGISTER_R61, REGISTER_R62, REGISTER_R63, 
	REGISTER_R64, REGISTER_R65, REGISTER_R66, REGISTER_R67, REGISTER_R68, REGISTER_R69, REGISTER_R70, REGISTER_R71, 
	REGISTER_R72, REGISTER_R73, REGISTER_R74, REGISTER_R75, REGISTER_R76, REGISTER_R77, REGISTER_R78, REGISTER_R79, 
	REGISTER_R80, REGISTER_R81, REGISTER_R82, REGISTER_R83, REGISTER_R84, REGISTER_R85, REGISTER_R86, REGISTER_R87, 
	REGISTER_R88, REGISTER_R89, REGISTER_R90, REGISTER_R91, REGISTER_R92, REGISTER_R93, REGISTER_R94, REGISTER_R95, 
	REGISTER_R96, REGISTER_R97, REGISTER_R98, REGISTER_R99, REGISTER_R100, REGISTER_R101, REGISTER_R102, REGISTER_R103, 
	REGISTER_R104, REGISTER_R105, REGISTER_R106, REGISTER_R107, REGISTER_R108, REGISTER_R109, REGISTER_R110, REGISTER_R111, 
	REGISTER_R112
};	


void writeBits(uint32_t word, uint8_t nbits)
{
	//Serial.print(nbits);
	//Serial.print(" ");
	//Serial.println(word, HEX);
	//Serial.print(" ");
	for(uint32_t b = nbits ; b > 0 ; b--){
		digitalWrite(SCK, LOW);
		if(word & 0x80000000){
			digitalWrite(SDI_DATA_OUT, HIGH);
			//Serial.print("1");
		}
		else{
			digitalWrite(SDI_DATA_OUT, LOW);
			//Serial.print("0");
		}
		digitalWrite(SCK, HIGH);
		word <<= 1;
	}
}

uint16_t readBits(uint8_t nbits)
{
	uint16_t data;
	for(uint32_t b = nbits ; b > 0 ; b--){
		digitalWrite(SCK, LOW);
		data <<= 1;
		data |= digitalRead(MUX);
		digitalWrite(SCK, HIGH);
	}
	return data;
}



void sendWord(uint32_t word)
{
	digitalWrite(CSB, LOW);
	writeBits(word, 24);
	digitalWrite(SCK, LOW);
	digitalWrite(CSB, HIGH);
	digitalWrite(SDI_DATA_OUT, HIGH);
}

uint16_t receiveWord(uint32_t word){
	uint16_t d;
	digitalWrite(CSB, LOW);
	writeBits(word, 8);
	d = readBits(16);
	digitalWrite(SCK, LOW);
	digitalWrite(CSB, HIGH);
	digitalWrite(SDI_DATA_OUT, HIGH);
	return d;
}

uint16_t readRegister(uint32_t addr){
	addr &= 0x7F;
	addr |= 0x80;
	return receiveWord(addr<<24);
}

void writeRegister(uint32_t addr, uint32_t data){
	addr &= 0x7F;
	data &= 0xFFFF;
	sendWord(addr<<24|data<<8);
}

void init_registers(){
	for(int i = 112 ; i >= 0 ; i--) writeRegister(i, default_regmap[i]);
}





void replayMacro(){
	uint8_t nm=EEPROM.read(0);
	Serial.print("Replaying macro consisting of ");
	Serial.print(nm);
	Serial.println(" writes");
	if(nm == 0xFF){
		Serial.println("Nonexistant macro, exiting");
		return;
	}
	for(int i = 0 ; i < nm ; i++){
		uint32_t sw;
		EEPROM.get(1+i*4, sw);
		Serial.print("Macro ");
		Serial.print(i);
		Serial.print(" value ");
		Serial.println(sw);
		sendWord(sw);
	}
}




void setup() {                
	Serial.begin(115200);
	// initialize the digital pin as an output.
	pinMode(SYSREF, OUTPUT);
	pinMode(CSB, OUTPUT);
	pinMode(MUX, INPUT);
	pinMode(SDI_DATA_OUT, OUTPUT);
	pinMode(SCK, OUTPUT);
	pinMode(RAMPCLK, OUTPUT);
	pinMode(RAMPDIR, OUTPUT);
	pinMode(CE, OUTPUT);
	pinMode(SYNC, OUTPUT);
	pinMode(A0, OUTPUT);
	pinMode(A1, OUTPUT);
	pinMode(A2, OUTPUT);
	pinMode(A3, OUTPUT);
	digitalWrite(CSB, HIGH);
	Serial.println("CE LOW");
	digitalWrite(CE, LOW);
	delay(50);
	Serial.println("CE HIGH");
	digitalWrite(CE, HIGH);
	delay(50);
	Serial.println("SPI RST");
	//writeRegister(0, 2);
	//delay(100);
	//writeRegister(0, 0);
	Serial.println("SPI RST DIS");
	//init_registers();
	replayMacro();
	Serial.println("READY");
	
}

uint8_t copyTok(char *dst, char *src, int8_t n){
	for(;*src!='\0'; src++){
		if(*src == ' ') n--;
		else if(n == 0) *dst++=*src;
		if(n < 0 || (n==0 && *(src+1) =='\0')){
			*dst='\0'; 
			return 0;
		}
	}
	return -1;
}


void commandWriteRegister(char *cmdstr){
	uint32_t sw;
	uint32_t addr;
	uint32_t value;	
	char tokbuf[32];
	if(copyTok(tokbuf, cmdstr, 1)){
		Serial.println("First argument must be address");
	}
	addr=atoi(tokbuf);
	if(copyTok(tokbuf, cmdstr, 2)){
		Serial.println("Second argument must be value");
	}
	value=atol(tokbuf);
	/*Serial.println(cmdstr);
	Serial.print("Writing ");
	Serial.print(value);
	Serial.print(" to ");
	Serial.println(addr);*/

	addr &= 0x7F;
	value &= 0xFFFF;
	sw=addr<<24|value<<8;
	//uint32_t sw=(addr&0xF)|((value<<4)&0xFFFFFFF0);
	sendWord(sw);
	
	if(macro){
		EEPROM.put(4*macroPosition+1, sw);
		macroPosition++;
	}
}

void commandReadRegister(char *cmdstr){
	uint32_t addr;
	char tokbuf[32];
	if(copyTok(tokbuf, cmdstr, 1)){
		Serial.println("First argument must be address");
	}
	addr=atoi(tokbuf);
	Serial.print("Read: "); Serial.print(readRegister(addr)); Serial.print(" From register address: "); Serial.println(addr);
}

void commandEnable(char *cmdstr){
	uint8_t enable;
	char tokbuf[32];
	if(copyTok(tokbuf, cmdstr, 1)){
		Serial.println("Firt argument must be 0 to disable or nonzero to enable");
	}
	enable=atoi(tokbuf);
	if(enable){
		pinMode(CE, INPUT);
		digitalWrite(CE, HIGH);
		Serial.println("Chip enabled");
	}else{
		pinMode(CE, OUTPUT);
		digitalWrite(CE, LOW);
		Serial.println("Chip disabled");
	}	
}

void commandMacroRecord(char *cmdstr){
	uint8_t enable;
	char tokbuf[32];
	if(copyTok(tokbuf, cmdstr, 1)){
		Serial.println("First argument must be 0 to finish recording or nonzero to start");
	}
	enable=atoi(tokbuf);
	if(enable){
		if(not macro){
			macro=true;
			macroPosition=0;
			Serial.println("Macro enabled");
		} else {
			Serial.println("Macro was already enabled");
		}
	}else{
		if(macro){
			macro=false;
			EEPROM.write(0, macroPosition);
			macroPosition=0;
			Serial.println("Macro disabled");
		} else {
			Serial.println("macro was already disabled");
		}
	}	

}

void commandReboot(char *cmdstr){
	Serial.println("toggling CE");
	pinMode(CE, OUTPUT);
	digitalWrite(CE, LOW);
	delay(100);
	digitalWrite(CE, HIGH);
	pinMode(CE, INPUT);
	Serial.println("SPI RST");
	writeRegister(0, 2);
	delay(100);
	writeRegister(0, 0);
	Serial.println("SPI RST DIS");
	init_registers();
	replayMacro();

}

void commandOutput(char *cmdstr){
	Serial.print("PLL Output ");
	Serial.println(digitalRead(MUX));
}
void commandGpio(char *cmdstr){
	uint8_t p;
	char tokbuf[32];
	if(copyTok(tokbuf, cmdstr, 1)){
		Serial.println("First argument must be decimal number representing the bit pattern to be output");
	}
	p=atoi(tokbuf);
	digitalWrite(A0, p&0x01);
	digitalWrite(A1, p&0x02);
	digitalWrite(A2, p&0x04);
	digitalWrite(A3, p&0x08);
}


void executeCommand(char* cmdstr){
	char tokbuf[32];
	if(copyTok(tokbuf, cmdstr, 0) == 0){
		switch(tokbuf[0]){
			case 'W':
			case 'w':
				commandWriteRegister(cmdstr);
				break;
			case 'R':
			case 'r':
				commandReadRegister(cmdstr);
				break;

			case 'E':
			case 'e':
				commandEnable(cmdstr);
				break;

			case 'M':
			case 'm':
				commandMacroRecord(cmdstr);
				break;

			case 'B':
			case 'b':
				commandReboot(cmdstr);
				break;

			case 'O':
			case 'o':
				commandOutput(cmdstr);
				break;
			case 'G':
			case 'g':
				commandGpio(cmdstr);
				break;
			default:
				Serial.print("Unrecognized command ");
				Serial.println(cmdstr);
			


		}
	}
	
}




void loop() {
	char d;
	static char cmd[32];
	static uint8_t cp=0;
	if(Serial.available()){
		d=Serial.read();
		if(d=='\r' ||  d== '\n'){
			cmd[cp]='\0';
			executeCommand(cmd);
			Serial.println("DONE");
			cp=0;
		} else {
			if(cp < 15){
				cmd[cp++]=d;
			} else {
				cmd[0]='\0';
				Serial.println("Command too long. Max 15 bytes. Press enter and enter new command");
			} 
		}
	}
}
