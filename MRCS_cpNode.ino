/*==================================================================================

  cpNode - Control Point CMRI Node
  Version 1.6

------------------------------------------------------------------------------------
  cpNode concept and design committed on 5/31/2013 by Chuck Catania and Seth Neumann
  Model Railroad Control Systems, LLP
  
  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License. 
  To view a copy of the license, visit http://creativecommons.org/licenses/by-sa/3.0/deed.en_US

------------------------------------------------------------------------------------

  Author: Chuck Catania - Model Railroad Control Systems 
  System Board:  Modern Device BBLeo (Arduino Style Leonardo)
  
  Revision History:
   v1.6   05/25/2021  Plocher:
                      Significant code cleanup, refactoring and simplification
                      Add debugging flags and associated print statements...
                      Removed APortMap array and convoluted indexing in favor of unrolled digitalRead/Writes
                      Uses less FLASH and RAM
                      Added PROMINI_8OUT8IN for MRCS cpNode Control Point Pro Mini
                      Renamed BASE_NODE* to BBLEO*
                      Use runtime constants to take advantage of compiler optimizer to remove unused and unreachable code
                      Add support for active low or active high as a default
                      Add support for inverting inputs and outputs

                      Code sizes:                      
                      1.6 BBLeo
                      Sketch uses 9088 bytes (31%) of program storage space. Maximum is 28672 bytes.
                      Global variables use 1115 bytes (43%) of dynamic memory, leaving 1445 bytes for local variables. Maximum is 2560 bytes.
                      
                      1.6 ProMini (no debug serial port)
                      Sketch uses 6132 bytes (19%) of program storage space. Maximum is 30720 bytes.
                      Global variables use 975 bytes (47%) of dynamic memory, leaving 1073 bytes for local variables. Maximum is 2048 bytes.
                      
   v1.5   09/12/2016  Changed digital pin name mnemonics to hard coded pin numbers to keep the pre-processor happy.
                      Code size:
                      1.5 BBLeo Only
                      Sketch uses 9418 bytes (32%) of program storage space. Maximum is 28672 bytes.
                      Global variables use 1169 bytes (45%) of dynamic memory, leaving 1391 bytes for local variables. Maximum is 2560 bytes.
   v1.4.4 03/28/2016  Removed #define BASE_NODE_SERVO as there was no support for the servo library
   v1.4.2 05/27/2015  (TVerberg) Corrected typo errors in Base_Node_12out_4in output unpacking, 
                      (TVerberg) Corrected Base_Node_Servo input packing routines.
                      Rearranged CMRInet protocol responses to put non-message functions at the front of the list.
                      Added Init message DL/DH delay processing for Classic node compatability. 
                      Added Init_CA_Ports_OFF boolean.  If set to true, all ports driving Common Anode LEDs will be forced OFF at bootup.   
                      Changed CMRInet_BufSize to int and increased length to 260 to handle the max SUSIC + 4 pad buffer
                      Changed Flush_CMRInet_To_ETX() to exit on either seeing ETX or empty serial buffer
                      Changed DEBOUNCE_DELAY from 10 ms to 2 ms
                      Added BASE_NODE_RSMC_LOCK configuration to support locking an RSMC controlled turnout with
                       Dennis Drury's switch lock board.
                      Added BASE_NODE_8OUT8IN.  Sets D4-D11 as outputs, D12-A5 as inputs.
   v1.4.1 04/15/2015  Changed CMRINET_SPEED definition from int to long for network speeds greater than 28800 bps        
   v1.4   06/25/2014  Added the 12 output, 4 input standard configuration per Dick Johannes of the NMRA HUB Division
                      Moved debug option and SN variables out of Node Configuration Parameters area.
   v1.3   04/06/2014  Fixed issue with BASE_NODE_8IN8OUT, BASE_NODE_16OUT high bit B8 output not moved to A5.
                      Bit extraction loop ended one bit early.  Other routines worked because loop limit was less than
                      maximum port map index.
   v1.2   03/06/2014  Fixed issue with BASE_NODE_8IN8OUT where port setup did not match specification.
                      This was an implementation deviation from the design specification.
   v1.1   03/01/2014  Fixed issue with BASE_NODE_8IN8OUT where the byte assignment was flipped.
   v1.0   01/04/2014  Released 
    0.0d  08/24/2013  Initial template definition
    
  This sketch is the code template for a CMRI serial protocol node implemented in an Arduino style system board.  
  The Modern Devices BBLeo, Bare Bones Leonardo (ATMega32u4) is the target Arduino style system board. 
  
  Implements the CMRI Serial Protocol designed by Dr. Bruce Chubb and published publicly in various books, magazines, and articles.
  The physical link is RS485, 4-wire, half duplex, serial.  
  Each node has a one byte address in the range of 0-127.  
  The Host polls the CMRI nodes for data.   
  Port assignments represent the defined capability of a cpNode.
  The base node data configuration is two bytes in, two bytes out.
  All needed signal pins are connected via pin headers to the node board.  
  
  Reserved I/O pins are:
   D0 - RX  CMRI RS485 Receive
   D1 - TX  CMRI RS485 Transmit
   
   BBLEO:
   D2 - SDA I2C Data
   D3 - SCL I2C Clock
   
   ProMini:
   A4 - SDA I2C Data
   A5 - SCL I2C Clock

   All: onboard I/O pins map to IB[0,1] and OB[0,1]
   
                   SERIAL PORT ASSIGNMENTS
           
  Serial        Bootloader/Debug monitor (USB Built in on LEO, unusable on ProMini)
  CMRI_SERIAL       CMRI Port (RS-485) RX(D0) and TX(D1) pins
  
 **************************************************************
 **********        CMRI Protocol Message Format      **********
 **************************************************************
    - Initialization Message (I)  HOST to NODE
      SYN SYN STX <UA> <I><NDP><dH><dL><NS><CT(1)><CT(NS)> ETX

    - Poll for Data (P)  HOST to NODE
      SYN SYN STX <UA> <P> ETX

    - Read Data (R)  NODE To HOST  (Response to Poll)
      SYN SYN STX <UA> <R><IB(1)><IB(NS)> ETX

    - Transmit Data (T)  HOST to NODE
      SYN SYN STX <UA> <T><OB(1)><OB(NS)> ETX
*/

/*************************************************************
 *********            Library Header Files           *********
 *************************************************************/
#include <SoftwareSerial.h>
#include <Wire.h>  // for the I/O expander

const byte max_IOX = 16;  // device = MCP28017 chip

enum {
      BASE_NONE,          // no direct connected I/O pins, UNTESTED
      BBLEO,              // 10 out,  6 in
      BBLEO_8IN8OUT,      //  8 in,   8 out
      BBLEO_8OUT8IN,      //  8 out,  8 in  DEFAULT
      BBLEO_12OUT4IN,     // 12 out,  4 in
      BBLEO_16IN,         //  0 out, 16 in
      BBLEO_16OUT,        // 16 out,  0 in
      BBLEO_RSMC,         // 13 out,  3 in
      BBLEO_RSMC_LOCK,    // 12 out,  4 in
      PROMINI_8OUT8IN,    //  8 out,  8 in  DEEK-ROBOT Pro Mini

      // TODO: when adding new boards or configs, look for TODO comments like this
      //
      // First:  Add a new config name here, 
      // Second: Create new setup_XXX, packXXX and unpackXXX functions (see lines ~500 to ~900)
      // Third:  Following the existing patterns there, add these function calls to the switch 
      //         statements in the setup_Node, Pack_Node_Inputs and unpack_Node_Outputs routines.
      };

//==============================================//
//====    NODE CONFIGURATION PARAMETERS     ====//
//==============================================//
                                                //
int nodeID = 20;                                //
                                                //
const long CMRINET_SPEED =   19200;             //         
           //      
                                                //  
#define USE_IOX     // Use I/O Expander boards  //
                                                //  
// I2C port direction bytes                     //
const int IOX_ioMap[max_IOX]   = {              //
                                                // 
// 0 = OUTPUT  1 = INPUT  -1 = Not Assigned     //
                                                // 
// Addr  0x20     0x21     0x22     0x23        //
// Port  A  B     A  B     A  B     A  B        //
         1, 1,    1, 1,    0, 0,    0, 0,       //
                                                //  
// Addr  0x24     0x25     0x26     0x27        //
//       A  B     A  B     A  B     A  B        //
        -1,-1,   -1,-1,   -1,-1,   -1,-1 };     //
                                                //  
//--------------------------------------------- //
// Base port configuration                      //
// choose one from the enum list above          //
//--------------------------------------------- //
                                                // 
#define NODE PROMINI_8OUT8IN  

                                                //
// set all Common Anode ports to 0?             //
boolean Init_CA_Ports_OFF = false;   

#define ACTIVE_LOW_INPUTS
#define ACTIVE_LOW_OUTPUTS

                                                // 
//==============================================//
//====    NODE CONFIGURATION PARAMETERS     ====//
//==============================================//

// Program debugging - OK to ignore...
const boolean DEBUG_ANNOUNCE = false,    // print config info at end of setup...
              DEBUG_PROTOCOL = false,
              DEBUG_POLL     = false,
              DEBUG_INIT     = false,
              DEBUG_IOX      = false,
              debugging  = (DEBUG_ANNOUNCE || DEBUG_PROTOCOL || DEBUG_POLL || DEBUG_INIT || DEBUG_IOX);

#if defined(ARDUINO_AVR_LEONARDO)
#define CMRI_SERIAL     Serial1
#define MONITOR_SERIAL  Serial
#elif defined(ARDUINO_AVR_PRO)
#define CMRI_SERIAL     Serial
#undef  MONITOR_SERIAL
#else
#error "Unsupported processor"
#endif

//************************************************
//*****        STANDARD DATA PORT MAP        *****         
//************************************************
const int 
    // RS-485 Port
    //------------
           RX      =  0,    // D0  CMRI RS485 Receive
           TX      =  1;    // D1  CMRI RS485 Transmit
    
// Protocol characters 
//--------------------
const char STX    = 0x02,
           ETX    = 0x03,
           DLE    = 0x10,
           SYN    = 0xFF;

// Read responses
//---------------
const char respNone    = 0,  //  No character received
           respErr     = 1,  //  Error occured
           respIgnore  = 2,  //  Not for this node, flush to ETX
           respInit    = 3,  //  "I" Message
           respPoll    = 4,  //  "P" Message
           respRead    = 5,  //  "R" Message
           respTransmit= 6;  //  "T" Message

// Debug monitor speed  
//--------------------
const long int MON_SPEED      = 115200;
const int      DEBOUNCE_DELAY = 2;

//---------------
// Node variables
//---------------
const char cpNODE_NDP = 'C';    // Node Definition Parameter for a cpNode - Control Point Node

byte  UA,           // Node address, stored as UA + UA_Offset
      matchID;      // Incomming address in message to match to node UA
int   c;            // Handy character variable

const int UA_Offset = 65,       // 0x41 Per CMRI protocol spec
          CMRInet_BufSize  = 260;  //  Max SUSIC + 4 pad

// Data buffers
//-------------
const byte cpNode_bufSize= 20,   //  cpNode (2) + IOXx8 (16) + 4 pad
           IO_bufsize    = 20,   //  cpNode (2) + IOXx8 (16) + 4 pad
          
           nIB         = (NODE == BASE_NONE) ? 0 : 2,    //  Total configured onboard input bytes
           nOB         = (NODE == BASE_NONE) ? 0 : 2;    //  Total configured onboard output bytes 
          
byte       CMRInet_Buf[CMRInet_BufSize],   // CMRI message buffer

           OB[IO_bufsize],  // Output bits  HOST to NODE
           IB[IO_bufsize];  // Input bits   NODE to HOST
                      
unsigned long DL  = 0;      // Transmit character delay value in 10 us increments
int           DLH = 0,
              DLL = 0;
              
//**********************************************************************
//**********                      IOX VARIABLES               **********
//**********************************************************************

#ifdef USE_IOX

// I2C command registers
//----------------------
const byte ACTIVELOW_REG_BASE  = 0x02,  // Port A, Port B = 0x02
           PULLUP_REG_BASE     = 0x0C,  // Port A, Port B = 0x0D
           GPIO_REG_BASE       = 0x12,  // Port A, Port B = 0x13
           
           SET_PORT_OUTPUT     = 0x00,
           SET_PORT_INPUT      = 0xFF,
           SET_PORT_PULLUPS    = 0xFF,
           SET_PORT_ACTIVELOW  = 0xFF;
           
                               // Board I2C Addresses (duplicated for ease of indexing)   
const int IOX_devAddr[max_IOX] = {0x20,0x20, 
                                  0x21,0x21, 
                                  0x22,0x22, 
                                  0x23,0x23, 
                                  0x24,0x24, 
                                  0x25,0x25, 
                                  0x26,0x26, 
                                  0x27,0x27  };

// Data byte mapping table created from devAddr/ioMap configuration
// Created by Configure_IOX_Ports

// [port index][0] is I2C chip address
// [port index][1] is I2C port 0=A, 1=B
//-----------------------------------------------------------------
const byte I2C_ADDR = 0,
           I2C_PORT = 1;
          
int IOX_Output_Map[max_IOX][2],
    IOX_Input_Map[max_IOX][2];
    
// After init, holds actual count of defined I2C input and output ports
//---------------------------------------------------------------------
byte numIOX_OUT= 0,
     numIOX_IN = 0;
    
// I2C input and output buffers
//-----------------------------
int IOX_inBuf[max_IOX+2],
    IOX_outBuf[max_IOX+2];

#endif // USE_IOX


//************************************************************************
//***********                IOX SUPPORT ROUTINES                 ********
//*************************************************************************
//  For each defined IOX board, setup the two ports for direction.
//  Input ports will have the weak pull up enabled.
//  
//  Packing and Unpacking routines will place data bytes sequentially in
//  the IOX buffers in the order of device address.    
//  Empty routines will be optimized away when USE_IOX is not defined
//*************************************************************************

//------------------------------------------------------------------------
// Collect the input/output byte assignments for the number of IOX's
// Build the tables used for processing the data bytes
//------------------------------------------------------------------------
void Configure_IOX_Ports() {
#ifdef USE_IOX

    // Count the number of defined ports by direction.  
    // Setup the input and output IOX byte map for the CMRI buffers
    //-------------------------------------------------------------
  
    for (byte i = 0; i < max_IOX; i += 2) {
        for (byte j = 0; j < 2; j++)  {
            byte k = i+j;
            
            if (IOX_ioMap[k] != -1) {
#if defined(MONITOR_SERIAL) && DEBUG_IOX
                  MONITOR_SERIAL.print("IOX["); MONITOR_SERIAL.print(k); MONITOR_SERIAL.print("]  ");
                  MONITOR_SERIAL.print(" I2C Address: 0x");     MONITOR_SERIAL.print(IOX_devAddr[k], HEX); 
                  MONITOR_SERIAL.print(" Port ");               MONITOR_SERIAL.print( (j == 0) ? "A " : "B ");
                  MONITOR_SERIAL.println( (IOX_ioMap[k] == 0) ? "OUTPUT" :
                                (IOX_ioMap[k] == 1) ? "INPUT"  :
                                "INVALID"); 
#endif
            }
            switch (IOX_ioMap[k]) {
             case  0:   // Outputs
                                  IOX_Output_Map[numIOX_OUT][I2C_ADDR] = IOX_devAddr[k];
                                  IOX_Output_Map[numIOX_OUT][I2C_PORT] = j;
                                  numIOX_OUT++;
                                  break;
             case  1:   // Inputs
                                  IOX_Input_Map[numIOX_IN][I2C_ADDR] = IOX_devAddr[k];
                                  IOX_Input_Map[numIOX_IN][I2C_PORT] = j;
                                  numIOX_IN++;
                                  break;
             default: ; // No Port in slot
            } 
        }      
    } 
#endif
}


//-------------------------------------------------------------------------------
// For each of the defined OUTPUT bytes, set the register parameters for the chip
//-------------------------------------------------------------------------------
void Setup_IOX_Outputs()  {
#ifdef USE_IOX
    for (byte i=0; i<numIOX_OUT; i++)  {      
        Wire.beginTransmission(IOX_Output_Map[i][I2C_ADDR]);  // Board Address
        Wire.write(IOX_Output_Map[i][I2C_PORT]);              // A or B Port
        Wire.write(SET_PORT_OUTPUT);                          // Command to set to Output
        Wire.endTransmission();   
    }
#endif
} 


//-------------------------------------------------------------------------------
// For each of the defined INPUT bytes, set the register parameters for the chip
//-------------------------------------------------------------------------------
void Setup_IOX_Inputs() {  
#ifdef USE_IOX
  for (byte i=0; i<numIOX_IN; i++) {     
    byte addr =  IOX_Input_Map[i][I2C_ADDR];
    byte port =  IOX_Input_Map[i][I2C_PORT]; 
    
    // Set port to Inputs
    //-------------------
    Wire.beginTransmission(addr); 
    Wire.write(port);              
    Wire.write(SET_PORT_INPUT);                   
    Wire.endTransmission();    
    
    // Set port to use weak pullups
    //-----------------------------
    Wire.beginTransmission(addr);         
    Wire.write(PULLUP_REG_BASE + port);  
    Wire.write(SET_PORT_PULLUPS);                              
    Wire.endTransmission();    
    
    // Set port to active LOW polarity
    //--------------------------------
    Wire.beginTransmission(addr);            
    Wire.write(ACTIVELOW_REG_BASE + port); 
    Wire.write(SET_PORT_ACTIVELOW);                               
    Wire.endTransmission();   
  }
#endif
} 
  
//--------------------------------------------------------------------------
// Unpack the bytes sent in the transmit message and output to the I2C ports
//--------------------------------------------------------------------------
void Unpack_IOX_Outputs() {
#ifdef USE_IOX
      
    // Set up values to assign CMRI output bytes to I2C bytes
    //-------------------------------------------------------
    if (numIOX_OUT > 0) {         
        // Transfer CMRI output bytes to the I2C buffer
        //---------------------------------------------    
        for (byte n = 0; n < numIOX_OUT; n++)  {
            IOX_outBuf[n] = CMRInet_Buf[nOB + n]; 
#ifndef ACTIVE_LOW_OUTPUTS
            IOX_outBuf[n] = ~(IOX_outBuf[n]);   // invert the bits if not active-low
#endif
        } 
        
        // Write the bytes to the I2C chips
        //---------------------------------    
        for (byte i=0; i<numIOX_OUT; i++) { 
#if defined(MONITOR_SERIAL) && DEBUG_IOX
                MONITOR_SERIAL.print("I2C Write OutNum=");
                MONITOR_SERIAL.print(i, DEC);
                MONITOR_SERIAL.print(" I2C Addr=0x");
                MONITOR_SERIAL.print(IOX_Output_Map[i][I2C_ADDR], HEX);
                MONITOR_SERIAL.print(" bank=");
                MONITOR_SERIAL.print((IOX_Output_Map[i][I2C_PORT] == 0) ? "A " : "B ");
                MONITOR_SERIAL.print(" data=0x");
                MONITOR_SERIAL.print(IOX_outBuf[i], HEX);
                MONITOR_SERIAL.println();
#endif
          
            Wire.beginTransmission(IOX_Output_Map[i][I2C_ADDR]);        // Board Address
            Wire.write(GPIO_REG_BASE + IOX_Output_Map[i][I2C_PORT]);    // A or B Port
            Wire.write(IOX_outBuf[i]);                                  // Data to send
            Wire.endTransmission();  
        } 
    }
#endif
} 

//-----------------------------------------------------------------------------------
// Read the input bytes from the I2C ports and collect them into the I2C input buffer
// The bits will be latched and cleared when sent in the poll response.
//-----------------------------------------------------------------------------------
void Pack_IOX_Inputs() {
#ifdef USE_IOX
  // Transfer the input bytes to the I2C buffer
  //-------------------------------------------    

  for (byte i=0; i<numIOX_IN; i++) {  
        Wire.beginTransmission(IOX_Input_Map[i][I2C_ADDR]);        // Board Address
        Wire.write(GPIO_REG_BASE + IOX_Input_Map[i][I2C_PORT]);    // A or B Port
        Wire.endTransmission();  
  
        Wire.requestFrom(IOX_Input_Map[i][I2C_ADDR],1);            // Data to read
        IOX_inBuf[i] = IOX_inBuf[i] | Wire.read();                 // Latch the inputs
#ifndef ACTIVE_LOW_INPUTS
        IOX_inBuf[i] = ~(IOX_inBuf[i]);
#endif
        delay(DEBOUNCE_DELAY);
  } 
  
#endif
}


//*************************************************************************
//**********            GENERAL SUPPORT ROUTINES                 **********
//*************************************************************************

//-----------------------
//  Get the available ram 
//-----------------------
int freeRam() {
    extern int __heap_start, *__brkval; 
    int v; 
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



//------------------------------------------------
//  Set the Node ID 
//------------------------------------------------
byte SetNodeAddr(byte nodeAddr) {
    
    // Set node address to 127 if an invalid decimal address is passed
    //----------------------------------------------------------------
    if (nodeAddr > 127) {
        nodeAddr = 127; 
    }
     
    UA = nodeAddr + UA_Offset;
    return nodeAddr;  // in case it changed...
}


//****************************************************************************************
//**********                     DATA BIT PACK/UNPACK ROUTINES                  **********
//**********  create [setup_, pack_ and unpack] for each configuration          **********
//****************************************************************************************

// BBLEO has Arduino pins 
//     4-11  routed out to BANK 1, and
//    12-A5  routed out to BANK 2
//
void setup_BBLEO(void) {
    pinMode( 4, OUTPUT);        // D4  SG1A-R   
    pinMode( 5, OUTPUT);        // D5  SG1A-Y   
    pinMode( 6, OUTPUT);        // D6  SG1A-G   
    pinMode( 7, OUTPUT);        // D7  SG1B-R  
    pinMode( 8, OUTPUT);        // D8  SG1B-Y   
    pinMode( 9, OUTPUT);        // D9  SG2-R   
    pinMode(10, OUTPUT);        // D10 SG2-Y   
    pinMode(11, OUTPUT);        // D11 SG2-G
    
    pinMode(12, OUTPUT);        // D12 SG3-R  
    pinMode(13, OUTPUT);        // D13 SG3-Y   
    pinMode(A0, INPUT_PULLUP);  // A0  TC1  Track Circuit 1   
    pinMode(A1, INPUT_PULLUP);  // A1  TC2  Track Circuit 2   
    pinMode(A2, INPUT_PULLUP);  // A2  OS1  OS Circuit 1   
    pinMode(A3, INPUT_PULLUP);  // A3  SW1  Turnout throw input   
    pinMode(A4, INPUT_PULLUP);  // A4  AUX1   
    pinMode(A5, INPUT_PULLUP);  // A5  AUX2  
}
void pack_BBLEO(void) {
    IB[0] = IB[1] = 0;
    //        Pin 12 is an output
    //        Pin 13 is an output
    IB[0] |= (!digitalRead(A0) << 0);
    IB[0] |= (!digitalRead(A1) << 1);
    IB[0] |= (!digitalRead(A2) << 2);
    IB[0] |= (!digitalRead(A3) << 3);
    IB[0] |= (!digitalRead(A4) << 4);
    IB[0] |= (!digitalRead(A5) << 5);
    
    //    Pin  4  is an output
    //    Pin  5  is an output
    //    Pin  6  is an output
    //    Pin  7  is an output
    //    Pin  8  is an output
    //    Pin  9  is an output
    //    Pin 10  is an output
    //    Pin 11  is an output
}
void unpack_BBLEO(void) {
    // OB0 8 bits     OB1 2 bits      -> NODE
    // OOOOOOOO       xxxxxxOO 
    //----------------------------
    // lower8
    digitalWrite(4,  (( OB[0] >> 0) &  0x01) );
    digitalWrite(5,  (( OB[0] >> 1) &  0x01) );
    digitalWrite(6,  (( OB[0] >> 2) &  0x01) );
    digitalWrite(7,  (( OB[0] >> 3) &  0x01) );
    digitalWrite(8,  (( OB[0] >> 4) &  0x01) );
    digitalWrite(9,  (( OB[0] >> 5) &  0x01) );
    digitalWrite(10, (( OB[0] >> 6) &  0x01) );
    digitalWrite(11, (( OB[0] >> 7) &  0x01) );
    
    digitalWrite(12, (( OB[1] >> 0) &  0x01) );
    digitalWrite(13, (( OB[1] >> 1) &  0x01) );
}
//****************************************************************************************

void setup_BBLEO_8IN8OUT(void) {
    pinMode( 4, INPUT_PULLUP);  // D4   
    pinMode( 5, INPUT_PULLUP);  // D5   
    pinMode( 6, INPUT_PULLUP);  // D6   
    pinMode( 7, INPUT_PULLUP);  // D7  
    pinMode( 8, INPUT_PULLUP);  // D8   
    pinMode( 9, INPUT_PULLUP);  // D9   
    pinMode(10, INPUT_PULLUP);  // D10   
    pinMode(11, INPUT_PULLUP);  // D11
  
    pinMode(12, OUTPUT);        // D12  
    pinMode(13, OUTPUT);        // D13   
    pinMode(A0, OUTPUT);        // A0 
    pinMode(A1, OUTPUT);        // A1   
    pinMode(A2, OUTPUT);        // A2   
    pinMode(A3, OUTPUT);        // A3   
    pinMode(A4, OUTPUT);        // A4   
    pinMode(A5, OUTPUT);        // A5
}
void pack_BBLEO_8IN8OUT(void) {
     // IB0 8 bits     IB1 All Zero
     // IIIIIIII       00000000
     //----------------------------
    IB[0] = IB[1] = 0;
    IB[0] |= (!digitalRead(4)  << 0);
    IB[0] |= (!digitalRead(5)  << 1);
    IB[0] |= (!digitalRead(6)  << 2);
    IB[0] |= (!digitalRead(7)  << 3);
    IB[0] |= (!digitalRead(8)  << 4);
    IB[0] |= (!digitalRead(9)  << 5);
    IB[0] |= (!digitalRead(10) << 6);
    IB[0] |= (!digitalRead(11) << 7);
}
void unpack_BBLEO_8IN8OUT(void) {
    // OB0 8 bits     OB1 All Zero    -> NODE
    // OOOOOOOO       xxxxxxxx 
    //----------------------------
    digitalWrite(12, (( OB[0] >> 0) &  0x01) );
    digitalWrite(13, (( OB[0] >> 1) &  0x01) );
    digitalWrite(A0, (( OB[0] >> 2) &  0x01) );
    digitalWrite(A1, (( OB[0] >> 3) &  0x01) );
    digitalWrite(A2, (( OB[0] >> 4) &  0x01) );
    digitalWrite(A3, (( OB[0] >> 5) &  0x01) );
    digitalWrite(A4, (( OB[0] >> 6) &  0x01) );
    digitalWrite(A5, (( OB[0] >> 7) &  0x01) );
}
//****************************************************************************************

void setup_BBLEO_8OUT8IN(void) {
    // bank 1 is OUTPUT, bank 2 is INPUT
    pinMode( 4, OUTPUT);        // D4   
    pinMode( 5, OUTPUT);        // D5   
    pinMode( 6, OUTPUT);        // D6   
    pinMode( 7, OUTPUT);        // D7  
    pinMode( 8, OUTPUT);        // D8   
    pinMode( 9, OUTPUT);        // D9   
    pinMode(10, OUTPUT);        // D10   
    pinMode(11, OUTPUT);        // D11
  
    pinMode(12, INPUT_PULLUP);  // D12  
    pinMode(13, INPUT_PULLUP);  // D13   
    pinMode(A0, INPUT_PULLUP);  // A0 
    pinMode(A1, INPUT_PULLUP);  // A1   
    pinMode(A2, INPUT_PULLUP);  // A2   
    pinMode(A3, INPUT_PULLUP);  // A3   
    pinMode(A4, INPUT_PULLUP);  // A4   
    pinMode(A5, INPUT_PULLUP);  // A5  
}
void pack_BBLEO_8OUT8IN(void) {
    // D12 - A5 are inputs
    // IB0 8 bits     IB1 All Zero
    // IIIIIIII       00000000
    //----------------------------
    IB[0] = IB[1] = 0;
    IB[0] |= (!digitalRead(12) << 0);
    IB[0] |= (!digitalRead(13) << 1);
    IB[0] |= (!digitalRead(A0) << 2);
    IB[0] |= (!digitalRead(A1) << 3);
    IB[0] |= (!digitalRead(A2) << 4);
    IB[0] |= (!digitalRead(A3) << 5);
    IB[0] |= (!digitalRead(A4) << 6);
    IB[0] |= (!digitalRead(A5) << 7);
}
void unpack_BBLEO_8OUT8IN(void) {
    // D4 - D11 are ouptuts
    // OB0 8 bits     OB1 All Zero
    // OOOOOOOO       xxxxxxxx 
    //----------------------------
    digitalWrite(4,  (( OB[0] >> 0) &  0x01) );
    digitalWrite(5,  (( OB[0] >> 1) &  0x01) );
    digitalWrite(6,  (( OB[0] >> 2) &  0x01) );
    digitalWrite(7,  (( OB[0] >> 3) &  0x01) );
    digitalWrite(8,  (( OB[0] >> 4) &  0x01) );
    digitalWrite(9,  (( OB[0] >> 5) &  0x01) );
    digitalWrite(10, (( OB[0] >> 6) &  0x01) );
    digitalWrite(11, (( OB[0] >> 7) &  0x01) );
}
//****************************************************************************************

void setup_BBLEO_12OUT4IN(void) {
    pinMode( 4, OUTPUT);        // D4   
    pinMode( 5, OUTPUT);        // D5    
    pinMode( 6, OUTPUT);        // D6     
    pinMode( 7, OUTPUT);        // D7   
    pinMode( 8, OUTPUT);        // D8     
    pinMode( 9, OUTPUT);        // D9     
    pinMode(10, OUTPUT);        // D10    
    pinMode(11, OUTPUT);        // D11 
    
    pinMode(12, OUTPUT);        // D12   
    pinMode(13, OUTPUT);        // D13    
    pinMode(A0, OUTPUT);        // A0 
    pinMode(A1, OUTPUT);        // A1
    pinMode(A2, INPUT_PULLUP);  // A2   
    pinMode(A3, INPUT_PULLUP);  // A3   
    pinMode(A4, INPUT_PULLUP);  // A4   
    pinMode(A5, INPUT_PULLUP);  // A5  
}
void pack_BBLEO_12OUT4IN(void) {
     // IB0 4 bits     IB1 All Zero
     // 0000IIII       00000000
     //----------------------------
    IB[0] = IB[1] = 0;
    IB[0] |= (!digitalRead(A2) << 0);
    IB[0] |= (!digitalRead(A3) << 1);
    IB[0] |= (!digitalRead(A4) << 2);
    IB[0] |= (!digitalRead(A5) << 3);
}
void unpack_BBLEO_12OUT4IN(void) {
    // TVerberg
    // OB0 8 bits     OB1 4 bits      -> NODE
    // OOOOOOOO       xxxxOOOO 
    //----------------------------
    // lower8
    digitalWrite(4,  (( OB[0] >> 0) &  0x01) );
    digitalWrite(5,  (( OB[0] >> 1) &  0x01) );
    digitalWrite(6,  (( OB[0] >> 2) &  0x01) );
    digitalWrite(7,  (( OB[0] >> 3) &  0x01) );
    digitalWrite(8,  (( OB[0] >> 4) &  0x01) );
    digitalWrite(9,  (( OB[0] >> 5) &  0x01) );
    digitalWrite(10, (( OB[0] >> 6) &  0x01) );
    digitalWrite(11, (( OB[0] >> 7) &  0x01) );

    digitalWrite(12, (( OB[1] >> 0) &  0x01) );
    digitalWrite(13, (( OB[1] >> 1) &  0x01) );
    digitalWrite(A0, (( OB[1] >> 2) &  0x01) );
    digitalWrite(A1, (( OB[1] >> 3) &  0x01) );
}
//****************************************************************************************

void setup_BBLEO_16IN(void) {
    pinMode( 4, INPUT_PULLUP);  // D4   
    pinMode( 5, INPUT_PULLUP);  // D5   
    pinMode( 6, INPUT_PULLUP);  // D6   
    pinMode( 7, INPUT_PULLUP);  // D7  
    pinMode( 8, INPUT_PULLUP);  // D8   
    pinMode( 9, INPUT_PULLUP);  // D9   
    pinMode(10, INPUT_PULLUP);  // D10   
    pinMode(11, INPUT_PULLUP);  // D11
  
    pinMode(12, INPUT_PULLUP);  // D12  
    pinMode(13, INPUT_PULLUP);  // D13   
    pinMode(A0, INPUT_PULLUP);  // A0 
    pinMode(A1, INPUT_PULLUP);  // A1   
    pinMode(A2, INPUT_PULLUP);  // A2   
    pinMode(A3, INPUT_PULLUP);  // A3   
    pinMode(A4, INPUT_PULLUP);  // A4   
    pinMode(A5, INPUT_PULLUP);  // A5  
}
void pack_BBLEO_16IN(void) {
    // IB0 8 bits     IB1 8 bits
    // IIIIIIII       IIIIIIII
    //----------------------------
    IB[0] = IB[1] = 0;

    IB[0] |= (!digitalRead(4)  << 0);
    IB[0] |= (!digitalRead(5)  << 1);
    IB[0] |= (!digitalRead(6)  << 2);
    IB[0] |= (!digitalRead(7)  << 3);
    IB[0] |= (!digitalRead(8)  << 4);
    IB[0] |= (!digitalRead(9)  << 5);
    IB[0] |= (!digitalRead(10) << 6);
    IB[0] |= (!digitalRead(11) << 7);
    
    IB[1] |= (!digitalRead(12) << 0);
    IB[1] |= (!digitalRead(13) << 1);
    IB[1] |= (!digitalRead(A0) << 2);
    IB[1] |= (!digitalRead(A1) << 3);
    IB[1] |= (!digitalRead(A2) << 4);
    IB[1] |= (!digitalRead(A3) << 5);
    IB[1] |= (!digitalRead(A4) << 6);
    IB[1] |= (!digitalRead(A5) << 7);
}
void unpack_BBLEO_16IN(void) {
    // OB1 All Zero   OB2 All Zero    -> NODE
    // xxxxxxxx       xxxxxxxx 
    //----------------------------
    // NO-OP
}
//****************************************************************************************

void setup_BBLEO_16OUT(void) {
    pinMode( 4, OUTPUT);  // D4   
    pinMode( 5, OUTPUT);  // D5   
    pinMode( 6, OUTPUT);  // D6   
    pinMode( 7, OUTPUT);  // D7  
    pinMode( 8, OUTPUT);  // D8   
    pinMode( 9, OUTPUT);  // D9   
    pinMode(10, OUTPUT);  // D10   
    pinMode(11, OUTPUT);  // D11
  
    pinMode(12, OUTPUT);  // D12  
    pinMode(13, OUTPUT);  // D13   
    pinMode(A0, OUTPUT);  // A0 
    pinMode(A1, OUTPUT);  // A1   
    pinMode(A2, OUTPUT);  // A2   
    pinMode(A3, OUTPUT);  // A3   
    pinMode(A4, OUTPUT);  // A4   
    pinMode(A5, OUTPUT);  // A5  
}
void pack_BBLEO_16OUT(void) {
    // IB0 All Zero   IB1 All Zero
    // 00000000       00000000
    //----------------------------
    IB[0] = 0;
    IB[1] = 0;  
}
void unpack_BBLEO_16OUT(void) {
    // OB0 8 bits     OB1 8 bits      -> NODE
    // OOOOOOOO       OOOOOOOO 
    //----------------------------
    // lower8
    digitalWrite(4,  (( OB[0] >> 0) &  0x01) );
    digitalWrite(5,  (( OB[0] >> 1) &  0x01) );
    digitalWrite(6,  (( OB[0] >> 2) &  0x01) );
    digitalWrite(7,  (( OB[0] >> 3) &  0x01) );
    digitalWrite(8,  (( OB[0] >> 4) &  0x01) );
    digitalWrite(9,  (( OB[0] >> 5) &  0x01) );
    digitalWrite(10, (( OB[0] >> 6) &  0x01) );
    digitalWrite(11, (( OB[0] >> 7) &  0x01) );
    // upper8
    digitalWrite(12, (( OB[1] >> 0) &  0x01) );
    digitalWrite(13, (( OB[1] >> 1) &  0x01) );
    digitalWrite(A0, (( OB[1] >> 2) &  0x01) );
    digitalWrite(A1, (( OB[1] >> 3) &  0x01) );
    digitalWrite(A2, (( OB[1] >> 4) &  0x01) );
    digitalWrite(A3, (( OB[1] >> 5) &  0x01) );
    digitalWrite(A4, (( OB[1] >> 6) &  0x01) );
    digitalWrite(A5, (( OB[1] >> 7) &  0x01) );
}
//****************************************************************************************

void setup_BBLEO_RSMC(void) {             //----------  Base Node with remote stall motor ---------- 
    pinMode( 4, OUTPUT);       // D4  SG1A-R   
    pinMode( 5, OUTPUT);       // D5  SG1A-Y   
    pinMode( 6, OUTPUT);       // D6  SG1A-G   
    pinMode( 7, OUTPUT);       // D7  SG1B-R  
    pinMode( 8, OUTPUT);       // D8  SG1B-Y   
    pinMode( 9, OUTPUT);       // D9  SG2-R   
    pinMode(10, OUTPUT);       // D10 SG2-Y   
    pinMode(11, OUTPUT);       // D11 SG2-G
    pinMode(12, OUTPUT);       // D12 SG3-R  
    pinMode(13, OUTPUT);       // D13 SG3-Y   
    pinMode(A0, OUTPUT);       // A0  SW1  Turnout throw input to RSMC
    
    pinMode(A1, INPUT_PULLUP);  // A1  AUX1   
    pinMode(A2, INPUT_PULLUP);  // A2  AUX2   
    pinMode(A3, INPUT_PULLUP);  // A3  TC1  Track Circuit 1   
    pinMode(A4, INPUT_PULLUP);  // A4  TC2  Track Circuit 2   
    pinMode(A5, INPUT_PULLUP);  // A5  OS1  OS Circuit 1 
}
void pack_BBLEO_RSMC(void) {
     // IB0 4 bits     IB1 All Zero
     // 00000III       00000000
     //----------------------------
    IB[0] = IB[1] = 0;
    IB[0] |= (!digitalRead(A1) << 0);
    IB[0] |= (!digitalRead(A2) << 1);
    IB[0] |= (!digitalRead(A3) << 2);
    IB[0] |= (!digitalRead(A4) << 3);
    IB[0] |= (!digitalRead(A5) << 4);
}
void unpack_BBLEO_RSMC(void) {
    // OB0 8 bits     OB1 4 bits      -> NODE
    // OOOOOOOO       xxxxxOOO 
    //----------------------------
    // lower8
    digitalWrite(4,  (( OB[0] >> 0) &  0x01) );
    digitalWrite(5,  (( OB[0] >> 1) &  0x01) );
    digitalWrite(6,  (( OB[0] >> 2) &  0x01) );
    digitalWrite(7,  (( OB[0] >> 3) &  0x01) );
    digitalWrite(8,  (( OB[0] >> 4) &  0x01) );
    digitalWrite(9,  (( OB[0] >> 5) &  0x01) );
    digitalWrite(10, (( OB[0] >> 6) &  0x01) );
    digitalWrite(11, (( OB[0] >> 7) &  0x01) );

    digitalWrite(12, (( OB[1] >> 0) &  0x01) );
    digitalWrite(13, (( OB[1] >> 1) &  0x01) );
    digitalWrite(A0, (( OB[1] >> 2) &  0x01) );
}
//****************************************************************************************

void setup_BBLEO_RSMC_LOCK(void) {    //----------  Base Node with remote stall motor & Lock---------- 
    pinMode( 4, OUTPUT);        // D4  SG1A-R   
    pinMode( 5, OUTPUT);        // D5  SG1A-Y   
    pinMode( 6, OUTPUT);        // D6  SG1A-G   
    pinMode( 7, OUTPUT);        // D7  SG1B-R  
    pinMode( 8, OUTPUT);        // D8  SG1B-Y   
    pinMode( 9, OUTPUT);        // D9  SG2-R   
    pinMode(10, OUTPUT);        // D10 SG2-Y   
    pinMode(11, OUTPUT);        // D11 SG2-G
    pinMode(12, OUTPUT);        // D12 SG3-R  
    pinMode(13, OUTPUT);        // D13 SG3-Y   
    pinMode(A0, OUTPUT);        // A0  SW Turnout throw input to RSMC
    pinMode(A1, OUTPUT);        // A1  SW Lock Control to Switch Lock controller   
  
    pinMode(A2, INPUT_PULLUP);  // A2  Fascia switch control   
    pinMode(A3, INPUT_PULLUP);  // A3  TC1  Track Circuit 1   
    pinMode(A4, INPUT_PULLUP);  // A4  TC2  Track Circuit 2   
    pinMode(A5, INPUT_PULLUP);  // A5  OS1  OS Circuit 1 
}
void pack_BBLEO_RSMC_LOCK(void) {
     // IB0 4 bits     IB1 All Zero
     // 0000IIII       00000000
     //----------------------------
    IB[0] = IB[1] = 0;
    IB[0] |= (!digitalRead(A2) << 0);
    IB[0] |= (!digitalRead(A3) << 1);
    IB[0] |= (!digitalRead(A4) << 2);
    IB[0] |= (!digitalRead(A5) << 3);
}
void unpack_BBLEO_RSMC_LOCK(void) {
    // OB0 8 bits     OB1 4 bits      -> NODE
    // OOOOOOOO       xxxxOOOO 
    //----------------------------
    // lower8
    digitalWrite(4,  (( OB[0] >> 0) &  0x01) );
    digitalWrite(5,  (( OB[0] >> 1) &  0x01) );
    digitalWrite(6,  (( OB[0] >> 2) &  0x01) );
    digitalWrite(7,  (( OB[0] >> 3) &  0x01) );
    digitalWrite(8,  (( OB[0] >> 4) &  0x01) );
    digitalWrite(9,  (( OB[0] >> 5) &  0x01) );
    digitalWrite(10, (( OB[0] >> 6) &  0x01) );
    digitalWrite(11, (( OB[0] >> 7) &  0x01) );

    digitalWrite(12, (( OB[1] >> 0) &  0x01) );
    digitalWrite(13, (( OB[1] >> 1) &  0x01) );
    digitalWrite(A0, (( OB[1] >> 2) &  0x01) );
    digitalWrite(A1, (( OB[1] >> 3) &  0x01) );
}

//****************************************************************************************
// ProMini has Arduino pins 
//     2- 9  routed out to BANK 1, and
//    10-A3  routed out to BANK 2
//
void setup_PROMINI_8OUT8IN(void) {
    pinMode( 2, OUTPUT);        // D2   
    pinMode( 3, OUTPUT);        // D3   
    pinMode( 4, OUTPUT);        // D4   
    pinMode( 5, OUTPUT);        // D5  
    pinMode( 6, OUTPUT);        // D6   
    pinMode( 7, OUTPUT);        // D7   
    pinMode( 8, OUTPUT);        // D8   
    pinMode( 9, OUTPUT);        // D9
  
    pinMode(10, INPUT_PULLUP);  // D10  
    pinMode(11, INPUT_PULLUP);  // D11   
    pinMode(12, INPUT_PULLUP);  // D12
    pinMode(13, INPUT_PULLUP);  // D13  
    pinMode(A0, INPUT_PULLUP);  // A0  
    pinMode(A1, INPUT_PULLUP);  // A1   
    pinMode(A2, INPUT_PULLUP);  // A2   
    pinMode(A3, INPUT_PULLUP);  // A3  
}
void pack_PROMINI_8OUT8IN(void) {
    // D10 - A3 are inputs
    // IB0 8 bits     IB1 All Zero
    // IIIIIIII       00000000
    //----------------------------
    IB[0] = IB[1] = 0;
    IB[0] |= (!digitalRead(10) << 0);
    IB[0] |= (!digitalRead(11) << 1);
    IB[0] |= (!digitalRead(12) << 2);
    IB[0] |= (!digitalRead(13) << 3);
    IB[0] |= (!digitalRead(A0) << 4);
    IB[0] |= (!digitalRead(A1) << 5);
    IB[0] |= (!digitalRead(A2) << 6);
    IB[0] |= (!digitalRead(A3) << 7);
}
void unpack_PROMINI_8OUT8IN(void) {
    // D2 - D9 are ouptuts
    // OB0 8 bits     OB1 All Zero
    // OOOOOOOO       xxxxxxxx 
    //----------------------------
    // lower8
    digitalWrite( 2, (( OB[0] >> 0) &  0x01) );
    digitalWrite( 3, (( OB[0] >> 1) &  0x01) );
    digitalWrite( 4, (( OB[0] >> 2) &  0x01) );
    digitalWrite( 5, (( OB[0] >> 3) &  0x01) );
    digitalWrite( 6, (( OB[0] >> 4) &  0x01) );
    digitalWrite( 7, (( OB[0] >> 5) &  0x01) );
    digitalWrite( 8, (( OB[0] >> 6) &  0x01) );
    digitalWrite( 9, (( OB[0] >> 7) &  0x01) );
}
//****************************************************************************************
//TODO:  create your own setup, pack and unpack functions like the above.
//       Suggest:  use the same "name" for these routines as the enum name you added up above
//                 in the same way the routines jyst above this comment do...



//*************************************************************************
//******       cpNode onboard I/O bit mapping for 16 bits            ******
//******          Only one mapping is active at a time               ******
//*************************************************************************

//----------------------------------------------------------------------
//  The setup routines set the direction of the various onboard pins
//  Perform the base cpNode setup at bootup
//----------------------------------------------------------------------


void SetUp_Node(void) {
    switch (NODE) {
    case BBLEO:           setup_BBLEO();           break;
    case BBLEO_8IN8OUT:   setup_BBLEO_8IN8OUT();   break;
    case BBLEO_8OUT8IN:   setup_BBLEO_8OUT8IN();   break;
    case BBLEO_12OUT4IN:  setup_BBLEO_12OUT4IN();  break;
    case BBLEO_16IN:      setup_BBLEO_16IN();      break;
    case BBLEO_16OUT:     setup_BBLEO_16OUT();     break;
    case BBLEO_RSMC:      setup_BBLEO_RSMC();      break;
    case BBLEO_RSMC_LOCK: setup_BBLEO_RSMC_LOCK(); break;
    case PROMINI_8OUT8IN: setup_PROMINI_8OUT8IN(); break;
    // TODO: Add new config setup_XXX here
    }
}

//----------------------------------------------------------------------
//  The input routines collect the bits from the digitalRead() calls and
//  put the them into the correct bytes for transmission.
//
//  The ports are read twice with a delay between for input debounce.
//  plocher: This comment does not match the actual code:
//       no debounce checking is done!
//       instead, the inputs are simply read twice, with the first
//       values discarded.
//  
//  Except for NODE == BASE_NONE, Two bytes are stored, IB[0] and IB[1]
//-----------------------------------------------------------------------
void Pack_Node_Inputs() {  
  for (byte i=0; i<2; i++)   {
      switch (NODE) {
      case BBLEO:           pack_BBLEO();           break;
      case BBLEO_8IN8OUT:   pack_BBLEO_8IN8OUT();   break;
      case BBLEO_8OUT8IN:   pack_BBLEO_8OUT8IN();   break;
      case BBLEO_12OUT4IN:  pack_BBLEO_12OUT4IN();  break;
      case BBLEO_16IN:      pack_BBLEO_16IN();      break;
      case BBLEO_16OUT:     pack_BBLEO_16OUT();     break;
      case BBLEO_RSMC:      pack_BBLEO_RSMC();      break;
      case BBLEO_RSMC_LOCK: pack_BBLEO_RSMC_LOCK(); break;
      case PROMINI_8OUT8IN: pack_PROMINI_8OUT8IN(); break;
      // TODO: Add new config unpack_XXX here

      }
     // Wait for debounce time
     //-----------------------
     delay(DEBOUNCE_DELAY);
  }
#ifndef ACTIVE_LOW_INPUTS
  IB[0] = ~(IB[0]);
  IB[1] = ~(IB[1]);
#endif
}

// --------------------------------------------------------------------------
//  The output routines takes received bits from the ouput buffer and 
//  writes them to the correct output port using digitalWrite() based 
//  upon the value of cpNode_ioMap
//---------------------------------------------------------------------------
void Unpack_Node_Outputs() {
        
    // Move the received bytes to the output buffer
    //---------------------------------------------
    for (byte i=0; i<nOB; i++) {
        OB[i] = CMRInet_Buf[i];
#ifndef ACTIVE_LOW_OUTPUTS
        OB[i] = ~(OB[i]);   // invert the bits if not active-low
#endif
    }

    switch (NODE) {
    case BBLEO:           unpack_BBLEO();           break;
    case BBLEO_8IN8OUT:   unpack_BBLEO_8IN8OUT();   break;
    case BBLEO_8OUT8IN:   unpack_BBLEO_8OUT8IN();   break;
    case BBLEO_12OUT4IN:  unpack_BBLEO_12OUT4IN();  break;
    case BBLEO_16IN:      unpack_BBLEO_16IN();      break;
    case BBLEO_16OUT:     unpack_BBLEO_16OUT();     break;
    case BBLEO_RSMC:      unpack_BBLEO_RSMC();      break;
    case BBLEO_RSMC_LOCK: unpack_BBLEO_RSMC_LOCK(); break;
    case PROMINI_8OUT8IN: unpack_PROMINI_8OUT8IN(); break;
    // TODO: Add new config unpack_XXX here
    }
}


//*************************************************************************
//**********            CMRInet SERIAL SUPPORT ROUTINES          **********
//*************************************************************************

//-----------------------------------
//CMRInet Option Bit ProcessING
//-----------------------------------
void Process_cpNode_Options() {
    // CPNODE IGNORES OPTION BITS
}

//-----------------------------------------------------------------------------------------
//  Perform any initialization and setup using the initialization message
//  NDP must be a "C" (cpNode) for initialization to be done
//  
//    - cpNode Initialization Message (I)
//      SYN SYN STX <UA> <I><NDP> <DLH><DLL> <opts1><opts2> <NIN><NOUT> <000000><ETX>  
//-----------------------------------------------------------------------------------------
void Initialize_cpNode() {
    // Set up transmit delay
    // 1 unit of delay(DL) is 10 microseconds
    //----------------------------------------
    DLH = CMRInet_Buf[1];
    DLL = CMRInet_Buf[2];
    DL  = (DLH * 256) + DLL;    // Transmit character delay value in 10 us increments
    DL  = DL * 10;

#if defined(MONITOR_SERIAL) && DEBUG_INIT
        MONITOR_SERIAL.print("INIT: ");
        MONITOR_SERIAL.print("DLH=");    MONITOR_SERIAL.print(DLH);
        MONITOR_SERIAL.print(" DLL=");   MONITOR_SERIAL.print(DLL);
        MONITOR_SERIAL.print(" DL/10="); MONITOR_SERIAL.print(DL/10);
        MONITOR_SERIAL.print(" DL=");    MONITOR_SERIAL.print(DL);
#endif
 
    // Check if initialize message is for a cpNode
    // if so, process any options
    //--------------------------------------------
    if (cpNODE_NDP == CMRInet_Buf[0]) {
        Process_cpNode_Options();
    } 
}


// -----------------------------------------------------
//  FLUSH the serial input buffer until an ETX
//  is seen or the input serial buffer is empty. 
// 
//  Used to ignore any inbound messages
//  not addressed to the node or to re-sync the protocol
//  parser if a garbled message found is.
// -----------------------------------------------------
void Flush_CMRInet_To_ETX() {
    boolean done = false;
    while (!done) {
        if (CMRI_SERIAL.available()) {
            if (CMRI_SERIAL.read() == ETX) {
                done = true;
            }
         } else {
            done = true;
         }
    }
}


// --------------------------------------------------------
//  Read a byte from the specified serial port.
//  Return the character read 
// --------------------------------------------------------
char Read_CMRI_Byte() {
    while (true) { 
        if (CMRI_SERIAL.available() > 0) {
            return char(CMRI_SERIAL.read());
        }
    }
}


// ----------------------------------------------------------------------
//  Read the message from the Host and determine if the message is 
//  for this node.  The whole message is read, any data to be processed
//  is stored in CMRInet_Buf[], stripped of protocol characters.
// 
//  If the node address does not match, the data is ignored.
//  
//  The data message body is processed by an appropriate message handler. 
//
//    - Initialization Packet (I)
//      SYN SYN STX <UA> <I> <NDP> <dH><dL> <NS> <CT(1)><CT(NS)> ETX
//
//    - Poll for Data (P)
//      SYN SYN STX <UA> <P> ETX
//
//    - Read Data (R)
//      SYN SYN STX <UA> <R> <IB(1)><IB(NS)> ETX
//----------------------------------------------------------------------

int CMRI_Read() {
    byte resp = respErr;
    int  inCnt;
        
    boolean reading = true,
            inData = false;
  
    //-----------------------------------
    // Check input buffer for a character
    //-----------------------------------
    if (CMRI_SERIAL.available() <= 0) {
       return respNone;
    }
     
    //--------------------
    // Process the message
    //--------------------
    matchID = 0;
    inCnt = 0;
    
    do {
       c = Read_CMRI_Byte();  // read the byte
  
       switch( int(c) ) {
        case STX:         // Start of message header, start parsing protocol message
#if defined(MONITOR_SERIAL) && DEBUG_PROTOCOL
          MONITOR_SERIAL.print("STX");
#endif
  
          // Read node address and message type
          //-----------------------------------       
          matchID = Read_CMRI_Byte();  // Node Address  
#if defined(MONITOR_SERIAL) && DEBUG_PROTOCOL
          MONITOR_SERIAL.print(" ua=");MONITOR_SERIAL.print(matchID-UA_Offset); MONITOR_SERIAL.print(" ");
#endif
          
          // If node ID does not match, exit and flush to ETX in outer loop
          //---------------------------------------------------------------
          if (matchID != UA)  {
#if defined(MONITOR_SERIAL) && DEBUG_PROTOCOL
              MONITOR_SERIAL.print("NOT MINE: "); MONITOR_SERIAL.println(matchID);
#endif
              resp = respIgnore;
              reading=false;
          } else {  
             // Set response code based upon message type
             //------------------------------------------
             c = Read_CMRI_Byte();        // Message Type
#if defined(MONITOR_SERIAL) && DEBUG_PROTOCOL
             MONITOR_SERIAL.print("MsgType=");MONITOR_SERIAL.print(char(c)); MONITOR_SERIAL.print(" IB=");
#endif
  
             switch( c ) {
               case 'I':          // Initialization
                                  resp = respInit;      break;
               case 'P':          // Poll
                                  resp = respPoll;      break;
               case 'R':          // Read
                                  resp = respRead;      break;
               case 'T':          // Write (Transmit)
                                  resp = respTransmit;  break;
               default:           // Unknown - Error
                                  resp = respErr;
                                  reading = false;
                                  break;
              }
  
            // Completed the header, go into message data mode
            //------------------------------------------------
            inData = true;     
         }
        break;
  
        case ETX:         // End of message, read complete
#if defined(MONITOR_SERIAL) && DEBUG_PROTOCOL
                          MONITOR_SERIAL.print(" ETX ");   
#endif      
                          reading = false;
                          break;
  
        case DLE:         // Read the next byte regardless of value and store it
                          CMRInet_Buf[inCnt++] = Read_CMRI_Byte();
#if defined(MONITOR_SERIAL) && DEBUG_PROTOCOL
                          MONITOR_SERIAL.print("DLE(");
                          MONITOR_SERIAL.print(byte(CMRInet_Buf[inCnt-1]),HEX);
                          MONITOR_SERIAL.print(") ");
#endif
                        break;
        
        case SYN:         // Sync character Ignore it if not reading data
                          if (inData) CMRInet_Buf[inCnt++] = c; 
#if defined(MONITOR_SERIAL) && DEBUG_PROTOCOL
                          MONITOR_SERIAL.print("SYN ");
#endif
                        break;
        
        default:  // Stuff the data character into the receive buffer
#if defined(MONITOR_SERIAL) && DEBUG_PROTOCOL
                         MONITOR_SERIAL.print("{"); MONITOR_SERIAL.print(byte(c),HEX); MONITOR_SERIAL.print("}");
#endif
                         CMRInet_Buf[inCnt++] = c;
                        break;       
      }    
  
      // Check for buffer overrun and terminate the read if true
      //--------------------------------------------------------     
      if (inCnt > CMRInet_BufSize) {
          reading = false;
          resp = respErr;
#if defined(MONITOR_SERIAL) && DEBUG_PROTOCOL
          MONITOR_SERIAL.print("\nBuffer Overrun inCnt = "); MONITOR_SERIAL.println(inCnt);
#endif
       }
    } while (reading);
  
    // Null terminate the input buffer
    //--------------------------------
    CMRInet_Buf[inCnt] = 0;

    //---------------------------------------------------------
    // Match the node address in the message to the UA+65 value
    // if no match, ignore message, not addessed to this node
    //--------------------------------------------------------- 

#if defined(MONITOR_SERIAL) && DEBUG_PROTOCOL
        MONITOR_SERIAL.print("\n ->inCnt = "); MONITOR_SERIAL.println(inCnt);
        MONITOR_SERIAL.println();
#endif

    return resp;
    
}  // CMRI READ


// ----------------------------------------------------------
// Send the input bytes to the host in response to a poll message.  
// Bytes are moved from the IB(n) buffer to the transmit buffer.
// DLE characters are inserted for data values which are also
// protocol characters.
// 
//    - Read Data (R) Message
//      SYN SYN STX <UA> <R><IB(1)><IB(NS)> ETX
//
//NB: Change to int if longer buffers are implemented
//------------------------------------------------------------*/
void CMRI_Poll_Resp() {
    byte i=0;
    
    // Packet Header
    //--------------
    CMRInet_Buf[i++] = SYN;
    CMRInet_Buf[i++] = SYN;
    CMRInet_Buf[i++] = STX;
    
    // Message Header
    //---------------
    CMRInet_Buf[i++] = UA;
    CMRInet_Buf[i++] = 'R';
    
    // Load the onboard input bytes into the output buffer
    //----------------------------------------------------
    if (nIB > 0) {
        for (byte j=0; j < nIB; j++) {
            c = IB[j];  // Insert a DLE if the output byte value is a protocol character
            switch(c)  {
            //  case SYN:   // Syncs are ignored to conform to the published protocol
                case STX:
                case ETX:
                case DLE:
                          CMRInet_Buf[i++] = DLE;
                          break;
            }
            CMRInet_Buf[i++] = c;
            IB[j] = 0;   // Clear the latched inputs
        }
    }

#ifdef USE_IOX
    // Load the IOX input bytes into the output buffer
    //------------------------------------------------
    if (numIOX_IN > 0) {
        for (byte j=0; j < numIOX_IN; j++) {
            c = IOX_inBuf[j];  // Insert a DLE if the output byte value is a protocol character
            switch(c)  {
            //   case SYN:    Syncs are ignored to conform to the published protocol
                 case STX:
                 case ETX:
                 case DLE:
                          CMRInet_Buf[i++] = DLE;
                          break;
            }
            CMRInet_Buf[i++] = c;
            IOX_inBuf[j] = 0;   // Clear the latched inputs
        }
    }
#endif
     
    // Add the ETX and send the complete buffer
    //-----------------------------------------
    CMRInet_Buf[i++] = ETX;
  
    // Send the packet to the host
    //----------------------------
    for (byte j=0; j<i; j++) {
        CMRI_SERIAL.write(CMRInet_Buf[j]);
     
        // If a transmit delay was set, delay microseconds
        //------------------------------------------------
        if (DL > 0) {
            delayMicroseconds( DL );   // value in microseconds
        }
    } 
  
#if defined(MONITOR_SERIAL) && DEBUG_POLL
        MONITOR_SERIAL.print("Poll Response "); MONITOR_SERIAL.print("nIDB= "); MONITOR_SERIAL.print(nIB,HEX); MONITOR_SERIAL.print(" "); 
        for (byte j=0; j<i; j++) {
            MONITOR_SERIAL.print(CMRInet_Buf[j],HEX); 
            MONITOR_SERIAL.print(" "); 
        }
        MONITOR_SERIAL.println();
#endif
}


void setup(void) {

    // *************************************************
    // *******            Setup               **********
    // *************************************************
   
    // RS-485 Port
    //------------
    pinMode(RX, INPUT);          // D0  CMRI RS485 Receive    
    pinMode(TX, OUTPUT);         // D1  CMRI RS485 Transmit 

    // Call the default initialization routine
    //----------------------------------------
    SetUp_Node();

    
    // Open the monitor port if in debug mode on a BBLEO
    //--------------------------------------------------
#if defined(MONITOR_SERIAL)
    if (debugging) {
        MONITOR_SERIAL.begin(MON_SPEED); 
        while(!MONITOR_SERIAL) { };
    } else {
        MONITOR_SERIAL.end();
    }
#endif

    // Open the CMRInet port
    //----------------------
    CMRI_SERIAL.begin(CMRINET_SPEED); 
    while(!CMRI_SERIAL) { };
    
    // Set the node address
    //---------------------
    nodeID = SetNodeAddr( nodeID );
  
    // Set up I/O Expander (IOX) access if present
    //--------------------------------------------
#ifdef USE_IOX
    Wire.begin();
    Configure_IOX_Ports();
    Setup_IOX_Inputs();
    Setup_IOX_Outputs();
#endif
   
    //---------------------------------------------------
    //  Set all outputs off if Common Anode LEDs are used
    //---------------------------------------------------
    byte initval = 0xEE;
    if (Init_CA_Ports_OFF) { initval = 0xFF; }
    
    for (byte i=0; i<nOB; i++) {
        CMRInet_Buf[i] = initval;
    }
    Unpack_Node_Outputs();
    
    for (byte i=0; i<numIOX_OUT; i++) {
       CMRInet_Buf[nOB + i] = initval;  
    }   
    Unpack_IOX_Outputs();


#if defined(MONITOR_SERIAL)
    if (debugging) {  // DEBUG_ANNOUNCE
        MONITOR_SERIAL.println(F("\nCMRI Node configuration:")); 
        MONITOR_SERIAL.print(F("    Baud Rate:        ")); MONITOR_SERIAL.println(CMRINET_SPEED, DEC);
        MONITOR_SERIAL.print(F("    Node ua:          ")); MONITOR_SERIAL.println(nodeID); 
        MONITOR_SERIAL.print(F("    Onboard:          ")); 
          MONITOR_SERIAL.print(nIB); MONITOR_SERIAL.print(F(" Inputs, "));
          MONITOR_SERIAL.print(nOB); MONITOR_SERIAL.println(F(" Outputs"));
#ifdef USE_IOX
        MONITOR_SERIAL.print(F("    IOX cards:        ")); 
          MONITOR_SERIAL.print(numIOX_IN); MONITOR_SERIAL.print(F(" Inputs, "));
          MONITOR_SERIAL.print(numIOX_OUT); MONITOR_SERIAL.println(F(" Outputs"));
#endif
        MONITOR_SERIAL.print(F("    Memory Available: "));  MONITOR_SERIAL.println(freeRam());
    }
#endif
}


// ***************************************************
// *******          Main Loop               **********
// ***************************************************
void loop() {
    //----------------------------------------------
    //  Check for any messages from the host
    //----------------------------------------------
    switch( CMRI_Read() ) {
      case respNone:     break;                                           // No data recveived, ignore
                       
      case respErr:      
                         // FALLTHROUGH
      case respIgnore:   Flush_CMRInet_To_ETX();                          // Flush input buffer to ETX for various reasons
                         break; 
      
      case respInit:     Initialize_cpNode();                             // "I" Initialize Message  HOST -> NODE, set configuration parameters
                         break;

                                                                          // "P" Poll Message     HOST -> NODE request for input data
      case respPoll:     CMRI_Poll_Resp();                                // "R" Receive Message  NODE -> HOST send input port data to host
                         break;
      
      case respTransmit: Unpack_Node_Outputs();                           // "T" Transmit (Write) Message  HOST -> NODE, set output bits
                         Unpack_IOX_Outputs();   
                         break;

      default:           Flush_CMRInet_To_ETX();
                         break;      
     }  
      
     // Read the input bits and latch for poll response
     //-----------------------------------------------
     Pack_Node_Inputs();   // Pack the onboard inputs
     Pack_IOX_Inputs();    // Pack any inputs off the I2C bus
}
