# MRCS_cpNode 
## License: Creative Commons Attribution-ShareAlike 3.0 Unported License

MRCS cpNode CMRI Kernel sketch

This sketch is the code template for a CMRI serial protocol node implemented in an Arduino style system board.  
The Modern Devices BBLeo, Bare Bones Leonardo (ATMega32u4) is the target Arduino style system board. 
  
Implements the CMRI Serial Protocol designed by Dr. Bruce Chubb and published publicly in various books, magazines, and articles.
  * The physical link is RS485, 4-wire, half duplex, serial.  
  * Each node has a one byte address in the range of 0-127.  
  * The Host polls the CMRI nodes for data.   
  * Port assignments represent the defined capability of a cpNode.
  * The base node data configuration is two bytes in, two bytes out.
  * All needed signal pins are connected via pin headers to the node board.  

Author:  Chuck Catania, 2013-2016

See
  * Eagle Project [MRCS cpNode](https://www.spcoast.com/pages/MRCS-cpNode.html)

Release Notes:

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
