/**
 Software License Agreement (BSD License)

 Copyright (c) 2013-2015 Robotics, Vision and Control Group (GRVC), University of Seville.
    2013 Fernando Caballero <fcaballero@us.es>    
    2014 Felipe R.Fabresse <felramfab@us.es>
    2014 Alfredo Vazquez <avringeniero@gmail.com>
    2015 Felipe R.Fabresse <felramfab@gmail.com>
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef __CNTRONBASE_H__
#define __CNTRONBASE_H__

#include <termios.h>
#include <string>
#include <time.h>
#include <vector>
#include <map>

#define MAX_AGE 100
#define MAX_NO_ACK 20
#define PENALIZACION 5
#define DEFAULT_ID -1
// Max. time a beacon can be penalized (seconds)
#define MAX_PENALIZATION_TIME 15

#define RANGETO(ID) std::string("RangeTo ") + ID + "\r\n"
#define GETBEACONS(AGE) std::string("GetNodeIDList ") + #AGE + "\r\n"
#define READNODEID std::string("ReadNodeIDAdd\r\n")

//!Nanotron range data
struct CNTronRange {
	int emitterId;		// Unique emitter node ID
	int beaconId;		// Unique recever node ID
	int time;           // Age of the range.
	float range;		// Estimated range in meters
};

//!Nanotron node info
struct CNTronNode {
	int failures;
	int penalization;
	int id;
	timespec penalizationTime;
	char mac[13];
};

class CNTronBase {
public:

	//!Default constructor
	CNTronBase(int baseId = DEFAULT_ID);

	//!Default destructor
	~CNTronBase(void);

	/**
	 Intialize the serial port to the given values.
	 The function open the serial port in read-write mode

	 \param pDev Port of the serial device (e.g. '/dev/ttyUSB0')
	 
	 \return
	 -  0: success
	 - -1: error while open the device
	 - -2: erroneous parity
	 - -3: erroneous dataBits
	 - -4: erroneous stopBits
	 */
	int init(const char *pDev);

	//!Finish the serial communication (closes serial port)
	void finish(void);

	/**
	 * This function parse a complete message sent by the sensor board.
	 * First sends a RangeTo command and then reads the result.
	 * The beacon to be used for range request is selected automatically, reading periodically
	 * the list of available beacons and then selecting the next beacon to be ranged.
	 *
	 * Sensor information is sent in binary mode.
	 *
	 * \param node The node to be requested to range.
	 * \param pData Pointer to the structure where range information will be located.
	 * \param msg A reference to a string where any error message will be saved.
	 *
	 * \return
	 * - <0: Error
	 * -  0: No new data available
	 * -  1: New data available
	 **/
	int readRange(unsigned int node, CNTronRange& pData, std::string& msg);

	/**
	 * Return the current number of detected nodes.
	 **/
	unsigned int numBeacons() const;

	/**
	 * Checks if the penalization of a node must be set to 0 beacuse of a delay time.
	 *
	 * \param nodeInf The node information.
	 *
	 * \return The node penalization.
	 *
	 **/
	int checkPenalization(CNTronNode& nodeInf);

	//! Returns the unique identifier of the base node (node connected to PC).
	int getBaseId() const;

	/**
	 * Detects the list of beacons available.
	 *
	 * \return
	 *  - -1: Error sending command.
	 *  - -5: Empty buffer.
	 *  -  0: Beacons detected correctly.
	 */
	int detectBeacons();

	// Print functions
	void printBeacons() const;

protected:

	/**
	 * Reads the base node ID (MAC of node connected to serial port)
	 *
	 * return= -1; Error sending command. [ SendCommand() devuelve -6 ].
	 * return= -5; Empty serial buffer. [ read() devuelve 0 ].
	 * return= 0; Success.
	 **/
	int readBaseID();

	/**
	 * Sends a stream of data to the configured serial port.
	 *
	 * \param command Data stream
	 * \param size Length of the data stream.
	 *
	 * \return 0 if command was sent successfully -6 if any error. 
	 **/
	int sendCommand(const char* command, int size);

	/**
	 * Reads the last part of the MAC address given and returns the ID associated.
	 *
	 * This function reads the part of the MAC address associated to the unique ID
	 * of Nanotron range sensors, i.e. it reads the last 2 bytes of the MAC address.
	 *
	 * \param cnum Hexadecimal MAC address.
	 *
	 * \return The unique ID associated to a MAC address.
	 */
	int chartohex(char * cnum);

	//!Serial port handler
	int m_portHandler;

	//!Reading buffer
	char m_readBuff[256];

	//!Base Id
	int m_baseId;

	//!Nodes unique ID
	std::vector<int> m_nodeIDs;

	//!Nodes information
	std::map<int, CNTronNode> m_nodes;
};

#endif

