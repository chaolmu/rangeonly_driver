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
#include "nanotron_swarm/cntronbase.h"

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string.h>
#include <vector>

CNTronBase::CNTronBase(int baseId) {
	m_portHandler = 0;
	m_baseId = baseId;
}

CNTronBase::~CNTronBase(void) {
	finish();
}

int CNTronBase::getBaseId(void) const {
	return m_baseId;
}

unsigned int CNTronBase::numBeacons() const {
	return m_nodes.size();
}

void CNTronBase::printBeacons() const {
	int nodeID;
	int penalization, failures;
	std::map<int, CNTronNode>::const_iterator it;

	std::cout << "\n[DEBUG] Detected Beacons (N = " << m_nodes.size() << "):"
			<< std::endl;
	for (unsigned int i = 0; i < m_nodes.size(); i++) {

		nodeID = m_nodeIDs[i];
		//std::cout << "[DEBUG] Node " << nodeID << std::endl;
		penalization = m_nodes.at(nodeID).penalization;
		failures = m_nodes.at(nodeID).failures;

		std::cout << "[DEBUG] Node " << nodeID << " - Penalization: "
				<< penalization << " - Failures: " << failures << std::endl;
	}
}

// Initialize the serial port to the given values
int CNTronBase::init(const char *pDev) {
	struct termios my_termios;

	// Make sure port is closed 
	if (m_portHandler > 0)
		close(m_portHandler);

	// Open the port in read-write mode 
	m_portHandler = open(pDev, O_RDWR | O_NOCTTY);
	if (m_portHandler < 0)
		return -5;

	/* Get the port attributes and flush all data on queues*/
	tcgetattr(m_portHandler, &my_termios);
	tcflush(m_portHandler, TCIOFLUSH);

	/* Setup the communication */
	my_termios.c_iflag &= ~(BRKINT | IGNPAR | PARMRK | INPCK | ISTRIP | IXON
			| INLCR | IGNCR | ICRNL);
	my_termios.c_iflag |= IGNBRK | IXOFF;
	my_termios.c_oflag &= ~(OPOST);
	my_termios.c_cflag |= CLOCAL | CREAD;
	my_termios.c_cflag &= ~PARENB;
	my_termios.c_cflag |= CS8;
	my_termios.c_cflag &= ~CSTOPB;
	my_termios.c_cflag &= ~CRTSCTS;
	my_termios.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL | ICANON | NOFLSH
			| TOSTOP | ISIG | IEXTEN);
	my_termios.c_cc[VMIN] = 1; //Each simple read call will be blocked until receive at least one byte

	//VTIME = Timeout. Is a character count ranging from 0 to 255 characters.
	//It is time measured in 0.1 second intervals, (0 to 25.5 seconds).
	//More information in http://www.unixwiz.net/techtips/termios-vmin-vtime.html

	my_termios.c_cc[VTIME] = 2.5;	//0 = No timeout for reading
	cfsetispeed(&my_termios, B115200);
	cfsetospeed(&my_termios, B115200);
	tcsetattr(m_portHandler, TCSANOW, &my_termios);

	/* Read base node ID */
	if (m_baseId == DEFAULT_ID)
		return readBaseID();

	/* Return 0 if no errors occurred */
	return 0;
}

// Close serial communication
void CNTronBase::finish(void) {
	/* Close the port */
	if (m_portHandler > 0) {
		close(m_portHandler);
		m_portHandler = 0;
	}
}

// Método que lee el ID de la base.
int CNTronBase::readBaseID() {
	std::string cmd = READNODEID;
	int result, size;
	char c;

	//Transformamos el objeto string 'cmd' en una cadena de caracteres char. 
	result = sendCommand(cmd.c_str(), cmd.size());
	if (result < 0)
		return -1;

	//	std::cout << "[DEBUG] Reading base MAC ..." << std::endl;

	// Read result
	c = 0;
	size = 0;
	while (c != '\n') {
		result = read(m_portHandler, &c, 1);
		if (result < 0) {
			usleep(10000);
			return -5;
		}

		if (result == 1) {
			m_readBuff[size++] = c;
		}
	}
	m_readBuff[size] = '\0';

	//std::cout << "[DEBUG] Base MAC: " << m_readBuff;

	//Nos quedamos con los primeros 2 bytes de la dirección MAC.
	//En este punto limitamos el rango de valor de ID a 0-255.

	char dirMAC[12];
	strncpy(dirMAC,m_readBuff,12);

	m_baseId=chartohex(dirMAC);

	return 0;
	/* Posibles valores de return:
	 return= -1; Error al enviar el comando.           [ SendCommand() devuelve -6 ].
	 return= -5; Error al encontrarse el buffer vacío. [ read() devuelve 0 ].
	 return= 0; Lectura del ID de la base realizada con éxito.
	 */
}

// Send a new data stream to the serial port
int CNTronBase::sendCommand(const char* command, int size) {
	int i = 1;
	int dataWritten;

	for (i = 0; i < size; i++) {
		dataWritten = write(m_portHandler, command + i, 1);
		if (dataWritten < 0)
			return -6;
	}
	return 0;
}

// Sends a range command and reads the result which is returned to user
int CNTronBase::readRange(unsigned int node, CNTronRange& pData,
		std::string& msg) {

	int result, size, err_code, returnCode;
	float range;
	char c;
	int nodeID;
	std::map<int, CNTronNode>::iterator it;
	CNTronNode nodeInf;

	if (node < m_nodes.size()) {
		nodeID = m_nodeIDs[node];
		nodeInf = m_nodes[nodeID];
	} else {
		msg = "Node not detected";
		return -1;
	}

	if (checkPenalization(nodeInf) != 0) {
		return 0;
	}

	// Send range request
	//std::cout<<"[DEBUG] RANGING: Tx "<< nodeInf.mac << std::endl;
	std::string cmd = RANGETO(nodeInf.mac);
	sendCommand(cmd.c_str(), cmd.length());

	// Read result
	c = 0;
	size = 0;
	while (c != '\n') {
		result = read(m_portHandler, &c, 1);
		if (result < 0) {
			usleep(10000);
			return -5;
		}

		if (result == 1) {
			m_readBuff[size++] = c;
		}
	}
	m_readBuff[size] = '\0';
	sscanf(m_readBuff, "%d, %f, %*d", &err_code, &range);

	returnCode = 0;
	/*CODIGOS DE ERROR QUE PODEMOS RECIBIR:
	 Errorcode = 0: Indica el estado de la operación de ranging.
	 Errorcode = 1: success ranging result valid
	 Errorcode = 2: ranging to own ID
	 Errorcode = 3: ID out of range, no ACK
	 Errorcode = 4: ranging unsuccessful, ACK OK, then timeout
	 */
	std::stringstream ss;
	switch (err_code) {
	case 0: // Success

		nodeInf.failures = 0;
		pData.beaconId = nodeInf.id;

		// sscanf(m_beacons.at(m_lastBeacon).c_str(), "%*2x%2x%*2x%*2x%*2x%*2x",pData.beaconId);
		pData.emitterId = m_baseId;
		pData.range = range;

		if (range < 0) {
			msg = "[WARNING] Negative range received";
		} else {
			returnCode = 1;
		}

		break;
	case 2: // Ranging to own ID. ID out of range, no ACK
		// msg = "[WARNING] " + m_beacons.at(m_lastBeacon) + " is de BASE ID. Invalid ID to range (NO_ACK).";
		//m_b_noack[m_lastBeacon]= m_b_noack[m_lastBeacon]+1;

		//break;
	case 3: // ID out of range, no ACK 
		nodeInf.failures++;

		//ss << "[WARNING] " << nodeInf.id << " is out of range (NO_ACK).";
		//msg = ss.str();

		break;
	case 4: // Only one ranging operation successful in diversity mode
		msg = "[WARNING] Timeout while ranging with " + nodeInf.id;
		break;
	default:
		// Do nothing
		break;
	}

	if (nodeInf.failures >= MAX_NO_ACK) {
		nodeInf.penalization = PENALIZACION;
		if (clock_gettime(CLOCK_MONOTONIC_RAW, &nodeInf.penalizationTime)
				== -1) {
			nodeInf.penalizationTime.tv_sec = 0;
			nodeInf.penalizationTime.tv_nsec = 0;
		}
	}

	m_nodes[nodeID] = nodeInf;

	return returnCode;
}

int CNTronBase::checkPenalization(CNTronNode& nodeInf) {
	timespec now;
	long delay = -1;

	if (nodeInf.penalization > 0) {	
		if (clock_gettime(CLOCK_MONOTONIC_RAW, &now) != -1) {
			delay = now.tv_sec
					- nodeInf.penalizationTime.tv_sec;
		}

		if (delay > 0 && delay > MAX_PENALIZATION_TIME) {
			nodeInf.penalization = 0;
			nodeInf.failures = 0;
		}
	}

	return nodeInf.penalization;
}

// Updates the list of available beacons
int CNTronBase::detectBeacons() {
	int result, size, lines, line = 0, age;
	char c;
	std::string cmd = GETBEACONS(255);
	char id[12]; // MAC of a beacon (contains the ID of the beacon).
	CNTronNode node;

	result = sendCommand(cmd.c_str(), cmd.size());
	if (result < 0)
		return -1;

	// Find the beginning of the stream
	c = 0;
	while (c != '#') {
		result = read(m_portHandler, &c, 1);
		if (result < 0) {
			usleep(10000);
			return -5;
		}
	}

	// Read first line (number of remaining lines)
	c = 0;
	size = 0;
	while (c != '\n') {
		result = read(m_portHandler, &c, 1);
		if (result < 0) {
			usleep(10000);
			return -5;
		}
		if (result == 1) {
			m_readBuff[size++] = c;
		}
	}
	m_readBuff[size] = '\0';
	sscanf(m_readBuff, "%d", &lines);

	// Read list of nodes detected
	for (line = 1; line <= lines; line++) {
		// Read new line
		c = 0;
		size = 0;
		while (c != '\n') {
			result = read(m_portHandler, &c, 1);
			if (result < 0) {
				usleep(10000);
				return -5;
			}

			if (result == 1) {
				m_readBuff[size++] = c;
			}
		}

		// IDs start at line 3
		if (line >= 3) {
			m_readBuff[size] = '\0';

			sscanf(m_readBuff, "%d:%s", &age, id);

			if (age <= MAX_AGE) {

				int nodeID = chartohex(id);
				if (m_nodes.count(nodeID) == 0) { // Add a new beacon
					node.failures = 0;
					node.penalization = 0;
					node.id = nodeID;

					strncpy(node.mac, id, 12);
					node.mac[12] = '\0';

					m_nodes[nodeID] = node;
					m_nodeIDs.push_back(nodeID);
				} else { // Check penalization
					if (checkPenalization(m_nodes[nodeID]) > 0) {
						m_nodes[nodeID].penalization--;
					}
				}
			}
		}
	}

#if DEBUG
	printBeacons();
#endif

	return 0;
}

int CNTronBase::chartohex(char * cnum) {
	int result = 0;
	int num[2];
	int i;

	i = 10; // Penultimate part of MAC address
	if ((cnum[i] >= 'A') && (cnum[i] <= 'F'))
		num[1] = (cnum[i] - 'A') + 10;
	else
		num[1] = (cnum[i] - '0');

	i = 11; // Last part of MAC address
	if ((cnum[i] >= 'A') && (cnum[i] <= 'F'))
		num[0] = (cnum[i] - 'A') + 10;
	else
		num[0] = (cnum[i] - '0');

	result = num[1] * 16 + num[0];

	return result;
}
