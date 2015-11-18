/**
 Software License Agreement (BSD License)

 Copyright (c) 2013-2015 Robotics, Vision and Control Group (GRVC), University of Seville.
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
#include "nanotron_swarm/P2PRange.h"

#include <iostream>
#include <unistd.h>
#include <ros/ros.h>

#define DEFAULT_PORT "/dev/ttyUSB0"
#define DEFAULT_FRAME_ID "/base_nanotron"

void detectBeacons(CNTronBase& nodoBase);
bool rangingRequest(CNTronBase& nodoBase, int node, CNTronRange& nodo);
void publishRange(CNTronRange& range, ros::Publisher publisher, std::string frameID);

int main(int argc, char** argv) {

	CNTronBase baseNode = CNTronBase();
	bool validRange;

	//ROS initialization.
	ros::init(argc, argv, "nanotron_swarm");
	ros::NodeHandle n;
	ros::NodeHandle privNode("~");

	// Get serial port
	std::string devicePort = DEFAULT_PORT;
	privNode.getParam("device", devicePort);

	// Get frame id
	std::string frameID = DEFAULT_FRAME_ID;
	privNode.getParam("frame_id", frameID);

	// Open serial device port
	ROS_INFO("Initializing range sensor on %s", devicePort.c_str());
	int baseID = baseNode.init(devicePort.c_str());
	if (baseID < 0) {
		ROS_ERROR("Device initialization error: %s", devicePort.c_str());
	} else {
		ROS_INFO("The ID of the base node is: %d", baseNode.getBaseId());
	}

	std::string topic = "range";
	privNode.getParam("topic", topic);
	ros::Publisher rangePub = n.advertise<nanotron_swarm::P2PRange>(topic,
			1000);
	ROS_INFO("Range measurements published on %s", rangePub.getTopic().c_str());

	// Structure which contains the information provided by a range sensor.
	CNTronRange range;

	// Main loop
	while (ros::ok()) {
		detectBeacons(baseNode);

		for (unsigned int i = 0; i < baseNode.numBeacons(); i++) {

			validRange = rangingRequest(baseNode, i, range);

			if (validRange)
				publishRange(range, rangePub, frameID);
		}
	}

	// Close serial device port
	baseNode.finish();

	return 0;
}

void detectBeacons(CNTronBase& baseNode) {
	int result = baseNode.detectBeacons();

	switch (result) {
	case -1:
		ROS_ERROR("Command GetNodeIDList could not be sent.");
		break;
	case -5:
		ROS_ERROR("Empty buffer when reaing the list of available nodes.");
		break;
	case 0:
		// Success
		break;
	default:
		ROS_ERROR(
				"Unknown error while trying to read the list of available nodes.");
		break;
	}
}

bool rangingRequest(CNTronBase& baseNode, int node, CNTronRange& range) {

	bool valid = false;
	std::string msgerr;

	int result = baseNode.readRange(node, range, msgerr);

	switch (result) {
	case 0:
		if (!msgerr.empty())
			ROS_ERROR("%s", msgerr.c_str());
		break;
	case -5:
		ROS_ERROR("Empty buffer while trying to read a range measurement");
		break;
	case 1:
		valid = true;
		// Success
		ROS_DEBUG("RANGING: Tx %d --> Rx %d => Distance %f m.\n",
				range.emitterId, range.beaconId, range.range);
		break;
	default:
		ROS_ERROR("Unknown Error while ranging");
		break;
	}

	return valid;
}

void publishRange(CNTronRange& range, ros::Publisher publisher,
		std::string frameID) {

	static long sequence = 0;

	nanotron_swarm::P2PRange msg;

	msg.header.stamp = ros::Time::now();
	msg.header.seq = sequence++;
	msg.header.frame_id = frameID;

	msg.radiation_type = nanotron_swarm::P2PRange::RADIO; // The type of radiation used by the sensor.

	msg.emitter_id = range.emitterId;                 // Unique emitter node ID.
	msg.emitter_type = nanotron_swarm::P2PRange::BASE; // Type of the emitter range sensor.

	msg.receiver_id = range.beaconId;                // Unique recever node ID.
	msg.receiver_type = nanotron_swarm::P2PRange::BEACON; // Type of the receiver range sensor.

	msg.range = (double) range.range;              // Estimated range in meters.
	msg.variance = 2.25;                       // Variance of range measurement.

	publisher.publish(msg);

}
