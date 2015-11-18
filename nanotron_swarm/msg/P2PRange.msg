# Software License Agreement (BSD License)
#
# Copyright (C) 2014 - 2015 Robotics, Vision and Control Group, University of Seville.
#   2014 Felipe R.Fabresse <fabresse@us.es>
#   2015 Felipe R.Fabresse <felramfab@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# This message represents a single range-only reading from a pair of devices 
# (emitter and receiver). 
# This message is  not appropriate for laser scanners or other non pair 
# to pair range sensor. See the LaserScan message if you are working with 
# a laser scanner and Range if you are working with non pair single active 
# ranger.

# In pair to pair range-only sensors the emitter sensor is the one which makes a 
# ranging request to another receiver device.

Header header

# Radiation type enumeration values
# If you want a value added to this list, send an email to felramfab@gmail.com
uint8 ULTRASOUND=0
uint8 RADIO=1

# Types of range devices (enumeration)
# If you want a value added to this list, send an email to felramfab@gmail.com
uint8 BASE=0 # Range sensor on-board a robot.
uint8 ANCHOR=1 # Range sensor (not BASE) which position is known.
uint8 BEACON=2 # Range sensor (not BASE) which position is unknown (mainly used in 
               # range-only mapping applications).

# The type of radiation used by the sensor
uint8 radiation_type    # (ULTRASOUND, RADIO, etc) [enum]

# Emitter sensor ID
uint8 emitter_id

# Type of the emitter range sensor
uint8 emitter_type      # (BASE, ANCHOR, BEACON, etc) [enum]

# Receiver sensor ID
uint8 receiver_id

# Type of the receiver range sensor
uint8 receiver_type      # (BASE, ANCHOR, BEACON, etc) [enum]

# Range measurement between emitter and receiver (meters)
float64 range

# Variance of range measurement.
float64 variance        # 0 is interpreted as variance unknown
