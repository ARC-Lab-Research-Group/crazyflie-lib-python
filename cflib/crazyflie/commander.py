#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Used for sending control setpoints to the Crazyflie
"""
import struct

from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort

__author__ = 'Bitcraze AB'
__all__ = ['Commander']

TYPE_STOP = 0
TYPE_VELOCITY_WORLD = 1
TYPE_ZDISTANCE = 2
TYPE_HOVER = 5
TYPE_POSITION = 7
TYPE_LQR = 8


class Commander():
    """
    Used for sending control setpoints to the Crazyflie
    """

    def __init__(self, crazyflie=None):
        """
        Initialize the commander object. By default the commander is in
        +-mode (not x-mode).
        """
        self._cf = crazyflie
        self._x_mode = False

    def set_client_xmode(self, enabled):
        """
        Enable/disable the client side X-mode. When enabled this recalculates
        the setpoints before sending them to the Crazyflie.
        """
        self._x_mode = enabled

    def send_setpoint(self, roll, pitch, yaw, thrust):
        """
        Send a new control setpoint for roll/pitch/yaw/thrust to the copter

        The arguments roll/pitch/yaw/trust is the new setpoints that should
        be sent to the copter
        """
        if thrust > 0xFFFF or thrust < 0:
            raise ValueError('Thrust must be between 0 and 0xFFFF')

        if self._x_mode:
            roll, pitch = 0.707 * (roll - pitch), 0.707 * (roll + pitch)

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER
        pk.data = struct.pack('<fffH', roll, pitch, yaw, thrust)
        self._cf.send_packet(pk)

    def send_stop_setpoint(self):
        """
        Send STOP setpoing, stopping the motors and (potentially) falling.
        """
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<B', TYPE_STOP)
        self._cf.send_packet(pk)

    def send_velocity_world_setpoint(self, vx, vy, vz, yawrate):
        """
        Send Velocity in the world frame of reference setpoint.

        vx, vy, vz are in m/s
        yawrate is in degrees/s
        """
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<Bffff', TYPE_VELOCITY_WORLD,
                              vx, vy, vz, yawrate)
        self._cf.send_packet(pk)

    def send_zdistance_setpoint(self, roll, pitch, yawrate, zdistance):
        """
        Control mode where the height is send as an absolute setpoint (intended
        to be the distance to the surface under the Crazflie).

        Roll, pitch, yawrate are defined as degrees, degrees, degrees/s
        """
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<Bffff', TYPE_ZDISTANCE,
                              roll, pitch, yawrate, zdistance)
        self._cf.send_packet(pk)

    def send_hover_setpoint(self, vx, vy, yawrate, zdistance):
        """
        Control mode where the height is send as an absolute setpoint (intended
        to be the distance to the surface under the Crazflie).

        vx and vy are in m/s
        yawrate is in degrees/s
        """
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<Bffff', TYPE_HOVER,
                              vx, vy, yawrate, zdistance)
        self._cf.send_packet(pk)

    def send_position_setpoint(self, x, y, z, yaw):
        """
        Control mode where the position is sent as absolute x,y,z coordinate in
        meter and the yaw is the absolute orientation.

        x and y are in m
        yaw is in degrees
        """
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<Bffff', TYPE_POSITION,
                              x, y, z, yaw)
        self._cf.send_packet(pk)

    def send_LQR_setpoint(self, x_c):
        """
        26 bytes
        The command sent to the crazyflie is the state vector
        [x y z phi theta psi dx dy dz] in m, m/s and rad
        and the nominal input [T p q r]

        This function encodes the state vector because
        the crazyflie expects units of mm and rad to be able to
        compress the floats into int_16t and send over crazyradio
        """

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC

        # Encode postion [x y z] to mm
        x = int(x_c[0]*1000.0)
        y = int(x_c[1]*1000.0)
        z = int(x_c[2]*1000.0)

        # Encode attitude [phi theta psi] to millirad
        roll = int(x_c[3]*1000)
        pitch = int(x_c[4]*1000)
        yaw = int(x_c[5]*1000)

        # Encode velocity [vx vy vz] to mm/s
        vx = int(x_c[6]*1000.0)
        vy = int(x_c[7]*1000.0)
        vz = int(x_c[8]*1000.0)

        # Encode thrust to mm/s^2
        thrust = int(x_c[9]*1000.0)

        # Encode angular velocity [p q r] to millirad/s
        p = int(x_c[10]*1000.0)
        q = int(x_c[11]*1000.0)
        r = int(x_c[12]*1000.0)

        pk.data = struct.pack('<Bhhhhhhhhhhhhh', TYPE_LQR, x, y, z,
                              roll, pitch, yaw, vx, vy, vz,
                              thrust, p, q, r)

        self._cf.send_packet(pk)

    def send_SDLQR_row_K(self, row, data):
        """
        19 bytes
        Send a row of the Kalman Gain matrix to the Crazyflie
        The row is packed as a uint8_t and the data for each
        entry of K is compressed (x1000) into int16_t[9]

        """
        # Check the row is valid [0,3]
        if not isinstance(row,int) or row > 3 or row < 0:
            raise ValueError('Row must be a valid integer between 0 and 3')
        # Check length of data tuple
        if len(a) != 9:
            raise ValueError('Expected 9 entries in data tuple for K matrix')

        # Init packet
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_SDLQR

        # Compress the data tuple
        for i in range(len(a)):
            data[i] = int(data[i]*1000.0)

        # Populate packet and send
        pk.data = struct.pack('<BBhhhhhhhhh', row, data[0], data[1], data[2],
                                                   data[3], data[4], data[5],
                                                   data[6], data[7], data[8])
        self._cf.send_packet(pk)
