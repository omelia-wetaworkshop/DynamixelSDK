#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

from .robotis_def import *
from .protocol2_packet_handler import PKT_ID, PKT_PARAMETER0

PARAM_NUM_DATA = 0
PARAM_NUM_ADDRESS = 1
PARAM_NUM_LENGTH = 2


class GroupBulkRead:
    def __init__(self, port, ph):
        self.port = port
        self.ph = ph

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if not self.data_dict:
            return

        self.param = []

        for dxl_id in self.data_dict:
            if self.ph.getProtocolVersion() == 1.0:
                self.param.append(self.data_dict[dxl_id][2])  # LEN
                self.param.append(dxl_id)  # ID
                self.param.append(self.data_dict[dxl_id][1])  # ADDR
            else:
                self.param.append(dxl_id)  # ID
                self.param.append(DXL_LOBYTE(self.data_dict[dxl_id][1]))  # ADDR_L
                self.param.append(DXL_HIBYTE(self.data_dict[dxl_id][1]))  # ADDR_H
                self.param.append(DXL_LOBYTE(self.data_dict[dxl_id][2]))  # LEN_L
                self.param.append(DXL_HIBYTE(self.data_dict[dxl_id][2]))  # LEN_H

    def addParam(self, dxl_id, start_address, data_length):
        if dxl_id in self.data_dict:  # dxl_id already exist
            return False

        data = []  # [0] * data_length
        self.data_dict[dxl_id] = [data, start_address, data_length]

        self.is_param_changed = True
        return True

    def removeParam(self, dxl_id):
        if dxl_id not in self.data_dict:  # NOT exist
            return

        del self.data_dict[dxl_id]

        self.is_param_changed = True

    def clearParam(self):
        self.data_dict.clear()
        return

    async def txPacket(self):
        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        if self.ph.getProtocolVersion() == 1.0:
            return await self.ph.bulkReadTx(self.port, self.param, len(self.data_dict.keys()) * 3)
        else:
            return await self.ph.bulkReadTx(self.port, self.param, len(self.data_dict.keys()) * 5)

    async def rxPacket(self):
        self.last_result = False

        result = COMM_RX_FAIL

        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        for dxl_id in self.data_dict:
            self.data_dict[dxl_id][PARAM_NUM_DATA], result, _ = await self.ph.readRx(self.port, dxl_id,
                                                                               self.data_dict[dxl_id][PARAM_NUM_LENGTH])
            if result != COMM_SUCCESS:
                return result

        if result == COMM_SUCCESS:
            self.last_result = True

        return result

    async def txRxPacket(self):
        async with self.port.lock:
            result = await self.txPacket()
            if result != COMM_SUCCESS:
                return result

            return await self.rxPacket()

    def isAvailable(self, dxl_id, address, data_length):
        if self.last_result is False or dxl_id not in self.data_dict:
            return False

        start_addr = self.data_dict[dxl_id][PARAM_NUM_ADDRESS]

        if (address < start_addr) or (start_addr + self.data_dict[dxl_id][PARAM_NUM_LENGTH] - data_length < address):
            return False

        return True

    def getData(self, dxl_id, address, data_length):
        if not self.isAvailable(dxl_id, address, data_length):
            return 0

        start_addr = self.data_dict[dxl_id][PARAM_NUM_ADDRESS]

        if data_length == 1:
            return self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr]
        elif data_length == 2:
            return DXL_MAKEWORD(self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr],
                                self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 1])
        elif data_length == 4:
            return DXL_MAKEDWORD(DXL_MAKEWORD(self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 0],
                                              self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 1]),
                                 DXL_MAKEWORD(self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 2],
                                              self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 3]))
        else:
            return 0

class GroupFastBulkRead(GroupBulkRead):
    def makeParam(self):
        if not self.data_dict or self.ph.getProtocolVersion() == 1.0:
            return

        self.param = []

        for dxl_id, data in self.data_dict.items():
            self.param.append(dxl_id)  # ID
            self.param.append(DXL_LOBYTE(data[PARAM_NUM_ADDRESS]))  # ADDR_L
            self.param.append(DXL_HIBYTE(data[PARAM_NUM_ADDRESS]))  # ADDR_H
            self.param.append(DXL_LOBYTE(data[PARAM_NUM_LENGTH]))  # LEN_L
            self.param.append(DXL_HIBYTE(data[PARAM_NUM_LENGTH]))  # LEN_H

    async def txPacket(self):
        if self.ph.getProtocolVersion() != 2.0 or not self.data_dict:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return await self.ph.fastBulkReadTx(self.port, self.param, len(self.data_dict.keys()) * 5)

    async def rxPacket(self):
        self.last_result = False

        result = COMM_RX_FAIL

        if self.ph.getProtocolVersion() == 1.0 or not self.data_dict:
            return COMM_NOT_AVAILABLE

        while True:
            rxpacket, result = await self.ph.rxPacket(self.port, skip_stuffing=True)
            if result != COMM_SUCCESS or rxpacket[PKT_ID] == BROADCAST_ID:
                break
        if result == COMM_SUCCESS and rxpacket[PKT_ID] == BROADCAST_ID:
            index = PKT_PARAMETER0
            for data in self.data_dict.values():
                data[PARAM_NUM_DATA] = rxpacket[index + 2:index + 2 + data[PARAM_NUM_LENGTH]]
                index += data[PARAM_NUM_LENGTH] + 4
            self.last_result = True

        return result
