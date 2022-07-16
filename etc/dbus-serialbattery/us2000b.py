# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function, unicode_literals
from battery import Protection, Battery, Cell
from utils import *
from struct import *
from pylontech import PylontechRS485, Rs485Handler
from pylontech import PylontechDecode
from pylontech import PylontechEncode


class PylontechStack:
    """! Whole battery stack abstraction layer.
    This class provides an easy-to-use interface to poll all batteries and get
    a ready calculated overall status for te battery stack.
    All Data polled is attached as raw result lists as well.
    """

    def __init__(self, device, baud=9600, manualBattcountLimit=15, timeout=5, group=0):
        """! The class initializer.
        @param device  RS485 device name (ex Windows: 'com0', Linux: '/dev/ttyUSB0').
        @param baud  RS485 baud rate. Usually 9500 or 115200 for pylontech.
        @param manualBattcountLimit  Class probes for the number of batteries in stack which takes a lot of time.
        @param group Group number if more than one battery groups are configured
        This parameter limits the probe action for quick startup. (especially useful for Testing)

        @return  An instance of the Sensor class initialized with the specified name.
        """
        self.pylon = PylontechRS485(device=device, baud=baud)
        self.encode = PylontechEncode()
        self.decode = PylontechDecode()

        self.timeout = timeout

        self.pylonData = {}
        self.group = group

        serialList = []
        for batt in range(0, manualBattcountLimit, 1):
            self.pylon.send(self.encode.getSerialNumber(
                battNumber=batt, group=self.group))
            raws = self.pylon.receive()
            if raws is None:
                break
            self.decode.decode_header(raws[0])
            decoded = self.decode.decodeSerialNumber()
            serialList.append(decoded['ModuleSerialNumber'])
        self.pylonData['SerialNumbers'] = serialList
        self.battcount = len(serialList)

        self.pylonData['Calculated'] = {}

    def update(self):
        """! Stack polling function.
        @return  A dict with all collected Information.
        """
        analoglList = []
        chargeDischargeManagementList = []
        alarmInfoList = []

        meanVoltage = 0
        totalCurrent = 0
        maxChargeCurrent = 0
        maxDischargeCurrent = 0
        totalCapacity = 0
        remainCapacity = 0
        power = 0
        for batt in range(0, self.battcount):
            self.pylon.send(self.encode.getAnalogValue(
                battNumber=batt, group=self.group))
            raws = self.pylon.receive()
            self.decode.decode_header(raws[0])
            decoded = self.decode.decodeAnalogValue()
            analoglList.append(decoded)
            remainCapacity = remainCapacity + decoded['RemainCapacity']
            totalCapacity = totalCapacity + decoded['ModuleTotalCapacity']
            power = power + (decoded['Voltage'] * decoded['Current'])
            meanVoltage = meanVoltage + decoded['Voltage']
            totalCurrent = totalCurrent + decoded['Current']

            self.pylon.send(self.encode.getChargeDischargeManagement(
                battNumber=batt, group=self.group))
            raws = self.pylon.receive()
            self.decode.decode_header(raws[0])
            decoded = self.decode.decodeChargeDischargeManagementInfo()
            chargeDischargeManagementList.append(decoded)
            maxChargeCurrent = maxChargeCurrent + decoded['ChargeCurrent']
            maxDischargeCurrent = maxDischargeCurrent + \
                decoded['DischargeCurrent']
            
            self.pylon.send(self.encode.getAlarmInfo(
                battNumber=batt, group=self.group))
            raws = self.pylon.receive()
            self.decode.decode_header(raws[0])
            decoded = self.decode.decodeAlarmInfo()
            alarmInfoList.append(decoded)

        self.pylonData['AnaloglList'] = analoglList
        self.pylonData['ChargeDischargeManagementList'] = chargeDischargeManagementList
        self.pylonData['AlarmInfoList'] = alarmInfoList

        self.pylonData['Calculated']['MaxChargeCurrent_A'] = 0
        self.pylonData['Calculated']['MaxDischargeCurrent_A'] = 0
        self.pylonData['Calculated']['MeanVoltage_V'] = 0
        self.pylonData['Calculated']['TotalCurrent_A'] = 0
        self.pylonData['Calculated']['TotalCapacity_Ah'] = 0
        self.pylonData['Calculated']['RemainCapacity_Ah'] = 0
        self.pylonData['Calculated']['Remain_Percent'] = 0
        self.pylonData['Calculated']['Power_kW'] = 0
        self.pylonData['Calculated']['ChargePower_kW'] = 0
        self.pylonData['Calculated']['DischargePower_kW'] = 0

        try:
            self.pylonData['Calculated']['MaxChargeCurrent_A'] = round(maxChargeCurrent, 2)
            self.pylonData['Calculated']['MaxDischargeCurrent_A'] = round(maxDischargeCurrent, 2)
            self.pylonData['Calculated']['MeanVoltage_V'] = round(meanVoltage / float(self.battcount), 2)
            self.pylonData['Calculated']['TotalCurrent_A'] = round(totalCurrent, 2)
            self.pylonData['Calculated']['TotalCapacity_Ah'] = totalCapacity
            self.pylonData['Calculated']['RemainCapacity_Ah'] = remainCapacity
            self.pylonData['Calculated']['Remain_Percent'] = round((remainCapacity / totalCapacity) * 100, 0)

            self.pylonData['Calculated']['Power_kW'] = round(power / 1000, 5)
            if self.pylonData['Calculated']['Power_kW'] > 0:
                self.pylonData['Calculated']['ChargePower_kW'] = self.pylonData['Calculated']['Power_kW']
                self.pylonData['Calculated']['DischargePower_kW'] = 0
            else:
                self.pylonData['Calculated']['ChargePower_kW'] = 0
                self.pylonData['Calculated']['DischargePower_kW'] = -1.0 * self.pylonData['Calculated']['Power_kW']
        except Exception as err:
            logger.error("pylonstack data error: " + str(err))

        return self.pylonData


class us2000b(Battery):

    def __init__(self, port, baud):
        super(us2000b, self).__init__(port, baud)
        self.stack = None
        self.type = self.BATTERYTYPE
        globals()['MAX_CELL_VOLTAGE'] = 52.5 / 15.0
        globals()['MIN_CELL_VOLTAGE'] = 47.8 / 15.0

    BATTERYTYPE = "US2000B"

    def test_connection(self):
        # call a function that will connect to the battery, send a command and retrieve the result.
        # The result or call should be unique to this BMS. Battery name or version, etc.
        # Return True if success, False for failure
        return self.read_status_data()

    def get_settings(self):
        # After successful  connection get_settings will be call to set up the battery.
        # Set the current limits, populate cell count, etc
        # Return True if success, False for failure
        self.capacity = 0
        self.cell_count = 0
        self.cell_max_voltage = 0
        self.cell_min_voltage = 0
        self.max_battery_voltage = 0
        self.min_battery_voltage = 0
        self.online = False

        data = self.read_pylon_stack()
        # check if connection success
        if data is False:
            return False

        try:
            self.capacity = data.pylonData['Calculated']['TotalCapacity_Ah']
            self.cell_count = 15
            self.cell_max_voltage = MAX_CELL_VOLTAGE
            self.cell_min_voltage = MIN_CELL_VOLTAGE
            self.max_battery_voltage = MAX_CELL_VOLTAGE * self.cell_count
            self.min_battery_voltage = MIN_CELL_VOLTAGE * self.cell_count
            self.online = True
        except:
            pass

        return True

    def refresh_data(self):
        result = self.read_bat_data()
        return result

    def read_status_data(self):
        result = self.read_bat_data()
        return result

    def read_bat_data(self):
        self.voltage = 0
        self.current = 0
        self.soc = 0
        self.max_battery_current = 0
        self.max_battery_discharge_current = 0
        self.online = False

        data = self.read_pylon_stack()
        # check if connection success
        if data is False:
            return False

        try:
            self.voltage = data.pylonData['Calculated']['MeanVoltage_V']
            self.current = data.pylonData['Calculated']['TotalCurrent_A']
            self.soc = data.pylonData['Calculated']['Remain_Percent']
            self.max_battery_current = data.pylonData['Calculated']['MaxChargeCurrent_A']
            self.max_battery_discharge_current = -1 * data.pylonData['Calculated']['MaxDischargeCurrent_A']
            self.online = True
            print(str(data.pylonData['Calculated']))
        except:
            pass

        return True

    def read_pylon_stack(self):
        try:
            if(self.stack == None):
                self.stack = PylontechStack(self.port, baud=115200, manualBattcountLimit=8)
            if(self.stack.pylon.rs485.ser.isOpen):
                self.stack.pylon.rs485.ser.close()
            self.stack.pylon.rs485 = Rs485Handler(self.port, baud=115200)
            if(self.stack.pylon.rs485.ser.isOpen):
                self.stack.update()
            else:
                return False
            if(self.stack.pylon.rs485.ser.isOpen):
                self.stack.pylon.rs485.ser.close()
            return self.stack
        except Exception as err:
            logger.error(">>> ERROR: PylontechStack update: " + str(err))
        
        return False

