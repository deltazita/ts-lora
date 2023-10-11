# modified version of https://github.com/ehong-tl/micropySX126X
from _sx126x import *
from machine import SPI, Pin
from time import sleep_us, ticks_ms, sleep_ms, ticks_diff, ticks_us

class SX126X:

    def __init__(self, spi_bus, clk, mosi, miso, cs, irq, rst, gpio):
        self._irq = irq
        try:
            self.spi = SPI(spi_bus, mode=SPI.MASTER, baudrate=2000000, pins=(clk, mosi, miso))        # Pycom variant uPy
        except:
            self.spi = SPI(spi_bus, baudrate=2000000, sck=Pin(clk), mosi=Pin(mosi), miso=Pin(miso))   # Generic variant uPy
        self.cs = Pin(cs, mode=Pin.OUT)
        self.irq = Pin(irq, mode=Pin.IN)
        self.rst = Pin(rst, mode=Pin.OUT)
        self.gpio = Pin(gpio, mode=Pin.IN)

        self._bwKhz = 0
        self._sf = 0
        self._bw = 0
        self._cr = 0
        self._ldro = 0
        self._crcType = 0
        self._preambleLength = 0
        self._tcxoDelay = 0
        self._headerType = 0
        self._implicitLen = 0
        self._txIq = 0
        self._rxIq = 0
        self._invertIQ = 0
        self._ldroAuto = True

        self._br = 0
        self._freqDev = 0
        self._rxBw = 0
        self._rxBwKhz = 0
        self._pulseShape = 0
        self._addrComp = 0
        self._syncWordLength = 0
        self._whitening = 0
        self._packetType = 0
        self._dataRate = 0
        self._packetLength = 0
        self._preambleDetectorLength = 0

    def begin(self, bw, sf, cr, syncWord, currentLimit, preambleLength, tcxoVoltage, useRegulatorLDO=False, txIq=False, rxIq=False):
        self._bwKhz = bw
        self._sf = sf

        self._bw = SX126X_LORA_BW_125_0
        self._cr = SX126X_LORA_CR_4_7
        self._ldro = 0x00
        self._crcType = SX126X_LORA_CRC_ON
        self._preambleLength = preambleLength
        self._tcxoDelay = 0
        self._headerType = SX126X_LORA_HEADER_EXPLICIT
        self._implicitLen = 0xFF

        self._txIq = txIq
        self._rxIq = rxIq
        self._invertIQ = SX126X_LORA_IQ_STANDARD

        state = self.reset()
        ASSERT(state)

        state = self.standby()
        ASSERT(state)

        state = self.config(SX126X_PACKET_TYPE_LORA)
        ASSERT(state)

        if tcxoVoltage > 0.0:
            state = self.setTCXO(tcxoVoltage)
            ASSERT(state)

        state = self.setSpreadingFactor(sf)
        ASSERT(state)

        state = self.setBandwidth(bw)
        ASSERT(state)

        state = self.setCodingRate(cr)
        ASSERT(state)

        state = self.setSyncWord(syncWord)
        ASSERT(state)

        state = self.setCurrentLimit(currentLimit)
        ASSERT(state)

        state = self.setPreambleLength(preambleLength)
        ASSERT(state)

        state = self.setDio2AsRfSwitch(True)
        ASSERT(state)

        if useRegulatorLDO:
            state = self.setRegulatorLDO()
        else:
            state = self.setRegulatorDCDC()

        return state


    def reset(self, verify=True):
        self.rst.value(1)
        sleep_us(150)
        self.rst.value(0)
        sleep_us(150)
        self.rst.value(1)
        sleep_us(150)

        if not verify:
            return ERR_NONE

        start = ticks_ms()
        while True:
            state = self.standby()
            if state == ERR_NONE:
                return ERR_NONE
            if abs(ticks_diff(start, ticks_ms())) >= 3000:
                return state
            sleep_ms(10)

    def transmit(self, data, len_, addr=0):
        state = self.standby()
        ASSERT(state)

        if len_ > SX126X_MAX_PACKET_LENGTH:
            return ERR_PACKET_TOO_LONG

        timeout = 0

        modem = self.getPacketType()
        if modem == SX126X_PACKET_TYPE_LORA:
            timeout = int((self.getTimeOnAir(len_) * 3) / 2)
        else:
            return ERR_UNKNOWN

        state = self.startTransmit(data, len_, addr)
        ASSERT(state)

        start = ticks_us()
        while not self.irq.value():
            yield_()
            if abs(ticks_diff(start, ticks_us())) > timeout:
                self.clearIrqStatus()
                self.standby()
                return ERR_TX_TIMEOUT

        elapsed = abs(ticks_diff(start, ticks_us()))

        self._dataRate = (len_*8.0)/(float(elapsed)/1000000.0)

        state = self.clearIrqStatus()
        ASSERT(state)

        state = self.standby()

        return state

    def receive(self, data, len_, timeout_en, timeout_ms):
        state = self.standby()
        ASSERT(state)

        timeout = 0

        modem = self.getPacketType()
        if modem == SX126X_PACKET_TYPE_LORA:
            symbolLength = float(1 << self._sf) / float(self._bwKhz)
            timeout = int(symbolLength * 100.0 * 1000.0)
        else:
            return ERR_UNKNOWN

        if timeout_ms == 0:
            pass
        else:
            timeout = timeout_ms * 1000

        if timeout_en:
            timeoutValue = int(float(timeout) / 15.625)
        else:
            timeoutValue = SX126X_RX_TIMEOUT_NONE

        state = self.startReceive(timeoutValue)
        ASSERT(state)

        start = ticks_us()
        while not self.irq.value():
            yield_()
            if timeout_en:
                if abs(ticks_diff(start, ticks_us())) > timeout:
                    self.fixImplicitTimeout()
                    self.clearIrqStatus()
                    self.standby()
                    return ERR_RX_TIMEOUT

        if self._headerType == SX126X_LORA_HEADER_IMPLICIT and self.getPacketType() == SX126X_PACKET_TYPE_LORA:
            state = self.fixImplicitTimeout()
            ASSERT(state)

        return self.readData(data, len_)

    def transmitDirect(self, frf=0):
        state = ERR_NONE
        if frf != 0:
            state = self.setRfFrequency(frf)
        ASSERT(state)

        data = [SX126X_CMD_NOP]
        return self.SPIwriteCommand([SX126X_CMD_SET_TX_CONTINUOUS_WAVE], 1, data, 1)

    def receiveDirect(self):
        return ERR_UNKNOWN

    def scanChannel(self):
        if self.getPacketType() != SX126X_PACKET_TYPE_LORA:
            return ERR_WRONG_MODEM

        state = self.standby()
        ASSERT(state)

        state = self.setDioIrqParams(SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_CAD_DONE, SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_CAD_DONE)
        ASSERT(state)

        state = self.clearIrqStatus()
        ASSERT(state)

        state = self.setCad()
        ASSERT(state)

        while not self.irq.value():
            yield_()

        cadResult = self.getIrqStatus()
        if cadResult & SX126X_IRQ_CAD_DETECTED:
            self.clearIrqStatus()
            return LORA_DETECTED
        elif cadResult & SX126X_IRQ_CAD_DONE:
            self.clearIrqStatus()
            return CHANNEL_FREE

        return ERR_UNKNOWN

    def sleep(self, retainConfig=True):
        sleepMode = [SX126X_SLEEP_START_WARM | SX126X_SLEEP_RTC_OFF]
        if not retainConfig:
            sleepMode = [SX126X_SLEEP_START_COLD | SX126X_SLEEP_RTC_OFF]
        state = self.SPIwriteCommand([SX126X_CMD_SET_SLEEP], 1, sleepMode, 1, False)

        sleep_us(500)

        return state

    def standby(self, mode=SX126X_STANDBY_RC):
        data = [mode]
        return self.SPIwriteCommand([SX126X_CMD_SET_STANDBY], 1, data, 1)

    def setDio1Action(self, func):
        try:
            self.irq.callback(trigger=Pin.IRQ_RISING, handler=func)     # Pycom variant uPy
        except:
            self.irq.irq(trigger=Pin.IRQ_RISING, handler=func)          # Generic variant uPy

    def clearDio1Action(self):
        self.irq = Pin(self._irq, mode=Pin.IN)

    def startTransmit(self, data, len_, addr=0):
        if len_ > SX126X_MAX_PACKET_LENGTH:
            return ERR_PACKET_TOO_LONG

        state = ERR_NONE
        modem = self.getPacketType()
        if modem == SX126X_PACKET_TYPE_LORA:
            if self._txIq:
                self._invertIQ = SX126X_LORA_IQ_INVERTED
            else:
                self._invertIQ = SX126X_LORA_IQ_STANDARD

            if self._headerType == SX126X_LORA_HEADER_IMPLICIT:
                if len_ != self._implicitLen:
                    return ERR_INVALID_PACKET_LENGTH

            state = self.setPacketParams(self._preambleLength, self._crcType, len_, self._headerType, self._invertIQ)
        else:
            return ERR_UNKNOWN
        ASSERT(state)

        state = self.setDioIrqParams(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT, SX126X_IRQ_TX_DONE)
        ASSERT(state)

        state = self.setBufferBaseAddress()
        ASSERT(state)

        state = self.writeBuffer(data, len_)
        ASSERT(state)

        state = self.clearIrqStatus()
        ASSERT(state)

        state = self.fixSensitivity()
        ASSERT(state)

        state = self.setTx(SX126X_TX_TIMEOUT_NONE)
        ASSERT(state)

        while self.gpio.value():
            yield_()

        return state

    def startReceive(self, timeout=SX126X_RX_TIMEOUT_INF):
        state = ERR_NONE
        modem = self.getPacketType()
        if modem == SX126X_PACKET_TYPE_LORA:
            if self._rxIq:
                self._invertIQ = SX126X_LORA_IQ_INVERTED
            else:
                self._invertIQ = SX126X_LORA_IQ_STANDARD

            state = self.setPacketParams(self._preambleLength, self._crcType, self._implicitLen, self._headerType, self._invertIQ)
        else:
            return ERR_UNKNOWN
        ASSERT(state)

        state = self.startReceiveCommon()
        ASSERT(state)

        state = self.setRx(timeout)

        return state

    def startReceiveDutyCycle(self, rxPeriod, sleepPeriod):
        transitionTime = int(self._tcxoDelay + 1000)
        sleepPeriod -= transitionTime

        rxPeriodRaw = int((rxPeriod * 8) / 125)
        sleepPeriodRaw = int((sleepPeriod * 8) / 125)

        if rxPeriodRaw & 0xFF000000 or rxPeriodRaw == 0:
            return ERR_INVALID_RX_PERIOD

        if sleepPeriodRaw & 0xFF000000 or sleepPeriodRaw == 0:
            return ERR_INVALID_SLEEP_PERIOD

        state = self.startReceiveCommon()
        ASSERT(state)

        data = [int((rxPeriodRaw >> 16) & 0xFF), int((rxPeriodRaw >> 8) & 0xFF), int(rxPeriodRaw & 0xFF),
                int((sleepPeriodRaw >> 16) & 0xFF),int((sleepPeriodRaw >> 8) & 0xFF),int(sleepPeriodRaw & 0xFF)]
        return self.SPIwriteCommand([SX126X_CMD_SET_RX_DUTY_CYCLE], 1, data, 6)

    def startReceiveDutyCycleAuto(self, senderPreambleLength=0, minSymbols=8):
        if senderPreambleLength == 0:
            senderPreambleLength = self._preambleLength

        sleepSymbols = int(senderPreambleLength - 2 * minSymbols)

        if (2 * minSymbols) > senderPreambleLength:
            return self.startReceive()

        symbolLength = int(((10*1000) << self._sf) / (10 * self._bwKhz))
        sleepPeriod = symbolLength * sleepSymbols

        wakePeriod = int(max((symbolLength * (senderPreambleLength + 1) - (sleepPeriod - 1000)) / 2, symbolLength * (minSymbols + 1)))

        if sleepPeriod < (self._tcxoDelay + 1016):
            return self.startReceive()

        return self.startReceiveDutyCycle(wakePeriod, sleepPeriod)

    def startReceiveCommon(self):
        state = self.setDioIrqParams(SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR | SX126X_IRQ_HEADER_ERR, SX126X_IRQ_RX_DONE)
        ASSERT(state)

        state = self.setBufferBaseAddress()
        ASSERT(state)

        state = self.clearIrqStatus()

        modem = self.getPacketType()
        if modem == SX126X_PACKET_TYPE_LORA:
            state = self.setPacketParams(self._preambleLength, self._crcType, self._implicitLen, self._headerType, self._invertIQ)
        else:
            return ERR_UNKNOWN

        return state

    def readData(self, data, len_):
        state = self.standby()
        ASSERT(state)

        irq = self.getIrqStatus()
        crcState = ERR_NONE
        if irq & SX126X_IRQ_CRC_ERR or irq & SX126X_IRQ_HEADER_ERR:
            crcState = ERR_CRC_MISMATCH

        length = len_
        if len_ == SX126X_MAX_PACKET_LENGTH:
            length = self.getPacketLength()

        state = self.readBuffer(data, length)
        ASSERT(state)

        state = self.clearIrqStatus()

        ASSERT(crcState)

        return state

    def setBandwidth(self, bw):
        if self.getPacketType() != SX126X_PACKET_TYPE_LORA:
            return ERR_WRONG_MODEM

        if not ((bw > 0) and (bw < 510)):
            return ERR_INVALID_BANDWIDTH

        bw_div2 = int(bw / 2 + 0.01)
        switch = {3: SX126X_LORA_BW_7_8,
                  5: SX126X_LORA_BW_10_4,
                  7: SX126X_LORA_BW_15_6,
                  10: SX126X_LORA_BW_20_8,
                  15: SX126X_LORA_BW_31_25,
                  20: SX126X_LORA_BW_41_7,
                  31: SX126X_LORA_BW_62_5,
                  62: SX126X_LORA_BW_125_0,
                  125: SX126X_LORA_BW_250_0,
                  250: SX126X_LORA_BW_500_0}
        try:
            self._bw = switch[bw_div2]
        except:
            return ERR_INVALID_BANDWIDTH

        self._bwKhz = bw
        return self.setModulationParams(self._sf, self._bw, self._cr, self._ldro)

    def setSpreadingFactor(self, sf):
        if self.getPacketType() != SX126X_PACKET_TYPE_LORA:
            return ERR_WRONG_MODEM

        if not ((sf >= 5) and (sf <= 12)):
            return ERR_INVALID_SPREADING_FACTOR

        self._sf = sf
        return self.setModulationParams(self._sf, self._bw, self._cr, self._ldro)

    def setCodingRate(self, cr):
        if self.getPacketType() != SX126X_PACKET_TYPE_LORA:
            return ERR_WRONG_MODEM

        if not ((cr >= 5) and (cr <= 8)):
            return ERR_INVALID_CODING_RATE

        self._cr = cr - 4
        return self.setModulationParams(self._sf, self._bw, self._cr, self._ldro)

    def setSyncWord(self, syncWord, *args):
        if self.getPacketType() == SX126X_PACKET_TYPE_LORA:
            if len(args) > 0:
                controlBits = args[0]
            else:
                controlBits = 0x44
            data = [int((syncWord & 0xF0) | ((controlBits & 0xF0) >> 4)), int(((syncWord & 0x0F) << 4) | (controlBits & 0x0F))]
            return self.writeRegister(SX126X_REG_LORA_SYNC_WORD_MSB, data, 2)
        else:
            return ERR_WRONG_MODEM

    def setCurrentLimit(self, currentLimit):
        if not ((currentLimit >= 0) and (currentLimit <= 140)):
            return ERR_INVALID_CURRENT_LIMIT

        rawLimit = [int(currentLimit / 2.5)]

        return self.writeRegister(SX126X_REG_OCP_CONFIGURATION, rawLimit, 1)

    def getCurrentLimit(self):
        ocp = bytearray(1)
        ocp_mv = memoryview(ocp)
        self.readRegister(SX126X_REG_OCP_CONFIGURATION, ocp_mv, 1)

        return float(ocp[0]) * 2.5

    def setPreambleLength(self, preambleLength):
        modem = self.getPacketType()
        if modem == SX126X_PACKET_TYPE_LORA:
            self._preambleLength = preambleLength
            return self.setPacketParams(self._preambleLength, self._crcType, self._implicitLen, self._headerType, self._invertIQ)

        return ERR_UNKNOWN

    def setCRC(self, len_, initial=0x1D0F, polynomial=0x1021, inverted=True):
        modem = self.getPacketType()

        if modem == SX126X_PACKET_TYPE_LORA:

            if len_:
                self._crcType = SX126X_LORA_CRC_ON
            else:
                self._crcType = SX126X_LORA_CRC_OFF

            return self.setPacketParams(self._preambleLength, self._crcType, self._implicitLen, self._headerType, self._invertIQ)

        return ERR_UNKNOWN

    def getDataRate(self):
        return self._dataRate

    def getRSSI(self):
        packetStatus = self.getPacketStatus()
        rssiPkt = int(packetStatus & 0xFF)
        return -1.0 * rssiPkt/2.0

    def getSNR(self):
        if self.getPacketType() != SX126X_PACKET_TYPE_LORA:
            return ERR_WRONG_MODEM

        packetStatus = self.getPacketStatus()
        snrPkt = int((packetStatus >> 8) & 0xFF)
        if snrPkt < 128:
            return snrPkt/4.0
        else:
            return (snrPkt - 256)/4.0

    def getPacketLength(self, update=True):
        rxBufStatus = bytearray(2)
        rxBufStatus_mv = memoryview(rxBufStatus)
        self.SPIreadCommand([SX126X_CMD_GET_RX_BUFFER_STATUS], 1, rxBufStatus_mv, 2)
        return rxBufStatus[0]

    def getTimeOnAir(self, len_):
        if self.getPacketType() == SX126X_PACKET_TYPE_LORA:
            symbolLength_us = int(((1000 * 10) << self._sf) / (self._bwKhz * 10))
            sfCoeff1_x4 = 17
            sfCoeff2 = 8
            if self._sf == 5 or self._sf == 6:
                sfCoeff1_x4 = 25
                sfCoeff2 = 0
            sfDivisor = 4*self._sf
            if symbolLength_us >= 16000:
                sfDivisor = 4*(self._sf - 2)
            bitsPerCrc = 16
            N_symbol_header = 20 if self._headerType == SX126X_LORA_HEADER_EXPLICIT else 0

            bitCount = int(8 * len_ + self._crcType * bitsPerCrc - 4 * self._sf  + sfCoeff2 + N_symbol_header)
            if bitCount < 0:
                bitCount = 0

            nPreCodedSymbols = int((bitCount + (sfDivisor - 1)) / sfDivisor)

            nSymbol_x4 = int((self._preambleLength + 8) * 4 + sfCoeff1_x4 + nPreCodedSymbols * (self._cr + 4) * 4)

            return int((symbolLength_us * nSymbol_x4) / 4)

    def implicitHeader(self, len_):
        return self.setHeaderType(SX126X_LORA_HEADER_IMPLICIT, len_)

    def explicitHeader(self):
        return self.setHeaderType(SX126X_LORA_HEADER_EXPLICIT)

    def setRegulatorLDO(self):
        return self.setRegulatorMode(SX126X_REGULATOR_LDO)

    def setRegulatorDCDC(self):
        return self.setRegulatorMode(SX126X_REGULATOR_DC_DC)

    def forceLDRO(self, enable):
        if self.getPacketType() != SX126X_PACKET_TYPE_LORA:
            return ERR_WRONG_MODEM

        self._ldroAuto = False
        self._ldro = enable
        return self.setModulationParams(self._sf, self._bw, self._cr, self._ldro)

    def autoLDRO(self):
        if self.getPacketType() != SX126X_PACKET_TYPE_LORA:
            return ERR_WRONG_MODEM

        self._ldroAuto = True
        return self.setModulationParams(self._sf, self._bw, self._cr, self._ldro)

    def setTCXO(self, voltage, delay=5000):
        self.standby()

        if self.getDeviceErrors() & SX126X_XOSC_START_ERR:
            self.clearDeviceErrors()

        if abs(voltage - 0.0) <= 0.001:
            return self.reset()

        data = [0,0,0,0]
        if abs(voltage - 1.6) <= 0.001:
            data[0] = SX126X_DIO3_OUTPUT_1_6
        elif abs(voltage - 1.7) <= 0.001:
            data[0] = SX126X_DIO3_OUTPUT_1_7
        elif abs(voltage - 1.8) <= 0.001:
            data[0] = SX126X_DIO3_OUTPUT_1_8
        elif abs(voltage - 2.2) <= 0.001:
            data[0] = SX126X_DIO3_OUTPUT_2_2
        elif abs(voltage - 2.4) <= 0.001:
            data[0] = SX126X_DIO3_OUTPUT_2_4
        elif abs(voltage - 2.7) <= 0.001:
            data[0] = SX126X_DIO3_OUTPUT_2_7
        elif abs(voltage - 3.0) <= 0.001:
            data[0] = SX126X_DIO3_OUTPUT_3_0
        elif abs(voltage - 3.3) <= 0.001:
            data[0] = SX126X_DIO3_OUTPUT_3_3
        else:
            return ERR_INVALID_TCXO_VOLTAGE

        delayValue = int(float(delay) / 15.625)
        data[1] = int((delayValue >> 16) & 0xFF)
        data[2] = int((delayValue >> 8) & 0xFF)
        data[3] = int(delayValue & 0xFF)

        self._tcxoDelay = delay

        return self.SPIwriteCommand([SX126X_CMD_SET_DIO3_AS_TCXO_CTRL], 1, data, 4)

    def setDio2AsRfSwitch(self, enable=True):
        data = [0]
        if enable:
            data = [SX126X_DIO2_AS_RF_SWITCH]
        else:
            data = [SX126X_DIO2_AS_IRQ]
        return self.SPIwriteCommand([SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL], 1, data, 1)

    def setTx(self, timeout=0):
        data = [int((timeout >> 16) & 0xFF), int((timeout >> 8) & 0xFF), int(timeout & 0xFF)]
        return self.SPIwriteCommand([SX126X_CMD_SET_TX], 1, data, 3)

    def setRx(self, timeout):
        data = [int((timeout >> 16) & 0xFF), int((timeout >> 8) & 0xFF), int(timeout & 0xFF)]
        return self.SPIwriteCommand([SX126X_CMD_SET_RX], 1, data, 3)

    def setCad(self):
        return self.SPIwriteCommand([SX126X_CMD_SET_CAD], 1, [], 0)

    def setPaConfig(self, paDutyCycle, deviceSel, hpMax=SX126X_PA_CONFIG_HP_MAX, paLut=SX126X_PA_CONFIG_PA_LUT):
        data = [paDutyCycle, hpMax, deviceSel, paLut]
        return self.SPIwriteCommand([SX126X_CMD_SET_PA_CONFIG], 1, data, 4)

    def writeRegister(self, addr, data, numBytes):
        cmd = [SX126X_CMD_WRITE_REGISTER, int((addr >> 8) & 0xFF), int(addr & 0xFF)]
        state = self.SPIwriteCommand(cmd, 3, data, numBytes)
        return state

    def readRegister(self, addr, data, numBytes):
        cmd = [SX126X_CMD_READ_REGISTER, int((addr >> 8) & 0xFF), int(addr & 0xFF)]
        return self.SPItransfer(cmd, 3, False, [], data, numBytes, True)

    def writeBuffer(self, data, numBytes, offset=0x00):
        cmd = [SX126X_CMD_WRITE_BUFFER, offset]
        state = self.SPIwriteCommand(cmd, 2, data, numBytes)

        return state

    def readBuffer(self, data, numBytes):
        cmd = [SX126X_CMD_READ_BUFFER, SX126X_CMD_NOP]
        state = self.SPIreadCommand(cmd, 2, data, numBytes)

        return state

    def setDioIrqParams(self, irqMask, dio1Mask, dio2Mask=SX126X_IRQ_NONE, dio3Mask=SX126X_IRQ_NONE):
        data = [int((irqMask >> 8) & 0xFF), int(irqMask & 0xFF),
                int((dio1Mask >> 8) & 0xFF), int(dio1Mask & 0xFF),
                int((dio2Mask >> 8) & 0xFF), int(dio2Mask & 0xFF),
                int((dio3Mask >> 8) & 0xFF), int(dio3Mask & 0xFF)]
        return self.SPIwriteCommand([SX126X_CMD_SET_DIO_IRQ_PARAMS], 1, data, 8)

    def getIrqStatus(self):
        data = bytearray(2)
        data_mv = memoryview(data)
        self.SPIreadCommand([SX126X_CMD_GET_IRQ_STATUS], 1, data_mv, 2)
        return int((data[0] << 8) | data[1])

    def clearIrqStatus(self, clearIrqParams=SX126X_IRQ_ALL):
        data = [int((clearIrqParams >> 8) & 0xFF), int(clearIrqParams & 0xFF)]
        return self.SPIwriteCommand([SX126X_CMD_CLEAR_IRQ_STATUS], 1, data, 2)

    def setRfFrequency(self, frf):
        data = [int((frf >> 24) & 0xFF),
                int((frf >> 16) & 0xFF),
                int((frf >> 8) & 0xFF),
                int(frf & 0xFF)]
        return self.SPIwriteCommand([SX126X_CMD_SET_RF_FREQUENCY], 1, data, 4)

    def calibrateImage(self, data):
        return self.SPIwriteCommand([SX126X_CMD_CALIBRATE_IMAGE], 1, data, 2)

    def getPacketType(self):
        data = bytearray([0xFF])
        data_mv = memoryview(data)
        self.SPIreadCommand([SX126X_CMD_GET_PACKET_TYPE], 1, data_mv, 1)
        return data[0]

    def setTxParams(self, power, rampTime=SX126X_PA_RAMP_200U):
        data = [power, rampTime]
        return self.SPIwriteCommand([SX126X_CMD_SET_TX_PARAMS], 1, data, 2)

    def setHeaderType(self, headerType, len_=0xFF):
        if self.getPacketType() != SX126X_PACKET_TYPE_LORA:
            return ERR_WRONG_MODEM

        state = self.setPacketParams(self._preambleLength, self._crcType, len_, headerType, self._invertIQ)
        ASSERT(state)

        self._headerType = headerType
        self._implicitLen = len_

        return state

    def setModulationParams(self, sf, bw, cr, ldro):
        if self._ldroAuto:
            symbolLength = float((1 << self._sf)) / float(self._bwKhz)
            if symbolLength >= 16.0:
                self._ldro = SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_ON
            else:
                self._ldro = SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF
        else:
            self._ldro = ldro

        data = [sf, bw, cr, self._ldro]
        return self.SPIwriteCommand([SX126X_CMD_SET_MODULATION_PARAMS], 1, data, 4)

    def setPacketParams(self, preambleLength, crcType, payloadLength, headerType, invertIQ=SX126X_LORA_IQ_STANDARD):
        state = self.fixInvertedIQ(invertIQ)
        ASSERT(state)
        data = [int((preambleLength >> 8) & 0xFF), int(preambleLength & 0xFF),
                headerType, payloadLength, crcType, invertIQ]
        return self.SPIwriteCommand([SX126X_CMD_SET_PACKET_PARAMS], 1, data, 6)

    def setBufferBaseAddress(self, txBaseAddress=0x00, rxBaseAddress=0x00):
        data = [txBaseAddress, rxBaseAddress]
        return self.SPIwriteCommand([SX126X_CMD_SET_BUFFER_BASE_ADDRESS], 1, data, 2)

    def setRegulatorMode(self, mode):
        data = [mode]
        return self.SPIwriteCommand([SX126X_CMD_SET_REGULATOR_MODE], 1, data, 1)

    def getStatus(self):
        data = bytearray(1)
        data_mv = memoryview(data)
        self.SPIreadCommand([SX126X_CMD_GET_STATUS], 1, data_mv, 1)
        return data[0]

    def getPacketStatus(self):
        data = bytearray(3)
        data_mv = memoryview(data)
        self.SPIreadCommand([SX126X_CMD_GET_PACKET_STATUS], 1, data_mv, 3)
        return (data[0] << 16) | (data[1] << 8) | data[2]

    def getDeviceErrors(self):
        data = bytearray(2)
        data_mv = memoryview(data)
        self.SPIreadCommand([SX126X_CMD_GET_DEVICE_ERRORS], 1, data_mv, 2)
        opError = ((data[0] & 0xFF) << 8) & data[1]
        return opError

    def clearDeviceErrors(self):
        data = [SX126X_CMD_NOP, SX126X_CMD_NOP]
        return self.SPIwriteCommand([SX126X_CMD_CLEAR_DEVICE_ERRORS], 1, data, 2)

    def setFrequencyRaw(self, freq):
        frf = int((freq * (1 << SX126X_DIV_EXPONENT)) / SX126X_CRYSTAL_FREQ)
        return self.setRfFrequency(frf)

    def fixSensitivity(self):
        sensitivityConfig = bytearray(1)
        sensitivityConfig_mv = memoryview(sensitivityConfig)
        state = self.readRegister(SX126X_REG_SENSITIVITY_CONFIG, sensitivityConfig_mv, 1)
        ASSERT(state)

        if self.getPacketType() == SX126X_PACKET_TYPE_LORA and abs(self._bwKhz - 500.0) <= 0.001:
            sensitivityConfig_mv[0] &= 0xFB
        else:
            sensitivityConfig_mv[0] |= 0x04
        return self.writeRegister(SX126X_REG_SENSITIVITY_CONFIG, sensitivityConfig, 1)

    def fixPaClamping(self):
        clampConfig = bytearray(1)
        clampConfig_mv = memoryview(clampConfig)
        state = self.readRegister(SX126X_REG_TX_CLAMP_CONFIG, clampConfig_mv, 1)
        ASSERT(state)

        clampConfig_mv[0] |= 0x1E
        return self.writeRegister(SX126X_REG_TX_CLAMP_CONFIG, clampConfig, 1)

    def fixImplicitTimeout(self):
        if not (self._headerType == SX126X_LORA_HEADER_IMPLICIT and self.getPacketType() == SX126X_PACKET_TYPE_LORA):
            return ERR_WRONG_MODEM

        rtcStop = [0x00]
        state = self.writeRegister(SX126X_REG_RTC_STOP, rtcStop, 1)
        ASSERT(state)

        rtcEvent = bytearray(1)
        rtcEvent_mv = memoryview(rtcEvent)
        state = self.readRegister(SX126X_REG_RTC_EVENT, rtcEvent_mv, 1)
        ASSERT(state)

        rtcEvent_mv[0] |= 0x02
        return self.writeRegister(SX126X_REG_RTC_EVENT, rtcEvent, 1)

    def fixInvertedIQ(self, iqConfig):
        iqConfigCurrent = bytearray(1)
        iqConfigCurrent_mv = memoryview(iqConfigCurrent)
        state = self.readRegister(SX126X_REG_IQ_CONFIG, iqConfigCurrent_mv, 1)
        ASSERT(state)

        if iqConfig == SX126X_LORA_IQ_STANDARD:
            iqConfigCurrent_mv[0] &= 0xFB
        else:
            iqConfigCurrent_mv[0] |= 0x04

        return self.writeRegister(SX126X_REG_IQ_CONFIG, iqConfigCurrent, 1)

    def config(self, modem):
        state = self.setBufferBaseAddress()
        ASSERT(state)

        data = [0,0,0,0,0,0,0]
        data[0] = modem
        state = self.SPIwriteCommand([SX126X_CMD_SET_PACKET_TYPE], 1, data, 1)
        ASSERT(state)

        data[0] = SX126X_RX_TX_FALLBACK_MODE_STDBY_RC
        state = self.SPIwriteCommand([SX126X_CMD_SET_RX_TX_FALLBACK_MODE], 1, data, 1)
        ASSERT(state)

        data[0] = SX126X_CAD_ON_8_SYMB
        data[1] = self._sf + 13
        data[2] = 10
        data[3] = SX126X_CAD_GOTO_STDBY
        data[4] = 0x00
        data[5] = 0x00
        data[6] = 0x00
        state = self.SPIwriteCommand([SX126X_CMD_SET_CAD_PARAMS], 1, data, 7)
        ASSERT(state)

        state = self.clearIrqStatus()
        state |= self.setDioIrqParams(SX126X_IRQ_NONE, SX126X_IRQ_NONE)
        ASSERT(state)

        data[0] = SX126X_CALIBRATE_ALL
        state = self.SPIwriteCommand([SX126X_CMD_CALIBRATE], 1, data, 1)
        ASSERT(state)

        sleep_ms(5)

        while self.gpio.value():
            yield_()

        return ERR_NONE

    def SPIwriteCommand(self, cmd, cmdLen, data, numBytes, waitForBusy=True):
        return self.SPItransfer(cmd, cmdLen, True, data, [], numBytes, waitForBusy)

    def SPIreadCommand(self, cmd, cmdLen, data, numBytes, waitForBusy=True):
        return self.SPItransfer(cmd, cmdLen, False, [], data, numBytes, waitForBusy)

    def SPItransfer(self, cmd, cmdLen, write, dataOut, dataIn, numBytes, waitForBusy, timeout=5000):
        self.cs.value(0)

        start = ticks_ms()
        while self.gpio.value():
            yield_()
            if abs(ticks_diff(start, ticks_ms())) >= timeout:
                self.cs.value(1)
                return ERR_SPI_CMD_TIMEOUT

        for i in range(cmdLen):
            self.spi.write(bytes([cmd[i]]))

        status = 0

        if write:
            for i in range(numBytes):
                try:
                    in_ = self.spi.read(1, dataOut[i])
                except:
                    in_ = self.spi.read(1, write=dataOut[i])

                if (in_[0] & 0b00001110) == SX126X_STATUS_CMD_TIMEOUT or\
                   (in_[0] & 0b00001110) == SX126X_STATUS_CMD_INVALID or\
                   (in_[0] & 0b00001110) == SX126X_STATUS_CMD_FAILED:
                    status = in_[0] & 0b00001110
                    break
                elif (in_[0] == 0x00) or (in_[0] == 0xFF):
                    status = SX126X_STATUS_SPI_FAILED
                    break
        else:
            try:
                in_ = self.spi.read(1, SX126X_CMD_NOP)
            except:
                in_ = self.spi.read(1, write=SX126X_CMD_NOP)

            if (in_[0] & 0b00001110) == SX126X_STATUS_CMD_TIMEOUT or\
               (in_[0] & 0b00001110) == SX126X_STATUS_CMD_INVALID or\
               (in_[0] & 0b00001110) == SX126X_STATUS_CMD_FAILED:
                status = in_[0] & 0b00001110
            elif (in_[0] == 0x00) or (in_[0] == 0xFF):
                status = SX126X_STATUS_SPI_FAILED
            else:
                for i in range(numBytes):
                    try:
                        dataIn[i] = self.spi.read(1, SX126X_CMD_NOP)[0]
                    except:
                        dataIn[i] = self.spi.read(1, write=SX126X_CMD_NOP)[0]

        self.cs.value(1)

        if waitForBusy:
            sleep_us(1)
            start = ticks_ms()
            while self.gpio.value():
                yield_()
                if abs(ticks_diff(start, ticks_ms())) >= timeout:
                    status =  SX126X_STATUS_CMD_TIMEOUT
                    break

        switch = {SX126X_STATUS_CMD_TIMEOUT: ERR_SPI_CMD_TIMEOUT,
                  SX126X_STATUS_CMD_INVALID: ERR_SPI_CMD_INVALID,
                  SX126X_STATUS_CMD_FAILED: ERR_SPI_CMD_FAILED,
                  SX126X_STATUS_SPI_FAILED: ERR_CHIP_NOT_FOUND}
        try:
            return switch[status]
        except:
            return ERR_NONE
