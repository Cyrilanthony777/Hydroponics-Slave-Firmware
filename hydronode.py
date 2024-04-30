from machine import I2C,Pin,ADC,UART,Timer
import onewire,ds18x20
import utime
import struct
import math
from micropython import const



class BH1750:
    MEASUREMENT_MODE_CONTINUOUSLY = const(1)
    MEASUREMENT_MODE_ONE_TIME = const(2)
    
    RESOLUTION_HIGH = const(0)
    RESOLUTION_HIGH_2 = const(1)
    RESOLUTION_LOW = const(2)
    
    MEASUREMENT_TIME_DEFAULT = const(69)
    MEASUREMENT_TIME_MIN = const(31)
    MEASUREMENT_TIME_MAX = const(254)

    def __init__(self, address, i2c):
        self._address = address
        self._i2c = i2c
        self._measurement_mode = BH1750.MEASUREMENT_MODE_ONE_TIME
        self._resolution = BH1750.RESOLUTION_HIGH
        self._measurement_time = BH1750.MEASUREMENT_TIME_DEFAULT
        
        self._write_measurement_time()
        self._write_measurement_mode()
        
    def configure(self, measurement_mode: int, resolution: int, measurement_time: int):
        if measurement_time not in range(BH1750.MEASUREMENT_TIME_MIN, BH1750.MEASUREMENT_TIME_MAX + 1):
            raise ValueError("measurement_time must be between {0} and {1}"
                             .format(BH1750.MEASUREMENT_TIME_MIN, BH1750.MEASUREMENT_TIME_MAX))
        
        self._measurement_mode = measurement_mode
        self._resolution = resolution
        self._measurement_time = measurement_time
        
        self._write_measurement_time()
        self._write_measurement_mode()
    
    def _write_measurement_time(self):
        try:
            buffer = bytearray(1)
            
            high_bit = 1 << 6 | self._measurement_time >> 5
            low_bit = 3 << 5 | (self._measurement_time << 3) >> 3
                    
            buffer[0] = high_bit
            self._i2c.writeto(self._address, buffer)
            
            buffer[0] = low_bit
            self._i2c.writeto(self._address, buffer)
        except:
            pass
        
    def _write_measurement_mode(self):
        try:
            buffer = bytearray(1)
                    
            buffer[0] = self._measurement_mode << 4 | self._resolution
            self._i2c.writeto(self._address, buffer)
            utime.sleep_ms(24 if self._measurement_time == BH1750.RESOLUTION_LOW else 180)
        except:
            pass
        
    def reset(self):
        self._i2c.writeto(self._address, bytearray(b'\x07'))
    
    def power_on(self):
        self._i2c.writeto(self._address, bytearray(b'\x01'))

    def power_off(self):
        self._i2c.writeto(self._address, bytearray(b'\x00'))

    @property
    def measurement(self) -> float:
        try:
            if self._measurement_mode == BH1750.MEASUREMENT_MODE_ONE_TIME:
                self._write_measurement_mode()
                
            buffer = bytearray(2)
            self._i2c.readfrom_into(self._address, buffer)
            lux = (buffer[0] << 8 | buffer[1]) / (1.2 * (BH1750.MEASUREMENT_TIME_DEFAULT / self._measurement_time))
            
            if self._resolution == BH1750.RESOLUTION_HIGH_2:
                return lux / 2
            else:
                return lux
        except:
            return 0.0


class NodeType():
    CLIMATE = 0x01
    NUTRIENT = 0x02
    AUX = 0x03


class Node():
    def __init__(self,address,nodeType : NodeType):
        self.address = address
        self.nodeType = nodeType
    
    def initialize(self):
        pass

    def process(self):
        pass

    def printData(self):
        pass

    def getData(self):
        pass


    def calcCRC(data,length):
        ret = 0x00
        for x in range(0,length):
            ret ^= data[x]
        return ret
    
    
class SHT:
    def __init__(self,i2c : I2C,addr):
        self.i2c = i2c
        self.temperature = 0
        self.humidity = 0
        self.addr = addr
    
    def update(self):
        self.humidity = 0
        self.temperature = 0
        try:
            status = self.i2c.writeto(self.addr,b'\x24\x00')
            utime.sleep(1)
            databytes = self.i2c.readfrom(self.addr, 6)
            dataset = [databytes[0],databytes[1]]
            dataset = [databytes[3],databytes[4]]
            temperature_raw = databytes[0] << 8 | databytes[1]
            self.temperature =  round((175.0 * float(temperature_raw) / 65535.0) - 45,1)
            humidity_raw = databytes[3] << 8  | databytes[4]
            self.humidity = int(100.0 * float(humidity_raw) / 65535.0)
        except Exception as e:
            print(e)
        
    def getTemperature(self):
        return self.temperature
    
    def getHumidity(self):
        return self.humidity
    

class ClimateNode(Node):
    def __init__(self,address,nodeType : NodeType):
        super().__init__(address,nodeType)
        self.ph = 0.0
        self.ec = 0.0
        self.water_temp = 0.0
        self.lux = 0.0
        self.humidity = 0
        self.air_temp = 0.0
        self.flow = 0.0
        self.level = 0
        self.SHT = None
        self.luxSensor = None
        self.i2c = None
    
    def initialize(self):
        self.i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=100000)
        self.SHT = SHT(self.i2c,0x44)
        self.luxSensor = BH1750(0x23,self.i2c)
            
    def process(self):
        self.SHT.update()
        self.air_temp = self.SHT.getTemperature()
        self.humidity = self.SHT.getHumidity()
        self.lux = self.luxSensor.measurement
            
    def printData(self):
        print("temperature : {} - humidity : {} - lux : {}".format(self.air_temp,self.humidity,self.lux))
        
    def getData(self):
        retData = [0xAA,0xCC,self.address,self.nodeType,0xBA,0x07]
        retData = retData + list(struct.pack('f',self.air_temp))
        retData.append(self.humidity)
        retData = retData + list(struct.pack('<h',self.lux))
        crc = Node.calcCRC(retData,len(retData))
        retData.append(crc)
        return bytearray(retData)
    
class NutrientNode(Node):
    def __init__(self,address,nodeType : NodeType):
        super().__init__(address,nodeType)
        self.ph = 0.0
        self.ec = 0.0
        self.water_temp = 0.0
        self.lux = 0.0
        self.humidity = 0
        self.air_temp = 0.0
        self.flow = 0.0
        self.level = 0
        self.dsSensor = None
        self.sensors = []
        self.analog = None
        self.tempFlow = 0
        self.timer = None
    
    def initialize(self):
        self.dsSensor = ds18x20.DS18X20(onewire.OneWire(Pin(22)))
        self.sensors = self.dsSensor.scan()
        self.analog = ADC(Pin(26))
        Pin(21, Pin.IN).irq(trigger=Pin.IRQ_RISING, handler=self.flowInterrupt)
        self.timer = Timer(mode=Timer.PERIODIC, period=1000, callback=self.timerInterrupt)
            
    def process(self):
        self.dsSensor.convert_temp()
        utime.sleep_ms(750)
        if len(self.sensors) > 0:
            self.water_temp = round(self.dsSensor.read_temp(self.sensors[0]),1)
        else:
            self.water_temp = 0.0
        temp = 0
        for x in range(0,10):
            temp += self.analog.read_u16()
            utime.sleep_ms(2)
        self.ec = round(temp/10)
            
    def flowInterrupt(self):
        self.tempFlow += 1
            
    def timerInterrupt(self,ts):
        self.flow = int(self.tempFlow)
        self.tempFlow = 0
            
    def printData(self):
        print("EC : {} - Temperature : {} - Flow : {}".format(self.ec,self.water_temp,self.flow))
        
    def getData(self):
        retData = [0xAA,0xCC,self.address,self.nodeType,0xBA,0x06]
        retData = retData + list(struct.pack('f',self.water_temp))
        retData = retData + list(struct.pack('H',self.ec))
        crc = Node.calcCRC(retData,len(retData))
        retData.append(crc)
        return bytearray(retData)
    

class AuxNode(Node):
    def __init__(self,address,nodeType : NodeType):
        super().__init__(address,nodeType)
        self.ph = 0.0
        self.ec = 0.0
        self.water_temp = 0.0
        self.lux = 0.0
        self.humidity = 0
        self.air_temp = 0.0
        self.flow = 0.0
        self.level = 0
        self.analog = None
    
    def initialize(self):
        self.analog = ADC(Pin(26))
            
    def process(self):
        temp = 0
        for x in range(0,10):
            temp += self.analog.read_u16()
            utime.sleep_ms(2)
        self.ph = round(temp/10)
            
    def printData(self):
        print("pH : {} - Level : {}".format(self.ph,self.level))
        
    def getData(self):
        retData = [0xAA,0xCC,self.address,self.nodeType,0xBA,0x03]
        retData = retData + list(struct.pack('H',self.ph))
        retData.append(self.level)
        crc = Node.calcCRC(retData,len(retData))
        retData.append(crc)
        return bytearray(retData)
    

class NodeFactory:
    @staticmethod
    def createNode(address,nodeType : NodeType):
        if nodeType == NodeType.CLIMATE:
            return ClimateNode(address,nodeType)
        elif nodeType == NodeType.NUTRIENT:
            return NutrientNode(address,nodeType)
        elif nodeType == NodeType.AUX:
            return AuxNode(address,nodeType)
        else:
            return None





class Communication:
    def __init__(self,address,node):
        self.uart = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17), bits=8, parity=None, stop=1,timeout=100)
        self.led = Pin(25, Pin.OUT)
        self.address = address
        self.node = node
    
    def checkCommand(self):
        data = self.uart.read(7)
        if data != None and len(data) == 7:
            if data[0] == 0xAA and data[1] == 0xCC and data[2] == self.address and data[3] == self.node and data[4] == 0xAB:
                if data[6] == Node.calcCRC(data,6):
                    self.led.toggle()
                    return True
        else:
            pass
        return False
    
    
    def sendData(self,data):
        self.uart.write(data)
                    
        
        
    
        
        

        
    