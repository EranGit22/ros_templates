#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import sleep
from smbus2 import SMBus
import math
import time

# Models
MODEL_02BA = 0
MODEL_30BA = 1

# Oversampling options (convertion time)
OSR_256 = 0
OSR_512 = 1
OSR_1024 = 2
OSR_2048 = 3
OSR_4096 = 4
OSR_8192 = 5

# kg/m^3 convenience
DENSITY_FRESHWATER = 997
DENSITY_SALTWATER = 1029

# Conversion factors (from native unit, mbar)
UNITS_Pa = 100.0
UNITS_hPa = 1.0
UNITS_kPa = 0.1
UNITS_mbar = 1.0
UNITS_bar = 0.001
UNITS_atm = 0.000986923
UNITS_Torr = 0.750062
UNITS_psi = 0.014503773773022

# Valid units
UNITS_Centigrade = 1
UNITS_Farenheit = 2
UNITS_Kelvin = 3


class MS5837_30BA(Node):
    """ MS5837 BA30 measurements node"""

    # Registers
    _MS5837_ADDR = 0x76
    _MS5837_RESET = 0x1E
    _MS5837_ADC_READ = 0x00
    _MS5837_PROM_READ = 0xA0
    _MS5837_CONVERT_D1_256 = 0x40
    _MS5837_CONVERT_D2_256 = 0x50

    def __init__(self, model=MODEL_30BA):
        """BA30 node initialization, service and open i2c bus."""
        self._model = model
        self.port="/dev/i2c-pressuresensor"

        #self.server_=self.create_service(Send30BAData, 'Measure_30BA',self.callback_Measure_30BA)
        self.get_logger().info('Measure_30BA server has been started')
        try:
            self._bus = SMBus(self.port)
            sleep(1) #necessary!
        except:
            print("Pressure Sensor's bus (probably 5) is not available.\nWrite 'ls /dev/ in terminal to see the available busses.")
            self._bus = None
        #initializing variables
        self.P0=1000 #default number
        self._C = []
        self._fluidDensity = DENSITY_FRESHWATER
        self._pressure = 0
        self._temperature = 0
        self._D1 = 0
        self._D2 = 0
        self.initialize()
        self.read()
        self.calc_P0()

    def initialize(self):

        #double check for bus
        print(self._bus)
        if self._bus is None:
            print("STOPPED! There is a problem with the bus!")
            return False

        # initializing the sensor

        self._bus.write_byte(self._MS5837_ADDR, self._MS5837_RESET)

        # Wait for reset to complete
        sleep(0.01)

        #self._C = []

        # Read calibration values and CRC (cyclic rebundancy check)
        for i in range(7):
            c = self._bus.read_word_data(
                self._MS5837_ADDR, self._MS5837_PROM_READ + 2*i) #read 2bytes each iteration (we have 16 bytes of data, 8 registers)
            # SMBus is little-endian for word transfers, we need to swap MSB and LSB
            c = ((c & 0xFF) << 8) | (c >> 8) #shit left/right 8 bits
            #c = (c << 8) | (c >> 8) should work either
            self._C.append(c)
        #CRC Check
        crc = (self._C[0] & 0xF000) >> 12
        if crc != self._crc4(self._C):
            print("PROM read error, CRC failed!")
            return False
        return True

    def read(self, oversampling=OSR_8192): #oversampling=5 by default
        if self._bus is None:
            print("No bus!")
            return False

        if oversampling < OSR_256 or oversampling > OSR_8192:
            print("Invalid oversampling option!")
            return False

        # Request D1 conversion (pressure)
        self._bus.write_byte(
            self._MS5837_ADDR, self._MS5837_CONVERT_D1_256 + 2*oversampling)

        # Maximum conversion time increases linearly with oversampling
        # max time (seconds) ~= 2.2e-6(x) where x = OSR = (2^8, 2^9, ..., 2^13)
        # We use 2.5e-6 for some overhead
        sleep(2.5e-6 * 2**(8+oversampling))

        d = self._bus.read_i2c_block_data(
            self._MS5837_ADDR, self._MS5837_ADC_READ, 3)
        self._D1 = d[0] << 16 | d[1] << 8 | d[2]

        # Request D2 conversion (temperature)
        self._bus.write_byte(
            self._MS5837_ADDR, self._MS5837_CONVERT_D2_256 + 2*oversampling)

        # As above
        sleep(2.5e-6 * 2**(8+oversampling))

        d = self._bus.read_i2c_block_data(
            self._MS5837_ADDR, self._MS5837_ADC_READ, 3)# (addr,reg,data)
        self._D2 = d[0] << 16 | d[1] << 8 | d[2] #little endian

        # Calculate compensated pressure and temperature
        # using raw ADC values and internal calibration
        self._calculate()

        return True
    
    def calc_P0(self):
        print("Computing current air's pressure ; P0 (30 seconds)...\nNOTICE THAT THE PRESSURE SENSOR IS NOT IN THE WATER!")
        timeout=time.time()+ 30
        avg_p0=0
        cnt=0
        while True:
            if time.time()>timeout:
                break
            cnt+=1
            avg_p0+= self.pressure()
        self.P0=avg_p0/cnt    
        print("Done.  Air's pressure: "+str(self.P0)+' [mbar] , ' +str(self.P0*0.000986923)+ ' [atm]' )

    def setFluidDensity(self, denisty):
        self._fluidDensity = denisty

    # Pressure in requested units
    # mbar * conversion
    def pressure(self, conversion=UNITS_mbar):
        return self._pressure * conversion

    # Temperature in requested units
    # default degrees C
    def temperature(self, conversion=UNITS_Centigrade):
        degC = self._temperature 
        if conversion == UNITS_Farenheit:
            return (9.0/5.0)*degC + 32
        elif conversion == UNITS_Kelvin:
            return degC + 273

        return degC
    
    def get_FW_density(self):# fresh water density
        T=self._temperature
        self.density=999.83311+0.0752*T-0.0089*pow(T,2)+7.36413*pow(10,-5)*pow(T,3)+4.74639*pow(10,-7)*pow(T,4)+1.34888*pow(10,-9)*pow(T,5)
        return self.density
    
    def get_SW_density(self):# salt water density
        T=self._temperature
        self.density=999.842594+6.7939532*pow(10,-2)*T-9.9059290*pow(10,-3)*pow(T,2)+1.001685*pow(10,-4)*pow(T,3)-1.120083*pow(10,-6)*pow(T,4)+6.536332*pow(10,-9)*pow(T,5)
        return self.density
    # Depth relative to MSL pressure in given fluid density
    def depth(self):
        return ((self.pressure()-self.P0)*100)/(self.density*9.807)

    # Altitude relative to MSL pressure
    def altitude(self):
        #p0=1013.25 #[mbar], sea level's pressure
        #g=9.807 #[m/s^2], gravity
        #M=0.2896 #[kg/mol], molar mass of earth's air
        #T=self.temperature(UNITS_Kelvin) #[K], temperature in KELVIN!
        #R=8.3143 #[N*m/mol*K], universal gas constant
        #return -(R*T/(M*g))*math.log(self.P0*0.000986923)
        return (1-pow(self.P0/1013.25,0.190284))*145366.45*0.3048
    # Cribbed from datasheet
    def _calculate(self):
        OFFi = 0
        SENSi = 0
        Ti = 0
        
        dT = self._D2-self._C[5]*pow(2,8)
        
        self._temperature = 2000+dT*self._C[6]/pow(2,23)
        
        SENS = self._C[1]*pow(2,15)+(self._C[3]*dT)/pow(2,8)
        OFF = self._C[2]*pow(2,16)+(self._C[4]*dT)/pow(2,7)
    
        self._pressure = (self._D1*SENS/pow(2,21)-OFF)/pow(2,13)

       

        # Second order compensation
     
        if (self._temperature/100) < 20:  # Low temp
            Ti = 3*pow(dT,2)/pow(2,33)
            OFFi = 3*pow(self._temperature-2000,2)/2
            SENSi = 5*pow(self._temperature-2000,2)/pow(2,3)
            if (self._temperature/100) < -15:  # Very low temp
                OFFi = OFFi+7*pow(self._temperature+1500,2)
                SENSi = SENSi+4*pow(self._temperature+1500,2) 
        else:  # High temp
                Ti = 2*pow(dT,2)/pow(2,37)
                OFFi = pow(self._temperature-2000,2)/pow(2,4)
                SENSi = 0

        OFF2 = OFF-OFFi
        SENS2 = SENS-SENSi

        
        self._temperature = (self._temperature-Ti)/100
        self._pressure = (((self._D1*SENS2)/pow(2,21)-OFF2)/pow(2,13))/10.0

    # Cribbed from datasheet
    def _crc4(self, n_prom):
        n_rem = 0

        n_prom[0] = ((n_prom[0]) & 0x0FFF)
        n_prom.append(0)

        for i in range(16):
            if i % 2 == 1:
                n_rem ^= ((n_prom[i >> 1]) & 0x00FF)
            else:
                n_rem ^= (n_prom[i >> 1] >> 8)

            for n_bit in range(8, 0, -1):
                if n_rem & 0x8000:
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem = (n_rem << 1)

        n_rem = ((n_rem >> 12) & 0x000F)

        self.n_prom = n_prom
        self.n_rem = n_rem

        return n_rem ^ 0x00



def main(args=None):
    rclpy.init(args=args)
    node = MS5837_30BA()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
