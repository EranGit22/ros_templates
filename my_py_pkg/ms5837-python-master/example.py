#!/usr/bin/env python3
import ms5837
import time
from time import sleep

sensor = ms5837.MS5837_30BA()  # Default I2C bus is 1 (Raspberry Pi 3)
# sensor = ms5837.MS5837_30BA(0) # Specify I2C bus
#sensor = ms5837.MS5837_02BA()
#sensor = ms5837.MS5837_02BA(0)
# sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, bus=0) # Specify model and bus

# We must initialize the sensor before reading it
#sensor.init()
#sensor.read()
#sensor.calc_P0()
# if not sensor.init():
#        print("Sensor could not be initialized")
#        exit(1)

# We have to read values from sensor to update pressure and temperature
# if not sensor.read():
#    print("Sensor read failed!")
#    exit(1)

print("Pressure:" + str(sensor.pressure(ms5837.UNITS_atm)) + " [atm] , " +
      str(sensor.pressure(ms5837.UNITS_Torr)) + "[Torr] , " + str(sensor.pressure(ms5837.UNITS_psi)) + " [psi]")

print("Temperature:" + str(sensor.temperature(ms5837.UNITS_Centigrade)) + ' [C] , ' +
      str(sensor.temperature(ms5837.UNITS_Farenheit)) + '[F] , ' + str(sensor.temperature(ms5837.UNITS_Kelvin))+' [K]')


density=sensor.get_FW_density()
print('Fresh water density: ' + ' ' + str(density)+ ' [kg/m^3]')
freshwaterDepth = sensor.depth()  # default is freshwater
#sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
#saltwaterDepth = sensor.depth()  # No nead to read() again
#print("Depth: %.3f m (freshwater)  %.3f m (saltwater)" %
      #(freshwaterDepth, saltwaterDepth))
print("Depth: %.3f m (freshwater)" % (freshwaterDepth))
# ASK IF IT'S NEEDED
sensor.setFluidDensity(1000)  # kg/m^3, 1000-density in the air
# fluidDensity doesn't matter for altitude() (always MSL air density)
# relative to Mean Sea Level pressure in air
print("MSL Relative Altitude: %.2f [m]" % sensor.altitude())


time.sleep(5)

# Spew readings
while True:
    if sensor.read():
        sensor.get_FW_density()
        print("P: %.1f [mbar]  %.3f [psi] , T: %.2f [C]  %.2f [F] Depth: %.2f [m]" % (sensor.pressure(),sensor.pressure(ms5837.UNITS_psi),sensor.temperature(),sensor.temperature(ms5837.UNITS_Farenheit),sensor.depth()))
        sleep(3)
    else:
        print("Sensor read failed!")
        exit(1)
