import sys, os
import time
import json

from bluepy import btle


class Peripheral(object):
    """
    Establish a connection to the device by
    hardware address.
    """

    def __init__(self, address):
        self.sensor_tag = btle.Peripheral(address)


class Sensor(object):
    """
    Base class for a SensorTag CC2650.

    Documentation on this Sensor can be found on the projects wiki:
    http://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide
    """

    def __init__(self, peripheral):
        self.sensor_tag = peripheral.sensor_tag

        # a lot of the handles I use will be specific
        # to the cc2650 sensor tag. If we do not identify
        # this as a cc2650 we should raise an exception before
        # moving on.
        if self.device_model() != 'CC2650 SensorTag':
            raise Exception('This library works with the CC2650 SensorTag')

    def device_name(self):
        return self.sensor_tag.readCharacteristic(0x3)

    def device_model(self):
        return self.sensor_tag.readCharacteristic(0x10)

    def device_firmware(self):
        return self.sensor_tag.readCharacteristic(0x14)

    def device_hardware(self):
        return self.sensor_tag.readCharacteristic(0x16)

    def device_manufacturer(self):
        return self.sensor_tag.readCharacteristic(0x1A)


class Temperature(Sensor):
    """
    The SensorTag CC2650 uses the TMP007 Temperature sensor.

    This class will perform the needed calculations to return
    ambient or object's temperature in celsius and fahrenheit.
    """

    def __as_celsius(self, msb, lsb):
        return ((msb * 256 + lsb) / 4) * 0.03125

    def __as_fahrenheit(self, msb, lsb):
        c = self.__as_celsius(msb, lsb)
        return c * 9/5.0 + 32

    def start(self):
        self.sensor_tag.writeCharacteristic(0x24, '\x01', withResponse=True)

    def stop(self):
        self.sensor_tag.writeCharacteristic(0x24, '\x00', withResponse=True)

    def read(self):
        return self.sensor_tag.readCharacteristic(0x21)

    def get_ambient(self):
        data = self.read()
        if data:
            return ord(data[2]), ord(data[3])

    def get_object(self):
        data = self.read()
        if data:
            return ord(data[0]), ord(data[1])

    def get_ambient_celsius(self):
        lsb, msb = self.get_ambient()
        return self.__as_celsius(msb, lsb)

    def get_object_celsius(self):
        lsb, msb = self.get_object()
        return self.__as_celsius(msb, lsb)

    def get_ambient_fahrenheit(self):
        lsb, msb = self.get_ambient()
        return self.__as_fahrenheit(msb, lsb)

    def get_object_fahrenheit(self):
        lsb, msb = self.get_object()
        return self.__as_fahrenheit(msb, lsb)


class Light(Sensor):
    """
    The SensorTag CC2650 uses the OPT3001 ambient light sensor.

    This class will perform the needed calculations to return
    lux reading from the sensor.
    """

    def start(self):
        self.sensor_tag.writeCharacteristic(0x44, '\x01', withResponse=True)

    def stop(self):
        self.sensor_tag.writeCharacteristic(0x44, '\x00', withResponse=True)

    def read(self):
        return self.sensor_tag.readCharacteristic(0x41)

    def get_ambient_luxs(self):
        lsb, msb = self.read()
        raw = ord(msb) * 256 + ord(lsb)
        m = raw & 0x0FFF
        e = (raw & 0xF000) >> 12
        return m * (0.01 * pow(2.0, e))


class Humidity(Sensor):
    """
    The SensorTag CC2650 uses the HDC1000 humidity sensor.

    This class will perform the needed calculations to return
    humidity reading from the sensor.
    """

    def start(self):
        self.sensor_tag.writeCharacteristic(0x2C, '\x01', withResponse=True)

    def stop(self):
        self.sensor_tag.writeCharacteristic(0x2C, '\x00', withResponse=True)

    def read(self):
        return self.sensor_tag.readCharacteristic(0x29)

    def get_humidity(self):
        _, _, lsb, msb = self.read()
        raw = ord(msb) * 256 + ord(lsb)
        return (raw / 65536.0) * 100


class Pressure(Sensor):
    """
    The SensorTag CC2650 uses the BMP280 humidity sensor.

    This class will perform the needed calculations to return
    barometric pressure reading from the sensor.
    """

    def start(self):
        self.sensor_tag.writeCharacteristic(0x34, '\x01', withResponse=True)

    def stop(self):
        self.sensor_tag.writeCharacteristic(0x34, '\x00', withResponse=True)

    def read(self):
        return self.sensor_tag.readCharacteristic(0x31)

    def get_barometric_pressure(self):
        _, _, _, byte1, byte2, byte3 = self.read()
        byte_a = ord(byte3) << 16
        byte_b = ord(byte2) << 8
        byte_c = ord(byte1)
        return (byte_a + byte_b + byte_c) / 100.0


def main():

    if len(sys.argv) == 2:
        peripheral = Peripheral(sys.argv[1])  # connection to SensorTag

        # instantiate classes for each of our sensors.
        temperature = Temperature(peripheral)
        light = Light(peripheral)
        humidity = Humidity(peripheral)
        pressure = Pressure(peripheral)

        # start measuring on our sensors
        temperature.start()
        light.start()
        humidity.start()
        pressure.start()

        os.system('clear')

        time.sleep(2)  # warm up

        while True:

            # gather results in a python dictionary
            results = dict(
                t=round(temperature.get_ambient_fahrenheit(), 2),
                l=round(light.get_ambient_luxs(), 2),
                h=round(humidity.get_humidity(), 2),
                p=round(pressure.get_barometric_pressure(), 2)
            )

            os.system('clear')

            # pretty print our results
            print 'Temperature: %s' % results['t']
            print 'Light: %s' % results['l']
            print 'Humidity: %s' % results['h']
            print 'Pressure: %s' % results['p']

            f = open('/sensor.json', 'w')
            f.write(json.dumps(results))
            f.close()

            time.sleep(5)

if __name__ == "__main__":
    main()