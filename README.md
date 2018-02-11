# NRF24L01+
Dedicated tutorial to install this library for a Raspberry.<br/>
The main goal of this tutorial is prepare a new environment to install and use the dedicated library to use properly the NRF24L01+ with Python 2 and Python 3

Documentation about classes:<br/>
* [RF24](http://tmrh20.github.io/RF24) - Official library documentation

## Prepare the system
- Create a dedicated user
- Enable SPI pins (depending on the Raspberry version)
```bash
sudo raspi-config
```
- Depending on the version, you should go into the "Advanced Options" or "Interfacing Options"
- Enable the SPI kernel module
- Reboot your system
```bash
sudo reboot
```
- Add your new user into the GPIO group
```bash
usermod -aG gpio user_name
```
- Add your new user into the SPI group
```bash
usermod -aG spi user_name
```
- To continue these installation steps, please install these packages
```bash
apt-get install python-pip python-dev python3-dev build-essential libboost-python-dev python-setuptools
```
- Create a symlink with the python 3 libboost (the real file name depends on your python 3 version)
```bash
ln -s /usr/lib/arm-linux-gnueabihf/libboost_python-py35.so /usr/lib/arm-linux-gnueabihf/libboost_python3.so
```

## Installation into Python virtualenv
### Virtualenv requirements
You have to be into your home directory to execute these commands.<br/>
For this installation, we use Python 3.
- Install virtualenv python library
```bash
pip install virtualenv virtualenvwrapper
```
- Make a backup of your bashrc
```bash
cp .bashrc .bashrc.bk
```
- This command will create a directory to save every virtualenv
```bash
printf '\n%s\n%s\n%s' '# virtualenv' 'export WORKON_HOME=~/virtualenvs' 'source /usr/local/bin/virtualenvwrapper.sh' >> ~/.bashrc
```
- Reload the .bashrc
```bash
source .bashrc
```
- Create the directory (if not exist)
```bash
mkdir -p $WORKON_HOME
```
- Create a virtualenv using python 3 (replace the project with your project name)
```bash
mkvirtualenv -p $(which python3) project
```

### Clone the project
- Then clone the project into the workspace directory
```bash
git clone repository_url
```

### Installation of python requirements
For these commands, you should be into the cloned project directory.<br/>
If these commands doesn't work, you can try with the sudo but be careful, you have to use your virtualenv and not the Python included into your system.<br/>
If you have to do this in sudo, replace the python work with the location of your python into your virtualenv.
- Example:
```bash
python3 setup.py install
```
- will become
```bash
sudo location/to/my/virtualenv/bin/python3 setup.py install
```
So continue this tutorial installation:
- Install python requirements
```bash
pip install -r requirements.txt
```

### Installation of the python library (for the NRF24L01+)
- Go into the pyRF24 directory
```bash
cd pyRF24
```
- Build the library (into your virtualenv)
```bash
python3 setup.py build
```
- Then install the library (into your virtualenv)
```bash
python3 setup.py install
```

## Use the NRF24 library with Python
- Finally, create your python file and execute it
```bash
python3 my_file.py
```
If you have some issue with the error message "Segmentation fault", it's because you have to use your python in sudo mode.<br/>
I had the same issue, even when I added the access rights to GPIO and SPI.<br/>
I currently doesn't have any idea why this issue happens.<br/>
So, just like explained before: execute this command to run your python file in sudo mode:
```bash
sudo location/to/my/virtualenv/bin/python3 my_file.py
```

## Example
### Raspberry configuration
#### PIN
NRF24L01+ Pin name (Pin number) => Raspberry Pin number (GPIO)<br/>
GND (1) => 6 (Ground; any GND pin)<br/>
VCC (2) => 1 (3.3v; any 3.3v pin)<br/>
CE (3) => 15 (GPIO22)<br/>
CSN (4) => 24 (GPIO08; SPI_CE0_N)<br/>
SCLK (5) => 23 (GPIO11; SPI_CLK)<br/>
MOSI (6) => 19 (GPIO10; SPI_MOSI)<br/>
MISO (7) => 21 (GPIO09; SPI_MISO)<br/>
IRQ (8) => 18 (GPIO24) Optional<br/>

#### Code (python)
```python
#!/usr/bin/env python
# Homiot Copyright 2018
# Private software - No copy/reproduction without explicit authorization
#
import time
from RF24 import *
import RPi.GPIO as GPIO

irq_gpio_pin = None
# RF24(CE Pin, CSN Pin, SPI Speed)
# Setup for GPIO 22 CE and CE0 CSN with SPI Speed @ 16Mhz
# Pin CE = 15; CSN = 0 (it's not a pin, it's the SPI interface to use, between 0 and 1)
radio = RF24(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_16MHZ)
# Alternative configuration, depending on the Raspberry version (not tested)
#RPi B
# Setup for GPIO 15 CE and CE1 CSN with SPI Speed @ 8Mhz
#radio = RF24(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_8MHZ)
#RPi B+
# Setup for GPIO 22 CE and CE0 CSN for RPi B+ with SPI Speed @ 8Mhz
#radio = RF24(RPI_BPLUS_GPIO_J8_15, RPI_BPLUS_GPIO_J8_24, BCM2835_SPI_SPEED_8MHZ)
# Setup for connected IRQ pin, GPIO 24 on RPi B+;
# If you plug the IRQ pin, you can uncomment this line
#irq_gpio_pin = RPI_BPLUS_GPIO_J8_18


##########################################
def read_something(number_pin_event=False):
    if radio.available():
        while radio.available():
            size = radio.getDynamicPayloadSize()
            raw_data = radio.read(size)
            message = raw_data.decode('utf-8')
            print('Got payload size={} value="{}"'.format(size, message))
            # First, stop listening so we can talk
            radio.stopListening()
            # Encode it (str to byte)
            write_something(str.encode(message))

def write_something(message):
    radio.stopListening()
    v = radio.write(message)
    if v:
        print("Write success")
    else:
        prin("Write fail")
    radio.startListening()

pipe_read = 0xF0F0F0F0E1
pipe_write = 0xE8E8F0F0E1
channel = 0x76
radio.begin()
radio.enableDynamicPayloads()
radio.enableAckPayload()
#radio.setRetries(0,0)
radio.setPALevel(RF24_PA_MIN)
radio.setAutoAck(True)
radio.setChannel(channel)
radio.printDetails()

if irq_gpio_pin:
    # set up callback for irq pin
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(irq_gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(irq_gpio_pin, GPIO.FALLING, callback=read_something)

radio.openWritingPipe(pipe_write)
radio.openReadingPipe(1, pipe_read)
radio.startListening()
# Send a first message
write_something(b"Coucou - Hello - Haha")
# forever loop
cpt = 0
while 1:
    # if there is data ready
    if not irq_gpio_pin:
        # no irq pin is set up -> poll it
        read_something()
    else:
        # callback routine set for irq pin takes care for reading -
        # do nothing, just sleeps in order not to burn cpu by looping
        time.sleep(0.11)
    cpt += 1
    if cpt > 100:
        cpt = 0
        write_something(b"Je n'ai pas eu ton message =/")
```

### Arduino configuration
#### PIN
NRF24L01+ Pin name (Pin number) => Arduino Pin number<br/>
GND (1) => Any GND pin<br/>
VCC (2) => Any 3.3v pin<br/>
CE (3) => 7<br/>
CSN (4) => 8<br/>
SCLK (5) => 52<br/>
MOSI (6) => 51<br/>
MISO (7) => 50<br/>
IRQ (8) => Not used<br/>

#### Code (C)
```C
#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(0x76);
  radio.openWritingPipe(0xF0F0F0F0E1LL);
  radio.openReadingPipe(1, 0xE8E8F0F0E1LL);
  radio.enableDynamicPayloads();
}

void loop() {
  boolean sent = false;
  int cpt = 0;
  int retry = 10;
  radio.startListening();
  char receivedMessage[32];
  if(radio.available()){
      Serial.println("Radio available to receive something");
      int size = radio.getDynamicPayloadSize();
      radio.read(receivedMessage, size);
      Serial.println(receivedMessage);
      radio.stopListening();
      Serial.println("Re-send the message");
      while (cpt < retry && !radio.write(&receivedMessage, sizeof(receivedMessage))) {
        cpt++;
      }
      if(cpt <= retry){
        sent = true;
      }
      if(sent){
        Serial.println("Re-Send SUCCESS");
      }
      else {
        Serial.println("Re-Send FAIL!");
      }
      delay(100);
  }
  delay(110);
}
```
Note: if you have issue about packet lost during message sending, you can try to put a 10µf (10V) on the NRF24L01+ module, from the GND and the 3.3v

See http://tmrh20.github.io/RF24 for all documentation
