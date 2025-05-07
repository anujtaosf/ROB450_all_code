<img src="https://github.com/anujtaosf/ROB450_all_code/tall_ballbot_vector.png](https://github.com/anujtaosf/ROB450_all_code/blob/main/tall_ballbot_vector.png" width="1024">


## Rpi Setup

If you have not already configured your Raspberry Pi, please first follow the instructions in the [Rpi Setup Documentation](https://aquamarine-law-a2d.notion.site/ROB311-Instruction-Raspberry-Pi-Setup-15a0653ccdcc8030a996c165d6098bb4) for more information.

## Installation

After cloning the repo, run the setup script:

```bash
./setup.sh
```
This setup script will install CMAKE tools and other dependencies that are essential to build the firmware.

## Building the MBot firmware

Build as follows:
```bash
cd ./mbot-omni-firmware/build
cmake ..
make
```

## Flashing the MBot firmware on RaspberryPi Pico

```bash
picoload /dev/sda1
```
Note: The drive name ("sda1") changes everytime you connect the pico to the Rpi.

## Enabling SPI and I2C
Please enable **SPI** and **I2C** using **raspi-config**
```bash
sudo raspi-config
```
Navigate to **Interface Options** and then enable **SPI** and **I2C**.
