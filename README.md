# SMI230 Input Sensor-API and Sensor Driver

## Table of Contents
 - [Introduction](#Intro)
 - [License](#License)
 - [Sensor interfaces](#interfaces)
 - [Architecture](#Architecture)
 - [Continuous Integration](#Continuous-Integration)
 - [Operation examples](#examples)

## Introduction <a name=Intro></a>

SMI230 is a system-in-package inertial measurement unit which offers accurate acceleration and angular rate measurements.
Due to system-in-package approach of SMI230 (two sensors in single package), the gyroscope and acceleration data is acquired in a non-synchronized manner.
However, synchronization between accelerometer and gyroscope can be achieved:
The software modules in this repository are provided as reference for SMI230 users and shall demonstrate exemplarily the usage of the following features:
- Data synchronization.
- Data collection from FIFO.

_Note: SMI230 is rather an Industrial I/O (IIO) than an input device. Therefore, we now implement SMI230 as an IIO driver (see [GitHub](https://github.com/boschmemssolutions/SMI230-Linux-Driver-IIO)). Further development will only happen on the IIO driver, not on the input driver anymore._

_Note: The sensor driver utilizes a sensor API, which is following BMI08x's sensor API, available on [GitHub](https://github.com/BoschSensortec/BMI08x-Sensor-API/releases/tag/bmi08x_v1.4.4)._

_Note: The data synchronization feature utilizes the sensor's configuration, which is following BMI08x sensor configuration, available on [GitHub](https://github.com/BoschSensortec/BMI08x-Sensor-API/releases/tag/bmi08x_v1.2.0)._

## License <a name=License></a>
See [LICENSE](drivers/input/sensors/smi230/LICENSE.md) file

## Sensor interfaces <a name=interfaces></a>
* I2C
* SPI

## Architecture <a name=Architecture></a>
```
                  User space
-------------------------------------------------------
                 |          |
               sysfs       dev
                 \          /
               input-subsystem
                      |
sensor_API <-- smi230_driver --> smi230_SPI/I2C_driver
                                           |
                                      SPI/I2C_bus
                                           |
-------------------------------------------------------
                  Hardware
```
## Continuous Integration <a name="Continuous-Integration"></a>
Changes to the repository are automatically checked through Continuous Integration. A CI/OSE hosted [Jenkins Master as a Service (JMaaS)](https://inside-docupedia.bosch.com/confluence/x/L2JgH) instance is used for hosting the automation pipeline. This project's instance is called [RBK-BSEHSW](https://rb-jmaas.de.bosch.com/RBK-BSEHSW/blue/pipelines). Connected to this instance are multiple self-hosted on-premise Jenkins Agents:
* `rt-v-00010.rt.de.bosch.com` - Virtual Machine hosted inside NE/SW-Campus Workplace Sandbox. **Not intended for productive use!.** The VM is NAT'ed. Inbound connections to this VM must be performed through the VM-Host `rt-x7270.rt.de.bosch.com`. SSH port-forwarding from Host to VM is set up on port 5555➔22.
* `rt-z0jxb.rt.bosch.com` - Physical Machine (Laptop) located in-office.

As the SMI230 driver may be built using different (sometimes mutually exclusive) configuration options, some form of **Variant Management** has to be in place. In Jenkins, Variant Management is typically performed using [Matrix Pipelines](https://www.jenkins.io/blog/2019/11/22/welcome-to-the-matrix/). In a Matrix Pipeline, variants are described by so-called axes. To explain it using examples, one such axis may describe the target platform, e.g. `raspberrypi-2`, `raspberrypi-3`, `raspberrypi-4`, etc. A second axis may describe the kernel configuration, e.g. `datasync`, `fifo`, etc. The Matrix Pipeline will now combine both axes and execute the declared build stages for each one. This results in $N \times M$ parallel jobs, $N$ being the number of target platforms and $M$ being the number of kernel configurations. The number of axes is arbitrary. To avoid an explosion of variants it was decided to only verify use-case oriented variants.

Adding more variants to the CI build pipeline is described [here](jenkins/readme.md).

## Operation examples <a name=examples></a>
1. Userspace
The driver exposes a device file node under /dev/input/event*, which can be read as a normal Linux file. Tools like evtest can also be used for read data out. E.g.:
```
sudo evtest /dev/input/event0
```
The data will be displayed on the console with timestamp.

2. Sysfs
The driver also exposes a set of sysfs nodes under `/sys/devices/virtual/input/input*`, where users can get information about the sensor and also control the sensor. E.g.:
```
# read the acc power config
cat /sys/devices/virtual/input/SMI230ACC/pwr_cfg

# set the acc power config active, this command is needed if acc needs to be fully fuctional.
echo 0 > /sys/devices/virtual/input/SMI230ACC/pwr_cfg

# set the acc power config suspend
echo 3 > /sys/devices/virtual/input/SMI230ACC/pwr_cfg

# read the gyro power config
cat /sys/devices/virtual/input/SMI230GYRO/pwr_cfg

# set the gyro power config active, this command is needed if gyro needs to be fully fuctional.
echo 0 > /sys/devices/virtual/input/SMI230GYRO/pwr_cfg

# set the gyro power config suspend
echo 3 > /sys/devices/virtual/input/SMI230GYRO/pwr_cfg

# read the chip id
cat /sys/devices/virtual/input/SMI230GYRO/chip_id

# read the synced acc data
cat /sys/devices/virtual/input/SMI230ACC/data_sync

# read the asynced acc data
cat /sys/devices/virtual/input/SMI230ACC/acc_value

# read the gyro data
cat /sys/devices/virtual/input/SMI230GYRO/gyro_value

```
