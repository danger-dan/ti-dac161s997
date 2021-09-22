# ti-dac161s997
Linux kernel driver for Texas Instruments DAC161S997 16-bit Single channel 4-20mA DAC. 

https://www.ti.com/lit/ds/symlink/dac161s997.pdf?ts=1616550300980&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FDAC161S997

Simply clone onto target machine run make and make install(su). This will install the module into the /lib/modules/kernel/extras folder. 
Use depmod and modprobe to enable the module. The device tree will also need to be configured for the driver on an SPI bus.
  
  compatible = "ti,dac161s997";
  
  spi-max-frequency = <10000000>;
  
This driver uses the IIO device class and will be found in /sys/devices/iio:device[n] and /sys/bus/iio/devices/iio:device[n].

Copyright (C) 2021 Daniel Tritscher <daniel.j.tritscher@gmail.com>

GPLv2 
