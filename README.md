# PM2.5 Sensor API Library #

## Introduction ##

This library creates a simple API to send commands and retrieve data
from a PMS 5003 PM2.5 Sensor. With the addition of a user-supplied
interface library, this API should produce reliable communication
between an interfacing MCU, computer, or other such device.

## License ##

PM2.5 Sensor API Library is written by Tyler Anderson and is released
under a BSD 3-Clause license (see [LICENSE](./LICENSE) file for more
details).

## Building ##

This project uses cmake to generate build systems in a portable
manner. The library is defined as an interface library, so when linked
to other projects it will not build as a shared or static archive to
link to during linking or running.

To build and link to this library from an existing project, clone the
repository or copy the tarball into your project directory and add the
following to your CMakeLists.txt:

```cmake
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/pm2_5-sensor-api)

target_link_libraries(<your-interface> pm2_5-sensor-api)
```

Depending on your project layout you may need to customize this to
your needs.

If you do not use cmake, simply point your build tools the
[`src`](src) directory for sources to compile and the
[`include`](include) for includes.

## Testing ##

PM2.5 Sensor API Library comes with a short testing suite that by
default will not be built or included by projects linking the library.

To build tests, the `PM2_5_BUILD_TESTS` variable will need to be
defined. Note that this must be compiled with a compiler matching the
architecture of the system you will run the tests on, likely your
workstation's, and not necessarily the target platform.

The procedure for building and running tests is as follows:

```bash
# Build the tests
cd pm2_5-sensor-api
cmake -DPM2_5_BUILD_TESTS=1 -S . -B build
cmake --build build

# Run the tests
cd build
ctest
```

## Usage ##

To use this library include the headers as shown below, then write the
interface callbacks to allow use of the UART hardware, software,
and/or firmware on your platform.

Use the following UART settings (or use macros in parenthesis):

* Baud rate: 9600 (`PM2_5_DEFAULT_BAUD`)
* Stop bits: 1 (`PM2_5_STOP_BIT`)
* Parity: NONE

```c
#include <pm2_5.h> /* Must include this to use API */
#include <pm2_5-error.h> /* Optional: for decoding errors */

#include <your-hardware-api>

/* Set user callbacks, using your platform's uart RX and TX
 * functions for receive and send respectively */

int8_t pm2_5_user_send(const uint8_t *data,
                       uint8_t len, void *intf_ptr)
{
        /* Your interface code here
         * return 0 on success, return otherwise on failure */
}

int8_t pm2_5_user_receive(uint8_t *data,
                          uint8_t len, void *intf_ptr)
{
        /* Your interface code here
         * return 0 on success, return otherwise on failure */
}

uint8_t pm2_5_user_available(void *intf_ptr)
{
        /* Your interface code here
         * return 0 if no data, return 1 if data */
}
```

Once you have set your callbacks, configure the `pm2_5_dev` object
with the callbacks and your interface pointer (a pointer you define to
pass platform data to callbacks).

```c
int main()
{
	int8_t rslt;
	pm2_5_dev dev; /* Pass this to all API functions */
	pm2_5_data data; /* Load data into this with pm2_5_get_data() */
	
	/* replace the following with your platform's uart interface */
	my_platform_uart *intf = MY_PLATFORM_UART_X_PTR;
	
	/* Set your callbacks and interface ptr */
	dev.send_cb = pm2_5_user_send;
	dev.receive_cb = pm2_5_user_receive;
	dev.available_cb = pm2_5_user_available;
	dev.intf_ptr = (void*) intf;
	
	/*
	 * Add your platform's UART setup code here
	 */
	
	/* Initialize the sensor object */
	rslt = pm2_5_init(&dev);
	
	if (rslt != PM2_5_OK) {
		/* Handle error if initialization failed */
	}
	
	/* Put sensor into passive mode to read data on-demand */
	rslt = pm2_5_set_mode(&dev, PM2_5_MODE_PASSIVE);
	
	if (rslt != PM2_5_OK) {
		/* Handle error if failed to change modes */
	}
	
	for (;;) {
		/* read data from sensor */
		rslt = pm2_5_get_data(&dev, &data);
		
		if (rslt != PM2_5_OK) {
			/* Handle error if failed to get data */
		}
		
		/*
		 * Do stuff with received data (like print to stdout)
		 *
		 * Then do other things (sleep, poll, check other hardware)
		 */
	}
	
	return 0; /* Return as desired when loop is broken */
}
```

Please see [`include/pm2_5.h`](include/pm2_5.h) for a complete list of
API functions and [`include/pm2_5-error.h`](include/pm2_5-error.h) for
the error decoding API.

## External Links ##

The PMS 5003 PM2.5 Air Quality sensor can be purchased from
[Adafruit](https://www.adafruit.com) at the following address:

* [Product Page](https://www.adafruit.com/product/3686#technical-details)

The manual for the PMS 5003 with discussion of installation and
communications protocol can be found at the below link:

* [PMS 5003 Manual](https://cdn-shop.adafruit.com/product-files/3686/plantower-pms5003-manual_v2-3.pdf)
