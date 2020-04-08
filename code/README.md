# OREO SOFTWARE
The OREO robot uses Technosoft multi-axis IPOS360 control boards to control the motors on the robot. Simply, the system architecture utilizes a Raspberry Pi to house all control software from which motor commands are sent/received to/from the motor controllers. The control software is split into a communication library ("macaque library"). 

## Macaque Library
The motor controllers support both RS232 and CAN communication protocols. The manual detailing message formats can be found [here](https://www.technosoftmotion.com/ESM-um-html/index.html?tml_electronic_gearing_mode.htm). The macaque library encapsulates all the functionality required to communicate with the controllers. Currently, the macaque library supports 

### Building
The macaque library was built with a Windows Raspberry Pi cross-compiler based on the arm-linux-gnueabihf toolchain ([link](https://gnutoolchains.com/raspberry/)). The following *gcc* command will build it.

    <toolchain>\arm-linux-gnueabihf-gcc.exe -Wall -Werror -Wpedantic -shared 'oreo\code\libmacaque_RS232\macaque_linux.c' 'oreo\code\sock_interface\sock.c' 'oreo\code\TML_RS232_lib\TML_RS232_lib.c' -I 'oreo\code' -o 'oreo\code\build\libmacaque_eth.so' 'lpthread -lm -lrt
    
This will output a *.so* file which can be linked with your control software. The control application which uses the macaque library can be built as follows:

    <toolchain>\arm-linux-gnueabihf-gcc.exe -Wall -Werror -Wpedantic 'your\test\app' -I '\oreo\code' -o 'oreo\code\build\test_pi' -lpthread -lm -lrt -L 'oreo\code\build' -lmacaque_eth<addr>

### Usage
The interface for utilizing the macaque library is found in the **macaque\_linux.h** and **TML\_RS232\_lib.h** header files. 
The **TML\_RS232\_lib.h** provides functions which formats motor commands and add them queues them for sending to the motor controllers. An example is the following: 

    void SetVal16(dest_dev_t dev, motor_id_t* dest, uint16_t reg_addr, uint16_t val)

All motor commands have *dev* and *dest* arguments. The *dev* argument specifies which motor controller (eye or neck) is the destination. The *dest* argument defines which motors on the controller will receive the message. There are three types of messages: group, axis, and broadcast. Then depending on the command type, there are arguments based on payload.

The **macaque\_linux.h** has some functionality to switch between motion control modes (torque or position) and setting motion parameters. The library is started and shutdown automatically on application start/end. This interface also provides the ability to read the current position/torque values of each of the motors.

## Raspberry Pi Setup
To move the application to the Raspberry Pi, *scp* or *ftp* protocols can be used. For proper run-time linking of the library, the library location needs to be added to the *LD\_LIBRARY\_PATH* system variable. This can be done with the following bash command

    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/your/lib/path/" >> ~/.bashrc

The only other setup that needs to be done on the Raspberry Pi is to set the IP address of the ethernet adapter to match with the *LOCAL_IP* defined in **macaque\_linux.h**. This can easily be done using the *ifconfig* command set.

## Motor Controller Setup
