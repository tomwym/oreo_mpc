# OREO SOFTWARE
The OREO robot uses Technosoft multi-axis IPOS360 control boards to control the motors on the robot. Simply, the system architecture utilizes a Raspberry Pi to house all control software from which motor commands are sent/received to/from the motor controllers. The control software is split into a communication library ("macaque library"). 

## Macaque Library
The motor controllers support both RS232 and CAN communication protocols. The manual detailing message formats can be found [here](https://www.technosoftmotion.com/ESM-um-html/index.html?tml_electronic_gearing_mode.htm). The macaque library encapsulates all the functionality required to communicate with the controllers. There are three modules which make up the library:

* Library (*libmacaque\_RS232/*
* Communication module (*TML\_RS232\_lib/*)
* Socket interface (*sock\_interface/*)

Currently, the macaque library supports RS232 communication (over Ethernet) only. If CAN communication was required, then the socket interface would need to be altered to support socketCAN interface. Further, the communication module would need to be rewritten to format TML messages into the CAN protocol. The library module was designed in a manner in which the library itself would require minimal changes.

### Building
The macaque library was built with a Windows Raspberry Pi cross-compiler based on the arm-linux-gnueabihf toolchain ([link](https://gnutoolchains.com/raspberry/)). The following *gcc* command will build it. 

    <toolchain>\arm-linux-gnueabihf-gcc.exe -Wall -Werror -Wpedantic -shared 'oreo\code\libmacaque_RS232\macaque_linux.c' 'oreo\code\sock_interface\sock.c' 'oreo\code\TML_RS232_lib\TML_RS232_lib.c' -I 'oreo\code' -o 'oreo\code\build\libmacaque_eth.so' 'lpthread -lm -lrt
    
This will output a *.so* file which can be linked with your control software. The control application which uses the macaque library can be built as follows:

    <toolchain>\arm-linux-gnueabihf-gcc.exe -Wall -Werror -Wpedantic 'your\test\app' -I '\oreo\code' -o 'oreo\code\build\test_pi' -lpthread -lm -lrt -L 'oreo\code\build' -lmacaque_eth<addr>

### Usage
The interface for utilizing the macaque library is found in the **macaque\_linux.h** and **TML\_RS232\_lib.h** header files. 
The **TML\_RS232\_lib.h** provides functions which formats motor commands and queues them for sending to the motor controllers. An example function header is the following:

    void SetVal16(dest_dev_t dev, motor_id_t* dest, uint16_t reg_addr, uint16_t val)

All motor commands have *dev* and *dest* arguments. The *dev* argument specifies which motor controller (eye or neck) is the destination. The *dest* argument defines which motors on the controller will receive the message. There are three types of messages: group, axis, and broadcast. Then depending on the command type, there are arguments based on payload.

The **macaque\_linux.h** has some functionality to switch between motion control modes (torque or position) and setting motion parameters. The library is started and shutdown automatically on application start/end. This interface also provides the ability to read the current position/torque values of each of the motors.

Further, the macaque library provides functionality to perform logging of ongoing motion parameters. Periodically, the library will write the contents of the messages it receives from the motors to .csv files in the same directory as the library. These can easily be opened by a spreadsheet application to process data further. The log files are outputted as follows:

	Timestamp,Log Index,Value\n

The timestamp is calculated in reference to the time of application start. The log index defines the axis and type of data that log entry refers to. The mapping is defined in the **macaque\_linux.h** with the *eye_log_data_id_t* and *neck_log_data_id_t* enums.

If the current motion parameters need to be accessed, the *GetEyeData()*, *GetEyeCalData()*, and *GetNeckData()*. These provide reference to structs holding all the current motion parameters and calibration data. Currently, only position feedback is properly setup for use by other applications. If torque feedback needs to be accessed, mutexes must be added to protect these.

Startup and cleanup of the library are performed automatically on start/end of the control application itself.

## Raspberry Pi Setup
To move the application to the Raspberry Pi, *scp* or *ftp* protocols can be used. For proper run-time linking of the library, the library location needs to be added to the *LD\_LIBRARY\_PATH* system variable. This can be done with the following bash command

    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/your/lib/path/" >> ~/.bashrc

The only other setup that needs to be done on the Raspberry Pi is to set the IP address of the ethernet adapter to match with the *LOCAL_IP* defined in **macaque\_linux.h**. This can easily be done using the *ifconfig* command set.

## Control App
Currently, a position control app (in *control/*) has been developed for the robot. It uses a positional PID loop for each of the motors which uses positional feedback and outputs a torque to set the motors to. The *pid\_loop\_t* struct holds all the parameters of the PID loop which can be tuned properly according to the setup.

The application provides a cmd line interface for the user to control the operation. There are two methods to set the target positions of the motors. First, the user interface provides an option to manually set each of the motors' target positions. Second, a .csv file can be used to set the target positions. It should be formatted as follows:

	target_eye1,target_eye2,target_eye3,target_eye4\n
	target_neck1,target_neck2,target_neck3\n
	
The target positions should be order in ascending axis id's. The order of the rows can be switched as needed.

## Motor Controller Setup
