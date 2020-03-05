LINUX VERSION:
Used Windows Raspberry Pi cross-compiler (choose version corresponding to Rasbian version on-board) which is based on arm-linux-gnueabihf toolchain found here: https://gnutoolchains.com/raspberry/
Following commands will build the cansock and tml_can_lib modules (need to ensure the socketcan lib is present on your system)
<toolchain>\arm-linux-gnueabihf-gcc.exe -c -Wall -Werror -Wpedantic -fPIC '<location>\cansock.c' -o '<location>\cansock.o'
<toolchain>\arm-linux-gnueabihf-gcc.exe -c -Wall -Werror -Wpedantic -Winline -fPIC '<location>\TML_CAN_lib.c' -o '<location>\TML_CAN_lib.o'
This command will tie those modules into the library build
<toolchain>\arm-linux-gnueabihf-gcc.exe -Wall -Werror -Wpedantic -shared '<location>macaque_linux.c' '<location>cansock.o' -o '<location>libmacaque.so' -lpthread -lm
WINDOWS VERSION:
gcc -shared -o macaque.dll macaque.c -lwsock32
