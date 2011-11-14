################################################################################

1. How to Build
	- get Toolchain
		From android git server , codesourcery and etc ..
		 - arm-eabi-4.4.0
		
	- edit Makefile
		edit "CROSS_COMPILE" to right toolchain path(You downloaded).
		  EX)  CROSS_COMPILE= $(android platform directory you download)/android/prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin
		  Ex)  CROSS_COMPILE=/usr/local/toolchain/arm-eabi-4.4.0/bin/arm-eabi-		// check the location of toolchain
  	
		$ make arch=arm c1_rev02_defconfig
	    $ make

2. Output files
	- Kernel : arch/arm/boot/zImage
	- module : drivers/*/*.ko

3. How to Clean	
		$ make clean
################################################################################
