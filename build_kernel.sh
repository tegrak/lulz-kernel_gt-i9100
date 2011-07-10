#!/bin/bash

# Set Default Path
TOP_DIR=$PWD
KERNEL_PATH=$TOP_DIR/kernel

# TODO: Set toolchain and root filesystem path
TOOLCHAIN="$TOP_DIR/toolchain/arm-eabi-4.4.0/bin/arm-eabi-"
ROOTFS_PATH="$TOP_DIR/root"

# Copy Kernel Configuration File
cp -f $KERNEL_PATH/arch/arm/configs/c1_defconfig $KERNEL_PATH/.config

make -C $KERNEL_PATH oldconfig || exit -1
make -C $KERNEL_PATH ARCH=arm CROSS_COMPILE=$TOOLCHAIN CONFIG_INITRAMFS_SOURCE="$ROOTFS_PATH" || exit -1

# Copy Kernel Image
cp -f $KERNEL_PATH/arch/arm/boot/zImage .
