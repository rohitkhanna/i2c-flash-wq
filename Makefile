# Makefile for BeagleBoard Assignment 3 Task3
#	How to Use --->
# 		export PATH=/usr/local/angstrom/arm/bin:$PATH
# 		make ARCH=arm CROSS_COMPILE=arm-angstrom-linux-gnueabi-
	

# Compiler options
CC = /usr/local/angstrom/arm/bin/arm-angstrom-linux-gnueabi-gcc
FLAGS = -Wall -I -L -g -lpthread

#Remote deploy ( plz make sure SSH_IP is IP address of beagleboard )
SSH_IP = 192.168.1.115
SSH_USER = root
SSH_DIR = /home/root/Lab3

DRIVER_NAME1 = i2c-flash-wq
APP_NAME1 = user-app-task3

obj-m := $(DRIVER_NAME1).o
PWD := $(shell pwd)
KDIR := /home/rohit/cse598-ESP/TA-copy/kernel/

all:
	make -C $(KDIR) SUBDIRS=$(PWD) modules
	
app: $(APP_NAME1) 
	$(CC) $(FLAGS) -o $(APP_NAME1) $(APP_NAME1).c

clean:
	make -C $(KDIR) SUBDIRS=$(PWD) clean
	rm -f $(APP_NAME)

load-driver: 
	@echo "Copying ko to BeagleBoard..."
	scp $(DRIVER_NAME1).ko  $(SSH_USER)@$(SSH_IP):$(SSH_DIR)
	@echo "Done!"
	
load-app:
	@echo "Copying app to BeagleBoard..."
	scp  $(APP_NAME1) $(SSH_USER)@$(SSH_IP):$(SSH_DIR)
	@echo "Done!"


