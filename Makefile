#**********************************
#
# src 测试使用的Makefile
#
#**********************************

# 命令变量



CC = gcc
AR = ar
RM = rm
CP = 
#OBJDIR=./obj

# 局部变量
TARGET  = ndw_main

all: $(TARGET)

ndw_main: ndw_main.o 
	$(CC) -Wall -o $@ ndw_main.o
	

clean:
	rm *.o $(TARGET)



