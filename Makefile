#**********************************
#
# 测试传感网数据的Makefile
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

ndw_main: ndw_main.o ndw_main.c
#$(CC) -Wall -o -lpthread $@ ndw_main.o
	gcc ndw_main.c -o ndw_main -lpthread
	

clean:
	rm *.o $(TARGET)



