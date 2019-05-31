#!/usr/bin/python2
# -*- coding=utf-8 -*-
# website: http://www.cnblogs.com/huxi/archive/2010/
# website: https://www.jianshu.com/p/56ad74bc7ac3
from __future__ import print_function
import sys
import time
import thread
import serial


def OpenSerial(_portName, _baudRate):
    try:
        _serial = serial.Serial(_portName, _baudRate)
    except BaseException, err:
        print("Failed to open: ", err, _portName, _baudRate)
        sys.exit(-1)

    if not _serial.isOpen():
        print("Failed to open: ", _portName)
        sys.exit(-1)
    print("Open ", _portName, " successfully.")
    return _serial


def ReadSerialToFile(_serial, _outputFileName):
    try:
        outputFile = open(_outputFileName, "w")
    except BaseException, err:
        print("Failed to open: ", err)
        return

    while True:
        count = _serial.inWaiting()
        if count <= 0:
            continue

        try:
            data = _serial.read(count)
        except BaseException, err:
            print("Failed to read: ", err)
            outputFile.close()
            return

        try:
            outputFile.write(data.decode("utf-8"))
        except BaseException, err:
            # print("Failed to decode: ", data, err)
            continue

    outputFile.close()


def ReadFileToSerial(_inputFileName, _serial, _isGonnaSet5651):
    try:
        inputFile = open(_inputFileName, "r")
    except BaseException, err:
        print("Failed to open: ", err)
        return

    isSetter = True
    for line in inputFile:
        if "setter" in line:
            isSetter = True
            continue
        if "getter" in line:
            isSetter = False
            continue

        if not "cmd" in line:
            print("Illegal line: ", line.rstrip())
            continue

        if _isGonnaSet5651:
            print("Send: ", line.rstrip())
            n = _serial.write(line)
            time.sleep(0.2)
        else:
            if isSetter:
                print("No set: ", line.rstrip())
            else:
                print("Get: ", line.rstrip())
                n = _serial.write(line)
                time.sleep(0.2)

    inputFile.close()


def PrintOutput(_fileName):
    try:
        file = open(_fileName, "r")
    except BaseException, err:
        print("Failed to open: ", err)
        return

    print("\nResult is: ")
    for line in file:
        if not "cmd" in line:
            continue
        if "Config,OK" in line:
            # print()
            continue

        print(line.rstrip())

    file.close()


def PrintUsage():
    print("Usage:\n1. The default behavior of this program is print IMU5651's setting to screen, by running \"python2 set_5651.py\" in terminal;\n2. Run \"python2 set_5651.py SET\" to set IMU5651 according to 5651.conf.\n")


# g_isStopThread = False
if __name__ == "__main__":
    # PrintUsage()
    scriptPath = sys.path[0]
    # isGonnaSet5651 = False
    # if 2 == len(sys.argv):
    #     argv1 = sys.argv[1]
    #     if "SET" == sys.argv[1]:
    #         isGonnaSet5651 = True

    portName = "/dev/ttyUSB2"
    baudRate = 460800
    serial = OpenSerial(portName, baudRate)

    outputFileName = scriptPath + "/output.txt"
    ReadSerialToFile(serial, outputFileName)

    # inputFileName = scriptPath + "/5651.conf"
    # ReadFileToSerial(inputFileName, serial, isGonnaSet5651)

    # g_isStopThread = True
    # print("Waiting for thread to stop...")
    # time.sleep(0.5)

    # PrintOutput(outputFileName)
    serial.close()

