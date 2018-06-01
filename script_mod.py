#!/usr/bin/python
from subprocess import call
import os
import smtplib

percent = ["80","60","40","20"]
BitRate = ["1419000","1064000","709000","354000"]

for i in range(len(percent)):

	execcod = "./bin/TAppEncoderStatic"
	execc = " -c cfg/encoder_randomaccess_main.cfg -c cfg/per-sequence/BQSquare.cfg --BitstreamFile=BQSquare_" + percent[i] + ".bin" + " --TargetBitrate=" + BitRate[i] + " > BQSquare_" + percent[i] + "info.txt"

	exe = execcod + execc

	rename = "mv traceMatlab.csv BQSquare" + percent[i] + ".csv"

	print exe
	os.system(exe)

	print rename
	os.system(rename)	