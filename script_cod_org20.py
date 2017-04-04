#!/usr/bin/python
from subprocess import call
import os
import smtplib

cfgArray = ["BlowingBubbles","RaceHorses","KristenAndSara","SlideEditing","BasketballDrive","Traffic"]
BitRate = ["354935","974541","454773","301906","2422925","2656191"]
#nomeArray = ["BlowingBubbles_80.bin","RaceHorses_80.bin","KristenAndSara_80.bin","SlideEditing_80.bin","BasketballDrive_80.bin","Traffic_80.bin",]
#matlabArray = ["BlowingBubbles_80.csv","RaceHorses_80.csv","KristenAndSara_80.csv","SlideEditing_80.csv","BasketballDrive_80.csv","Traffic_80.csv",]

for i in range(len(cfgArray)):

	execcod = "./bin/TAppEncoderStatic"
	execc = " -c cfg/encoder_randomaccess_main.cfg -c cfg/per-sequence/" + cfgArray[i] + ".cfg " + "--BitstreamFile=" + cfgArray[i] + "_20.bin" + " --TargetBitrate=" + BitRate[i] + " > " + cfgArray[i] + "_20info.txt"

	exe = execcod + execc

	rename = "mv traceMatlab.csv " + cfgArray[i] + "_20.csv"

	print exe
	os.system(exe)

	print rename
	os.system(rename)	
