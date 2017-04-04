#!/usr/bin/python
from subprocess import call
import os
import smtplib

cfgArray = ["BlowingBubbles","RaceHorses","KristenAndSara","SlideEditing","BasketballDrive","Traffic",]
#nomeArray = ["BlowingBubbles_100.bin","RaceHorses_100.bin","KristenAndSara_100.bin","SlideEditing_100.bin","BasketballDrive_100.bin","Traffic_100.bin",]
#matlabArray = ["BlowingBubbles_100.csv","RaceHorses_100.csv","KristenAndSara_100.csv","SlideEditing_100.csv","BasketballDrive_100.csv","Traffic_100.csv",]

for cfg in cfgArray :

	execcod = "./bin/TAppEncoderStatic"
	execc = " -c cfg/encoder_randomaccess_main.cfg -c cfg/per-sequence/" + cfg + ".cfg " + "--BitstreamFile=" + cfg + "_100.bin"

	exe = execcod + execc

	rename = "mv traceMatlab.csv " + cfg + "_100.csv"

	print exe
	os.system(exe)

	print rename
	os.system(rename)	