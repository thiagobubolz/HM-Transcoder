#!/usr/bin/python
from subprocess import call
import os
import smtplib

percentage = ["1", "0.8", "0.6", "0.4", "0.2"]
cfg = ["BlowingBubbles_","RaceHorses_","KristenAndSara_","SlideEditing_","BasketballDrive_","Traffic_"]

for perc in percentage:
	for i in range(6) :

		exe = "../bin/TAppDecoderStatic -b " + cfg[i] + perc + "intra.bin -o " + cfg[i] + perc + "intra.yuv"

		print exe
		os.system(exe)