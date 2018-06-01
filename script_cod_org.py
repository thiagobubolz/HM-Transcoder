#!/usr/bin/python
from subprocess import call
import os
import smtplib

cfgArray = ["SlideShow","Johnny","BQSquare","PartyScene","RollerCoaster","ToddlerFountain",]
for cfg in cfgArray :

	execcod = "./bin/TAppEncoderStatic"
	execc = " -c cfg/encoder_randomaccess_main.cfg -c cfg/per-sequence/" + cfg + ".cfg " + "--BitstreamFile=" + cfg + "_100.bin --QP=\"22\" >> info" + cfg + ".txt"

	exe = execcod + execc

	rename = "mv traceMatlab.csv " + cfg + "_100.csv"

	print exe
	os.system(exe)

	print rename
	os.system(rename)