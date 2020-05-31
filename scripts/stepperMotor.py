#!/usr/bin/env python

StepCount = 8
Seq = range(0, StepCount)
Seq[0] = [0,1,0,0]
Seq[1] = [0,1,0,1]
Seq[2] = [0,0,0,1]
Seq[3] = [1,0,0,1]
Seq[4] = [1,0,0,0]
Seq[5] = [1,0,1,0]
Seq[6] = [0,0,1,0]
Seq[7] = [0,1,1,0]

def setStep(pins, cmd):
    GPIO.output(pins[0], cmd[0])
    GPIO.output(pins[1], cmd[1])
    GPIO.output(pins[2], cmd[2])
    GPIO.output(pins[3], cmd[3])

def forward(delay, steps, pins):
    for i in range(steps):
        for j in range(StepCount):
            setStep(pins, Seq[j][:])
            time.sleep(delay)

def backwards(delay, steps):
    for i in range(steps):
        for j in reversed(range(StepCount)):
            setStep(pins, Seq[j][:])
            time.sleep(delay)
