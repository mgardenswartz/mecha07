# -*- coding: utf-8 -*-
"""
CreatSed on Tue Sep 26 16:02:53 2023

@author: Jonathan Lam and Max Gardenswartz
"""
# import from python library
from pyb import ExtInt, Pin, ADC, Timer
from array import array

idx=0                          #start index at 0
PC0 = Pin(Pin.cpu.C0,mode=Pin.OUT_PP) #define pin C0 (output)
adc = ADC(PC0)                 #define were ADC will be located
PC1 = Pin(Pin.cpu.C1,mode=Pin.OUT_PP) #define pin C1 (input)
tim7 = Timer(7, freq=1000)     #it will be a timer 7 with 1 khz
data = array('H', 2500*[0])    # make data for 2500 entries at 0 each
PC1.low()                      #turn the input off to start out low

def tim_cb(tim7):              #define the callback every time Timer 7 overflows
    global data, idx            #make the data and index be global
   
    if idx==2500:               #if the idx is at the max capacity of the array do this
       tim7.callback(None)     # disable the callback
    else:                       # if is less than the max capacity of the array do this
       data [idx] = adc.read() # read an analog value
       idx += 1                #and increase the array positioning 

def collect_data():            #define what collect data is so we need to type this on putty
    global idx                #make the index global
    PC1.high()                #turn the input on
    tim7.callback(tim_cb)     #generate a callback
     
    for i in range(len(data)):#generate a list up to 2500
        print(f"{i}, {data[i]}") #print it into the array up to 2500
    PC1.low()                 #turn input off
    idx = 0                   #after doing all that make the index start at 0 position again so it redo it

