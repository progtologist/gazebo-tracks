# -*- coding: utf-8 -*-
"""
A script that can create a track driven sdf model for gazebo.
To alter the dimensions of the robot edit the file settings.config
minimum_pads is essential for the tuning of the model to determine collision problems with the drums

@author: Aris Synodinos
"""
import lxml.etree as ltr
import decimal
from gazebo_sdf import *

file = open('settings.config','r')  
for line in file:
    if '\n' == line[-1]:
        line = line[:-1]
    exec(line.split(' = ',2)[0] + ' = ' + line.split(' = ',2)[1])
file.close
[alpha,num_of_pads,new_drum_radius,new_drum_separation] = calculate_alpha(drum_radius,drum_separation,pad_thickness,minimum_pads)
pad_separation = round(decimal.Decimal(0.2*alpha),4)
pad_length = round(decimal.Decimal(0.8*alpha),4)

# CIRCLE THROUGH +/-
sign = [1, -1]
    
ROOT = ltr.Element("sdf", version="1.4")
MODEL = ltr.SubElement(ROOT, "model", name = "track_creator")

# CREATE THE BASE_LINK
create_base(MODEL,"base_link",0,0,2*drum_radius,drum_separation,track_separation,2*drum_radius)

# CREATE THE DRUMS
for i in range(4):
    x = sign[i//2] * drum_separation / 2
    y = sign[i%2] * (track_separation / 2 + drum_width / 2)
    z = pad_thickness + drum_radius
    create_drum(MODEL,"drum"+str(i+1),x,y,z,drum_radius,drum_width)
    create_rev_joint(MODEL,"drum_joint"+str(i+1),"base_link","drum"+str(i+1),"0 0 0 0 0 0","0 1 0")

# CREATE THE TRACKS

# CREATE THE RIGHT TRACK
i = 0
totalAlpha = [0]
padcoords = [0, (track_separation+drum_width)/2 , pad_thickness/2, 0, 0, 0]
while i < num_of_pads:
    padcoords = transform_pad(padcoords,pad_length,pad_separation,pad_thickness,new_drum_separation,drum_radius,totalAlpha)
    add_pad(MODEL,"right_pad"+str(i+1),padcoords,pad_length,pad_width,pad_thickness)    
    if i != (num_of_pads-1):
        create_rev_joint(MODEL, "right_pad_joint"+str(i+1),"right_pad"+str(i+1),"right_pad"+str(i+2),str(-(pad_length+pad_separation)/2)+" 0 0 0 0 0","0 1 0")
    else:
        create_rev_joint(MODEL, "right_pad_joint"+str(i+1),"right_pad"+str(i+1),"right_pad1",str(-(pad_length+pad_separation)/2)+" 0 0 0 0 0","0 1 0")
    i += 1

# CREATE THE LEFT TRACK
i = 0
totalAlpha = [0]
padcoords = [0, -(track_separation+drum_width)/2 , pad_thickness/2, 0, 0, 0]
while i < num_of_pads:
    padcoords = transform_pad(padcoords,pad_length,pad_separation,pad_thickness,new_drum_separation,drum_radius,totalAlpha)
    add_pad(MODEL,"left_pad"+str(i+1),padcoords,pad_length,pad_width,pad_thickness)
    if i != (num_of_pads-1):
        create_rev_joint(MODEL, "left_pad_joint"+str(i+1),"left_pad"+str(i+1),"left_pad"+str(i+2),str(-(pad_length+pad_separation)/2)+" 0 0 0 0 0","0 1 0")
    else:
        create_rev_joint(MODEL, "left_pad_joint"+str(i+1),"left_pad"+str(i+1),"left_pad1",str(-(pad_length+pad_separation)/2)+" 0 0 0 0 0","0 1 0")
    i += 1  

# WRITE THE MODEL
f = open(output, 'w')
f.write(ltr.tostring(ROOT, pretty_print=True))
f.close()