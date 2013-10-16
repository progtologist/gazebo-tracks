import math
import lxml.etree as ltr
import transformations as trn
import numpy as np
np.set_printoptions(precision=3)

def create_box(root,x,y,z):
    GEOMETRY = ltr.SubElement(root,"geometry")
    BOX = ltr.SubElement(GEOMETRY,"box")
    SIZE = ltr.SubElement(BOX,"size")
    SIZE.text = str(x) +" "+str(y)+" "+str(z)
    return root
    
def create_cylinder(root,radius,length):
    GEOMETRY = ltr.SubElement(root,"geometry")
    CYLINDER = ltr.SubElement(GEOMETRY,"cylinder")
    RADIUS = ltr.SubElement(CYLINDER,"radius")
    LENGTH = ltr.SubElement(CYLINDER,"length")
    RADIUS.text = str(radius)
    LENGTH.text = str(length)
    return root

def create_drum(root,drum_name,x,y,z,radius,width):
    LINK = ltr.SubElement(root, "link", name=drum_name)
    SELF_COLLIDE = ltr.SubElement(LINK, "self_collide")
    SELF_COLLIDE.text = "1"
    POSE = ltr.SubElement(LINK, "pose")
    POSE.text = str(x) +" "+str(y)+" "+str(z)+" "+ str(math.pi/2) +" "+ str(0) +" "+str(0)
    COLLISION = ltr.SubElement(LINK, "collision",name=drum_name+"_collision")
    create_cylinder(COLLISION,radius,width)
    VISUAL = ltr.SubElement(LINK, "visual",name=drum_name+"_visual")
    create_cylinder(VISUAL,radius,width)
    return root
    
def create_base(root,base_name,x,y,z,dx,dy,dz):
    LINK = ltr.SubElement(root, "link", name=base_name)
    SELF_COLLIDE = ltr.SubElement(LINK, "self_collide")
    SELF_COLLIDE.text = "0"
    POSE = ltr.SubElement(LINK, "pose")
    POSE.text = str(x) +" "+str(y)+" "+str(z)+" "+ str(0)+" "+str(0)+" "+str(0)
    COLLISION = ltr.SubElement(LINK, "collision",name=base_name+"_collision")
    create_box(COLLISION,dx,dy,dz)
    VISUAL = ltr.SubElement(LINK, "visual",name=base_name+"_visual")
    create_box(VISUAL,dx,dy,dz)
    return root
    
def create_rev_joint(root,joint_name,parent,child,pose,xyz):
    JOINT = ltr.SubElement(root, "joint", type="revolute", name=joint_name)
    POSE = ltr.SubElement(JOINT, "pose")
    POSE.text = str(pose)
    PARENT = ltr.SubElement(JOINT,"parent")
    PARENT.text = str(parent)
    CHILD = ltr.SubElement(JOINT,"child")
    CHILD.text = str(child)
    AXIS = ltr.SubElement(JOINT,"axis")
    XYZ = ltr.SubElement(AXIS,"xyz")
    XYZ.text = str(xyz)
    return root
    
def add_pad(root,pad_name,coords,dx,dy,dz):
    LINK = ltr.SubElement(root, "link", name=pad_name)
    SELF_COLLIDE = ltr.SubElement(LINK, "self_collide")
    SELF_COLLIDE.text = "1"
    POSE = ltr.SubElement(LINK, "pose")
    POSE.text = str(coords[0]) +" "+str(coords[1])+" "+str(coords[2])+" "+ str(coords[3])+" "+str(coords[4])+" "+str(coords[5])
    COLLISION = ltr.SubElement(LINK, "collision",name=pad_name+"_collision")
    create_box(COLLISION,dx,dy,dz)
    VISUAL = ltr.SubElement(LINK, "visual",name=pad_name+"_visual")
    create_box(VISUAL,dx,dy,dz)
    return root
    
def transform_pad(padcoords,pad_length,pad_separation,pad_thickness,drum_separation,drum_radius,totalAlpha):
    TranG = trn.translation_matrix(( padcoords[0],padcoords[1],padcoords[2] ))
    if padcoords[4]!=0:
        RotG = trn.rotation_matrix(padcoords[4],[0,1,0])
    else:
        RotG = trn.identity_matrix()
    TranJ = trn.translation_matrix(( (pad_length+pad_separation),0,0))
    if (padcoords[0]+pad_separation+pad_length) >(drum_separation/2):
        TranJ_Rot = trn.translation_matrix((-(pad_length+pad_separation)/2,0,0))
        alpha = - np.arctan((pad_length+pad_separation)/(drum_radius))
        
        totalAlpha[0] += alpha
        if totalAlpha[0]<-math.pi:
            alpha -= (totalAlpha[0] - math.pi)
            totalAlpha[0] = -math.pi
        RotJ = trn.rotation_matrix(alpha,[0,1,0],[TranJ_Rot[0][3],TranJ_Rot[1][3],TranJ_Rot[2][3]])
        Final = trn.concatenate_matrices(TranG,RotG,TranJ,RotJ)
        
        angle, direction, point = trn.rotation_from_matrix(Final)
    elif (padcoords[0]-pad_separation-pad_length)<-(drum_separation/2):
        TranJ_Rot = trn.translation_matrix((-(pad_length+pad_separation)/2,0,0))
        alpha = - np.arctan((pad_length+pad_separation)/(drum_radius))
        
        totalAlpha[0] += alpha
        if totalAlpha[0]<-2*math.pi:
            alpha -= (totalAlpha[0] - 2*math.pi)
            totalAlpha[0] = -2*math.pi
        RotJ = trn.rotation_matrix(alpha,[0,1,0],[TranJ_Rot[0][3],TranJ_Rot[1][3],TranJ_Rot[2][3]])
        Final = trn.concatenate_matrices(TranG,RotG,TranJ,RotJ)
        
        angle, direction, point = trn.rotation_from_matrix(Final)
    else :
        Final = trn.concatenate_matrices(TranG,RotG,TranJ)
        angle, direction, point = trn.rotation_from_matrix(Final)
    padcoords = [Final[0][3],Final[1][3],Final[2][3],0,angle,0]
    return padcoords
   
def calculate_alpha(drum_radius,drum_separation,pad_thickness,minimum_pads):
    min_perimeter = 2 * drum_separation + 2 * math.pi * (drum_radius+pad_thickness/2)
    k = 1
    perimeter = 0
    new_drum_radius = 0
    while perimeter<min_perimeter or k<minimum_pads:
        alpha = (2*drum_separation+2*math.pi*(drum_radius+pad_thickness/2)) / k
        for mi in range(k):
            new_drum_radius = mi * alpha / math.pi
            if new_drum_radius>(drum_radius+pad_thickness/2):
                break
        perimeter = 2 * drum_separation + 2 * math.pi * new_drum_radius 
        k += 1
        if k>100:
            raise Exception("No solution found!")
    num_of_pads = np.round(perimeter/alpha)
    perimeter = num_of_pads * alpha
    new_drum_separation = (perimeter - 2 * math.pi * drum_radius)/2
    print "Drum Separation = " + str(drum_separation)
    print "New Drum Separation = " + str(new_drum_separation)
    print "Min Radius = " + str(drum_radius+pad_thickness/2)
    print "Radius = " + str(new_drum_radius)
    print "Min Perimeter = " + str(min_perimeter)
    print "Perimeter = " + str(perimeter)
    return [alpha,num_of_pads,new_drum_radius,new_drum_separation]