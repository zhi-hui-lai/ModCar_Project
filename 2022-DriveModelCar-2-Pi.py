#!/usr/bin/env python3
#============================================================================

Debug = 0
#debug Lidar drive
DebugD=0
Sim_speed=1
Offset = 0
Delay = 0.1

Logging_Performance = 1
Multi_Logging = 0
Hard_Brake_Power = 0.1

#=============================================================================

import os
import sys
import time
import math
import ctypes
import socket
import datetime
from RoBIOS import *
#https://www.tensorflow.org/install/pip

import cv2  
#  os.system('sudo -H pip3 install opencv-python')
    
import pygame  
#  os.system('sudo -H pip3 install pygame==2.0.0.dev6')
    
from PIL import Image, ImageDraw 
#  os.system('sudo -H pip3 install Pillow')

import numpy as np

#========================================================================
    
if(os.path.exists('/home/pi') == True):
    SIMULATION = False
    User = 'sudo '
    Map_Size = 12.5
    Debug = 0
    Stamp_Type = 'Car_'

else:

    if(Debug == 1):
        SIMULATION = False
        User = ''
        
    else:
        SIMULATION = True
        User = ''
        Sim_Scale = 2.5
        Map_Size = 12.5
        Stamp_Type = 'Sim_'

Script = os.path.dirname(os.path.abspath(__file__))
ModelCar_Path = Script[:-9]

CNN_Model_Path = os.path.join(ModelCar_Path, 'Models')
Log_Car_Path = os.path.join(CNN_Model_Path, 'Logs')
Data_Path = os.path.join(ModelCar_Path, 'Data')

if(SIMULATION == True):
    Image_Path = os.path.join(Data_Path, 'Images_Sim')
    Train_Image_Path = os.path.join(Image_Path, 'Train_Images_Sim')
    Test_Image_Path = os.path.join(Image_Path, 'Test_Images_Sim')
    Edge_Path = os.path.join(Data_Path, 'Edges_Sim')
    Training_Path = os.path.join(Edge_Path, 'Training_Sim')
    Validation_Path = os.path.join(Edge_Path, 'Validation_Sim')
    Testing_Path = os.path.join(Edge_Path, 'Testing_Sim')
    Manual_Path = os.path.join(Data_Path, 'Manual_Sim.npy')
    Edge_Camera_Path = os.path.join(Data_Path, 'Edge_Camera_Sim.npy')
    Nvidia_Camera_Path = os.path.join(Data_Path, 'Nvidia_Camera_Sim.npy')
    Lidar_Path = os.path.join(Data_Path, 'Lidar_Sim.npy')
    Edge_Camera_Path = os.path.join(Data_Path, 'Edge_Camera_Sim.npy')
    Nvidia_Camera_Path = os.path.join(Data_Path, 'Nvidia_Camera_Sim.npy')
    Lidar_Path = os.path.join(Data_Path, 'Lidar_Sim.npy')
    Scan_Path = os.path.join(Data_Path, 'Scans_Sim')
    
else:
    Image_Path = os.path.join(Data_Path, 'Images_Car')
    Train_Image_Path = os.path.join(Image_Path, 'Train_Images_Car')
    Test_Image_Path = os.path.join(Image_Path, 'Test_Images_Car')
    Edge_Path = os.path.join(Data_Path, 'Edges_Car')
    Training_Path = os.path.join(Edge_Path, 'Training_Car')
    Validation_Path = os.path.join(Edge_Path, 'Validation_Car')
    Testing_Path = os.path.join(Edge_Path, 'Testing_Car')
    Edge_Camera_Path = os.path.join(Data_Path, 'Edge_Camera_Car.npy')
    Nvidia_Camera_Path = os.path.join(Data_Path, 'Nvidia_Camera_Car.npy')
    Lidar_Path = os.path.join(Data_Path, 'Lidar_Car.npy')
    Manual_Path = os.path.join(Data_Path, 'Manual_Car.npy')
    Edge_Camera_Path = os.path.join(Data_Path, 'Edge_Camera_Car.npy')
    Nvidia_Camera_Path = os.path.join(Data_Path, 'Nvidia_Camera_Car.npy')
    Scan_Path = os.path.join(Data_Path, 'Scans_Car')

Servo_Path = os.path.join(ModelCar_Path, 'Software/ServoBlaster/user/servod')

BreezyLidar_Path = os.path.join(ModelCar_Path, 'Software/BreezyLidar/python/setup.py')
BreezySLAM_Path = os.path.join(ModelCar_Path, 'Software/BreezySLAM/python/setup.py')
Urg_Path = '/dev/ttyACM0'

if(os.path.exists(ModelCar_Path) == False):
    print('[Error!] Software Folder Path not Found')
    sys.exit()
    
elif(os.listdir(ModelCar_Path) == []):
    print('[Error!] Software Folder is Empty')
    sys.exit()

#=============================================================================

Steer_Delta = 4

URG_Distance_Range = 5600
URG_Scan_Range = 240
URG_Scan_Size = 682                         
URG_Angles = []

Draw_Scale = 120/float(URG_Distance_Range)
Draw_Angles = []

if(SIMULATION == False):

    for i in range(682):
        URG_Angles.append(np.deg2rad(-120 + (i*(URG_Scan_Range/URG_Scan_Size))))
        Draw_Angles.append(np.deg2rad(-30 + (i*(URG_Scan_Range/URG_Scan_Size))))

else:
    
    for i in range(682):
        URG_Angles.append(np.deg2rad(120 - (i*(URG_Scan_Range/URG_Scan_Size))))
        Draw_Angles.append(np.deg2rad(210 - (i*(URG_Scan_Range/URG_Scan_Size))))
        
F = URG_Distance_Range
F_Left = URG_Distance_Range
F_Right = URG_Distance_Range
F_Front = URG_Distance_Range   

Min_Distance = 20 
Max_Distance = 5600             

Red_Zone_Max = 5600
Red_Zone_Min = 510


Centre_Steering = 150
#Left_Steering = 200
#Right_Steering = 100
Left_Steering = 195
Right_Steering = 105
Previous_Steer = 150
Steering = Centre_Steering
Rev_Steering = Centre_Steering

#Max_Speed = 175
#Min_Speed = 158
#Max_Allow_Speed = 170
Max_Speed = 175
Min_Speed = 163
Max_Allow_Speed = 170
Normal_Speed = 164
Brake_Speed = 134
Speed_In = 150
Speed_Out = 0
Rev_Speed = 135
old_Speed = 150
Cam_Angle_Half = 90
Angle_Step = 5
Turn_Step = 2

Left_Steer_Flag = 0
Right_Steer_Flag = 0
Image_Turn_Flag = 0

Direction = 1
Turn_State = 0
Turn_State_Count = 0
Auto_Drive_State = 0
Rev_Count = 0
Con_Rev_Counter = 0

Partial_Brake_Offset = 130
Opening_Thres_Max = 1000
Opening_Depth_Min = 2000
#LSTM
sequence=[]
sequence_len=5
#=============================================================================
  
def LIDARBot(scan):     
    global URG_Distance_Range
    global URG_Scan_Range
    global URG_Scan_Size
    global URG_Angles
    
    global Max_Distance
    global Min_Distance
        
    global F 
    global F_Left 
    global F_Right 
    global F_Front
    
    F = URG_Distance_Range
    Average_L = 0
    Average_R = 0
    
    global Red_Zone_Max
    global Red_Zone_Min
    
    global Centre_Steering
    global Left_Steering
    global Right_Steering
    global Steering
    global Previous_Steer
    global Rev_Steering
    
    global Speed_In
    global Speed_Out
    global Max_Speed
    global Min_Speed
    global Max_Allow_Speed
    global Rev_Speed
    Speed = 150 
    
    global Cam_Angle_Half
    global Angle_Step
    global Turn_Step
    Num_Lobes = int(Cam_Angle_Half/Angle_Step)
    
    global Left_Steer_Flag
    global Right_Steer_Flag
    global Image_Turn_Flag
    
    global Direction
    global Turn_State
    global Turn_State_Count
    global Auto_Drive_State
    global Rev_Count
    global Con_Rev_Counter
    
    global Partial_Brake_Offset
    global Opening_Thres_Max

    global Left_Steer_Flag
    global Right_Steer_Flag
    global Image_Turn_Flag 
    
    L_Y = []
    R_Y = []
    L_D = []
    R_D = []
    
    for i in range(0, Num_Lobes + 1):
        L_Y.append(0)
        R_Y.append(0)
        L_D.append(0)
        R_D.append(0)
    
    for i in range(len(scan)):
   
        if(scan[i] <= Min_Distance):
            Dist = Max_Distance


        elif(scan[i] >= Max_Distance):
            Dist = Max_Distance

        else:
            Dist = scan[i]

        Angle_Radians = URG_Angles[i]
        Angle = (Angle_Radians*180)/(math.pi)
        
        if((Angle <= -90) or (Angle >= 90)):
            continue
        
        if(Angle_Radians == math.pi):
            continue
        
        if(Angle_Radians < 0):
            Angle_Radians = Angle_Radians*-1
            
        Dist_Y = math.sin(Angle_Radians)*Dist
        
        if((Dist_Y <= 220) and (Dist_Y >= 0) and (((Angle < -15) and (Angle > -90)) or ((Angle > 15)  and (Angle < 90)))):

           if(Dist < F):
               F = Dist
               F_Front = F
               
        if((Angle >= -15) and (Angle <= 15)):
            
            if(Dist < F):
                F = Dist
                
        if((Angle > -40) and (Angle < -15)):
            
            if(Dist < F_Right):
                F_Right = Dist
                
        if((Angle > 15) and (Angle < 40)):
            
            if(Dist < F_Left):
                F_Left = Dist
                
        if(Angle < 0):
            New_Location = math.ceil(Angle/Angle_Step)*-1  
            
            if((Dist < R_D[New_Location]) or (R_D[New_Location] == 0)):
                R_D[New_Location] = Dist
                
            if((Dist_Y < R_Y[New_Location]) or (R_Y[New_Location] == 0)):
                R_Y[New_Location] = Dist_Y
            
        else:
            New_Location = math.floor(Angle/Angle_Step)  
            
            if((Dist < L_D[New_Location]) or (L_D[New_Location] == 0)):
                L_D[New_Location] = Dist
                    
            if((Dist_Y < L_Y[New_Location]) or (L_Y[New_Location] == 0)):
                L_Y[New_Location] = Dist_Y
            
    if(Max_Speed >= Min_Speed):
        Red_Zone_Test = F
        
        if(F >= Red_Zone_Max):
            Speed = Max_Speed
            if(DebugD == 1):
                print("A1")
                
        elif(F > Red_Zone_Min):
            Speed = Max_Allow_Speed - round((Red_Zone_Max - F)/((Red_Zone_Max - Red_Zone_Min)/(Max_Allow_Speed - Min_Speed)))
            if(DebugD == 1):
                print("A2")
                
        else:
            Speed = Brake_Speed
            if(DebugD == 1):
                print("A3")
            if((Turn_State == 0) and (Speed_In <= 150)  and (Con_Rev_Counter <= 1)):
                Turn_State = 420
                Con_Rev_Counter = Con_Rev_Counter + 1
                if(DebugD == 1):
                    print("B")
                
            elif((Turn_State == 0) and (Speed_In <= 150) and (Con_Rev_Counter == 2)):
                #Turn_State = 0
                #Con_Rev_Counter = Con_Rev_Counter + 1
                Con_Rev_Counter = 0
                if(DebugD == 1):
                    print("C")
        
        # if((F < Red_Zone_Min) and ((F_Left < (2*Red_Zone_Min/3) or (F_Right < (2*Red_Zone_Min/3))))):
        #     Speed = Brake_Speed
                
        if(Speed >= Max_Speed):
            Speed = Max_Speed
            
        if((F > (1.8*Red_Zone_Min) or (F_Left > (1.8*Red_Zone_Min)) or (F_Right > (1.8*Red_Zone_Min))) and (Direction != -1) and (Turn_State > 0) and (Turn_State < 420)):
            Turn_State = 0
            Turn_State_Count = 0
                
        if((Speed > Speed_In) and (Speed_In >= Min_Speed)):
            Red_Zone_Test = round(((((Speed_In + 1) - Min_Speed)*(Red_Zone_Max - Red_Zone_Min))/(Max_Allow_Speed - Min_Speed)) + Red_Zone_Min)
            
            if((Red_Zone_Test < (0.7*F)) and (Speed > (Speed_In+1))):
                Speed = Speed_In + 1

            else:
                Speed = Speed_In
                
    elif(Max_Speed == 150):
        Speed = Brake_Speed
    
    elif(Max_Speed < 150):
        Speed = Max_Speed
    
    else:
        Speed = 150
    
    Side_Lobe_Start = round(Num_Lobes/6)
    Opening_Thres = Opening_Thres_Max
    Opening_Start = Side_Lobe_Start
    Opening_Start_Mag = 0
    Opening_Width = 0
    Opening_Depth = 0
    Opening_End = 0
    
    Turn_Dist = 0
    Turn_Dist_Low_Thres = 0
    Turn_Dist_Max_Thres = 0
    Turn = 0
    Turn2 = 0
    Allignment = 0
    Lobes_Used_for_Avg = 0
    Steering = Centre_Steering
    
    for j in range(Side_Lobe_Start, Num_Lobes - Side_Lobe_Start + 1):
        Average_L += L_Y[j]
        Average_R += R_Y[j]
        Lobes_Used_for_Avg = Lobes_Used_for_Avg  + 1
        
    Average_L = int(Average_L/Lobes_Used_for_Avg)
    Average_R = int(Average_R/Lobes_Used_for_Avg)

    if(Turn_State == 0):
        
        #if(F > 1500):
        if(Average_L >= Average_R or Left_Steer_Flag == 1): 
            Turn_Dist_Low_Thres = Average_L * 1.2
            
        elif(Average_L < Average_R or Right_Steer_Flag == 1): 
            Turn_Dist_Low_Thres = Average_R * 1.2
            
        # else:
        #     Turn_Dist_Low_Thres = (Average_L + Average_R)/4
        #     Opening_Thres = 2*Opening_Thres_Max/3
            
        Turn_Dist_Max_Thres = Turn_Dist_Low_Thres*4
        
        if(DebugD == 1):
            print("Turn_Dist_Low_Thres ", Turn_Dist_Low_Thres)
        if(((Average_L >= (Average_R*Direction)) or (Left_Steer_Flag == 1)) and (F > 2.5*Red_Zone_Min) and Right_Steer_Flag == 0):
            #Left_Steer_Flag = 1
            
            for l in range(Side_Lobe_Start*2, Num_Lobes + 1):
            #for l in range(0, Num_Lobes + 1):
                
                if(L_Y[l] < Turn_Dist_Low_Thres):
                   
                    if(Opening_Start < Opening_End):
                        Opening_Width = (math.cos((Opening_Start + 1)*Angle_Step*math.pi/180)*Opening_Start_Mag) - (math.cos((Opening_End)*Angle_Step*math.pi/180)*L_D[Opening_End])
                    
                    else:
                        Opening_Width = 0
                        Opening_Depth = 0
                        
                    if(Opening_Width > Opening_Thres):
                        Turn = Opening_End
                        Left_Steer_Flag = 1
                        Right_Steer_Flag = 0
                        Steering = Centre_Steering + (Turn*Turn_Step)
                        break
                    else:
                        Opening_Depth = 0
                        
                    Opening_Start = l
                    
                    for m in range(0, (Opening_Start - Side_Lobe_Start+ 1)):
                        
                        if(m == 0):
                            Opening_Start_Mag = L_D[Opening_Start - m]
                            Opening_Start = Opening_Start - m
                            
                        if(L_D[Opening_Start - m - 1] > L_D[Opening_Start - m]):
                            Opening_Start_Mag = L_D[Opening_Start - m]
                            Opening_Start = Opening_Start - m
                            break
                        
                if(L_Y[l] >= Turn_Dist_Low_Thres):
                    
                    if(L_D[l] > Opening_Depth):
                        Opening_Depth = L_D[l]
                        
                        # if(DebugD == 1):
                        #     print('Left Opening Depth: ', Opening_Depth)
                        
                    if(L_Y[l] > Turn_Dist):
                        Turn_Dist = L_Y[l]
                        
                    if((l == Num_Lobes - 1) and (Opening_Depth > Opening_Depth_Min)):
                        Opening_End = Num_Lobes
                        Opening_Width = (math.cos((Opening_Start + 1)*Angle_Step*math.pi/180)*Opening_Start_Mag) - (math.cos((Opening_End)*Angle_Step*math.pi/180)*L_D[Opening_End])
                        
                        if(Opening_Width > Opening_Thres):
                            Turn = Opening_End
                            Left_Steer_Flag = 1
                            Right_Steer_Flag = 0
                            Steering = Centre_Steering + (Turn*Turn_Step)
                            break
                        else:
                            Opening_Depth = 0
                        
                    if(Opening_Depth > Opening_Depth_Min):    
                        Opening_End = l + 1
                    # if(DebugD == 1):
                    #     print("L_Y[l]",L_Y[l],"Turn_Dist_Low_Thres",Turn_Dist_Low_Thres,"Opening_End",Opening_End)
                        
            if(DebugD == 1):
                print("Sharp turn Left.", Steering, "Opening_Width", Opening_Width)
                    
        if(((Average_L <= (Average_R*Direction)) or (Right_Steer_Flag == 1)) and (F > 2.5*Red_Zone_Min) and Left_Steer_Flag == 0):
            #Right_Steer_Flag = 1
            
            for l in range(Side_Lobe_Start*2, Num_Lobes + 1):
            #for l in range(0, Num_Lobes + 1):
                # print("Opening_Start",Opening_Start,"Opening_End",Opening_End)
                if(R_Y[l] < Turn_Dist_Low_Thres):
                    
                    if(Opening_Start < Opening_End):
                        Opening_Width = (math.cos((Opening_Start + 1)*Angle_Step*math.pi/180)*Opening_Start_Mag) - (math.cos((Opening_End)*Angle_Step*math.pi/180)*R_D[Opening_End])
                    
                    else:
                        Opening_Width = 0
                        Opening_Depth = 0
                        
                    if(Opening_Width > Opening_Thres):
                        Turn = Opening_End
                        Left_Steer_Flag = 0
                        Right_Steer_Flag = 1
                        Steering = Centre_Steering - (Turn*Turn_Step)
                        break
                    
                    Opening_Start = l
                    
                    for m in range(0, (Opening_Start - Side_Lobe_Start + 1)):
                        
                        if(m == 0):
                            Opening_Start_Mag = R_D[Opening_Start - m]
                            Opening_Start = Opening_Start - m
                            
                        if(R_D[Opening_Start - m - 1] > R_D[Opening_Start - m]):
                            Opening_Start_Mag = R_D[Opening_Start - m]
                            Opening_Start = Opening_Start - m
                            
                            break
                        
                if(R_Y[l] >= Turn_Dist_Low_Thres):
                    
                    if(R_D[l] > Opening_Depth):
                        Opening_Depth = R_D[l]
                        
                        # if(DebugD == 1):
                        #     print('Right Opening Depth: ', Opening_Depth)
                        
                    if(R_Y[l] > Turn_Dist):
                        Turn_Dist = R_Y[l]
                        
                    if((l == Num_Lobes - 1) and (Opening_Depth > Opening_Depth_Min)):
                        Opening_End = Num_Lobes
                        Opening_Width = (math.cos((Opening_Start + 1)*Angle_Step*math.pi/180)*Opening_Start_Mag) - (math.cos((Opening_End)*Angle_Step*math.pi/180)*R_D[Opening_End])
                    
                        if(Opening_Width > Opening_Thres):
                            Turn = Opening_End
                            Left_Steer_Flag = 0
                            Right_Steer_Flag = 1
                            Steering = Centre_Steering - (Turn*Turn_Step)
                            break
                    if(Opening_Depth > Opening_Depth_Min):    
                        Opening_End = l + 1
                    # if(DebugD == 1):
                    #     print("R_Y[l]",R_Y[l],"Turn_Dist_Low_Thres",Turn_Dist_Low_Thres,"Opening_End",Opening_End)
            if(DebugD == 1):
                print("Sharp turn Right.", Steering, "Opening_Width", Opening_Width)
        Max_Left = 0
        Max_Right = 0
        F_Max_Left = 0
        F_Max_Right = 0
        F_Temp = 0
        
        if((Steering > 135) and (Steering < 165) and (F > (2.5*Red_Zone_Min))):
            Left_Steer_Flag = 0
            Right_Steer_Flag = 0
            F_Avg = 0
            F_Max_Left = 1.05*F
            F_Max_Right = 1.05*F
            # count = 0
            # for n in range(0, int(Num_Lobes/4), 1):
            #     F_Avg = F_Avg +  L_D[n]
            #     F_Avg = F_Avg +  R_D[n]
            #     count = count + 2
            # F_Avg = F_Avg / count    
            Max_Right = 0
            Max_Left = 0 
            F_Temp_L = 0
            F_Temp_R = 0
            count = 0
            
            for n in range(0, int(Num_Lobes - Num_Lobes/3), 1):
                
                if(L_D[n] >= F_Max_Left):
                    F_Temp_L = F_Temp_L +  L_D[n] 
                    Max_Left = n*-1
                    count = count + 1
                    
                else:
                    break
                
            if(count > 0):
                F_Temp_L = F_Temp_L /count
            
            count = 0
            
            for n in range(0, int(Num_Lobes - Num_Lobes/3), 1):
                
                if(R_D[n] >= F_Max_Right):
                    F_Temp_R = F_Temp_R +  R_D[n]
                    Max_Right = n
                    count = count + 1
                    
                else:
                    break
                
            if(count > 0):
                F_Temp_R = F_Temp_R/count
            # if(DebugD == 1):
            #     print("F_Temp_L",F_Temp_L, "F", F, "F_Temp_R", F_Temp_R)
            #     print("Max_Left",Max_Left,"Max_Right",Max_Right)
                
            if(Max_Right == Max_Left):
                if((F_Temp_L > F_Temp_R*1.1)):
                    Max_Left = int(Max_Left * 0.66)
                    Max_Right = int(Max_Right * 0.33)
                    
                if((F_Temp_R > F_Temp_L*1.1)):
                    Max_Right = int(Max_Right * 0.66)
                    Max_Left = int(Max_Left * 0.33)
                    
            # if(DebugD == 1):
            #     print("Max_Left",Max_Left,"Max_Right",Max_Right)
            # if(abs(Max_Left)>Max_Right):   
            #     #Turn2 = int(Max_Left + Max_Right/2)
            #     Turn2 = int(Max_Left)
            # else:
            #     #Turn2 = int(Max_Right + Max_Left/2)
            #     Turn2 = int(Max_Right)
            Turn2 = int((Max_Right + Max_Left))
            Steering = Centre_Steering - (Turn2*Turn_Step)
            
            if(Steering > 164):
                Steering = 164
                
            if(Steering < 136):
                Steering = 136
        
            if(DebugD == 1):
                print("Turn.", Steering)
        if((Steering >= 146) and (Steering <= 154) and (F > (4*Red_Zone_Min)) and Speed >= Speed_In):
            Allignment = 1
            if(DebugD == 1):
                print("Average_L" ,Average_L,"Average_R",Average_R)
            if(Average_R > (Average_L*1.3)):
                Steering = Centre_Steering - 4
                
            # elif(Average_R > (Average_L*1.2)):
            #     Steering = Centre_Steering - 4
                
            elif(Average_R > (Average_L*1.1)):
                Steering = Centre_Steering - 2

            elif(Average_L > (Average_R*1.3)):
                Steering = Centre_Steering + 4
                
            # elif(Average_L > (Average_R*1.2)):
            #     Steering = Centre_Steering + 4
                
            elif(Average_L > (Average_R*1.1)):
                Steering = Centre_Steering + 2
                
            else:
                Steering = Centre_Steering
                
            if(DebugD == 1):
                print("Allignment.", Steering)   
                
        
            
        if((F <= (2.5*Red_Zone_Min)) and (Speed_In >= Brake_Speed)):
            
            if(((Left_Steer_Flag == 1) or (Average_L > Average_R)) and (Right_Steer_Flag == 0)):
                Turn2 = 0
                Turn = Num_Lobes
               
                if(F > (1.8*Red_Zone_Min)):
                    Steering = Left_Steering - 10
                   
                else:
                    Steering = Left_Steering
                    
                Left_Steer_Flag = 1
                
            else:
                Turn2 = 0
                Turn = Num_Lobes
                
                if(F > (1.8*Red_Zone_Min)):
                    Steering = Right_Steering + 10
                    
                else:
                    Steering = Right_Steering
            
                Right_Steer_Flag = 1
            
            
              
            #if(Speed > Min_Speed):
            #    Speed = 125
                
            if(DebugD == 1):
                print("Sharp turn to avoid collision.", Steering)            
        
        if((Turn_Dist > 0) and (Turn_Dist_Max_Thres != Turn_Dist_Low_Thres)):
            Turn_Dist = (Turn_Dist - Turn_Dist_Low_Thres)*100/(Turn_Dist_Max_Thres - Turn_Dist_Low_Thres)
            
        if(Turn_Dist > 50):
            Turn_Dist = 50

    if(Turn_State == 420):  
        Turn_State = Turn_State + 1
        Speed = Brake_Speed
        
        if(Con_Rev_Counter == 2):
            Turn_State = 430
            
        if(((Left_Steer_Flag == 1) or (Average_L > Average_R)) and (Right_Steer_Flag == 0)):
            Steering = Left_Steering
            Left_Steer_Flag = 1
            
        else:
            Steering = Right_Steering
            Right_Steer_Flag = 1
            
    elif((Turn_State <= 435) and (Turn_State > 420)):
        Speed = Brake_Speed
        
        if(Left_Steer_Flag == 1):
            Steering = Left_Steering
            
        else:
            Steering = Right_Steering
            
        if(Turn_State > 430):
            Steering = Centre_Steering
            Speed = 150
            
        Turn_State = Turn_State + 1
        
    elif((Turn_State > 435) and (Turn_State < 455)):
    
        if(Turn_State == 436):
        
            if(Left_Steer_Flag):
                Rev_Steering = Right_Steering
                
            elif(Right_Steer_Flag):
                Rev_Steering = Left_Steering
                
            elif(F_Left > 1.5*F_Right):
                Rev_Steering = Right_Steering
                Left_Steer_Flag = 1
                Right_Steer_Flag = 0
                
            elif(F_Right > 1.5*F_Left):
                Rev_Steering = Left_Steering
                Left_Steer_Flag = 0
                Right_Steer_Flag = 1
                
            else:
                Rev_Steering = Centre_Steering
                Left_Steer_Flag = 0
                Right_Steer_Flag = 0
                
        Steering = Rev_Steering
        Speed = Rev_Speed
        
        if(Turn_State > 450):
            Turn_State = 455;
            Steering = Centre_Steering;
            Speed = 155

        elif((F > (2*Red_Zone_Min)) and (Turn_State < 450)):
            Turn_State = 455
            Steering = Centre_Steering
            Speed = 155
            Left_Steer_Flag = 0
            Right_Steer_Flag = 0
            

        else:
            Turn_State = Turn_State + 1
            
    elif(Turn_State >= 455):
    
        if(((Left_Steer_Flag == 1) or (Average_L > Average_R)) and (Right_Steer_Flag == 0)):
            Turn2 = 0
            Turn = Num_Lobes
            
            if(F > 1.8*Red_Zone_Min):
                Steering = Left_Steering - 10
                
            else:
                Steering = Left_Steering
                
            Opening_right = 0
            Opening_left = 1
            Left_Steer_Flag = 1
            
        else:
            Turn2 = 0
            Turn = Num_Lobes
            
            if(F > 1.8*Red_Zone_Min):
                Steering = Right_Steering + 10
                
            else:
                Steering = Right_Steering
                
            Opening_right = 1
            Opening_left = 0
            Right_Steer_Flag = 1
            
        if(F > 3*Red_Zone_Min):
            Turn_State = 0
            Turn_State_counter = 0
            Left_Steer_Flag = 0
            Right_Steer_Flag = 0
            Auto_Drive_State = 2
            
        else:
            Turn_State = Turn_State + 1
        
        if((F>(2*Red_Zone_Min/3)) or (F_Left>(2*Red_Zone_Min/3)) or (F_Right>(2*Red_Zone_Min/3))):
            Speed = Min_Speed
            Auto_Drive_State = 2
            
        else:
            Speed = Brake_Speed
            Turn_State = 0
            Turn_State_counter = 0
            Left_Steer_Flag = 0
            Right_Steer_Flag = 0
            
    if(DebugD == 1):
        print('Turn DIst', Turn_Dist)
        
    if((Speed_In > (Min_Speed + 1)) and (Turn_Dist > 20) and (Turn_State == 0)):
        Turn_Reduction_Speed = int(((Speed_In - Min_Speed)*(50 - Turn_Dist))/50) + Min_Speed
        #Stutter start issue begins here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        if(Turn_Reduction_Speed <= (Min_Speed + 1)):
            Turn_Reduction_Speed = Min_Speed + 1
            
        Speed = Turn_Reduction_Speed

    if(DebugD == 1):
        print('Test: Speed',Speed,'Speed_In',Speed_In,'Turn_State',Turn_State)
        
    if((Speed <= 150) and (Turn_State == 0)):
        Speed_Out = Speed

    elif((Speed_In < Min_Speed) and (Speed >= Min_Speed) and (Turn_State == 0)):
        Speed_Out = Min_Speed

    elif(Speed >= Speed_In) and (Speed_In >= Min_Speed) and (Turn_State == 0):
        Speed_Out = Speed
        
    #elif((Speed < Speed_In) and (Speed >= Min_Speed) and (Turn_State == 0)):
    #    Speed_Out = round(Partial_Brake_Offset - 2*(Speed_In - Speed) - 5*((Speed_In/5) - 30))
    #    print("My_Speed_Out:",Speed_Out,"Speed_In:",Speed_In,"Speed:",Speed)

    else:
        Speed_Out = Speed

    if(Speed_Out < Brake_Speed):
        Speed_Out = Brake_Speed
    
    if(DebugD == 1):
        print('Speed_out : ', Speed_Out)
        print('Speed_In', Speed_In) 
        
    if(Auto_Drive_State == 0):
        Auto_Drive_State = 1
        Speed_Out = 155
        
    elif((Speed_Out < 145) and ((Auto_Drive_State == 1) or (Auto_Drive_State == 2)) and (Turn_State < 5)):
        Speed_Out = Speed_Out
        Auto_Drive_State = 2
        
    elif((Speed_Out < 145) and (Auto_Drive_State == 3) and (Turn_State < 5)):
        Speed_Out = 150
        Auto_Drive_State = 3
        
    elif((Speed_Out < 145) and ((Auto_Drive_State == 3) or (Auto_Drive_State == 4)) and (Turn_State >= 5)):
        Speed_Out = Rev_Speed
        Auto_Drive_State = 4
        
    elif((Speed_Out >= 145) and (Speed_Out <= 155)):
        
        if((Auto_Drive_State == 1) and (Turn_State == 0)):
            Auto_Drive_State = 2 
            Speed_Out = Brake_Speed

        elif((Auto_Drive_State == 1) and (Turn_State > 0)):
            Auto_Drive_State = 3
            Speed_Out = 150
        
        elif((Auto_Drive_State == 2) and (Turn_State == 0)):
            Auto_Drive_State = 2
            Speed_Out = Brake_Speed

        elif((Auto_Drive_State == 2) and (Turn_State > 1)):
            Auto_Drive_State = 3
            Speed_Out = 150
            
        elif((Auto_Drive_State == 3) or (Auto_Drive_State == 4)):
            Auto_Drive_State = 3
            Speed_Out = 150
            
    elif((Speed_In > 150) and (Direction == -1)):
        Auto_Drive_State = 3
        Speed_Out = 150

    if((Speed_In >= Brake_Speed) and (Speed_Out >= Brake_Speed) and (Turn_State == 0)):

        if((Speed_In < 105) and (Speed_Out <= Partial_Brake_Offset)):
            Speed_In = Brake_Speed

        elif((Speed_Out <= Partial_Brake_Offset) and (Speed_Out > Brake_Speed)):
            Speed_In = Speed_In -1

        elif((Speed_Out == Brake_Speed) and (Speed_In > 140)):
            Speed_In = Speed_In - 1

        else:
            Speed_In = Speed_Out

    else:
        Speed_In = Speed_Out

    if(Speed_In <= Normal_Speed):
        Steer_Delta = 6
        
    elif(Speed_In <= 166):
        Steer_Delta = 8
        
    elif(Speed_In <= 170):
        Steer_Delta = 10
        
    else:
        Steer_Delta = 12

    if((Steering > 120) and (Steering < 180) and (Previous_Steer > 110) and (Previous_Steer < 190)):
        #Change made here!!!!!!!!!!!!!!!!
        
        if(Steering > (Previous_Steer + Steer_Delta)):
            Steering = Previous_Steer + Steer_Delta
            
        elif(Steering < (Previous_Steer - Steer_Delta)):
             Steering = Previous_Steer - Steer_Delta 
        
    Lidar_Speed = int(Speed_Out)
    Lidar_Steering = int(Steering)
    Previous_Steer = Lidar_Steering
    
    if(DebugD == 1):
        print('============================================================')
        
    return (Lidar_Speed, Lidar_Steering)

#=============================================================================

if(Debug == 1):
    
    if(os.path.exists(Scan_Path) == False):
        print('[Error!] There is no Scan File Directory')
        sys.exit()
        
    if(len(os.listdir(Scan_Path)) == 0):
        print('[Error!] There are no Scan Files in Directory')
        sys.exit()
    
    Scan_File_Array_List = []
    Start_Pause = False
    Draw_Scale = 140/float(URG_Distance_Range)

    for Scan_Files in os.listdir(Scan_Path):
        Search_Folder = os.path.join(Scan_Path, Scan_Files)
        
        for Scans_Folders in os.listdir(Search_Folder):
            Scan = np.load(os.path.join(Search_Folder, Scans_Folders), allow_pickle=True)
            Scan[Scan<=20] = 5600
            Scan_File_Array_List.append(Scan)
            
    for i in range(Offset, int(len(Scan_File_Array_List))):
        Current_Scan = Scan_File_Array_List[i]
        (Speed, Steering) = LIDARBot(Current_Scan)
        image = Image.new(mode = 'RGB', size = (480, 280), color=(0, 0, 0))
        Draw = ImageDraw.Draw(image)
        
        for j in range(len(Current_Scan)):
    
            if((Current_Scan[j] <= 20) and (Start_Pause == False)):
                Current_Scan[j] = 5600
                            
            X = 240 - (-math.cos(Draw_Angles[j])*Draw_Scale*Current_Scan[j])
            Y = 140 - ( math.sin(Draw_Angles[j])*Draw_Scale*Current_Scan[j])
            Draw.line((240, 140, X, Y), fill=(255, 255, 255))

        Draw.polygon([(240, 140), (235, 155), (245, 155)], fill = (255, 0, 0))
        
        Steer_Ang = np.deg2rad(90 - (150 - Steering))
        X1 = 240 - (-math.cos(Steer_Ang)*Draw_Scale*(5600*((Speed - 158)/27)))
        Y1 = 140 - (math.sin(Steer_Ang)*Draw_Scale*(5600*((Speed - 158)/27)))
        Draw.line((240, 140, X1, Y1), fill=(255, 0, 0))
        
        image = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
        cv2.imshow('Debugging Mode', image)
        K = cv2.waitKey(0)
        print(i)
        
        if(K == 27):
            cv2.destroyAllWindows()
            sys.exit()
            
        time.sleep(Delay)
        
        if(i == (int(len(Scan_File_Array_List)) - 1)):
            cv2.destroyAllWindows()
            time.sleep(2)
            sys.exit()

#=============================================================================

if(os.path.exists(Data_Path) == False):
    os.makedirs(Data_Path)
    os.system('sudo chmod 777 -R ' + Data_Path)

if(os.path.exists(Image_Path) == False):
    os.makedirs(Image_Path)
    os.system('sudo chmod 777 -R ' + Image_Path)
    
if(os.path.exists(Edge_Path) == False):
    os.makedirs(Edge_Path)
    os.system('sudo chmod 777 -R ' + Edge_Path)

if(os.path.exists(Scan_Path) == False):
    os.makedirs(Scan_Path)
    os.system('sudo chmod 777 -R ' + Scan_Path)

if(os.path.exists(CNN_Model_Path) == False):
    os.makedirs(CNN_Model_Path)
    os.system('sudo chmod 777 -R ' + CNN_Model_Path)
    
if(os.path.exists(Train_Image_Path) == False):
    os.makedirs(Train_Image_Path)
    os.system('sudo chmod 777 -R ' + Train_Image_Path)
    
if(os.path.exists(Test_Image_Path) == False):
    os.makedirs(Test_Image_Path)
    os.system('sudo chmod 777 -R ' + Test_Image_Path)
    
if(os.path.exists(Training_Path) == False):
    os.makedirs(Training_Path)
    os.system('sudo chmod 777 -R ' + Training_Path)
    
if(os.path.exists(Validation_Path) == False):
    os.makedirs(Validation_Path)
    os.system('sudo chmod 777 -R ' + Validation_Path)
    
if(os.path.exists(Testing_Path) == False):
    os.makedirs(Testing_Path)
    os.system('sudo chmod 777 -R ' + Testing_Path)
    
Model_Name = '-'  
Edge_Model_Name = '-'  
Lidar_Exists = '-'
Cruise_Brake = False
Controller_Exit = False

from breezylidar import URG04LX as LIDAR
from breezyslam.sensors import Laser
from breezyslam.sensors import URG04LX as SLAM
from breezyslam.algorithms import RMHC_SLAM
                  
    #not working, need to manually install:
    #cd BreezyLidar_Path 
    #sudo python3 setup.py install
    #cd BreezySLAM_Path 
    #sudo python3 setup.py install

#=========================================================================

if(SIMULATION == True):

    def Simulation_Drive(Blaster_Speed, Blaster_Steering):
        Simulation_Speed = int((2/5)*Blaster_Speed - 60)   
        Simulation_Steering = int(((51/20)*Blaster_Steering) - 255)
        MOTORDrive(1, Simulation_Speed)
        SERVOSet(1, Simulation_Steering)
    
#=============================================================================

pygame.init()

if(pygame.joystick.get_count() < 1):
    LCDClear()
    LCDMenu('Start', '', '', 'Exit')
    LCDSetPrintf(16, 0, '[Notice!] There is no Controller Connected')
    LCDSetPrintf(18, 0, 'Do You Wish to Continue?')
    Controller_Connected = False
    Joystick_Connected = 'None'
                        
    while(True):
        KEY =  KEYRead()
                        
        if(KEY == 1):
            break
            
        elif(KEY == 8):
            Controller_Exit = True
            break
            
    def A_Button():
        return False

    def B_Button():
        return False

    def X_Button():
        return False

    def Y_Button():
        return False
            
    def UP_Button():
        return False 
            
    def DOWN_Button():
        return False

    def LEFT_Button():
        return False 
            
    def RIGHT_Button():
        return False

    def L1_Button():
        return False

    def R1_Button():
        return False        

    def Steering_Control():
        return 150

    def Speed_Control():
        global Cruise_Brake
        Cruise_Brake = False
        return 150

else:
    pygame.joystick.Joystick(0).init()
    Controller_Connected = True
    Controller_Decline = False
    Joystick_Connected = pygame.joystick.Joystick(0).get_name()
    
    def A_Button():
        
        if(pygame.joystick.Joystick(0).get_button(0) == 1):
            return True

        else:
            return False

    def B_Button():

        if(pygame.joystick.Joystick(0).get_button(1) == 1):
            return True

        else:
            return False

    def X_Button():

        if(pygame.joystick.Joystick(0).get_button(2) == 1):
            return True

        else:
            return False

    def Y_Button():

        if(pygame.joystick.Joystick(0).get_button(3) == 1):
            return True

        else:
            return False
            
    def UP_Button():

        if(pygame.joystick.Joystick(0).get_hat(0) == (0, 1)):
            return True

        else:
            return False  
            
    def DOWN_Button():

        if(pygame.joystick.Joystick(0).get_hat(0) == (0, -1)):
            return True

        else:
            return False

    def LEFT_Button():

        if(pygame.joystick.Joystick(0).get_hat(0) == (-1, 0)):
            return True

        else:
            return False  
            
    def RIGHT_Button():

        if(pygame.joystick.Joystick(0).get_hat(0) == (1, 0)):
            return True

        else:
            return False

    def L1_Button():

        if(pygame.joystick.Joystick(0).get_button(4) == 1):
            return True

        else:
            return False

    def R1_Button():

        if(pygame.joystick.Joystick(0).get_button(5) == 1):
            return True

        else:
            return False        

    def Steering_Control():
        pygame.event.pump()
        return int((pygame.joystick.Joystick(0).get_axis(0)*-50) + 150)

    def Speed_Control():
        global Cruise_Brake
        
        pygame.event.pump()
        Throttle = pygame.joystick.Joystick(0).get_axis(5)
        Brake = pygame.joystick.Joystick(0).get_axis(2)

        if((Throttle > -0.98) and (Brake < -0.98)):
            Cruise_Brake = False
            return int((pygame.joystick.Joystick(0).get_axis(5)*12.5) + 162.5)

        elif((Brake > -0.98) and (Throttle < -0.98)):
            Cruise_Brake = True
            return int((pygame.joystick.Joystick(0).get_axis(2)*-10) + 140)

        elif((Brake > -0.98) and (Throttle > -0.98)):
            Cruise_Brake = True
            return int((pygame.joystick.Joystick(0).get_axis(2)*-25) + 125)

        else:
            Cruise_Brake = False
            return 150
            
#=============================================================================

def IP_Address():
    S = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        S.connect(('1.1.1.1', 80))
        return S.getsockname()[0]

    except:
        return '-'   
     
#=============================================================================

if SIMULATION:
    def Image_Processing(image):
        image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
        #image = cv2.GaussianBlur(image, (3, 3), 0)
        #image = cv2.resize(image, (230, 66))
        
        return image 
else:   #reality
    def Image_Processing(image):
        image = image[60:165, :, :]
        #105,320
        #105,280
        image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
        #image = cv2.GaussianBlur(image, (3, 3), 0)
        #image = cv2.resize(image, (230, 66))
        image = image[:, 25:295, :]
        return image    
#=============================================================================

if(SIMULATION == False):
    #os.system(user + Servo_Path)
    ServoBlaster = open('/dev/servoblaster', 'w')

#=============================================================================
   
CAM =  CAMInit(1)

if(CAM != 0):
    LCDClear()
    LCDMenu('', '', '', 'Exit')
    LCDSetPrintf(16, 0, '[Error!] No Cameras are Connected')
                        
    while(True):
        pygame.event.pump()
        KEY =  KEYRead()
                        
        if((KEY == 8) or (X_Button() == True)):
            Controller_Exit = True
            break
       
#=============================================================================

Switch_Mode = 0

Camera_Drive_Check = False
Lidar_Drive_Check = False
Exit_Drive_Check = True

Model_Change = False
Keras_Loaded = False
Edge_Model_Loaded = False

Manual = []
Nvidia_Camera = []
Edges_Camera = []
Lidar = []

Manual_Display_Modes = ['', 'COL ', 'EDGE']
Lidar_Display_Modes = ['', 'SCAN', 'MAP ', 'SLAM', 'CAM ', 'EDGE']

Image_Count = 0
Edge_Count = 0
Scan_Count = 0

if(SIMULATION == False):
    SSID = 'robot'
    PASS = 'raspberry'
    Display_Line_X = 0
    Display_Line_Y = 0
    Display_Line_Extra = 0
    
else:
    SSID = '-'
    PASS = '-'
    Display_Line_X = 2
    Display_Line_Y = 22
    Display_Line_Extra = 1

#=============================================================================

while(True):
    Image_Set = (len(os.listdir(Image_Path)) - 2)
    Edge_Set = (len(os.listdir(Edge_Path)) - 3) 
    Scan_Set = (len(os.listdir(Scan_Path)))

    if(os.path.exists(Urg_Path) == True):
        Lidar_State = Lidar_Exists
        Lidar_Exists = 'Hokuyo URG-04LX'
        
        if(Lidar_State != Lidar_Exists):
            Change = True
            
        else:
            Change = False

    else:
        Lidar_State = Lidar_Exists
        Lidar_Exists = '-'
        
        if(Lidar_State != Lidar_Exists):
            Change = True
            
        else:   
            Change = False
            
        if(SIMULATION == True):
            Lidar_Exists = 'EyeSim'
            Change = False

    if((Exit_Drive_Check == True) or (Change == True)):   
        LCDClear()
        LCDMenu('Manual', 'Camera', 'Lidar', 'Exit')
        LCDSetColor(16777215, 0)
        LCDSetPrintf(1, 0,  'Welcome to the ModelCar-2   [ 7]')
        LCDSetPrintf(4, 0,  'Hostname     :')
        LCDSetPrintf(5, 0,  'IP Address   :')
        LCDSetPrintf(6, 0,  'WIFI SSID    :')
        LCDSetPrintf(7, 0,  'WIFI PASS    :')
        LCDSetPrintf(9, 0,  'Nvidia Model :')
        LCDSetPrintf(10, 0, 'Edge Model   :')
        LCDSetPrintf(11, 0, 'LIDAR        :')
        LCDSetPrintf(13, 0, 'Image Sets   :')
        LCDSetPrintf(14, 0, 'Scan Sets    :')
        LCDSetPrintf(15, 0, 'Edge Sets    :')
        LCDSetColor(16776960, 0)
        LCDSetPrintf(4, 15,  socket.gethostname())
        LCDSetPrintf(5, 15,  IP_Address())
        LCDSetPrintf(6, 15,  SSID)
        LCDSetPrintf(7, 15,  PASS)
        LCDSetPrintf(9, 15,  Model_Name)
        LCDSetPrintf(10, 15, Edge_Model_Name)
        LCDSetPrintf(11, 15, Lidar_Exists)
        LCDSetPrintf(13, 15, str(Image_Set))
        LCDSetPrintf(14, 15, str(Edge_Set))
        LCDSetPrintf(15, 15, str(Scan_Set))
        Exit_Drive_Check = False
        
    KEY =  KEYRead()
    pygame.event.pump() 
    Using_Edge_Mode = False
    
#=============================================================================

    if((KEY == 1) or (B_Button() == True) or (Switch_Mode == 1)):
        
        if(Controller_Connected == False):
            LCDClear()
            LCDMenu('Ok', '', '', 'Exit')
            LCDSetPrintf(16, 0, '[Notice!] There is no Controller Connected')
            LCDSetPrintf(18, 0, 'Some Features May be Disabled')
            
            while(True):
                KEY =  KEYRead()
                            
                if(KEY == 1):
                    Controller_Decline = False
                    break
                
                elif(KEY == 8):
                    Controller_Decline = True
                    break
        
        Switch_Mode = 0
        LCDClear()
        LCDMenu('Record', 'Cruise', 'Display', 'Back')
        LCDSetColor(16777215, 0)
        LCDSetPrintf(1, 37 + Display_Line_Y, 'Manual Drive')
        LCDImageStart(0, 0, 320, 240)
        
        Recording_Data = -1
        Record_Button = 9
        Cruise_Control = -1
        Cruise_Button = 9
        Cruise_Brake_Speed = 0
        New_Pause = False
        New_Cruise = False
        New_Uncruise = True
        New_Recording = False
        New_Unrecording = True
        Hard_Pause_Brake = False
        
        Speed_Past = 150
        Display = 1
        Display_Button = 9
        Iterations = 0
        time.sleep(0.2)
        Start_FPS = time.time()

        while(True):
            Start = time.time()
            img =  CAMGet()
            image = np.array(img, dtype='uint8')
            image = np.reshape(image, (240, 320, 3))
            KEY =  KEYRead()
            pygame.event.pump()
            
            if((X_Button() == True) or (KEY == 8) or (Controller_Decline == True)):
                Switch_Mode = 0
                break
                
            if(L1_Button() == True):
                Switch_Mode = 3
                break
                
            if(R1_Button() == True):
                Switch_Mode = 2
                break
                
            Steering_Angle = Steering_Control()
            Speed = Speed_Control()
            
            if((Cruise_Brake == True) and (Cruise_Control == 1)):
                Cruise_Control = -1
                New_Pause = True
                New_Uncruise = True
            
            if((KEY == 1) or ((B_Button() == True) and ((time.time() - Record_Button) > 0.2))):
                Recording_Data = Recording_Data*-1
                Record_Button = time.time()

                if(Recording_Data == 1):
                    New_Recording = True
                    
                    Now = str(datetime.datetime.now())
                    Image_Stamp = 'Manual_Image_' + Stamp_Type + Now[:-7]
                    New_Image_Save_Path = os.path.join(Image_Path, Image_Stamp)
                    os.makedirs(New_Image_Save_Path)
                    Image_Count = 0

                else:
                    New_Unrecording = True
                
            if((KEY == 2) or ((A_Button() == True) and ((time.time() - Cruise_Button) > 0.2))):
                Cruise_Control = Cruise_Control*-1
                Cruise_Button = time.time()
                Cruise_Speed = Speed
                
                if(Cruise_Control == -1):
                    New_Pause = True
                    New_Uncruise = True

                else:
                    New_Cruise = True

            if((KEY == 4) or ((Y_Button() == True) and ((time.time() - Display_Button) > 0.2))):
                Display += 1
                Display_Button = time.time()

                if(Display == 3):
                    Display = 1
                    
                if(Display == 2):
                    Recording_Data = -1
                    New_Unrecording = True
                
            if(Cruise_Control == 1):
                Speed = Cruise_Speed

                if(New_Cruise == True):
                    LCDCircle(400, 180, 50, 65280, 1)
                    LCDSetColor(16777215, 0)
                    LCDSetPrintf(14 + Display_Line_X, 36 + Display_Line_Y, 'Cruise Control')
                    New_Cruise = False
            
            else:

                if(New_Uncruise == True):
                    LCDCircle(400, 180, 50, 0, 1)
                    LCDSetColor(0, 0)
                    LCDSetPrintf(14 + Display_Line_X, 36 + Display_Line_Y, 'Cruise Control')
                    New_Uncruise = False
            
            if(New_Pause == True):
                New_Pause = False
                            
                if(Speed_Past > Normal_Speed):
                    Hard_Pause_Brake = True
                                
                else:
                    Hard_Pause_Brake = False
                            
            if((Hard_Pause_Brake == True) and (Iterations > 25)):
                            
                if(SIMULATION == False):
                    Steering_Angle = 150
                    Speed = Brake_Speed
                    ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                    ServoBlaster.flush()
                    ServoBlaster.write('1=' + str(Speed) + '\n')
                    ServoBlaster.flush()
                    time.sleep(Hard_Brake_Power)
                    Hard_Pause_Brake = False
                    
                else:
                    Steering_Angle = 150
                    Speed = 150
                    Simulation_Drive(Speed, Steering_Angle)
                   
                Recording_Data = -1
            
            else: 
                
                if(SIMULATION == False):
                    ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                    ServoBlaster.flush()
                    ServoBlaster.write('1=' + str(Speed) + '\n')
                    ServoBlaster.flush()            
                else:
                    Simulation_Drive(Speed, Steering_Angle)
                    
                New_Pause_Iteration = 0
                
            if(Display == 1):
                LCDImage(img)
            
            elif(Display == 2):
                Canny_Image = cv2.Canny(image, 150, 200)
                Canny_Image = (Canny_Image.ctypes.data_as(ctypes.POINTER(ctypes.c_byte)))
                LCDImageGray(Canny_Image)

            if(Recording_Data == 1):
                Image_Array = [image, Speed, Steering_Angle]
                Image_Filename = 'Manual_Image_' + str(Image_Count) + '_' + str(Speed) + '_' + str(Steering_Angle)
                np.save(os.path.join(New_Image_Save_Path, Image_Filename), np.array(Image_Array))
                Image_Count += 1
                
                Manual.append(time.time() - Start)
                Display = 1

                if(New_Recording == True):
                    LCDCircle(400, 80, 50, 16711680, 1)
                    LCDSetColor(16777215, 0)
                    LCDSetPrintf(7 + Display_Line_X, 36 + Display_Line_Y, 'Recording Data')
                    New_Recording = False

            else:

                if(New_Unrecording == True):
                    LCDCircle(400, 80, 50, 0, 1)
                    LCDSetColor(0, 0)
                    LCDSetPrintf(7 + Display_Line_X, 36 + Display_Line_Y, 'Recording Data')
                    New_Unrecording = False

            LCDSetColor(16777215, 0)
            LCDSetPrintf(16 + Display_Line_X + Display_Line_Extra, 0, "Steering Angle: %d    Speed: %d    Display: %s" %(Steering_Angle, Speed, Manual_Display_Modes[Display]))
            LCDSetPrintf(17 + Display_Line_X + Display_Line_Extra, 0, 'Joystick Connected: %s' %(Joystick_Connected))
                
            Iterations += 1
            Speed_Past = Speed
            
        if(SIMULATION == False):
            
            if(Speed > Normal_Speed):
                ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                ServoBlaster.flush()
                ServoBlaster.write('1=' + str(Brake_Speed) + '\n')
                ServoBlaster.flush()
                time.sleep(Hard_Brake_Power)
            ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
            ServoBlaster.flush()
            ServoBlaster.write('1=' + str(150) + '\n')
            ServoBlaster.flush()    
            
        else:
            Simulation_Drive(150, 150)
        
        if(Switch_Mode == 0):
            End_FPS = time.time()
            Manual_FPS = float(Iterations/(End_FPS - Start_FPS))
            
            LCDClear()
            LCDMenu('Continue', '', '', '')
            LCDSetPrintf(13, 0, 'Manual Drive')
            LCDSetPrintf(15, 0, '[ModelCar-2] Frames/Second : %.2f' %(Manual_FPS))
                            
            while(True):
                pygame.event.pump()
                KEY =  KEYRead()
                            
                if((KEY == 1) or (B_Button() == True)):
                    break
        
        time.sleep(0.2)
        Exit_Drive_Check = True
              
#=============================================================================

    elif((KEY == 2) or (A_Button() == True) or (Model_Change == True) or (Switch_Mode == 2)):
        Switch_Mode = 0
        time.sleep(0.2)
    
        if(True):
            LCDClear()
            LCDMenu('NVIDIA', 'Edge', '', 'Back')
            LCDSetColor(16777215, 0)
            LCDSetPrintf(16, 0, '[ModelCar-2] Choose Which Neural Network to Use?')
                    
            while(True):
                pygame.event.pump()
                KEY =  KEYRead()

                if((KEY == 1) or (B_Button() == True) or (Model_Change == True)):
                
                    if(Camera_Drive_Check == True):
                        break
                        
                    Models = ['', '', '', '', '', '']

                    for Filename in os.listdir(CNN_Model_Path):

                        if(Filename[-3:] == '.h5'):
                            Models.append(Filename)

                    if(len(Models) < 7):
                        LCDClear()
                        LCDMenu('', '', '', 'Back')
                        LCDSetPrintf(16, 0, '[Error!] There are No Nvidia Models in Directory')
                        Camera_Drive_Check = False
                        Switch_Mode = 0
                                
                        while(True):
                            pygame.event.pump()
                            KEY =  KEYRead()
                                
                            if((KEY == 8) or (X_Button() == True)):
                                break
                                
                        break
                                
                    LCDClear()
                    LCDMenu('Select', 'Down', 'Up', 'Back')
                    LCDSetPrintf(2, 0, '[Keras] Please Choose a Model')
                    Selector = 6
                    Select_Change = True

                    if(len(Models) > 17):
                        Cutoff = 17

                    elif(len(Models) < 17):
                        Cutoff = len(Models)

                    time.sleep(0.2)
                            
                    while(True):
                        pygame.event.pump()
                        KEY =  KEYRead()

                        if(Select_Change == True):
                            Select_Change = False
                            
                            for i in range(6, Cutoff):
                                        
                                if(Selector == i):
                                    LCDSetColor(16776960, 0)
                                    LCDSetPrintf(i, 0, '%d: %s' %(i - 5, Models[i]))
                                            
                                else:
                                    LCDSetColor(16777215, 0)
                                    LCDSetPrintf(i, 0, '%d: %s' %(i - 5, Models[i]))

                        if((KEY == 1) or (B_Button() == True)):
                            LCDClear()
                            LCDSetColor(16777215, 0)
                            LCDSetPrintf(16, 0, '[Keras] Loading Model...')
                            
                            Model_Path = os.path.join(CNN_Model_Path, Models[Selector])
                            Model_Name = Models[Selector]

                            if('LSTM_S3' in Model_Name):
                                sequence_len=3        
                            if('Speed' not in Model_Name):
                                Use_Speed = False
                                
                            else:
                                Use_Speed = True
                            
                            if(Keras_Loaded == False):
                                
                                try:
                                    from tensorflow.keras.models import load_model
                      
                                except ImportError:
                                    os.system('sudo -H pip3 install keras')
        
                                    try:
                                        from tensorflow.keras.models import load_model
                      
                                    except ImportError:
                                        LCDClear()
                                        LCDMenu('', '', '', 'Exit')
                                        LCDSetPrintf(16, 0, '[Error!] Keras cant be imported')
                                        LCDSetPrintf(17, 0, 'Check Installation')
                                                        
                                        while(True):
                                            pygame.event.pump()
                                            KEY =  KEYRead()
                                    
                                            if((KEY == 8) or (X_Button() == True)):
                                                break
                                            
                                        sys.exit()

                                Keras_Loaded = True
                                
                            model = load_model(Model_Path)
                            
                            LCDClear()
                            LCDSetColor(16777215, 0)
                            LCDMenu('Start', '', '', '')
                            LCDSetPrintf(16, 0, '[Keras] Nvidia Model Successfully Loaded')
                            Camera_Drive_Check = True
                                
                            while(True):
                                pygame.event.pump()
                                KEY =  KEYRead()
                                
                                if((KEY == 1) or (B_Button() == True)):
                                    break
                                
                            break
                                    
                        elif((KEY == 2) or (A_Button() == True) or (DOWN_Button() == True)):
                            Selector += 1
                                    
                            if(Selector >= Cutoff - 1):
                                Selector = Cutoff - 1
                                        
                            Select_Change = True
                            time.sleep(0.2)
                                    
                        elif((KEY == 4) or (Y_Button() == True) or (UP_Button() == True)):
                            Selector -= 1
                                    
                            if(Selector <= 6):
                                Selector = 6

                            Select_Change = True            
                            time.sleep(0.2)
                                        
                        elif((KEY == 8) or (X_Button() == True)):
                        
                            if(Model_Change == True):
                                Camera_Drive_Check = True
                        
                            else:
                                Camera_Drive_Check = False
                                
                            break                            
                            
                    break
                    
#=============================================================================
                
                elif((KEY == 2) or (A_Button() == True)):
                    
                    if(Edge_Model_Loaded == False):
                        Edge_Model_Exists = 0
                        
                        for DNN_Models in os.listdir(CNN_Model_Path):
                            
                            if(DNN_Models[-5:] == '00001'):
                                DNN_Edge_Model = os.path.join(CNN_Model_Path, DNN_Models)
                                Edge_Model_Exists += 1
                                
                        if(Edge_Model_Exists == 0):
                            LCDClear()
                            LCDMenu('', '', '', 'Back')
                            LCDSetPrintf(16, 0, '[Error!] There are No Edge Models in Directory')
                            Edge_Model_Loaded = False
                            
                            while(True):
                                pygame.event.pump()
                                KEY =  KEYRead()
                                
                                if((KEY == 8) or (X_Button() == True)):
                                    break
                        else:    
                            LCDClear()
                            LCDSetColor(16777215, 0)
                            LCDSetPrintf(16, 0, '[TensorFlow] Loading Model...')
                            
                            try:
                                import tensorflow as tf 
                  
                            except ImportError:
                                os.system('sudo -H pip3 install tensorflow==1.13.1')
    
                                try:
                                    import tensorflow as tf
                  
                                except ImportError:
                                    LCDClear()
                                    LCDMenu('', '', '', 'Exit')
                                    LCDSetPrintf(16, 0, '[Error!] TensorFlow cant be imported')
                                    LCDSetPrintf(17, 0, 'Check Installation')
                                                    
                                    while(True):
                                        pygame.event.pump()
                                        KEY =  KEYRead()
                                
                                        if((KEY == 8) or (X_Button() == True)):
                                            break
                                        
                                    sys.exit()
                                    
                            try:
                                import tflearn
                                from tflearn.layers.conv import conv_2d, max_pool_2d
                                from tflearn.layers.core import input_data, dropout, fully_connected
                                from tflearn.layers.estimator import regression
                                from tflearn.layers.merge_ops import merge
                  
                            except ImportError:
                                os.system('sudo -H pip3 install tflearn')
    
                                try:
                                    import tflearn
                                    from tflearn.layers.conv import conv_2d, max_pool_2d
                                    from tflearn.layers.core import input_data, dropout, fully_connected
                                    from tflearn.layers.estimator import regression
                                    from tflearn.layers.merge_ops import merge
                  
                                except ImportError:
                                    LCDClear()
                                    LCDMenu('', '', '', 'Exit')
                                    LCDSetPrintf(16, 0, '[Error!] TFLearn cant be imported')
                                    LCDSetPrintf(17, 0, 'Check Installation')
                                                    
                                    while(True):
                                        pygame.event.pump()
                                        KEY =  KEYRead()
                                
                                        if((KEY == 8) or (X_Button() == True)):
                                            break
                                        
                                    sys.exit()
                        
#=============================================================================
                              
                            max_pooling = 2
                            base_mul = 4
                            filter_size = 4
                            
                            IMG_SIZE_X = 16
                            IMG_SIZE_Y = 4
                            layers = 1
                            
                            LR = 1e-4
                            
#=============================================================================
                            
                            Steer_val = []
                            Speed_val = []
                            
                            Steer_val.append(100)
                            
                            for j in range(45):
                                Steer_val.append((j*2) + 106)
                                
                            Steer_val.append(200)
                            
                            for j in range(6):
                                Speed_val.append(100 + (j*5))
                                
                            Speed_val.append(143)
                            Speed_val.append(150)
                            Speed_val.append(155)
                            
                            for j in range(18):
                                Speed_val.append(j+ 158)
                            
                            classes_Steer = len(Steer_val)
                            classes_Speed = len(Speed_val)
                            
#=============================================================================
      
                            tf.reset_default_graph()
                            
                            Ang = input_data(shape=[None, IMG_SIZE_X,IMG_SIZE_Y,1], name='Ang')
                            Mag = input_data(shape=[None, IMG_SIZE_X,IMG_SIZE_Y,1], name='Mag')
                            Count = input_data(shape=[None, IMG_SIZE_X,IMG_SIZE_Y,1], name='Count')
                            Speed_In = input_data(shape=[None, classes_Speed], name='Speed_In')
                            
                            controls = fully_connected(Speed_In, 300, activation='relu')
                            controls = dropout(controls, 0.6)
                            
                            convnetAng = conv_2d(Ang, base_mul*3, filter_size, activation='relu')
                            convnetAng = max_pool_2d(convnetAng, max_pooling)
                            
                            convnetAng = conv_2d(convnetAng, base_mul*4, filter_size, activation='relu')
                            convnetAng = max_pool_2d(convnetAng, max_pooling)
                            
                            convnetAng = fully_connected(convnetAng, 300, activation='relu')
                            convnetAng = dropout(convnetAng, 0.6)
                            
                            convnetMag = conv_2d(Mag, base_mul*3, filter_size, activation='relu')
                            convnetMag = max_pool_2d(convnetMag, max_pooling)
                            
                            convnetMag = conv_2d(convnetMag, base_mul*4, filter_size, activation='relu')
                            convnetMag = max_pool_2d(convnetMag, max_pooling)
                            
                            convnetMag = fully_connected(convnetMag, 300, activation='relu')
                            convnetMag = dropout(convnetMag, 0.6)
                            
                            convnetCount = conv_2d(Count, base_mul*3, filter_size, activation='relu')
                            convnetCount = max_pool_2d(convnetCount, max_pooling)
                            
                            convnetCount = conv_2d(convnetCount, base_mul*4, filter_size, activation='relu')
                            convnetCount = max_pool_2d(convnetCount, max_pooling)
                            
                            convnetCount = fully_connected(convnetCount, 300, activation='relu')
                            convnetCount = dropout(convnetCount, 0.6)
                            
                            convnet = merge([convnetAng, convnetMag, convnetCount, controls], 'concat')
                            
                            convnet = fully_connected(convnet, 100, activation='relu')
                            
                            convnetSp = fully_connected(convnet, classes_Speed, activation='softmax')
                            convnetSp = regression(convnetSp, optimizer='adam', learning_rate=LR, loss='categorical_crossentropy', name='targetsSp')
                            
                            convnetSt = fully_connected(convnet, classes_Steer, activation='softmax')
                            convnetSt = regression(convnetSt, optimizer='adam', learning_rate=LR, loss='categorical_crossentropy', name='targetsSt')
                            
                            mergeIn = merge([convnetSp, convnetSt], 'concat')
                            
                            Edge_Model = tflearn.DNN(mergeIn, tensorboard_dir=Log_Car_Path)
                            
#=============================================================================
                            
                            Edge_Model.load(DNN_Edge_Model[:-20])
                            
                            LCDClear()
                            LCDSetColor(16777215, 0)
                            LCDMenu('Start', '', '', '')
                            LCDSetPrintf(16, 0, '[TensorFlow] Edge Model Successfully Loaded')
                            Edge_Model_Loaded = True
                            Edge_Model_Name = os.path.basename(DNN_Edge_Model)
                            Edge_Model_Name = Edge_Model_Name[:-26]
                                            
                            while(True):
                                pygame.event.pump()
                                KEY =  KEYRead()
                                
                                if((KEY == 1) or (B_Button() == True)):
                                    break
                        
#=============================================================================
                     
                    LCDClear()
                    LCDMenu('Record', 'Pause', 'Display', 'Back')
                    LCDSetColor(16777215, 0)
                    LCDSetPrintf(1, 37 + Display_Line_Y, 'Camera Drive')
                    LCDSetPrintf(17 + Display_Line_X + Display_Line_Extra, 0, 'Edge-Features CNN')
                    LCDImageStart(0, 0, 320, 240)
                    
                    Display = 1
                    Display_Button = 9
                    Recording_Data = -1
                    Record_Button = 9
                    Pause = 1
                    Pause_Button = 9
                    Start_Pause = True
                    New_Pause = True
                    New_Unpause = False
                    New_Recording = False
                    New_Unrecording = True
                    Hard_Pause_Brake = False
                    
                    Edge_Iterations = 0
                    Sector_Size = 20
                    Edge_Pixel_Thres = 0
                    Range_X = int(80/Sector_Size)
                    Range_Y = int(320/Sector_Size)
                    Offset_X = int(100/Sector_Size)
                    Speed_Past = 150
                    Speed = 150
                    Steering_Angle = 150
                    
                    Edge_X_Current = np.zeros((Range_X, Range_Y))
                    Edge_Y_Current = np.zeros((Range_X, Range_Y))
                    Edge_X_Past = np.zeros((Range_X, Range_Y))
                    Edge_Y_Past = np.zeros((Range_X, Range_Y))
                    
                    Trail_Mag = np.zeros((Range_X, Range_Y))
                    Trail_Angle = np.zeros((Range_X, Range_Y))
                    Trail_Count = np.zeros((Range_X, Range_Y))
        
                    Iterations = 0
                    Interventions = 0
                    Total_Injury_Time = 0
                    time.sleep(0.2)
                    Start_FPS = time.time()
                    
                    while(True):
                        Start = time.time()
                        img =  CAMGet()
                        image = np.array(img, dtype='uint8')
                        image = np.reshape(image, (240, 320, 3))
                        Canny_Image_Current = cv2.Canny(image, 150, 200)
                        pygame.event.pump()
                        KEY =  KEYRead()
                        
                        if(Edge_Model_Exists == 0):
                            Switch_Mode = 0
                            break
                    
                        if((X_Button() == True) or (KEY == 8)):
                            Switch_Mode = 0
                            break
                
                        if(L1_Button() == True):
                            Switch_Mode = 1
                            break
                
                        if(R1_Button() == True):
                            Switch_Mode = 3
                            break

                        Speed = 150
                        Steering_Angle = 150
                        
#=============================================================================

                        if(Iterations > 0):
                        
                            for i in range(Offset_X, Range_X + Offset_X):
        
                                for j in range(0, Range_Y):
                                    Small_Image_Current = Canny_Image_Current[i*Sector_Size:(i*Sector_Size) + Sector_Size, j*Sector_Size:(j*Sector_Size) + Sector_Size]
                                    Small_Image_Past = Canny_Image_Past[i*Sector_Size:(i*Sector_Size) + Sector_Size, j*Sector_Size:(j*Sector_Size) + Sector_Size]
                                    
                                    Edges_Current = np.where(Small_Image_Current == 255)
                                    Edges_Past = np.where(Small_Image_Past == 255)
                                    
                                    Edges_Pixels_Past = len(Edges_Past[0])
                                    Edges_Pixels_Current = len(Edges_Current[0])
                                    
                                    Edge_Diff = np.absolute(Edges_Pixels_Current - Edges_Pixels_Past)
                                    Edge_Avg = (Edges_Pixels_Current + Edges_Pixels_Past)/2
                                    
                                    if(Edge_Avg > 0):
                                        Edge_Diff = Edge_Diff*100/Edge_Avg
        
                                    if((Edges_Pixels_Current > Edge_Pixel_Thres) and (Edges_Pixels_Past > Edge_Pixel_Thres) and (Edge_Diff < 60) and (Edge_Iterations > 0)):
                                        Edge_X = (np.average(Edges_Current[0]) - np.average(Edges_Past[0]))**2
                                        Edge_Y = (np.average(Edges_Current[1]) - np.average(Edges_Past[1]))**2
                                        
                                        Trail_Mag[i - Offset_X][j]  = math.sqrt(Edge_X + Edge_Y)
                                        Edge_Angle = math.atan2(Edge_Y, Edge_X)*(180/math.pi)
                                        Trail_Angle[i - Offset_X][j]  = (round((Edge_Angle + 180)/22.5))/16
                                        Trail_Count[i - Offset_X][j]  = Edges_Pixels_Current
        
                                    elif((Edges_Pixels_Current > Edge_Pixel_Thres) and (Edge_Iterations > 0) and (i > Offset_X) and (i < Range_X + Offset_X - 1) and (j > 0) and (j < Range_Y - 1)  and (Edge_Iterations > 0)):
                                        Small_Image_Current = Canny_Image_Current[(i-1)*Sector_Size:((i+1)*Sector_Size) + Sector_Size, (j-1)*Sector_Size:((j+1)*Sector_Size) + Sector_Size]
                                        Small_Image_Past = Canny_Image_Past[(i-1)*Sector_Size:((i+1)*Sector_Size) + Sector_Size, (j-1)*Sector_Size:((j+1)*Sector_Size) + Sector_Size]
                                        
                                        Edges_Current = np.where(Small_Image_Current == 255)
                                        Edges_Past = np.where(Small_Image_Past == 255)
                                    
                                        Edges_Pixels_Past = len(Edges_Past[0])
                                        Edges_Pixels_Current = len(Edges_Current[0])
                                        
                                        Edge_Diff = np.absolute(Edges_Pixels_Current - Edges_Pixels_Past)
                                        Edge_Avg = (Edges_Pixels_Current + Edges_Pixels_Past)/2
                                        
                                        if(Edge_Avg > 0):
                                            Edge_Diff = Edge_Diff*100/Edge_Avg
        
                                        if((Edges_Pixels_Current > Edge_Pixel_Thres) and (Edges_Pixels_Past > Edge_Pixel_Thres) and (Edge_Diff < 20)):
                                            Edge_X = (np.average(Edges_Current[0]) - np.average(Edges_Past[0]))**2
                                            Edge_Y = (np.average(Edges_Current[1]) - np.average(Edges_Past[1]))**2
                                            
                                            Trail_Mag[i - Offset_X][j] = math.sqrt(Edge_X + Edge_Y)
                                            Edge_Angle = math.atan2(Edge_Y, Edge_X)*(180/math.pi)
                                            Trail_Angle[i - Offset_X][j]  = (round((Edge_Angle + 180)/22.5))/16
                                            Trail_Count[i - Offset_X][j]  = round(Edges_Pixels_Current/9)
                                            
                                    else:
                                        Trail_Mag[i - Offset_X][j]  = 0
                                        Trail_Angle[i - Offset_X][j]  = -1
                                        Trail_Count[i - Offset_X][j]  = 0
            
#=============================================================================
                        
                            Trail_Mag[:] = [x/25 for x in Trail_Mag]
                            Trail_Count[:] = [x/400 for x in Trail_Count]
                            
                            Train_Temp_Speed_In = np.zeros((1, classes_Speed))
                            
                            if((j >= 100) and (j <= 125)):  #Brakes, raonge 0->6 (0 strongest, 6 weak)
                                j = int(round((j - 100)/5))
                                
                            elif(j == 143):  #Reverse
                                j = 6
                                
                            elif(j == 150):           #Neutral to allow reverse
                                j = 7 
                                
                            elif(j == 155):            #Treat 155 as 158
                                j = 8
                                
                            elif(j >= 158):            #Forward 
                                j = int(9 + (j - 158))
                                
                            else:
                                print('Something wrong with Speed In')
                            
                            Train_Temp_Speed_In[0][j] = 1
                            Train_Temp_Speed_In_Vectors = np.array(Train_Temp_Speed_In, dtype='uint8')
                            
#=============================================================================   
    
                            X1 = np.array(Trail_Mag).reshape(-1, IMG_SIZE_X, IMG_SIZE_Y, 1)
                            X2 = np.array(Trail_Angle).reshape(-1, IMG_SIZE_X, IMG_SIZE_Y, 1)
                            X3 = np.array(Trail_Count).reshape(-1, IMG_SIZE_X,IMG_SIZE_Y, 1)
                            X4 = np.array(Train_Temp_Speed_In_Vectors).reshape(-1, classes_Speed)
                            Model_Output = Edge_Model.predict([X1, X2, X3, X4])
                            
                            Model_Output_Speed = Model_Output[0,0:27]
                            Model_Output_Steering = Model_Output[0,27:]
                            Output_Speed = np.argmax(Model_Output_Speed)
                            Output_Steering = np.argmax(Model_Output_Steering)
                            
                            if(Output_Speed < 6): 
                                Speed = (Output_Speed*5) + 100
                                
                            elif(Output_Speed == 6):
                                Speed = 143
                                
                            elif(Output_Speed == 7):
                                Speed = 150
                                
                            elif(Output_Speed == 8):     
                                Speed = 155
                                
                            elif(Output_Speed >= 9):  
                                Speed = Output_Speed - 9 + 158
                                
                            else:
                                print('Something wrong with Speed')
                            
                            if(Output_Steering == 0):
                                Steering_Angle = 100
                                
                            elif((Output_Steering > 0) and (Output_Steering < 46)):
                                Steering_Angle = (Output_Steering + 52)*2
                                
                            elif(Output_Steering == 46):
                                Steering_Angle = 200
                                
                            else:
                                print('Something wrong with Steer')
                                
                            Speed = int(Speed)
                            Steering_Angle = int(Steering_Angle)

                        Canny_Image_Past = Canny_Image_Current
                        
#=============================================================================   
                     
                        if((KEY == 1) or ((B_Button() == True) and ((time.time() - Record_Button) > 0.2))):
                            Recording_Data = Recording_Data*-1
                            Record_Button = time.time()
            
                            if(Recording_Data == 1):
                                New_Recording = True
            
                            else:
                                New_Unrecording = True
                                
                        if((KEY == 2) or ((A_Button() == True) and ((time.time() - Pause_Button) > 0.2))):
                            Pause = Pause*-1
                            Pause_Button = time.time()
                            
                            if(Start_Pause == True): 
                                Start_Pause = False
                                Start_Autonomy = time.time()
            
                            if(Pause == 1):
                                Interventions += 1
                                New_Pause = True
                                Recording_Data = -1
                                New_Unrecording = True
                                
                            else:
                                New_Unpause = True
                        
                        if((KEY == 4) or ((Y_Button() == True) and ((time.time() - Display_Button) > 0.2))):
                            Display += 1
                            Display_Button = time.time()
                            
                            Recording_Data = -1
                            New_Unrecording = True
            
                            if(Display == 3):
                                Display = 1
                         
                        if(Pause == 1):
                            Steering_Angle = 150
                            Speed = 150
                            
                            if(New_Pause == True):
                            
                                if(Start_Pause == False):
                                    Start_Injury_Time = time.time()
                                    
                                LCDCircle(400, 180, 50, 65280, 1)
                                LCDSetColor(16777215, 0)
                                LCDSetPrintf(14 + Display_Line_X, 40 + Display_Line_Y, 'Paused')
                        
                        else:
                            
                            if(New_Unpause == True):
                            
                                if(Interventions > 0):
                                    End_Injury_Time = time.time() - Start_Injury_Time
                                    Total_Injury_Time += End_Injury_Time
                                    
                                LCDCircle(400, 180, 50, 0, 1)
                                LCDSetColor(0, 0)
                                LCDSetPrintf(14 + Display_Line_X, 40 + Display_Line_Y, 'Paused')
                                New_Unpause = False
                            
                        if(New_Pause == True):
                            New_Pause = False
                            
                            if(Speed_Past > Normal_Speed):
                                Hard_Pause_Brake = True
                                
                            else:
                                Hard_Pause_Brake = False
                            
                        if((Hard_Pause_Brake == True) and (Iterations > 25)):
                            
                            if(SIMULATION == False):
                                Steering_Angle = 150
                                Speed = Brake_Speed
                                ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                                ServoBlaster.flush()
                                ServoBlaster.write('1=' + str(Speed) + '\n')
                                ServoBlaster.flush()
                                time.sleep(Hard_Brake_Power)
                                Hard_Pause_Brake = False
                                
                            else:
                                Steering_Angle = 150
                                Speed = 150
                                Simulation_Drive(Speed, Steering_Angle)
                                
                            Recording_Data = -1
                        
                        else:
                        
                            if(SIMULATION == False):
                                ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                                ServoBlaster.flush()                         
                                if(Speed==Brake_Speed):
                                    ServoBlaster.write('1=' + str(150) + '\n')
                                    ServoBlaster.flush()
                                    ServoBlaster.write('1=' + str(Speed) + '\n')
                                    ServoBlaster.flush()
                                else:
                                    ServoBlaster.write('1=' + str(Speed) + '\n')
                                    ServoBlaster.flush()   
                            else:
                                Simulation_Drive(Speed, Steering_Angle)
                                
                            New_Pause_Iteration = 0
                                
                        if(Display == 1):
                            LCDImage(img)
                        
                        elif(Display == 2):
                            Canny_Image = (Canny_Image_Current.ctypes.data_as(ctypes.POINTER(ctypes.c_byte)))
                            LCDImageGray(Canny_Image)
                            
                        if(Recording_Data == 1):
                            Edges_Camera.append(time.time() - Start)
            
                            if(New_Recording == True):
                                LCDCircle(400, 80, 50, 16711680, 1)
                                LCDSetColor(16777215, 0)
                                LCDSetPrintf(7 + Display_Line_X, 36 + Display_Line_Y, 'Recording Data')
                                New_Recording = False
                            
                        else:
            
                            if(New_Unrecording == True):
                                LCDCircle(400, 80, 50, 0, 1)
                                LCDSetColor(0, 0)
                                LCDSetPrintf(7 + Display_Line_X, 36 + Display_Line_Y, 'Recording Data')
                                New_Unrecording = False
            
                        LCDSetColor(16777215, 0)
                        LCDSetPrintf(16 + Display_Line_X + Display_Line_Extra, 0, 'Steering Angle: %d    Speed: %d    Display: %s' %(Steering_Angle, Speed, Manual_Display_Modes[Display]))
            
                        Iterations += 1
                        Speed_Past = Speed
                            
                    if(SIMULATION == False):
                        
                        if(Speed > Normal_Speed):
                            ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                            ServoBlaster.flush()
                            ServoBlaster.write('1=' + str(Brake_Speed) + '\n')
                            ServoBlaster.flush()
                            time.sleep(Hard_Brake_Power)
                            
                        ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                        ServoBlaster.flush()
                        ServoBlaster.write('1=' + str(150) + '\n')
                        ServoBlaster.flush()
                        
                    else:
                        Simulation_Drive(150, 150)
                    
                    if((Switch_Mode == 0) and (Edge_Model_Exists != 0) and (Start_Pause == False)):
                        End_SPS = time.time()
                        Lidar_SPS = float(Iterations/(End_SPS - Start_Autonomy))
                        
                        LCDClear()
                        LCDMenu('Continue', '', '', '')
                        LCDSetPrintf(15, 0, '[ModelCar-2] Scans/Second      : %.2f' %(Lidar_SPS))
                        
                        if(Start_Pause == False):
                            Autonomy = float((1 - ((Interventions*5)/(End_SPS - Start_FPS - Total_Injury_Time)))*100)
                            LCDSetPrintf(16, 0, '[ModelCar-2] Autonomy(Percent) : %.2f' %(Autonomy))
                            
                        else:
                            LCDSetPrintf(16, 0, '[ModelCar-2] No Interventions Recorded')
                                    
                        while(True):
                            pygame.event.pump()
                            KEY =  KEYRead()
                                    
                            if((KEY == 1) or (B_Button() == True)):
                                break
                            
                    Using_Edge_Mode = True
                    break
                    
#=============================================================================
               
                elif((KEY == 8) or (X_Button() == True)):
                
                    if(Model_Change == True):
                        Camera_Drive_Check = True
                        
                    else:
                        Camera_Drive_Check = False
                    
                    break
                    
                if(L1_Button() == True):
                    Switch_Mode = 1
                    break
                    
                if(R1_Button() == True):
                    Switch_Mode = 3
                    break
                                     
#=============================================================================

        LCDClear()
        LCDMenu('Record', 'Pause', 'Model', 'Back')
        LCDSetColor(16777215, 0)
        LCDSetPrintf(1, 37 + Display_Line_Y, 'Camera Drive')
        LCDSetPrintf(17 + Display_Line_X + Display_Line_Extra, 0, 'NVIDIA PilotNet')
        LCDImageStart(0, 0, 320, 240)
        
        Recording_Data = -1
        Record_Button = 9
        Pause = 1
        Pause_Button = 9
        Start_Pause = True
        New_Pause = True
        New_Unpause = False
        New_Recording = False
        New_Unrecording = True
        Hard_Pause_Brake = False
        
        Speed_Past = 150
        Change = 0
        Change_Button = 9
        Iterations = 0
        Interventions = 0
        Total_Injury_Time = 0
        time.sleep(0.2)
        Start_FPS = time.time()

        while(True):
            
            if(Using_Edge_Mode == True):
                break
        
            if((Camera_Drive_Check == False) and (Switch_Mode == 0)):
                break
                
            elif(Switch_Mode != 0):
                break
                
            Start = time.time()
            img =  CAMGet()
            LCDImage(img)
            image = np.array(img, dtype='uint8')
            image = np.reshape(image, (240, 320, 3))
            pygame.event.pump()
            KEY =  KEYRead()
            
            if((X_Button() == True) or (KEY == 8)):
                Switch_Mode = 0
                break
                
            if(L1_Button() == True):
                Switch_Mode = 1
                break
                
            if(R1_Button() == True):
                Switch_Mode = 3
                break
            
            if(('LSTM' not in Model_Name) and ('3dcnn' not in Model_Name)):    
                image = Image_Processing(image)
                X = np.expand_dims(image, axis=0)
                if(Use_Speed == True):
                    (Speed, Steering_Angle) = model.predict(X)
                    Speed = int(Speed)
                    Steering_Angle = int(Steering_Angle)
                    if(Steering_Angle< 105):
                        Steering_Angle=105
                    elif(Steering_Angle> 195):
                        Steering_Angle=195
                    if(Speed< 145 and Speed> 135):
                        Speed=135
                    elif(Speed> 155 and Speed< 162):
                        Speed=162

                else:
                    Steering_Angle = int(model.predict(X))
                    Speed = 163
            else:
                image = Image_Processing(image)
                sequence.append(image)
                if(len(sequence)==sequence_len):
                    X = np.expand_dims(sequence, axis=0)
                    sequence.pop(0)
                    if(Use_Speed == True):
                        (Speed, Steering_Angle) = model.predict(X)
                        Speed = int(Speed)
                        Steering_Angle = int(Steering_Angle)
                        if(Steering_Angle< 105):
                            Steering_Angle=105
                        elif(Steering_Angle> 195):
                            Steering_Angle=195
                        if(Speed< 150 and Speed> 135):
                            Speed=135
                        elif(Speed>= 150 and Speed< 162):
                            Speed=162
                        #Speed=164
                    else:
                        Steering_Angle = int(model.predict(X))
                        Speed = 163
            
                
            if((KEY == 1) or ((B_Button() == True) and ((time.time() - Record_Button) > 0.2))):
                Recording_Data = Recording_Data*-1
                Record_Button = time.time()

                if(Recording_Data == 1):
                    New_Recording = True
                    Now = str(datetime.datetime.now())
                    Image_Stamp = 'Nvidia_Image_' + Stamp_Type + Now[:-7]
                    New_Image_Save_Path = os.path.join(Image_Path, Image_Stamp)
                    os.makedirs(New_Image_Save_Path)
                    Image_Stamp = 0

                else:
                    New_Unrecording = True
                
            if((KEY == 2) or ((A_Button() == True) and ((time.time() - Pause_Button) > 0.2))):
                Pause = Pause*-1
                Pause_Button = time.time()
                
                if(Start_Pause == True): 
                    Start_Pause = False
                    Start_Autonomy = time.time()

                if(Pause == 1):
                    Interventions += 1
                    New_Pause = True
                    Recording_Data = -1
                    New_Unrecording = True
                    
                else:
                    New_Unpause = True
                    
            if((KEY == 4) or ((Y_Button() == True) and ((time.time() - Change_Button) > 0.2))):
                Change = 1
                Change_Button = time.time()
                Switch_Mode = 0
                break
                
            if(Pause == 1):
                Steering_Angle = 150
                Speed = 150
                
                if(New_Pause == True):
                
                    if(Start_Pause == False):
                        Start_Injury_Time = time.time()
                        
                    LCDCircle(400, 180, 50, 65280, 1)
                    LCDSetColor(16777215, 0)
                    LCDSetPrintf(14 + Display_Line_X, 40 + Display_Line_Y, 'Paused')
            
            else:
                
                if(New_Unpause == True):
                
                    if(Interventions > 0):
                        End_Injury_Time = time.time() - Start_Injury_Time
                        Total_Injury_Time += End_Injury_Time
                        
                    LCDCircle(400, 180, 50, 0, 1)
                    LCDSetColor(0, 0)
                    LCDSetPrintf(14 + Display_Line_X, 40 + Display_Line_Y, 'Paused')
                    New_Unpause = False
                    
            if(New_Pause == True):
                New_Pause = False
                        
                if(Speed_Past > Normal_Speed):
                    Hard_Pause_Brake = True 
                            
                else:
                    Hard_Pause_Brake = False
                        
            if((Hard_Pause_Brake == True) and (Iterations > 25)):
                        
                if(SIMULATION == False):
                    Steering_Angle = 150
                    Speed = Brake_Speed
                    ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                    ServoBlaster.flush()
                    #ServoBlaster.write('1=' + str(150) + '\n')
                    #ServoBlaster.flush()
                    #time.sleep(Hard_Brake_Power)
                    ServoBlaster.write('1=' + str(Speed) + '\n')
                    ServoBlaster.flush()
                    #time.sleep(Hard_Brake_Power)
                    Hard_Pause_Brake = False
                    
                else:
                    Steering_Angle = 150
                    Speed = 150
                    Simulation_Drive(Speed, Steering_Angle)
                    
                Recording_Data = -1
            
            else:
            
                if(SIMULATION == False):
                    ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                    ServoBlaster.flush()
                    if(old_Speed>=150 and Speed<150):
                        ServoBlaster.write('1=' + str(150) + '\n')
                        ServoBlaster.flush()
                        time.sleep(0.2)
                        ServoBlaster.write('1=' + str(148) + '\n')
                        ServoBlaster.flush()
                        #ServoBlaster.write('1=' + str(142) + '\n')
                        #ServoBlaster.flush()
                        time.sleep(0.2)
                    ServoBlaster.write('1=' + str(Speed) + '\n')
                    ServoBlaster.flush()
                    old_Speed=Speed                       
                else:
                    Simulation_Drive(Speed, Steering_Angle)
                    
                New_Pause_Iteration = 0

            if(Recording_Data == 1):
                Image_Array = [image, Speed, Steering_Angle]
                Image_Filename = 'Nvidia_Image_' + str(Image_Count) + '_' + str(Speed) + '_' + str(Steering_Angle)
                np.save(os.path.join(New_Image_Save_Path, Image_Filename), np.array(Image_Array))
                Image_Count += 1
                Nvidia_Camera.append(time.time() - Start)

                if(New_Recording == True):
                    LCDCircle(400, 80, 50, 16711680, 1)
                    LCDSetColor(16777215, 0)
                    LCDSetPrintf(7 + Display_Line_X, 36 + Display_Line_Y, 'Recording Data')
                    New_Recording = False
                
            else:

                if(New_Unrecording == True):
                    LCDCircle(400, 80, 50, 0, 1)
                    LCDSetColor(0, 0)
                    LCDSetPrintf(7 + Display_Line_X, 36 + Display_Line_Y, 'Recording Data')
                    New_Unrecording = False

            LCDSetColor(16777215, 0)
            LCDSetPrintf(16 + Display_Line_X + Display_Line_Extra, 0,'Steering Angle: %d    Speed: %d' % (Steering_Angle, Speed))
            LCDSetPrintf(17 + Display_Line_X + Display_Line_Extra, 0, 'Model Loaded: %s' %(Model_Name))
                
            Iterations += 1
            Speed_Past = Speed
                
        if(SIMULATION == False):
            
            if(Speed > Normal_Speed):
                ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                ServoBlaster.flush()
                ServoBlaster.write('1=' + str(Brake_Speed) + '\n')
                ServoBlaster.flush()
                time.sleep(Hard_Brake_Power)
            ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
            ServoBlaster.flush()
            ServoBlaster.write('1=' + str(150) + '\n')
            ServoBlaster.flush()

        else:
            Simulation_Drive(150, 150)
        
        if((Camera_Drive_Check == True) and (Change == 0) and (Switch_Mode == 0)):
            End_SPS = time.time()
            Lidar_SPS = float(Iterations/(End_SPS - Start_Autonomy))
            
            LCDClear()
            LCDMenu('Continue', '', '', '')
            LCDSetPrintf(13, 0, 'Camera Drive')
            LCDSetPrintf(15, 0, '[ModelCar-2] Scans/Second      : %.2f' %(Lidar_SPS))
            
            if(Start_Pause == False):
                Autonomy = float((1 - ((Interventions*5)/(End_SPS - Start_FPS - Total_Injury_Time)))*100)
                LCDSetPrintf(16, 0, '[ModelCar-2] Autonomy(Percent) : %.2f' %(Autonomy))
                
            else:
                LCDSetPrintf(16, 0, '[ModelCar-2] No Interventions Recorded')
                        
            while(True):
                pygame.event.pump()
                KEY =  KEYRead()
                        
                if((KEY == 1) or (B_Button() == True)):
                    break
        
        time.sleep(0.2)
        
        if(Change == 1):
            Model_Change = True
            Camera_Drive_Check = False
            
        else:
            Model_Change = False
            
        Exit_Drive_Check = True
              
#=============================================================================

    elif((KEY == 4) or (Y_Button() == True) or (Switch_Mode == 3)):
        Switch_Mode = 0
        
        if(Lidar_Drive_Check == False):
            
            if(SIMULATION == True):
                LIDARSet(240, 0, 682)
                lidar = Laser(682, 10, 240, 5600)
                slam = RMHC_SLAM(lidar, 240, Map_Size)
                mapbytes = bytearray(240*240)
                Lidar_Drive_Check = True
            
            elif((os.path.exists(Urg_Path) == True)):
                
                try:
                    lidar = LIDAR(Urg_Path)
                    
                except(TypeError):
                    
                    try:
                        lidar = LIDAR(Urg_Path)
                        
                    except(TypeError):
                        LCDClear()
                        LCDMenu('', '', '', 'Back')
                        LCDSetColor(16777215, 0)
                        LCDSetPrintf(16, 0, '[Error!] Check Lidar is Connected, then Restart')
                        Lidar_Drive_Check = False
                        
                        while(True):
                            pygame.event.pump()
                            KEY =  KEYRead()
        
                            if((KEY == 8) or (X_Button() == True)):
                                break
                    
                slam = RMHC_SLAM(SLAM(), 240, Map_Size) 
                mapbytes = bytearray(240*240)
                Lidar_Drive_Check = True

            else:
                LCDClear()
                LCDMenu('', '', '', 'Back')
                LCDSetColor(16777215, 0)
                LCDSetPrintf(16, 0, '[Error!] Check Lidar is Connected, then Restart')
                Lidar_Drive_Check = False
                
                while(True):
                    pygame.event.pump()
                    KEY =  KEYRead()

                    if((KEY == 8) or (X_Button() == True)):
                        break
                                              
#=============================================================================

        LCDClear()
        LCDMenu('Record', 'Pause', 'Display', 'Back')
        LCDSetColor(16777215, 0)
        LCDSetPrintf(1, 37 + Display_Line_Y, ' Lidar Drive')

        Sector_Size = 20
        Edge_Pixel_Thres = 0
        Range_X = int(80/Sector_Size)
        Range_Y = int(320/Sector_Size)
        Offset_X = int(100/Sector_Size)
        Speed_Past = 150
        
        Edge_X_Current = np.zeros((Range_X, Range_Y))
        Edge_Y_Current = np.zeros((Range_X, Range_Y))
        Edge_X_Past = np.zeros((Range_X, Range_Y))
        Edge_Y_Past = np.zeros((Range_X, Range_Y))
        
        Priority = 'Straight'
        #Priority = 'Right   '
        Recording_Data = -1
        Record_Button = 9
        Increase_Button = 9
        Decrease_Button = 9
        Left_Side_Button = 9
        Right_Side_Button = 9
        Pause = 1
        Pause_Button = 9
        New_Pause = True
        New_Unpause = False
        New_Recording = False
        New_Unrecording = True
        Hard_Pause_Brake = False
        
        Display = 1
        Display_Button = 9
        Iterations = 0
        Edge_Iterations = 0
        Interventions = 0
        Total_Injury_Time = 0
        Start_Pause = True
        time.sleep(0.2)
        Start_Autonomy = time.time()
        
        if(SIMULATION == True):
            x1, y1, z1, th1 =  SIMGetRobot( OSMachineID()) 
            x1 = x1.value
            y1 = y1.value
            z1 = z1.value
            th1 = th1.value
            t1 = time.time()

        while(True):
            Start = time.time()
            img =  CAMGet()
            image = np.array(img, dtype='uint8')
            image = np.reshape(image, (240, 320, 3))
            pygame.event.pump()
            KEY =  KEYRead()
            
            if(Lidar_Drive_Check == False):
                Switch_Mode = 0
                break
            
            if((X_Button() == True) or (KEY == 8)):
                Switch_Mode = 0
                break
                
            if(L1_Button() == True):
                Switch_Mode = 2
                break
                
            if(R1_Button() == True):
                Switch_Mode = 1
                break
                
            if((UP_Button() == True) and ((time.time() - Increase_Button) > 0.2)):
                Max_Allow_Speed += 1
                Increase_Button = time.time()
                
                if(Max_Allow_Speed > 175):
                    Max_Allow_Speed = 175
                    
            if((DOWN_Button() == True) and ((time.time() - Decrease_Button) > 0.2)):
                Max_Allow_Speed -= 1
                Decrease_Button = time.time()
                
                if(Max_Allow_Speed < 159):
                    Max_Allow_Speed = 159

            if((LEFT_Button() == True) and ((time.time() - Left_Side_Button) > 0.2)):
                Left_Side_Button = time.time()
                
                if(Direction == 4):
                    Direction = 1
                    Priority = 'Straight'
                    
                else:
                    Direction = 0.25
                    Priority = 'Left    '
                   
            if((RIGHT_Button() == True) and ((time.time() - Right_Side_Button) > 0.2)):
                Right_Side_Button = time.time()
                
                if(Direction == 0.25):
                    Direction = 1
                    Priority = 'Straight'
                    
                else:
                    Direction = 4
                    Priority = 'Right   '
                
            if((os.path.exists(Urg_Path) == False) and (SIMULATION == False)):
                LCDClear()
                LCDMenu('', '', '', 'Back')
                LCDSetPrintf(16, 0, '[Error!] Check Lidar is Connected, then Restart')
                Lidar_Check = False
                
                while(True):
                    pygame.event.pump()
                    KEY =  KEYRead()

                    if((KEY == 8) or (X_Button() == True)):
                        break
                
                break
   
            if((KEY == 1) or ((B_Button() == True) and ((time.time() - Record_Button) > 0.2))):
                Recording_Data = Recording_Data*-1
                Record_Button = time.time()

                if(Recording_Data == 1):
                    New_Recording = True
                    
                    if((Display == 4) or (Multi_Logging == True)):
                        Now = str(datetime.datetime.now())
                        Image_Stamp = 'Lidar_Image_' + Stamp_Type + Now[:-7]
                        New_Image_Save_Path = os.path.join(Image_Path, Image_Stamp)
                        os.makedirs(New_Image_Save_Path)
                        Image_Count = 0
                        
                    if(Display == 5):
                        Now = str(datetime.datetime.now())
                        Edge_Stamp = 'Edge_' + Stamp_Type + Now[:-7]
                        New_Edge_Save_Path = os.path.join(Edge_Path, Edge_Stamp)
                        os.makedirs(New_Edge_Save_Path)
                        Edge_Count = 0
                     
                    elif(Display == 1):
                        Now = str(datetime.datetime.now())
                        Scan_Stamp = 'Scan_' + Stamp_Type + Now[:-7]
                        New_Scan_Save_Path = os.path.join(Scan_Path, Scan_Stamp)
                        os.makedirs(New_Scan_Save_Path)
                        Scan_Count = 0

                else:
                    New_Unrecording = True
            
            if((KEY == 2) or ((A_Button() == True) and ((time.time() - Pause_Button) > 0.2))):
                Pause = Pause*-1
                Pause_Button = time.time()
                
                if(Start_Pause == True):
                    Start_Pause = False
                    Start_SPS = time.time()

                if(Pause == 1):
                    Interventions += 1
                    New_Pause = True
                    Recording_Data = -1
                    New_Unrecording = True

                else:
                    New_Unpause = True
                
            if((KEY == 4) or ((Y_Button() == True) and ((time.time() - Display_Button) > 0.2))):
                Display += 1
                Display_Button = time.time()
                
                Recording_Data = -1
                New_Unrecording = True

                if(Display == 6):
                    Display = 1
                    
            if(SIMULATION == False):
                Scan = lidar.getScan()

                if((Recording_Data == 1) and (Display == 1)):
                    Scan_Filename = 'Scan_' + str(Scan_Count) + '_' + str(Speed) + '_' + str(Steering_Angle)
                    np.save(os.path.join(New_Scan_Save_Path, Scan_Filename), np.array(Scan))
                    Scan_Count += 1

            else:
                Scan =  LIDARGet()
                Scan = [x*Sim_Scale for x in Scan]
                if((Recording_Data == 1) and (Display == 1)):
                    Scan_Filename = 'Scan_' + str(Scan_Count) + '_' + str(Speed) + '_' + str(Steering_Angle)
                    np.save(os.path.join(New_Scan_Save_Path, Scan_Filename), np.array(Scan))
                    Scan_Count += 1   
            if(Start_Pause == False):
            
                if(SIMULATION == False):
                    slam.update(Scan)
                    
                else:
                    x2, y2, z2, th2 =  SIMGetRobot( OSMachineID()) #C type, etc: c_int(361)
                    x2 = x2.value
                    y2 = y2.value
                    z2 = z2.value
                    th2 = th2.value
                    dx = x2 - x1
                    dy = y2 - y1
                    dth = -(th2 - th1)
                    dxy = math.sqrt(((dx)**2) + ((dy)**2))         
                    x1 = x2
                    y1 = y2 
                    th1 = th2 
                    t2 = time.time()
                    dt = t2 - t1
                    t1 = t2
                    
                    slam.update(Scan, (dxy, dth, dt))
                
                (Speed, Steering_Angle) = LIDARBot(Scan)
                #print("Speed: ",Speed, "Steering Angle: ",Steering_Angle)
                
                if((SIMULATION == True) and (Sim_speed ==0)):
                    Speed = 165
                    
                x, y, theta = slam.getpos()
                slam.getmap(mapbytes)

            if(Pause == 1):
                Steering_Angle = 150
                Speed = 150

                if(New_Pause == True):
                    
                    if(Start_Pause == False):
                        Start_Injury_Time = time.time()
                        
                    LCDCircle(400, 180, 50, 65280, 1)
                    LCDSetColor(16777215, 0)
                    LCDSetPrintf(14 + Display_Line_X, 40 + Display_Line_Y, 'Paused')

#=============================================================================

                    F = URG_Distance_Range
                    F_Left = URG_Distance_Range
                    F_Right = URG_Distance_Range
                    F_Front = URG_Distance_Range

                    Steering = Centre_Steering
                    Rev_Steering = Centre_Steering
             
                    Speed_In = 150
                    Speed_Out = 0

                    Left_Steer_Flag = 0
                    Right_Steer_Flag = 0
                    Image_Turn_Flag = 0

                    Direction = 1
                    Turn_State = 0
                    Turn_State_Count = 0
                    Auto_Drive_State = 0
                    Rev_Count = 0
                    Con_Rev_Counter = 0
                    
                    Priority = 'Straight'

#=============================================================================

            else:

                if(New_Unpause == True):
                    
                    if(Interventions > 0):
                        End_Injury_Time = time.time() - Start_Injury_Time
                        Total_Injury_Time += End_Injury_Time
                        
                    LCDCircle(400, 180, 50, 0, 1)
                    LCDSetColor(0, 0)
                    LCDSetPrintf(14 + Display_Line_X, 40 + Display_Line_Y, 'Paused')
                    New_Unpause = False
            
            if(New_Pause == True):
                New_Pause = False
                            
                if(Speed_Past > Normal_Speed):
                    Hard_Pause_Brake = True
                                
                else:
                    Hard_Pause_Brake = False
                            
            if((Hard_Pause_Brake == True) and (Iterations > 25)):
                            
                if(SIMULATION == False):
                    Steering_Angle = 150
                    Speed = Brake_Speed
                    ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                    ServoBlaster.flush()
                    ServoBlaster.write('1=' + str(Speed) + '\n')
                    ServoBlaster.flush()
                    time.sleep(Hard_Brake_Power)
                    Hard_Pause_Brake = False
                    
                else:
                    Speed = 150
                    Steering_Angle = 150
                    Simulation_Drive(Speed, Steering_Angle)
                    
                Recording_Data = -1
            
            else: 
            
                if(SIMULATION == False):
                    ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                    ServoBlaster.flush()
                    if(Speed==Brake_Speed):
                        ServoBlaster.write('1=' + str(150) + '\n')
                        ServoBlaster.flush()
                        ServoBlaster.write('1=' + str(Speed) + '\n')
                        ServoBlaster.flush()
                    else:
                        ServoBlaster.write('1=' + str(Speed) + '\n')
                        ServoBlaster.flush()   
                else:
                    Simulation_Drive(Speed, Steering_Angle)
                    
                New_Pause_Iteration = 0
            
            if(Display == 3):   
                Slam_Image = np.array(mapbytes)
                Slam_Image = (Slam_Image.ctypes.data_as(ctypes.POINTER(ctypes.c_byte)))
              
                LCDImageStart(0, 0, 240, 240)
                LCDArea(240, 0, 320, 240, 0, 1)
                LCDImageGray(Slam_Image)
                
            elif((Display == 1) or (Display == 2)):
                image = Image.new(mode = 'RGB', size = (320, 240), color=(0, 0, 0))
                Draw = ImageDraw.Draw(image)
                
                for i in range(len(Scan)):
    
                    if(Display == 1):
                    
                         if((Scan[i] <= 20) and (Start_Pause == False)):
                            Scan[i] = 5600
                            
                         X = 160 - (-math.cos(Draw_Angles[i])*Draw_Scale*Scan[i])
                         Y = 160 - ( math.sin(Draw_Angles[i])*Draw_Scale*Scan[i])
                         Draw.line((160, 160, X, Y), fill=(255, 255, 255))
                        
                    elif(Display == 2):
                         X = 160 - (-math.cos(Draw_Angles[i])*Draw_Scale*Scan[i])
                         Y = 160 - ( math.sin(Draw_Angles[i])*Draw_Scale*Scan[i])
                         Draw.point((X, Y), fill=(255, 255, 255))
                
                Draw.polygon([(160, 160), (155, 175), (165, 175)], fill = (255, 0, 0))
                Scan_Image = np.array(image)
                Scan_Image = (Scan_Image.ctypes.data_as(ctypes.POINTER(ctypes.c_byte)))
                
                LCDImageStart(0, 0, 320, 240)
                LCDImage(Scan_Image)
                
            elif(Display == 4):
                
                if(Recording_Data == 1):
                    Image_Array = [image, Speed, Steering_Angle]
                    Image_Filename = 'Lidar_Image_' + str(Image_Count) + '_' + str(Speed) + '_' + str(Steering_Angle)
                    np.save(os.path.join(New_Image_Save_Path, Image_Filename), np.array(Image_Array))
                    Image_Count += 1
                    
                LCDImageStart(0, 0, 320, 240)
                LCDImage(img)

            elif(Display == 5):
                Canny_Image_Current = cv2.Canny(image, 150, 200)
                
                Trail_Mag = np.zeros((Range_X, Range_Y))
                Trail_Angle = np.zeros((Range_X, Range_Y))
                Trail_Count = np.zeros((Range_X, Range_Y))

                if(Edge_Iterations > 0):
                
                    for i in range(Offset_X, Range_X + Offset_X):

                        for j in range(0, Range_Y):
                            Small_Image_Current = Canny_Image_Current[i*Sector_Size:(i*Sector_Size) + Sector_Size, j*Sector_Size:(j*Sector_Size) + Sector_Size]
                            Small_Image_Past = Canny_Image_Past[i*Sector_Size:(i*Sector_Size) + Sector_Size, j*Sector_Size:(j*Sector_Size) + Sector_Size]
                            
                            Edges_Current = np.where(Small_Image_Current == 255)
                            Edges_Past = np.where(Small_Image_Past == 255)
                            
                            Edges_Pixels_Past = len(Edges_Past[0])
                            Edges_Pixels_Current = len(Edges_Current[0])
                            
                            Edge_Diff = np.absolute(Edges_Pixels_Current - Edges_Pixels_Past)
                            Edge_Avg = (Edges_Pixels_Current + Edges_Pixels_Past)/2
                            
                            if(Edge_Avg > 0):
                                Edge_Diff = Edge_Diff*100/Edge_Avg

                            if((Edges_Pixels_Current > Edge_Pixel_Thres) and (Edges_Pixels_Past > Edge_Pixel_Thres) and (Edge_Diff < 60) and (Edge_Iterations > 0)):
                                Edge_X = (np.average(Edges_Current[0]) - np.average(Edges_Past[0]))**2
                                Edge_Y = (np.average(Edges_Current[1]) - np.average(Edges_Past[1]))**2
                                
                                Trail_Mag[i - Offset_X][j]  = math.sqrt(Edge_X + Edge_Y)
                                Edge_Angle = math.atan2(Edge_Y, Edge_X)*(180/math.pi)
                                Trail_Angle[i - Offset_X][j]  = (round((Edge_Angle + 180)/22.5))/16
                                Trail_Count[i - Offset_X][j]  = Edges_Pixels_Current

                            elif((Edges_Pixels_Current > Edge_Pixel_Thres) and (Edge_Iterations > 0) and (i > Offset_X) and (i < Range_X + Offset_X - 1) and (j > 0) and (j < Range_Y - 1)  and (Edge_Iterations > 0)):
                                Small_Image_Current = Canny_Image_Current[(i-1)*Sector_Size:((i+1)*Sector_Size) + Sector_Size, (j-1)*Sector_Size:((j+1)*Sector_Size) + Sector_Size]
                                Small_Image_Past = Canny_Image_Past[(i-1)*Sector_Size:((i+1)*Sector_Size) + Sector_Size, (j-1)*Sector_Size:((j+1)*Sector_Size) + Sector_Size]
                                
                                Edges_Current = np.where(Small_Image_Current == 255)
                                Edges_Past = np.where(Small_Image_Past == 255)
                            
                                Edges_Pixels_Past = len(Edges_Past[0])
                                Edges_Pixels_Current = len(Edges_Current[0])
                                
                                Edge_Diff = np.absolute(Edges_Pixels_Current - Edges_Pixels_Past)
                                Edge_Avg = (Edges_Pixels_Current + Edges_Pixels_Past)/2
                                
                                if(Edge_Avg > 0):
                                    Edge_Diff = Edge_Diff*100/Edge_Avg

                                if((Edges_Pixels_Current > Edge_Pixel_Thres) and (Edges_Pixels_Past > Edge_Pixel_Thres) and (Edge_Diff < 20)):
                                    Edge_X = (np.average(Edges_Current[0]) - np.average(Edges_Past[0]))**2
                                    Edge_Y = (np.average(Edges_Current[1]) - np.average(Edges_Past[1]))**2
                                    
                                    Trail_Mag[i - Offset_X][j] = math.sqrt(Edge_X + Edge_Y)
                                    Edge_Angle = math.atan2(Edge_Y, Edge_X)*(180/math.pi)
                                    Trail_Angle[i - Offset_X][j]  = (round((Edge_Angle + 180)/22.5))/16
                                    Trail_Count[i - Offset_X][j]  = round(Edges_Pixels_Current/9)
                                    
                            else:
                                Trail_Mag[i - Offset_X][j]  = 0
                                Trail_Angle[i - Offset_X][j]  = -1
                                Trail_Count[i - Offset_X][j]  = 0
    
                Canny_Image_Past = Canny_Image_Current
                Servo_Controls = np.array([Speed_Past, Speed, Steering_Angle])
                
                if((Recording_Data == 1)):
                    Edge_Array = [Trail_Mag, Trail_Angle, Trail_Count, Servo_Controls]
                    Edge_Filename = 'Edge_' + str(Edge_Count) + '_' + str(Speed) + '_' + str(Steering_Angle)
                    np.save(os.path.join(New_Edge_Save_Path, Edge_Filename), np.array(Edge_Array))
                    Edge_Count += 1
                    
                    if(Multi_Logging == True):
                        Image_Array = [image, Speed, Steering_Angle]
                        Image_Filename = 'Lidar_Image_' + str(Image_Count) + '_' + str(Speed) + '_' + str(Steering_Angle)
                        np.save(os.path.join(New_Image_Save_Path, Image_Filename), np.array(Image_Array))
                        Image_Count += 1
                
                del Trail_Mag
                del Trail_Angle
                del Trail_Count
                
                Edge_Iterations += 1
                    
                Canny_Image = (Canny_Image_Current.ctypes.data_as(ctypes.POINTER(ctypes.c_byte)))
                LCDImageStart(0, 0, 320, 240)
                LCDImageGray(Canny_Image)

            if(Recording_Data == 1):
                Lidar.append(time.time() - Start)

                if(New_Recording == True):
                    LCDCircle(400, 80, 50, 16711680, 1)
                    LCDSetColor(16777215, 0)
                    
                    if(Multi_Logging == False):
                        LCDSetPrintf(7 + Display_Line_X, 36 + Display_Line_Y, 'Recording Data')
                        
                    else:
                        LCDSetPrintf(7 + Display_Line_X, 36 + Display_Line_Y, 'Multilogg Data')
                        
                    New_Recording = False
                
            else:

                if(New_Unrecording == True):
                    LCDCircle(400, 80, 50, 0, 1)
                    LCDSetColor(0, 0)
                    
                    if(Multi_Logging == False):
                        LCDSetPrintf(7 + Display_Line_X, 36 + Display_Line_Y, 'Recording Data')
                        
                    else:
                        LCDSetPrintf(7 + Display_Line_X, 36 + Display_Line_Y, 'Multilogg Data')
                        
                    New_Unrecording = False
            
            LCDSetColor(16777215, 0)
            LCDSetPrintf(16 + Display_Line_X + Display_Line_Extra, 0, 'Steering Angle: %d    Speed: %d    Display: %s' %(Steering_Angle, Speed, Lidar_Display_Modes[Display]))
            LCDSetPrintf(17 + Display_Line_X + Display_Line_Extra, 0, 'Maximum Speed Allowed: %d   Priority: %s' %(Max_Allow_Speed, Priority))
                
            Iterations += 1
            Speed_Past = Speed
        
        if(SIMULATION == False):
            
            if(Speed > Normal_Speed):
                ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
                ServoBlaster.flush()
                ServoBlaster.write('1=' + str(150) + '\n')
                ServoBlaster.flush()
                ServoBlaster.write('1=' + str(Brake_Speed) + '\n')
                ServoBlaster.flush()
                time.sleep(Hard_Brake_Power)
                
            ServoBlaster.write('0=' + str(Steering_Angle) + '\n')
            ServoBlaster.flush()
            ServoBlaster.write('1=' + str(150) + '\n')
            ServoBlaster.flush()

            
        else:
            Simulation_Drive(150, 150)
        
        if((Lidar_Drive_Check == True) and (Switch_Mode == 0)):
            End_SPS = time.time()
            Lidar_SPS = float(Iterations/(End_SPS - Start_Autonomy))
            
            LCDClear()
            LCDMenu('Continue', '', '', '')
            LCDSetPrintf(13, 0, 'Lidar Drive')
            LCDSetPrintf(15, 0, '[ModelCar-2] Scans/Second      : %.2f' %(Lidar_SPS))
            
            if(Start_Pause == False):
                Autonomy = float((1 - ((Interventions*5)/(End_SPS - Start_SPS - Total_Injury_Time)))*100)
                LCDSetPrintf(16, 0, '[ModelCar-2] Autonomy(Percent) : %.2f' %(Autonomy))
                
            else:
                LCDSetPrintf(16, 0, '[ModelCar-2] No Interventions Recorded')
                
            while(True):
                pygame.event.pump()
                KEY =  KEYRead()
                            
                if((KEY == 1) or (B_Button() == True)):
                    break
        
        time.sleep(0.2)
        Exit_Drive_Check = True
              
#=============================================================================

    elif((KEY == 8) or (X_Button() == True) or (Controller_Exit == True)):
        LCDClear()
        LCDSetColor(16777215, 0)
        LCDSetPrintf(16, 0, '[ModelCar-2] Exiting Program...')
        break

if(SIMULATION == False):
    ServoBlaster.close()
    
CAMRelease()
pygame.quit()
        
#=============================================================================

if(Logging_Performance == True):
    
    if(len(Manual) > 0):
        
        if(os.path.exists(Manual_Path) == True):
            Manual = np.concatenate((np.load(Manual_Path), np.array(Manual)), axis=None)
            np.save(Manual_Path, Manual)
        
        else:
            np.save(Manual_Path, Manual)
            
    if(len(Edges_Camera) > 0):
        
        if(os.path.exists(Edge_Camera_Path) == True):
            Edges_Camera = np.concatenate((np.load(Edge_Camera_Path), np.array(Edges_Camera)), axis=None)
            np.save(Edge_Camera_Path, Edges_Camera)
        
        else:
            np.save(Edge_Camera_Path, Edges_Camera)
            
    if(len(Nvidia_Camera) > 0):
        
        if(os.path.exists(Nvidia_Camera_Path) == True):
            Nvidia_Camera = np.concatenate((np.load(Nvidia_Camera_Path), np.array(Nvidia_Camera)), axis=None)
            np.save(Nvidia_Camera_Path, Nvidia_Camera)
        
        else:
            np.save(Nvidia_Camera_Path, Nvidia_Camera)
    
    if(len(Lidar) > 0):
        
        if(os.path.exists(Lidar_Path) == True):
            Lidar = np.concatenate((np.load(Lidar_Path), np.array(Lidar)), axis=None)
            np.save(Lidar_Path, Lidar)
        
        else:
            np.save(Lidar_Path, Lidar)
              
#=============================================================================
        
Image_Count = len(os.listdir(Image_Path)) - 2
Edge_Count = len(os.listdir(Edge_Path)) - 3
Scan_Count = len(os.listdir(Scan_Path))        
       
if(Image_Count == 0):
    
    if(os.listdir(Train_Image_Path) == []):
            os.system(User + 'rm -rf ' + Train_Image_Path) 
            
    if(os.listdir(Test_Image_Path) == []):
            os.system(User + 'rm -rf ' + Test_Image_Path) 
            
    if(os.listdir(Image_Path) == []):
        os.system(User + 'rm -rf ' + Image_Path)
    
if(Edge_Count == 0):
        
    if(os.listdir(Training_Path) == []):
            os.system(User + 'rm -rf ' + Training_Path) 
            
    if(os.listdir(Validation_Path) == []):
            os.system(User + 'rm -rf ' + Validation_Path) 
            
    if(os.listdir(Testing_Path) == []):
            os.system(User + 'rm -rf ' + Testing_Path)
        
    if(os.listdir(Edge_Path) == []):
        os.system(User + 'rm -rf ' + Edge_Path)

if(Scan_Count == 0):

    if(os.listdir(Scan_Path) == []):
            os.system(User + 'rm -rf ' + Scan_Path)    

if(os.listdir(CNN_Model_Path) == []):
        os.system(User + 'rm -rf ' + CNN_Model_Path)
        
if(os.listdir(Data_Path) == []):
        os.system(User + 'rm -rf ' + Data_Path)
        
#=============================================================================
