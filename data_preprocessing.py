#!/usr/bin/env python
import numpy as np
import os
import cv2 
from os.path import join, exists, dirname, abspath
import shutil
#Image_Path='/home/zhihuilai/Erik_WorkStation/ModelCar-2-Ubuntu/Data/Images_Car/CME_Images'
Image_Path='/media/zhihuilai/RAID1/Erik-ModelCar-Data/CME_Images_Processed'
#index_list_path='/home/zhihuilai/Erik_WorkStation/ModelCar-2-Ubuntu/Data/Images_Car/index_list'
#index_list_final_path='/home/zhihuilai/Erik_WorkStation/ModelCar-2-Ubuntu/Data/Images_Car/index_list.npy'
Sorted_Data_Path='/media/zhihuilai/RAID1/Erik-ModelCar-Data/CME_Images_Processed'

# Write some Text
font=cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText=(0,200)
fontScale=0.6
fontColor=(100,100,200)
lineType=1

list_b=[736, 846, 1300, 1410, 1854, 1964, 2976, 3086, 3548, 3658, 4676, 4786, 10248, 10358, 11428, 11538, 12536, 12646, 13080, 13190, 15356, 
15466, 17112, 17222, 17696, 17806, 18900, 19010, 21452, 21562, 22074, 22184, 22714, 22824, 26488, 26598, 27778, 27888, 30330, 30440, 434, 544, 
986, 1096, 1516, 1626, 2048, 2158, 2578, 2688, 3120, 3230, 3662, 3772, 4200, 4310, 4762, 4872, 5310, 5420, 5868, 5978, 6410, 6520, 6956, 7066, 
7518, 7628, 8068, 8178, 8598, 8708, 9128, 9238, 9652, 9762, 10184, 10294, 10722, 10832, 11264, 11374, 11792, 11902, 12316, 12426, 12858, 12968, 
13364, 13474, 13904, 14014, 14176, 14286, 14450, 14560, 14996, 15106, 15526, 15636, 16056, 16166, 16576, 16686, 17126, 17236, 17678, 17788, 17960, 
18070, 18232, 18342, 18788, 18898, 19344, 19454, 19896, 20006, 20454, 20564, 21026, 21136, 21590, 21700, 22144, 22254, 22706, 22816, 23250, 23360, 
23808, 23918, 24094, 24204, 24380, 24490, 24944, 25054, 25504, 25614, 26066, 26176, 26656, 26766, 26948, 27058, 27250, 27360, 27830, 27940, 18, 128, 
588, 698, 1728, 1838, 2908, 3018, 4060, 4170, 4658, 4768, 5262, 5372, 5846, 5956, 144, 254, 682, 792, 1256, 1366, 2398, 2508, 3540, 3650, 4090, 4200, 
4350, 4460, 4636, 4746, 5202, 5312, 5796, 5906, 6078, 6188, 6358, 6468, 6938, 7048, 9338, 9448, 9932, 10042]

#part one
index_list=[]
for Filepath in sorted(os.listdir(Image_Path), key=lambda x: int(x)):

    Search_Folder = os.path.join(Image_Path, Filepath)
    print(Search_Folder)                                              
    for image in sorted(os.listdir(Search_Folder), key=lambda x: int(x.split("_")[2])):
        if int(image.split("_")[2])%2==0:
            #print(image.split("_")[2])
            img_array = np.load(os.path.join(Search_Folder,image),allow_pickle=True)
            if img_array[2]<140:
                fontColor=(0,255,0)
            else:
                fontColor=(100,100,200)
            img=cv2.putText(img_array[0],image, bottomLeftCornerOfText, font, fontScale,fontColor,lineType)
            cv2.imshow('Record',img)
            key=cv2.waitKey()
            #if key== ord('s'):
            #    index_list.append(int(image.split("_")[2])-10)
            #    print("checkpoint: ", int(image.split("_")[2])-10)
            if key==27: break
    #np.save(index_list_path, np.array(index_list))
    print(index_list)
cv2.destroyAllWindows()



# if exists(Image_Path):
#     #index_list_final=np.load(index_list_final_path,allow_pickle=True)
#     i=0
#     x=0
#     #current_index=index_list_final[i][0]
#     #current_category=index_list_final[i][1]
#     for Folders in os.listdir(Image_Path):
#         Search_Folder = join(Image_Path, Folders)
#         for image in sorted(os.listdir(Search_Folder), key=lambda x: int(x.split("_")[2])):  
#             if int(image.split("_")[2])==list_b[i]:
#                 new_folder=join(Sorted_Data_Path,str(x))
#                 os.makedirs(new_folder)
#                 i=i+1
#             elif int(image.split("_")[2])==0:
#                 new_folder=join(Sorted_Data_Path,str(x))
#                 os.makedirs(new_folder)
#             if i%2==0:
#                 shutil.copyfile(join(Search_Folder,image), join(new_folder,image))
#             else:
#                 img_array = np.load(os.path.join(Search_Folder,image),allow_pickle=True)
#                 img=img_array[0]
#                 img = cv2.flip(img, 1)
#                 speed=img_array[1]
#                 steering=300-img_array[2]
#                 new_img_array = [img, speed, steering]
#                 #new_image_name=str(image.split("_")[0:4])+'_'+str(steering)+'.npy'
#                 new_image_name=image.split("_")[0]+'_'+image.split("_")[1]+'_'+image.split("_")[2]+'_'+image.split("_")[3]+'_'+str(steering)+'.npy'
#                 new_image_path=os.path.join(new_folder,new_image_name)
#                 np.save(new_image_path, np.array(new_img_array))
#             x=x+1
# #print(os.path.getsize(Sorted_Data_Path))


# for Folder in os.listdir(Sorted_Data_Path):
#     Search_Folder = join(Sorted_Data_Path, Folder)
#     for File in sorted(os.listdir(Search_Folder), key=lambda x: int(x.split("_")[1])):

#         #shutil.copyfile(join(Search_Folder,File), join(Selected_Data_Path,File))
#         if int(File.split("_")[1])==index_list_final[i][0]:
#             i=i+1

# list_a=[736, 1300, 1854, 2976, 3548, 4676, 10248, 11428, 12536, 13080, 15356, 17112, 17696, 18900, 21452, 22074, 22714, 26488, 27778, 30330, 434, 
# 986, 1516, 2048, 2578, 3120, 3662, 4200, 4762, 5310, 5868, 6410, 6956, 7518, 8068, 8598, 9128, 9652, 10184, 10722, 11264, 11792, 12316, 12858, 
# 13364, 13904, 14176, 14450, 14996, 15526, 16056, 16576, 17126, 17678, 17960, 18232, 18788, 19344, 19896, 20454, 21026, 21590, 22144, 22706, 23250, 
# 23808, 24094, 24380, 24944, 25504, 26066, 26656, 26948, 27250, 27830, 18, 588, 1728, 2908, 4060, 4658, 5262, 5846, 144, 682, 1256, 2398, 3540, 4090, 
# 4350, 4636, 5202, 5796, 6078, 6358, 6938, 9338, 9932]
# list_b=[]
# for i in list_a:
#     list_b.append(i)
#     list_b.append(i+110)
# print(list_b)
