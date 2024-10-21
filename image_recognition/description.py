import urllib.request
import cv2
import numpy as np
import os

def create_pos_n_neg():
    for file_type in ['neg_small']:

        print(str(file_type))
        
        for img in os.listdir(file_type):

            if file_type == 'pos':
                line = file_type+'/'+img+' 1 0 0 50 50\n'
                with open('info.dat','a') as f:
                    f.write(line)
            elif file_type == 'neg_small':
                line = file_type+'/'+img+'\n'
                with open('neg_des.txt','a') as f:
                    f.write(line)
                    print(str(img))


create_pos_n_neg()

