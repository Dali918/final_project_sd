import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision.transforms as transforms
from torch.utils.data import DataLoader
from torch.utils.data import Dataset
import cv2
import numpy as np
#import os
#import PIL
import matplotlib.pyplot as plt
#import random
#import math
#import tqdm

#from model.unet import UNet
from Perception.utils.utils import find_edge_channel
#device = "cuda" if torch.cuda.is_available() else "cpu"
device = "cpu"

'''
trained_model = "./epoch_39.pt"
dir = "./data/custom/inputs/"
'''


def evaluate(model, image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges,edges_inv = find_edge_channel(image)
    output_image = np.zeros((gray.shape[0],gray.shape[1],3),dtype=np.uint8)
    output_image[:,:,0] = gray
    output_image[:,:,1] = edges
    output_image[:,:,2] = edges_inv
    transform = transforms.Compose([transforms.ToPILImage(),
                                    transforms.Resize((180,330)),
                                    transforms.ToTensor()])
    test_img = transform(output_image).unsqueeze(0).to(device)
    output = model(test_img)
    pred = torch.sigmoid(output)
    # pred = torch.where(pred > 0.5 , 255, 0).type(torch.uint8)
    pred = torch.where(pred > 0.5 , 1, 0)
    pred = pred.detach().cpu().squeeze().numpy()


    pred = cv2.resize(pred.astype(float) , (gray.shape[1], gray.shape[0]),cv2.INTER_AREA)

    image[pred.astype(bool),0] = 155
    image[pred.astype(bool),1:] = 0 


    return pred

'''
model = UNet()
save_dict = torch.load(trained_model, map_location=device)
model.load_state_dict(save_dict["model"])



model.to(device)
model.eval()

datas = os.listdir(dir)
for i in datas:
    path = dir + i
    

    image = cv2.imread(path)
    print(image.shape)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges,edges_inv = find_edge_channel(image)
    output_image = np.zeros((gray.shape[0],gray.shape[1],3),dtype=np.uint8)
    output_image[:,:,0] = gray
    output_image[:,:,1] = edges
    output_image[:,:,2] = edges_inv
    transform = transforms.Compose([transforms.ToPILImage(),
                                    transforms.Resize((180,330)),
                                    transforms.ToTensor()])
    test_img = transform(output_image).unsqueeze(0).to(device)
    output = model(test_img)
    pred = torch.sigmoid(output)
    # pred = torch.where(pred > 0.5 , 255, 0).type(torch.uint8)
    pred = torch.where(pred > 0.5 , 1, 0)
    pred = pred.detach().cpu().squeeze().numpy()


    pred = cv2.resize(pred.astype(float) , (1080, 720),cv2.INTER_AREA)
    print(pred.shape)

    image[pred.astype(bool),0] = 155
    image[pred.astype(bool),1:] = 0 


    print(image.shape)
    print(pred.shape)

    



    cv2.imshow("asd",image)
    cv2.waitKey()
'''
