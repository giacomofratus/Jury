#!/usr/bin/env python
# coding: utf-8

# In[24]:


import os
import numpy as np
import math
import matplotlib.pyplot as plt

d = 50  #diameter of wheel (mm)
W = 90 #width of robot (mm)
R = d/2 #radius of wheel  
w_max = 100*2*math.pi/60



class Arena:
    
    #constructor
    def __init__(self,size = [1000,1000], init_state = np.zeros(3), dt = 0.1):
        self.size = size
        #state = [x,y,theta(degrees)]
        self.state = init_state
        
        #Time scale
        self.time = dt
        
        #wheel angular velocities
        self.wR = 0.0
        self.wL = 0.0
        
        #history for plotting
        self.x_hist = []
        self.y_hist = []
        
        
    def pwmToAngular(self,pwmValue = 0.0):
        if (pwmValue < 0.05 and pwmValue > -0.05):
            return 0
        
        elif (pwmValue > 0.3):
            return (0.5*(pwmValue-0.3)+0.3)*w_max
        
        elif (pwmValue < -0.3):
            return (0.5*(pwmValue+0.3)-0.3)*w_max
        
        else:
            return pwmValue*w_max
    
    def state_update(self, i = np.zeros(2)):
        
        self.x_hist.append(self.state[0])
        self.y_hist.append(self.state[1])
        
        #inputs
        self.wL = self.pwmToAngular(i[1])
        self.wR = self.pwmToAngular(i[0])
        
        #current state
        x = self.state[0]
        y = self.state[1]
        theta = self.state[2]*math.pi/180
        
        #Time Scale
        T = self.time
        self.state[0] = x + 0.5*R*T*self.wR*math.cos(theta) + 0.5*R*T*self.wL*math.cos(theta)
        self.state[1] = y + 0.5*R*T*self.wR*math.sin(theta) + 0.5*R*T*self.wL*math.sin(theta)
        self.state[2] = ((180/math.pi)*(theta + R*T*self.wR/W - R*T*self.wL/W))%360
        
        #check out of bounds
        if (self.state[0] > self.size[0]):
            self.state[0] = self.size[0]

        if (self.state[0] < 0):
            self.state[0] = 0
            
        if (self.state[1] > self.size[1]):
            self.state[1] = self.size[1]
            
        if (self.state[1] < 0):
            self.state[1] = 0
  
        
    
    #Calculate euclidean distance from current state to coord = (x,y)
    def distance(self,coord = [0,0]):
        x_dist = (coord[0] - self.state[0])**2
        y_dist = (coord[1] - self.state[1])**2
        return (x_dist + y_dist)**(.5)
    
    #check if coord = (x,y) is inside the boundries
    def check_in(self,coord = [0,0]):
        if (coord[0] < 0 or coord[0] > self.size[0]):
            return False
        if (coord[1] < 0 or coord[1] > self.size[1]):
            return False
        return True
  
    
    #returns the observation; [df,dr,wc,hx,hy]
    def observation(self):
        #angle in radians
        theta = self.state[2]*math.pi/180
        tf = math.tan(theta)
        tr = math.tan(theta - (math.pi)/2)
        Width = self.size[0];
        Height = self.size[1];
        x0 = self.state[0];
        y0 = self.state[1];
        #df and dr as specified in the doc
        df = -1
        dr = -1
        # Case theta = 0
        if theta == 0:
            df = Width - x0
            dr = y0
        # Case theta = 90 degrees
        elif theta == (math.pi)/2:
            df = Height - y0
            dr = Width - x0
        # Case theta = 180 degrees  
        elif theta == math.pi:
            df = x0
            dr = Height - y0
        #Case theta = 270 degrees
        elif theta == 3*(math.pi)/2:
            df = y0
            dr = x0
        #Case the
        elif theta < math.pi/2:
            y = tf*(Width - x0) + y0
            if self.check_in([Width, y]) == True:
                df = self.distance([Width, y])
            else:
                x = x0 + (Height - y0)/tf
                df = self.distance([x, Height])
            y = tr*(Width - x0) + y0
            if self.check_in([Width, y]) == True:
                dr = self.distance([Width, y])
            else:
                x = x0 - (y0)/tr
                dr = self.distance([x, 0])
                
        elif theta < math.pi:
            y = tf*(0 - x0) + y0
            if self.check_in([0, y]) == True:
                df = self.distance([0, y])
            else:
                x = x0 + (Height - y0)/tf
                df = self.distance([x, Height])
            y = tr*(Width - x0) + y0
            if self.check_in([Width, y]) == True:
                dr = self.distance([Width, y])
            else:
                x = x0 + (Height - y0)/tr
                dr = self.distance([x, Height])
                
        elif theta < 3*math.pi/2:
            y = tf*(0 - x0) + y0
            if self.check_in([0, y]) == True:
                df = self.distance([0, y])
            else:
                x = x0 + (0 - y0)/tf
                df = self.distance([x, 0])
            y = tr*(0 - x0) + y0
            if self.check_in([0, y]) == True:
                dr = self.distance([0, y])
            else:
                x = x0 + (Height - y0)/tr
                dr = self.distance([x, Height])
                
        elif theta < 2*math.pi:
            y = tf*(Width - x0) + y0
            if self.check_in([Width, y]) == True:
                df = self.distance([Width, y])
            else:
                x = x0 + (0 - y0)/tf
                df = self.distance([x, 0])
            y = tr*(0 - x0) + y0
            if self.check_in([0, y]) == True:
                dr = self.distance([0, y])
            else:
                x = x0 - (y0)/tr
                dr = self.distance([x, 0])
                
        # angular rate )
        wC = (self.wR*R - self.wL*R)/W
    
        #magnetic field components
        Hx = -1*math.cos(theta)
        Hy = math.sin(theta)
        return [df,dr,wC,Hx,Hy]
  
    
    def get_state(self):
        return self.state
    
    def plot_state(self):
        plt.scatter(self.x_hist,self.y_hist)
        plt.xlim([0,self.size[0]])
        plt.ylim([0,self.size[1]])
        plt.grid(True)
    
    
  


# In[84]:


def main():
    ArenaTest = Arena(init_state = [200,600,0])

    for i in range(0,30):
        #print("State: ")
        #print(ArenaTest.get_state())
        #print("Observation: ")
        print(ArenaTest.observation())
        ArenaTest.state_update(i = [.5,.5])
    
    for i in range(0,20):
        #print("State: ")
        #print(ArenaTest.get_state())
        #print("Observation: ")
        print(ArenaTest.observation())
        ArenaTest.state_update(i = [0.4,.1])
        
    for i in range(0,50):
        #print("State: ")
        #print(ArenaTest.get_state())
        #print("Observation: ")
        print(ArenaTest.observation())
        ArenaTest.state_update(i = [.2,.3])
        
    
    ArenaTest.plot_state()
    
        
    


# In[85]:



main()


# In[ ]:





# In[ ]:




