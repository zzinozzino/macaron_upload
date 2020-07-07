#!/usr/bin/env python
#-*-coding:utf-8-*_

## talker demo that published std_msgs/ColorRGBA messages
## to the 'color' topic. To see these messages, type:
## rostopic echo color
## this demo shows some of the more advanced APIs in rospy
import os, sys,time
from scipy.integrate import quad
from scipy.interpolate import InterpolatedUnivariateSpline

import numpy as np
import math
import roslib
roslib.load_manifest('macaron')
import rospy

import matplotlib.pyplot as plt
from macaron.msg import Floats
from macaron.msg import base_frame

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
PI=math.acos(-1)

present_num=0

'''
my_y=200043
my_x=550928



    plt.plot(ix, iy, 'bo', label='lins')
    plt.grid(True)
    plt.show()



    ##initial value setting
    check_val = 5000000
    r_detect = 0
    max_track_val = b.shape[0]
    started = False
    pub = rospy.Publisher('/track_point', Float64MultiArray, queue_size=3)
    now_x = now_y = rel_x = rel_y = started = final_angle = 0
    find_activated = 0
    index_value_x = 0
    s_x=s_y=0
'''
class GPSTxT_leader():
        def __init__(self):
                self.base_frame=rospy.Publisher('base_frame',base_frame,queue_size=3)
                self.map_initializer()
                self.GPSTxT_leader()
        
        def map_initializer(self):
            global p,poly,dt,Nsample,p_length,leng
            a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/8line.txt",delimiter=',',dtype='double')
            line_name="catkin_ws/src/macaron/scripts/eightlight/8line_npy"+".npy"
            np.save(line_name,a)
            b=np.load(line_name)
        
        
            print(b)
           # plt.plot(b[:,0],b[:,1])
           # plt.grid(True)
        
            #from std_msgs.msg import ColorRGBA
            P=b.T
            leng=int(P.size/2)

        
            alpha = 0.5 # 접선벡터의 가중치. 이 값에 따라 곡선의 장력이 결정된다.
            dp = np.zeros((2, leng))
        
            for xy in range(0,2):
                for a in range(1,leng-1):
                    dp[xy, a] = (1 - alpha)*(P[xy, a+1] - P[xy, a-1])
        
            print(dp)
            dt = 0.1
            Nsample = int(1/ dt)
            p = np.zeros((2, Nsample * (leng-1) + 1))
            poly=np.zeros((4,leng,2))
            #print(p.shape)
            p_length=int(p.size/2)
            for big_node in range(0,leng -1):
                for xy in range(0,2):
                    t = 0
                    poly[0][big_node][xy]= +2*P[xy,big_node] -2*P[xy,big_node+1]+1*dp[xy,big_node]+1*dp[xy,big_node + 1]
                    poly[1][big_node][xy]= -3*P[xy,big_node] +3*P[xy,big_node+1]-2*dp[xy,big_node]-1*dp[xy,big_node + 1]
                    poly[2][big_node][xy] = dp[xy,big_node]
                    poly[3][big_node][xy] = P[xy,big_node]
                    for small_node in range(0,Nsample):
                        p[xy][small_node + Nsample * big_node] = poly[0][big_node][xy]*t*t*t + poly[1][big_node][xy]*t*t + poly[2][big_node][xy]*t + poly[3][big_node][xy]
                        t=t+dt
        
            #print(poly)
        
            p[0, Nsample * (leng-1)] = P[0, leng-1]
            p[1, Nsample * (leng-1)] = P[1, leng-1]
        
           # plt.plot(p[0][:],p[1][:], 'r', label='f(x)')
           # plt.grid(True)
            #plt.show()

        def listener(self):
            my_State=NavSatFix()
            my_State=rospy.wait_for_message("/fix", NavSatFix)
            lon =my_State.longitude
            lat =my_State.latitude
        
            st_lat = 38.0
            st_lon = 127.0
            k_0 =1.0 
            a = 6378137.0 
            b = 6356752.31 
            f =(a-b)/a 
            d_y = 200000.0 
            d_x = 600000.0 
        
            e = (a*a - b*b) / (a*a) 
            e_2 = (a*a - b*b) / (b*b) 
            M = a * ((1 - e/4.0 - 3.0*e*e/64.0 - 5.0*pow(e,6.0)/256.0)* math.radians(lat) - (3.0*e/8.0 + 3.0*e*e/32.0 + 45.0*e*e*e/1024.0)* math.sin(2* math.radians(lat)) + (15.0*e*e/256.0 + 45.0*e*e*e/1024.0)* math.sin(4.0* math.radians(lat)) - 35.0*e*e*e/3072.0* math.sin(6.0* math.radians(lat))) 
            C = (e / (1-e))* math.cos( math.radians(lat)) 
            T = pow( math.tan( math.radians(lat)),2) 
            A = ( math.radians(lon - st_lon))* math.cos( math.radians(lat)) 
            N = a/ math.sqrt(1-(e)*( math.sin( math.radians(lat))* math.sin( math.radians(lat)))) 
            M_0 = a * ((1 - e/4.0 - 3.0*e*e/64.0 - 5.0*e*e*e/256.00)* math.radians(st_lat) - (3.0*e/8.0 + 3.0*e*e/32.0 + 45.0*e*e*e/1024.0)* math.sin(2.0* math.radians(st_lat)) + (15.0*e*e/256.0 + 45.0*e*e*e/1024.0)* math.sin(4.0* math.radians(st_lat)) - 35.0*e*e*e/3072.0* math.sin(6.0* math.radians(st_lat))) 
            y_tm = (d_y+k_0*N*(A + (A*A*A/6.0)*(1-T+C) + (A*A*A*A*A/120.0) * (5.0 - 18.0*T + T*T + 72.0*C - 58.0*e_2))) 
            x_tm = (d_x + k_0*(M - M_0 + N* math.tan(math.radians(lat))*(A*A/2.0 + (A*A*A*A/24.0)*(5.0-T+9.0*C+4.0*C*C)+(A*A*A*A*A*A/720.0)*(61.0-58.0*T+T*T+600.0*C-330.0*e_2)))) 
            print(x_tm,y_tm)
            return x_tm,y_tm

        def GPSTxT_leader(self):
                global distance_difference,present_num,search
                base=base_frame()
                while not rospy.is_shutdown():
                        my_x,my_y=self.listener()

                        path_leng=0
                        search=0
                        a=0
                        a_grad=0
                        distance_difference=100

                        while(abs(distance_difference)>0.05):
                                if present_num>p_length-1:
                                        present_num=Nsample * (leng-1)
                        
                                elif present_num<0:
                                        present_num=0
                                
                                distance_difference=(my_x-p[0][present_num+1])**2 + (my_y-p[1][present_num+1])**2-(my_x-p[0][present_num])**2-(my_y-p[1][present_num])**2
                                search=search+1
                    
                                a=-distance_difference/(1+search/100)
                    
                                if(a>-1 and a<0):
                                        a=-1
                                elif(a<1 and a>0):
                                        a=1
                                
                                
                                present_num=present_num+int(a)

                                #print(search,present_num,distance_difference,a)
                                if(search==100):
                                        print("binzino")
                                        break
                    
                    #    plt.plot(my_x,my_y, 'ro', label='position')
                    
                        Fsum=0
                        st=[0]
                        s=[0]
                    
                        while(Fsum<10):
                                big_node=int((present_num+path_leng)/Nsample)
                                ft = lambda t: ((3* poly[0][big_node][1]*t*t + 2*poly[1][big_node][1]*t + poly[2][big_node][1])**2+(3* poly[0][big_node][0]*t*t + 2*poly[1][big_node][0]*t + poly[2][big_node][0])**2)**0.5
                                for small_node in range(0,Nsample):
                                        F,err=quad(ft, float(small_node)/Nsample,float(small_node+1)/Nsample)
                                        Fsum=Fsum+F
                                        path_leng=path_leng+1
                                        if Fsum>10:
                                                break
                                        elif (present_num+path_leng)>(Nsample * (leng-1)):
                                                break
                                        s.append(Fsum)
                    
                                #s=[Fsum]
                        
                        distance=((my_x-p[0][present_num])**2 + (my_y-p[1][present_num])**2)**0.5      
                        fpx=np.poly1d(np.polyfit(s,p[0][present_num:present_num+path_leng],3))
                        fpy=np.poly1d(np.polyfit(s,p[1][present_num:present_num+path_leng],3))
                        ix=fpx([0,1,2,3,4,5,6,7,8,9,10])
                        iy=fpy([0,1,2,3,4,5,6,7,8,9,10])
                        iangle=[]
                        idistance=[]
                        for scurve in range(0,Nsample):
                                iangle.append(math.atan2(iy[scurve+1]-iy[scurve],ix[scurve+1]-ix[scurve])*180/PI)
                                idistance.append(math.sqrt((iy[scurve+1]-iy[scurve])**2+(ix[scurve+1]-ix[scurve])**2))
                        
                        print(distance)
                        print(ix)
                        print(iy)
                        print(iangle)
                        print(present_num)
                        print(idistance)
                        
                        base.distance=distance
                        base.s_x=ix
                        base.s_y=iy
                        base.s_a=iangle
                        base.s_d=idistance

                        self.base_frame.publish(base)

def main():
        rospy.init_node('jGPSj')
        try:
                GPSTxT_leader()

        except rospy.ROSInterruptException:
                pass


## start code
if __name__ == '__main__':
        main()



'''


    for big_node in range(0,leng-1):
            ft = lambda t: ((3* poly[0][big_node][1]*t*t + 2*poly[1][big_node][1]*t + poly[2][big_node][1])**2+(3* poly[0][big_node][0]*t*t + 2*poly[1][big_node][0]*t + poly[2][big_node][0])**2)**0.5
            l_hat=np.linspace(0,14.4,10)
            t1=0.
            t2=1.
            tmid=(t1+t2)/2
            result=100.
            for node in range(1,11):
                    while abs(result)>0.001:
                            result=14.4-quad(ft,t1,tmid)
                            if(result>0):
                                    tmid=(t2+tmid)/2
                            else :
                                    tmid=(t1+tmid)/2
                    st.append(tmid+big_node*100)
                    Fsum=Fsum+quad( ft, t1,tmid)


    print(st)
    print(s)



    itx=InterpolatedUnivariateSpline(s,x,k=4)
    ity=InterpolatedUnivariateSpline(s,y,k=4)

    ix=itx()
    iy=ity(np.linspace(0,155,1))
    plt.plot(x,y, 'r', label='f(x)')
    plt.plot(ix, iy, 'b', label='interpolation')
    plt.legend(loc=0)
    plt.grid(True)
    plt.show()







    V = [ [0:0.01:10]', x.', y.' ] 
    L = Fsum 
    s0 = 0 
    N = length(s) 
    Xq = np.linspace(s0,L,(leng-1)*Nsample)  % equally spaced indices

     

    Vq = interp1(X,V(1:1000,:),Xq) 

    xs = Vq(:,2) 

    ys = Vq(:,3) 
'''