#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
import math
from math import radians, copysign, sqrt, pow, pi, atan2, sin, cos, tan
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray, Float64, Bool
import threading
import time

min_map_x = 0.0
max_map_x = 3.0
min_map_y = 0.0
max_map_y = 3.0

current_z_1 = 0.0
current_x_1 = 0.0

class Control():

    def __init__(self, class_name):
        self.name = class_name
        rospy.init_node('control_control_1', anonymous=False)

        self.goal_xx = [] #함수 초기화 
        self.goal_yy = []
        self.current_x = 0.0
        self.current_y= 0.0
        self.current_z = 0.0
        self.obstacle_xy = [0.0, 0.0]
        self.prev_time = 0
        self.current_time = 0
        self.dt = 0
        self.dt_1 = 0
        self.d = 0
        self.prev_d = 0
        self.first_run = True
        self.current_angle = 0 
        self.prev_angle = 0
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.object_sub = rospy.Subscriber('/Dynamic_Stop', Bool, self.lidarcallback)
        self.btn_sub = rospy.Subscriber('/btn', Bool, self.btncallback)
        self.odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback)
        self.coord_x_sub = rospy.Subscriber('/array_x', Float64MultiArray, self.callback_x)
        self.coord_y_sub = rospy.Subscriber('/array_y', Float64MultiArray, self.callback_y)

    def run(self):

        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(30)
        
        prev_angle = 0
        rotation_z_diff = 0
        if(len(self.goal_xx) != 0 and len(self.goal_yy) != 0 and self.current_x != 0.0): #만약에 글로벌패스와 좌표가 달라지면 실행 x

            while not rospy.is_shutdown(): #stanelymehod
                
                weight1 = 1.5 #차선 곡률 오차 gain값(알파1) #실험값
                weight2 = 8.0 #횡방향 오차 gain값(알파2)
                weight3 = 1.5 #차선 곡률 오차 미분 gain값
                weight4 = 15.0 #횡방향 오차 미분 gain값



                self.current_time = rospy.get_time()
                self.dt = self.current_time - self.prev_time
                # self.prev_time = self.current_time




                goal_x = self.goal_xx[0] #local path 가장 가까운 점
                goal_y = self.goal_yy[0] 
                goal_x1 = self.goal_xx[1] #두번째점
                goal_y1 = self.goal_yy[1]
                target_X = self.goal_xx[9] #local path 목표점. 사실 distance 구할 때만 사용함.
                target_Y = self.goal_yy[9]

                #------------------------------------------------------------- 차선 곡률 오차 계산 시작
                target_Z = atan2(goal_y1-goal_y,goal_x1-goal_x) #차선 곡률 계산

                #print("절대 각도 :", target_Z)

                if(target_Z < 0):
                    target_Z = target_Z + 2.0*pi #세타 방향 양값으로 변환


                


                rotation_z = 0
                if(target_Z > self.current_z):                  #차선 곡률과 차량 헤딩값 오차 계산. P제어만 넣음
                    if(target_Z > self.current_z + pi):
                        self.current_angle = -(2*pi - target_Z + self.current_z)
                        rotation_z = weight1* self.current_angle #우측으로 돌아서 -(2*pi)
                        rotation_z_diff = weight3*(self.current_angle - self.prev_angle)/self.dt

  
                    else:
                        self.current_angle = target_Z - self.current_z
                        rotation_z = weight1* self.current_angle
                        rotation_z_diff = weight3*(self.current_angle - self.prev_angle)/self.dt

                        
                else:
                    if (target_Z+pi < self.current_z):
                        self.current_angle = 2*pi-self.current_z+target_Z
                        rotation_z = weight1*self.current_angle
                        rotation_z_diff = weight3*(self.current_angle - self.prev_angle)/self.dt
    
                    else :
                        self.current_angle = -(self.current_z-target_Z)
                        rotation_z = weight1* self.current_angle
                        rotation_z_diff = weight3*(self.current_angle - self.prev_angle)/self.dt

                        
                # print("self.dt: ", self.dt)
                #------------------------------------------------------------- 차선 곡률 오차 계산 끝

                #------------------------------------------------------------- 횡방향 오차 계산 시작
                if(goal_x1 == goal_x): #예외처리 분모에 0 들어간느 경우 85번줄
                    self.d = abs(goal_x - self.current_x)            #차선이 y축에 평행하다면(x 좌표로 수정)
                else:
                    a = -(goal_y1-goal_y)/(goal_x1-goal_x)      #점과 직선과의 거리 계산
                    b = 1
                    c = (goal_y1-goal_y)/(goal_x1-goal_x)*goal_x-goal_y
                    self.d = abs((a*self.current_x + b*self.current_y + c)/sqrt(a*a+b*b))    # d : 점과 직선 사이 거리값

                vector_global_path = [goal_x1 - goal_x, goal_y1 - goal_y]               # ccw. 차가 차선의 오른편에 있는지, 왼편에 있는지 확인(벡터음수양수로 확인)
                vector_global_turtle = [self.current_x - goal_x, self.current_y - goal_y]
                vector_product = vector_global_turtle[0]*vector_global_path[1] - vector_global_turtle[1]*vector_global_path[0] #벡터 외적값 음수 양수 파악

                if(vector_product > 0):                         #차가 차선의 오른편에 있다
                    move_cmd.angular.z = rotation_z + weight2*self.d + rotation_z_diff + weight4*(self.d-self.prev_d)/self.dt

                else:                                           #차가 차선의 왼편에 있다
                    move_cmd.angular.z = rotation_z - weight2*self.d + rotation_z_diff - weight4*(self.d-self.prev_d)/self.dt

                #------------------------------------------------------------- 횡방향 오차 계산 끝
                print('d:',self.d)
                print('pre_d:',self.prev_d)

                # self.prev_d = self.d

            
                
                if(move_cmd.angular.z < 0): #각도 음수 양수로 회전 판단
                    print("우회전")
                else:
                    print("좌회전")
               
                distance = sqrt(pow(target_X - self.current_x, 2) + pow(target_Y - self.current_y, 2))
                # print("distance: ", distance)
                
                move_cmd.linear.x = max(distance/2.5, 0.15) #속도 0.2보다는 빠르게 달리자
                
                self.cmd_vel.publish(move_cmd)

                r.sleep()
                # time.sleep(0.2)z
        self.cmd_vel.publish(Twist()) #터틀봇 꺼지면 정지 Twist는 0값


    def shutdown(self):
        print("stop!!")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def callback(self, msg): # 터틀봇 현재 위치 받아오는 함수 weight2*d 
        current_pose = msg
        self.current_x = current_pose.pose.pose.position.x # 현재 좌표값 저장
        self.current_y = current_pose.pose.pose.position.y
        rotation = current_pose.pose.pose.orientation
        self.current_z = self.quaternion_to_euler_angle(rotation) #로테이션값(터틀봇이 보고있는 방향)
        
        global current_z_1
        global current_x_1

        if current_z_1 != self.current_z:
            print("-----------------------------------------")
            self.prev_time = self.current_time
            current_z_1 = self.current_z
            self.prev_d = self.d
            self.prev_angle = self.current_angle

    def callback_x(self, msg):
        self.goal_xx = msg.data #local path x좌표  planning path에서 보냄 경로에서 지나는 점 20개 인덱스로 

        # print("goal_x : " , goal_x)
        
    def callback_y(self, msg):
        self.goal_yy = msg.data
        # print("goal_y : ", goal_y)
          
    def quaternion_to_euler_angle(self, msg): #current z 동기화 , 좌표계 변환 
        x = msg.x
        y = msg.y
        z = msg.z
        w = msg.w

        ysqr = y * y
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = atan2(t3, t4)

        if(Z < 0):
            Z = Z + 2*pi
        
        return Z
    
    def lidarcallback(self, msg):
        self.dynamicObject = msg.data

        if(self.dynamicObject == 1):
            self.cmd_vel.publish(Twist())
            

    def btncallback(self, msg):
        self.btnPushed = msg.data

        if(self.btnPushed == 1):
            self.cmd_vel.publish(Twist())
            
    

if __name__ == '__main__':
    try:
        robot1=Control("robot1")
        r2 = rospy.Rate(5)
        print("control_1 on. waiting for local path")

        while not rospy.is_shutdown():
            robot1.run()
            r2.sleep()

    except:
        rospy.loginfo("shutdown program.")