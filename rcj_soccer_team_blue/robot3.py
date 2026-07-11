import math
from math import atan , cos, sin
from re import L
import struct
from turtle import distance, heading
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot3(RCJSoccerRobot):

    def __init__(self, robot):
        self.flg=0
        super(MyRobot3,self).__init__(robot)
    def receipt_task(self):  # receipt task robot for left or right motor
        if(self.team == 'B'):
            return 1
        else:
            return 2

    def convert_head(self, heading):  # convert head to Standard
        task = self.receipt_task()
        heading = int(heading * 180 / math.pi)
        if (heading >= 0 and heading <= 180):
            heading = 360 - heading
        elif (heading < 0 and heading >= -90):
            heading = -heading
        elif(task == 1):
            heading = 360 - heading
        else:
            heading = -heading
        if (heading == 360):
            heading = 0
        return heading

    # convrt angle between ball and robot
    def convert_ball_angle(self, angle1, angle2, direction):

        ball_angle = 55555
        angle1 = int(angle1 * 100)
        angle2 = int(angle2 * 100)
        if(angle1 > 0 and angle1 <= 99 and angle2 < 0 and angle2 >= -99):
            if(angle2 == 0):
                ball_angle = 0
            else:
                ball_angle = (-angle2 * 90) / 99
        elif(angle1 >= 0 and angle1 < 99 and angle2 > 0 and angle2 <= 99):
            ball_angle = ((angle1 * 89) / 98) + 270
            if(ball_angle == 360):
                ball_angle = 0
        elif(angle1 < 0 and angle1 > -99 and angle2 > 0 and angle2 < 99):
            ball_angle = ((angle2 * 89) / 98) + 180
        elif(angle1 < 0 and angle1 >= -99 and angle2 <= 0 and angle2 > -99):
            ball_angle = ((-angle1 * 90) / 99) + 90
        ball_angle = int(ball_angle)
        if(ball_angle == 55555 and direction == 0):
            ball_angle = 0
        return ball_angle

    # set head robot to direction ball
    def set_head_direction_ball(self, angle, heading, speed_l,speed_r, speed2_l,speed2_r):
        sp = 0
        tolerance = int(((speed_r * 7) / 10) + 1.5)
        #tolerance=5
        tolerance_right = angle + tolerance
        tolerance_left = angle - tolerance
        if(tolerance_left < 0):
            tolerance_left = 360 + tolerance_left
            sp = 1
        if(tolerance_right > 359):
            tolerance_right = tolerance_right - 360
            sp = 1
        if(heading < tolerance_right and heading > tolerance_left and sp == 0):
            self.left_motor.setVelocity(speed2_l)
            self.right_motor.setVelocity(speed2_r)
        elif(sp == 1 and (heading < tolerance_right or heading > tolerance_left)):
            self.left_motor.setVelocity(speed2_l)
            self.right_motor.setVelocity(speed2_r)
        else:
            task = self.receipt_task()
            if(task == 1):  # blue team
                if(angle >= heading):
                    if(angle - heading <= 180):
                        self.left_motor.setVelocity(-speed_l)
                        self.right_motor.setVelocity(speed_r)
                    else:
                        self.left_motor.setVelocity(speed_l)
                        self.right_motor.setVelocity(-speed_r)
                else:
                    if(heading - angle <= 180 ):
                        self.left_motor.setVelocity(speed_l)
                        self.right_motor.setVelocity(-speed_r)
                    else:
                        self.left_motor.setVelocity(-speed_l)
                        self.right_motor.setVelocity(speed_r)
            else:  # yellow team
                if(angle >= heading):
                    if(angle - heading <= 180):
                        self.left_motor.setVelocity(-speed_l)
                        self.right_motor.setVelocity(speed_r)
                    else:
                        self.left_motor.setVelocity(speed_l)
                        self.right_motor.setVelocity(-speed_r)
                else:
                    if(heading - angle <= 180):
                        self.left_motor.setVelocity(speed_l)
                        self.right_motor.setVelocity(-speed_r)
                    else:
                        self.left_motor.setVelocity(-speed_l)
                        self.right_motor.setVelocity(speed_r)

    def convert_distance(self, distance):  # convert distance to 5-25

        if(distance >= 100):  # 100-255
            return (((distance - 100) * 4) / 155) + 20
        elif(distance > 23):  # 23-100
            return (((distance - 23) * 4) / 77) + 16
        else:  # 2-23
            return (((distance - 2) * 16) / 21) + 1

    def convert_distance_by_coordinate(self, distance):  # convert distance to 0-0.55

        if(distance >= 105):  # 105-255  to   0.05-0.1
            
            return (0.55 - ((((distance - 105) * 0.05) / 150) + 0.45))
        elif(distance > 45):  # 45-105   to   0.1-0.15
            
            return 0.55 - ((((distance - 45) * 0.05) / 60) + 0.4)
        elif(distance > 11):  # 11-44   to   0.2-0.4
           
            return 0.55 - ((((distance - 11) * 0.2) / 33) + 0.2)
        else:  # 2-11    to   0-0.2
            
            return 0.55 - (((distance - 2) * 0.2) / 9)

    # get angle between ball and earth
    def get_angle_ball_earth(self, heading, ball_angle):

        max = 360 - heading
        if(ball_angle <= max):
            return heading + ball_angle
        else:
            return heading - (360 - ball_angle)

    def soft_motion(self, right, left):

        self.left_motor.setVelocity(right)
        self.right_motor.setVelocity(left)

    def angle_gate(self,y_robot,x_robot):
        x_gate=0
        y_gate=-0.7
        delta__x=abs(x_robot)
        delta__y=abs(y_robot+0.7)
        if(delta__y==0):
            return 0
        angle4 = (math.atan(abs(delta__x) / abs(delta__y))) *180 / math.pi
        if(x_robot==0):
            return 0
        if(x_robot>0):
            return angle4
        if(x_robot<0):
            return 360-angle4
        
    def move_ball(self, distance, ball_earth, heading,x_robot,y_robot):
        #distance = 20 - distance
        r = 0.15
        #print("s")
        angle = 0
        #e=(distance *3/0.55)+6
        e=9
        if(distance>r):
            self.set_head_direction_ball(ball_earth, heading, 10,10,10, 10)
        elif(ball_earth < 20 or ball_earth > 320):
            angle5=self.angle_gate(y_robot,x_robot)
            self.set_head_direction_ball(angle5, heading, 10,10,10, 10)
        elif(ball_earth >= 40 and ball_earth < 180):
            # if(distance >= r):
            #     angle = ((math.atan(r / distance)) * 180) / math.pi
            #     angle = ball_earth + angle
            #     self.set_head_direction_ball(angle, heading, 10,10,e,10)
            # elif(ball_earth < 150):
            #     print("x")
            #     self.set_head_direction_ball(ball_earth + 30, heading, 10,10,e, 10)
            # else:
            #     print("x")
            #     #self.set_head_direction_ball(270, heading, 10,e,10, e)
            #     self.set_head_direction_ball(ball_earth, heading, e,10,e, 10)
            angle = ((math.atan(r / distance)) * 180) / math.pi
            angle = ball_earth + angle
            self.set_head_direction_ball(angle, heading, 10,10,10,10)
        elif(ball_earth >= 180 and ball_earth < 320):
            # if(distance >= r):
            #     angle = ((math.atan(r / distance)) * 180) / math.pi
            #     angle = ball_earth - angle
            #     self.set_head_direction_ball(angle, heading, 10,10,10, e)
            # elif(ball_earth > 210):
            #     self.set_head_direction_ball(ball_earth - 30, heading, 10,10,10, e)
            # elif(distance < r):
            #     print("y")
            #     self.set_head_direction_ball(90, heading, 10,10,10, e)
            angle = ((math.atan(r / distance)) * 180) / math.pi
            angle = ball_earth - angle
            self.set_head_direction_ball(angle, heading, 10,10,10,10)
        # else:
        #     self.set_head_direction_ball(angle, heading, 10,10,10, 10)

    def move_ball2(self, distance, ball_earth, heading,x_robot,y_robot):
            #distance = 20 - distance
        if(ball_earth>=80 and ball_earth<=280 ):
            r = 0.1
        else:
            r=0.1
        angle = 0
        #e=(distance *3/0.55)+6
        e=9
        if(ball_earth < 30 or ball_earth > 330):
            angle5=self.angle_gate(y_robot,x_robot)
            self.set_head_direction_ball(angle5, heading, 10,10,10, 10)
        # elif(distance>r+0.03):
        #     if(ball_earth >= 15 and ball_earth < 180):
        #         self.set_head_direction_ball(ball_earth+10, heading, 10,10,10,8)
        #     elif(ball_earth >= 180 and ball_earth < 345):
        #         self.set_head_direction_ball(ball_earth-10, heading, 10,10,8,10)
        if(distance<=r):
            if(ball_earth >= 35 and ball_earth < 180):
                self.set_head_direction_ball(ball_earth+90, heading, 10,10,10,5)
            elif(ball_earth >= 180 and ball_earth < 325):
                self.set_head_direction_ball(ball_earth-90, heading, 10,10,5,10)
        else:
            self.set_head_direction_ball(ball_earth, heading, 10,10,10, 10)
            
    def move_ball3(self,ball_earth,heading,distance,x_robot,y_robot):
        r = 0.15
        if(ball_earth<40 or ball_earth>320):
            if(distance<r):
                angle5=self.angle_gate(y_robot,x_robot)
                self.set_head_direction_ball(angle5, heading, 10,10,10, 10)
            else:
                self.set_head_direction_ball(ball_earth, heading, 10,10,10, 10)
        elif(ball_earth>140 and ball_earth<220):
            self.move_ball2(distance, ball_earth, heading,x_robot,y_robot)
        else:
            self.set_head_direction_ball(ball_earth, heading, 10,10,10, 10)
            
    def goto_coordinates(self, x1, y1, x_robot, y_robot, heading):
        task = self.receipt_task()
        if(task == 1):  # blue_team
            if(abs(y_robot - y1) <= 0.03 and abs(x_robot - x1) <= 0.03):
                self.set_head_direction_ball(0, heading, 10,10,0,0)
            elif(y_robot == y1):
                if(x_robot > x1):
                    self.set_head_direction_ball(90, heading, 10,10,10, 10)
                else:
                    self.set_head_direction_ball(270, heading, 10,10,10, 10)
            elif(x_robot == x1):
                if(y_robot > y1):
                    self.set_head_direction_ball(0, heading, 10,10,10, 10)
                else:
                    self.set_head_direction_ball(180, heading, 10,10,10, 10)
            else:
                
                delta_x = x1 - x_robot
                delta_y = y1 - y_robot
                angle3 = (math.atan(abs(delta_y) / abs(delta_x))) * 180 / math.pi
                if(delta_x > 0 and delta_y > 0):
                    self.set_head_direction_ball(270 - angle3, heading, 10,10,10, 10)
                elif(delta_x > 0 and delta_y < 0):
                    self.set_head_direction_ball(270 + angle3, heading, 10,10,10, 10)
                elif(delta_x < 0 and delta_y > 0):
                    self.set_head_direction_ball(90 + angle3, heading, 10,10,10, 10)
                elif(delta_x < 0 and delta_y < 0):
                    self.set_head_direction_ball(90 - angle3, heading, 10,10,10, 10)
        else:
            if(abs(y_robot - y1) <= 0.03 and abs(x_robot - x1) <= 0.03):
                self.set_head_direction_ball(0, heading, 0,0,0,0)
            elif(y_robot == y1):
                if(x_robot > x1):
                    self.set_head_direction_ball(270, heading, 10,10,10, 10)
                else:
                    self.set_head_direction_ball(90, heading, 10,10,10, 10)
            elif(x_robot == x1):
                if(y_robot > y1):
                    self.set_head_direction_ball(180, heading, 10,10,10, 10)
                else:
                    self.set_head_direction_ball(0, heading, 10,10,10, 10)
            else:
                delta_x = x1 - x_robot
                delta_y = y1 - y_robot
                angle3 = (math.atan(abs(delta_y) / abs(delta_x))) *180 / math.pi
                if(delta_x > 0 and delta_y > 0):
                    self.set_head_direction_ball(90 - angle3, heading, 10,10,10, 10)
                elif(delta_x > 0 and delta_y < 0):
                    self.set_head_direction_ball(90 + angle3, heading, 10,10,10, 10)
                elif(delta_x < 0 and delta_y > 0):
                    self.set_head_direction_ball(270 + angle3, heading, 10,10,10, 10)
                elif(delta_x < 0 and delta_y < 0):
                    self.set_head_direction_ball(270 - angle3, heading, 10,10,10, 10)

    # def set_robot_kepper(self, distance, ball_earth, x_robot, y_robot, heading):
    #     if(y_robot < 0.39):

    #         self.set_head_direction_ball(180, heading, 10, 10)
    #     elif(y_robot > 0.41):

    #         self.set_head_direction_ball(0, heading, 10, 10)
    #     else:

    #         if(ball_earth < 5 or ball_earth > 355):
    #             self.set_head_direction_ball(90, heading, 10, 0)
    #         if(ball_earth > 5 and ball_earth < 90):
    #             self.set_head_direction_ball(90, heading, 10, 10)
    #         if(ball_earth > 270 and ball_earth < 355):
    #             self.set_head_direction_ball(90, heading, 10, -10)

    def get_ball_coordinates(self, ball_earth, x_robot, y_robot, distance):
        task = self.receipt_task()
        if(task == 1):  # blue_team
            if(ball_earth == 0):
                return {x_robot, y_robot - distance}
            elif(ball_earth == 90):
                return {x_robot - distance, y_robot}
            elif(ball_earth == 180):
                return {x_robot, y_robot + distance}
            elif(ball_earth == 270):
                return {x_robot + distance, y_robot}
            elif(ball_earth > 0 and ball_earth < 90):
                
                ball_earth = 90 - ball_earth
                
                return {x_robot - ((math.cos(ball_earth * math.pi / 180)) * distance), y_robot - ((math.sin(ball_earth * math.pi / 180)) * distance)}
            elif(ball_earth > 90 and ball_earth < 180):
                
                ball_earth = ball_earth - 90
                return {x_robot - ((math.cos(ball_earth * math.pi / 180)) * distance), y_robot + ((math.sin(ball_earth * math.pi / 180)) * distance)}
            elif(ball_earth > 180 and ball_earth < 270):
                
                ball_earth = 270 - ball_earth
                return {x_robot + ((math.cos(ball_earth * math.pi / 180)) * distance), y_robot + ((math.sin(ball_earth * math.pi / 180)) * distance)}
            elif(ball_earth > 270 and ball_earth < 360):
                
                ball_earth = ball_earth - 270
                return {x_robot + ((math.cos(ball_earth * math.pi / 180)) * distance), y_robot - ((math.sin(ball_earth * math.pi / 180)) * distance)}
        else:
            if(ball_earth == 0):
                return {x_robot, y_robot + distance}
            elif(ball_earth == 90):
                return {x_robot + distance, y_robot}
            elif(ball_earth == 180):
                return {x_robot, y_robot - distance}
            elif(ball_earth == 270):
                return {x_robot - distance, y_robot}
            elif(ball_earth > 0 and ball_earth < 90):
                
                ball_earth = 90 - ball_earth
                
                return {x_robot + ((math.cos(ball_earth * math.pi / 180)) * distance), y_robot + ((math.sin(ball_earth * math.pi / 180)) * distance)}
            elif(ball_earth > 90 and ball_earth < 180):
                
                ball_earth = ball_earth - 90
                return {x_robot + ((math.cos(ball_earth * math.pi / 180)) * distance), y_robot - ((math.sin(ball_earth * math.pi / 180)) * distance)}
            elif(ball_earth > 180 and ball_earth < 270):
               
                ball_earth = 270 - ball_earth
                return {x_robot - ((math.cos(ball_earth * math.pi / 180)) * distance), y_robot - ((math.sin(ball_earth * math.pi / 180)) * distance)}
            elif(ball_earth > 270 and ball_earth < 360):
               
                ball_earth = ball_earth - 270
                return {x_robot - ((math.cos(ball_earth * math.pi / 180)) * distance), y_robot + ((math.sin(ball_earth * math.pi / 180)) * distance)}

    def goal_state(self,x_robot,y_robot,heading):
        if self.receipt_task() == 1:
            self.goto_coordinates(0, 0.5, x_robot, y_robot, heading)
        else:
            self.goto_coordinates(0, -0.5, x_robot, y_robot, heading)

    def goal_keeper(self, x_ball, y_ball, x_robot, y_robot, heading,ball_earth):
        if self.receipt_task() == 1:
            if y_ball < 0.5:
                if(y_robot<0.47 ):
                    self.set_head_direction_ball(0, heading, 10, 10,-10,-10)
                elif(y_robot>0.53):
                    self.set_head_direction_ball(0, heading, 10,10,10, 10)
                elif(y_robot>0.51):
                    if(ball_earth<3 or ball_earth>357):
                        self.set_head_direction_ball(90, heading, 10,10,0, 0)
                    elif (ball_earth>3 and ball_earth<=90):
                        self.set_head_direction_ball(90, heading, 10,10,9, 10)
                    elif(ball_earth>=270 and ball_earth< 357):
                        self.set_head_direction_ball(90, heading, 10,10,-9, -10)
                elif(y_robot<0.49):
                    if(ball_earth<3 or ball_earth>357):
                        self.set_head_direction_ball(90, heading, 10,10,0, 0)
                    elif (ball_earth>3 and ball_earth<=90):
                        self.set_head_direction_ball(90, heading, 10,10,10, 9)
                    elif(ball_earth>=270 and ball_earth< 357):
                        self.set_head_direction_ball(90, heading, 10,10,-10, -9)
                else:
                    if(ball_earth<3 or ball_earth>357):
                        self.set_head_direction_ball(90, heading, 10,10,0, 0)
                    elif (ball_earth>3 and ball_earth<=90):
                        self.set_head_direction_ball(90, heading, 10,10,10, 10)
                    elif(ball_earth>=270 and ball_earth< 357):
                        self.set_head_direction_ball(90, heading, 10,10,-10, -10)
            else:
                if(x_ball>0.36):
                    self.goto_coordinates(0.24, 0.77, x_robot, y_robot, heading)
                if(x_ball<-0.36):
                    self.goto_coordinates(-0.24, 0.77, x_robot, y_robot, heading)
        else:
            if y_ball > -0.5:
                if(y_robot> -0.47 ):
                    self.set_head_direction_ball(0, heading, 10,10,-10, -10)
                elif(y_robot< -0.53):
                    self.set_head_direction_ball(0, heading, 10,10,10, 10)
                elif(y_robot< -0.51):
                    if(ball_earth<3 or ball_earth>357):
                        self.set_head_direction_ball(90, heading, 10,10,0, 0)
                    elif (ball_earth>3 and ball_earth<=90):
                        self.set_head_direction_ball(90, heading, 10,10,9, 10)
                    elif(ball_earth>=270 and ball_earth< 357):
                        self.set_head_direction_ball(90, heading, 10,10,-9, -10)
                elif(y_robot> -0.49):
                    if(ball_earth<3 or ball_earth>357):
                        self.set_head_direction_ball(90, heading, 10,10,0, 0)
                    elif (ball_earth>3 and ball_earth<=90):
                        self.set_head_direction_ball(90, heading, 10,10,10, 9)
                    elif(ball_earth>=270 and ball_earth< 357):
                        self.set_head_direction_ball(90, heading, 10,10,-10, -9)
                else:
                    if(ball_earth<3 or ball_earth>357):
                        self.set_head_direction_ball(90, heading, 10,10,0, 0)
                    elif (ball_earth>3 and ball_earth<=90):
                        self.set_head_direction_ball(90, heading, 10,10,10, 10)
                    elif(ball_earth>=270 and ball_earth< 357):
                        self.set_head_direction_ball(90, heading, 10,10,-10, -10)
            else:
                if(x_ball>0.36):
                    self.goto_coordinates(0.24, -0.77, x_robot, y_robot, heading)
                if(x_ball<-0.36):
                    self.goto_coordinates(-0.24, -0.77, x_robot, y_robot, heading)
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.flg=0
                data = self.get_new_data()
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()
                robot_pos = self.get_gps_coordinates()
                
                x_robot = robot_pos[0]
                y_robot = robot_pos[1]
                heading = self.get_compass_heading()
                #print(heading*180/math.pi)
                heading = self.convert_head(heading)
                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                    self.flg=1
                else:
                    self.flg=0
                    
                
                #print(heading)
                if(self.flg==1):
                    sonar_values = self.get_sonar_values()
                    direction = utils.get_direction(ball_data["direction"])
                    distance = ball_data['strength']
                    distance = self.convert_distance_by_coordinate(distance)
                    if(distance < 0):
                        distance = 0
                    if(distance > 0.55):
                        distance = 0.55
                    ball_angle = self.convert_ball_angle(ball_data['direction'][0], ball_data['direction'][1], direction)
                    ball_earth = self.get_angle_ball_earth(heading, ball_angle)
                    if(ball_earth == 360):
                        ball_earth = 0
                    speed = 0
                    try:
                        cor = list(self.get_ball_coordinates(
                            ball_earth, x_robot, y_robot, distance))
                    except:
                        continue
                    if(self.player_id==1 ):
                        self.goal_keeper(cor[0],cor[1],x_robot,y_robot,heading,ball_earth)
                    elif(self.player_id==2 or self.player_id==3):
                        self.move_ball3(ball_earth,heading,distance,x_robot,y_robot)
                if(self.flg==0):
                    print("y")
                    if(self.player_id==1):
                        print("b")
                        self.goal_state(x_robot,y_robot,heading)
                    else:
                        self.left_motor.setVelocity(0)
                        self.right_motor.setVelocity(0)
                        continue
                
                self.send_data_to_team(self.player_id)

