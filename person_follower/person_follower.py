# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
flag = False
class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, input_msg):
        global flag
        angle_min = input_msg.angle_min
        angle_max = input_msg.angle_max
        angle_increment = input_msg.angle_increment
        ranges = input_msg.ranges
        #
        # your code for computing vx, wz
        #
        deteccionFrontal=min(ranges[170:185])# si detecta nos da la distancia de detecciòn
        deteccionIzq=min(ranges[191:270])
        deteccionDer=min(ranges[90:169])
        deteccionIniStart=min(ranges[170:190])
        distanciaIniStart=1
        distanciaMinimaIzq=0.2#0.1
        distanciaMinimaDer=0.3#0.2
        distanciaMinimaAtr=0.2
        distanciaMaximaAtr=1 #1.2
        distanciaMinimaFrontal=0.4#0.3
        distanciaMaxima=1.2
        distanciaMaximaDer=0.8
        if  distanciaIniStart >= deteccionIniStart:
            flag = True
        else:
            vx = 0.0 #velocidad lineal    
            wz = 0.0

        if flag == True:
           
          if deteccionFrontal > distanciaMinimaFrontal and deteccionFrontal < distanciaMaxima:
            vx = 0.5 #velocidad lineal    
            wz = 0.0 # velocidad angular
            print('frontal',deteccionFrontal)

         
         
          elif deteccionIzq > distanciaMinimaIzq and deteccionIzq < distanciaMaxima:
            vx = 0.5 #velocidad lineal    
            wz = 0.8 # velocidad angular
            print('izquierdo',deteccionIzq)

          elif deteccionDer > distanciaMinimaDer and deteccionDer < distanciaMaximaDer:
            vx = 0.3 #velocidad lineal    
            wz = -0.8 # velocidad angular
            print('derecha',deteccionDer)
           
          else:
            vx = 0.0 #velocidad lineal    
            wz = 0.0 # velocidad angular
           
       
        #
        #
        #
        output_msg = Twist()
        output_msg.linear.x = vx
        output_msg.angular.z = wz
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
