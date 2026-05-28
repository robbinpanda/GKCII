#! /usr/bin/env python
from Car import Car
import time

if __name__=='__main__':
    path = '/home/computenest/catkin_ws/src/site_model/src/scripts_wx/RMMDet_MAB/data_MAB_car02_04.txt'
    car1 = Car('car2', 2, path)
    car1.run()
