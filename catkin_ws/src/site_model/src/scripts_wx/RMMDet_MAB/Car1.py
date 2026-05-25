#! /usr/bin/env python
from Car import Car
import time

if __name__=='__main__':
    path = '/home/computenest/catkin_ws/src/site_model/src/scripts_wx/RMMDet_MAB/data_raw_OneCar_08.txt'
    car1 = Car('car1', 1, path)
    car1.run()
