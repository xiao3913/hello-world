#!/usr/bin/env python
import numpy
import rospy
import os
import sys

import transitfeed
import utm

from datetime import datetime
from interlink.msg import Gtfs, GpsCoordinat, StopType, TimeTopic, VehicleService, OndemandTopic, Passengers, PassengerType, HandlePassenger

from make_gtfs_zip import *


class Service_Station():
    def __init__(self, Service_Station_id, x, y):
        self.Service_Station_id = Service_Station_id
        self.Service_Station_pose_x = x
        self.Service_Station_pose_y = y

        self.Service_Station_Vehicles = []

    def check_station_availability(self):
        if self.Service_Station_Vehicles == []:
            return False
        else:
            return True

    def add_vehicle(self, vehicle_id):

        self.Service_Station_Vehicles.append(vehicle_id)

    def remove_vehicle(self, vehicle_id):

        self.Service_Station_Vehicles.remove(vehicle_id)


class Service_Station_Control():
    def __init__(self, gtfs_file_name = 'SML.zip'):
        self.base_location = os.path.dirname(__file__)
        self.gtfs_file_name = gtfs_file_name

        # transform parameters to Lab Arena
        self.utm_to_local_x = 1 / 136.3217
        self.utm_to_local_y = 1 / 150.1244
        self.translate_x = 4899.37214474
        self.translate_y = 43890.58463      

        # load the gtfs file into GTFS_TRIP_DICT
        self.load_gtfs_file()
        # load and create all service station and store into self.Service_Station_Dict
        self.load_service_stations()

        # initialize the trip_plan_queue for sending 
        self.gtfs_trip_plan = {}
        self.gtfs_trip_plan_history = {}
        for station_id in self.Service_Station_Dict:
            self.gtfs_trip_plan[station_id] = []
            self.gtfs_trip_plan[station_id].append(self.GTFS_TRIP_DICT[station_id].pop())
            self.gtfs_trip_plan_history[station_id] = []
            print station_id
        

        rospy.init_node('Service_Center')
        self.gtfs_pub = rospy.Publisher('GTFS_TOPIC', Gtfs, queue_size = 1)

        rospy.Subscriber('Clock', TimeTopic, self.send_gtfs_plan)
        rospy.Subscriber('VehicleService', VehicleService, self.update_service_station_status)

        rospy.Subscriber('OndemandUpdate', OndemandTopic , self.on_demand_update)

        rospy.spin()
    def on_demand_update(self, data):
        if data.new_trip == False:
            if self.GTFS_TRIP_DICT[data.service_station_id] != []:
                self.gtfs_trip_plan[data.service_station_id].append(self.GTFS_TRIP_DICT[data.service_station_id].pop())
                self.gtfs_trip_plan_history[data.service_station_id].pop()
            elif data.new_trip == True:
                print('a replacement trip is been called')
                new_trip = self.gtfs_trip_plan_history[data.service_station_id].pop()
                new_trip.trip_id = new_trip.trip_id + 'new'

                self.gtfs_trip_plan[data.service_station_id].append(new_trip)

    def update_service_station_status(self, data):
        if data.on_duty == False:
            self.Service_Station_Dict[data.service_station_id].add_vehicle(data.vehicle_id)

    def send_gtfs_plan(self, data):
        for station in self.Service_Station_Dict:
            trip = self.gtfs_trip_plan[station]
            if trip != []:
                trip = self.gtfs_trip_plan[station][0]
                if self.Service_Station_Dict[station].check_station_availability():
                    if trip.stops[0].arrival_time < data.time_stamp:
                        self.gtfs_trip_plan[station].pop()
                        if self.GTFS_TRIP_DICT[station] != []:
                            self.gtfs_trip_plan[station].append(self.GTFS_TRIP_DICT[station].pop())
                    else:
                        gtfs_plan = self.gtfs_trip_plan[station].pop()
                        gtfs_plan.vehicle_id = station.Service_Station_Vehicles[0]
                        gtfs_plan.service_station_id = station

                        self.gtfs_pub.publish(gtfs_request)
                        self.gtfs_trip_plan_history[station] = [gtfs_plan]

                        self.Service_Station_Dict[station].remove(gtfs_plan.vehicle_id)
                else:
                    if trip.stops[0].arrival_time < data.time_stamp:
                        self.gtfs_trip_plan[station].pop()
    def load_service_stations(self):
        self.Service_Station_Dict = {}
        for shape_id in self.GTFS_TRIP_DICT:
            station_x = self.GTFS_TRIP_DICT[shape_id][0].shapes[0].x
            station_y = self.GTFS_TRIP_DICT[shape_id][0].shapes[0].y

            #transform to Lab Arena coordinate
            station_x = self.utm_to_local_x * station_x - self.translate_x
            station_y = self.utm_to_local_y * station_y - self.translate_y


            self.Service_Station_Dict[shape_id] = Service_Station(shape_id, station_x, station_y)

    def load_gtfs_file(self):
        gtfs_file_location = self.base_location + '/' + self.gtfs_file_name

        # load the GTFS time Schedule
        schedule = transitfeed.Schedule() #Create schedule object
        loader = transitfeed.Loader(gtfs_file_location, schedule)
        loader.Load() #Load .zip into schedule

        self.shape_id_list = []

        self.GTFS_TRIP_DICT = {}

        trip_list = schedule.GetTripList() 
        trip_list.sort(key=lambda x: x.trip_id,reverse=False) #Sort trips based on name

        #Creates one msg for every trip
        for n in range(len(trip_list)):
            GTFS_MSG = Gtfs() #Msg format from file

            trip_id = trip_list[n].trip_id
            shape_id = schedule.GetTrip(trip_id).shape_id

            GTFS_MSG.trip_id = trip_id #Add the trip name

            #Get the shape points for this trip
            shape_list = []
            for i in range(len(schedule.GetShape(shape_id).points)):
                utm_x, utm_y, utm_zone, utm_letter = utm.from_latlon(schedule.GetShape(shape_id).points[i][0],schedule.GetShape(shape_id).points[i][1])
                msg_pose = GpsCoordinat(utm_x,utm_y)
                shape_list.append(msg_pose)

            GTFS_MSG.shapes = shape_list #Add the shape

            #Gets all the shapes and stores as a stop type. Includes name, position and arr/dep times. 
            stop_list = []
            for i in range(len(schedule.GetTrip(trip_id).GetTimeStops())):
                stop_id = schedule.GetTrip(trip_id).GetTimeStops()[i][2].stop_id
                msg_stop = StopType()
                msg_stop.stop_id = stop_id
                utm_x, utm_y, utm_zone, utm_letter = utm.from_latlon(schedule.GetStop(stop_id).stop_lat,schedule.GetStop(stop_id).stop_lon)
                msg_stop.position = GpsCoordinat(utm_x,utm_y)

                stoptimes = schedule.GetTrip(trip_id).GetStopTimes()

                #Do-while loop. Stops not in order so checks all until found right one. 
                ii = 0
                while True:
                    arrival = schedule.GetTrip(trip_id).GetStopTimes()[ii].arrival_time
                    departure = schedule.GetTrip(trip_id).GetStopTimes()[ii].departure_time
                    if ii >= len(schedule.GetTrip(trip_id).GetStopTimes()) or stoptimes[ii].stop_id == stop_id:
                        break
                    ii = ii + 1

                msg_stop.arrival_time = arrival
                msg_stop.departure_time = departure

                stop_list.append(msg_stop)
                stop_list.sort(key=lambda x: x.arrival_time,reverse=False) #Sort based on the time

            GTFS_MSG.stops = stop_list #Add the stops
            GTFS_MSG.zone_id = utm_zone #Add zone id for conversion from UTM

            #Add unique shapes
            if not shape_id in self.shape_id_list:
                self.shape_id_list.append(shape_id)
                self.GTFS_TRIP_DICT[shape_id] = [GTFS_MSG]
            else:
                self.GTFS_TRIP_DICT[shape_id].append(GTFS_MSG) #Stack the messages

        for s_id in self.shape_id_list:
            self.GTFS_TRIP_DICT[s_id].reverse() #Reverse the order

if __name__ == "__main__":
    source_file = '/josm_data.xml'
    make_gtfszip_from_shapexml(source_file)
    control_center = Service_Station_Control()