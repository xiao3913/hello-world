#!/usr/bin/env python
import sys
import csv
import copy
import os
import shutil
from datetime import datetime, timedelta
import xml.etree.ElementTree as ET

"""
Creates the file SML.zip from path_src=.xml-file with shape and stops, and path_out=folder with gtfs-files.
SML.zip created in the current folder.
"""

class Point(object): #2D-point
    def __init__(self,lat,lon):
        self.lat = lat
        self.lon = lon

class Shape_node(object): #Node on trip/shape.
    def __init__(self,ref,point):
        self.ref = ref #Reference number from file
        self.point = point
        self.name = "" #Named if a stop
        self.seq = "" #Sequence on trip

class Gtfs_shape(object): #For each shape
    def __init__(self,shape_id,shape_nodes,stop_nodes,shape_type,shape_nr,stop_seconds, nr_of_trips, start_minutes):
        self.shape_id = shape_id #Id number
        self.shape_nodes = shape_nodes #List of nodes
        self.stop_nodes = stop_nodes #List of stops
        self.shape_type = shape_type #Shape type name
        self.shape_nr = shape_nr #Number of the type
        self.stop_seconds = stop_seconds #How long between stops
        self.nr_of_trips = nr_of_trips #How many trips
        self.start_minutes = start_minutes #First trip start delay from nominal
        #1 min between all trips

#path_src = source .xml, path_out = output folder,
#number_of_trips = list of trips per shape, desired_times = list of added minutes per shape
def make_gtfszip_from_shapexml(source_file): #Main function
    base_location = os.path.dirname(__file__)

    path_src = base_location + source_file
    newpath = base_location+'/new_SML' #Make subfolder
    if not os.path.exists(newpath):
        os.makedirs(newpath)

    tree = ET.parse(path_src)
    root = tree.getroot() #Root is the top layer in the xml-file

    shape_nodes = [] #Store all nodes
    stops = [] #Store node that are stops
    for node in root.iter('node'): #Layer below root
        point = Point(node.attrib['lat'],node.attrib['lon'])
        shape = Shape_node(node.attrib['id'],point) #Make the node
        shape_nodes.append(shape)
        for tag in node.iter('tag'): #Next layer if stop
            shape.name = tag.attrib['v']
            stops.append(shape)

    gtfs_shapes = [] #Store for each shape/trip
    for way in root.iter('way'): #Layer below root
        shape_id = way.attrib['id']
        trip_nodes = [] #Store all nodes in the shape/trip
        stop_nodes = []
        seq = 1 #Assign sequence in the shape
        for nd in way.iter('nd'): #Layer below way, nodes
            ref = nd.attrib['ref']
            for node in shape_nodes: #Check every node and see if its in the shape/trip
                if node.ref == ref:
                    if node.seq == "": #If we havent used it before, add it
                        node.seq = seq
                        trip_nodes.append(node)
                    else: #Otherwise make a copy first
                        new_node = copy.copy(node)
                        new_node.seq = seq
                        trip_nodes.append(new_node)
            for stop in stops: #Check which stops are in the shape
                if stop.ref == ref:
                    if stop.seq == "": #If we havent used it before, add it
                        stop.seq = seq
                        stop_nodes.append(stop)
                    else: #Otherwise make a copy first
                        new_node = copy.copy(stop)
                        new_node.seq = seq
                        stop_nodes.append(new_node)
            seq = seq+1

        #Get values for all
        for tag in way.iter('tag'): 
            if tag.attrib['k'] == "Type":
                shape_type = tag.attrib['v']
            if tag.attrib['k'] == "Stop seconds":
                stop_seconds = int(tag.attrib['v'])
            if tag.attrib['k'] == "Nr":
                shape_nr = int(tag.attrib['v'])
            if tag.attrib['k'] == "Nr of trips":
                nr_of_trips = int(tag.attrib['v'])
            if tag.attrib['k'] == "Start minutes":
                start_minutes = int(tag.attrib['v'])

        trip_nodes.sort(key=lambda x: x.seq) #Sort based on sequence
        stop_nodes.sort(key=lambda x: x.seq) #Sort based on sequence

        #Make shape object
        gtfs_shape = Gtfs_shape(shape_id,trip_nodes,stop_nodes,shape_type,shape_nr,stop_seconds, nr_of_trips, start_minutes)
        gtfs_shapes.append(gtfs_shape) #Add to list
        gtfs_shapes.sort(key=lambda x: (x.shape_type, -x.shape_nr), reverse=True) #Sort so Trip before Return

    #Make shapes
    with open(newpath+'/shapes.txt', 'wb') as csv: 
        columnTitleRow = "shape_id,shape_pt_lat,shape_pt_lon,shape_pt_sequence,shape_dist_traveled\n"
        csv.write(columnTitleRow)
        for shape in gtfs_shapes:
            shape_id = shape.shape_id[1:]
            for node in shape.shape_nodes: #Extract the values for each row
                lat = node.point.lat
                lon = node.point.lon
                seq = str(node.seq)
                row = shape_id+","+lat+","+lon+","+seq+","+"\n"
                csv.write(row)

    #Make stops
    with open(newpath+'/stops.txt', 'wb') as csv:
        columnTitleRow = "stop_id,stop_code,stop_name,stop_desc,stop_lat,stop_lon,zone_id,stop_url\n"
        csv.write(columnTitleRow)
        for node in stops: #Extract the values for each row
            stop_id = "S_" + node.name[0:2]
            stop_code = node.ref[1:]
            name = node.name
            lat = node.point.lat
            lon = node.point.lon
            row = stop_id+","+stop_code+","+name+","+","+lat+","+lon+","+","+"\n"
            csv.write(row)

    #Make trips and stop_times
    arrival_base = "00:00:00"
    arrival_base = datetime.strptime(arrival_base,'%H:%M:%S') #Sets first trip in 1 min
    arrival_base = arrival_base + timedelta(hours=datetime.now().hour,minutes=datetime.now().minute+1,seconds=datetime.now().second)
    departure_base = arrival_base + timedelta(seconds=30)
    with open(newpath+'/trips.txt','wb') as csv_trip: #Make trips
        columnTitleRow = "route_id,service_id,trip_id,trip_headsign,direction_id,block_id,shape_id\n"
        csv_trip.write(columnTitleRow)
        with open(newpath+'/stop_times.txt','wb') as csv_times: #Make stoptimes
            columnTitleRow = "trip_id,arrival_time,departure_time,stop_id,stop_sequence,stop_headsign,pickup_type,drop_off_type,shape_dist_traveled\n"
            csv_times.write(columnTitleRow)
            for m, shape in enumerate(gtfs_shapes): #Go through all shapes
                if shape.shape_type == "Trip":
                    trip_addedtime = timedelta(minutes=1) #1 min between trips
                    stop_addedtime = timedelta(seconds=shape.stop_seconds)+timedelta(seconds=30) #Seconds between stops
                elif shape.shape_type == "Return":
                    trip_addedtime = timedelta(minutes=1)
                    stop_addedtime = timedelta(seconds=shape.stop_seconds)+timedelta(seconds=30)
                extra_starttime = timedelta(minutes=shape.start_minutes) #Start of the first trip
                for n in range(shape.nr_of_trips): #Do for each given number of trips
                    route_id = "Route1"
                    service_id = "KH"
                    trip_headsign = "Shuttlebus"
                    direction_id = "1"
                    block_id = str(m)+str(n+1) #Makes unique id
                    shape_id = shape.shape_id
                    trip_id = shape_id[1:]+"_"+str(n) #666_3 for shape_id = 666, number of trips = 3
                    row = route_id+","+service_id+","+trip_id+","+trip_headsign+","+direction_id+","+block_id+","+shape_id[1:]+"\n"
                    csv_trip.write(row)
                    for k, stop in enumerate(shape.stop_nodes): #Make stop_times for each trip
                        stop_headsign = ""
                        pickup_type = ""
                        drop_off_type = ""
                        shape_dist_traveled = ""
                        stop_id = "S_" + stop.name[0:2] #S_Ce for Stop name = Centralen
                        stop_sequence = str(k+1) #Increment for each stop
                        if k == 0 and m == 0:
                            arrival_time = arrival_base
                            arrival_time = datetime.strftime(arrival_time,'%H:%M:%S') #Add all times
                        else:
                            arrival_time = arrival_base+k*stop_addedtime+n*trip_addedtime+extra_starttime
                            arrival_time = datetime.strftime(arrival_time,'%H:%M:%S') #Add all times

                        departure_time = departure_base+k*stop_addedtime+n*trip_addedtime+extra_starttime
                        departure_time = datetime.strftime(departure_time,'%H:%M:%S') #Add all times
                        row = trip_id+","+arrival_time+","+departure_time+","+stop_id+","+stop_sequence+","+stop_headsign+","+pickup_type+","+drop_off_type+","+shape_dist_traveled+"\n"
                        csv_times.write(row)


    #Copy all remaining to folder and make zip
    shutil.make_archive(base_location+'/SML', 'zip', newpath)

if __name__ == "__main__":
    source_file = '/josm_data.xml'
    make_gtfszip_from_shapexml(source_file)