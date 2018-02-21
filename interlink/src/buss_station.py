#!/usr/bin/env python
import rospy
import curses
import os
import transitfeed

from interlink.msg import TimeTopic, PassengerType
from datetime import datetime

class BusStation():
    def __init__(self, name):
        self.name = name
        # passenger container that holds all passenger at the station
        self.passengers = []
    def make_passenger(self, destination, time):
        passenger = PassengerType()
        passenger.destination = destination
        passenger.location = self.name
        passenger.start_time = time
        passenger.boarding_time = ""
        passenger.arrival_time = ""

        self.passengers.append(passenger)

class BussStationControl():
    def __init__(self, gtfs_file_name = 'SML.zip'):
        self.base_location = os.path.dirname(__file__)
        self.gtfs_file_name = gtfs_file_name

        self.station_dict = {}


        rospy.init_node('BusStationControl')
        rospy.Subscriber('Clock', TimeTopic, self.update_current_time)

        rospy.Subscriber('HandlePassenger', HandlePassenger, self.send_passenger_to_vehicle)

        self.passenger_pub = rospy.Publisher('PassengerTopic', Passengers, queue_size = 1)

        self.load_stations()
        self.key_board_input()
        rospy.spin()


    def senf_passenger_to_vehicle(self, data):
        current_stop_name = data.stop_id
        
        departure_time_datetime = datetime.strptime(data.departure_time,'%H:%M:%S')
        passengers_to_send = []
        while len(passengers_to_send) < data.seats_available and self.current_time_datetime <= departure_time_datetime - timedelta(seconds=5):
            for passenger in self.station_dict[current_stop_name].passengers:
                for stop in data.remaining_stops:
                    if passenger.destination == stop.stop_id:
                        #passenger.boarding_time = self.current_time
                        passengers_to_send.append(passenger)
                        self.station_dict[current_stop_name].passengers.remove(passenger)

        for passenger in passengers_to_send:
            passenger.boarding_time = self.current_time
        msg = Passengers()
        msg.passengers = passengers_to_send
        print('passenger sending to the bus', msg)
        self.passenger_pub.publish(msg)

    def update_current_time(self, data):
        self.current_time = data.time_stamp
        self.current_time_datetime = datetime.strptime(self.current_time,'%H:%M:%S')

    def load_stations(self):
        gtfs_file_location = self.base_location + '/' + self.gtfs_file_name
        
        # load the GTFS time Schedule
        schedule = transitfeed.Schedule() #Create schedule object
        loader = transitfeed.Loader(gtfs_file_location, schedule)
        loader.Load() #Load .zip into schedule

        self.station_names = []

        #Creates one msg for every trip
        trip_list = schedule.GetTripList() 
        for n in range(len(trip_list)):
            trip_id = trip_list[n].trip_id
            for i in range(len(schedule.GetTrip(trip_id).GetTimeStops())):
                stop_id = schedule.GetTrip(trip_id).GetTimeStops()[i][2].stop_id

                if not stop_id in self.station_names:
                    self.station_names.append(stop_id)
                    self.station_dict[stop_id] = BusStation(stop_id)

        print self.station_names

    def key_board_input(self):
        stdscr = curses.initscr()
        curses.cbreak()
        stdscr.keypad(1)
        stdscr.refresh()

        key = ''
        while key != ord('q'):
            key = stdscr.getch()
            stdscr.refresh()

            if key == ord('a'):
                station_name = 'S_Gr'
                self.station_dict[station_name].make_passenger('S_He',self.current_time)

            elif key == ord('d'):
                station_name = 'S_Ki'
                self.station_dict[station_name].make_passenger('S_He',self.current_time)

            elif key == ord('g'):
                station_name = 'S_He'
                self.station_dict[station_name].make_passenger('S_Ki',self.current_time)

            num = 20
            for station_name in self.station_dict:
                stdscr.addstr(2, num, "Station: " + station_name)
                stdscr.addstr(3, num, "Number of passengers: " + str(len(self.station_dict[station_name].passengers)))
                for i in range(0, len(self.station_dict[station_name].passengers)):
                    stdscr.addstr(4+i, num, "Passenger " + str(i) + " at " + str(self.station_dict[station_name].passengers[i].start_time)+ " heads to: " + self.station_dict[station_name].passengers[i].destination)
                num += 50 
        cyrses.endwin()

if __name__ == '__main__':
    stationcontrol = BussStationControl()