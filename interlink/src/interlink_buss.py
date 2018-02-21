#!/usr/bin/env python
import rospy
import numpy
import sys
import json
import os
import utm
import requests

from datetime import datetime, timedelta

from interlink.msg import Gtfs, GpsCoordinat, StopType, TimeTopic, VehicleState, drive_param, VehicleService, OndemandTopic

from geomery_msgs.msg import Twist
#Every passenger is an object. Update location when moving. Travel times printed on deletion. 
class Passenger(object):
    def __init__(self,destination,location,start_time,boarding_time):
        self.destination = destination
        self.location = location
        self.start_location = location
        self.start_time = start_time
        self.boarding_time = boarding_time
        self.arrival_time = ""
        self.FMT = '%H:%M:%S'

    def set_location(self,location):
        self.location = location

    def set_arrival_time(self,time):
        self.arrival_time = time
        print 'Passenger arrived!'

    def get_travel_times(self):
        if self.arrival_time:
            #Calculate the travel times. 
            self.waiting_time = datetime.strptime(self.boarding_time, self.FMT)-datetime.strptime(self.start_time, self.FMT)
            self.travel_time = datetime.strptime(self.arrival_time, self.FMT)-datetime.strptime(self.boarding_time, self.FMT)
            print 'Passenger waited at', self.start_location, 'for', self.waiting_time.seconds, 'seconds,'
            print 'and traveled to', self.destination, 'over', self.travel_time.seconds, 'seconds.'
        else:
            print 'Passenger has not arrived yet!'

    def __del__(self):
        self.get_travel_times()


class MiniTruck():
    def __init__(self,vehicle_id = 1, vehicle_type = 'sim', x = -2.04886153309, y = -1.02537927844, yaw = 0., v = 0., steering = 0., axel_length = 0.11):
        self.new_trip_created = False


        
        # define the vehicle identity
        self.base_location = os.path.dirname(__file__)
        self.vehicle_id = vehicle_id
        self.class_name = self.__class__.__name__
        self.vehicle_name = self.class_name + str(self.vehicle_id)
        self.vehicle_type = vehicle_type

        # vehicle parameter
        self.axel_length = axel_length

        # define the service capacity and status
        self.vehicle_max_capacity = 4
        self.vehicle_onboard_passenger = 0
        self.service_ratio = self.vehicle_onboard_passenger / self.vehicle_max_capacity
        # only activate passenger counting when it's at stop
        self.open_for_passenger = False

        # define vehicle trip status
        self.on_duty = False
        self.trip_status = 'driving'
        self.current_stop = None


        # set vehicle state
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_yaw = self.yaw

        self.v_desired = 0.1
        self.desired_steering = steering
        self.v_max = 0.2

        self.steering_command = 0
        self.velocity_command = 0

        # define frame translate
        self.utm_to_local_x = 1 / 136.3217
        self.utm_to_local_y = 1 / 150.1244
        self.translate_x = 4899.37214474
        self.translate_y = 43890.58463
        self.points_per_meter = 20
        self.x_utm = (self.x + self.translate_x)/self.utm_to_local_x
        self.y_utm = (self.y + self.translate_y)/self.utm_to_local_y


        self.zone_id = 33
        self.delta_t = 0.05
        self.time_total = 0
        self.unload_time = 0
        self.FMT = '%H:%M:%S'

        self.passengers = []
        self.passengers_completed = []

	    self.api_address = 'interlink.livemap24.com/vehicle/update2'
	    try:
	        with open(self.base_station + '/veridict_realtime.json') as json_format:
	            self.realtime_info = json.load(json_format)
	    except:
	        print "can't load the json file correctly"



        # define ROS vehicle Node
        rospy.init_node(self.vehicle_name, log_level = rospy.WARN)
        #get the trip plan
        rospy.Subscriber('GTFS_TOPIC', Gtfs, self.register_duty)
        #get the passenger flow
        rospy.Subscriber(Passenger_Topic, Passengers, self.self.passenger_count_sim)

        


        #if using real vehicle
        if self.vehicle_type == 'real':
        	#update vehicle state from mocap
        	self.mocap_name = 'Mocapstate'+str(self.vehicle_id)
        	rospy.Subscriber(self.mocap_name, VehicleState, self.update_vehicle_state_from_qualisys)

        	# activate real vehicle control step
        	rospy.Subscriber('Clock', TimeTopic, self.control_step_real)

        	# publisher for pwm signal
        	self.pwm_pub = rospy.Publisher(self.vehicle_name+'/cmd_vel', Twist, queue_size = 1)
        else:
        	# activate simulation vehicle control step
        	rospy.Subscriber('Clock', TimeTopic, self.control_step_sim)


        self.vehicle_service_publisher = rospy.Publisher('/VehicleService', VehicleService, queue_size = 1)
        self.ondemand_publisher = rospy.Publisher('OndemandUpdate', OndemandTopic, queue_size = 1)
        self.handle_passenger = rospy.Publisher('HandlePassenger', HandlePassenger, queue_size = 10)



        rospy.sleep(1)

        srv_msg = VehicleService()
        srv_msg.on_duty = self.on_duty
        srv_msg.vehicle_id = self.vehicle_id
        srv_msg.service_station_id = self.service_station_id
        srv_msg.x = self.x
        srv_msg.y = self.y
        self.vehicle_service_publisher.publish(srv_msg)


        rospy.spin()


    def control_step_sim(self, data):
        self.current_time = data.time_stamp

        #Send position every second
        self.time_total += self.delta_t
        if self.time_total >= 1:
            try:
                self.upload_realtime_vehicle_info()
            except:
                print "Error sending data!"
                pass
            self.time_total = 0

        if self.on_duty == True:
            if self.trip_status == 'driving' and self.check_at_stop():
                # check if the vehicle has arrived at its current station
                self.open_for_passenger = True
                self.trip_status = 'waiting'
                print('Vehicle has arrived at station: ', str(self.current_stop.stop_id))

                #Update the onboard passengers
                passengers_remain = []
                for passenger in self.passengers:
                    passenger.set_location(self.current_stop.stop_id)
                    if passenger.destination == passenger.location:
                        passenger.set_arrival_time(self.current_time)
                        self.passengers_completed.append(passenger)
                    else:
                        passengers_remain.append(passenger)
                self.passengers = passengers_remain
                #Update number of passengers
                self.vehicle_onboard_passenger = len(self.passengers)
                print 'Passengers remaining in the vehicle: ', len(self.passengers)

                #Send available seats
                passenger_msg = HandlePassenger()
                passenger_msg.stop_id = self.current_stop.stop_id
                passenger_msg.departure_time = self.current_stop.departure_time
                passenger_msg.seats_available = self.vehicle_max_capacity - self.vehicle_onboard_passenger
                passenger_msg.remaining_stops = self.stops
                self.handle_passenger.publish(passenger_msg)

            elif self.trip_status == 'driving' and not self.check_at_stop():
                # check if the vehicle is driving towards its current station
                
                time_interval = datetime.strptime(self.current_stop.arrival_time, self.FMT)-datetime.strptime(self.current_time, self.FMT)
                time_interval_sec = time_interval.seconds

                self.total_distance = self.route_dist[-1]

                self.distance_to_stop = self.total_distance - self.total_distance_covered

                try:
                    self.velocity_command = float(self.distance_to_stop)/time_interval_sec
                except ZeroDivisionError:
                    self.velocity_command = 0

                distance_covered = self.velocity_command * self.delta_t

                print('total distance:', self.total_distance)
                print('distance_covered:', distance_covered)
                print('distance to stop:', self.distance_to_stop)
                print('time_interval', time_interval_sec)
                print('velocity_command:', self.velocity_command)
                print('current_stop:', self.current_stop)
                print('vehicle_x:', self.x)
                print('vehicle_y:', self.y)
                #print('route:', self.route)


                self.total_distance_covered += distance_covered

                print('total_distanc_covered:', self.total_distance_covered)
                if self.total_distance_covered >= self.total_distance:
                    self.total_distance_covered = self.total_distance
                    self.x = self.route[0][-1]
                    self.y = self.route[1][-1]
                    self.yaw = self.route[2][-1]

                else:
                    index = 0
                    while self.total_distance_covered >= self.route_dist[index]:
                        index += 1

                    distance_left = self.total_distance_covered - self.route_dist[index]

                    state_x = self.route[0][index]
                    state_y = self.route[1][index]
                    state_yaw = self.route[2][index]

                    self.x = state_x + distance_left * numpy.cos(state_yaw)
                    self.y = state_y + distance_left * numpy.sin(state_yaw)
                    self.yaw = state_yaw

                print('new_x', self.x)
                print('new_y', self.y)

                self.x_utm = (self.x + self.translate_x)/self.utm_to_local_x
                self.y_utm = (self.y + self.translate_y)/self.utm_to_local_y


            if self.trip_status == 'waiting' and self.check_departure():
                self.open_for_passenger = False
                self.unload_time = 0

                # check if it is time for the vehicle to departure for the next station
                if len(self.stops) == 0:
                    self.on_duty = False
                    srv_msg = VehicleService()
                    srv_msg.on_duty = self.on_duty
                    srv_msg.vehicle_id = self.vehicle_id
                    srv_msg.service_station_id = self.service_station_id
                    srv_msg.x = self.x
                    srv_msg.y = self.y
                    self.vehicle_service_publisher.publish(srv_msg)

                    # Empty the vehicle when at the final station
                    for passenger in self.passengers:
                        passenger.set_location(self.current_stop.stop_id)
                        passenger.set_arrival_time(self.current_time)
                        self.passengers_completed.append(passenger)
                    
                    self.passengers = []
                    passengers_remain = []
                    self.passengers_completed = []
                    self.vehicle_onboard_passenger = 0

                    print('trip duty completed')
                    
                else:
                    if self.new_trip_created == False:
                        if self.current_stop.stop_id == self.start_station_id:
                            ondemand_msg = OndemandTopic()
                            ondemand_msg.new_trip = False
                            ondemand_msg.service_station_id = self.service_station_id
                            self.ondemand_publisher.publish(ondemand_msg)

                    self.current_stop = self.stops.pop()
                    self.route = self.sub_duty_dict[self.current_stop.stop_id]

                    self.route_dist = self.sub_duty_distance_dict[self.current_stop.stop_id]
                    self.traj_num_points = len(self.route[0])

                    print('the route length:', len(self.route[0]))
                    self.total_distance_covered = 0
                    self.trip_status = 'driving'

            elif self.trip_status == 'waiting' and not self.check_departure():
                self.open_for_passenger = True

                self.velocity_command = 0
                self.unload_time += self.delta_t

                #Potential to leave early if vehicle full after 10 seconds on arrival.
                if self.unload_time >= 10 and self.check_onboard_passenger():
                    current_time_datetime = datetime.strptime(self.current_time,self.FMT)
                    current_departure_datetime = datetime.strptime(self.current_stop.departure_time,self.FMT)
                    time_difference = current_departure_datetime-current_time_datetime
                    #Done if the time to original time is more than 20 seconds. 
                    if time_difference >= timedelta(seconds=20): #Must be > than dep - arr time
                        delay = -time_difference+timedelta(seconds=10) #Negative delay for leaving early
                        self.update_trip_times(delay) #Not made!
                        print('Leaving in 10 seconds!')
                        #If standing at first stop, the original time is copied for the next vehicle. 
                        if self.current_stop.stop_id == self.start_station_id:
                            self.new_trip_created = True
                            print('Sending new trip')
                            ondemand_msg = OndemandTopic()
                            ondemand_msg.new_trip = True
                            ondemand_msg.service_station_id = self.service_station_id
                            self.ondemand_publisher.publish(ondemand_msg)

        else:
            self.velocity_command = 0


    #Control loop. Callback of time_node. 
    def control_step_real(self, data):
        self.current_time = data.time_stamp

        #Send position every second
        self.time_total += self.delta_t
        if self.time_total >= 1:
            try:
                self.upload_realtime_vehicle_info()
            except:
                print "Error sending data!"
                pass
            self.time_total = 0


        FMT = '%H:%M:%S'
        self.lookahead_dist = 0.3
        min_dist = 10000
        if self.on_duty == True:
            if self.trip_status == 'driving' and self.check_at_stop():
                # check if the vehicle has arrived at its current station
                self.open_for_passenger = True
                self.trip_status = 'waiting'
                print('Vehicle has arrived at station: ', str(self.current_stop.stop_id))

                #Update the onboard passengers
                passengers_remain = []
                for passenger in self.passengers:
                    passenger.set_location(self.current_stop.stop_id)
                    if passenger.destination == passenger.location:
                        passenger.set_arrival_time(self.current_time)
                        self.passengers_completed.append(passenger)
                    else:
                        passengers_remain.append(passenger)
                self.passengers = passengers_remain
                #Update number of passengers
                self.vehicle_onboard_passenger = len(self.passengers)
                print 'Passengers remaining in the vehicle: ', len(self.passengers)

                #Send available seats
                passenger_msg = HandlePassenger()
                passenger_msg.stop_id = self.current_stop.stop_id
                passenger_msg.departure_time = self.current_stop.departure_time
                passenger_msg.seats_available = self.vehicle_max_capacity - self.vehicle_onboard_passenger
                passenger_msg.remaining_stops = self.stops
                self.handle_passenger.publish(passenger_msg)

            elif self.trip_status == 'driving' and not self.check_at_stop():
                # check if the vehicle is driving towards its current station
                time_interval = datetime.strptime(self.current_stop.arrival_time, FMT)-datetime.strptime(self.current_time, FMT)
                time_interval_sec = time_interval.seconds

                self.traj_x = self.route[0]
                self.traj_y = self.route[1]

                #get nearest index
                for i in range(0, self.traj_num_points):
                    temp_dist = numpy.sqrt((self.x-self.traj_x[i])**2 + (self.y-self.traj_y[i])**2)
                    if temp_dist <= min_dist:
                        min_dist = temp_dist
                        min_index = i
                self.nearest_point_index = min_index
                #get reference state
                if min_dist > self.lookahead_dist:
                    ref_state = [self.traj_x[self.nearest_point_index], self.traj_y[self.nearest_point_index]]
                else:
                    ref_index = self.nearest_point_index + int(self.lookahead_dist * self.points_per_meter)
                    ref_index = ref_index % self.traj_num_points
                    ref_state = [self.traj_x[ref_index], self.traj_y[ref_index]]
                #calculate steering command
                dx = ref_state[0] - self.x
                dy = ref_state[1] - self.y
                y_goal = -numpy.sin(self.yaw) * dx + numpy.cos(self.yaw) *dy
                distance_to_goal = numpy.sqrt(dx**2 + dy**2)
                self.steering_command = numpy.arctan(self.axel_length * 2 * y_goal / distance_to_goal ** 2)


                #calculate remain distance to stop
                if self.traj_num_points == 1:
                    distance_to_stop = numpy.sqrt((self.current_stop.position.x - self.x)**2 + (self.current_stop.position.y - self.y)**2)
                else:
                    distance_to_stop = numpy.sqrt((self.current_stop.position.x - self.x)**2 + (self.current_stop.position.y - self.y)**2)
                    if distance_to_stop >= 0.4:
                        distance_to_stop = abs(self.traj_num_points - self.nearest_point_index) * (1 / float(self.points_per_meter))
                #claculate velocity command
                try:
                    self.velocity_command = distance_to_stop/time_interval_sec
                except ZeroDivisionError:
                    self.velocity_command = 0


                print('current schedule arrival time:', self.current_stop.arrival_time)
                print('current time:', self.current_time)
                print('distance_to_stop:', distance_to_stop)
                print('velocity_command:', self.velocity_command)


                #Update times if delayed
                #If vehicle is going above v_max, adds delay for making v = v_desired
                if self.velocity_command >= self.v_max:
                    delay_sec = distance_to_stop/self.v_desired - time_interval_sec
                    delay = timedelta(seconds=delay_sec)
                    self.update_trip_times(delay)
                    print "Delayed! Updating trip"
                    print('distance to stop:', distance_to_stop)
                    print('velocity too high:', self.velocity_command)


            if self.trip_status == 'waiting' and self.check_departure():
                self.open_for_passenger = False
                self.unload_time = 0

                # check if it is time for the vehicle to departure for the next station
                if len(self.stops) == 0:
                    self.on_duty = False
                    srv_msg = VehicleService()
                    srv_msg.on_duty = self.on_duty
                    srv_msg.vehicle_id = self.vehicle_id
                    srv_msg.service_station_id = self.service_station_id
                    srv_msg.x = self.x
                    srv_msg.y = self.y
                    self.vehicle_service_publisher.publish(srv_msg)

                    # Empty the vehicle when at the final station
                    for passenger in self.passengers:
                        passenger.set_location(self.current_stop.stop_id)
                        passenger.set_arrival_time(self.current_time)
                        self.passengers_completed.append(passenger)
                    
                    self.passengers = []
                    passengers_remain = []
                    self.passengers_completed = []
                    self.vehicle_onboard_passenger = 0

                    print('trip duty completed')

                else:
                    if self.new_trip_created == False:
                        if self.current_stop.stop_id == self.start_station_id:
                            ondemand_msg = OndemandTopic()
                            ondemand_msg.new_trip = False
                            ondemand_msg.service_station_id = self.service_station_id
                            self.ondemand_publisher.publish(ondemand_msg)

                    self.current_stop = self.stops.pop()
                    self.route = self.sub_duty_dict[self.current_stop.stop_id]

                    self.traj_num_points = len(self.route[0])
                    #print(self.route)
                    self.trip_status = 'driving'

            elif self.trip_status == 'waiting' and not self.check_departure():
                self.open_for_passenger = True

                self.velocity_command = 0
                self.unload_time += self.delta_t

                #Potential to leave early if vehicle full after 10 seconds on arrival.
                if self.unload_time >= 10 and self.check_onboard_passenger():
                    current_time_datetime = datetime.strptime(self.current_time,self.FMT)
                    current_departure_datetime = datetime.strptime(self.current_stop.departure_time,self.FMT)
                    time_difference = current_departure_datetime-current_time_datetime
                    #Done if the time to original time is more than 20 seconds. 
                    if time_difference >= timedelta(seconds=20): #Must be > than dep - arr time
                        delay = -time_difference+timedelta(seconds=10) #Negative delay for leaving early
                        self.update_trip_times(delay) #Not made!
                        print('Leaving in 10 seconds!')
                        #If standing at first stop, the original time is copied for the next vehicle. 
                        if self.current_stop.stop_id == self.start_station_id:
                            self.new_trip_created = True
                            print('Sending new trip')
                            ondemand_msg = OndemandTopic()
                            ondemand_msg.new_trip = True
                            ondemand_msg.service_station_id = self.service_station_id
                            self.ondemand_publisher.publish(ondemand_msg)
        else:
            self.steering_command = 0
            self.velocity_command = 0

        self.send_control_commands()


    #Updates the current stoptimes in the vehicle
    def update_trip_times(self,delay):
        GTFS_MSG_new = Gtfs() #Make copy of old message
        GTFS_MSG_new.vehicle_id = self.vehicle_id
        GTFS_MSG_new.trip_id = self.trip_id
        GTFS_MSG_new.zone_id = self.zone_id
        GTFS_MSG_new.shapes = self.shapes

        #Add the delay for current times
        departure = datetime.strptime(self.current_stop.departure_time,'%H:%M:%S')+delay
        self.current_stop.departure_time = datetime.strftime(departure,'%H:%M:%S')
        if delay >= timedelta(seconds=0): #For real delays
            arrival = datetime.strptime(self.current_stop.arrival_time,'%H:%M:%S')+delay
            self.current_stop.arrival_time = datetime.strftime(arrival,'%H:%M:%S')

        #Make new stop list
        stop_list_new = []
        stop_list_new.append(self.current_stop)

        #Add the delay for all stops
        for stop in self.stops:
            arrival = datetime.strptime(stop.arrival_time,'%H:%M:%S')+delay
            departure = datetime.strptime(stop.departure_time,'%H:%M:%S')+delay
            stop.arrival_time = datetime.strftime(arrival,'%H:%M:%S')
            stop.departure_time = datetime.strftime(departure,'%H:%M:%S')
            stop_list_new.append(stop)
        print('Updated times')
        GTFS_MSG_new.stops = stop_list_new #Save the stops in the message
        #self.send_updated_times(GTFS_MSG_new,delay) #NOT COMPLETE!

    #update the vehicle state from mocap
    def update_vehicle_state_from_qualisys(self, data):
        self.x = data.x
        self.y = data.y
        self.yaw = data.yaw * numpy.pi / 180   # measured in radian

        dx = self.x - self.prev_x
        dy = self.y - self.prev_y

        self.v = numpy.sqrt(dx*dx + dy*dy) / self.delta_t

        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_yaw = self.yaw

        self.x_utm = (self.x + self.translate_x)/self.utm_to_local_x
        self.y_utm = (self.y + self.translate_y)/self.utm_to_local_y

    #send pwm signal to the vehicle
    def send_control_commands(self):
        if int(self.vehicle_id) == 1:
            #vel = int(120 * self.velocity_command + 110)
            vel = int(150 * self.velocity_command + 115)
            steer = int(100 -160 * self.steering_command)

            msg = drive_param()
            msg.velocity = vel
            msg.angle = steer

            self.vehicle_variable_publisher.publish(msg)
        elif int(self.vehicle_id) == 2:
            #vel = int(120 * self.velocity_command + 110)
            #vel = int(125 * self.velocity_command + 100)
            vel = int(150 * self.velocity_command + 100)
            steer = int(100 -160 * self.steering_command)


            msg = drive_param()
            msg.velocity = vel
            msg.angle = steer

            self.vehicle_variable_publisher.publish(msg)
        elif int(self.vehicle_id) == 3:

            vel = int(120 * self.velocity_command + 110)
            steer = int(100 -160 * self.steering_command)


            msg = drive_param()
            msg.velocity = vel
            msg.angle = steer

            self.vehicle_variable_publisher.publish(msg)



    #Callback to Passenger_Topic. Receives passenger data and creates an object. 
    def passenger_count_sim(self,data):
        if self.open_for_passenger == True:
            for passenger in data.passengers:
                self.passengers.append(Passenger(passenger.destination,passenger.location,passenger.start_time,passenger.boarding_time))
            self.vehicle_onboard_passenger = len(self.passengers)
    
    #Check if vehicle is full
    def check_onboard_passenger(self):
        if self.vehicle_onboard_passenger >= self.vehicle_max_capacity:
            return True
        else:
            return False

    #Check if vehicle has arrived at current stop
    def check_at_stop(self):
        distance_to_stop = numpy.sqrt((self.current_stop.position.x - self.x)**2 + (self.current_stop.position.y - self.y)**2)
        if distance_to_stop <= 0.1:
            return True
        else:
            return False

    #Check if it is time for departure
    def check_departure(self):
        departure_time = self.current_stop.departure_time
        if self.current_time >= departure_time:
            return True
        else:
            return False



    #Registers the incoming GTFS-messages and makes paths between stops
    def register_duty(self, data):
        if data.vehicle_id == self.vehicle_id:
            self.on_duty = True

            self.realtime_info['vehicleUpdates'][0]['gtfsSelector']['feedId'] = 'SML'
            self.realtime_info['vehicleUpdates'][0]['gtfsSelector']['tripId'] = self.trip_id
            self.realtime_info['vehicleUpdates'][0]['gtfsSelector']['scheduledStart'] =            

            self.service_station_id = data.service_station_id
            self.trip_id = data.trip_id
            self.zone_id = data.zone_id

            # translate the utm frame to local frame
            shapes = data.shapes

            # to send to veridict
            self.shapes = data.shapes
            for i in range(len(data.shapes)):
                shapes[i].x = self.utm_to_local_x * data.shapes[i].x - self.translate_x
                shapes[i].y = self.utm_to_local_y * data.shapes[i].y - self.translate_y

            # translate the utm frame to local frame
            self.stops = data.stops
            for i in range(len(self.stops)):
                self.stops[i].position.x = self.utm_to_local_x * self.stops[i].position.x - self.translate_x
                self.stops[i].position.y = self.utm_to_local_y * self.stops[i].position.y - self.translate_y

            self.sub_duty_dict = {}
            self.sub_duty_distance_dict = {}

            index = 0
            for stop in self.stops:
                shape = []
                while not (shapes[index].x == stop.position.x and shapes[index].y == stop.position.y):
                    shape.append(shapes[index])
                    index = index + 1
                shape.append(shapes[index])
                self.sub_duty_dict[stop.stop_id] = shape

                print('shape', shape)

            for stop in self.stops:
                shape = self.sub_duty_dict[stop.stop_id]
                route_x = []
                route_y = []

                self.accumulated_route_distance_list = []
                self.accumulated_route_distance = 0
                for index in range(0, len(shape)):
                    if index == 0:
                        self.accumulated_route_distance_list.append(0)
                    else:
                        temp_dist = numpy.sqrt((shape[index].x-shape[index-1].x)**2 + (shape[index].y-shape[index-1].y)**2)
                        self.accumulated_route_distance += temp_dist
                        self.accumulated_route_distance_list.append(self.accumulated_route_distance)
                    route_x.append(shape[index].x)
                    route_y.append(shape[index].y)


                print('distance',self.accumulated_route_distance_list)
                direction_list = []
                for index in range(1, len(route_x)):
                    x = route_x[index] - route_x[index-1]
                    y = route_y[index] - route_y[index-1]

                    direction = numpy.arctan2(y, x)
                    direction_list.append(direction)
                direction_list.append(0)

                self.sub_duty_dict[stop.stop_id] = [route_x, route_y, direction_list]

                self.sub_duty_distance_dict[stop.stop_id] = self.accumulated_route_distance_list

            self.stops.reverse()
            if self.current_stop == None:
                self.current_stop = self.stops.pop()
                self.route = self.sub_duty_dict[self.current_stop.stop_id]
                self.route_dist = self.sub_duty_distance_dict[self.current_stop.stop_id]

                if self.vehicle_type == 'sim':
	                self.x = self.route[0][0]
	                self.y = self.route[1][0]
	                self.yaw = self.route[2][0]
	                self.total_distance_covered = 0

                self.start_station_id = self.current_stop.stop_id

    def upload_realtime_vehicle_info(self):
        sendTime
        try:
            self.realtime_info['sentTime'] = sendTime

            lat, lon = utm.to_latlon(self.x_utm, self.y_utm, self.zone_id, 'V')
            self.realtime_info['sensory']['latitude'] = lat
            self.realtime_info['sensory']['longitude'] = lon
            self.realtime_info['sensory']['time'] =

            json_msg = json.dumps(self.tripupdate_msg)
            try:
                r = requests.post(self.api_address, data = json_msg)

            except requests.exceptions.RequestException as e:
                print e
                pass
        except Exception:
            print "Unable to send realtime info"
            pass                


if __name__ == '__main__':
    interlink_bus1 = MiniTruck(int(sys.argv[1]))