#!/usr/bin/env python


# firebase connect
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

# init api_key to connect with firebase 
import os
current_path = os.path.dirname(os.path.realpath(__file__))
cred = credentials.Certificate("{}/firebase_key.json".format(current_path))
app = firebase_admin.initialize_app(cred)
db = firestore.client()
doc_ref = db.collection(u'Ego').document(u'Ego_status')
 
# ROS
import rospy
# message import 
from morai_msgs.msg import EgoVehicleStatus
from morai_msgs.msg import GPSMessage




class get_Ego_Status:
    def __init__(self):
        rospy.init_node("Ego_status_to_firebase", anonymous=True)
        self.Ego_status_callback = rospy.Subscriber("/gps", GPSMessage, self.Ego_callback)
        #self.current_acceleration = 0
        #self.current_brake = 0
        self.current_position_x = 0
        self.current_position_y = 0
        #self.current_velocity_x = 0
        self.is_Ego_data_received = False

        rate = rospy.Rate(1) # 1 times / 1 sec
        while not rospy.is_shutdown():
            if self.is_Ego_data_received == True:
                print("Ego_data was just written to Firebase_storage")
                doc_ref.set({
		    #u'current_acceleration': self.current_acceleration,
   	            #u'current_brake': self.current_brake,
                    u'current_position_x': self.current_position_x,
                    u'current_position_y': self.current_position_y,
                    #u'current_velocity': self.current_velocity_x,
                })
            else:
                print("waiting for Ego data...")
                
            rate.sleep()


    def Ego_callback(self, Ego_data):
        print()
        self.is_Ego_data_received = True
        #self.current_velocity_x = Ego_data.velocity.x
        #self.current_acceleration = Ego_data.acceleration.x
        #self.current_brake = Ego_data.brake
        self.current_position_x = Ego_data.latitude
        self.current_position_y = Ego_data.longitude


if __name__ == "__main__":
    try:
        get_Ego_Status()
    except rospy.ROSInterruptException:
        pass