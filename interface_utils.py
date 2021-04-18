"""Defines utility functions used for interfacing with the server."""

import roslibpy
import roslibpy.actionlib
from sensor_msgs.msg import NavSatFix

def convert_to_NSF(lat, long, alt):
    """Convert geographic coordinates into a NavSatFix dict object.
    """
    ret = {}
    ret["header"] = {'seq': 885, 'stamp': {'secs' : 1552399290, 'nsecs': 267234086}, 'frame_id': "/wgs84"}
    ret["status"] = {"status": 0, "service": 1}
    ret["latitude"] = lat
    ret["longitude"] = long
    ret["altitude"] = alt
    ret["position_covariance"] = [0,0,0,0,0,0,0,0,0]
    ret["position_covariance_type"] = 0
    return ret

def register_dummy_drone(client, drone_name, drone_type, drone_topics):
    """Registers a drone with the server.

    Arguments:
        drone_name: name of the drone
        drone_type: type of the drone (DjiMatrice, Mavros)
        drone_topics: list of dicts specifying name and type of
                      the topic that the drone publishes to 
    """
    register_service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
    register_request = roslibpy.ServiceRequest({'drone_name': drone_name, "drone_type": drone_type})
    print("Registering drone...")
    result = register_service.call(register_request)
    print(f"Service response: {result}")
    print()

    drone_id = result.data['id']
    save_topics_service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/SaveDroneTopics')
    save_topics_request = roslibpy.ServiceRequest({'id': drone_id, 'publishes': drone_topics})

    print('Calling topics_service...')
    result = save_topics_service.call(save_topics_request)
    print(f'Topics_service response{result}')
    print()
    return drone_id


class ActionClientWorkaround(roslibpy.actionlib.ActionClient):
    def setCustomTopics(self):
        # Sets topic to custom action topics
        self.feedback_listener = roslibpy.Topic(self.ros, self.server_name + '/Actionfeedback', self.action_name + 'ActionFeedback')
        self.result_listener = roslibpy.Topic(self.ros, self.server_name + '/Actionresult', self.action_name + 'ActionResult')
        self.goal_topic = roslibpy.Topic(self.ros, self.server_name + '/Actiongoal', self.action_name + 'ActionGoal')
        # Advertise the goal and cancel topics
        self.goal_topic.advertise()
        # Subscribe to the feedback topic
        if not self.omit_feedback:
            self.feedback_listener.subscribe(self._on_feedback_message)
        # Subscribe to the result topic
        if not self.omit_result:
            self.result_listener.subscribe(self._on_result_message)
