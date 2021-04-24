"""Defines the ISAACS Server Interface."""

import time
import rospy
import roslibpy
import roslibpy.actionlib
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import WaypointReached

import interface_utils


class RadiationSubscriber():
    """Class that defines a ROS listener that saves radiation readings."""

    def __init__(self, radiation_topic):
        self.sub = rospy.Subscriber(radiation_topic, Vector3, self.callback)
        self.readings = []

    def clear_readings(self):
        """Clear the outstanding readings.

        This is called when a waypoint is reached to start collecting
        a new batch of readings.
        """
        self.readings = []

    def callback(self, data):
        """Callback to save subscriber data."""
        self.readings.append(data)


class WaypointProgressSubscriber():
    """Class that defines a ROS listener that listens to mission progress updates."""

    def __init__(self, waypoint_topic, radiation_sub):
        self.sub = rospy.Subscriber(waypoint_topic, WaypointReached,
                                    self.callback)
        self.radiation_sub = radiation_sub
        self.data = {}

    def callback(self, data):
        """Callback that saves the corresponding radiation readings."""
        waypoint_id = data[
            'waypoint_id']  # needs to be updated to match codebase
        self.data[waypoint_id] = self.radiation_sub.readings
        print(
            f"Hit waypoint {waypoint_id}, collected the corresponding readings: {self.data[waypoint_id]}"
        )
        self.radiation_sub.clear_readings()


"""Defines the IsaacsServerInterface class, which takes in a drone id and
sends the waypoints to that drone."""


class IsaacsServerInterface:
    """Connects to server, registers the drone and sends waypoints"""

    def __init__(self, radiation_topic, waypoint_topic):
        self.drone_id = None
        self.radiation_sub = RadiationSubscriber(radiation_topic)
        self.waypoint_sub = WaypointProgressSubscriber(waypoint_topic,
                                                       self.radiation_sub)

    def start_client(self, server_addr='localhost', server_port=9090):
        """Start the Roslib client and connect to server.

        Also initializes the ActionClientWorkarounds."""
        self.client = roslibpy.Ros(host=server_addr, port=server_port)
        self.client.run()
        print(self.client.is_connected)
        self.mission_uploader = roslibpy.actionlib.ActionClient(
            self.client, "isaacs_server/upload_mission",
            'isaacs_server/UploadMissionAction')
        self.drone_controller = roslibpy.actionlib.ActionClient(
            self.client, "isaacs_server/control_drone",
            'isaacs_server/ControlDroneAction')

    def get_drone_id(self, drone_name):
        """Call the all_drones_available service to find the drone ID.


        The all_drones_available service's callback asynchronously calls
        set_drone_id.

        Arguments:
            drone_name (str): The name of the drone of interest.
        """
        service = roslibpy.Service(self.client,
                                   'isaacs_server/all_drones_available',
                                   'isaacs_server/AllDronesAvailable')
        request = roslibpy.ServiceRequest({})
        print('Calling all_drones_available service...')
        result = service.call(
            request, callback=lambda r: self.set_drone_id(r, drone_name))
        print('Service response: {}'.format(result))

    def set_drone_id(self, response, drone_name):
        """Sets the interface's drone ID.

        Arguments:
            response (dict): The all_drones_available service response.
            drone_name (str): The name of the drone of interest.
        """
        for available_drones in response['drones_available']:
            if available_drones['name'] == drone_name:
                self.drone_id = available_drones['id']
                print("Setting drone_id to %d" % self.drone_id)
                return

    def manual_set_id(self, drone_id):
        """Manually set drone ID."""
        self.drone_id = drone_id

    def send_waypoints(self, waypoints):
        """Send waypoints to drone and upload mission."""
        if not self.drone_id:
            raise Exception("Drone id is not set.")
        mission = []
        for waypoint in waypoints:
            formatted_waypoint = interface_utils.convert_to_NSF(*waypoint)
            mission.append(formatted_waypoint)

        upload_goal = roslibpy.actionlib.Goal(
            self.mission_uploader,
            roslibpy.Message({
                'id': self.drone_id,
                "waypoints": mission
            }))
        upload_goal.on('feedback', lambda f: print(f['progress']))
        upload_goal.send()
        upload_result = upload_goal.wait(10)
        if not upload_result["success"]:
            raise Exception("Failure to upload mission")

        run_mission_goal = roslibpy.actionlib.Goal(
            self.drone_controller,
            roslibpy.Message({
                'id': self.drone_id,
                "control_task": "start_mission"
            }))
        run_mission_goal.on('feedback', lambda f: print(f['progress']))
        run_mission_goal.send()
        run_mission_result = run_mission_goal.wait(10)
        if not run_mission_result["success"]:
            raise Exception("Failure to run mission")


# "/mavros/mission/reached"
# http://docs.ros.org/en/api/mavros_msgs/html/msg/WaypointReached.html
# https://github.com/immersive-command-system/drone-mavros

def main():
    rospy.init_node("server_interface")

    interface = IsaacsServerInterface(radiation_topic="/lamp/data",
                                      waypoint_topic="/mavros/mission/reached")
    interface.start_client()

    topics_published = [{"name": "/lamp/data", "type": "geometry_msgs/Vector3"}]
    interface.get_drone_id("hexacopter")
    interface_utils.save_topics_to_drone(interface.client, 1, topics_published)

    time.sleep(5)

    waypoints = [(1, 0, 0), (1, 2, 3), (4, 5, 6), (10, 12, 14)]
    interface.send_waypoints(waypoints)

    interface.client.terminate()


if __name__ == "__main__":
    main()
