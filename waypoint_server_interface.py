"""Defines the WaypointServerInterface class, which registers drones with the ISAACS server and
can send waypoints for each registered drone."""

import roslibpy
from sensor_msgs.msg import NavSatFix

"""Preprocess the container point cloud and find the raster path"""
search_directory = "./"
in_pc_path = os.path.join(search_directory, "25d_051_2020_11_25_18_54_50_cleaned.ply")
cpc = ContainerPointCloud(in_pc_path=in_pc_path, verbose=False)
cpc.remove_ground_points()
cpc.cluster_entities()
cpc.calculate_surfaces()
cpc.rasterize_surfaces()

tour, avg_pts, connections = container_extraction.connect_faces(cpc.raster_paths)
final_connections = container_extraction.make_full_path(tour,
    avg_pts, cpc.raster_paths, connections, cpc.container_surface_corners)

class WaypointServerInterface:
    """Connects to server, registers the drone and sends waypoints"""

    def __init__(self, server_addr='localhost', server_port=9090):
        self.server_addr = server_addr
        self.server_port = server_port
        self.client = roslibpy.Ros(host=self.server_addr, port=self.server_port)
        self.client.run()

    def register_drone_with_server(self, drone_name, drone_type, drone_topics):
        """Registers a drone with the server.

        Arguments:
            drone_name: name of the drone
            drone_type: type of the drone (DjiMatrice, Mavros)
            drone_topics: list of dicts specifying name and type of
                          the topic that the drone publishes to 
        """
        register_service = roslibpy.Service(self.client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        register_request = roslibpy.ServiceRequest({'drone_name': drone_name, "drone_type": drone_type})
        print("Registering drone...")
        result = register_service.call(register_request)
        print(f"Service response: {result}")
        print()

        drone_id = result.data['id']
        save_topics_service = roslibpy.Service(self.client, 'isaacs_server/save_drone_topics', 'isaacs_server/save_drone_topics')
        save_topics_request = roslibpy.ServiceRequest({'id': drone_id, 'publishes': drone_topics})

        print('Calling topics_service...')
        result = save_topics_service.call(save_topics_request)
        print(f'Topics_service response{result}')
        print()

        return drone_id

    def send_waypoints(self, drone_id, waypoints):
        """Send 3D waypoints to server.

        Arguments:
            drone_id: id of the drone
            waypoints: list of tuples; each tuple specifies latitude, longitude, and altitude
        """
        service = roslibpy.Service(self.client, 'isaacs_server/upload_mission', 'isaacs_server/upload_mission')
        satellite_points = []
        for point in waypoints:
            satellite_point = {}
            satellite_point['latitude'], satellite_point['longitude'], satellite_point['altitude'] = point  
            satellite_points.append(satellite_point)
        request = roslibpy.ServiceRequest({'id': drone_id, 'waypoints': satellite_points})
        print("Sending request drone...")
        result = service.call(request)
        print(f"Service response: {result}")
        return result

if __name__ == "__main__":
    drone_name = 'test_drone'
    drone_type = "DjiMatrice"
    topics_published = [{'name': 'test_name',
                        'type': 'std_msgs/String'}]
    waypoints = [(1, 0, 0), (1, 2, 3), (4, 5, 6)]

    wsi = WaypointServerInterface()
    drone_id = wi.register_drone_with_server(drone_name, drone_type, topics_published)
    wsi.send_waypoints(drone_id, final_connections)