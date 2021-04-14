"""Defines the IsaacsServerInterface class, which takes in a drone id and
sends the waypoints to that drone."""

import roslibpy

class IsaacsServerInterface:
    """Connects to server, registers the drone and sends waypoints"""

    def __init__(self, server_addr='localhost', server_port=9090):
        self.drone_id = None
        self.server_addr = server_addr
        self.server_port = server_port
        self.client = roslibpy.Ros(host=self.server_addr, port=self.server_port)
        self.client.run()

    def get_drone_id():
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/AllDronesAvailable')
        request = roslibpy.ServiceRequest({})

        print('Calling all_drones_available service...')
        result = service.call(request, callback=lambda r : print("hi", r))
        print('Service response: {}'.format(result))