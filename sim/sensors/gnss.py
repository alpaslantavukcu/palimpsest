import math

class GnssHelper:
    def spawn(self, sim_world, blueprint, transform, attach):
        self.blueprint = blueprint
        self.transform = transform

        with sim_world:
            self.gnss = sim_world.spawn(self, attach)
            self.gnss.listen(self.on_received_gnss_data)
        
        self.snapshot = sim_world.snapshot.find(self.gnss.id)
    
    def on_received_gnss_data(self, gnss_data):
        self.x, self.y, alt = self.from_gps(gnss_data.latitude, gnss_data.longitude, gnss_data.altitude)
        #print("{}, {}".format(x, y))
    
    def from_gps(self, latitude: float, longitude: float, altitude: float):
        """Creates Location from GPS (latitude, longitude, altitude).
        This is the inverse of the _location_to_gps method found in
        https://github.com/carla-simulator/scenario_runner/blob/master/srunner/tools/route_manipulation.py

        https://github.com/erdos-project/pylot/blob/342c32eb598858ff23d7f24edec11414a3227885/pylot/utils.py
        """
        EARTH_RADIUS_EQUA = 6378137.0
        # The following reference values are applicable for towns 1 through 7,
        # and are taken from the corresponding OpenDrive map files.
        # LAT_REF = 49.0
        # LON_REF = 8.0
        # TODO: Do not hardcode. Get the references from the open drive file.
        LAT_REF = 49.0
        LON_REF = 8.0

        scale = math.cos(LAT_REF * math.pi / 180.0)
        basex = scale * math.pi * EARTH_RADIUS_EQUA / 180.0 * LON_REF
        basey = scale * EARTH_RADIUS_EQUA * math.log(
            math.tan((90.0 + LAT_REF) * math.pi / 360.0))

        x = scale * math.pi * EARTH_RADIUS_EQUA / 180.0 * longitude - basex
        y = scale * EARTH_RADIUS_EQUA * math.log(
            math.tan((90.0 + latitude) * math.pi / 360.0)) - basey

        # This wasn't in the original method, but seems to be necessary.
        y *= -1

        return x, y, altitude
