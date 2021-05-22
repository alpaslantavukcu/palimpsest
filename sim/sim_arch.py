import carla
from pynput import keyboard
from configparser import ConfigParser
import math
import pygame
import cv2
import asyncio
import numpy



class KeyboardControl:
    def __init__(self, vehicle):
        self.ego_vehicle = vehicle

    def listen(self):
        # Collect events until released
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

    def on_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.ego_vehicle.controller.throttle = min(self.ego_vehicle.controller.throttle + 0.01, 1)
            else:
                self.ego_vehicle.controller.throttle = 0.0

            if key == keyboard.Key.down:
                self.ego_vehicle.controller.brake = min(self.ego_vehicle.controller.brake + 0.2, 1)
            else:
                self.ego_vehicle.controller.brake = 0
            
            self.ego_vehicle.update()

            #print('alphanumeric key {0} pressed'.format(key.char))
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_release(self, key):
        #print('{0} released'.format(key))
        if key == keyboard.Key.esc:
            # Stop listener
            return False


# Direksiyon kontrolü için yazılan bir sınıf.

class JoystickControl:
    class JoystickInfo:
        def __init__(self):
            # Konfigürasyon dosyasından mevcut joystick elemanına dair ayarlar okunuyor.
            parser = ConfigParser()
            parser.read(r'D:\Belgeler D\BÇ\palimpsest\palimpsest\sim\wheel_config.ini')
            self.steer_idx = int(parser.get('G29 Racing Wheel', 'steering_wheel'))
            self.throttle_idx = int(parser.get('G29 Racing Wheel', 'throttle'))
            self.brake_idx = int(parser.get('G29 Racing Wheel', 'brake'))
            self.reverse_idx = int(parser.get('G29 Racing Wheel', 'reverse'))
            self.handbrake_idx = int(parser.get('G29 Racing Wheel', 'handbrake'))
    
    #Carla controlü temsilen Joystick test için yazılmış bir sınıf muhtemelen silinecek.
    class ControlDummy: 
        def __init__(self):
            self.steer = None
            self.brake = None
            self.throttle = None
            self.hand_brake = None

        def __str__(self):
            return "Steer : {} || Throttle : {} || Brake : {}".format(self.steer, self.throttle, self.brake)
    
    def __init__(self):
        #pygame başlatılıyor ve joysticke bağlanmaya çalışılıyor
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.clock = pygame.time.Clock()
        self.jinfo = JoystickControl.JoystickInfo()
        self.cdumm = JoystickControl.ControlDummy()
        self.joystick.init()

    #joystick'ten veri okuyup okuduğu verileri carla için anlamlı parçalara bölen fonksiyon
    def parse_vehicle_wheel(self, controller):
        numAxes = self.joystick.get_numaxes()
        jsInputs = [float(self.joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self.joystick.get_button(i)) for i in
                    range(self.joystick.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self.jinfo.steer_idx])

        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self.jinfo.throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self.jinfo.brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        controller.steer = steerCmd
        controller.brake = brakeCmd
        controller.throttle = throttleCmd

        #toggle = jsButtons[self._reverse_idx]
        #self.cdumm.hand_brake = bool(jsButtons[self.jinfo.handbrake_idx])

        return steerCmd, brakeCmd, throttleCmd
    
    # control komutlarını döndüren fonksiyon
    def get_control(self, controller):
        pygame.event.pump()
        self.clock.tick_busy_loop(1000)
        return self.parse_vehicle_wheel(controller)


class GnssHelper:
    def spawn(self, sim_world, blueprint, transform, attach):
        self.blueprint = blueprint
        self.transform = transform

        with sim_world:
            self.gnss = sim_world.spawn(self, attach)
            self.gnss.listen(self.on_received_gnss_data)
        
        self.snapshot = sim_world.snapshot.find(self.gnss.id)
    
    def on_received_gnss_data(self, gnss_data):
        x, y, alt = self.from_gps(gnss_data.latitude, gnss_data.longitude, gnss_data.altitude)
        print("{}, {}".format(x, y))
    
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


class ImuHelper:
    def spawn(self, sim_world, blueprint, transform, attach):
        self.blueprint = blueprint
        self.transform = transform
        self.Vx = 0
        self.Yr = 0

        with sim_world:
            self.imu = sim_world.spawn(self, attach)
            self.imu.listen(self.on_received_imu_data)
        
        self.snapshot = sim_world.snapshot.find(self.imu.id)
    
    def on_received_imu_data(self, imu_data):
        #print(imu_data)
        dt = 0.1
        self.Vx = self.Vx + imu_data.accelerometer.x * dt
        self.Yr = imu_data.gyroscope.z
        print("Vx : {} , Yaw Rate : {}".format(self.Vx, self.Yr))


# Kamera aktörünü temsil eden sınıf
# Kamera fps artırmak için ne yapılabilir? Gerçek zamanlı kamera uygulaması için ne gerekli.

class Camera:
    def spawn(self, sim_world, blueprint, transform, attach):
        self.blueprint = blueprint
        self.transform = transform
        self.img_dict = {}

        with sim_world:
            self.cam = sim_world.spawn(self, attach)
            self.cam.listen(self.listener)
        self.snapshot = sim_world.snapshot.find(self.cam.id)
    
    # kameranın yakaladığı her frame tetiklediği fonksiyon
    def listener(self, image):
        self.img_dict[image.frame] = image
        #print(image.frame)
        #asyncio.create_task(image.save_to_disk('output/%06d.png' % image.frame))
    
    def to_bgra_array(self, image):
        """Convert a CARLA raw image to a BGRA numpy array."""
        array = numpy.frombuffer(image.raw_data, dtype=numpy.dtype("uint8"))
        array = numpy.reshape(array, (image.height, image.width, 4))
        return array


    def to_rgb_array(self, image):
        """Convert a CARLA raw image to a RGB numpy array."""
        array = self.to_bgra_array(image)
        # Convert BGRA to RGB.
        array = array[:, :, :3]
        #array = array[:, :, ::-1]
        return array
    
    def flush(self):
        video_name = 'mest.avi'
        video = cv2.VideoWriter(video_name, 0, 10, (800, 600))
        for frame, img in self.img_dict.items():
            #img.save_to_disk('output/%06d.png' % frame)
            video.write(self.to_rgb_array(img))
        video.release()
            




class EgoVehicle:
    def __init__(self, controller = carla.VehicleControl()):
        self.controller = controller
        
    
    def spawn(self, sim_world, blueprint, transform):
        self.blueprint = blueprint
        self.transform = transform

        with sim_world:
            self.car = sim_world.spawn(self)
        self.snapshot = sim_world.snapshot.find(self.car.id)
    
    def __enter__(self):
        pass
    
    # bu sentaks update işleminin sürekli manuel çağrılmaması ve alternatif senaryolar için düşünüldü
    # bununla beraber enter fonkiyonunun olmaması gerekliliğini sorgulatmakta
    def __exit__(self, type, value, traceback):
        self.update()

    def update(self):
        self.car.apply_control(self.controller)



class SimWorld:
    def __init__(self, client):
        self.world = client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.snapshot = self.world.get_snapshot()
        self.spectator = self.world.get_spectator()
        self.set_setting()
    
    def set_setting(self, sync_mode = True, fixed_delta = 0.1):
        with self:
            self.settings = self.world.get_settings()
            self.settings.synchronous_mode = sync_mode
            self.settings.fixed_delta_seconds = fixed_delta
            self.world.apply_settings(self.settings)

    
    def __enter__(self): pass
    
    def __exit__(self, type, value, traceback):
        self.world.tick()
        self.snapshot = self.world.get_snapshot()

    def spawn(self, actor, attach = None):
        return self.world.spawn_actor(actor.blueprint, actor.transform, attach_to = attach)
        



class Simulator:
    def __init__(self, client = carla.Client('localhost', 2000)):
        self.client = client
        self.sim_world = SimWorld(client)

        # Vehicle Init
        self.ego_vehicle = EgoVehicle()
        self.rgb_cam = Camera()
        self.gnss = GnssHelper()
        self.imu = ImuHelper()
    def setup(self):
        # Vehicle Spawn
        ego_blueprint = self.sim_world.blueprint_library.find('vehicle.mini.cooperst')
        ego_transform = carla.Transform(carla.Location(x=-88.3, y=21.5, z=0.35), carla.Rotation(pitch=0.35, yaw=89.8, roll=-0.0))
        self.ego_vehicle.spawn(self.sim_world, ego_blueprint, ego_transform)
        print(self.ego_vehicle.snapshot.get_transform())

        # Camera Spawns
        camera_bp = self.sim_world.blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('sensor_tick', '0.02')
        camera_transform = carla.Transform(carla.Location(x = -10, z=10), carla.Rotation(pitch = -30))
        self.rgb_cam.spawn(self.sim_world, camera_bp, camera_transform, attach = self.ego_vehicle.car)
        print(self.rgb_cam.snapshot.get_transform())

        # Gnss Spawn
        gnss_bp = self.sim_world.blueprint_library.find('sensor.other.gnss')
        self.gnss.spawn(self.sim_world, gnss_bp, carla.Transform(), attach=self.ego_vehicle.car)

        # Imu Spawn
        imu_bp = self.sim_world.blueprint_library.find('sensor.other.imu')
        self.imu.spawn(self.sim_world, imu_bp, carla.Transform(), attach=self.ego_vehicle.car)
        
        with self.sim_world:
            self.sim_world.spectator.set_transform(self.rgb_cam.snapshot.get_transform())
        
    
    def loop(self):
        kb_control = KeyboardControl(self.ego_vehicle)
        kb_control.listen()
        try : 
            while True:
                with self.sim_world:
                    self.sim_world.spectator.set_transform(self.sim_world.snapshot.find(self.rgb_cam.cam.id).get_transform())
                    with self.ego_vehicle:
                        print('')
        except RuntimeError:
            pass
        finally:
            self.rgb_cam.flush()





s = Simulator()
s.setup()
s.loop()


