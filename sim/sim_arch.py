import carla
import math
import pygame
import cv2
import numpy as np
import time
import random
from lane_detection.lane_detector_helper import LaneDetectorHelper
from sensors.camera import Camera
from manual_controls import JoystickControl
from manual_controls import KeyboardControl
from sensors.gnss import GnssHelper
from sensors.imu import ImuHelper
from object_detection.object_detector import ObjectDetector
from localization.ekf import EKF
from pure_pursuit import PurePursuitPlusPID


def carla_vec_to_np_array(vec):
    return np.array([vec.x,
                     vec.y,
                     vec.z])

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
    def __init__(self, client, spawn_actors = False):
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
    def __init__(self, client = carla.Client('localhost', 2000), spawn_flag = False):
        self.client = client
        self.sim_world = SimWorld(client)
        self.pygame_setup()
        # Vehicle Init
        self.ego_vehicle = EgoVehicle()
        self.rgb_cam = Camera()
        self.gnss = GnssHelper()
        self.imu = ImuHelper()
        
        if spawn_flag:
            self.spawn_passive_actors()
    

    def spawn_passive_actors(self, n = 50):
        world = self.sim_world.world
        for i in range(n):
            blueprint = random.choice(world.get_blueprint_library().filter('walker.*'))
            spawn_points = world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            world.try_spawn_actor(blueprint, spawn_point)


    def pygame_setup(self):
        pygame.init()
        w = 1024
        h = 512
        size = (w, h)
        self.screen = pygame.display.set_mode(size)
        self.clk = pygame.time.Clock()

        
    def setup(self):
        # Vehicle Spawn
        ego_blueprint = self.sim_world.blueprint_library.find('vehicle.mini.cooperst')
        ego_transform = carla.Transform(carla.Location(x=-88.6, y=151.9, z=0.35), carla.Rotation(pitch=0.35, yaw=89.8, roll=-0.0))
        self.ego_vehicle.spawn(self.sim_world, ego_blueprint, ego_transform)
        print(self.ego_vehicle.snapshot.get_transform())

        # Camera Spawns
        camera_bp = self.sim_world.blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('sensor_tick', '0.1')
        camera_bp.set_attribute("image_size_x",str(1024))
        camera_bp.set_attribute("image_size_y",str(512))
        camera_bp.set_attribute('fov', '60')
        camera_transform = carla.Transform(carla.Location(x = 0.5, z = 1.3),carla.Rotation(pitch = -5))
        self.rgb_cam.spawn(self.sim_world, camera_bp, camera_transform, attach = self.ego_vehicle.car)
        print(self.rgb_cam.snapshot.get_transform())

        # Lane Detector
        self.ldetector = LaneDetectorHelper()
        # Object Detector
        self.odetector = ObjectDetector()
        # Gnss Spawn
        gnss_bp = self.sim_world.blueprint_library.find('sensor.other.gnss')
        self.gnss.spawn(self.sim_world, gnss_bp, carla.Transform(), attach=self.ego_vehicle.car)

        # Imu Spawn
        imu_bp = self.sim_world.blueprint_library.find('sensor.other.imu')
        self.imu.spawn(self.sim_world, imu_bp, carla.Transform(), attach=self.ego_vehicle.car)
        
        with self.sim_world:
            self.sim_world.spectator.set_transform(self.rgb_cam.snapshot.get_transform())
        
        # EKF
        self.ekf = EKF()
        self.ekf.set(-584315, 4116700, 1, 0.1)

        # PurePursuit
        self.controller = PurePursuitPlusPID()
        
    
    def loop(self):
        kb_control = KeyboardControl(self.ego_vehicle)
        kb_control.listen()
        #js_control = JoystickControl()
        try : 
            while True:
                with self.sim_world:
                    self.sim_world.spectator.set_transform(self.sim_world.snapshot.find(self.rgb_cam.cam.id).get_transform())
                    with self.ego_vehicle:
                        try :
                            #js_control.get_control(self.ego_vehicle.controller)
                            self.ego_vehicle.controller.throttle = 0.3
                            cur_img = self.rgb_cam.pop_image()
                            if len(cur_img) == 0:
                                pass
                            else:
                                #img = pygame.image.load('output/temp.png')
                                for event in pygame.event.get():
                                    if event.type == pygame.QUIT:
                                        break
                                
                                cur_img = cur_img[:,:,::-1]
                                cur_img = self.ldetector.detect(cur_img)
                                self.odetector.detect(cur_img)
                                surface = pygame.surfarray.make_surface(cur_img)
                                surface = pygame.transform.rotate(surface, -90)
                                surface = pygame.transform.flip(surface, True, False)
                                self.screen.blit(surface, (0,0))
                                pygame.display.flip()
                                self.clk.tick(30)
                                #pygame.time.delay(1500)
                                traj = self.ldetector.get_trajectory_from_lane_detector(cur_img)
                                
                                # get velocity and angular velocity
                                vel = carla_vec_to_np_array(self.ego_vehicle.car.get_velocity())
                                forward = carla_vec_to_np_array(self.ego_vehicle.car.get_transform().get_forward_vector())
                                right = carla_vec_to_np_array(self.ego_vehicle.car.get_transform().get_right_vector())
                                up = carla_vec_to_np_array(self.ego_vehicle.car.get_transform().get_up_vector())
                                vx = vel.dot(forward)
                                vy = vel.dot(right)
                                vz = vel.dot(up)
                                ang_vel = carla_vec_to_np_array(self.ego_vehicle.car.get_angular_velocity())
                                w = ang_vel.dot(up)
                                print("vx vy vz w {:.2f} {:.2f} {:.2f} {:.5f}".format(vx,vy,vz,w))

                                speed = np.linalg.norm( carla_vec_to_np_array(self.ego_vehicle.car.get_velocity()))
                                """
                                actor_vel = self.ego_vehicle.car.get_velocity()
                                vx = actor_vel.x
                                vy = actor_vel.y
                                vz = actor_vel.z
                                
                                print("vx vy vz w {:.2f} {:.2f} {:.2f}".format(vx,vy,vz))
                                speed = np.linalg.norm([actor_vel.x, actor_vel.y, actor_vel.z])
                                """
                                throttle, steer = self.controller.get_control(traj, speed, desired_speed=25, dt=0.1)
                                print("steer : {}".format(steer))
                                self.ego_vehicle.controller.steer = steer

                            Vx = self.ego_vehicle.car.get_velocity().x
                            Yr = self.imu.Yr
                            x = self.gnss.x
                            y = self.gnss.y
                            hxEst, hxTrue = self.ekf.run(x, y, Vx, Yr)
                            print("-------------------------------------------------------")
                            print(hxEst)
                            print("Sensor Data : ")
                            print("x : {}, y : {}, Vx : {}, Yr : {} ".format(x, y, Vx, Yr))
                            print("-------------------------------------------------------")
                            self.ego_vehicle.controller.throttle = 0.4
                            flag = False
                        except TypeError:
                            pass
        except RuntimeError:
            pass
        finally:
            #self.rgb_cam.flush()
            pass





s = Simulator(spawn_flag=True)
s.setup()
s.loop()


