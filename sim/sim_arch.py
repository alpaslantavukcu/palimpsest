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
from syi.syi_dummy import SyiDummy
#from syi.dpc.exp.dist_pc import predict


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
    def __init__(self, client = carla.Client('localhost', 2000), spawn_flag = False, show_lane_object = False):
        self.client = client
        self.sim_world = SimWorld(client)
        self.pygame_setup()
        # Vehicle Init
        self.ego_vehicle = EgoVehicle()
        self.rgb_cam = Camera()
        self.gnss = GnssHelper()
        self.imu = ImuHelper()

        self.show_lane_obj = show_lane_object
        
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
        
        pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.
        self.font = pygame.font.SysFont(pygame.font.get_default_font(), 24)

        
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

        # SYI
        self.syi = SyiDummy()

        # Cam Capture
        self.cap = cv2.VideoCapture(0)

        #width, height = 256, 144

        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 144)

        # distracted predict
        #self.pred = predict()
    
    def loop(self):
        kb_control = KeyboardControl(self.ego_vehicle)
        kb_control.listen()
        js_control = JoystickControl()
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
                                if self.show_lane_obj:
                                    cur_img = self.ldetector.detect(cur_img)
                                    self.odetector.detect(cur_img)

                                surface = pygame.surfarray.make_surface(cur_img)
                                surface = pygame.transform.rotate(surface, -90)
                                surface = pygame.transform.flip(surface, True, False)
                                self.screen.blit(surface, (0,0))

                                ret, frame = self.cap.read()
                                img = cv2.resize(frame, (256, 144), interpolation = cv2.INTER_AREA)
                                img = pygame.image.frombuffer(img, (256, 144), "BGR")
                                self.screen.blit(img, (768,0))
                                
                                #self.pred.mainloop(frame)
                                #self.pred.detection(frame)

                                
                                tpm, pc, sdlp, strongest_label, PERCLOS, point = self.syi.update(frame)
                                #tpm, pc, sdlp = self.syi.update_random()
                                dec = self.syi.decision()
                                text_tpm  = self.font.render("TPM  : %.2f" % tpm,  True,  (255, 0, 0))
                                text_pc   = self.font.render("PC   : %.2f" % pc,   True,   (255, 0, 0))
                                text_sdlp = self.font.render("SDLP : %.2f" % sdlp, True, (255, 0, 0))
                                text_dec  = self.font.render("DEC  : %.2f" % dec, True, (255, 0, 0))
                                text_label = self.font.render("Label : %s" % strongest_label, True, (255, 0, 0))
                                text_pc2  = self.font.render("PER  : %.2f" % PERCLOS, True, (255, 0, 0))
                                text_point = self.font.render("Poi  : %.2f" % point, True, (255, 0, 0))
                                self.screen.blit(text_tpm, (768, 144))
                                self.screen.blit(text_pc, (768, 174))
                                self.screen.blit(text_sdlp, (768, 204))
                                self.screen.blit(text_dec, (768, 234))
                                self.screen.blit(text_label, (768, 264))
                                self.screen.blit(text_pc2, (768, 294))
                                self.screen.blit(text_point, (768, 324))

                                pygame.display.flip()
                                self.clk.tick(30)
                                
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

                                if kb_control.flag:
                                    traj = self.ldetector.get_trajectory_from_lane_detector(cur_img)
                                    throttle, steer = self.controller.get_control(traj, speed, desired_speed=10, dt=0.1)
                                    print("steer : {}".format(steer))
                                    print("throttle : {}".format(throttle))
                                    self.ego_vehicle.controller.steer = steer
                                    self.ego_vehicle.controller.throttle = np.clip(throttle, 0, 0.7)
                                else:
                                    print("Manual Control!!!")
                                    js_control.get_control(self.ego_vehicle.controller)
                                    #self.ego_vehicle.controller.throttle = 0.4

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


