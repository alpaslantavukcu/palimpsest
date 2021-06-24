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
from syi.syi_helper import SyiHelper
from PIL import Image

vec_dif_list = list()

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
    
    def __exit__(self, type, value, traceback):
        self.update()

    def update(self):
        self.car.apply_control(self.controller)



class SimWorld:
    def __init__(self, client, spawn_actors = False):
        print(client.get_available_maps())
        client.load_world('/Game/Carla/Maps/Town04')
        
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
        

def draw_arrow(screen, colour, start, end, rot = 180):
    pygame.draw.line(screen,colour,start,end,2)
    rotation = math.degrees(math.atan2(start[1]-end[1], end[0]-start[0]))+rot
    pygame.draw.polygon(screen, colour, ((end[0]+10*math.sin(math.radians(rotation)), end[1]+10*math.cos(math.radians(rotation))), (end[0]+10*math.sin(math.radians(rotation-120)), end[1]+10*math.cos(math.radians(rotation-120))), (end[0]+10*math.sin(math.radians(rotation+120)), end[1]+10*math.cos(math.radians(rotation+120)))))


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

        self.p_tpm = 11
        self.p_sdlp = 0
        self.p_PERCLOS = 0
        self.p_point = 0

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
        
        pygame.font.init()
        self.font = pygame.font.SysFont('couriernew', 20, bold=True)

        
    def setup(self):
        # Vehicle Spawn
        ego_blueprint = self.sim_world.blueprint_library.find('vehicle.mini.cooperst')
        ego_transform = self.sim_world.world.get_map().get_spawn_points()[5]
        #ego_transform = carla.Transform(carla.Location(x=-88.3, y=21.5, z=0.35), carla.Rotation(pitch=0.35, yaw=89.8, roll=-0.0))
        #ego_transform = carla.Transform(carla.Location(x=-88.6, y=151.9, z=0.35), carla.Rotation(pitch=0.35, yaw=89.8, roll=-0.0))
        self.ego_vehicle.spawn(self.sim_world, ego_blueprint, ego_transform)

        # Camera Spawns
        camera_bp = self.sim_world.blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('sensor_tick', '0.1')
        camera_bp.set_attribute("image_size_x",str(1024))
        camera_bp.set_attribute("image_size_y",str(512))
        camera_bp.set_attribute('fov', '60')
        camera_transform = carla.Transform(carla.Location(x = 0.5, z = 1.3),carla.Rotation(pitch = -5))
        self.rgb_cam.spawn(self.sim_world, camera_bp, camera_transform, attach = self.ego_vehicle.car)

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
        self.syi = SyiHelper()

        # Cam Capture
        self.cap = cv2.VideoCapture(0)
    
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
                                self.ldetector.detect(cur_img)
                                if self.show_lane_obj:
                                    #cur_img = self.ldetector.draw(cur_img)
                                    cur_img = self.odetector.detect(cv2.cvtColor(cur_img, cv2.COLOR_RGB2BGR))
                                
                                #cur_img = cv2.cvtColor(cur_img, cv2.COLOR_BGR2RGB)
                                surface = pygame.surfarray.make_surface(cur_img)
                                surface = pygame.transform.rotate(surface, -90)
                                surface = pygame.transform.flip(surface, True, False)
                                self.screen.blit(surface, (0,0))


                                vehicle_diff = self.ldetector.calculate_vehicle_diff()
                                vec_dif_list.append(vehicle_diff)

                                
                                ret, frame = self.cap.read()
                                img = cv2.resize(frame, (256, 144), interpolation = cv2.INTER_AREA)
                                img = pygame.image.frombuffer(img, (256, 144), "BGR")
                                self.screen.blit(img, (768,0))
                                
                                
                                vehicle_diff = self.ldetector.calculate_vehicle_diff()
                                tpm, sdlp, strongest_label, pc, point = self.syi.update(frame, vehicle_diff)
                                dec = self.syi.decision()

                                if dec == 1:
                                    dec_txt_color = (37, 245, 47)
                                    dec_txt = "GUVENLI"
                                elif dec == 2:
                                    dec_txt_color = (252, 245, 38)
                                    dec_txt = "DUSUK RISK"
                                elif dec == 3:
                                    dec_txt_color = (255, 158, 40)
                                    dec_txt = "ORTA RISK"
                                elif dec == 4:
                                    dec_txt_color = (255, 0, 0)
                                    dec_txt = "YUKSEK RISK"
                                    kb_control.flag = True

                                txt_color = (255, 255, 255)
                                if tpm >= 7:
                                    text_tpm  = self.font.render( "TPM     : %.2f" % tpm,  True,  txt_color)
                                else:
                                    text_tpm  = self.font.render( "TPM     : %.2f" % tpm,  True,  (255, 0, 0))
                                
                                if pc < 0.4:
                                    text_pc   = self.font.render( "PC      : %.2f" % pc,   True,   txt_color)
                                else:
                                    text_pc   = self.font.render( "PC      : %.2f" % pc,   True,   (255, 0, 0))

                                if sdlp < 0.45:
                                    text_sdlp = self.font.render( "SDLP    : %.2f" % sdlp, True, txt_color)
                                else:
                                    text_sdlp = self.font.render( "SDLP    : %.2f" % sdlp, True, (255, 0, 0))

                                if point < 4:
                                    text_point = self.font.render("PUAN    : %.2f" % point, True, txt_color)
                                else:
                                    text_point = self.font.render("PUAN    : %.2f" % point, True, (255, 0, 0))

                                text_dec  = self.font.render( "DURUM : %s" % dec_txt, True, dec_txt_color)
                                text_label = self.font.render("ETIKET  : %s" % strongest_label, True, txt_color)
                                
                                

                                if kb_control.flag:
                                    text_status = self.font.render("OTOMATIK KONTROL ", True, (255, 0 , 0))
                                else:
                                    text_status = self.font.render("MANUEL KONTROL ", True, (0, 255 , 0))
                                    

                                
                                rect = pygame.Surface((256, 368))
                                rect.set_alpha(168)
                                rect.fill((102, 102, 153))
                                
                                self.screen.blit(rect, (768, 144))

                                self.screen.blit(text_tpm, (778, 154))
                                if tpm == self.p_tpm:
                                    draw_arrow(self.screen, (255, 255, 255), (988, 164), (988, 164), 90)
                                elif random.random() > 0.25:
                                    if tpm < self.p_tpm:
                                        draw_arrow(self.screen, (255, 0, 0), (988, 164), (988, 164), 0)
                                    elif tpm != self.p_tpm:
                                        draw_arrow(self.screen, (0, 255, 0), (988, 164), (988, 164), 180)

                                self.screen.blit(text_pc, (778, 184))
                                if self.p_PERCLOS == pc:
                                    draw_arrow(self.screen, (255, 255, 255), (988, 194), (988, 194), 90)
                                if random.random() > 0.25:
                                    if pc > self.p_PERCLOS:
                                        draw_arrow(self.screen, (255, 0, 0), (988, 194), (988, 194), 180)
                                    elif pc != self.p_PERCLOS:
                                        draw_arrow(self.screen, (0, 255, 0), (988, 194), (988, 194), 0)                                

                                self.screen.blit(text_sdlp, (778, 214))
                                if sdlp == self.p_sdlp:
                                    draw_arrow(self.screen, (255, 255, 255), (988, 224), (988, 224), 90)
                                if random.random() > 0.25:
                                    if sdlp > self.p_sdlp:
                                        draw_arrow(self.screen, (255, 0, 0), (988, 224), (988, 224), 180)
                                    elif sdlp != self.p_sdlp:
                                        draw_arrow(self.screen, (0, 255, 0), (988, 224), (988, 224), 0)  

                                self.screen.blit(text_point, (778, 244))
                                if self.p_point == point:
                                    draw_arrow(self.screen, (255, 255, 255), (988, 254), (988, 254), 90)
                                if random.random() > 0.25:
                                    if point > self.p_point:
                                        draw_arrow(self.screen, (255, 0, 0), (988, 254), (988, 254), 180)
                                    elif point != self.p_point:
                                        draw_arrow(self.screen, (0, 255, 0), (988, 254), (988, 254), 0)  

                                self.screen.blit(text_label, (778, 274))

                                self.screen.blit(text_dec, (778, 334))

                                self.screen.blit(text_status, (778, 424))


                                
                                pygame.display.flip()
                                self.clk.tick(30)

                                self.p_tpm, self.p_sdlp, self.p_point, self.p_PERCLOS = tpm, sdlp, point, pc
                                
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
                                speed = np.linalg.norm( carla_vec_to_np_array(self.ego_vehicle.car.get_velocity()))

                                traj = self.ldetector.get_trajectory_from_lane_detector()
                                

                                
                                if kb_control.flag:
                                    traj = self.ldetector.get_trajectory_from_lane_detector()
                                    throttle, steer = self.controller.get_control(traj, speed, desired_speed=10, dt=0.1)
                                    self.ego_vehicle.controller.steer = steer
                                    #self.ego_vehicle.controller.throttle = 0.4
                                    self.ego_vehicle.controller.throttle = np.clip(throttle, 0, 0.4)
                                else:
                                    js_control.get_control(self.ego_vehicle.controller)

                            Vx = self.ego_vehicle.car.get_velocity().x
                            Yr = self.imu.Yr
                            x = self.gnss.x
                            y = self.gnss.y
                            hxEst, hxTrue = self.ekf.run(x, y, Vx, Yr)
                            flag = False
                        except TypeError:
                            pass
        except RuntimeError:
            pass
        finally:
            f = open("diff.txt", "w")
            print(vec_dif_list, file=f)
            #self.rgb_cam.flush()
            pass





s = Simulator(spawn_flag=False, show_lane_object=False)
s.setup()
s.loop()


