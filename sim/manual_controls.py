import pygame

from pynput import keyboard
from configparser import ConfigParser

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
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_release(self, key):
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



