import numpy as np

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
    
    def pop_image(self):
        key_list = list(self.img_dict.keys())
        if len(key_list) < 2:
            return list()
        return self.to_rgb_array(self.img_dict[key_list[-2]])
        


    def to_bgra_array(self, image):
        """Convert a CARLA raw image to a BGRA np array."""
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        return array


    def to_rgb_array(self, image):
        """Convert a CARLA raw image to a RGB np array."""
        array = self.to_bgra_array(image)
        # Convert BGRA to RGB.
        array = array[:, :, :3]
        #array = array[:, :, ::-1]
        return array
    
    def flush(self, show = False, video_flag = False, height = 1024, width = 512):
        video_name = 'sim_{}'.format(time.time())
        if video_flag:
            video = cv2.VideoWriter(video_name, 0, 10, (height, width))
        
        for frame, img in self.img_dict.items():
            img.save_to_disk('output/%06d.png' % frame)
            print(type(img))
            if video_flag:
                video.write(self.to_rgb_array(img))
        
        if video_flag:
            video.release()