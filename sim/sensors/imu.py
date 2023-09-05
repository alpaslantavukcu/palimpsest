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
        #print("Vx : {} , Yaw Rate : {}".format(self.Vx, self.Yr))

