class LaserRangeSensor:
    def __init__(self, robot, timestep):
        """Initialize laser range sensor"""
        # Get and enable the laser range sensor if available
        try:
            self.laser = robot.getDevice('laser')
            if self.laser:
                print("Successfully initialized laser range sensor")
                self.laser.enable(timestep)
            else:
                print("Laser range sensor 'laser' not found")
        except Exception as e:
            print(f"Error initializing laser range sensor: {e}")
            self.laser = None
            
        # Get and enable point cloud if available
        try:
            self.point_cloud = robot.getDevice('laser_point_cloud')
            if self.point_cloud:
                print("Successfully initialized laser point cloud")
                self.point_cloud.enable(timestep)
            else:
                print("Laser point cloud 'laser_point_cloud' not found")
        except Exception as e:
            print(f"Error initializing laser point cloud: {e}")
            self.point_cloud = None
            
        # Only wait for reading if we have sensors
        if self.laser or self.point_cloud:
            robot.step(timestep)
            print("Laser sensor(s) ready for reading")
        
    def get_distances(self):
        """Get current distance readings"""
        # Get raw laser data if available
        ranges = None
        if self.laser:
            #try:
                ranges = self.laser.getRangeImage()
                #if ranges and len(ranges) > 0:
                  #  print(f"Received laser scan with {len(ranges)} points")
           # except Exception as e:
           #     print(f"Error getting laser range data: {e}")
        
        # Get point cloud if available
        points = None
        if self.point_cloud:
            try:
                points = self.point_cloud.getPointCloud()
                if points and len(points) > 0:
                    print(f"Received point cloud with {len(points)} points")
            except Exception as e:
                print(f"Error getting point cloud data: {e}")
            
        return {
            'ranges': ranges,
            'point_cloud': points,
            'has_sensor': bool(self.laser or self.point_cloud)
        } 