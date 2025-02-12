class LaserRangeSensor:
    def __init__(self, robot, timestep):
        """Initialize laser range sensor"""
        # Get and enable the laser range sensor if available
        try:
            self.laser = robot.getDevice('laser')
            if self.laser:
                self.laser.enable(timestep)
        except:
            self.laser = None
            
        # Get and enable point cloud if available
        try:
            self.point_cloud = robot.getDevice('laser_point_cloud')
            if self.point_cloud:
                self.point_cloud.enable(timestep)
        except:
            self.point_cloud = None
            
        # Only wait for reading if we have sensors
        if self.laser or self.point_cloud:
            robot.step(timestep)
        
    def get_distances(self):
        """Get current distance readings"""
        # Get raw laser data if available
        ranges = None
        if self.laser:
            ranges = self.laser.getRangeImage()
        
        # Get point cloud if available
        points = None
        if self.point_cloud:
            points = self.point_cloud.getPointCloud()
            
        return {
            'ranges': ranges,
            'point_cloud': points,
            'has_sensor': bool(self.laser or self.point_cloud)
        } 