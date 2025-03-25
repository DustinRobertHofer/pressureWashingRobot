class RobotInterface:
    _instance = None
    _boundary_points = None

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def set_boundary_points(self, points):
        """Set the boundary points"""
        self._boundary_points = points
        print(f"Boundary points set: {self._boundary_points}")

    def get_boundary_points(self):
        """Get the current boundary points"""
        return self._boundary_points 