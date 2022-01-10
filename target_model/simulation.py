import math
import pymap3d as pm

class TargetModelCircle:
    def __init__(self, lat0, lon0, h0):
        self.vmax = 5 # m/s
        self.vx = self.vmax
        self.vy = 0.0
        self.vz = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.lat0 = lat0
        self.lon0 = lon0
        self.lat = lat0
        self.lon = lon0
        self.h0 = h0
        self.current_time = 0.0
        self.t0 = 0.0
        pass

    def update(self,current_time):
        if self.t0 == 0.0:
            self.t0 = current_time
            self.current_time = current_time
            return

        dt = current_time - self.current_time
        self.current_time = current_time

        # Update veolocity
        t = current_time - self.t0
        
        circle_time = 20.0 # time for target to go full circle
        self.vx = math.cos((t)/circle_time*2*math.pi)*self.vmax
        self.vy = math.sin((t)/circle_time*2*math.pi)*self.vmax

        # Integrate velocity
        self.x += self.vx*dt
        self.y += self.vy*dt
        self.z += self.vz*dt

        # Calculate lat/lon
        self.lat, self.lon, self.alt = pm.ned2geodetic(self.x, self.y, self.z, self.lat0, self.lon0, self.h0)

class TargetModelStraightLine:
    def __init__(self, lat0, lon0, h0):
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.lat0 = lat0
        self.lon0 = lon0
        self.h0 = h0
        self.current_time = 0.0
        pass

    def update(self,current_time):
        dt = current_time - self.current_time
        self.current_time = current_time

        # First accelerate forwards, and then reverse after a while
        if current_time >= 13 and current_time < 18:
            self.ax = 1.0
        elif current_time >= 18 and current_time < 20:
            self.ax = 0
        elif current_time >= 20 and current_time < 25:
            self.ax = -1.0
        else:
            self.ax = 0.0
            self.vx = 0.0

        # Integrate acceleration
        self.vx += self.ax*dt
        self.vy += self.ay*dt
        self.vz += self.az*dt

        # Integrate velocity
        self.x += self.vx*dt
        self.y += self.vy*dt
        self.z += self.vz*dt

        # Calculate lat/lon
        self.lat, self.lon, self.alt = pm.ned2geodetic(self.x, self.y, self.z, self.lat0, self.lon0, self.h0)
