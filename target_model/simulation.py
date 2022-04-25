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
        if current_time >= 12 and current_time < 22: # Usually after takeoff, arming, time is already at 13 sec.
            self.ax = 1.0
        elif current_time >= 22 and current_time < 24:
            self.ax = 0.0
            self.vx = 0.0
        elif current_time >= 24 and current_time < 34:
            self.ax = -0.8
        else:
            self.ax = 0.0
            self.vx = 0.0

        print('X, Vx, Ax :', self.x, self.vx, self.ax)

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

class TargetModelRectangle:
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

        if current_time < 5:
            self.vx = 2.0
            self.vy = 0.0
        elif current_time < 10:
            self.vx = 0.0
            self.vy = 2.0
        elif current_time < 15:
            self.vx = -2.0
            self.vy = 0.0
        else:
            self.vx = 0.0
            self.vy = -2.0

        # Integrate velocity
        self.x += self.vx*dt
        self.y += self.vy*dt
        self.z += self.vz*dt

        # Calculate lat/lon
        self.lat, self.lon, self.alt = pm.ned2geodetic(self.x, self.y, self.z, self.lat0, self.lon0, self.h0)

class TargetModelPoint:
    def __init__(self, lat0, lon0, h0):
        self.vmax = 5 # m/s
        self.vx = 0.0
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

        # Calculate lat/lon
        self.lat, self.lon, self.alt = pm.ned2geodetic(self.x, self.y, self.z, self.lat0, self.lon0, self.h0)

class TargetModelUnitSpeed:
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

        if current_time < 5:
            self.vx = 1.0
            self.vy = 0.0
        elif current_time < 10:
            self.vx = 0.0
            self.vy = 1.0
        elif current_time < 15:
            self.vx = -1.0
            self.vy = 0.0
        else:
            self.vx = 0.0
            self.vy = -1.0

        # Integrate velocity
        self.x += self.vx*dt
        self.y += self.vy*dt
        self.z += self.vz*dt

        # Calculate lat/lon
        self.lat, self.lon, self.alt = pm.ned2geodetic(self.x, self.y, self.z, self.lat0, self.lon0, self.h0)

class TargetModelGoAndStop:
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

        if current_time < 3:
            # Go X direction at 5m/s
            self.vx = 5.0
            self.vy = 0.0
        elif current_time < 5:
            # Stop (t = 3 ~ 5)
            self.vx = 0.0
            self.vy = 0.0
        elif current_time < 8:
            # Go Y direction at 3 m/s
            self.vx = 0.0
            self.vy = 3.0
        elif current_time < 10:
            # Stop (t = 8 ~ 10)
            self.vx = 0.0
            self.vy = 0.0
        elif current_time < 13:
            # Go in XY direction at (4,3) speed
            self.vx = 4.0
            self.vy = 3.0
        elif current_time < 15:
            # Stop (t = 13 ~ 15)
            self.vx = 0.0
            self.vy = 0.0

        # Integrate velocity
        self.x += self.vx*dt
        self.y += self.vy*dt
        self.z += self.vz*dt

        # Calculate lat/lon
        self.lat, self.lon, self.alt = pm.ned2geodetic(self.x, self.y, self.z, self.lat0, self.lon0, self.h0)