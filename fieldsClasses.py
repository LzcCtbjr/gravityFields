import math


# abstract class
class Dot:

    def __init__(self, x, y, xvel, yvel):
        self.x = x
        self.y = y

    # what to do when a time interval passes
    def step(self, stepSize):
        pass

    def constrain(self, screenSize):
        # screenSize is of format [y, x]

        """
        if self.x < 0:
            self.x = round(0)
        elif self.x > screenSize[0]:
            self.x = round(screenSize[0] - 1)
        if self.y < 0:
            self.y = round(0)
        elif self.y > screenSize[1]:
            self.y = round(screenSize[1] - 1)
        """

        # when outside the bounds, the point "rolls over" back into bounds
        self.x = self.x % screenSize[0]
        self.y = self.y % screenSize[1]

    # returns the location
    def getLocation(self):
        return [self.x, self.y]

    # gets the distance between the points
    def getDistance(self, p):
        delx = self.x - p.x
        dely = self.y - p.y
        d = math.sqrt((delx*delx) + (dely*dely))    # this square root is very time expensive, so it would be nice to find something to replace it with
        return d

    def repel(self, p, t):
        pass

    def attract(self, p, t):
        pass


class Particle(Dot):

    def __init__(self, x0, y0, xvel, yvel):
        self.x = x0
        self.y = y0
        self.xvel = xvel
        self.yvel = yvel
        self.type = 0

    def step(self, stepSize=1):
        # updates the position based on velocity and time interval
        self.x = self.x + (self.xvel * stepSize)
        self.y = self.y + (self.yvel * stepSize)
        # decays the velocity
        self.xvel = 9.9 * self.xvel/10
        self.yvel = 9.9 * self.yvel/10

    def constrain(self, screenSize, strength=1/100, radius=None):
        # this breaks if I put this as a default parameter, and I dont know why :(
        if radius is None:
            radius = float(max(screenSize)) * 0.05
        
        # this implementation of constrain repels the points from the boundaries
        # x dimension constraining
        if self.x < radius or self.x > screenSize[0] - radius:
            if self.x < radius:
                distance = self.x
                scaling = (radius - distance) * strength
                self.xvel = self.xvel + (scaling * distance)
            else:
                distance = screenSize[0] - self.x
                scaling = (radius - distance) * strength
                self.xvel = self.xvel - (scaling * distance)

        if self.y < radius or self.y > screenSize[1] - radius:
            if self.y < radius:
                distance = self.y
                scaling = (radius - distance) * strength
                self.yvel = self.yvel + (scaling * distance)
            else:
                distance = screenSize[1] - self.y
                scaling = (radius - distance) * strength
                self.yvel = self.yvel - (scaling * distance)

        # just in case a particle is moving too fast to slow down before getting within the radius
        self.x = self.x % screenSize[0]
        self.y = self.y % screenSize[1]

    def repel(self, p, radius, strength=1/100):
        scaling = (radius - self.getDistance(p)) * strength
        delx = self.x - p.x
        dely = self.y - p.y

        # changes velocity of the self particle
        self.xvel = self.xvel + (scaling * delx/radius)
        self.yvel = self.yvel + (scaling * dely/radius)

        # adds that "equal but opposite" force that newton was about
        p.xvel = p.xvel - (scaling * delx/radius)
        p.yvel = p.yvel - (scaling * dely/radius)

    def attract(self, p, radius, strength=1/100):
        scaling = (radius - self.getDistance(p)) * strength
        delx = self.x - p.x
        dely = self.y - p.y

        # changes velocity of the self particle
        self.xvel = self.xvel - (scaling * delx/radius)
        self.yvel = self.yvel - (scaling * dely/radius)

        # adds the "equal but opposite" force that newton liked
        p.xvel = p.xvel + (scaling * delx/radius)
        p.yvel = p.yvel + (scaling * dely/radius)
