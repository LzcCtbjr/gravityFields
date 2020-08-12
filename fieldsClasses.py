import math


class Dot:

    def __init__(self, x, y, dx, dy):
        self.x = x
        self.y = y

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

        # for some reason self.x and self.y are floats without this code
        self.x = round(self.x % screenSize[0])
        self.y = round(self.y % screenSize[1])

    def getLocation(self):
        return [self.x, self.y]

    def getDistance(self, p):
        delx = self.x - p.x
        dely = self.y - p.y
        d = math.sqrt((delx*delx) + (dely*dely))
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
        self.x = self.x + (self.xvel * stepSize)
        self.y = self.y + (self.yvel * stepSize)
        self.xvel = 9.9*self.xvel/10
        self.yvel = 9.9*self.yvel/10

    # def constrain(self, screenSize):

        # TODO: make this method repel the point from the boundary
        # of the simulation

    def repel(self, p, radius, strength=1/100):
        scaling = (radius - self.getDistance(p)) * strength
        delx = self.x - p.x
        dely = self.y - p.y

        self.xvel = self.xvel + (scaling * delx/115)
        self.yvel = self.yvel + (scaling * dely/115)

    def attract(self, p, radius, strength=1/100):
        scaling = (radius - self.getDistance(p)) * strength
        delx = self.x - p.x
        dely = self.y - p.y

        self.xvel = self.xvel - (scaling * delx/radius)
        self.yvel = self.yvel - (scaling * dely/radius)
