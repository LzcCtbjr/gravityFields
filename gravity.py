import PIL, cv2, random
import numpy as np
from fieldsClasses import *
from PIL import Image
import math 

#def generate():
    #param1 = random.random()
    #param2 = random.random()
    #param3 = random.random()
    #def f(t):
        #return param1*math.sin(t * param2) + param3
    #return f

x = 500
y = 500
screenSize = [x,y]
stepSize = 0.5
threshold = 200
buffer = 75

dots = list(())
for i in range(50):
    dots.append(Particle(x * random.random(), y * random.random(), random.random(), random.random()))
j = Dot(x//2, y//2, 0, 0)

while True:
    image = np.zeros((screenSize[1], screenSize[0], 3))

    for p in dots:
        p.step(stepSize)
        p.constrain(screenSize)
        cv2.circle(image, (p.getLocation()[0], p.getLocation()[1]), p.type * 5, (0,0,255))

    dotsLen = len(dots)
    for i in dots:
        d = i.getDistance(j)
        colorScale = (1 - d/150)
        iLoc = i.getLocation()
        jLoc = j.getLocation()

        if d < threshold:
            pass
        elif d > threshold and d < threshold + buffer:
            cv2.line(image, (iLoc[0], iLoc[1]), (jLoc[0], jLoc[1]), (colorScale,colorScale,colorScale))
            i.attract(j, threshold + buffer)

    cv2.imshow("fields", image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()