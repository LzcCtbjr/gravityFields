import cv2
import random
import numpy as np
from fieldsClasses import Particle


def main(framesLeft=-1, output=False):

    # setup variables
    random.seed(7)
    numPoints = 50
    x = 600
    y = 600
    boundary = 50
    screenSize = [x, y]
    stepSize = 0.01

    # physics variables
    # interesting parameters (with seed 7):
    # (50, 50, 1/100, 1/10, 1/5)
    # (100, 50, 1/100, 1/50, 1/5)
    threshold = 100             # the threshold within which points will repel
    buffer = 50                 # the buffer that is the size of the sweet spot
    attractIndex = 1/100        # how strongly points attract one another
    repelIndex = 1/25           # how strongly points repel each other
    constrainIndex = 1/5        # how strongly points are repelled from the boundaries

    # display variables
    dampeningCoeffA = 0.95      # what percent of the color from the previous iteration is carried over for "direction"
    dampeningCoeffB = 0.90      # what percent of the color from the previous iteration is carried over for "speed"
    speedCoeffA = 1/50          # how much of the velocity is used for "direction"
    speedCoeffB = 1/200         # how much of the velocity is used for "speed"

    # setup list of points
    dots = list(())
    for i in range(numPoints):

        # creates distribution of points that is random within the middle 50%
        xPos = (x/2) * random.random() + (x * 0.25)
        yPos = (y/2) * random.random() + (y * 0.25)
        """
        # creates random distribution of points
        xPos = x * random.random()
        yPos = y * random.random()
        """
        # gives random velocities
        xVel = random.random()
        yVel = random.random()

        # adds a new particle to our list
        dots.append(Particle(xPos, yPos, xVel, yVel))

    # initialize direction and speed
    direction = np.zeros((screenSize[1], screenSize[0], 3))
    speed = direction

    # if outputting to a video, set that up now
    if output:
        video = cv2.VideoWriter(
                            filename='./out/video.avi',
                            fourcc=cv2.VideoWriter_fourcc(*'DIVX'),
                            fps=20,
                            frameSize=((x + (2 * boundary)) * 3, y + (2 * boundary))
        )
        print(video)

    # loop!
    while framesLeft != 0:
        direction = direction * dampeningCoeffA
        points = np.zeros((screenSize[1], screenSize[0], 3))
        speed = speed * dampeningCoeffB
        prevdots = dots

        # constrains the points to within the window
        for p in dots:
            p.step(stepSize)
            p.constrain(screenSize, constrainIndex)

        # iterates through the points
        dotsLen = len(dots)
        for i in dots:
            iLoc = i.getLocation()

            for j in range(dots.index(i), dotsLen):
                # for j in dots:
                j = dots[j]
                d = i.getDistance(j)
                intensity = 0.75 * (1 - d/150)

                # gets and rounds the position of the particles
                iLoc = i.getLocation()
                iLoc = tuple([round(iLoc[0]), round(iLoc[1])])
                jLoc = j.getLocation()
                jLoc = tuple([round(jLoc[0]), round(jLoc[1])])

                # if the particles are closer than the threshold, they repel each other
                if d < threshold:
                    cv2.line(points, iLoc, jLoc, ((intensity, ) * 3), lineType=cv2.LINE_AA)
                    i.repel(j, threshold + buffer, repelIndex)
                # if the particles are in the sweet spot, they attract each other
                elif d > threshold and d < threshold + buffer:
                    cv2.line(points, iLoc, jLoc, ((intensity, ) * 3), lineType=cv2.LINE_AA)
                    i.attract(j, threshold + buffer, attractIndex)

            # renders "direction" for this iteration
            directionRender(direction, i, iLoc, speedCoeffA)

            oldPoint = prevdots[dots.index(i)]
            # renders "speed" for this iteration
            speedRender(speed, i, oldPoint, iLoc, speedCoeffB)

            # draws a circle at the point
            cv2.circle(points, iLoc, 2, (255, 255, 255))

        # will render raw images if uncommented
        """
        cv2.imshow("drawing", direction)
        cv2.imshow("points", points)
        cv2.imshow("speed", speed)
        """

        # pads immages
        bDirection = padImage(direction, boundary)
        bPoints = padImage(points, boundary)
        bSpeed = padImage(speed, boundary)

        # renders padded images
        cv2.imshow("direction", bDirection)
        cv2.imshow("points", bPoints)
        cv2.imshow("speed", bSpeed)

        if output:

            # code that works, albeit not quickly
            cv2.imwrite('./out/all.png', 255 * (np.concatenate((
                bDirection,
                bPoints,
                bSpeed),
                axis=1)))

            """
            # code for monitoring the output
            cv2.imshow('./out/all.png', (np.concatenate((
                bDirection,
                bPoints,
                bSpeed),
                axis=1)))
            """

            # writes this frame to the video
            video.write(cv2.imread('./out/all.png'))
            """

            # code that doesn't work for some reason, but is quick i guess
            video.write(np.concatenate((
                bDirection,
                bPoints,
                bSpeed),
                axis=1))

            """

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if framesLeft != -1:
            framesLeft -= 1

    cv2.destroyAllWindows()

    if output:
        video.release()
    print('Exited successfully')


# function to add padding to images
def padImage(image, padding=50):
    return cv2.copyMakeBorder(image,
                              padding,
                              padding,
                              padding,
                              padding,
                              cv2.BORDER_CONSTANT, None, 0)


# function to render to "direction"
def directionRender(direction, point, location, coefficient):
    dxIndex = point.xvel * coefficient
    dyIndex = point.yvel * coefficient
    cv2.circle(direction, (location[0], location[1]), 10, (1 - dxIndex, 1 - dyIndex, 0.1), thickness=-1)
    return


# function to render to "speed"
def speedRender(speed, point, oldPoint, location, coefficient):

    # we want the intensity of these indices to be inversely related to the velocity
    # the +0.01 in the denominator is so that we do not divide by zero
    dxIndex = 1/(abs(point.xvel * coefficient) + 0.01)
    dyIndex = 1/(abs(point.yvel * coefficient) + 0.01)

    prev_location = oldPoint.getLocation()
    prev_location = tuple([round(prev_location[0]), round(prev_location[1])])
    velIndex = int(abs(point.xvel + point.yvel) / 2) + 1
    cv2.line(speed, prev_location, location, (dxIndex, dyIndex, 255), velIndex, cv2.LINE_AA)

if __name__ == "__main__":
    main(-1, False)
