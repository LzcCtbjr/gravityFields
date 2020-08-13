import cv2
import random
import os
import numpy as np
from fieldsClasses import Particle


def mousePosition(event, x, y, flags, param):

    if event == cv2.EVENT_MOUSEMOVE:
        print(x, y)


def main(framesLeft=-1, output=False):
    # setup variables
    random.seed(7)
    numPoints = 100
    x = 500
    y = 980
    screenSize = [x, y]
    stepSize = 0.1

    # physics variables
    # interesting parameters:
    # (50, 50, 0.01, 0.1)
    # (100, 50, 1/100, 1/50)
    threshold = 100
    buffer = 50
    attractIndex = 1/100
    repelIndex = 1/50
    constrainIndex = 1/3

    # display variables
    dampeningCoeffA = 0.95
    dampeningCoeffB = 0.90
    speedCoeffA = 1/20
    speedCoeffB = 1/3

    # setup list of points
    dots = list(())
    for i in range(numPoints):

        xPos = (x/2) * random.random() + (x * 0.25)
        yPos = (y/2) * random.random() + (y * 0.25)
        """
        xPos = x * random.random()
        yPos = y * random.random()
        """
        xVel = random.random()
        yVel = random.random()

        dots.append(Particle(xPos, yPos, xVel, yVel))

    # initialize image and inverse
    image = np.zeros((screenSize[1], screenSize[0], 3))
    inverse = image

    # if outputting to a video, set that up now
    # TODO: SET UP COMMAND-LINE ARGUMENTS FOR THIS OPTION

    if output:
        video = cv2.VideoWriter(
                            filename='video.avi',
                            fourcc=cv2.VideoWriter_fourcc(*'DIVX'),
                            fps=30,
                            frameSize=((x + 100) * 3, y + 100)
        )
        print(video)

    # loop!
    while framesLeft != 0:
        image = image * dampeningCoeffA
        points = np.zeros((screenSize[1], screenSize[0], 3))
        inverse = inverse * dampeningCoeffB

        for p in dots:
            p.step(stepSize)
            p.constrain(screenSize, constrainIndex)

        # code attempting to implement mouse detection
        """
        pos = (0,0)
        cv2.setMouseCallback('test string', mousePosition)
        """

        dotsLen = len(dots)
        for i in dots:
            iLoc = i.getLocation()

            for j in range(dots.index(i), dotsLen):
                # for j in dots:
                j = dots[j]
                d = i.getDistance(j)
                intensity = 0.75 * (1 - d/150)

                iLoc = i.getLocation()
                iLoc = [round(iLoc[0]), round(iLoc[1])]
                jLoc = j.getLocation()
                jLoc = [round(jLoc[0]), round(jLoc[1])]

                if d < threshold:
                    cv2.line(points, (iLoc[0], iLoc[1]), (jLoc[0], jLoc[1]), ((intensity, ) * 3))
                    i.repel(j, threshold + buffer, repelIndex)

                elif d > threshold and d < threshold + buffer:
                    cv2.line(points, (iLoc[0], iLoc[1]), (jLoc[0], jLoc[1]), ((intensity, ) * 3))
                    i.attract(j, threshold + buffer, attractIndex)

            dxIndexA = i.xvel * speedCoeffA
            dyIndexA = i.yvel * speedCoeffA
            dxIndexB = i.xvel * speedCoeffB
            dyIndexB = i.yvel * speedCoeffB

            cv2.circle(image, (iLoc[0], iLoc[1]), 10, (dxIndexA, dyIndexA, 0), thickness=-1)
            cv2.circle(points, (iLoc[0], iLoc[1]), 2, (255, 255, 255))
            cv2.circle(inverse, (iLoc[0], iLoc[1]), 3, (dyIndexB, dxIndexB, 255))

        # will render raw images if uncommented
        """
        cv2.imshow("drawing", image)
        cv2.imshow("points", points)
        cv2.imshow("inverse", inverse)
        """

        # pads immages
        bImage = cv2.copyMakeBorder(image, 50, 50, 50, 50,cv2.BORDER_CONSTANT, None, 0)
        bPoints = cv2.copyMakeBorder(points, 50, 50, 50, 50, cv2.BORDER_CONSTANT, None, 0)
        bInverse = cv2.copyMakeBorder(inverse, 50, 50, 50, 50, cv2.BORDER_CONSTANT, None, 0)

        # renders padded images
        cv2.imshow("image", bImage)
        cv2.imshow("points", bPoints)
        cv2.imshow("inverse", bInverse)

        if output:

            # code that works, albeit not quickly
            cv2.imwrite('all.jpg', 63 * (np.concatenate((
                bImage,
                bPoints,
                bInverse),
                axis=1)))

            cv2.imshow('all.jpg', (np.concatenate((
                bImage,
                bPoints,
                bInverse),
                axis=1)))

            video.write(cv2.imread('all.jpg'))

            """

            # code that doesn't work for some reason, but is quick i guess
            video.write(np.concatenate((
                bImage,
                bPoints,
                bInverse),
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


def padImage(image, padding=50):
    return cv2.copyMakeBorder(image,
                              padding,
                              padding,
                              padding,
                              padding,
                              cv2.BORDER_CONSTANT, None, 0)


if __name__ == "__main__":
    main(-1, False)
