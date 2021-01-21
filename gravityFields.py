import cv2
import random
# import noise
import numpy as np
from fieldsClasses import Particle


def mousePosition(event, x, y, flags, param):

    if event == cv2.EVENT_MOUSEMOVE:
        print(x, y)


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
    threshold = 100
    buffer = 50
    attractIndex = 1/100
    repelIndex = 1/25
    constrainIndex = 1/5

    # display variables
    dampeningCoeffA = 0.95
    dampeningCoeffB = 0.90
    speedCoeffA = 1/50
    speedCoeffB = 1/200

    # setup list of points
    dots = list(())
    for i in range(numPoints):

        # distribution of points that is random but only in the middle 50%
        xPos = (x/2) * random.random() + (x * 0.25)
        yPos = (y/2) * random.random() + (y * 0.25)
        """
        # random distribution of points
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
        image = image * dampeningCoeffA
        points = np.zeros((screenSize[1], screenSize[0], 3))
        inverse = inverse * dampeningCoeffB
        prevdots = dots

        for p in dots:
            p.step(stepSize)
            p.constrain(screenSize, constrainIndex)

        dotsLen = len(dots)
        for i in dots:
            iLoc = i.getLocation()

            for j in range(dots.index(i), dotsLen):
                # for j in dots:
                j = dots[j]
                d = i.getDistance(j)
                intensity = 0.75 * (1 - d/150)

                iLoc = i.getLocation()
                iLoc = tuple([round(iLoc[0]), round(iLoc[1])])
                jLoc = j.getLocation()
                jLoc = tuple([round(jLoc[0]), round(jLoc[1])])

                if d < threshold:
                    cv2.line(points, iLoc, jLoc, ((intensity, ) * 3), lineType=cv2.LINE_AA)
                    i.repel(j, threshold + buffer, repelIndex)

                elif d > threshold and d < threshold + buffer:
                    cv2.line(points, iLoc, jLoc, ((intensity, ) * 3), lineType=cv2.LINE_AA)
                    i.attract(j, threshold + buffer, attractIndex)

            dxIndexA = i.xvel * speedCoeffA
            dyIndexA = i.yvel * speedCoeffA
            dxIndexB = 1/(abs(i.xvel * speedCoeffB) + 0.01) + 1
            dyIndexB = 1/(abs(i.yvel * speedCoeffB) + 0.01) + 1

            cv2.circle(image, (iLoc[0], iLoc[1]), 10, (1 - dxIndexA, 1 - dyIndexA, 0.1), thickness=-1)

            oldPoint = prevdots[dots.index(i)]
            prevLoc = oldPoint.getLocation()
            prevLoc = tuple([round(prevLoc[0]), round(prevLoc[1])])

            speedIndex = int(abs(i.xvel + i.yvel) / 2) + 1

            cv2.line(inverse, prevLoc, iLoc, (dxIndexB, dyIndexB, 255), speedIndex, cv2.LINE_AA)
            cv2.circle(points, iLoc, 2, (255, 255, 255))

        # will render raw images if uncommented
        """
        cv2.imshow("drawing", image)
        cv2.imshow("points", points)
        cv2.imshow("inverse", inverse)
        """

        # pads immages
        bImage = padImage(image, boundary)
        bPoints = padImage(points, boundary)
        bInverse = padImage(inverse, boundary)

        # renders padded images
        cv2.imshow("image", bImage)
        cv2.imshow("points", bPoints)
        cv2.imshow("inverse", bInverse)

        if output:

            # code that works, albeit not quickly
            cv2.imwrite('./out/all.png', 255 * (np.concatenate((
                bImage,
                bPoints,
                bInverse),
                axis=1)))

            """
            # code for monitoring the output
            cv2.imshow('./out/all.png', (np.concatenate((
                bImage,
                bPoints,
                bInverse),
                axis=1)))
            """

            video.write(cv2.imread('./out/all.png'))
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
    main(-1, True)
