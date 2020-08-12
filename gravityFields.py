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
    y = 500
    screenSize = [x, y]
    stepSize = 0.04

    # physics variables
    # interesting parameters:
    # (50, 50, 0.01, 0.1)
    # (100, 50, 1/100, 1/50)
    threshold = 100
    buffer = 50
    attractIndex = 1/100
    repelIndex = 1/50

    # display variables
    dampeningCoeffA = 0.99
    dampeningCoeffB = 0.95
    speedCoeff = 1/20

    # setup list of points
    dots = list(())
    for i in range(numPoints):
        xPos = x * random.random()
        yPos = y * random.random()
        xVel = random.random()
        yVel = random.random()

        dots.append(Particle(xPos, yPos, xVel, yVel))

    # initialize image and inverse
    image = np.zeros((screenSize[1], screenSize[0], 3))
    inverse = image

    # if outputting to a video, set that up now
    # TODO: PUT IF CONDITION HERE, AND SET UP COMMAND-LINE
    #       ARGUMENTS FOR THIS OPTION
    # frameNum = 0
    # directory = r'C:\Users\dunna\Documents\Stuff\Python\fields\images'
    # os.chdir(directory)
    video = cv2.VideoWriter(
                        filename='video.avi',
                        fourcc=cv2.VideoWriter_fourcc(*'DIVX'),
                        fps=20,
                        frameSize=(x + 100, (y + 100) * 3)
                        )
    print(video)

    # loop!
    while framesLeft != 0:
        image = image * dampeningCoeffA
        points = np.zeros((screenSize[1], screenSize[0], 3))
        inverse = inverse * dampeningCoeffB

        for p in dots:
            p.step(stepSize)
            p.constrain(screenSize)

        # code attempting to implement mouse detection
        """
        pos = (0,0)
        cv2.setMouseCallback('test string', mousePosition)
        """

        # dotsLen = len(dots)
        for i in dots:
            iLoc = i.getLocation()

            # for j in range(dots.index(i), dotsLen):
            for j in dots:
                # j = dots[j]
                d = i.getDistance(j)
                colorScale = (1 - d/150)
                iLoc = i.getLocation()
                jLoc = j.getLocation()

                if d < threshold:
                    cv2.line(points, (iLoc[0], iLoc[1]), (jLoc[0], jLoc[1]), (colorScale, colorScale, colorScale))
                    i.repel(j, threshold + buffer, repelIndex)
                elif d > threshold and d < threshold + buffer:
                    cv2.line(points, (iLoc[0], iLoc[1]), (jLoc[0], jLoc[1]), (colorScale, colorScale, colorScale))
                    i.attract(j, threshold + buffer, attractIndex)

            dxIndex = i.xvel * speedCoeff
            dyIndex = i.yvel * speedCoeff

            cv2.circle(image, (iLoc[0], iLoc[1]), 10, (dxIndex, dyIndex, 0), thickness=-1)
            cv2.circle(points, (iLoc[0], iLoc[1]), 2, (255, 255, 255))
            cv2.circle(inverse, (iLoc[0], iLoc[1]), 3, (dyIndex, dxIndex, 255))

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
        cv2.imshow("image", cv2.copyMakeBorder(
            image, 50, 50, 50, 50, cv2.BORDER_CONSTANT, None, 0)
            )
        cv2.imshow("points", cv2.copyMakeBorder(
            points, 50, 50, 50, 50, cv2.BORDER_CONSTANT, None, 0)
            )
        cv2.imshow("inverse", cv2.copyMakeBorder(
            inverse, 50, 50, 50, 50, cv2.BORDER_CONSTANT, None, 0)
            )

        # writes images to disk as video
        # TODO: ADD IF CONDITION HERE
        # cv2.imwrite('image' + str(frameNum), image)
        # cv2.imwrite('points.jpg', 255 * points)
        # video.write(cv2.imread('points.jpg'))
        # cv2.imwrite('inverse' + str(frameNum), inverse)

        if output:
            cv2.imwrite('all.jpg', 255 * (np.concatenate((
                bImage,
                bPoints,
                bInverse),
                axis=0)))
            video.write(cv2.imread('all.jpg'))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if framesLeft != -1:
            framesLeft -= 1

    cv2.destroyAllWindows()
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
