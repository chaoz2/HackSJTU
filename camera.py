import numpy as np
import cv2

cap = cv2.VideoCapture(0)


def toSmallFrame(frame, nwidth):
    shape = frame.shape
    nheight = nwidth * shape[0] / shape[1]
    smallFrame = cv2.resize(frame, (nwidth, nheight))
    print ("new height: %d" % (nheight))
    return smallFrame, nheight


def preprocess(frame):
    nwidth = 244;
    smallFrame, nheight = toSmallFrame(frame, nwidth)
    arraySize = nheight * nwidth
    b, g, r = cv2.split(smallFrame)
    vector = np.reshape(b, (-1, 1))
    vector = np.append(vector, [np.reshape(g, (-1, 1)), np.reshape(r, (-1, 1))])
    print vector.shape
    return smallFrame


def mainLoop():
    # rgbDisplayLoop()
    while (True):
        ret, frame = cap.read()
        smallFrame = preprocess(frame)

        cv2.imshow("resized", smallFrame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


def rgbDisplayLoop():
    while (True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        # print ("R:%x G:%x B:%x" % (frame[0][0][0], frame[0][0][1], frame[0][0][2]))

        # Our operations on the frame come here
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Display the resulting frame
        # cv2.imshow('frame',gray)
        # cv2.imshow('frame', frame)

        # bframe, gframe, rframe = cv2.split(frame)
        r, g, b = splitRGB(frame)
        for values, color, channel in zip((r, g, b), ('red', 'green', 'blue'), (2, 1, 0)):
            img = np.zeros((values.shape[0], values.shape[1], 3),
                           dtype=values.dtype)
            img[:, :, channel] = values
            cv2.imshow(color, img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


def splitRGB(frame):
    # b, g, r = cv2.split(frame)
    return frame[:, :, 2], frame[:, :, 1], frame[:, :, 0]


if __name__ == '__main__':
    mainLoop()
