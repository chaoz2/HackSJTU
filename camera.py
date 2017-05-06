import numpy as np
import cv2
import rospy
from std_msgs.msg import String
import caffe
from caffe.proto import caffe_pb2


GPU_ID = 0
caffe.set_mode_gpu()
caffe.set_device(GPU_ID)

cap = cv2.VideoCapture(0)
net = caffe.Net('/home/ubuntu/Documents/HackSJTU/nv-ssd-detection-model/model/deploy.prototxt',
                    '/home/ubuntu/Documents/HackSJTU/nv-ssd-detection-model/model/KC_NET_V1_VOC_224x224.caffemodel',
                    caffe.TEST)

global pub


def toSmallFrame(frame, nwidth):
    shape = frame.shape
    # nheight = nwidth * shape[0] / shape[1]
    nheight = 224
    smallFrame = cv2.resize(frame, (nwidth, nheight))
    return smallFrame, nheight


def preprocess(frame):
    nwidth = 224
    smallFrame, nheight = toSmallFrame(frame, nwidth)
    # arraySize = nheight * nwidth
    b, g, r = cv2.split(smallFrame)
    # vector = np.reshape(b, (-1, 1))
    # vector = np.append(vector, [np.reshape(g, (-1, 1)), np.reshape(r, (-1, 1))])
    # print vector.shape
    data = np.array(np.array([b, g, r]))
    return data

def setup():
    global pub
    pub = rospy.Publisher('chatter', String, queue_size = 10)
    rospy.init_node('talker', anonymous = True)

    # TODO: if need to set rate
    # rate = rospy.Rate(10)

def labelToObj(label):
    if (label == 2):
        return 'bike'
    elif (label == 3):
        return 'bird'
    elif (label == 4):
        return 'ship'
    elif (label == 6):
        return 'bus'
    elif (label == 7):
        return 'car'
    elif (label == 8):
        return 'cat'
    elif (label == 9):
        return 'chair'
    elif (label == 10):
        return 'bull'
    elif (label == 12):
        return 'dog'
    elif (label == 15):
        return 'horse'
    elif (label == 17):
        return 'sheep'
    elif (label == 18):
        return 'sofa'
    elif (label == 19):
        return 'train'
    else
        return 'other'

def mainLoop():
    # rgbDisplayLoop()
    while (True):
        ret, frame = cap.read()
        data = preprocess(frame)
        net.blobs['data'].data[...] = data
        out = net.forward()['detection_out'][0][0]
        # print out.shape
        # print ("label:%f conf:%f"%(out[0][1], out[0][2]))

        out = out[out[:, 2] > 0.5]
        print out.shape


        if len(out) != 0:
            max_conf = 0.0
            label = 0.0
            for res in out:
                # print ("label:%f conf:%f" % (out[0][1], out[0][2]))
                if (res[2] > max_conf):
                    label = int(res[1])
            thing = labelToObj(label)
            pub.publish(thing)


        cv2.imshow("camera", frame)

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
