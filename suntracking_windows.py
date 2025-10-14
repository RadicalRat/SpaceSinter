import cv2
import numpy as np


class SunTrackingNode():
    def __init__(self):
        print("setting up node")
        self.exposure = -12
        self.set_up_detector()
        self.cam = cv2.VideoCapture(0)


        self.cam.set(cv2.CAP_PROP_EXPOSURE, self.exposure)

        ret, frame = self.cam.read()

        self.width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.halfwidth = self.width // 2
        self.halfheight = self.height // 2

        cv2.imshow("frame",frame)


    def set_up_detector(self):
        # SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        params.filterByColor = False

        # Change thresholds
        params.minThreshold = 200
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 50

        # Filter by Circularity
        params.filterByCircularity = True
#         params.minCircularity = 0.75
        params.minCircularity = 0.5

        # Filter by Convexity
        params.filterByConvexity = True
#         params.minConvexity = 0.87
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Create a detector with the parameters
        self.detector = cv2.SimpleBlobDetector_create(params)

    def get_blob(self):
        # self.cap = cv2.VideoCapture(0)
        ret, frame = self.cam.read()
        ret, frame = self.cam.read()
        # self.cap.release()

        # Read image

        # Detect blobs.
        keypoints = self.detector.detect(frame)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
        # the size of the circle corresponds to the size of blob

        im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show blobs

        if len(keypoints) > 0:
            x,y = keypoints[0].pt
            sun = (int(round(x)), int(round(y)))
            im = cv2.line(im_with_keypoints, (self.halfwidth,self.halfheight), sun, (0,0,255), 2, lineType=cv2.LINE_8)
            im = cv2.line(im, (self.halfwidth,self.halfheight), (sun[0],self.halfheight), (255,0,0), 2, lineType=cv2.LINE_8) #draw x
            im = cv2.line(im, (sun[0],self.halfheight), (sun[0],sun[1]), (0,255,0), 2, lineType=cv2.LINE_8) #draw y
            im = cv2.putText(im, f"{sun[0] - self.halfwidth}", (sun[0] - 10, self.halfheight -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0))
            im = cv2.putText(im, f"{self.halfheight - sun[1]}", (sun[0] - 20 , self.halfheight - sun[1] ), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
            cv2.imshow("frame",im)
            print(keypoints[0].pt)
        else:
            print("no keypoints found")
            return None

        return keypoints[0].pt
    
    

def main():
    print("starting!")
    sunnode = SunTrackingNode()
    try:
        while True:
            sunnode.get_blob()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("Shutting down")
        pass


if __name__ == "__main__":
    main()