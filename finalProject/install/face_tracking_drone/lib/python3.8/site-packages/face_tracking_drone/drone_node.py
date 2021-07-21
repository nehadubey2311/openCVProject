from djitellopy import Tello
import cv2, math, time

def main():
    tello.connect()
    print(tello.get_battery())

    tello.streamon() 
    print(tello.get_height())

    while True:
        img = drone.get_frame_read().frame
        img = cv2.resize(img, (360, 240))
        cv2.imshow("Image", img)
        cv2.show()
        cv2.waitKey(1)


if __name__ == '__main__':
    main()