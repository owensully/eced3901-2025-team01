import cv2

dst_dir = "/home/student/ros2_ws/src/eced3901/test_scripts/pictures/"
 
cam = cv2.VideoCapture(0)

cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

cv2.waitKey(1000)

ret, pic = cam.read()

if ret:
    dst = dst_dir + "pic.jpg"
    cv2.imwrite(dst, pic)
    print("success")
else:
    print("fail")

cam.release()

qr_pic = cv2.imread(dst)

qr_detector = cv2.QRCodeDetector()

qr_data, bbox, _ = qr_detector.detectAndDecode(qr_pic)

if qr_data:
    print(qr_data)
else:
    print("fck off")
