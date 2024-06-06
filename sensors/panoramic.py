import cv2

path = '/home/billee/billee_ws/src/sensors/resource/'

img0 = cv2.imread(path + 'picture_0.png')
img1 = cv2.imread(path + 'picture_1.png')
img2 = cv2.imread(path + 'picture_2.png')
img3 = cv2.imread(path + 'picture_3.png')

images = [img0, img1, img2, img3]

stitcher = cv2.Stitcher.create(cv2.Stitcher_PANORAMA)

status, panorama = stitcher.stitch(images)

if status == cv2.Stitcher_OK:
    cv2.imwrite(path + 'panorama.png', panorama)
    print('OK')
else:
    print('BROKEN')