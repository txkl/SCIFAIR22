# Import libraries
import numpy
import cv2

# open the video
cap = cv2.VideoCapture(0)
detector = cv2.QRCodeDetector()
# Process until end.
while(True):
        _, img = cap.read()
        data, bbox, _ = detector.detectAndDecode(img)
        if bbox is not None:
                height_difference = (bbox[0][0][1]-bbox[0][1][1])
                if(height_difference < 0):
                        add = ((bbox[0][1][0]+bbox[0][0][0])/2-345)*0.1
                        print("add",add)
                        height_difference += add
                elif(height_difference < 0):
                        add = (345-(bbox[0][1][0]+bbox[0][0][0])/2)*0.1
                        print("add",add)
                        height_difference += add
                print(height_difference)
                        
                #print((bbox[0][2][0]-bbox[0][0][0])/(bbox[0][3][1]-bbox[0][0][1]))
        cv2.imshow('img', img)
        if cv2.waitKey(30) & 0xff == ord('q'):
                break		
		
# release the vid object
cap.release()

# close all the opened windows.
cv2.destroyAllWindows()
