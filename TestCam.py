
import cv2


cap = cv2.VideoCapture(0)
cap.release
ret, frame = cap.read();

while (True):
    ret, frame = cap.read();

    cv2.imshow("theFrame",frame)
    
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
    
cap.release
cv2.destroyAllWindows()