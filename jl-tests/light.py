import cv2


cap = cv2.VideoCapture(0)


lower_green = (36, 25, 25)
upper_green = (70, 255, 255)

while True:

    ret, frame = cap.read()


    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    mask = cv2.inRange(hsv, lower_green, upper_green)

    
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

   
    for c in contours:
        area = cv2.contourArea(c)
        x, y, w, h = cv2.boundingRect(c)

 
        if area > 500:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            
            cx = x + w // 2
            cy = y + h // 2
            print("Green object is at ({}, {})".format(cx, cy))

   
    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
