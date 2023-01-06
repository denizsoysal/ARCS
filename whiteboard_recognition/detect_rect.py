import numpy as np
import cv2 as cv

img = cv.imread("322947063_854199839028322_5119371634723512530_n.jpg")

width=0 
height=0

start_x=0 
start_y=0
end_x=0 
end_y=0


output = img.copy()
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)


#threshold
th = cv.adaptiveThreshold(gray,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,9,2)

cv.imshow("th",th)



#rectangle detection

contours, _ = cv.findContours(th, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

for contour in contours:

    print(contour)
    approx = cv.approxPolyDP(contour, 0.01* cv.arcLength(contour, True), True)
    
    cv.drawContours(img, [approx], 0, (0, 0, 0), 5)
    
    x = approx.ravel()[0]
    y = approx.ravel()[1]

    x1 ,y1, w, h = cv.boundingRect(approx)
    a=w*h    
    if len(approx) == 4 and x>15  :
            
        aspectRatio = float(w)/h
        if  aspectRatio >= 2.5 and a>1500:          
          print(x1,y1,w,h)
          width=w
          height=h   
          start_x=x1
          start_y=y1
          end_x=start_x+width
          end_y=start_y+height      
          cv.rectangle(output, (start_x,start_y), (end_x,end_y), (0,0,255),3)
          cv.putText(output, "rectangle "+str(x1)+" , " +str(y1-5), (x1, y1-5), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
          
cv.imshow("op",output)
cv.waitKey()

print("start",start_x,start_y)
print("end", end_x,end_y)
print("width",width)
print("height",height)