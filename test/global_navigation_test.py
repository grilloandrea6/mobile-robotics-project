from global_navigation import *

frame = cv2.imread("images/shapes.png")

thresh = prepareImage(frame)

cv2.namedWindow("Pic with contours", cv2.WINDOW_NORMAL)
cv2.imshow('Pic with contours',frame)

while True:
    # Display the resulting frame
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

scalingFactor = 100

contours,hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
frame, contourList = drawSimplifiedContours(contours, frame, scalingFactor)

y, x = frame.shape[0:2]
x_s, y_s = int(x*0.1), int(y*0.1)

goal = np.array([x-x_s,y-y_s])
start = np.array([x_s,y_s])

contourList = addStartAndGoal(contourList, start, goal, 1)

frame = cv2.circle(frame, (x_s, y_s), radius = 0, color = (0,0,255), thickness = 10)
frame = cv2.circle(frame, (x-x_s,y-y_s), radius = 0, color = (0,0,255), thickness = 10)

frame = cv2.putText(frame, 'Start', (x_s+10, y_s-10), cv2.FONT_HERSHEY_SIMPLEX,  
                fontScale = 1.5, color = (0,0,0), thickness = 2, lineType = cv2.LINE_AA)
frame = cv2.putText(frame, 'Goal', (x-x_s+10,y-y_s-10), cv2.FONT_HERSHEY_SIMPLEX,  
                fontScale = 1.5, color = (0,0,0), thickness = 2, lineType = cv2.LINE_AA) 

# contourList is a list of element representing the vertices of an obstacle (np.array of shape (n,2)).
# To acces points, write contourList[obstacle][vertices, coordinate].
    
cv2.namedWindow("Pic with contours", cv2.WINDOW_NORMAL)
cv2.imshow('Pic with contours',frame)

while True:
    # Display the resulting frame
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
# Finding shortest path
path = findShortestPath(contourList= contourList)

print(contourList)

print(path)

for i in range(len(path)-1):
    frame = cv2.line(frame, path[i, :], path[i+1, :], color=(0, 0, 255), thickness=2)
    
cv2.namedWindow("Pic with contours", cv2.WINDOW_NORMAL)
cv2.imshow("Pic with contours",frame)
       
while True:
    # Display the resulting frame
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break