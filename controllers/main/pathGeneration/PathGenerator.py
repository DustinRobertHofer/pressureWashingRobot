import os
import cv2
import numpy as np

# Input Point Coordinates in units of ft. (P1 is the starting point therefore its coordinates are (0,0))
P1 = np.array([0,0])
P2 = np.array([20,10])
P3 = np.array([-10,30])
P4 = np.array([20,30])
WaterSupplyCoord = np.array([2,-2])

# Input the surface cleaner diameter and the path overlap
surfaceCleanerDiameter = 12 # inches
pathOverlap = 0 # inches

# Combine all point coordinates into a single matrix
Points = np.array([P1, P2, P3, P4, WaterSupplyCoord])
# Convert from ft. to inches 
PointsInches = Points * 12
#print(PointsInches)

# Find the minimum and maximum x and y coordinates
minInputX = np.min(PointsInches[:,0])
maxInputX = np.max(PointsInches[:,0])
minInputY = np.min(PointsInches[:,1])
maxInputY = np.max(PointsInches[:,1])
# Find the coordinates midpoint of the area identified by the points
midpointCoord = np.array([(maxInputX + minInputX) / 2, (maxInputY + minInputY) / 2])
#print(midpointCoord)

# Calculate the slopes of the lines that form the edges of the area identified by the points if the lines are not vertical or horizontal
if P1[1] != P2[1]:
    lineP12Eq = (P2[1] - P1[1])/(P2[0] - P1[0])
else:
    lineP12Eq = 0

if P3[1] != P4[1]:
    lineP34Eq = (P4[1] - P3[1])/(P4[0] - P3[0])
else:
    lineP34Eq = 0
if P1[0] != P3[0]:  
    lineP13Eq = (P3[1] - P1[1])/(P3[0] - P1[0])
else:
    lineP13Eq = 0
if P2[0] != P4[0]:
    lineP24Eq = (P4[1] - P2[1])/(P4[0] - P2[0])
else:
    lineP24Eq = 0   


# Calculate the distances of each point from the midpoint of the area identified by the points
pointDistances = PointsInches - midpointCoord
# Identify the maximum distance from the midpoint to normalize the distances so they are between 0 and 1
normalizingFactor = np.max(abs(pointDistances))
#print(pointDistances)

# Normalize the distances so they are between 0 and 1
normalizedDistances = pointDistances / normalizingFactor
#print(normalizedDistances)

# Transform the normalized distances so they are between 0 and 512 pixels. A value of 250 is used to leaving a margin around the edges of the image.
transformedPDistances = np.int32(normalizedDistances * 250)
# Add the midpoint coordinates to the transformed distances to get the final coordinates in pixels
transformedPoints = transformedPDistances + np.array([256, 256])
#print(transformedPDistances)
#print(transformedPoints)

# Create an blank 512 x 512 image
image = np.zeros((512, 512, 3), np.uint8)

# Draw the lines that form the edges of the area identified by the points
cv2.line(image, (transformedPoints[0,0], transformedPoints[0,1]), (transformedPoints[1,0], transformedPoints[1,1]), (0, 255, 0), 2)
cv2.line(image, (transformedPoints[1,0], transformedPoints[1,1]), (transformedPoints[3,0], transformedPoints[3,1]), (0, 255, 0), 2)
cv2.line(image, (transformedPoints[3,0], transformedPoints[3,1]), (transformedPoints[2,0], transformedPoints[2,1]), (0, 255, 0), 2)
cv2.line(image, (transformedPoints[2,0], transformedPoints[2,1]), (transformedPoints[0,0], transformedPoints[0,1]), (0, 255, 0), 2)
cv2.circle(image, (transformedPoints[4,0], transformedPoints[4,1]), 5, (255, 0, 0), -1)

# Determine which direction to start based on the distance to the water supply
P2Distance = np.linalg.norm(P2 - WaterSupplyCoord)
P3Distance = np.linalg.norm(P3 - WaterSupplyCoord)
if P2Distance > P3Distance:
    PassEnd = 3
    PathEndPoint = PointsInches[2]
    if PointsInches[1,0] > PointsInches[3,0]:
        FinishPoint = PointsInches[1,0]
    else:
        FinishPoint = PointsInches[3,0]
    print(PassEnd)
    NumberofPasses = FinishPoint / surfaceCleanerDiameter
    print(NumberofPasses)
else:
    PassEnd = 2
    PathEndPoint = np.array([(PointsInches[1,0] - (surfaceCleanerDiameter/2)), PointsInches[1,1]])
    if PointsInches[2,1] > PointsInches[3,1]:
        FinishPoint = PointsInches[2,1]
    else:
        FinishPoint = PointsInches[3,1]
    print(PassEnd)
    NumberofPasses = np.int32(FinishPoint / surfaceCleanerDiameter)
    print(NumberofPasses)

PathStartPoint = PointsInches[0]


surfaceCleanerDiameterPixels = np.int32(surfaceCleanerDiameter / normalizingFactor * 250)

for i in range(50):
    #print(i)
    if PassEnd == 2:
        nextYPoint1 = PathStartPoint[1] + (surfaceCleanerDiameter/2) - pathOverlap
        nextYPoint2 = PathEndPoint[1] + (surfaceCleanerDiameter/2) - pathOverlap
        if lineP13Eq != 0:
            nextXPoint1 = (nextYPoint1 / lineP13Eq) - (surfaceCleanerDiameter/2)
        else:
            nextXPoint1 = PathStartPoint[0]

        if lineP24Eq != 0:
            nextXPoint2 = (nextYPoint2 / lineP24Eq) - (surfaceCleanerDiameter/2)
        else:
            nextXPoint2 = PathEndPoint[0]
        nextPoint1 = np.array([nextXPoint1, nextYPoint1])
        nextPoint2 = np.array([nextXPoint2, nextYPoint2])
        nextPoint1Pixels = np.int32(((nextPoint1 - midpointCoord) / normalizingFactor) * 250) + np.array([256, 256])
        nextPoint2Pixels = np.int32(((nextPoint2 - midpointCoord) / normalizingFactor) * 250) + np.array([256, 256])
        cv2.line(image, (nextPoint1Pixels[0], nextPoint1Pixels[1]), (nextPoint2Pixels[0], nextPoint2Pixels[1]), (0, 0, 255), surfaceCleanerDiameterPixels)
        PathStartPoint = nextPoint2
        PathEndPoint = nextPoint1
        PassEnd = 1
        print(PathStartPoint)
        print(PathEndPoint)
        print(PassEnd)

    elif PassEnd == 1:
        nextYPoint1 = PathStartPoint[1] + (surfaceCleanerDiameter/2) - pathOverlap
        nextYPoint2 = PathEndPoint[1] + (surfaceCleanerDiameter/2) - pathOverlap
        if lineP13Eq != 0:
            nextXPoint2 = (nextYPoint2 / lineP13Eq) - (surfaceCleanerDiameter/2)
        else:
            nextXPoint2 = PathEndPoint[0] - (surfaceCleanerDiameter/2)

        if lineP24Eq != 0:
            nextXPoint1 = (nextYPoint1 / lineP24Eq) - (surfaceCleanerDiameter/2)
        else:
            nextXPoint1 = PathStartPoint[0]
        nextPoint1 = np.array([nextXPoint1, nextYPoint1])
        nextPoint2 = np.array([nextXPoint2, nextYPoint2])
        nextPoint1Pixels = np.int32(((nextPoint1 - midpointCoord) / normalizingFactor) * 250) + np.array([256, 256])
        nextPoint2Pixels = np.int32(((nextPoint2 - midpointCoord) / normalizingFactor) * 250) + np.array([256, 256])
        cv2.line(image, (nextPoint1Pixels[0], nextPoint1Pixels[1]), (nextPoint2Pixels[0], nextPoint2Pixels[1]), (0, 0, 255), surfaceCleanerDiameterPixels)
        PathStartPoint = nextPoint2
        PathEndPoint = nextPoint1
        PassEnd = 2
        print(PathStartPoint)
        print(PathEndPoint)
        print(PassEnd)
    else:
        nextXPoint1 = PathStartPoint[0] 
    
    i = i + 1
    


# Flip the image vertically to orient the image correctly
flippedImage = cv2.flip(image, 0)

# Display the image
cv2.imshow("image", flippedImage)

# Wait for a key press to close the image window
cv2.waitKey(0)
cv2.destroyAllWindows()
 
