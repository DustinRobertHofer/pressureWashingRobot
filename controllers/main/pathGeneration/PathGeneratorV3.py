import os
import cv2
import numpy as np
import math

# Input Point Coordinates in units of ft. (P1 is the starting point therefore its coordinates are (0,0))
P1 = np.array([0,0])
P2 = np.array([20,10])
P3 = np.array([-10,30])
P4 = np.array([20,30])
WaterSupplyCoord = np.array([2,-2])

# Input the surface cleaner diameter and the path overlap
surfaceCleanerDiameter = 12 # inches
pathOverlap = 2 # inches
edgeBufferRadius = 8 # inches

# Maximum number of passes (to prevent infinite loops)
MaxNumberofPasses = 250

# Image Parameters
imageWidth = 1024
imageHeight = 1024
DisplayActualPath = True # By default the actual path is displayed

# Combine all point coordinates into a single matrix
Points = np.array([P1, P2, P3, P4, WaterSupplyCoord])
# Convert from ft. to inches 
PointsInches = Points * 12


# Define the function to calculate the intersection of two lines
def lineIntersection(line1, line2):

    # Define the lines coordinates
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2
    
    # Calculate the determinant of the lines
    det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    # If the determinant is 0, the lines are parallel and do not intersect
    if det == 0:
        return None
    
    # Calculate the intersection point
    x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / det
    y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / det

    return np.array([x, y])

def offsetLines(x1, y1, x2, y2, offset):
    # Calculate direction vector of original line
    dx = x2 - x1
    dy = y2 - y1
    
    # Normalize the direction vector
    length = np.sqrt(dx**2 + dy**2)
    dx /= length
    dy /= length

    # Calculate the perpendicular vector
    perpendicularX = -dy
    perpendicularY = dx

    # Calculate the coordinates of the parallel line
    parallelX1 = x1 + offset * perpendicularX
    parallelY1 = y1 + offset * perpendicularY
    parallelX2 = x2 + offset * perpendicularX
    parallelY2 = y2 + offset * perpendicularY

    return parallelX1, parallelY1, parallelX2, parallelY2


# Find the minimum and maximum x and y coordinates
minInputX = np.min(PointsInches[:,0])
maxInputX = np.max(PointsInches[:,0])
minInputY = np.min(PointsInches[:,1])
maxInputY = np.max(PointsInches[:,1])
# Find the coordinates midpoint of the area identified by the points
midpointCoord = np.array([(maxInputX + minInputX) / 2, (maxInputY + minInputY) / 2])
#print(midpointCoord)

# define the lines that form the edges of the area identified by the points
lineP12 = PointsInches[0,0], PointsInches[0,1], PointsInches[1,0], PointsInches[1,1]    
lineP34 = PointsInches[2,0], PointsInches[2,1], PointsInches[3,0], PointsInches[3,1]
lineP24 = PointsInches[1,0], PointsInches[1,1], PointsInches[3,0], PointsInches[3,1]
lineP13 = PointsInches[0,0], PointsInches[0,1], PointsInches[2,0], PointsInches[2,1]


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

# Offset the lines to form a buffer around the area identified by the points
lineP12Offset = np.array(offsetLines(PointsInches[0,0], PointsInches[0,1], PointsInches[1,0], PointsInches[1,1], edgeBufferRadius))
lineP34Offset = np.array(offsetLines(PointsInches[2,0], PointsInches[2,1], PointsInches[3,0], PointsInches[3,1], -edgeBufferRadius))
lineP24Offset = np.array(offsetLines(PointsInches[1,0], PointsInches[1,1], PointsInches[3,0], PointsInches[3,1], edgeBufferRadius))
lineP13Offset = np.array(offsetLines(PointsInches[0,0], PointsInches[0,1], PointsInches[2,0], PointsInches[2,1], -edgeBufferRadius))

# Find offset line intersections and overwrite offset points 
intersection = lineIntersection(lineP12Offset, lineP13Offset)
if intersection is not None:
    Point1Offset = intersection
intersection = lineIntersection(lineP12Offset, lineP24Offset)
if intersection is not None:
    Point2Offset = intersection
intersection = lineIntersection(lineP13Offset, lineP34Offset)
if intersection is not None:
    Point3Offset = intersection
intersection = lineIntersection(lineP24Offset, lineP34Offset)
if intersection is not None:
    Point4Offset = intersection

# Combine offset points into a single matrix
OffsetPoints = np.array([Point1Offset, Point2Offset, Point3Offset, Point4Offset])

# Calculate the distances of each point from the midpoint of the area identified by the points
pointDistances = PointsInches - midpointCoord
offsetPointDistances = OffsetPoints - midpointCoord

# Identify the maximum distance from the midpoint to normalize the distances so they are between 0 and 1
normalizingFactor = np.max(abs(pointDistances))


# Normalize the distances so they are between 0 and 1
normalizedDistances = pointDistances / normalizingFactor
normalizedOffsetDistances = offsetPointDistances / normalizingFactor


# Transform the normalized distances so they are between 0 and 512 pixels. A value of 200 is used to leave a margin around the edges of the image.
PixelMultiplier = 200
transformedPDistances = np.int32(normalizedDistances * PixelMultiplier)
transformedOffsetDistances = np.int32(normalizedOffsetDistances * PixelMultiplier)

# Add the midpoint coordinates to the transformed distances to get the final coordinates in pixels
transformedPoints = transformedPDistances + np.array([256, 256])
transformedOffsetPoints = transformedOffsetDistances + np.array([256, 256])

# Create an blank 512 x 512 image
image = np.zeros((512, 512, 3), np.uint8)

# Draw the lines that form the edges of the area identified by the points
cv2.line(image, (transformedPoints[0,0], transformedPoints[0,1]), (transformedPoints[1,0], transformedPoints[1,1]), (0, 0, 255), 2)
cv2.line(image, (transformedPoints[1,0], transformedPoints[1,1]), (transformedPoints[3,0], transformedPoints[3,1]), (0, 0, 255), 2)
cv2.line(image, (transformedPoints[3,0], transformedPoints[3,1]), (transformedPoints[2,0], transformedPoints[2,1]), (0, 0, 255), 2)
cv2.line(image, (transformedPoints[2,0], transformedPoints[2,1]), (transformedPoints[0,0], transformedPoints[0,1]), (0, 0, 255), 2)
cv2.line(image, (transformedOffsetPoints[0,0], transformedOffsetPoints[0,1]), (transformedOffsetPoints[1,0], transformedOffsetPoints[1,1]), (0, 255, 0), 2)
cv2.line(image, (transformedOffsetPoints[1,0], transformedOffsetPoints[1,1]), (transformedOffsetPoints[3,0], transformedOffsetPoints[3,1]), (0, 255, 0), 2)
cv2.line(image, (transformedOffsetPoints[3,0], transformedOffsetPoints[3,1]), (transformedOffsetPoints[2,0], transformedOffsetPoints[2,1]), (0, 255, 0), 2)
cv2.line(image, (transformedOffsetPoints[2,0], transformedOffsetPoints[2,1]), (transformedOffsetPoints[0,0], transformedOffsetPoints[0,1]), (0, 255, 0), 2)
cv2.circle(image, (transformedPoints[4,0], transformedPoints[4,1]), 5, (255, 0, 0), -1)

# Determine which direction to start based on the distance to the water supply
P2Distance = np.linalg.norm(P2 - WaterSupplyCoord)
P3Distance = np.linalg.norm(P3 - WaterSupplyCoord)
if P2Distance > P3Distance:
    PassEnd = 3
    PathEndPoint = PointsInches[2]
    #print(PassEnd)

else:
    PassEnd = 2
    PathEndPoint = PointsInches[1]
    #print(PassEnd)

PathStartPoint = PointsInches[0]

# Determine the width of the path to display
if DisplayActualPath:
    surfaceCleanerDiameterPixels = 2
else:
    surfaceCleanerDiameterPixels = np.int32(surfaceCleanerDiameter / normalizingFactor * PixelMultiplier)


for i in range(MaxNumberofPasses):
    print("NumberofPasses: ", i)
    #print("PassEnd: ", PassEnd)

    if PassEnd == 2:
        PreviousPassEnd = PassEnd
        if i == 0:
            passLine = lineP12Offset
        else:
            passLine = np.array(offsetLines(PathStartPoint[0], PathStartPoint[1], PathEndPoint[0], PathEndPoint[1], (surfaceCleanerDiameter/2 - pathOverlap)))
        #print(passLine)

        # Check intersection with the line P13
        intersection = lineIntersection(passLine, lineP13Offset)
        if intersection is not None:
            if intersection[1] + (surfaceCleanerDiameter/2) > PointsInches[2,1]:
                nextPoint1 = None
            else:
                nextPoint1 = intersection
        else:
            nextPoint1 = None
             
        # Check intersection with the line P24
        intersection = lineIntersection(passLine, lineP24Offset)
        if intersection is not None:
            if intersection[1] + (surfaceCleanerDiameter/2) > PointsInches[3,1]:
                nextPoint2 = None
            else:
                nextPoint2 = intersection
        else:
            nextPoint2 = None
           
        # Check intersection with the line P34
        if nextPoint1 is None and nextPoint2 is not None:
            intersection = lineIntersection(passLine, lineP34Offset)
            if intersection is not None:
                nextPoint1 = intersection
            else:
                nextPoint1 = None
        elif nextPoint1 is not None and nextPoint2 is None:
            intersection = lineIntersection(passLine, lineP34Offset)
            if intersection is not None:
                nextPoint2 = intersection      
            else:
                nextPoint2 = None
        elif nextPoint1 is not None and nextPoint2 is not None:
            nextPoint1 = nextPoint1
            nextPoint2 = nextPoint2
        else:
            nextPoint1 = None
            nextPoint2 = None
            break

        nextPoint1Pixels = np.int32(((nextPoint1 - midpointCoord) / normalizingFactor) * PixelMultiplier) + np.array([256, 256])
        nextPoint2Pixels = np.int32(((nextPoint2 - midpointCoord) / normalizingFactor) * PixelMultiplier) + np.array([256, 256])
        cv2.line(image, (nextPoint1Pixels[0], nextPoint1Pixels[1]), (nextPoint2Pixels[0], nextPoint2Pixels[1]), (230, 100, 20), surfaceCleanerDiameterPixels)
        PathStartPoint = nextPoint2
        PathEndPoint = nextPoint1
        PassEnd = 5

    elif PassEnd == 1:
        PreviousPassEnd = PassEnd
        passLine = np.array(offsetLines(PathStartPoint[0], PathStartPoint[1], PathEndPoint[0], PathEndPoint[1], -(surfaceCleanerDiameter/2 - pathOverlap)))
        #print(passLine)

        # Check intersection with the line P13
        intersection = lineIntersection(passLine, lineP13Offset)
        if intersection is not None:
            if intersection[1] + (surfaceCleanerDiameter/2) >= PointsInches[2,1]:
                nextPoint2 = None
            else:
                nextPoint2 = intersection
        else:
            nextPoint2 = None
            

        # Check intersection with the line P24
        intersection = lineIntersection(passLine, lineP24Offset)
        if intersection is not None:
            if intersection[1] + (surfaceCleanerDiameter/2) >= PointsInches[3,1]:
                nextPoint1 = None
            else:
                nextPoint1 = intersection
        else:
            nextPoint1 = None
            

        # Check intersection with the line P34
        if nextPoint1 is None and nextPoint2 is not None:
            intersection = lineIntersection(passLine, lineP34Offset)
            if intersection is not None:
                nextPoint1 = intersection  
            else:
                nextPoint1 = None
        elif nextPoint1 is not None and nextPoint2 is None:
            intersection = lineIntersection(passLine, lineP34Offset)
            if intersection is not None:
                nextPoint2 = intersection
            else:
                nextPoint2 = None
        elif nextPoint1 is not None and nextPoint2 is not None:
            nextPoint1 = nextPoint1
            nextPoint2 = nextPoint2
        else:
            nextPoint1 = None
            nextPoint2 = None
            break

        nextPoint1Pixels = np.int32(((nextPoint1 - midpointCoord) / normalizingFactor) * PixelMultiplier) + np.array([256, 256])
        nextPoint2Pixels = np.int32(((nextPoint2 - midpointCoord) / normalizingFactor) * PixelMultiplier) + np.array([256, 256])
        cv2.line(image, (nextPoint1Pixels[0], nextPoint1Pixels[1]), (nextPoint2Pixels[0], nextPoint2Pixels[1]), (230, 100, 20), surfaceCleanerDiameterPixels)
        PathStartPoint = nextPoint2
        PathEndPoint = nextPoint1
        PassEnd = 5

    elif PassEnd == 3:
        PreviousPassEnd = PassEnd
        if i == 0:
            passLine = lineP13Offset
        else:
            passLine = np.array(offsetLines(PathStartPoint[0], PathStartPoint[1], PathEndPoint[0], PathEndPoint[1], -(surfaceCleanerDiameter/2 - pathOverlap)))
        #print(passLine)

        # Check intersection with the line P12
        intersection = lineIntersection(passLine, lineP12Offset)
        if intersection is not None:
            nextPoint1 = intersection
            if intersection[0] + (surfaceCleanerDiameter/2) > PointsInches[1,0]:
                nextPoint1 = None
            else:
                nextPoint1 = intersection
        else:
            nextPoint1 = None

        # Check intersection with the line P34
        intersection = lineIntersection(passLine, lineP34Offset)
        if intersection is not None:
            nextPoint2 = intersection
            if intersection[0] + (surfaceCleanerDiameter/2) > PointsInches[3,0]:
                nextPoint2 = None
            else:
                nextPoint2 = intersection
        else:
            nextPoint2 = None

        # Check intersection with the line P24
        if nextPoint1 is None and nextPoint2 is not None:
            intersection = lineIntersection(passLine, lineP24Offset)
            if intersection is not None:
                nextPoint1 = intersection  
            else:
                nextPoint1 = None
        elif nextPoint1 is not None and nextPoint2 is None:
            intersection = lineIntersection(passLine, lineP24Offset)
            if intersection is not None:
                nextPoint2 = intersection
            else:
                nextPoint2 = None
        elif nextPoint1 is not None and nextPoint2 is not None:
            nextPoint1 = nextPoint1
            nextPoint2 = nextPoint2
        else:
            nextPoint1 = None
            nextPoint2 = None
            break

        nextPoint1Pixels = np.int32(((nextPoint1 - midpointCoord) / normalizingFactor) * PixelMultiplier) + np.array([256, 256])
        nextPoint2Pixels = np.int32(((nextPoint2 - midpointCoord) / normalizingFactor) * PixelMultiplier) + np.array([256, 256])
        cv2.line(image, (nextPoint1Pixels[0], nextPoint1Pixels[1]), (nextPoint2Pixels[0], nextPoint2Pixels[1]), (230, 100, 20), surfaceCleanerDiameterPixels)
        PathStartPoint = nextPoint2
        PathEndPoint = nextPoint1
        PassEnd = 5

    elif PassEnd == 12:
        PreviousPassEnd = PassEnd
        passLine = np.array(offsetLines(PathStartPoint[0], PathStartPoint[1], PathEndPoint[0], PathEndPoint[1], (surfaceCleanerDiameter/2 - pathOverlap)))
        #print(passLine)

        # Check intersection with the line P12
        intersection = lineIntersection(passLine, lineP34Offset)
        if intersection is not None:
            nextPoint1 = intersection
            if intersection[0] + (surfaceCleanerDiameter/2) > PointsInches[1,0]:
                nextPoint1 = None
            else:
                nextPoint1 = intersection
        else:
            nextPoint1 = None

        # Check intersection with the line P34
        intersection = lineIntersection(passLine, lineP12Offset)
        if intersection is not None:
            nextPoint2 = intersection
            if intersection[0] + (surfaceCleanerDiameter/2) > PointsInches[3,0]:
                nextPoint2 = None
            else:
                nextPoint2 = intersection
        else:
            nextPoint2 = None

        # Check intersection with the line P24
        if nextPoint1 is None and nextPoint2 is not None:
            intersection = lineIntersection(passLine, lineP24Offset)
            if intersection is not None:
                nextPoint1 = intersection  
            else:
                nextPoint1 = None
        elif nextPoint1 is not None and nextPoint2 is None:
            intersection = lineIntersection(passLine, lineP24Offset)
            if intersection is not None:
                nextPoint2 = intersection
            else:
                nextPoint2 = None
        elif nextPoint1 is not None and nextPoint2 is not None:
            nextPoint1 = nextPoint1
            nextPoint2 = nextPoint2
        else:
            nextPoint1 = None
            nextPoint2 = None
            break

        nextPoint1Pixels = np.int32(((nextPoint1 - midpointCoord) / normalizingFactor) * PixelMultiplier) + np.array([256, 256])
        nextPoint2Pixels = np.int32(((nextPoint2 - midpointCoord) / normalizingFactor) * PixelMultiplier) + np.array([256, 256])
        cv2.line(image, (nextPoint1Pixels[0], nextPoint1Pixels[1]), (nextPoint2Pixels[0], nextPoint2Pixels[1]), (230, 100, 20), surfaceCleanerDiameterPixels)
        PathStartPoint = nextPoint2
        PathEndPoint = nextPoint1
        PassEnd = 5
        
    elif PassEnd == 5: # 5 is the short path used to connect the longer parallel paths
        # Save the current point as the start point
        CurrentPoint = PathStartPoint
        #print("PreviousPassEnd: ", PreviousPassEnd)

        # Find the next pass line start point
        if PreviousPassEnd == 2:
            nextPassLine = np.array(offsetLines(PathStartPoint[0], PathStartPoint[1], PathEndPoint[0], PathEndPoint[1], -(surfaceCleanerDiameter/2 - pathOverlap)))
        elif PreviousPassEnd == 1:
            nextPassLine = np.array(offsetLines(PathStartPoint[0], PathStartPoint[1], PathEndPoint[0], PathEndPoint[1], (surfaceCleanerDiameter/2 - pathOverlap)))
        elif PreviousPassEnd == 3:
            nextPassLine = np.array(offsetLines(PathStartPoint[0], PathStartPoint[1], PathEndPoint[0], PathEndPoint[1], (surfaceCleanerDiameter/2 - pathOverlap)))
        elif PreviousPassEnd == 12:
            nextPassLine = np.array(offsetLines(PathStartPoint[0], PathStartPoint[1], PathEndPoint[0], PathEndPoint[1], -(surfaceCleanerDiameter/2 - pathOverlap)))
        #print(nextPassLine)

        if PreviousPassEnd == 2:
            intersection = lineIntersection(nextPassLine, lineP24Offset)
            if intersection is not None:
                if intersection[1] + (surfaceCleanerDiameter/2) > PointsInches[3,1]:
                    nextPointStart = None
                else:
                    nextPointStart = intersection
            else:
                nextPointStart = None
                break
                
        if PreviousPassEnd == 1:
            intersection = lineIntersection(nextPassLine, lineP13Offset)
            if intersection is not None:
                if intersection[1] + (surfaceCleanerDiameter/2) > PointsInches[2,1]:
                    nextPointStart = None
                else:
                    nextPointStart = intersection
            else:
                nextPointStart = None
                break
        
        if PreviousPassEnd == 3:
            intersection = lineIntersection(nextPassLine, lineP34Offset)
            if intersection is not None:
                nextPointStart = intersection
                if intersection[0] + (surfaceCleanerDiameter/2) > PointsInches[3,0]:
                    nextPointStart = None
                else:
                    nextPointStart = intersection
            else:
                nextPointStart = None
                break
        
        if PreviousPassEnd == 12:
            intersection = lineIntersection(nextPassLine, lineP12Offset)
            if intersection is not None:
                nextPointStart = intersection
                if intersection[0] + (surfaceCleanerDiameter/2) > PointsInches[1,0]:
                    nextPointStart = None
                else:
                    nextPointStart = intersection
            else:
                nextPointStart = None
                break

        if nextPointStart is None and PreviousPassEnd == 2:
            intersection = lineIntersection(nextPassLine, lineP34Offset)
            if intersection is not None:
                nextPointStart = intersection
            else:
                nextPointStart = None
                break
        if nextPointStart is None and PreviousPassEnd == 1:
            intersection = lineIntersection(nextPassLine, lineP12Offset)
            if intersection is not None:
                nextPointStart = intersection
            else:
                nextPointStart = None
                break

        if nextPointStart is None and PreviousPassEnd == 3:
            intersection = lineIntersection(nextPassLine, lineP24Offset)
            if intersection is not None:
                nextPointStart = intersection
            else:
                nextPointStart = None
                break
            
        if nextPointStart is None and PreviousPassEnd == 12:
            intersection = lineIntersection(nextPassLine, lineP24Offset)
            if intersection is not None:
                nextPointStart = intersection
            else:
                nextPointStart = None
                break

        CurrentPointPixels = np.int32(((CurrentPoint - midpointCoord) / normalizingFactor) * PixelMultiplier) + np.array([256, 256])
        nextPointStartPixels = np.int32(((nextPointStart - midpointCoord) / normalizingFactor) * PixelMultiplier) + np.array([256, 256])
        cv2.line(image, (CurrentPointPixels[0], CurrentPointPixels[1]), (nextPointStartPixels[0], nextPointStartPixels[1]), (230, 100, 20), surfaceCleanerDiameterPixels)
        
        if PreviousPassEnd == 2:
            PassEnd = 1
        elif PreviousPassEnd == 1:
            PassEnd = 2
        elif PreviousPassEnd == 3:
            PassEnd = 12
        elif PreviousPassEnd == 12:
            PassEnd = 3
        
    i = i + 1
    

# Flip the image vertically to orient the image correctly
flippedImage = cv2.flip(image, 0)
resizedImage = cv2.resize(flippedImage, (imageWidth, imageHeight))

# Display the image
cv2.imshow("image", resizedImage)

# Wait for a key press to close the image window
cv2.waitKey(0)
cv2.destroyAllWindows()
 
