import os
import cv2
import face_recognition
import numpy as np

def take_picture(name):
    # loading the image path into file_name variable - replace <INSERT YOUR IMAGE NAME HERE> with the path to your image
    camera = cv2.VideoCapture(0)
    success, image = camera.read() # returns 2 values

    if success:
        path = os.path.join('/Users/cynthia/face-recognition/images', name + '.jpg')
        cv2.imwrite(path, image)
        return image
    else:
        raise RuntimeError("Failed to capture image")

# take pictures
# img1 = take_picture("test")

# obtain list of all images in the images folder
path = '/Users/cynthia/face-recognition/images'
images = []
classNames = []
myList = os.listdir(path)

# remove ".jpg" from the names & put in classNames
for cl in myList:
    curImg = cv2.imread(f'{path}/{cl}')
    images.append(curImg)
    classNames.append(os.path.splitext(cl)[0])

# make a list of all image encodings (images represented as vectors)
def findEncodings(images):
    encodeList = []
    for img in images:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        encode = face_recognition.face_encodings(img)[0]
        encodeList.append(encode)
    return encodeList

encodeListKnown = findEncodings(images)

# turn on camera
cap = cv2.VideoCapture(0)

# find matches
while True:
    success, img = cap.read() # give image
    imgS = cv2.resize(img,(0,0), None, 0.25, 0.25) # crop image (speed up process)
    imgS = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # convert image to rbg

    facesCurFrame = face_recognition.face_locations(imgS)  # find exact location of live face
    encodesCurFrame = face_recognition.face_encodings(imgS, facesCurFrame) # get encoding of live face (vector format)

    for encodeFace, faceLoc in zip(encodesCurFrame, facesCurFrame): # iterate through live faces & compare to image in saved file
        matches = face_recognition.compare_faces(encodeListKnown, encodeFace) # compare list known & live face
        faceDis = face_recognition.face_distance(encodeListKnown,encodeFace)
        
        matchIndex = np.argmin(faceDis) # obtain index of the exa

        # smallest faceDis value
        best_match_distance = faceDis[matchIndex]
        print(best_match_distance)
        
        if(best_match_distance > 0.43):
            print("UNKNOWN")
        elif matches[matchIndex]:
            name = classNames[matchIndex].upper()
            print(name)
