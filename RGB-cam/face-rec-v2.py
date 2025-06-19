import os
import cv2
import face_recognition
import numpy as np

def take_picture(name, camera):
    success, image = camera.read()

    if success:
        path = os.path.join('/home/robot/summer-research-robot/RGB-cam/images', name + '.jpg')
        cv2.imwrite(path, image)
        if not os.path.exists(path):
            print(f"❌ Failed to save image to {path}")
    else:
        print("❌ Failed to capture image from camera")

path = '/home/robot/summer-research-robot/RGB-cam/images'
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

unknown_num = 0

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
        
        if(best_match_distance > 0.43): # unknown face
            print("UNKNOWN")
            unknown_num += 1 

            if(unknown_num > 3): # check three times that its unknown just to be sure
                name = input("Welcome new visitor! What is your name? : ")
                take_picture(name, cap) # enter your name

                unknown_num = 0 # reset unknown count to 0
                new_img = cv2.imread(f'{path}/{name}.jpg') # update list
                new_img_rgb = cv2.cvtColor(new_img, cv2.COLOR_BGR2RGB)
                new_encoding = face_recognition.face_encodings(new_img_rgb)
                if new_encoding:
                    encodeListKnown.append(new_encoding[0])
                    classNames.append(name)

        elif matches[matchIndex]: # found match!
            unknown_num = 0
            name = classNames[matchIndex].upper() # name of matched person
            print(name)



