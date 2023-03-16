import torch
import numpy as np
import cv2
from PIL import Image
from ImageProcessing import getInfo
from ImageProcessing import getInfoWolf
import os

def runYolov5(client, responses, model, vehicleName):
    # get response object with input image
    height, width, sceneRGB2 = getInfo.getSegInfo(responses)

    #print("RESULTS yolov5:")
    model.classes = [0]
    results=model(sceneRGB2)
    results.print()

    # get the bounding boxes and confidence scores for single image
    validDetection = False
    resultsPandas = results.pandas().xyxy[0]
    confidenceArr = resultsPandas.confidence
    xminArr = resultsPandas.xmin
    yminArr = resultsPandas.ymin
    xmaxArr = resultsPandas.xmax
    ymaxArr = resultsPandas.ymax

    maxConfidence = 0
    maxConfidenceGPS = [None, None]
    detection = 0
    maxConfidenceDetection = 0

    cwd = os.getcwd()
    dataDir=os.path.join(str(cwd),'yolov5Images')
    isExist=os.path.exists(dataDir)

    if not isExist:
        # make directory if not already there
        os.makedirs(dataDir)

    #ToDo: just pass loop index adn drone name
    # or in final build just overwite
    j=0
    while os.path.exists(dataDir + "/" + ('%s' % j)+"yoloDetect.jpg"):
        j+=1

    if(len(resultsPandas) > 0):
        confidence = confidenceArr[0]
        # if confidence is high enough use for GPS estimation
        if(confidence > 0.3):
            validDetection=True
            xmin, ymin, xmax, ymax = int(xminArr[0]), int(yminArr[0]), int(xmaxArr[0]), int(ymaxArr[0])
            start_point = (xmin, ymin)
            end_point = (xmax, ymax)
            newImag = cv2.rectangle(sceneRGB2, start_point, end_point, (0, 255, 0), 2)
            # save new image only with highest confidence detection
            cv2.imwrite(dataDir + "/" + ('%s' % j)+"newImg.jpg", newImag)

            #print("------------------------------------------------------------------------------------------------------------------")
            # use bb dimensions/location for GPS estimation
            alt, lat, lon = getInfoWolf.getWolfGPSEstimate(client, responses, vehicleName, xmin, ymin, xmax, ymax)
            #print("\tWOLF ESTIMATE: "+str(alt)+" alt, " + str(lat) + " lat, " + str(lon) + " lon")
            maxConfidenceGPS[1]=lat
            maxConfidenceGPS[0]=lon
            #print("------------------------------------------------------------------------------------------------------------------")

            # write corresponding text file
            with open(dataDir + "/" + ('%s' % j)+"GPSEstimate.txt", 'w') as f:
                f.write(str(resultsPandas))
                f.write("\n\tMax Confidence: "+str(confidence))
                f.write("\n\tMax Confidence Estimate:"+ str(lat) + " lat, " + str(lon) + " lon")
                f.close()

            # save yolov5 results image
            results.ims # array of original images (as np array) passed to model for inference
            results.render()  # updates results.ims with boxes and labels

            cwd = os.getcwd()
            dataDir=os.path.join(str(cwd),'yolov5Images')
            #print('CWD: '+dataDir)
            isExist=os.path.exists(dataDir)

            if not isExist:
                # make directory if not already there
                os.makedirs(dataDir)
                #print('Created: ' + dataDir)
                
            for im in results.ims:
                pil_image = Image.fromarray(im).convert('RGB') 
                open_cv_image = np.array(pil_image) 
                # Convert RGB to BGR 
                open_cv_image = open_cv_image[:, :, ::-1].copy()

                # TODO: Draw rectangles using bounding box dimensions array
                cv2.imwrite(dataDir + "/" + ('%s' % j)+"yoloDetect.jpg", open_cv_image)
                cv2.waitKey(1)


    # print("GPS estimation cycle complete.")
    # print("MAX CONFIDENCE ESTIMATE:"+ str(maxConfidenceGPS[1]) + " lat, " + str(maxConfidenceGPS[0]) + " lon")
    # print("\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n")
 

    return maxConfidenceGPS