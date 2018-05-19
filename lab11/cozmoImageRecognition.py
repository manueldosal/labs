#!/usr/bin/env python3

import asyncio
import sys
import time
import operator

import cv2
import numpy as np

import cozmo
from cozmo.util import degrees
from imgclassification import ImageClassifier

import pickle

async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    # Get image classifier
    img_clf = ImageClassifier()

    # Model was previously trained and saved in imgRecognition.sav
    # print("Started training algorithm")
    # classifierModel = img_clf.build_and_return_classifier()
    # print("Finished training algorithm")
    # print("Saving model")
    # pickle.dump(classifierModel, open("imgRecognition.sav", 'wb'))
    # print("Finished saving model")

    # Reading model previously saved
    savedClassifier = pickle.load(open("imgRecognition.sav", 'rb'))

    print("Putting Cozmo in a idle position")
    await robot.set_head_angle(degrees(0)).wait_for_completed()
    robot.move_lift(-3)
    print("Finished setting Cozmo")

    try:
        while True:

            symbolCount = {
                "drone" : 0,
                "hands" : 0,
                "inspection" : 0,
                "order" : 0,
                "place" : 0,
                "plane" : 0,
                "truck" : 0,
                "none" : 0,
            }

            for i in range(10):
                print("Getting cozmo's image")
                #get camera image
                event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
                print("Image received")

                #convert camera image to opencv format
                opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
                print("Converted to opencv format")

                #Find the symbol
                imageInstance = img_clf.extract_image_features([opencv_image])
                print("Generated instance by extracting features")
                symbolsList = savedClassifier.predict(imageInstance)
                print("Predicted label:", symbolsList[0])
                symbol = symbolsList[0]
                symbolCount[symbol] = symbolCount[symbol] + 1

            print("symbolCount:", symbolCount)
            finalSymbol = max(symbolCount.items(), key=operator.itemgetter(1))[0]

            # Make cozmo tell what he's seeing
            await robot.say_text(finalSymbol).wait_for_completed()

            if finalSymbol == "drone":
                await robot.play_anim(name="anim_bored_01").wait_for_completed()
            elif finalSymbol == "hands":
                await robot.play_anim(name="anim_poked_giggle").wait_for_completed()
            elif finalSymbol == "inspection":
                await robot.play_anim(name="anim_pounce_success_02").wait_for_completed()
            elif finalSymbol == "order":
                await robot.play_anim(name="anim_bored_event_02").wait_for_completed()
            elif finalSymbol == "place":
                await robot.play_anim(name="anim_bored_event_03").wait_for_completed()
            elif finalSymbol == "plane":
                await robot.play_anim(name="anim_petdetection_cat_01").wait_for_completed()
            elif finalSymbol == "truck":
                await robot.play_anim(name="anim_petdetection_dog_03").wait_for_completed()
            elif finalSymbol == "none":
                await robot.play_anim(name="anim_reacttoface_unidentified_02").wait_for_completed()
            
            print("Putting Cozmo in a idle position")
            await robot.set_head_angle(degrees(0)).wait_for_completed()
            robot.move_lift(-3)
            print("Finished setting Cozmo")

            time.sleep(5)

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print("COZMO IS BUSY")
        print(e)



if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)

