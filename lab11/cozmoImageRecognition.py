#!/usr/bin/env python3

import asyncio
import sys
import time

import cv2
import numpy as np

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps, Pose
from imgclassification import ImageClassifier

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


# Define a decorator as a subclass of Annotator; displays battery voltage
class BatteryAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)

# Define a decorator as a subclass of Annotator; displays the symbol outline
class SymbolAnnotator(cozmo.annotate.Annotator):

    symbol = None

    def apply(self, image, scale):
        if SymbolAnnotator.symbol is not None:
            #define and display bounding box with params:
            box = cozmo.util.ImageBox(SymbolAnnotator.symbol[0]-SymbolAnnotator.symbol[2],
                                      SymbolAnnotator.symbol[1]-SymbolAnnotator.symbol[2],
                                      SymbolAnnotator.symbol[2]*2, SymbolAnnotator.symbol[2]*2)
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            SymbolAnnotator.symbol = None


async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    #add annotators for battery level and symbol bounding box
    #robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    #robot.world.image_annotator.add_annotator('symbol', SymbolAnnotator)

    # Get image classifier
    #TODO: Create a mock classifier to develop Cozmo's animations faster
    print("Started training algorithm")
    img_clf = ImageClassifier()
    img_clf.build_classifier()
    print("Finished training algorithm")


    try:
        print("Putting Cozmo in a idle position")
        #await robot.set_head_angle(degrees(0)).wait_for_completed()
        #robot.move_lift(-3)
        print("Finished setting Cozmo")

        while True:
            print("Start getting image")

            #get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            print("Image received")

            #convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)

            print("Converted to opencv format")

            #find the symbol
            #TODO: Use the image classifier here
            imageInstance = img_clf.extract_image_features([opencv_image])
            print("Generated instance by extracting features")
            symbolsList = img_clf.predict_labels(imageInstance)
            print("Predicted label:", symbolsList[0])
            symbol = symbolsList[0]

            print("Image classified")

            #set annotator symbol
            #TODO: Change the annotator
            #SymbolAnnotator.symbol = symbol

            await robot.say_text(symbol, play_excited_animation=True).wait_for_completed()
            await robot.play_anim(name="anim_poked_giggle").wait_for_completed()
            time.sleep(1)


            # if symbol == "drone":
            #     robot.say_text(symbol)
            # elif symbol == "hands":
            #     robot.say_text(symbol)
            # elif symbol == "inspection":
            #     robot.say_text(symbol)
            # elif symbol == "order":
            #     robot.say_text(symbol)
            # elif symbol == "place":
            #     robot.say_text(symbol)
            # elif symbol == "plane":
            #     robot.say_text(symbol)
            # elif symbol == "truck":
            #     robot.say_text(symbol)
            # elif symbol == "none":
            #     robot.say_text(symbol)

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)



if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)

