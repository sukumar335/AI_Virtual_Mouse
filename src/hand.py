import cv2
import mediapipe as mp
import pyautogui
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
import screen_brightness_control as sbcontrol
import math
from enum import IntEnum
from google.protobuf.json_format import MessageToDict
pyautogui.FAILSAFE = False
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

class handgesture(IntEnum):
    fist = 0
    mid = 4
    last3 = 7
    index = 8
    first2 = 12
    last = 15
    palm = 31
    indandmid = 33
    twofingcls = 34
    handprimary = 35
    handsecondary = 36
    twofingwide = 37
class handselect(IntEnum):
    secondary = 0
    primary = 1
class handgest:

    def _init_(self, handpos):
        self.finger = 0
        self.ori_gesture = handgesture.palm
        self.prev_gesture = handgesture.palm
        self.frame_count = 0
        self.gestureoutput = None
        self.handpos = handpos

    def update_gestureoutput(self, gestureoutput):
        self.gestureoutput = gestureoutput

    def set_finger_state(self):
        if self.gestureoutput == None:
            return

        points = [[8, 5, 0], [12, 9, 0], [16, 13, 0], [20, 17, 0]]
        self.finger = 0
        self.finger = self.finger | 0
        for idx, point in enumerate(points):

            length1 = self.get_signed_length(point[:2])
            length2 = self.get_signed_length(point[1:])

            try:
                ratio = round(length1 / length2, 1)
            except:
                ratio = round(length1 / 0.01, 1)

            self.finger = self.finger << 1
            if ratio > 0.5:
                self.finger = self.finger | 1

    def get_signed_length(self, point):
        sign = -1
        if self.gestureoutput.landmark[point[0]].y < self.gestureoutput.landmark[point[1]].y:
            sign = 1
        length = (self.gestureoutput.landmark[point[0]].x - self.gestureoutput.landmark[point[1]].x) ** 2
        length += (self.gestureoutput.landmark[point[0]].y - self.gestureoutput.landmark[point[1]].y) ** 2
        length = math.sqrt(length)
        return length * sign

    def get_length(self, point):
        length = (self.gestureoutput.landmark[point[0]].x - self.gestureoutput.landmark[point[1]].x) ** 2
        length += (self.gestureoutput.landmark[point[0]].y - self.gestureoutput.landmark[point[1]].y) ** 2
        length = math.sqrt(length)
        return length


    def get_gesture(self):
        if self.gestureoutput == None:
            return handgesture.palm

        current_gesture = handgesture.palm
        if self.finger in [handgesture.last3, handgesture.last] and self.get_length([8, 4]) < 0.05:
            if self.handpos == handselect.secondary:
                current_gesture = handgesture.handsecondary
            else:
                current_gesture = handgesture.handprimary

        elif handgesture.first2 == self.finger:
            point = [[8, 12], [5, 9]]
            length1 = self.get_length(point[0])
            length2 = self.get_length(point[1])
            ratio = length1 / length2
            print(ratio)
            if 3.0 > ratio > 1.7:
                current_gesture = handgesture.indandmid
            else:
                if ratio < 1:
                    current_gesture = handgesture.twofingcls
                elif ratio > 3.0:
                    current_gesture = handgesture.twofingwide
                else:
                    current_gesture = handgesture.mid

        else:
            current_gesture = self.finger

        if current_gesture == self.prev_gesture:
            self.frame_count += 1
        else:
            self.frame_count = 0

        self.prev_gesture = current_gesture

        if self.frame_count > 4:
            self.ori_gesture = current_gesture
        return self.ori_gesture


class Controller:
    flag = False
    grabflag = False
    pinchmajorflag = False
    pinchminorflag = False
    pinchstartxcoord = None
    pinchstartycoord = None
    pinchdirectionflag = None
    prevpinchlv = 0
    pinchlv = 0
    framecount = 0
    prev_hand = None
    pinch_threshold = 0.3

    def get_position(gestureoutput):
        point = 9
        position = [gestureoutput.landmark[point].x, gestureoutput.landmark[point].y]
        sx, sy = pyautogui.size()
        x_old, y_old = pyautogui.position()
        x = int(position[0] * sx)
        y = int(position[1] * sy)
        if Controller.prev_hand is None:
            Controller.prev_hand = x, y
        delta_x = x - Controller.prev_hand[0]
        delta_y = y - Controller.prev_hand[1]

        lengthsq = delta_x * 2 + delta_y * 2
        ratio = 1
        Controller.prev_hand = [x, y]

        if lengthsq <= 25:
            ratio = 0
        elif lengthsq <= 900:
            ratio = 0.07 * (lengthsq ** (1 / 2))
        else:
            ratio = 2.1
        x, y = x_old + delta_x * ratio, y_old + delta_y * ratio
        return (x, y)

    def pinch_control_init(gestureoutput):
        Controller.pinchstartxcoord = gestureoutput.landmark[8].x
        Controller.pinchstartycoord = gestureoutput.landmark[8].y
        Controller.pinchlv = 0
        Controller.prevpinchlv = 0
        Controller.framecount = 0


    def pinch_control(gestureoutput, controlHorizontal, controlVertical):
        if Controller.framecount == 5:
            Controller.framecount = 0
            Controller.pinchlv = Controller.prevpinchlv

            if Controller.pinchdirectionflag == True:
                controlHorizontal()

            elif Controller.pinchdirectionflag == False:
                controlVertical()

        lvx = Controller.getpinchxlv(gestureoutput)
        lvy = Controller.getpinchylv(gestureoutput)

        if abs(lvy) > abs(lvx) and abs(lvy) > Controller.pinch_threshold:
            Controller.pinchdirectionflag = False
            if abs(Controller.prevpinchlv - lvy) < Controller.pinch_threshold:
                Controller.framecount += 1
            else:
                Controller.prevpinchlv = lvy
                Controller.framecount = 0

        elif abs(lvx) > Controller.pinch_threshold:
            Controller.pinchdirectionflag = True
            if abs(Controller.prevpinchlv - lvx) < Controller.pinch_threshold:
                Controller.framecount += 1
            else:
                Controller.prevpinchlv = lvx
                Controller.framecount = 0

    def getpinchylv(gestureoutput):
        length = round((Controller.pinchstartycoord - gestureoutput.landmark[8].y) * 10, 1)
        return length

    def getpinchxlv(gestureoutput):
        length = round((gestureoutput.landmark[8].x - Controller.pinchstartxcoord) * 10, 1)
        return length

    def changesystembrightness():
        currentBrightnessLv = sbcontrol.get_brightness() / 100.0
        currentBrightnessLv += Controller.pinchlv / 50.0
        if currentBrightnessLv > 1.0:
            currentBrightnessLv = 1.0
        elif currentBrightnessLv < 0.0:
            currentBrightnessLv = 0.0
        sbcontrol.fade_brightness(int(100 * currentBrightnessLv), start=sbcontrol.get_brightness())

    def changesystemvolume():
        devices = AudioUtilities.GetSpeakers()
        interface = devices.Activate(IAudioEndpointVolume.iid, CLSCTX_ALL, None)
        volume = cast(interface, POINTER(IAudioEndpointVolume))
        currentVolumeLv = volume.GetMasterVolumeLevelScalar()
        currentVolumeLv += Controller.pinchlv / 50.0
        if currentVolumeLv > 1.0:
            currentVolumeLv = 1.0
        elif currentVolumeLv < 0.0:
            currentVolumeLv = 0.0
        volume.SetMasterVolumeLevelScalar(currentVolumeLv, None)

    def scrollvertical():
        pyautogui.scroll(120 if Controller.pinchlv > 0.0 else -120)

    def scrollhorizontal():
        pyautogui.keyDown('shift')
        pyautogui.keyDown('ctrl')
        pyautogui.scroll(-120 if Controller.pinchlv > 0.0 else 120)
        pyautogui.keyUp('ctrl')
        pyautogui.keyUp('shift')

    def handle_controls(gesture, gestureoutput):
        x, y = None, None
        if gesture != handgesture.palm:
            x, y = Controller.get_position(gestureoutput)

        # flag reset
        if gesture != handgesture.fist and Controller.grabflag:
            Controller.grabflag = False
            pyautogui.mouseUp(button="left")

        if gesture != handgesture.handprimary and Controller.pinchmajorflag:
            Controller.pinchmajorflag = False

        if gesture != handgesture.handsecondary and Controller.pinchminorflag:
            Controller.pinchminorflag = False


        if gesture == handgesture.indandmid:
            Controller.flag = True
            pyautogui.moveTo(x, y, duration=0.1)

        elif gesture == handgesture.fist:
            if not Controller.grabflag:
                Controller.grabflag = True
                pyautogui.mouseDown(button="left")
            pyautogui.moveTo(x, y, duration=0.1)

        elif gesture == handgesture.mid and Controller.flag:
            pyautogui.click()
            Controller.flag = False

        elif gesture == handgesture.index and Controller.flag:
            pyautogui.click(button='right')
            Controller.flag = False

        elif gesture == handgesture.twofingcls and Controller.flag:
            pyautogui.doubleClick()
            Controller.flag = False

        elif gesture == handgesture.twofingwide and Controller.flag:
            pyautogui.keyDown('winleft')
            pyautogui.keyDown('prntscrn')
            pyautogui.keyUp('winleft')
            pyautogui.keyUp('prntscrn')
            Controller.flag = False

        elif gesture == handgesture.handsecondary:
            if Controller.pinchminorflag == False:
                Controller.pinch_control_init(gestureoutput)
                Controller.pinchminorflag = True
            Controller.pinch_control(gestureoutput, Controller.changesystembrightness, Controller.changesystemvolume)

        elif gesture == handgesture.handprimary:
            if Controller.pinchmajorflag == False:
                Controller.pinch_control_init(gestureoutput)
                Controller.pinchmajorflag = True
            Controller.pinch_control(gestureoutput, Controller.scrollhorizontal, Controller.scrollvertical)


class gestcnrl:
    dom_hand = True

    def classify_hands(results):
        left, right = None, None
        try:
            hand_dict = MessageToDict(results.multi_handedness[0])
            if hand_dict['classification'][0]['label'] == 'Right':
                right = results.multi_hand_landmarks[0]
            else:
                left = results.multi_hand_landmarks[0]
        except:
            pass

        try:
            hand_dict = MessageToDict(results.multi_handedness[1])
            if hand_dict['classification'][0]['label'] == 'Right':
                right = results.multi_hand_landmarks[1]
            else:
                left = results.multi_hand_landmarks[1]
        except:
            pass

        if gestcnrl.dom_hand == True:
            gestcnrl.hr_major = right
            gestcnrl.hr_minor = left
        else:
            gestcnrl.hr_major = left
            gestcnrl.hr_minor = right

    def _init_(self):
        gestcnrl.gc_mode = 1
        gestcnrl.cap = cv2.VideoCapture(0)
        gestcnrl.CAM_HEIGHT = gestcnrl.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        gestcnrl.CAM_WIDTH = gestcnrl.cap.get(cv2.CAP_PROP_FRAME_WIDTH)

    def start(self):

        handmajor = handgest(handselect.primary)
        handminor = handgest(handselect.secondary)

        with mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
            while gestcnrl.cap.isOpened() and gestcnrl.gc_mode:
                success, image = gestcnrl.cap.read()

                image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
                image.flags.writeable = False
                results = hands.process(image)

                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                if results.multi_hand_landmarks:
                    gestcnrl.classify_hands(results)
                    handmajor.update_gestureoutput(gestcnrl.hr_major)
                    handminor.update_gestureoutput(gestcnrl.hr_minor)

                    handmajor.set_finger_state()
                    handminor.set_finger_state()
                    gest_name = handminor.get_gesture()

                    if gest_name == handgesture.handsecondary:
                        Controller.handle_controls(gest_name, handminor.gestureoutput)
                    else:
                        gest_name = handmajor.get_gesture()
                        Controller.handle_controls(gest_name, handmajor.gestureoutput)

                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                else:
                    Controller.prev_hand = None
                cv2.imshow('Gesture Controller Window', image)
                if cv2.waitKey(5) & 0xFF == 13:
                    break
        gestcnrl.cap.release()
        cv2.destroyAllWindows()


gest_cntrl = gestcnrl()
gest_cntrl.start()