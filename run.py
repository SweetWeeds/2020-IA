import cv2
from src.multi_hand_tracker import MultiHandTracker3D
from src.multi_hand_tracker import calc_angle
from MyAX12A import AX12A_Handler
import MyAX12A
import threading
import time

WINDOW = "Hand Tracking"
PALM_MODEL_PATH = "models/palm_detection_without_custom_op.tflite"
LANDMARK_MODEL_PATH = "models/hand_landmark_3d.tflite"
ANCHORS_PATH = "models/anchors.csv"

HAND_MIN_ANGLE = 120
HAND_MAX_ANGLE = 170

PALM_MIN_ANGLE = 110
PALM_MAX_ANGLE = 170

MIDDLE_MOTOR_ID = 13
INDEX_MOTOR_ID  = 14
THUMB_MOTOR_ID  = 15
PALM_MOTOR_ID   = 17

MIDDLE_IDX = 10
INDEX_IDX = 6
THUMB_IDX = 2
PALM_IDX = 2

angle_target = {"PALM"  : [PALM_IDX,    PALM_MOTOR_ID],\
                "THUMB" : [THUMB_IDX,   THUMB_MOTOR_ID],\
                "INDEX" : [INDEX_IDX,   INDEX_MOTOR_ID],\
                "MIDDLE": [MIDDLE_IDX,  MIDDLE_MOTOR_ID]}

POINT_COLOR = (0, 255, 0)
CONNECTION_COLOR = (255, 0, 0)
THICKNESS = 2

cv2.namedWindow(WINDOW)
capture = cv2.VideoCapture(0)

if capture.isOpened():
    hasFrame, frame = capture.read()
else:
    hasFrame = False

#        8   12  16  20
#        |   |   |   |
#        7   11  15  19
#    4   |   |   |   |
#    |   6   10  14  18
#    3   |   |   |   |
#    |   5---9---13--17
#    2    \         /
#     \    \       /
#      1    \     /
#       \    \   /
#        ------0-
connections = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (5, 6), (6, 7), (7, 8),
    (9, 10), (10, 11), (11, 12),
    (13, 14), (14, 15), (15, 16),
    (17, 18), (18, 19), (19, 20),
    (0, 5), (5, 9), (9, 13), (13, 17), (0, 17)
]


detector = MultiHandTracker3D(
    PALM_MODEL_PATH,
    LANDMARK_MODEL_PATH,
    ANCHORS_PATH,
    box_shift=0.2,
    box_enlarge=1.3
)

DXL_IDs = [MIDDLE_MOTOR_ID, INDEX_MOTOR_ID, THUMB_MOTOR_ID, PALM_MOTOR_ID]
CURRENT_HAND_POSE = {MIDDLE_MOTOR_ID : HAND_MIN_ANGLE, INDEX_MOTOR_ID : HAND_MIN_ANGLE, THUMB_MOTOR_ID : HAND_MIN_ANGLE, PALM_MOTOR_ID : HAND_MIN_ANGLE}

angle_dct = dict()
AX12A_Hnd = AX12A_Handler(DXL_IDs=DXL_IDs, DEVICENAME="/dev/ttyUSB0", HAND_MAX_ANGLE=HAND_MAX_ANGLE, HAND_MIN_ANGLE=HAND_MIN_ANGLE, CONVERT_UNIT=True)

def AngleFilter(angle, isPalm=False):
    if not isPalm:
        result = int(MyAX12A.MOTOR_MIN_ANGLE + (angle - HAND_MIN_ANGLE) / (HAND_MAX_ANGLE - HAND_MIN_ANGLE) * (MyAX12A.MOTOR_MAX_ANGLE - MyAX12A.MOTOR_MIN_ANGLE))
    else:
        result = int(MyAX12A.PALM_MOTOR_MIN_ANGLE + (angle - PALM_MIN_ANGLE) / (PALM_MAX_ANGLE - PALM_MIN_ANGLE) * (MyAX12A.PALM_MOTOR_MAX_ANGLE - MyAX12A.PALM_MOTOR_MIN_ANGLE))
    #print("RESULT:{}".format(result))
    return result

def MotorThread():
    while(True):
        #print(angle_dct)
        if angle_dct == {}:
            continue
        #print(angle_dct)
        AX12A_Hnd.ControlPos(angle_dct)
        time.sleep(0.0001)

t = threading.Thread(target=MotorThread)
t.start()

while hasFrame:
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    points, _ = detector(image)
    if points is not None:
        #for points in hands:
        points = points[0]
        for point in points:
            if not point.all():
                break
            x, y, z = point
            cv2.circle(frame, (int(x), int(y)), THICKNESS * 2, POINT_COLOR, THICKNESS)
        for connection in connections:
            x0, y0, z0 = points[connection[0]]
            x1, y1, z1 = points[connection[1]]
            cv2.line(frame, (int(x0), int(y0)), (int(x1), int(y1)), CONNECTION_COLOR, THICKNESS)
        for target_part, VAL in angle_target.items():
            if VAL[1] == PALM_MOTOR_ID:
                print("THUMB:{}".format(calc_angle(VAL[0], points)))
            angle_dct.update({VAL[1] : AngleFilter(calc_angle(VAL[0], points), isPalm=VAL[1] == PALM_MOTOR_ID)})
        #AX12A_Hnd.ControlPos(angle_dct)
    print(angle_dct)
    cv2.imshow(WINDOW, frame)
    hasFrame, frame = capture.read()
    key = cv2.waitKey(1)
    if key == 27:
        break

capture.release()
cv2.destroyAllWindows()
