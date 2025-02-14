import cv2
import numpy as np
import mvsdk
import platform
import os
import time

# Параметры видео
OUTPUT_VIDEO_PATH = "output.avi"
OUTPUT_FRAMES_DIR = "frames"
FRAME_RATE = 900  # Количество кадров в секунду

frames = []

# Убедимся, что папка для раскадровки существует
if not os.path.exists(OUTPUT_FRAMES_DIR):
    os.makedirs(OUTPUT_FRAMES_DIR)

def saveFrame(pFrameBuffer, FrameHead, frame_count):
    mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)
            
    frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
    frame = np.frombuffer(frame_data, dtype=np.uint8)
    frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1))
    frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)

    frame_filename = os.path.join(OUTPUT_FRAMES_DIR, f"frame_{frame_count:04d}.png")
    cv2.imwrite(frame_filename, frame)

def main_loop():
    DevList = mvsdk.CameraEnumerateDevice()
    if len(DevList) < 1:
        print("No camera was found!")
        return
    
    DevInfo = DevList[0]
    hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
    
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
    
    mvsdk.CameraSetTriggerMode(hCamera, 0)
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
    mvsdk.CameraPlay(hCamera)
    
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)
    
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    out = cv2.VideoWriter(OUTPUT_VIDEO_PATH, fourcc, FRAME_RATE, (640, 480))
    
    frame_count = 0
    max_frames = 10  # Максимальное количество кадров для записи

    print("Begin")
    start_time = time.time()
    while frame_count < max_frames and (cv2.waitKey(1) & 0xFF) != ord('q'):
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            global frames
            frames.append([pFrameBuffer, FrameHead])
            frame_count += 1
            continue
            
            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)
            
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if monoCamera else 3))
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
            
            # Сохранение кадра
            frame_filename = os.path.join(OUTPUT_FRAMES_DIR, f"frame_{frame_count:04d}.png")
            cv2.imwrite(frame_filename, frame)
            
            # Запись видео
            if not monoCamera:
                out.write(frame)
            else:
                out.write(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR))
            
            frame_count += 1
            #cv2.imshow("Recording - Press 'q' to stop", frame)
            
        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print(f"CameraGetImageBuffer failed({e.error_code}): {e.message}")
    end_time = time.time()
    executiom_time = end_time - start_time
    print(f"Recording finished.: {executiom_time:.6f} секунд")
    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)
    out.release()
    cv2.destroyAllWindows()

    print("Saving started.")
    global frames
    for frameData, i in enumerate(frames):
        saveFrame(frameData[0], frameData[1], i)
    print("Saving finished.")

def main():
    try:
        main_loop()
    finally:
        cv2.destroyAllWindows()

main()