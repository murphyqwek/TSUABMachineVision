import cv2
import numpy as np
import mvsdk
import platform
import os
import time

# Параметры видео
OUTPUT_VIDEO_PATH = "output.avi"
OUTPUT_FRAMES_DIR = "frames"
FRAME_RATE = 300  # Количество кадров в секунду

# Убедимся, что папка для раскадровки существует
if not os.path.exists(OUTPUT_FRAMES_DIR):
    os.makedirs(OUTPUT_FRAMES_DIR)

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
    max_frames = 300  # Максимальное количество кадров для записи
    
    start_time = time.time()
    while frame_count < max_frames and (cv2.waitKey(1) & 0xFF) != ord('q'):
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
            
            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)
            
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if monoCamera else 3))
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
            
            # Сохранение кадра
            #frame_filename = os.path.join(OUTPUT_FRAMES_DIR, f"frame_{frame_count:04d}.png")
            #cv2.imwrite(frame_filename, frame)
            
            # Запись видео
            if not monoCamera:
                out.write(frame)
            else:
                out.write(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR))
            
            frame_count += 1
            cv2.imshow("Recording - Press 'q' to stop", frame)
            
        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print(f"CameraGetImageBuffer failed({e.error_code}): {e.message}")
    
    end_time = time.time()
    exec_time = end_time - start_time
    print(f"Finished. Time: {exec_time:.2f}")

    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)
    out.release()

    cap = cv2.VideoCapture(OUTPUT_VIDEO_PATH)

    frame_count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        # Сохраняем кадры как изображения
        cv2.imwrite(f'./1/frame_{frame_count}.jpg', frame)
        frame_count += 1


    cv2.destroyAllWindows()
    print("Recording finished.")

def main():
    try:
        main_loop()
    finally:
        cv2.destroyAllWindows()

main()