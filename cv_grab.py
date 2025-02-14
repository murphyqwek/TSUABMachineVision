#coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform

def main_loop():
	# Enumarate camers
	DevList = mvsdk.CameraEnumerateDevice()
	nDev = len(DevList)
	if nDev < 1:
		print("No camera was found!")
		return

	for i, DevInfo in enumerate(DevList):
		print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
	i = 0 if nDev == 1 else int(input("Select camera: "))
	DevInfo = DevList[i]
	print(DevInfo)

	# открыть камеру
	hCamera = 0
	try:
		hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
	except mvsdk.CameraException as e:
		print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
		return

	# Получить описание функций камеры
	cap = mvsdk.CameraGetCapability(hCamera)

	# Определите, черно-белая это камера или цветная.
	monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

	# Черно-белая камера позволяет интернет-провайдеру напрямую выводить данные MONO вместо расширения их в 24-битные оттенки серого с R=G=B.
	if monoCamera:
		mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
	else:
		mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

	# Переключите режим камеры на непрерывную съемку
	mvsdk.CameraSetTriggerMode(hCamera, 0)

	# Ручная экспозиция, время экспозиции 30 мс.
	mvsdk.CameraSetAeState(hCamera, 0)
	mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

	# Пусть внутренний поток создания изображений SDK начнет работать
	mvsdk.CameraPlay(hCamera)

	# Рассчитайте размер, необходимый для буфера RGB, который выделяется непосредственно в соответствии с максимальным разрешением камеры.
	FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

	# Выделить буфер RGB для хранения изображений, выводимых интернет-провайдером
	# Примечание: данные RAW передаются с камеры на ПК и преобразуются в данные RGB через программное обеспечение ISP на ПК (если это черно-белая камера, конвертировать формат нет необходимости, но у ISP есть другая обработка, поэтому этот буфер также необходимо выделить)
	pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

	while (cv2.waitKey(1) & 0xFF) != ord('q'):
		# Получить кадр с камеры
		try:
			pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
			mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
			mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

			# Данные изображения, полученные в Windows, перевернуты и сохранены в формате BMP. Чтобы преобразовать в opencv, вам нужно перевернуть его вверх и вниз, чтобы он был положительным.
			# Непосредственный вывод положительных значений в Linux без перелистывания вверх и вниз.
			if platform.system() == "Windows":
				mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)
			
			# На данный момент изображение сохранено в pFrameBuffer. Для цветных камер pFrameBuffer=данные RGB, а для черно-белых камер — pFrameBuffer=8-битные данные в оттенках серого.
			# Конвертируем pFrameBuffer в формат изображения opencv для последующей обработки алгоритма
			frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
			frame = np.frombuffer(frame_data, dtype=np.uint8)
			frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3) )

			frame = cv2.resize(frame, (640,480), interpolation = cv2.INTER_LINEAR)
			cv2.imshow("Press q to end", frame)
			
		except mvsdk.CameraException as e:
			if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
				print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message) )

	# Выключение
	mvsdk.CameraUnInit(hCamera)

	# Освободить фреймбуфер
	mvsdk.CameraAlignFree(pFrameBuffer)

def main():
	try:
		main_loop()
	finally:
		cv2.destroyAllWindows()

main()
