from multiprocessing import Process
from multiprocessing import Manager
import argparse
import signal
import time
import sys
sys.path.append('/usr/local/lib/python3.7/site-packages')
sys.path.append('/usr/local/lib/python3.7/dist-packages')
sys.path.append('/home/pi/Documents/pyimagesearch/objcenter')
from objcenter import ObjCenter
from imutils.video import VideoStream, FPS
import cv2
import RPi.GPIO as GPIO
from time import sleep
from rpi_ws281x import *


def obj_center(args, objX, objY, centerX, centerY):

	# signal trap to handle keyboard interrupt
	#signal.signal(signal.SIGINT, signal_handler)
	numFrames = 300 #num of frames to loop over
 
	# start the video stream and wait for the camera to warm up
	vs = VideoStream(usePiCamera=True, resolution=(320,240), framerate=10).start()
	print("videoStream starting")
	time.sleep(2.0)
 	
	# initialize the object center finder
	# Start FPS counter
	obj = ObjCenter(args["cascade"])
 	#fps = FPS().start()
	#print("Starting FPS counter")
	# loop indefinitely
	try:
		while True:

		# grab the frame from the threaded video stream and flip it
		# vertically (since our camera was upside down)
			frame = vs.read()

		# calculate the center of the frame as this is where we will
		# try to keep the object
			(H, W) = frame.shape[:2]
			centerX.value = W // 2
			centerY.value = H // 2
 
		# find the object's location
			objectLoc = obj.update(frame, (centerX.value, centerY.value))
			((objX.value, objY.value), rect) = objectLoc

 
		# extract the bounding box and draw it
			if rect is not None:
				(x, y, w, h) = rect
				cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0),2)
				print ("X:", objX.value, " Y:", objY.value," Center X: ", centerX.value, " Center Y: ",centerY.value)



		# display the frame to the screen
			cv2.imshow("Pan-Tilt Face Tracking", frame)
			cv2.waitKey(1)

		#fps.update()
	except KeyboardInterrupt:
	#fps.stop()
	#print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
	#print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
		cv2.destroyAllWindows()
		vs.stop()

def motor_driver(objX, centerX):
	DIR = 5
        STEP = 6
	HALF = 16
        CW = 1
        CCW = 0
        SPR = 48
        GPIO.setmode(GPIO.BCM)
	GPIO.setup(HALF, GPIO.OUT)
        GPIO.setup(DIR, GPIO.OUT)
        GPIO.setup(STEP, GPIO.OUT)
        delay = .04
	print("initiating Motors")
	try:
		while True:
			if(objX.value +20 < centerX.value):
				if(objX.value < centerX.value +50  and objX.value > centerX.value -50):
					GPIO.output(HALF, GPIO.HIGH)
					GPIO.output(DIR,CCW)
                                	GPIO.output(STEP, GPIO.HIGH)
                                	sleep(delay)
                                	GPIO.output(STEP, GPIO.LOW)
                                	sleep(delay)
				else:
					GPIO.output(HALF, GPIO.LOW)
        	        		GPIO.output(DIR,CCW)
               				GPIO.output(STEP, GPIO.HIGH)
	                		sleep(delay)
        	        		GPIO.output(STEP, GPIO.LOW)
                			sleep(delay)
        		elif(objX.value -20 > centerX.value):
                		if(objX.value < centerX.value +50  and objX.value > centerX.value -50):
                                        GPIO.output(HALF, GPIO.HIGH)
                                        GPIO.output(DIR,CW)
                                        GPIO.output(STEP, GPIO.HIGH)
                                        sleep(delay)
                                        GPIO.output(STEP, GPIO.LOW)
                                        sleep(delay)
                                else:
					GPIO.output(HALF, GPIO.LOW)
                                        GPIO.output(DIR,CW)
                                        GPIO.output(STEP, GPIO.HIGH)
                                        sleep(delay)
                                        GPIO.output(STEP, GPIO.LOW)
                                        sleep(delay)
	except KeyboardInterrupt:
		GPIO.cleanup()

def servo_driver(objY, centerY):
	GPIO.setmode(GPIO.BCM)
	servo_pin = 17
	GPIO.setup(servo_pin, GPIO.OUT)
	p = GPIO.PWM(servo_pin, 50)
	p.start(2.5)
	position = 5
	try:
		p.ChangeDutyCycle(position)
		sleep(1)
		while True:
			if(objY.value -50 > centerY.value):
				if(position<4):
					position = 5
				p.ChangeDutyCycle(position-.3)
				position = position - .3
				sleep(1.5)

			elif(objY.value +70 < centerY.value):
				if(position > 7):
					position = 5
				p.ChangeDutyCycle(position+.3)
				position = position + .3
				sleep(1.5)
	except KeyboardInterrupt:
		p.stop()
		GPIO.cleanup()

def led_strip(objX, centerX):
	# LED strip configuration:
	LED_COUNT      = 12      # Number of LED pixels.
	LED_PIN        = 12      # GPIO pin connected to the pixels (18 uses PWM!).
	#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/$
	LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
	LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
	LED_BRIGHTNESS = 50     # Set to 0 for darkest and 255 for brightest
	LED_INVERT     = False   # True to invert the signal (when using NPN transistor$
	LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 41, 45 or 53
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(19, GPIO.OUT)	

	# Create NeoPixel object with appropriate configuration.
	strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
	# Intialize the library (must be called once before other functions).
	strip.begin()
	
	try:
		while True:
			for i in range(strip.numPixels()):
				strip.setPixelColor(i, Color(0,0,0))
				strip.show()
			if(objX.value < centerX.value +20  and objX.value > centerX.value -20):
				for i in range(strip.numPixels()):
			        	strip.setPixelColor(i, Color(255,0,0))
        				strip.show()
				GPIO.output(19, GPIO.HIGH)
				sleep(.5)
				GPIO.output(19, GPIO.LOW)
			elif(objX.value < centerX.value +60  and objX.value > centerX.value -60):
				for i in range(strip.numPixels()):
                                        strip.setPixelColor(i, Color(255,255,0))
                                        strip.show()
				sleep(.5)
			else:
				for i in range(strip.numPixels()):
                                        strip.setPixelColor(i, Color(0,255,0))
                                        strip.show()
				sleep(.5)
	except KeyboardInterrupt:
		for i in range(strip.numPixels()):
                                strip.setPixelColor(i, Color(100,0,100))
                                strip.show()
		GPIO.cleanup()

# check to see if this is the main body of execution
if __name__ == "__main__":
	# construct the argument parser and parse the arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-c", "--cascade", type=str, required=True,
		help="path to input Haar cascade for face detection")
	args = vars(ap.parse_args())
	with Manager() as manager:
		# enable the servos
		#pth.servo_enable(1, True)
		#pth.servo_enable(2, True)
 
		# set integer values for the object center (x, y)-coordinates
		centerX = manager.Value("i", 0)
		centerY = manager.Value("i", 0)
 
		# set integer values for the object's (x, y)-coordinates
		objX = manager.Value("i", 0)
		objY = manager.Value("i", 0)
 

		processLedLights = Process(target=led_strip, args=(objX, centerX))
		processMotorDrive = Process(target=motor_driver, args=(objX, centerX))
		processServoDrive = Process(target=servo_driver, args=(objY, centerY))
		processObjectCenter = Process(target=obj_center, args=(args, objX, objY, centerX, centerY))

		processObjectCenter.start()
		processLedLights.start()
		processMotorDrive.start()
		processServoDrive.start()

		processServoDrive.join()
		processLedLights.join()
		processMotorDrive.join()
		processObjectCenter.join()
















