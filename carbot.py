import RPi.GPIO as GPIO
import numpy as np
import time
import zmq
import picamera
import math
import struct
from io import BytesIO

from enum import Enum
from abc import ABC, abstractmethod
from typing import Tuple
from multiprocessing import Process

GPIO_FREQUENCY = 50
GPIO_PERIOD = 1 / GPIO_FREQUENCY

LEFT_PIN = 12
RIGHT_PIN = 19

CAR_IP = "192.168.0.169"
VIDEO_PORT = 5000
CONTROL_PORT = 5001

STOP_THRESHOLD = 0.1

class CameraStreamer(Process):
    def run(self):
        context = zmq.Context()
        socket = context.socket(zmq.PUB)
        socket.bind(f"tcp://{CAR_IP}:{VIDEO_PORT}")

        camera = picamera.PiCamera()
        camera.resolution = (640, 480)
        time.sleep(2)
        stream = BytesIO()
        for _ in camera.capture_continuous(stream, 'jpeg'):
            stream.seek(0)
            frame = stream.read()
            socket.send_multipart((b"camera", frame))
            stream.seek(0)
            stream.truncate()


class Carbot:
    def __init__(self) -> None:
        GPIO.setmode(GPIO.BCM)
        self.left_servo = FS90R(LEFT_PIN)
        self.right_servo = FS90R(RIGHT_PIN)
        self.servos = [self.left_servo, self.right_servo]
        self.movement = (0, 0)

    def set_movement(self, movement: Tuple[float, float]) -> None:
        self.movement = movement
        self.left_servo.set_movement(movement[0])
        self.right_servo.set_movement(movement[1])

    def run(self) -> None:
        try:
            camera_proc = CameraStreamer()
            camera_proc.start()

            context = zmq.Context()
            socket = context.socket(zmq.SUB)
            socket.bind(f"tcp://{CAR_IP}:{CONTROL_PORT}")
            socket.subscribe("")
            while True:
                raw_data = socket.recv()
                left_move, right_move = struct.unpack(">dd", raw_data)
                print(left_move, right_move)
                self.set_movement((left_move, -right_move))
        finally:
            camera_proc.terminate()
            for servo in self.servos:
                servo.stop()
            GPIO.cleanup()


class Servo(ABC):
    def __init__(self, pin: int) -> None:
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = None
        self.pwm_started = False

    def set_movement(self, direction: float) -> None:
        assert direction >= -1
        assert direction <= 1
        if abs(direction) <= STOP_THRESHOLD and self.pwm_started:
            self.pwm.stop()
            self.pwm = None
            self.pwm_started = False
        elif abs(direction) > STOP_THRESHOLD:
            dc_percent = self.movement_dc_percent(direction)
            if self.pwm_started:
                self.pwm.ChangeDutyCycle(dc_percent)
            else:
                self.pwm = GPIO.PWM(self.pin, GPIO_FREQUENCY)
                self.pwm.start(dc_percent)
                self.pwm_started = True

    @abstractmethod
    def movement_dc_percent(self, direction: float) -> float:
        pass

    def stop(self) -> None:
        if self.pwm_started:
            self.set_movement(0)


class FS90R(Servo):
    def movement_dc_percent(self, direction: float) -> float:
        assert direction >= -1
        assert direction <= 1
        return 7.5 + 2.5 * direction


if __name__ == "__main__":
    Carbot().run()
