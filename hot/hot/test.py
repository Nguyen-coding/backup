import Jetson.GPIO as GPIO
import time

LED_PIN = 11  # BOARD 11번 (GPIO17)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_PIN, GPIO.OUT)

try:
    while True:
        GPIO.output(LED_PIN, GPIO.HIGH)
        print("LED ON")
        time.sleep(1)

        GPIO.output(LED_PIN, GPIO.LOW)
        print("LED OFF")
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.cleanup()
    print("LED test stopped.")
