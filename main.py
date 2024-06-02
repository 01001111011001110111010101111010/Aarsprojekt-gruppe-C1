import time
import board
import busio
import digitalio
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import json
from adafruit_rfm9x import RFM9x
from adafruit_matrixkeypad import Matrix_Keypad
from RPLCD.gpio import CharLCD
from pwm_contrast import LCDContrastPWM
from threading import Thread

TRANSISTOR_PIN = 15
RADIO_FREQ_MHZ = 868.0
CONTRAST_PIN = 12
CONTRAST_LEVEL = 200
MQTT_BROKER = "20.13.148.230"
MQTT_PORT = 1883
MQTT_TOPIC_SUBSCRIBE = "enhed1"
MQTT_TOPIC_PUBLISH = "enhed1"

order_prefix = "K1, "

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRANSISTOR_PIN, GPIO.OUT)
pwm = GPIO.PWM(TRANSISTOR_PIN, 1000)
pwm.start(0)

CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

try:
    rfm9x = RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)
    print("LoRa modul fundet")
except RuntimeError as e:
    print("Kunne ikke finde LoRa modul:", e)
    exit(1)

lcd = CharLCD(cols=20, rows=4, pin_rs=26, pin_e=19, pins_data=[13, 6, 24, 8], numbering_mode=GPIO.BCM)
pwm_control = LCDContrastPWM(pin=CONTRAST_PIN, contrast_level=CONTRAST_LEVEL)
pwm_control.set_contrast()

rows = [digitalio.DigitalInOut(board.D27), digitalio.DigitalInOut(board.D17), digitalio.DigitalInOut(board.D18), digitalio.DigitalInOut(board.D22)]
cols = [digitalio.DigitalInOut(board.D4), digitalio.DigitalInOut(board.D3), digitalio.DigitalInOut(board.D2)]

for row in rows:
    row.direction = digitalio.Direction.INPUT
    row.pull = digitalio.Pull.UP

for col in cols:
    col.direction = digitalio.Direction.OUTPUT
    col.value = True

keys = [
    ['1', '2', '3'],
    ['4', '5', '6'],
    ['7', '8', '9'],
    ['*', '0', '#']
]

keypad = Matrix_Keypad(rows, cols, keys)
input_buffer = []

def stay_awake_lcd(last_input_time):
    while True:
        if time.time() - last_input_time[0] < 10:
            pwm.ChangeDutyCycle(100)
        else:
            pwm.ChangeDutyCycle(0)
        time.sleep(1)

def send_lora_data(message):
    try:
        rfm9x.send(bytes(message, 'utf-8'))
        print("Sendt:", message)
    except Exception as e:
        print("Fejl ved afsendelse:", e)

def update_lcd(buffer):
    lcd.cursor_pos = (1, 6)
    lcd.write_string(' ' * 14)
    lcd.cursor_pos = (1, 6)
    lcd.write_string(''.join(buffer))

def on_mqtt_connect(client, userdata, flags, rc):
    client.subscribe(MQTT_TOPIC_SUBSCRIBE)

def on_mqtt_message(client, userdata, msg):
    global order_prefix
    message = msg.payload.decode()
    if message == "mad":
        order_prefix = "K1, "
    elif message == "drink":
        order_prefix = "B1, "
    print(f"Received message '{message}' on topic '{msg.topic}'")

def send_mqtt_data(client, message):
    try:
        client.publish(MQTT_TOPIC_PUBLISH, message)
        print("MQTT sendt:", message)
    except Exception as e:
        print("MQTT fejl ved afsendelse:", e)

def send_order_status(client, order_type):
    try:
        message = json.dumps({"ordre": order_type})
        client.publish(MQTT_TOPIC_PUBLISH, message)
        print("Ordre status sendt:", message)
    except Exception as e:
        print("Fejl i afsending af order status:", e)

lcd.write_string('Choose table')
lcd.cursor_pos = (1, 0)
lcd.write_string('Table: ')

last_input_time = [time.time()]
stay_awake_thread = Thread(target=stay_awake_lcd, args=(last_input_time,))
stay_awake_thread.daemon = True
stay_awake_thread.start()

mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_mqtt_connect
mqtt_client.on_message = on_mqtt_message

try:
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()
except Exception as e:
    print("Kunne ikke forbinde til MQTT broker:", e)
    exit(1)

try:
    while True:
        pressed_keys = keypad.pressed_keys
        if pressed_keys:
            last_input_time[0] = time.time()
            for key in pressed_keys:
                if key == '*':
                    if input_buffer:
                        input_buffer.pop()
                elif key == '#':
                    message = order_prefix + ''.join(input_buffer)
                    send_lora_data(message)
                    send_mqtt_data(mqtt_client, message)
                    if order_prefix == "K1, ":
                        send_order_status(mqtt_client, "mad")
                    elif order_prefix == "B1, ":
                        send_order_status(mqtt_client, "drink")
                    input_buffer = []
                else:
                    input_buffer.append(key)
                update_lcd(input_buffer)
        time.sleep(0.2)
except KeyboardInterrupt:
    lcd.close(clear=True)
    pwm_control.cleanup()
    GPIO.cleanup()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()