from gpiozero import OutputDevice
import time
import threading
import Adafruit_ADS1x15
import Adafruit_GPIO.I2C as I2C
from openpyxl import Workbook
from datetime import datetime

# Motor parameters
# each revolutuion 0.004 m
# distance = 0.004 * revolutions_
desired_velocity_1 = 0.000166667
revolutions_1 = 7.5
steps_per_revolution_1 = 400

desired_velocity_2 = 0.000166667
revolutions_2 = 7.5
steps_per_revolution_2 = 400

desired_velocity_3 = 0.000166667
revolutions_3 = 7.5
steps_per_revolution_3 = 400

desired_velocity_4 = 0.000166667
revolutions_4 = 7.5
steps_per_revolution_4 = 400

# Motor setup
DIR_PIN_1, PUL_PIN_1, EN_PIN_1 = 20, 21, 16
DIR_PIN_2, PUL_PIN_2, EN_PIN_2 = 19, 13, 26
DIR_PIN_3, PUL_PIN_3, EN_PIN_3 = 5, 0, 6
DIR_PIN_4, PUL_PIN_4, EN_PIN_4 = 7, 8, 1

direction_pins = [
    OutputDevice(DIR_PIN_1), OutputDevice(DIR_PIN_2), 
    OutputDevice(DIR_PIN_3), OutputDevice(DIR_PIN_4)
]
pulse_pins = [
    OutputDevice(PUL_PIN_1), OutputDevice(PUL_PIN_2), 
    OutputDevice(PUL_PIN_3), OutputDevice(PUL_PIN_4)
]
enable_pins = [
    OutputDevice(EN_PIN_1), OutputDevice(EN_PIN_2), 
    OutputDevice(EN_PIN_3), OutputDevice(EN_PIN_4)
]

# Enable all motors
for enable_pin in enable_pins:
    enable_pin.off()

# Force sensor setup
I2C.get_default_bus = lambda: 1
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
VREF = 4.096

# Create a new Excel workbook and select the active sheet
wb = Workbook()
ws = wb.active
ws.append(['Time', 'ADC Value 1', 'Voltage 1 (V)', 'Force 1 (N)', 'ADC Value 2', 'Voltage 2 (V)', 'Force 2 (N)'])

# Function to read force sensors
def read_force_sensors(start_time):
    print('| {0:>12} | {1:>12} | {2:>12} | {3:>12} | {4:>12} | {5:>12} | {6:>12} |'.format(
        'Elapsed Time', 'ADC 1', 'Voltage 1', 'Force 1', 'ADC 2', 'Voltage 2', 'Force 2'))
    print('-' * 90)
    
    while not stop_event.is_set():
        elapsed_time = time.time() - start_time
        
        # Read both sensors
        adc_value_1 = adc.read_adc(0, gain=GAIN)
        voltage_1 = (adc_value_1 / 32767.0) * VREF
        force_kg_1 = (voltage_1 / 5.0) * 20
        force_n_1 = force_kg_1 * 9.80665
        
        adc_value_2 = adc.read_adc(1, gain=GAIN)
        voltage_2 = (adc_value_2 / 32767.0) * VREF
        force_kg_2 = (voltage_2 / 5.0) * 20
        force_n_2 = force_kg_2 * 9.80665

        # Print and save the data
        print('| {0:>12.2f} | {1:>12} | {2:>12.4f} | {3:>12.4f} | {4:>12} | {5:>12.4f} | {6:>12.4f} |'.format(
            elapsed_time, adc_value_1, voltage_1, force_n_1, adc_value_2, voltage_2, force_n_2))
        ws.append([elapsed_time, adc_value_1, voltage_1, force_n_1, adc_value_2, voltage_2, force_n_2])
        
        time.sleep(0.5)

# Function to control a stepper motor
def step_motor(steps, direction, delay, motor_num):
    direction_pin = direction_pins[motor_num - 1]
    pulse_pin = pulse_pins[motor_num - 1]

    # Set direction
    if direction:
        direction_pin.on()
    else:
        direction_pin.off()

    for _ in range(steps):
        pulse_pin.on()
        time.sleep(delay)
        pulse_pin.off()
        time.sleep(delay)

# Motor control functions for individual motors
def control_motor_1():
    distance_per_step = 0.004 / steps_per_revolution_1
    delay = 1 / (2 * desired_velocity_1 / distance_per_step)
    steps = round(steps_per_revolution_1 * revolutions_1)
    step_motor(steps, True, delay, 1) #direction

def control_motor_2():
    distance_per_step = 0.004 / steps_per_revolution_2
    delay = 1 / (2 * desired_velocity_2 / distance_per_step)
    steps = round(steps_per_revolution_2 * revolutions_2)
    step_motor(steps, True, delay, 2) #direction

def control_motor_3():
    distance_per_step = 0.004 / steps_per_revolution_3
    delay = 1 / (2 * desired_velocity_3 / distance_per_step)
    steps = round(steps_per_revolution_3 * revolutions_3)
    step_motor(steps, True, delay, 3) #direction

def control_motor_4():
    distance_per_step = 0.004 / steps_per_revolution_4
    delay = 1 / (2 * desired_velocity_4 / distance_per_step)
    steps = round(steps_per_revolution_4 * revolutions_4)
    step_motor(steps, True, delay, 4) #direction

# Combined operation
try:
    start_time = time.time()

    # Start force sensor thread
    stop_event = threading.Event()
    force_sensor_thread = threading.Thread(target=read_force_sensors, args=(start_time,))
    force_sensor_thread.start()

    # Start motor control threads
    motor_1_thread = threading.Thread(target=control_motor_1)
    motor_2_thread = threading.Thread(target=control_motor_2)
    motor_3_thread = threading.Thread(target=control_motor_3)
    motor_4_thread = threading.Thread(target=control_motor_4)

    motor_1_thread.start()
    motor_2_thread.start()
    motor_3_thread.start()
    motor_4_thread.start()

    # Wait for all motors to finish
    motor_1_thread.join()
    motor_2_thread.join()
    motor_3_thread.join()
    motor_4_thread.join()

    # Stop force sensor thread
    stop_event.set()
    force_sensor_thread.join()

    # Save the Excel file
    filename = datetime.now().strftime("%Y%m%d_%H%M%S") + '.xlsx'
    wb.save(filename)
    print(f"\nData saved to {filename}")

except KeyboardInterrupt:
    # Disable all motors
    for enable_pin in enable_pins:
        enable_pin.on()  # High to disable
    stop_event.set()
    force_sensor_thread.join()
    print("Interrupted! Motor drivers disabled.")
