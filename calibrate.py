import serial
import time


def read_actuator_lengths():
    try:
        # Adjust the COM port and baud rate according to your Arduino setup
        ser = serial.Serial('COM3', 9600, timeout=1)
        time.sleep(2)  # Wait for serial connection to initialize

        # Request data from Arduino and read response
        ser.write(b'R\n')
        line = ser.readline().decode('utf-8').strip()

        # Split the data into individual actuator readings
        actuator_lengths = line.split(',')
        if len(actuator_lengths) == 6:
            # Convert to integers (or any calibration needed can be applied here)
            actuator_1_length = int(actuator_lengths[0])
            actuator_2_length = int(actuator_lengths[1])
            actuator_3_length = int(actuator_lengths[2])
            actuator_4_length = int(actuator_lengths[3])
            actuator_5_length = int(actuator_lengths[4])
            actuator_6_length = int(actuator_lengths[5])

            return {
                "actuator_1_length": actuator_1_length,
                "actuator_2_length": actuator_2_length,
                "actuator_3_length": actuator_3_length,
                "actuator_4_length": actuator_4_length,
                "actuator_5_length": actuator_5_length,
                "actuator_6_length": actuator_6_length
            }
        else:
            print("Error: Unexpected number of readings from Arduino.")
            return None

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return None
    finally:
        ser.close()


# Example usage
actuator_lengths = read_actuator_lengths()
if actuator_lengths:
    print("Actuator lengths:", actuator_lengths)