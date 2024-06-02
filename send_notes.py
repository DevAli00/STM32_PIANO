import serial
import keyboard

# Open the serial port
ser = serial.Serial('COM7', 9600)  # Adjust 'COM3' to your actual serial port

note_mapping = {
    'a': 'a',  # Mapping keyboard 'a' to note 'a'
    'z': 'b',  # Mapping keyboard 's' to note 'b'
    'e': 'c',  # Continue mapping as needed
    'r': 'd',
    't': 'e',
    'y': 'f',
    'u': 'g',
    'i': 'h',
    'o': 'i',
    'p': 'j',
    'q': 'k',
    's': 'l',
    'd': 'm',
    'f': 'n',
    'g': 'o',
    'h': 'p',
    'j': 'q',
    'k': 'r',
    'l': 's',
    'm': 't',
    'w': 'u',
    'x': 'v',
    'c': 'w',
    'v': 'x',
}

def send_note_to_stm32(key):
    if key in note_mapping:
        ser.write(note_mapping[key].encode())

keyboard.on_press(lambda event: send_note_to_stm32(event.name))



print("Press ESC to stop.")
keyboard.wait('esc')

ser.close()

