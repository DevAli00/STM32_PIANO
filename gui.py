import tkinter as tk
import serial
import keyboard

# Initialize serial connection
ser = serial.Serial('COM7', 9600)  # Adjust 'COM7' to the correct port for your system

# Note mapping dictionary
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

# Create the main application window
root = tk.Tk()
root.title("STM32 Piano Controller")

# Piano key dimensions
white_key_width = 30
white_key_height = 150
black_key_width = 20
black_key_height = 100

# Canvas for drawing piano keys
canvas = tk.Canvas(root, width=white_key_width * 14, height=white_key_height)
canvas.pack()

# Function to revert key color after click
def revert_key_color(key, original_color):
    canvas.itemconfig(key, fill=original_color)

# Function to create a key
def create_key(x, y, width, height, color, note_char):
    key = canvas.create_rectangle(x, y, x + width, y + height, fill=color, outline="black")
    def on_click(event):
        send_char(note_char) # Send note to STM32 when key is clicked
        canvas.itemconfig(key, fill='yellow')  # Change color to indicate click
        root.after(100, revert_key_color, key, color)  # Revert color after 100 ms
        keyboard.on_press(lambda event: send_char(event.name)) # Send note to STM32 when key is pressed
    canvas.tag_bind(key, '<Button-1>', on_click)

# Function to send a character to the STM32
def send_char(key_char):
    if key_char in note_mapping:
        ser.write(note_mapping[key_char].encode())

# Map the note_mapping to the visual keys on the GUI
keys = list(note_mapping.keys())

# Create white keys
num_white_keys = 14  # Set to the number of white keys you want to create
for i in range(num_white_keys):
    if i < len(keys):
        create_key(i * white_key_width, 0, white_key_width, white_key_height, 'white', keys[i])

# Create black keys (positions are adjusted for standard piano key pattern)
black_key_positions = [1, 2, 4, 5, 6, 8, 9, 11]
for i, pos in enumerate(black_key_positions):
    if i + num_white_keys < len(keys):
        create_key(pos * white_key_width - black_key_width//2, 0, black_key_width, black_key_height, 'black', keys[num_white_keys + i])



# Start the GUI event loop
root.mainloop()

# Close the serial connection when the application is closed
ser.close()
