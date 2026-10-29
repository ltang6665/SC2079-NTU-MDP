from picamera import PiCamera
from time import sleep
import os


counter_file = "/home/pi/MDP/photos/counter.txt"


os.makedirs("/home/pi/MDP/", exist_ok=True)


if os.path.exists(counter_file):
    with open(counter_file, "r") as f:
        try:
            counter = int(f.read().strip())
        except ValueError:
            counter = 21
else:
    counter = 21
    with open(counter_file, "w") as f:
        f.write(str(counter))
print("Success")
camera = PiCamera()
print("Yes")
camera.rotation = 180

while True:
    stro = input("Press Enter to take a photo or type 'e' to quit: ")
    if stro == 'e':
        break

    image_path = f"/home/pi/MDP/photos/image{counter}.jpg"
    camera.start_preview()
    camera.capture(image_path)
    camera.stop_preview()
    print(f"Saved: {image_path}")

    counter += 1
    with open(counter_file, "w") as f:
        f.write(str(counter))
