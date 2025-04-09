import time
import math
import cv2
from pyfirmata2 import Arduino
from ultralytics import YOLO

class ProstheticController:
    def __init__(self, arduino_port, servo_pins, model_path):
        # Try Connection to Hardware setup
        try:
            self.board = Arduino(arduino_port)
            self.servos = {name: self.board.get_pin(f'd:{pin}:s') for name, pin in servo_pins.items()}
        except Exception as e:
            raise RuntimeError(f"Arduino initialization failed: {e}")

        # AI model used
        self.model = YOLO(model_path)
        self.class_names = self.model.names

        # Grip configuration
        self.grip_strength_map = {"hand": 8, "phone": 10, "can": 10}
        self.last_grip_time = time.time()
        self.min_grip_duration = 3  # seconds
        self.grip_range_threshold = 200  # pixels

    def grip(self, strength: int) -> None:
        """Control all servos with scaled strength (0-180 degrees)."""
        for servo in self.servos.values():
            servo.write(strength)

    @staticmethod
    def calculate_distance(box1, box2):
        """Calculate distance between box centers."""
        #For Prosthetic ((x1 + y1) / 2, (x2 + y2) /2)
        center1 = ((box1[0] + box1[2]) / 2, (box1[1] + box1[3]) / 2)

        #For Other Object ((x1 + y1) / 2, (x2 + y2) /2)
        center2 = ((box2[0] + box2[2]) / 2, (box2[1] + box2[3]) / 2)

        #Distance formula
        return math.sqrt((center1[0] - center2[0])**2 + (center1[1] - center2[1])**2)

    def find_closest_object(self, results, reference_class: str = "prosthetic"):
        """Find the closest object to the reference and return grip strength."""
        ref_box = None

        # Locate reference object (prosthetic)
        for box in results[0].boxes:
            if box.names[int(box.cls)] == reference_class:
                ref_box = box.xyxy[0].tolist()
                break
                
        if not ref_box:
            return 0, None

        # Find closest target object
        closest_obj = None
        min_distance = float('inf')

        for box in results[0].boxes:
            obj_class = self.class_names[int(box.cls[0])]
            if obj_class == reference_class:
                continue

            distance = self.calculate_distance(ref_box, box.xyxy[0].tolist())
            if distance < self.grip_range_threshold and distance < min_distance:
                min_distance = distance
                closest_obj = obj_class

        grip_strength = self.grip_strength_map.get(closest_obj, 0)
        return grip_strength * 18, ref_box  # Scale to servo range

    def process_frame(self, frame):
        """Run detection and return annotated frame + grip strength."""
        results = self.model.predict(frame, conf=0.4)
        grip_strength, _ = self.find_closest_object(results)

        # Update grip if needed
        current_time = time.time()
        if grip_strength > 0 and (current_time - self.last_grip_time) >= self.min_grip_duration:
            self.grip(grip_strength)
            self.last_grip_time = current_time
        elif grip_strength == 0:
            self.grip(0)

        # Annotate frame
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            label = f"{self.class_names[int(box.cls[0])]}: {box.conf[0]:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        cv2.putText(frame, f"Grip: {grip_strength}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return frame, grip_strength

def main():
    # Custom Configuration
    config = {
        'arduino_port': '/dev/cu.usbmodem101',
        'servo_pins': {'thumb': 13, 'index': 11, 'middle': 9},
        'model_path': '/Users/hwangjunho/runs/detect/train10'
    }

    try:
        controller = ProstheticController(**config)

        #Connection to Webcam
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame, _ = controller.process_frame(frame)
            cv2.imshow("Prosthetic Control", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"Error: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        controller.grip(0)

if __name__ == "__main__":
    main()