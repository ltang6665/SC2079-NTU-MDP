from ultralytics import YOLO

model = YOLO("yolov8n.pt")

model.train(
    data="MDP Project.clean/data.yaml",
    epochs=100,
    imgsz=640,
    batch=16,
    workers=4,
    project="runs/detect",
    name="mdp_yolov8n",
)
