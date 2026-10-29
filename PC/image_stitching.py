from pathlib import Path
import cv2
import numpy as np
import math
from datetime import datetime
import random
import logging
logging.basicConfig(level=logging.INFO)

from ultralytics import YOLO
from image_labels import CLASS_IDS

def add_to_stitching_dict(stitching_dict, img_id, conf_level, frame):
    if img_id not in stitching_dict or (
        conf_level > stitching_dict[img_id][0]
    ):

        stitching_dict[img_id] = (
            conf_level,
            frame,
        )
        logging.info(f"Saw {img_id} with confidence level {conf_level}.")


def stitch_images(
    id_arr,
    stitching_dict,
    filename="task",
    tile_size=(320, 320),
    ncols=3,
    pad=8,
    bg=(0, 0, 0),
    show=True,
    label=True,
    topmost=False
):


    if not id_arr:
        print("[stitch] No images to stitch.")
        return None

    ids = [img_id for img_id in id_arr if img_id in stitching_dict]
    if not ids:
        print("[stitch] None of the requested image IDs exist in stitching_dict.")
        return None

    tw, th = int(tile_size[0]), int(tile_size[1])
    n = len(ids)
    nrows = math.ceil(n / ncols)


    canvas_w = ncols * tw + (ncols - 1) * pad
    canvas_h = nrows * th + (nrows - 1) * pad
    canvas = np.full((canvas_h, canvas_w, 3), bg, dtype=np.uint8)


    blank = np.zeros((th, tw, 3), dtype=np.uint8)

    def _resize_tile(img):

        ih, iw = img.shape[:2]
        if iw > tw or ih > th:
            interp = cv2.INTER_AREA
        else:
            interp = cv2.INTER_LINEAR
        return cv2.resize(img, (tw, th), interpolation=interp)

    for idx, img_id in enumerate(ids):
        confidence, frame = stitching_dict.get(img_id, (None, None))

        if frame is None:
            tile = blank.copy()
        else:

            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            tile = _resize_tile(frame)


        if label:
            label_text = img_id if confidence is None else f"{img_id} ({confidence:.2f}) {CLASS_IDS[img_id]}"
            cv2.putText(
                tile, label_text, (8, th - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
            )

        r, c = divmod(idx, ncols)
        y0 = r * (th + pad)
        x0 = c * (tw + pad)
        canvas[y0:y0+th, x0:x0+tw] = tile

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = f"{filename}_collage_{ts}.jpg"
    ok = cv2.imwrite(save_path, canvas)
    if not ok:
        print(f"[stitch] Failed to write {save_path}")
        return None

    if show:
        win = f"collage: {filename}"
        cv2.imshow(win, canvas)
        if topmost:
            try:
                cv2.setWindowProperty(win, cv2.WND_PROP_TOPMOST, 1)
            except cv2.error:
                pass

        cv2.waitKey(1)

    return save_path

def add_to_stitching_dict_2(stitching_dict, obstacle_id, img_id, conf_level, frame):

    if obstacle_id not in stitching_dict:
        stitching_dict[obstacle_id] = {}

    cur = stitching_dict[obstacle_id].get(img_id)
    if (cur is None) or (conf_level > cur[0]):
        stitching_dict[obstacle_id][img_id] = (conf_level, frame)
        logging.info(f"[stitch] Obstacle {obstacle_id} saw {img_id} @ {conf_level:.3f} (updated).")


def stitch_images_2(
    id_arr,
    stitching_dict,
    filename="task",
    tile_size=(320, 320),
    ncols=3,
    pad=8,
    bg=(0, 0, 0),
    show=True,
    label=True,
    topmost=False
):


    pairs = []

    def _best_pair_for_obstacle(oid):

        if oid not in stitching_dict or not stitching_dict[oid]:
            return None
        best_img = max(stitching_dict[oid].items(), key=lambda kv: kv[1][0])[0]
        return (oid, best_img)

    def _best_pair_for_img(img_id):

        best = None
        for oid, inner in stitching_dict.items():
            if img_id in inner:
                confidence = inner[img_id][0]
                if (best is None) or (confidence > best[2]):
                    best = (oid, img_id, confidence)
        return None if best is None else (best[0], best[1])

    for item in id_arr:
        if isinstance(item, tuple) and len(item) == 2:
            oid, iid = item
            if oid in stitching_dict and iid in stitching_dict[oid]:
                pairs.append((oid, iid))
        elif isinstance(item, (str, int)):

            p = _best_pair_for_obstacle(item)
            if p is None:
                p = _best_pair_for_img(item)
            if p is not None:
                pairs.append(p)
        else:

            pass

    if not pairs:
        print("[stitch] No valid (obstacle_id, img_id) pairs to stitch.")
        return None


    tw, th = int(tile_size[0]), int(tile_size[1])
    n = len(pairs)
    nrows = math.ceil(n / ncols)

    canvas_w = ncols * tw + (ncols - 1) * pad
    canvas_h = nrows * th + (nrows - 1) * pad
    canvas = np.full((canvas_h, canvas_w, 3), bg, dtype=np.uint8)

    blank = np.zeros((th, tw, 3), dtype=np.uint8)

    def _resize_tile(img):
        ih, iw = img.shape[:2]
        interp = cv2.INTER_AREA if (iw > tw or ih > th) else cv2.INTER_LINEAR
        return cv2.resize(img, (tw, th), interpolation=interp)


    for idx, (oid, iid) in enumerate(pairs):
        confidence, frame = stitching_dict.get(oid, {}).get(iid, (None, None))

        if frame is None:
            tile = blank.copy()
        else:
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            tile = _resize_tile(frame)

        if label:
            cls = ""
            try:
                cls = f" {CLASS_IDS[iid]}"
            except Exception:
                pass
            label_text = f"Obs {oid} | {iid}"
            if confidence is not None:
                label_text += f" ({confidence:.2f}){cls}"
            cv2.putText(
                tile, label_text, (8, th - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
            )

        r, c = divmod(idx, ncols)
        y0 = r * (th + pad)
        x0 = c * (tw + pad)
        canvas[y0:y0+th, x0:x0+tw] = tile

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = f"{filename}_collage_{ts}.jpg"
    ok = cv2.imwrite(save_path, canvas)
    if not ok:
        print(f"[stitch] Failed to write {save_path}")
        return None

    if show:
        win = f"collage: {filename}"
        cv2.imshow(win, canvas)
        if topmost:
            try:
                cv2.setWindowProperty(win, cv2.WND_PROP_TOPMOST, 1)
            except cv2.error:
                pass
        cv2.waitKey(1)

    return save_path

def test_stitch():
    paths = {
        "IMG1": "image1.jpg",
        "IMG2": "image2.jpg",
        "IMG3": "image3.jpg",
        "IMG4": "image4.jpg",
    }

    frames = {}

    model = YOLO(str(Path(__file__).resolve().parent / "bestv8n.pt"))
    stitching_dict = {}
    for img_id, p in paths.items():
        with open(p, "rb") as f:
            jpg_bytes = f.read()
            frame = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)

            result = model.predict(
                frame, save=False, imgsz=frame.shape[1],
                conf=0.7, verbose=False
            )[0]


            if result is not None:
                names = result.names

                largest_detection = None

                for box in result.boxes:
                    class_index = int(box.cls[0].item())
                    detected_image_id = names[class_index]
                    if str(detected_image_id) in ["45"]:
                        continue

                    w = float(box.xywh[0][2].item())
                    h = float(box.xywh[0][3].item())
                    area = w * h
                    confidence = float(box.conf[0].item()) if getattr(box.conf, "ndim", 0) else float(box.conf.item())

                    if (largest_detection is None or
                        area > largest_detection["area"] or
                        (area == largest_detection["area"] and confidence > largest_detection["conf"])):
                        largest_detection = {"detected_img_id": detected_image_id, "box": box, "area": area, "conf": confidence}

                if largest_detection is not None:
                    detected_image_id = largest_detection["detected_img_id"]
                    detection_confidence = largest_detection["conf"]

                    add_to_stitching_dict(stitching_dict, detected_image_id, detection_confidence, frame)

    id_arr = [id for id in stitching_dict.keys()]

    save_path = stitch_images(
        id_arr=id_arr,
        stitching_dict=stitching_dict,
        filename="task1",


    )
    print("Saved collage to:", save_path)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_stitch()
