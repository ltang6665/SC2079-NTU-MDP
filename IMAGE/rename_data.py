from pathlib import Path
import shutil

SOURCE_DIR = Path("MDP Project.cleaned")
OUTPUT_DIR = Path("MDP Project.rename.dataset")

SPLITS = ["train", "valid", "test"]


CLASS_MAP = {
    **{i: i - 3 for i in range(3, 33)},
    33: 30,
}


CLASS_NAMES = [str(i) for i in range(11, 41)] + ["45"]

IMAGE_EXTENSIONS = {
    ".jpg", ".jpeg", ".png", ".bmp", ".webp"
}


def convert_label_line(line):

    parts = line.split()

    if len(parts) < 5:
        return None

    try:
        old_class = int(parts[0])
        coords = [float(x) for x in parts[1:]]
    except ValueError:
        return None


    if old_class not in CLASS_MAP:
        return None


    if len(coords) == 4:
        cx, cy, width, height = coords


    elif len(coords) >= 6 and len(coords) % 2 == 0:
        xs = coords[0::2]
        ys = coords[1::2]

        if len(xs) < 3:
            return None

        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)

        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2
        width = x_max - x_min
        height = y_max - y_min

    else:
        return None


    values = [cx, cy, width, height]
    if not all(0 <= v <= 1 for v in values):
        return None
    if width <= 0 or height <= 0:
        return None

    new_class = CLASS_MAP[old_class]
    return (
        f"{new_class} "
        f"{cx:.6f} {cy:.6f} {width:.6f} {height:.6f}"
    )


def prepare_split(split):
    images_dir = SOURCE_DIR / split / "images"
    labels_dir = SOURCE_DIR / split / "labels"

    if not images_dir.exists() or not labels_dir.exists():
        raise FileNotFoundError(
            f"Expected image and label folders for split '{split}'."
        )

    output_images = OUTPUT_DIR / split / "images"
    output_labels = OUTPUT_DIR / split / "labels"
    output_images.mkdir(parents=True, exist_ok=True)
    output_labels.mkdir(parents=True, exist_ok=True)

    kept = 0
    skipped = 0

    for image_path in images_dir.iterdir():
        if image_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue

        label_path = labels_dir / f"{image_path.stem}.txt"

        if not label_path.exists():
            skipped += 1
            continue

        converted = []

        for line in label_path.read_text().splitlines():
            if not line.strip():
                continue

            result = convert_label_line(line)
            if result is not None:
                converted.append(result)


        if not converted:
            skipped += 1
            continue

        shutil.copy2(image_path, output_images / image_path.name)
        (output_labels / label_path.name).write_text(
            "\n".join(converted) + "\n"
        )
        kept += 1

    print(f"{split}: kept {kept} images, skipped {skipped}")


def main():
    if not SOURCE_DIR.exists():
        raise FileNotFoundError(f"Dataset not found: {SOURCE_DIR}")

    if OUTPUT_DIR.exists():
        raise FileExistsError(
            f"{OUTPUT_DIR} already exists. Rename it or choose a new "
            "OUTPUT_DIR to avoid mixing old and new prepared files."
        )

    for split in SPLITS:
        prepare_split(split)

    names_yaml = ", ".join(f"'{name}'" for name in CLASS_NAMES)

    yaml_text = (
        f"path: {OUTPUT_DIR.resolve().as_posix()}\n"
        "train: train/images\n"
        "val: valid/images\n"
        "test: test/images\n\n"
        f"nc: {len(CLASS_NAMES)}\n"
        f"names: [{names_yaml}]\n"
    )

    (OUTPUT_DIR / "data.yaml").write_text(yaml_text)

    print(f"\nPrepared dataset: {OUTPUT_DIR}")
    print(f"Classes: {len(CLASS_NAMES)}")
    print("data.yaml created successfully.")


if __name__ == "__main__":
    main()
