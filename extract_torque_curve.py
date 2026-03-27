"""
chart_digitizer.py
------------------
Automatically detects axis ranges from a chart image using OCR,
then extracts sample points from a target-coloured curve.

Dependencies:
    pip install opencv-python pytesseract pillow numpy
    sudo apt install tesseract-ocr      # Ubuntu
    brew install tesseract              # macOS
"""

import re
import cv2
import numpy as np
import easyocr


def detect_plot_area(img: np.ndarray) -> dict:
    """
    Find the pixel bounding box of the plot area using column/row
    intensity profiles (robust against dense grid lines).

    Returns dict with keys: x_left, x_right, y_top, y_bottom.
    """
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    inv  = 255 - gray                          # dark pixels → high score

    col_sum = inv.sum(axis=0).astype(float)    # profile along X
    row_sum = inv.sum(axis=1).astype(float)    # profile along Y

    x_left  = int(np.argmax(col_sum))
    x_right = int(np.argmax(col_sum[:x_left])) + int(w * 0.7)
    y_top   = int(np.argmax(row_sum[:int(h * 0.2)]))
    y_bottom= int(np.argmax(row_sum[int(h * 0.7):])) + int(h * 0.7)

    return dict(x_left=x_left, x_right=x_right,
                y_top=y_top,   y_bottom=y_bottom)


def _ocr_axis(img: np.ndarray, regions: dict=None, scale: int = 2) -> list[tuple]:
    """
    Run Tesseract on the full image (upscaled for accuracy).
    Returns list of dicts: {text, cx, cy, conf}
    where cx/cy are centre coordinates in *original* image pixels.
    """
    up   = cv2.resize(img[:, regions["x_left"]:], None, fx=scale, fy=scale,
                      interpolation=cv2.INTER_CUBIC)
    gray = cv2.cvtColor(up, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 0, 255,
                              cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    # cv2.imshow("plot", thresh)
    # cv2.waitKey(0)

    reader = easyocr.Reader(['en']) # this needs to run only once to load the model into memory

    # Extract X axis (Torque)
    x_strip = thresh[regions["y_bottom"]*scale:, :]
    dataX = reader.readtext(x_strip)

    for i, item in enumerate(dataX):
        transformed = tuple([[ [int(regions["x_left"]+x/scale), int(regions["y_bottom"]+y/scale)] for x,y in item[0] ], item[1], item[2]])
        dataX[i] = transformed


    # Extract Y axis (Current)
    y_strip = thresh[regions["y_top"]*scale:regions["y_bottom"]*scale, (regions["x_right"]-regions["x_left"])*scale:]
    dataY = reader.readtext(y_strip, rotation_info=[270])

    for i, item in enumerate(dataY):
        # transform back into original image
        transformed = tuple([[ [int(regions["x_right"]+x/scale), int(regions["y_top"]+y/scale)] for x,y in item[0]], item[1], item[2]])
        dataY[i] = transformed

    return dataX, dataY


def _clean_numeric(text: str) -> float | None:
    """
    Parse a tick label that may include OCR artefacts:
      - leading dash from right-axis prefix   e.g. "- 6.30" → 6.30
      - trailing dash (tick mark artefact)     e.g. "63-"   → 63
      - missing decimal point                  e.g. "150"   → handled by caller
    Returns float or None if not parseable.
    """
    cleaned = re.sub(r"^-\s*", "", text)   # strip leading dash
    cleaned = re.sub(r"-$",    "", cleaned) # strip trailing dash
    cleaned = cleaned.strip()
    try:
        return float(cleaned)
    except ValueError:
        return None


def extract_axis_ticks(img: np.ndarray, plot: dict) -> dict:
    """
    Classify every OCR'd number into one of four axes by its pixel position
    relative to the plot area, then return:

        {
          "x":           [(px_x,  value), ...],   sorted by px_x
          "y_right":     [(px_y,  value), ...],   right axis (Current)
        }

    X-axis: OCR sometimes drops the decimal point for values like 1.50 → "150".
    We correct these by dividing by 100 when the value is implausibly large
    (> 10) for an axis whose other ticks are all < 10.
    """
    tokensX, tokensY = _ocr_axis(img, plot)
    x_right  = plot["x_right"]
    y_bottom = plot["y_bottom"]

    x_ticks, y_ticks = [], []

    for tok in tokensX+tokensY:
        val = _clean_numeric(tok[1])
        if val is None:
            continue
        bbox = tok[0]

        cx, cy = int(bbox[0][0]+(bbox[1][0]-bbox[0][0])/2), int(bbox[1][1]+(bbox[2][1]-bbox[1][1])/2)

        if cy > y_bottom + 5:                         # below plot → X axis
            x_ticks.append((cx, val))
        elif cx > x_right + 5:                        # right of plot → Y-right
            y_ticks.append((cy, val))

    # OCR sometimes drops the decimal point for values like 1.50 → "150".
    # We correct these by dividing by 100 when the value is implausibly large
    # (> 10) for an axis whose other ticks are all < 10.
    small_x = [v for _, v in x_ticks if v <= 10]
    if small_x:
        threshold = max(small_x) * 10
        x_ticks = [(px, v / 100 if v > threshold else v)
                   for px, v in x_ticks]
    
    # First of x_ticks and last of y_ticks should be 0
    x_ticks[0] = (x_ticks[0][0], "0.0")
    y_ticks[-1] = (y_ticks[-1][0], "0.0")

    return {
        "x": sorted(x_ticks, key=lambda t: t[0]),
        "y": sorted(y_ticks, key=lambda t: t[0]),
    }



def calibrate_axis(ticks: list[tuple]) -> dict:
    """
    Fit a linear mapping  value = a * pixel + b  through the tick positions.
    Requires at least 2 ticks.

    Returns:
        {
          "min":          float,
          "max":          float,
          "px_to_value":  callable(px) → float,
          "value_to_px":  callable(val) → float,
        }
    """
    if len(ticks) < 2:
        raise ValueError(f"Need ≥2 ticks to calibrate axis, got {len(ticks)}")

    pixels = np.array([t[0] for t in ticks], dtype=float)
    values = np.array([t[1] for t in ticks], dtype=float)
    a, b   = np.polyfit(pixels, values, 1)

    return {
        "min":         float(values.min()),
        "max":         float(values.max()),
        "px_to_value": lambda px:  float(a * px + b),
        "value_to_px": lambda val: int((val - b) / a),
    }


def extract_curve_points(
    img:      np.ndarray,
    plot:     dict,
    x_axis:   dict,
    y_axis:   dict,
    hsv_ranges: list[tuple[np.ndarray, np.ndarray]],
    n_samples: int = 20,
) -> list[tuple[float, float]]:
    """
    Detect a curve by colour, sample one pixel per column inside the plot area,
    convert pixel coords to data coords using pre-calibrated axes.

    hsv_ranges: list of (lower, upper) HSV bounds — use multiple pairs for
                colours that wrap around the hue wheel (e.g. pink/magenta).
    n_samples:  number of evenly-spaced points to return (None = all columns).

    Returns list of (x_value, y_value) tuples.
    """
    hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in hsv_ranges:
        mask |= cv2.inRange(hsv, lo, hi)

    # Clean up noise
    k    = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    x_left   = plot["x_left"]
    x_right  = plot["x_right"]
    y_top    = plot["y_top"]
    y_bottom = plot["y_bottom"]

    raw_points = []
    for px_x in range(x_left, x_right + 1):
        ys = np.where(mask[:, px_x] > 0)[0]
        ys = ys[(ys >= y_top) & (ys <= y_bottom)]
        if len(ys) == 0:
            continue
        px_y = int(np.median(ys))
        x_val = x_axis["px_to_value"](px_x)
        y_val = y_axis["px_to_value"](px_y)
        raw_points.append((round(x_val, 3), round(y_val, 3)))

    if n_samples is None or len(raw_points) <= n_samples:
        return raw_points

    idx = np.linspace(0, len(raw_points) - 1, n_samples, dtype=int)
    return [raw_points[i] for i in idx]


def extract_torque_current_points(img_path: str):
    """
    Main function to extract curve points from a chart image.
    Steps:
        1. Detect plot area
        2. OCR tick labels and classify by axis
        3. Fit linear calibration for each axis
        4. Extract curve points by colour and convert to data coords

    Arguments:
        img_path: path to the chart image file
    Returns:
        points: list of (x_value, y_value) tuples
        x_axis: dict with keys "min", "max", "px_to_value", "value_to_px"
        y_axis: dict with keys "min", "max", "px_to_value", "value_to_px"
    """
    if img_path.startswith("http"):
        from urllib.request import urlopen
        req = urlopen(img_path)
        arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
        img = cv2.imdecode(arr, -1)
    else:
        img  = cv2.imread(img_path)
    if img is None:
        raise FileNotFoundError(f"Could not load {img_path}")

    # Step 1 – locate the plot area
    plot = detect_plot_area(img)
    print("Plot area (pixels):", plot)

    # Step 2 – OCR all tick labels and classify them per axis
    ticks = extract_axis_ticks(img, plot)
    print("\nDetected ticks:")
    for name, t in ticks.items():
        print(f"  {name}: {t}")

    # Step 3 – fit linear calibration for each axis
    x_torque = calibrate_axis(ticks["x"])
    y_current = calibrate_axis(ticks["y"])

    print(f"\nTorque axis: {x_torque['min']} - {x_torque['max']}")
    print(f"Current axis: {y_current['min']} - {y_current['max']}")

    # Step 4 – extract the pink/magenta (Current) curve
    # Pink/magenta wraps around both ends of the HSV hue wheel
    pink_ranges = [
        (np.array([140, 80, 80]),  np.array([180, 255, 255])),
        (np.array([0,   80, 80]),  np.array([10,  255, 255])),
    ]

    points = extract_curve_points(
        img, plot,
        x_axis  = x_torque,
        y_axis  = y_current,
        hsv_ranges = pink_ranges,
        n_samples  = 20,
    )

    return points, x_torque, y_current


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    IMG_PATH = "chart.png"
    img = cv2.imread(IMG_PATH)

    points, x_axis, y_axis = extract_points(IMG_PATH)

    print("\nCurrent curve sample points (Torque N·m → Current A):")
    for torque, current in points:
        print(f"  Torque={torque:.2f} N·m  →  Current={current:.2f} A")
        cv2.circle(img, (x_axis["value_to_px"](torque), y_axis["value_to_px"](current)), 2, (0, 255, 0), -1)

    cv2.imshow("plot", img)
    cv2.waitKey(0)