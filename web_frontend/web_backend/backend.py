from flask import Flask, request, jsonify
import cv2
import numpy as np
import pytesseract
import os
import json
from flask_cors import CORS

# Set up Tesseract path
pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"

app = Flask(__name__)
CORS(app)

def preprocess_image(image_path):
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print("ERROR: Image not loaded correctly!")
        return None

    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    processed = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 11, 3
    )
    return processed

def detect_table_cells(image_path):
    image = preprocess_image(image_path)
    if image is None:
        return []

    edges = cv2.Canny(image, 100, 500)
    kernel = np.ones((3,1), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)

    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cells = [cv2.boundingRect(c) for c in contours if 15 < cv2.boundingRect(c)[2] < 500 and 15 < cv2.boundingRect(c)[3] < 500]
    cells = sorted(cells, key=lambda b: (b[1], b[0]))
    return cells

def extract_text_from_cells(image_path, cells):
    original_image = cv2.imread(image_path)
    extracted_data = []

    for idx, (x, y, w, h) in enumerate(cells):
        cell_image = original_image[y:y+h, x:x+w]
        gray = cv2.cvtColor(cell_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        text = pytesseract.image_to_string(
            gray,
            config="--psm 7 --oem 3 -c tessedit_char_whitelist=YN0123456789"
        )
        text = text.encode("utf-8", "ignore").decode("utf-8").strip().upper()
        text = text.replace("YY", "Y").replace("NN", "N")
        if not text:
            text = "?"
        extracted_data.append(text)
    return extracted_data

def structure_data(data):
    structured_data = {}
    if len(data) < 50:
        return {}

    if len(data) > 50:
        data = data[:50]

    years = list(range(2016, 2016 + 10))
    regions = ["Region 1", "Region 2", "Region 3", "Region 4", "Region 5"]

    for i, year in enumerate(years):
        structured_data[year] = {}
        for j, region in enumerate(regions):
            index = i * 5 + j
            structured_data[year][region] = data[index] in ["Y", "YES"]

    return structured_data

@app.route('/process-table', methods=['POST'])
def process_table():
    try:
        if 'image' not in request.files:
            return jsonify({"error": "No image provided"}), 400

        image_file = request.files['image']
        file_path = "temp.png"
        image_file.save(file_path)

        cells = detect_table_cells(file_path)
        extracted_data = extract_text_from_cells(file_path, cells)
        structured_data = structure_data(extracted_data)

        os.remove(file_path)

        return jsonify({
            "structured": structured_data,
            "raw_ocr": extracted_data,
            "cell_count": len(cells)
        })

    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
