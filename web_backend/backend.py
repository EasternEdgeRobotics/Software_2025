from flask import Flask, request, jsonify
import cv2
import numpy as np
import pytesseract
import os
from flask_cors import CORS

pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"

app = Flask(__name__)
CORS(app)

def preprocess_image(image_path):
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cv2.imwrite("debug_preprocessed.png", binary)
    return binary

def detect_table_cells(image_path):
    image = preprocess_image(image_path)
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(image, kernel, iterations=2)

    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cells = [cv2.boundingRect(c) for c in contours if cv2.boundingRect(c)[2] > 20 and cv2.boundingRect(c)[3] > 20]
    cells = sorted(cells, key=lambda b: (b[1] // 10, b[0]))

    extracted_cells = []
    original_image = cv2.imread(image_path)

    for x, y, w, h in cells:
        cell_image = original_image[y:y+h, x:x+w]
        extracted_cells.append(cell_image)

    return extracted_cells

def extract_text_from_cells(cells):
    extracted_data = []
    for cell in cells:
        gray = cv2.cvtColor(cell, cv2.COLOR_BGR2GRAY)
        text = pytesseract.image_to_string(gray, config="--psm 6 --oem 3 -c tessedit_char_whitelist=YyNn0123456789")
        extracted_data.append(text.strip().upper())

    return extracted_data

def structure_data(data):
    structured_data = {}
    years = list(range(2016, 2016 + (len(data) // 6)))
    regions = ["Region 1", "Region 2", "Region 3", "Region 4", "Region 5"]

    for i, year in enumerate(years):
        structured_data[year] = {}
        for j in range(len(regions)):
            structured_data[year][regions[j]] = data[i * 6 + j + 1] in ["Y", "YES"]

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
        extracted_data = extract_text_from_cells(cells)
        structured_data = structure_data(extracted_data)

        os.remove(file_path)

        return jsonify(structured_data)

    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
