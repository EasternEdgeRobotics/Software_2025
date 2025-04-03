from flask import Flask, request, jsonify
import cv2
import numpy as np
import pytesseract
import os
import json
from flask_cors import CORS

#Put in so I can access tesseract here.
pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"

app = Flask(__name__)
CORS(app)

def preprocess_image(image_path):
    print("Loading image...")
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print("ERROR: Image not loaded correctly!")
        return None

    print("Applying preprocessing...")
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    processed = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 3)
    
    cv2.imwrite("debug_preprocessed.png", processed)
    print("Preprocessing complete. Check debug_preprocessed.png")
    
    return processed

def detect_table_cells(image_path):
    print("Detecting table structure...")
    image = preprocess_image(image_path)
    if image is None:
        return []

    edges = cv2.Canny(image, 100, 500)
    kernel = np.ones((3,1), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)
    cv2.imwrite("debug_edges.png", edges)

    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Found {len(contours)} contours.")

    
    cells = [cv2.boundingRect(c) for c in contours if 15 < cv2.boundingRect(c)[2] < 500 and 15 < cv2.boundingRect(c)[3] < 500]
    print(f"Identified {len(cells)} possible table cells.")
    
    cells = sorted(cells, key=lambda b: (b[1], b[0]))  

    debug_image = cv2.imread(image_path)
    for x, y, w, h in cells:
        cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 1)
    cv2.imwrite("debug_detected_cells.png", debug_image)
    
    return cells

def extract_text_from_cells(image_path, cells):
    print("Extracting text from cells...")
    original_image = cv2.imread(image_path)
    extracted_data = []

    for idx, (x, y, w, h) in enumerate(cells):
        cell_image = original_image[y:y+h, x:x+w]
        gray = cv2.cvtColor(cell_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        
        cv2.imwrite(f"debug_cell_{idx}.png", gray)

        text = pytesseract.image_to_string(gray, config="--psm 7 --oem 3 -c tessedit_char_whitelist=YN0123456789")

        text = text.encode("utf-8", "ignore").decode("utf-8").strip().upper()

        
        text = text.replace("YY", "Y").replace("NN", "N")

        if text == "":
            print(f"OCR failed on cell {idx}, adding '?' placeholder")
            text = "?"


       
        if text.strip() == "":
            print(f"Skipping empty cell {idx}")
            continue

        print(f"Extracted from cell {idx}: {text}")
        extracted_data.append(text)

    return extracted_data

def structure_data(data):
    print("Structuring extracted data...")
    structured_data = {}


    if len(data) < 50:
        print(f"ERROR: OCR extracted {len(data)} values, expected at least 50!")
        return {}

    
    if len(data) > 50:
        print(f"Warning: Extracted {len(data)} values, trimming to 50.")
        data = data[:50]

    years = list(range(2016, 2016 + 10))
    regions = ["Region 1", "Region 2", "Region 3", "Region 4", "Region 5"]

    for i, year in enumerate(years):
        structured_data[year] = {}
        for j, region in enumerate(regions):
            index = i * 5 + j
            structured_data[year][region] = data[index] in ["Y", "YES"]

    print("Data structured successfully!")
    return structured_data

@app.route('/process-table', methods=['POST'])
def process_table():
    try:
        if 'image' not in request.files:
            print("ERROR: No image file received.")
            return jsonify({"error": "No image provided"}), 400

        image_file = request.files['image']
        file_path = "temp.png"
        image_file.save(file_path)
        print(f"Image saved as {file_path}")

        cells = detect_table_cells(file_path)
        extracted_data = extract_text_from_cells(file_path, cells)
        structured_data = structure_data(extracted_data)

        os.remove(file_path)

        return jsonify(json.loads(json.dumps(structured_data, ensure_ascii=False)))

    except Exception as e:
        print(f"ERROR: {str(e)}")
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
