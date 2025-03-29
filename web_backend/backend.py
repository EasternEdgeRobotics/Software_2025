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
    """Preprocess the image to enhance OCR accuracy."""
    print(" Loading image...")
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(" ERROR: Image not loaded correctly!")
        return None

    print(" Applying preprocessing...")
    processed = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)


    cv2.imwrite("debug_preprocessed.png", processed)
    print(" Preprocessing complete. Saved debug_preprocessed.png")

    return processed

def detect_table_cells(image_path):
    """Detect table cells and extract their bounding boxes."""
    print(" Detecting table structure...")
    image = preprocess_image(image_path)
    if image is None:
        return []

    
    edges = cv2.Canny(image, 50, 150)
    cv2.imwrite("debug_edges.png", edges)  # Save for debugging

    
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f" Found {len(contours)} contours.")

   
    cells = [cv2.boundingRect(c) for c in contours if cv2.boundingRect(c)[2] > 20 and cv2.boundingRect(c)[3] > 20]
    print(f" Identified {len(cells)} possible table cells.")


    cells = sorted(cells, key=lambda b: (b[1] // 10, b[0]))

    if len(cells) == 0:
        print(" ERROR: No table cells detected!")

    return cells

def extract_text_from_cells(image_path, cells):
    """Extract text from each detected table cell."""
    print(" Extracting text from cells...")
    original_image = cv2.imread(image_path)
    extracted_data = []

    for idx, (x, y, w, h) in enumerate(cells):
        cell_image = original_image[y:y+h, x:x+w]
        gray = cv2.cvtColor(cell_image, cv2.COLOR_BGR2GRAY)

        
        cv2.imwrite(f"debug_cell_{idx}.png", gray)

        # Performing the OCR
        text = pytesseract.image_to_string(gray, config="--psm 6 --oem 3 -c tessedit_char_whitelist=YyNn0123456789").strip().upper()
        print(f" Extracted from cell {idx}: {text}")

        extracted_data.append(text if text else "UNKNOWN")

    if len(extracted_data) == 0:
        print(" ERROR: OCR extracted no text!")

    return extracted_data

def structure_data(data):
    """Structure extracted OCR data into a dictionary format."""
    print(" Structuring extracted data...")
    structured_data = {}

    if len(data) < 6:
        print(" ERROR: Not enough data extracted!")
        return {}

    years = list(range(2016, 2016 + (len(data) // 6)))
    regions = ["Region 1", "Region 2", "Region 3", "Region 4", "Region 5"]

    for i, year in enumerate(years):
        structured_data[year] = {}
        for j in range(len(regions)):
            index = i * 6 + j + 1
            structured_data[year][regions[j]] = data[index] in ["Y", "YES"]

    print(" Data structured successfully!")
    return structured_data

@app.route('/process-table', methods=['POST'])
def process_table():
    try:
        if 'image' not in request.files:
            print(" ERROR: No image file received.")
            return jsonify({"error": "No image provided"}), 400

        image_file = request.files['image']
        file_path = "temp.png"
        image_file.save(file_path)
        print(f" Image saved as {file_path}")

        cells = detect_table_cells(file_path)
        extracted_data = extract_text_from_cells(file_path, cells)
        structured_data = structure_data(extracted_data)

        os.remove(file_path)

        return jsonify(structured_data)

    except Exception as e:
        print(f" ERROR: {str(e)}")
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
