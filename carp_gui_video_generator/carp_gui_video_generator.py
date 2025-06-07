import cv2
import pytesseract
import pandas as pd 
import numpy as np 
import os
import sys

def extract_table_to_csv(image_path, debug = False):

    base, ext = os.path.splitext(image_path)

    # Use webcam instead of image file
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open webcam")
        return
    ret, image = cap.read()
    cap.release()
    if not ret:
        print("Failed to capture image from webcam")
        return

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if debug:
        cv2.imwrite(f"{base}_gray{ext}", gray)

    # Blur the image lightly (3x3 kernal) to remove small imperfrections taht could interfere with the following steps
    blur = cv2.GaussianBlur(gray, (3, 3), 0)

    if debug:
        cv2.imwrite(f"{base}_blur{ext}", blur)
    
    # Turn the image into black and white, where each pixel is considered wrt the mean of the surrounding 15x15 pixels minus a constant
    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY_INV, 17, 4)
    if debug:
        cv2.imwrite(f"{base}_thresh{ext}", thresh)

    # Create elements to use for detecting vertical and horizontal lines
    vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 30))
    horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 1))

    # Detect vertical lines 
    vertical_lines = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, vertical_kernel, iterations=1)
    if debug:
        cv2.imwrite(f"{base}_vertical_lines{ext}", vertical_lines)

    # Detect horizontal lines
    horizontal_lines = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, horizontal_kernel, iterations=1)
    if debug:
        cv2.imwrite(f"{base}_horizontal_lines{ext}", horizontal_lines)

    # Find all horizontal lines (contours)
    horizontal_line_contours, _ = cv2.findContours(horizontal_lines, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Get the smallest bounding rectangle of each contour
    horizontal_line_rectangles = [cv2.boundingRect(c) for c in horizontal_line_contours]

    # Sort the horizontal_line_rectangles lines top-to-bottom, then left-to-right
    horizontal_line_rectangles = sorted(horizontal_line_rectangles, key=lambda b: (b[1], b[0])) 

    if debug:
        horizontal_contour_all_lines_image = image.copy()
        i = 0
        for (x, y, w, h) in horizontal_line_rectangles:
            cv2.rectangle(horizontal_contour_all_lines_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(horizontal_contour_all_lines_image, str(i), (x + w + 10, y + h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            i+=1
        cv2.imwrite(f"{base}_horizontal_lines_numbered{ext}", horizontal_contour_all_lines_image) 

    VERTICAL_SPACING_TOLERANCE = 2

    # Attempt to construct an array of 12 lines that are evenly spaced
    table_cell_horizontal_lines_rectangles = []
    row_lines_required = 12
    row_lines_obtained = 0

    for horizontal_line_rectangle in horizontal_line_rectangles:
        if len(table_cell_horizontal_lines_rectangles) < 2:
            if len(table_cell_horizontal_lines_rectangles) == 1:
                if table_cell_horizontal_lines_rectangles[-1][2] >= horizontal_line_rectangle[2]*0.75 and table_cell_horizontal_lines_rectangles[-1][2] < horizontal_line_rectangle[2]*1.05:
                    table_cell_horizontal_lines_rectangles.append(horizontal_line_rectangle)
                    row_lines_obtained += 1
                else:
                    table_cell_horizontal_lines_rectangles = [horizontal_line_rectangle]
                    row_lines_obtained = 1
            else:
                table_cell_horizontal_lines_rectangles.append(horizontal_line_rectangle)
                row_lines_obtained += 1
                
        else:
            if len(horizontal_line_rectangles) == 3:
                # The first and second rows can have different distances
                difference_in_distance = 0
            else:
                difference_in_distance = (((table_cell_horizontal_lines_rectangles[-2][1]+table_cell_horizontal_lines_rectangles[-2][3]) - (table_cell_horizontal_lines_rectangles[-1][1])) -
                                        ((table_cell_horizontal_lines_rectangles[-1][1]+table_cell_horizontal_lines_rectangles[-1][3]) - horizontal_line_rectangle[1]))
            difference_in_line_width_acceptable = table_cell_horizontal_lines_rectangles[-1][2] >= horizontal_line_rectangle[2]*0.75 and table_cell_horizontal_lines_rectangles[-1][2] < horizontal_line_rectangle[2]*1.05
            if difference_in_distance > VERTICAL_SPACING_TOLERANCE or not difference_in_line_width_acceptable:
                table_cell_horizontal_lines_rectangles = [table_cell_horizontal_lines_rectangles[-1]] 
                row_lines_obtained = 1
            else:
                row_lines_obtained += 1
                table_cell_horizontal_lines_rectangles.append(horizontal_line_rectangle)
        if row_lines_obtained == row_lines_required:
            break

    if not row_lines_obtained == row_lines_required:
        print("Not enought lines")
        return
    
    # Draw contours on the original image for visualization
    if debug:
        horizontal_contour_image = image.copy()
        i = 0
        for (x, y, w, h) in table_cell_horizontal_lines_rectangles:
            cv2.rectangle(horizontal_contour_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(horizontal_contour_image, str(i), (x + w + 10, y + h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            i+=1
        cv2.imwrite(f"{base}_table_cell_row_lines{ext}", horizontal_contour_image)

    # Find all shapes (contours)
    vertical_line_contours, _ = cv2.findContours(vertical_lines, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Get the smallest bounding rectangle of each contour
    vertical_line_rectangles = [cv2.boundingRect(c) for c in vertical_line_contours]

    # Sort the vertical_line_rectangles lines left-to-right, then top-to-bottom
    vertical_line_rectangles = sorted(vertical_line_rectangles, key=lambda b: (b[0], b[1]))  

    # Now that we know the top and bottom horizontal lines, we know the upper and lower bounds of the table.
    table_y_start = table_cell_horizontal_lines_rectangles[0][1]
    table_y_end = table_cell_horizontal_lines_rectangles[-1][1]+table_cell_horizontal_lines_rectangles[-1][3]

    HORIZNOTAL_SPACING_TOLERANCE = 2

    # Attempt to construct an array of 12 lines that are evenly spaced
    table_cell_vertical_lines_rectangles = []
    sequence_of_same_space_vertical_lines_required = 7
    sequence_of_same_space_vertical_lines_detected = 0

    for vertical_line_rectangle in vertical_line_rectangles:
        # Check if this vertical line's center is within the upper and lower bounds of the table
        vertical_center = vertical_line_rectangle[1]+(vertical_line_rectangle[3]/2)
        if not (vertical_center > table_y_start and vertical_center < table_y_end):
            continue

        # Check to see if this vertical line is actually a continuation of another one
        line_combined = False
        vertical_line_horizontal_center = vertical_line_rectangle[0]+(vertical_line_rectangle[2]/2)
        for index, current_line_rectangle in enumerate(table_cell_vertical_lines_rectangles):
            current_line_horizontal_center = current_line_rectangle[0]+(current_line_rectangle[2]/2)
            if abs(current_line_horizontal_center-vertical_line_horizontal_center) < 20:
                # We will consider them to be the same vertical line
                current_line_rectangle_endpoint_x = current_line_rectangle[0]+current_line_rectangle[2]
                current_line_rectangle_endpoint_y = current_line_rectangle[1]+current_line_rectangle[3]
                vertical_line_rectangle_endpoint_x = vertical_line_rectangle[0]+vertical_line_rectangle[2]
                vertical_line_rectangle_endpoint_y = vertical_line_rectangle[1]+vertical_line_rectangle[3]

                new_x = min(current_line_rectangle[0], vertical_line_rectangle[0])
                new_y = min(current_line_rectangle[1], vertical_line_rectangle[1])
                new_width = max(current_line_rectangle_endpoint_x, vertical_line_rectangle_endpoint_x) - new_x
                new_height = max(current_line_rectangle_endpoint_y, vertical_line_rectangle_endpoint_y) - new_y
                
                table_cell_vertical_lines_rectangles[index] = new_x, new_y, new_width, new_height
                line_combined = True
                break
        if line_combined:
            continue
        
        if not sequence_of_same_space_vertical_lines_detected == sequence_of_same_space_vertical_lines_required:
            table_cell_vertical_lines_rectangles.append(vertical_line_rectangle)
            sequence_of_same_space_vertical_lines_detected += 1

    # Draw contours on the original image for visualization
    if debug:
        vertical_contour_image = image.copy()
        i = 0
        for (x, y, w, h) in table_cell_vertical_lines_rectangles:
            cv2.rectangle(vertical_contour_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(vertical_contour_image, str(i), (x + w + 10, y + h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            i+=1
        cv2.imwrite(f"{base}_table_cell_column_lines{ext}", vertical_contour_image)

        # Draw total table
        total_table_image = image.copy()
        i = 0
        for (x, y, w, h) in table_cell_vertical_lines_rectangles + table_cell_horizontal_lines_rectangles:
            cv2.rectangle(total_table_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            i+=1
        cv2.imwrite(f"table_extracted{ext}", total_table_image)

    # Split up table into cells based on the extracted boxes.

    table = [
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        []        
    ]
    # Ensure the output directory exists

    os.makedirs(f"{base}_cells", exist_ok=True)

    for row_number, _ in enumerate(table):
        
        if row_number > 0:

            row_start = table_cell_horizontal_lines_rectangles[row_number][1] + table_cell_horizontal_lines_rectangles[row_number][3]
            row_end = table_cell_horizontal_lines_rectangles[row_number+1][1]

            # Create six columns
            for column_number in range(6):
                if column_number > 0:
                    column_start =  table_cell_vertical_lines_rectangles[column_number][0] + table_cell_vertical_lines_rectangles[column_number][2]
                    column_end = table_cell_vertical_lines_rectangles[column_number+1][0]

                    # Crop the cell from the original image using the calculated start and end positions
                    cell = image[row_start:row_end, column_start:column_end]

                    # # Improve the resolution of the cropped cell using interpolation
                    # scale_factor = 2  # You can adjust this factor as needed
                    # cell = cv2.resize(cell, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_CUBIC)

                    cell_gray = cv2.cvtColor(cell, cv2.COLOR_BGR2GRAY)

                    # Remove 2 pixels from each side of the cell image
                    cell_gray = cell_gray[2:-2, 2:-2]

                    # Apply adaptive thresholding for better handling of varying lighting conditions
                    cell_thresh = cv2.adaptiveThreshold(
                        cell_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv2.THRESH_BINARY, 11, 2
                    )

                    # Remove noise: keep only the largest connected component (biggest black blob)
                    # Invert so blobs are white for connectedComponents
                    cell_inv = cv2.bitwise_not(cell_thresh)
                    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(cell_inv, connectivity=8)
                    if num_labels > 1:
                        # Ignore background (label 0), find largest component
                        largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
                        mask = np.zeros_like(cell_inv)
                        mask[labels == largest_label] = 255
                        # Invert back to original
                        clean_cell = cv2.bitwise_not(mask)
                    else:
                        clean_cell = cell_thresh.copy()

                    # Obtain the text from this cell assuming a unifrom block of text, and remove any whitespace with strip
                    text = pytesseract.image_to_string(cell, config='--psm 6').strip()

                    best_non_affirmative_score = None
                    for template_name in os.listdir("non_affirmative_templates"):
                        template_path = os.path.join("non_affirmative_templates", template_name)
                        template_img = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
                        res = cv2.matchTemplate(clean_cell, template_img, cv2.TM_CCOEFF_NORMED)
                        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                        if best_non_affirmative_score is None or max_val > best_non_affirmative_score:
                            best_non_affirmative_score = max_val

                    best_affirmative_score = None
                    for template_name in os.listdir("affirmative_templates"):
                        template_path = os.path.join("affirmative_templates", template_name)
                        template_img = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
                        res = cv2.matchTemplate(clean_cell, template_img, cv2.TM_CCOEFF_NORMED)
                        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                        if best_affirmative_score is None or max_val > best_affirmative_score:
                            best_affirmative_score = max_val

                    if best_affirmative_score > best_non_affirmative_score:
                        text = "Y"
                    else:
                        text = "N"

                    table[row_number].append(text)
                    if debug:
                        # Save the cropped cell image
                        cv2.imwrite(f"{base}_cells/{row_number}_{column_number}.png", clean_cell)
    
    regions = []
    for i in range(1, 6):
        if os.path.isfile(f"regions/Region{i}.png"):
            region_img = cv2.imread(f"regions/Region{i}.png")
            # Create a mask where red pixels stay red, others become white
            b, g, r = cv2.split(region_img)
            red_mask = (r > 200) & (g < 50) & (b < 50)
            mask = np.ones_like(region_img) * 255  # start with white
            mask[red_mask] = [0, 0, 255]  # set red pixels
            regions.append(mask)


    years = [str(year) for year in range(2016, 2026)]

    for row_number, _ in enumerate(table):
        
        if row_number > 0:
            blank_map = cv2.imread("blank_map.png")
            cv2.putText(
                blank_map,
                years[row_number-1],
                (20, 50),  # x, y position (top left quadrant)
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,       # font scale
                (0, 0, 0), # black color
                3,         # thickness
                cv2.LINE_AA
            )

            # Create six columns
            for column_number in range(5):
                if table[row_number][column_number] == "Y":
                    red_pixels = (regions[column_number][:, :, 2] == 255) & (regions[column_number][:, :, 1] == 0) & (regions[column_number][:, :, 0] == 0)
                    blank_map[red_pixels] = regions[column_number][red_pixels]
            
            # Save the modified map image
            os.makedirs("video_frames", exist_ok=True)
            output_path = os.path.join("video_frames", f"{row_number-1}.png")
            cv2.imwrite(output_path, blank_map)

    image_files = [f"video_frames/{i}.png" for i in range(10)]
    # Read the first image to get frame size
    frame = cv2.imread(image_files[0])
    height, width, layers = frame.shape

    # Define video writer (MP4, 1 fps)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter('output.mp4', fourcc, 1, (width, height))

    for img_file in image_files:
        img = cv2.imread(img_file)
        video.write(img)  # Each image is 1 frame (1 second at 1 fps)
        # Repeat the last frame to make it stay longer
        if img_file == image_files[-1]:
            for _ in range(2):  # Show last frame 2 extra seconds (adjust as needed)
                video.write(img)

    video.release()

    # Print the table nicely
    for row in table:
        print('\t'.join(row))


if __name__ == "__main__":
    debug = False
    if "debug" in sys.argv:
        debug = True
    # Example usage
    extract_table_to_csv("table_image.png", debug)
