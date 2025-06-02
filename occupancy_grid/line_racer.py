import cv2
import cv2.ximgproc as ximgproc
import numpy as np
from scipy.interpolate import splprep, splev

def mask_color(frame):
    # convert frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # blur
    blur_size = 3
    hsv = cv2.GaussianBlur(hsv, (blur_size, blur_size), 0)

    # mask by blue value
    lower_blue = np.array([86, 114, 31])  # Lower bound of H, S, V
    upper_blue = np.array([127, 255, 255]) # Upper bound of H, S, V
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # filter mask
    kernel_size = 2 # Start with a small kernel (3x3 or 5x5)
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    # OPEN to remove small noise spots
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # CLOSE to fill small gaps in the cable
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask


def find_cable(mask):
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    img_height, img_width = mask.shape[:2]

    # 3. Identify Candidate "Seed" Contours at the Bottom
    bottom_region_percentage = 0.15 # Consider bottom 15% of the image
    bottom_y_threshold = img_height * (1 - bottom_region_percentage)

    seeded_candidates = []
    for cnt in contours:
        # Check if any point in the contour is in the bottom region
        is_in_bottom = False
        for point in cnt:
            if point[0][1] >= bottom_y_threshold: # point is [ [[x,y]] ]
                is_in_bottom = True
                break
        
        if is_in_bottom:
            seeded_candidates.append(cnt)

    # 4. Filter Seeded Contours Based on Basic Properties
    MIN_CABLE_AREA = 300       # Adjust: Minimum area for the cable part
    MIN_CABLE_LENGTH = 50      # Adjust: Minimum perimeter for the cable part

    valid_seeded_contours = []
    for cnt in seeded_candidates:
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)

        if area >= MIN_CABLE_AREA and perimeter >= MIN_CABLE_LENGTH:
            valid_seeded_contours.append(cnt)

    # 5. Select the Most Promising Cable Candidate
    cable_contour = None
    max_upward_extension = -1 # We want the smallest min_y, so initialize appropriately or use a different metric

    if not valid_seeded_contours:
        pass
    elif len(valid_seeded_contours) == 1:
        cable_contour = valid_seeded_contours[0]
    else:
        # Criterion: Highest topmost point (smallest y value)
        # Alternative: Longest perimeter, largest area, etc.
        
        best_min_y = img_height # Initialize with the bottom of the image
        
        for cnt in valid_seeded_contours:
            # Get the bounding box to find the topmost y-coordinate
            x, y, w, h = cv2.boundingRect(cnt)
            current_min_y = y # Topmost y of the bounding box
            
            # You could also use np.min(cnt[:,:,1]) for the absolute topmost point of the contour itself
            # current_min_y_contour = np.min(cnt[:, :, 1])


            # Let's use a score: prioritize height (upward extension) and then perimeter
            # Score = (img_height - current_min_y) * some_weight + cv2.arcLength(cnt, True)
            # For simplicity now, let's just use topmost point.
            if current_min_y < best_min_y:
                best_min_y = current_min_y
                cable_contour = cnt


    # 6. Refine with Shape Characteristics (OPTIONAL - apply to 'cable_contour' if needed)
    if cable_contour is not None:
        # Example: Check extent of the chosen cable
        area_final = cv2.contourArea(cable_contour)
        rect_final = cv2.minAreaRect(cable_contour)
        (x_f,y_f), (w_f,h_f), ang_f = rect_final
        bbox_area_final = w_f * h_f
        if bbox_area_final > 0:
            extent_final = area_final / bbox_area_final
            MIN_FINAL_EXTENT = 0.2 # Example threshold

    return cable_contour


def find_skeleton(cable_contour, mask):
    if cable_contour is not None and len(cable_contour) > 0:
        filled_cable_mask = np.zeros(mask.shape[:2], dtype=np.uint8)
        cv2.drawContours(filled_cable_mask, [cable_contour], -1, (255), thickness=cv2.FILLED)
        # cv2.imshow("1. Filled Cable Mask", filled_cable_mask) # Keep for debugging

        try:
            skeleton = ximgproc.thinning(filled_cable_mask, thinningType=ximgproc.THINNING_ZHANGSUEN)
            return skeleton
            # cv2.imshow("2. Skeleton (Thinned)", skeleton) # Keep for debugging
        except AttributeError:
            print("cv2.ximgproc.thinning not available. Is opencv-contrib-python installed correctly?")
        except Exception as e:
            print(f"Error during thinning: {e}")

    return None


cap = cv2.VideoCapture(0)
while True:
    _, frame = cap.read()

    # Resize frame for processing
    processing_width = 640
    original_height, original_width = frame.shape[:2]
    scale_factor = processing_width / original_width
    processing_height = int(original_height * scale_factor)
    frame = cv2.resize(frame, (processing_width, processing_height), interpolation=cv2.INTER_AREA)

    mask = mask_color(frame)
    # cv2.imshow("dbg", mask)
    cable = find_cable(mask)
    skeleton = find_skeleton(cable, mask)

    if skeleton is not None:
        cv2.imshow("dbg", skeleton)

    if cv2.waitKey(1) == ord('q'): # Press 'q' to quit
        break
