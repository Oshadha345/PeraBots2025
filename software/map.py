import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
import sys

def process_slam_map(input_file="final_slam_map.npy", output_prefix="processed_map"):
    """
    Process the SLAM map to extract smooth borders and features
    
    Args:
        input_file: Path to the .npy file containing the SLAM map
        output_prefix: Prefix for output filenames
    
    Returns:
        processed_map: The processed binary map
        contours: The extracted contours/borders
    """
    print(f"Loading SLAM map from {input_file}...")
    
    # Load the SLAM map (probability grid)
    try:
        slam_map = np.load(input_file)
        print(f"Map loaded. Shape: {slam_map.shape}, Value range: {slam_map.min()} to {slam_map.max()}")
    except Exception as e:
        print(f"Error loading map: {e}")
        return None, None
    
    # Convert probabilities to binary map (1 = occupied, 0 = free space)
    # We use 0.3 as a threshold - values < 0.3 are considered free space
    # Values > 0.7 are considered occupied, values in between are uncertain
    binary_map = np.zeros_like(slam_map)
    binary_map[slam_map < 0.3] = 1  # Free space = 1 (white)
    binary_map[slam_map > 0.7] = 0  # Obstacles = 0 (black)
    
    # Convert to uint8 for OpenCV operations
    binary_map_uint8 = (binary_map * 255).astype(np.uint8)
    
    # Apply morphological operations to clean up the map
    kernel = np.ones((5, 5), np.uint8)
    cleaned_map = cv2.morphologyEx(binary_map_uint8, cv2.MORPH_OPEN, kernel)
    cleaned_map = cv2.morphologyEx(cleaned_map, cv2.MORPH_CLOSE, kernel)
    
    # Apply Gaussian blur for smoother borders
    smoothed_map = cv2.GaussianBlur(cleaned_map, (5, 5), 0)
    
    # Threshold again to get binary map with smooth borders
    _, smooth_binary = cv2.threshold(smoothed_map, 127, 255, cv2.THRESH_BINARY)
    
    # Find contours (borders)
    contours, _ = cv2.findContours(255 - smooth_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter out small contours (noise)
    significant_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]
    
    # Create a visualization of the contours
    contour_img = np.zeros((smooth_binary.shape[0], smooth_binary.shape[1], 3), dtype=np.uint8)
    cv2.drawContours(contour_img, significant_contours, -1, (0, 255, 0), 2)
    
    # Save the processed maps
    cv2.imwrite(f"{output_prefix}_binary.png", smooth_binary)
    cv2.imwrite(f"{output_prefix}_contours.png", contour_img)
    
    # Also save as numpy array for future use
    np.save(f"{output_prefix}_binary.npy", smooth_binary / 255.0)
    
    # Visualize results
    plt.figure(figsize=(15, 10))
    
    plt.subplot(221)
    plt.title("Original SLAM Map (Probability)")
    plt.imshow(slam_map, cmap='gray', origin='lower')
    plt.colorbar()
    
    plt.subplot(222)
    plt.title("Binary Map")
    plt.imshow(binary_map, cmap='gray', origin='lower')
    
    plt.subplot(223)
    plt.title("Smoothed Map")
    plt.imshow(smooth_binary, cmap='gray', origin='lower')
    
    plt.subplot(224)
    plt.title("Extracted Contours")
    plt.imshow(contour_img, origin='lower')
    
    plt.tight_layout()
    plt.savefig(f"{output_prefix}_visualization.png")
    plt.show()
    
    print(f"Processing complete. Files saved with prefix '{output_prefix}'")
    print(f"Found {len(significant_contours)} significant contours/borders")
    
    return smooth_binary / 255.0, significant_contours

def extract_obstacle_and_walls(binary_map, min_size=50):
    """
    Extract obstacles and walls from the binary map
    
    Args:
        binary_map: Binary map (0 = occupied, 1 = free space)
        min_size: Minimum area for an object to be considered an obstacle
    
    Returns:
        obstacles: List of obstacle contours
        walls: List of wall contours
    """
    # Convert to uint8 for OpenCV
    map_for_contours = (1 - binary_map).astype(np.uint8) * 255
    
    # Find all contours
    contours, _ = cv2.findContours(map_for_contours, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    # Separate into obstacles and walls based on size and shape
    obstacles = []
    walls = []
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_size:
            continue  # Too small to be significant
            
        # Calculate contour properties
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
            
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        
        # More circular shapes are likely obstacles, elongated shapes are walls
        if circularity > 0.5:
            obstacles.append(cnt)
        else:
            walls.append(cnt)
    
    return obstacles, walls

if __name__ == "__main__":
    # Use command line argument for input file if provided
    input_file = sys.argv[1] if len(sys.argv) > 1 else "final_slam_map.npy"
    
    # Process the map
    processed_map, contours = process_slam_map(input_file)
    
    if processed_map is not None:
        # Extract obstacles and walls
        obstacles, walls = extract_obstacle_and_walls(processed_map)
        
        # Visualize obstacles and walls separately
        result_img = np.zeros((processed_map.shape[0], processed_map.shape[1], 3), dtype=np.uint8)
        
        # Draw the binary map as background (grayscale)
        result_img[processed_map > 0.5] = [200, 200, 200]  # Free space as light gray
        
        # Draw obstacles in red
        cv2.drawContours(result_img, obstacles, -1, (0, 0, 255), 2)
        
        # Draw walls in blue
        cv2.drawContours(result_img, walls, -1, (255, 0, 0), 2)
        
        # Save the result
        cv2.imwrite("map_with_features.png", result_img)
        
        # Display information
        print(f"Found {len(obstacles)} obstacles and {len(walls)} walls")
        print(f"Visualization saved as map_with_features.png")