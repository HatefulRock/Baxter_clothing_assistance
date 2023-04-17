import os
import json
import shutil
import random
from tqdm import tqdm


data_dir = "data/images/train"
annos_dir = "data/images/train/annos"
images_dir = "data/images/train/image"

# Define the categories and the desired number of images per category
categories = {
    1: "short sleeve top",
    2: "long sleeve top",
    3: "short sleeve outwear",
    4: "long sleeve outwear",
    5: "vest",
    6: "sling",
    7: "shorts",
    8: "trousers",
    9: "skirt",
    10: "short sleeve dress",
    11: "long sleeve dress",
    12: "vest dress",
    13: "sling dress",
}
num_images_per_category = 1500 // len(categories)

# Create a directory to store the balanced dataset
output_dir = "./balanced_dataset"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Create directories for the images and annotations
output_images_dir = os.path.join(output_dir, "images")
output_annos_dir = os.path.join(output_dir, "annos")
if not os.path.exists(output_images_dir):
    os.makedirs(output_images_dir)
if not os.path.exists(output_annos_dir):
    os.makedirs(output_annos_dir)

# Loop over the categories and copy the desired number of images to the output directory
for category_id, category_name in categories.items():
    # Get a list of all the images in this category
    category_image_paths = []
    for filename in tqdm(os.listdir(images_dir)):
        if filename.endswith(".jpg"):
            json_filename = filename.replace(".jpg", ".json")
            json_path = os.path.join(annos_dir, json_filename)
            with open(json_path, "r") as f:
                json_data = json.load(f)
            # Check if the image contains an item of this category
            contains_item = False
            if json_data["item1"]["category_name"] == category_name:
                contains_item = True
            if contains_item:
                category_image_paths.append(os.path.join(images_dir, filename))
                if len(category_image_paths) >= num_images_per_category:
                    break
    
    # Shuffle the list of images and copy the desired number of images and annotations to the output directory
    random.shuffle(category_image_paths)
    for i in tqdm(range(num_images_per_category)):
        image_path = category_image_paths[i]
        shutil.copy(image_path, os.path.join(output_images_dir, os.path.basename(image_path)))
        json_filename = os.path.basename(image_path).replace(".jpg", ".json")
        json_path = os.path.join(annos_dir, json_filename)
        shutil.copy(json_path, os.path.join(output_annos_dir, json_filename))
