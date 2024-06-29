import yaml
import argparse
import json
from pathlib import Path
import shutil

import numpy as np


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input_folder', type=str, help='Path to the input folder with annotated images')
    parser.add_argument('output_folder', type=str, help='Path to the output folder')
    parser.add_argument('classes', type=str, help='Path to the label map file, yaml file with label map')
    args = parser.parse_args()

    input_folder = Path(args.input_folder)
    output_folder = Path(args.output_folder)
    classes = Path(args.classes)

    print(f'Input folder: {input_folder.absolute()}')
    print(f'Output folder: {output_folder.absolute()}')

    image_path = Path().cwd() / output_folder / 'images'
    label_path = Path().cwd() / output_folder / 'labels'
    classes_map = Path().cwd() / classes

    labels = {}


    with open(classes_map) as stream:
        lines = stream.readlines()
        for i,line in enumerate(lines):
            labels[line.strip()] = i
    # Find all json files in the input folder and its subfolders
    json_files = input_folder.glob('**/*.json')

    # Iterate over each json file
    for json_file in json_files:
        # Open the json file
        with open(json_file) as file:
            data = json.load(file)
            img_height = data["imageHeight"]
            img_width = data["imageWidth"]

                 # Create a txt file in output path with split
            txt_name = json_file.stem + '.txt'
            
            output_txt_file = label_path / txt_name
            output_txt_file.parent.mkdir(parents=True, exist_ok=True)
            with open(output_txt_file, 'w') as txt_file:

                for annotations in data["shapes"]:
                    x1 = annotations["points"][0][0] / img_width
                    y1 = annotations["points"][0][1] / img_height
                    x2 = annotations["points"][1][0] / img_width
                    y2 = annotations["points"][1][1] / img_height
                    label = annotations["label"]

                    if (label.startswith('cone')):
                        print(label)

                    if label not in labels.keys():
                        raise Exception(f'Label {label} not in label map.')
                    centerX = (x1 + x2) / 2
                    centerY = (y1 + y2) / 2
                    width = abs(x2 - x1)
                    height = abs(y2 - y1)
                    txt_file.write(f'{labels[label]} {centerX} {centerY} {width} {height}\n')
                    # Copy image to output path with split
            image_name = json_file.stem + '.png'
            image_file = json_file.parent / image_name
            output_image_file = image_path / image_name
            output_image_file.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(image_file, output_image_file)