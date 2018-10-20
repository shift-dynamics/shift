#!/usr/bin/env python
from PIL import Image
import os

# files = [f for f in os.listdir('.') if os.path.isfile(os.path.join('.', f))]
# for filename in files:
#     if ".png" in filename and "_folder.png" not in filename:
#         bg = Image.open("blank_folder.png").convert("RGBA")
#         fg = Image.open(filename).convert("RGBA")
#         fg_resized = fg.resize((256, 256), Image.LANCZOS)
#         bg.paste(fg_resized, (128, 156), fg_resized)
#         bg.save(os.path.splitext(filename)[0] + "_folder.png")

files = ["terrain.png", "pointcloud.png", "camera.png"]
for filename in files:
    if ".png" in filename and "_folder.png" not in filename:
        bg = Image.open("blank_folder.png").convert("RGBA")
        fg = Image.open(filename).convert("RGBA")
        fg_resized = fg.resize((256, 256), Image.LANCZOS)
        bg.paste(fg_resized, (128, 156), fg_resized)
        bg.save(os.path.splitext(filename)[0] + "_folder.png")
