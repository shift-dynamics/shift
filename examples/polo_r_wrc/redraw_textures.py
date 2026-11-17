"""Redraws the textures of the Polo R WRC glTF (see readme.rst) at 4096 x
4096, four or eight times the source's 1024 x 1024 or 512 x 512 (TEXTURES):
the livery, the details (the lamps, the spoiler, the wheels) and their
normal map, the clear coat of the livery, the tires, the underbody and the
glow of the lamps, for ../../scripts/vehicle_splits/polo_r_wrc.json, which
maps each as the image it replaces:

    python3 redraw_textures.py source/textures

Each texture is scaled up by bicubic interpolation, which blurs each edge of
the source (of a stripe, a letter, a logo) over as many pixels as it is
scaled up. Each edge lies
between two colors, those of the pixels on either side of it (sides); where
the scaled-up color lies between them, its place along the way from one to
the other is made as many times steeper about the middle, so that the edge is
as wide as a pixel of the result, and the colors themselves, with their
shading and dirt, are kept. Where the colors on either side differ little,
in the gradients of the source, it is only scaled up, as are the textures
that are soft throughout (the glow of the lamps) or not colors (the normal
map, whose steps would become ridges).

Writes the images of TEXTURES, next to this script.
"""

import os
import sys

import numpy as np
from PIL import Image
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
SIZE = 4096           # pixels along each side of the textures written
CONTRAST = (30.0, 80.0)  # color difference (0-255) from smooth to an edge


def zoom(image, scale, order=3):
    """image scaled up scale times by interpolation of order"""
    if image.ndim == 3:
        return np.stack([zoom(image[..., c], scale, order)
                         for c in range(image.shape[2])], axis=-1)
    return ndimage.zoom(image, scale, order=order, mode="nearest",
                        grid_mode=True)


def sides(source):
    """the two colors on either side of the edge through each pixel: of the
    pixels of its 3 x 3 neighbourhood, the one farthest from their mean and
    the one farthest from that, the darker first"""
    h, w, _ = source.shape
    padded = np.pad(source, ((1, 1), (1, 1), (0, 0)), mode="edge")
    window = np.stack([padded[i:i + h, j:j + w] for i in range(3)
                       for j in range(3)])
    mean = window.mean(axis=0)
    rows, cols = np.indices((h, w))
    first = window[np.argmax(((window - mean) ** 2).sum(-1), axis=0),
                   rows, cols]
    second = window[np.argmax(((window - first) ** 2).sum(-1), axis=0),
                    rows, cols]
    swap = (first.sum(-1) > second.sum(-1))[..., None]
    return np.where(swap, second, first), np.where(swap, first, second)


# the source's textures (as named in its textures directory), the images
# written for them, and whether their edges are made sharp or the texture
# is only scaled up
TEXTURES = {"material_baseColor.png": ("polo_r_wrc_livery.jpg", True),
            "material_clearcoat.png": ("polo_r_wrc_clearcoat.jpg", True),
            "Material.001_baseColor.png": ("polo_r_wrc_details.jpg", True),
            "Material.001_normal.png": ("polo_r_wrc_details_normal.jpg",
                                        False),
            "Material.002_baseColor.png": ("polo_r_wrc_lamps.jpg", False),
            "Material.004_baseColor.png": ("polo_r_wrc_tires.jpg", True),
            "Material.005_baseColor.png": ("polo_r_wrc_underbody.jpg", True)}


def redraw(path, output, sharp):
    """writes the texture at path, scaled up to SIZE, with sharp edges if
    sharp, to output"""
    source = np.asarray(Image.open(path).convert("RGB"), np.float32)
    assert source.shape[0] == source.shape[1] and \
        SIZE % source.shape[0] == 0, path
    scale = SIZE // source.shape[0]
    dark, light = sides(source)
    smooth = zoom(source, scale)
    dark, light = zoom(dark, scale, 1), zoom(light, scale, 1)
    d = light - dark
    length = np.maximum((d ** 2).sum(-1), 1e-6)
    # where the scaled-up color lies between the two sides, and that made
    # scale times steeper about the middle, so that an edge as wide as a
    # pixel of the source is as wide as a pixel of the result
    t = ((smooth - dark) * d).sum(-1) / length
    steep = np.clip(0.5 + (t - 0.5) * scale, 0.0, 1.0)
    weight = np.clip((np.sqrt(length) - CONTRAST[0]) /
                     (CONTRAST[1] - CONTRAST[0]), 0.0, 1.0) * \
        ((t > -0.25) & (t < 1.25))
    out = smooth + (weight * (steep - np.clip(t, 0.0, 1.0)))[..., None] * d
    if not sharp:
        out = smooth

    Image.fromarray(np.clip(out + 0.5, 0, 255).astype(np.uint8)).save(
        output, quality=95)


for name, (output, sharp) in TEXTURES.items():
    redraw(os.path.join(sys.argv[1], name), os.path.join(HERE, output), sharp)
