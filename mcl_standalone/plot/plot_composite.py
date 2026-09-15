import re
import time
import argparse

from pathlib import Path
from functools import partial

import numpy as np
import matplotlib.pyplot as plt

from PIL import Image


def get_index(folder_name_, path_):
    match_ = re.search(
        re.escape(folder_name_) + r"_(\d+)\.pgm$",
        path_.name,
    )

    if match_:
        return int(match_.group(1))

    return -1


def get_files_general(dir_):
    folder_name_ = dir_.name

    files_ = list(
        dir_.glob(f"{folder_name_}_*.pgm")
    )

    files_.sort(
        key=partial(get_index, folder_name_)
    )

    return files_


def main():
    parser_ = argparse.ArgumentParser()

    parser_.add_argument(
        "--dir",
        type=str,
        default=None
    )

    parser_.add_argument(
        "--delay",
        type=float,
        default=0.1
    )

    args_ = parser_.parse_args()

    project_dir_ = Path(__file__).resolve().parent.parent

    if args_.dir is None:
        maps_dir_ = project_dir_ / "maps"
    else:
        dir_ = Path(args_.dir)

        # --dir may point to either maps/lidar or maps/composite
        if dir_.name in ("lidar", "composite"):
            maps_dir_ = dir_.parent
        else:
            maps_dir_ = dir_

    composite_dir_ = maps_dir_ / "composite"
    lidar_dir_ = maps_dir_ / "lidar"

    if not composite_dir_.exists():
        raise RuntimeError(
            f"Composite directory not found: {composite_dir_}"
        )

    if not lidar_dir_.exists():
        raise RuntimeError(
            f"Lidar directory not found: {lidar_dir_}"
        )

    print(f"Composite: {composite_dir_}")
    print(f"Lidar:     {lidar_dir_}")

    composite_files_ = get_files_general(composite_dir_)
    lidar_files_ = get_files_general(lidar_dir_)

    composite_by_index_ = {
        get_index("composite", file_): file_
        for file_ in composite_files_
    }

    lidar_by_index_ = {
        get_index("lidar", file_): file_
        for file_ in lidar_files_
    }

    common_indices_ = sorted(
        set(composite_by_index_) &
        set(lidar_by_index_)
    )

    if not common_indices_:
        raise RuntimeError(
            "No matching composite/lidar frames found."
        )

    plt.ion()

    fig_, axes_ = plt.subplots(
        1,
        2,
        figsize=(12, 6)
    )

    composite_ax_, lidar_ax_ = axes_

    composite_plot_ = None
    lidar_plot_ = None

    for index_ in common_indices_:
        if not plt.fignum_exists(fig_.number):
            break

        composite_image_ = Image.open(
            composite_by_index_[index_]
        )

        lidar_image_ = Image.open(
            lidar_by_index_[index_]
        )

        composite_array_ = np.array(composite_image_)
        lidar_array_ = np.array(lidar_image_)

        if composite_plot_ is None:
            composite_plot_ = composite_ax_.imshow(
                composite_array_,
                cmap="gray",
                vmin=0,
                vmax=255
            )

            lidar_plot_ = lidar_ax_.imshow(
                lidar_array_,
                cmap="gray",
                vmin=0,
                vmax=255
            )

            composite_ax_.set_axis_off()
            lidar_ax_.set_axis_off()

        else:
            composite_plot_.set_data(composite_array_)
            lidar_plot_.set_data(lidar_array_)

        composite_ax_.set_title(
            f"Composite Map - Frame {index_}"
        )

        lidar_ax_.set_title(
            f"Lidar Scan - Frame {index_}"
        )

        fig_.canvas.draw_idle()

        plt.pause(args_.delay)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()