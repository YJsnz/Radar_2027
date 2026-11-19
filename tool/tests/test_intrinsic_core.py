from types import SimpleNamespace

import numpy as np

from tool.intrinsic_core import detect_board


def test_detect_board_uses_subpixel_corners_on_a_complete_board():
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    square = 40
    x0, y0 = 100, 60
    for row in range(9):
        for column in range(12):
            if (row + column) % 2 == 0:
                image[y0 + row * square:y0 + (row + 1) * square,
                      x0 + column * square:x0 + (column + 1) * square] = 0

    result = detect_board(image, SimpleNamespace(board=(11, 8, 17)))

    assert result["reason"] == "检测到完整棋盘格"
    assert result["corners"].shape == (88, 1, 2)
    points = result["corners"].reshape(8, 11, 2)
    assert np.allclose(points[0, 0], [x0 + square, y0 + square], atol=0.5)
