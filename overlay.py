"""Shared burned-in timestamp overlay, used by both camera sources.

Drawing a timestamp directly on the frame (rather than relying on wall-clock
metadata alongside it) is what makes it possible to eyeball, from the
streamed video alone, that both cameras are actually live and roughly in
sync - useful during this bring-up phase before any real synchronization or
logging exists.
"""

from datetime import datetime

import cv2

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_MARGIN = 4


def draw_timestamp(bgr_frame, ts: float | None = None) -> None:
    """Burn a "YYYY-MM-DD HH:MM:SS.mmm" stamp into the bottom-left corner, in place."""
    dt = datetime.fromtimestamp(ts if ts is not None else datetime.now().timestamp())
    text = dt.strftime("%Y-%m-%d %H:%M:%S") + f".{dt.microsecond // 1000:03d}"
    frame_h, frame_w = bgr_frame.shape[:2]
    max_width = frame_w - 2 * _MARGIN
    max_height = frame_h - 2 * _MARGIN

    scale = 0.5
    (text_w, text_h), baseline = cv2.getTextSize(text, _FONT, scale, 1)
    if text_w > max_width or (text_h + baseline) > max_height:
        scale *= min(max_width / text_w, max_height / (text_h + baseline))
        scale = max(scale, 0.1)
        (text_w, text_h), baseline = cv2.getTextSize(text, _FONT, scale, 1)

    org = (_MARGIN, frame_h - _MARGIN - baseline)
    cv2.putText(bgr_frame, text, org, _FONT, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(bgr_frame, text, org, _FONT, scale, (255, 255, 255), 1, cv2.LINE_AA)
