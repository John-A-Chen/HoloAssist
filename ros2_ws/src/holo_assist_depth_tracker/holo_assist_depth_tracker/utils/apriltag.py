from __future__ import annotations

from typing import List


def default_tag_frame_name(tag_family: str, tag_id: int) -> str:
    return f"tag{tag_family}:{int(tag_id)}"


def candidate_tag_frame_names(tag_family: str, tag_id: int, explicit_name: str = "") -> List[str]:
    explicit_name = str(explicit_name).strip()
    candidates = []
    if explicit_name:
        candidates.append(explicit_name)
    candidates.extend(
        [
            f"tag{tag_family}:{int(tag_id)}",
            f"tag{int(tag_id)}",
            f"tag_{int(tag_id)}",
            f"{tag_family}:{int(tag_id)}",
            f"apriltag_{int(tag_id)}",
        ]
    )

    output: List[str] = []
    seen = set()
    for frame in candidates:
        if frame and frame not in seen:
            seen.add(frame)
            output.append(frame)
    return output
