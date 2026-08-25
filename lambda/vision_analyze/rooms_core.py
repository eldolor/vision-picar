"""
rooms_core.py

Lambda-local copy of brain/rooms.py's identify_room() -- kept in sync
manually, packaged separately for deployment simplicity (no repo-relative
imports, no extra dependencies). See brain/rooms.py for the canonical
version and its own tests.
"""

ROOM_FEATURES = {
    "living room": ["sofa", "television", "rug", "coffee table", "armchair"],
    "kitchen": ["refrigerator", "white cabinets", "tile flooring", "oven", "sink"],
    "bedroom": ["bed", "dresser", "nightstand", "closet"],
    "hallway": ["doorway", "coat rack"],
}


def identify_room(visible_objects: list) -> str:
    if not visible_objects:
        return "unknown"

    lowered_objects = [obj.lower() for obj in visible_objects]

    best_room = "unknown"
    best_score = 0
    for room, features in ROOM_FEATURES.items():
        score = 0
        for feature in features:
            feature = feature.lower()
            for obj in lowered_objects:
                if feature in obj or obj in feature:
                    score += 1
                    break
        if score > best_score:
            best_score = score
            best_room = room

    return best_room
