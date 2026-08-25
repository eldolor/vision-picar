"""
rooms.py

Phase 5 (sim) -- semantic navigation & room recognition.

Stores landmark features per room type and matches a scene's visible
objects against them to guess which room the robot is likely in --
exactly the JSON-landmark idea from the build plan:

    {
      "kitchen": {
        "features": ["white cabinets", "refrigerator", "tile flooring"]
      }
    }

On real hardware/photos this is what a Vision LLM would infer from
visual context, since there's no ground truth "current room" available.
The grid-world *does* know the ground truth directly (frame['room']),
so identify_room() here is a standalone, independently testable
function rather than something navigation control flow depends on --
see brain/agent.py for how room-level goals are actually driven (off
the grid-world's ground truth, same as everything else in sim so far).
"""

# Deliberately generic landmark sets (typical objects for each room
# type), independent of what happens to be placed in any one map --
# this is meant to generalize the way a VLM's world knowledge would.
ROOM_FEATURES = {
    "living room": ["sofa", "television", "rug", "coffee table", "armchair"],
    "kitchen": ["refrigerator", "white cabinets", "tile flooring", "oven", "sink"],
    "bedroom": ["bed", "dresser", "nightstand", "closet"],
    "hallway": ["doorway", "coat rack"],
}


def identify_room(visible_objects: list) -> str:
    """
    Returns the best-matching room name given a list of visible object
    labels, or "unknown" if nothing matches any room's feature set.
    Matching is substring-based and case-insensitive in both directions
    (e.g. visible "white kitchen cabinets" matches feature "white
    cabinets", and visible "cabinets" alone also matches).
    """
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
