"""Ground support is exempt only for the generated floor collision identity."""

def is_floor(name):
    parts=name.split('::')
    return len(parts)>=3 and parts[-3:]==['floor','link','floor_collision']

def collision_count(contacts):
    # Empty/unknown names are conservative collisions; a string merely
    # containing "floor" cannot exempt a real obstacle with a similar name.
    return sum(not is_floor(c.collision1.name) and not is_floor(c.collision2.name) for c in contacts)
