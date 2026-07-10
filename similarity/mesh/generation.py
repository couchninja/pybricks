import trimesh


def create_cube(size: float = 1.0) -> trimesh.Trimesh:
    return trimesh.creation.box(extents=[size, size, size])
