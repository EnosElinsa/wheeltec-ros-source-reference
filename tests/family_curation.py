from pathlib import Path
import shutil

def extract_consumer_packages(workspace: Path, package_names: list[str], destination: Path) -> list[Path]:
    result=[]
    for name in package_names:
        source=workspace/name; target=destination/name
        shutil.copytree(source,target); result.append(target)
    return result

def classify_family_variants(paths: list[Path]) -> dict[str, int]:
    return {path.name: sum(1 for item in paths if item.name==path.name) for path in paths}

def validate_no_upstream_mirror(path: Path) -> list[str]:
    forbidden=('navigation2','cartographer','tensorflow','pytorch','librealsense','dataset')
    return [item.as_posix() for item in path.rglob('*') if any(token in item.name.lower() for token in forbidden)]
