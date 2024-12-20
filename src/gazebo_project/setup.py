import os
from glob import glob
from setuptools import find_packages, setup

package_name = "gazebo_project"


# Helper function to recursively get all files under a directory
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # Create relative path from current directory to file
            relative_path = os.path.relpath(path)
            paths.append((os.path.join('share', package_name, relative_path), 
                         [os.path.join(path, filename)]))
    return paths

data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Handle model directory and its subdirectories
        # *[(os.path.join('share', package_name, os.path.dirname(p)), [p]) 
        #   for p in glob('model/**/*', recursive=True) 
        #   if os.path.isfile(p)],
        *package_files('model'),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),  
    ]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nuke",
    maintainer_email="iamburakglr@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odometry_tf = gazebo_project.odometry_tf:main",
        ],
    },
)
