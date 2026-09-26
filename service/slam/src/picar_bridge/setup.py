from setuptools import setup

setup(
    name="picar_bridge",
    version="0.1.0",
    packages=["picar_bridge"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/picar_bridge"]),
        ("share/picar_bridge", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["bridge = picar_bridge.bridge:main"]},
)
