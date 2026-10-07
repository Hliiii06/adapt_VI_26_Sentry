from glob import glob

from setuptools import setup

package_name = "sentry_scan_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="adapt_VI_26_Sentry",
    maintainer_email="dev@example.com",
    description="RM to SCAN input adapter and shadow guard (no chassis output)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rm_input_adapter = sentry_scan_adapter.rm_input_adapter:main",
            "shadow_guard = sentry_scan_adapter.shadow_guard:main",
            "check_inputs = sentry_scan_adapter.check_inputs:main",
        ],
    },
)
