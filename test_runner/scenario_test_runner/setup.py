from glob import glob

from setuptools import setup

package_name = "scenario_test_runner"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/ament_index/resource_index/packages",
            ["resource/workflow_schema.yaml"],
        ),
        ("share/" + package_name + "/launch", glob("launch/*.launch.*")),
        ("share/" + package_name + "/test/scenario", glob("test/scenario/*")),
        ("share/" + package_name, ["config/planning_simulator_v2.rviz"]),
        ("share/" + package_name, ["config/workflow_example.yaml"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Taiki Tanaka, Tatsuya Yamasaki",
    maintainer_email="taiki.tanaka@tier4.jp, tatsuya.yamasaki@tier4.jp",
    description="scenario test runner package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scenario_test_runner = scenario_test_runner.scenario_test_runner:main",
            "result_checker       = scenario_test_runner.result_checker:main",
        ],
    },
)
