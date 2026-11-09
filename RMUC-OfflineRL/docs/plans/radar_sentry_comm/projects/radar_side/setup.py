from setuptools import find_packages, setup

setup(
    name="radar-sentry-radar",
    version="0.1.0",
    description="RMUC 2027 radar-side 0x0210 TX / 0x0220 RX service",
    packages=find_packages(include=["radar_sentry_radar", "radar_sentry_radar.*"]),
    python_requires=">=3.10",
    entry_points={"console_scripts": ["radar-sentry-radar=radar_sentry_radar.app:main"]},
)
