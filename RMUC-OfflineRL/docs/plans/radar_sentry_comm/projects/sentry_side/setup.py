from setuptools import find_packages, setup

setup(
    name="radar-sentry-sentry",
    version="0.1.0",
    description="RMUC 2027 sentry-side 0x0210 RX / 0x0220 TX service",
    packages=find_packages(include=["radar_sentry_sentry", "radar_sentry_sentry.*"]),
    python_requires=">=3.10",
    entry_points={"console_scripts": ["radar-sentry-sentry=radar_sentry_sentry.app:main"]},
)
