"""
Setup package for unit test and continuous integration, not very useful for donkeycar implementation
"""

from setuptools import setup, find_packages

setup(
    name='donkeyacc',
    version='0.0.1',
    description='Setup package for testing donkeycar adaptive cruise control',
    install_requires=['pytest'],
    packages=find_packages(include=['', 'donkeyacc']),
)
