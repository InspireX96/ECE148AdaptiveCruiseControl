from setuptools import setup, find_packages

setup(
    name='donkeyacc',
    version='0.0',
    install_requires=['pytest'],
    packages=find_packages(include=['', 'src']),

)