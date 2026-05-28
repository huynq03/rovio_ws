from setuptools import find_packages
from setuptools import setup

setup(
    name='rovio_interfaces',
    version='2.0.0',
    packages=find_packages(
        include=('rovio_interfaces', 'rovio_interfaces.*')),
)
