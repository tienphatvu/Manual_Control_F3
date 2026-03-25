from setuptools import find_packages
from setuptools import setup

setup(
    name='sick_safetyscanners2_interfaces',
    version='1.0.0',
    packages=find_packages(
        include=('sick_safetyscanners2_interfaces', 'sick_safetyscanners2_interfaces.*')),
)
