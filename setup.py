from setuptools import setup, find_packages

setup(
    name='drone-quadcopter-sim',
    version='0.1.0',
    description='Quadcopter drone simulation with 3D visualization',
    author='Your Name',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'matplotlib',
    ],
    python_requires='>=3.7',
)
