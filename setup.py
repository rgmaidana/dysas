from setuptools import setup

setup(name='MarineSystemSim',
      version='0.1',
      description='Python package for Marine Systems Simulation (e.g., models for marine vessels)',
      url='http://www.github.com/rgmaidana/python-mss',
      author='Renan Maidana',
      author_email='renan.g.maidana@ntnu.no',
      license='MIT',
      packages=['MarineSystemSim'],
      install_requires=[
          'numpy',
          'scipy'
      ],
      zip_safe=False
    )