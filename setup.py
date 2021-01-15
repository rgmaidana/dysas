from setuptools import setup

setup(name='DYSAS',
      version='0.3',
      description='Python package for the Dynamic Systems Accident Simulator (DySAS)',
      url='http://www.github.com/rgmaidana/dysas',
      author='Renan Maidana',
      author_email='renan.g.maidana@ntnu.no',
      license='MIT',
      packages=['DYSAS'],
      install_requires=[
          'numpy',
          'scipy'
      ],
      zip_safe=False
    )
