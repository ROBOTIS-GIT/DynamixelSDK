
from setuptools import setup, find_packages
import platform


authors_info = [
    ('Leon Jung', 'rwjung@robotis.com'),
    ('Wonho Yun', 'ywh@robotis.com'),
]

authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)

setup(
    name='dynamixel_sdk',
    version='3.8.3',
    packages=['dynamixel_sdk'],
    package_dir={'': 'src'},
    license='Apache 2.0',
    description='Dynamixel SDK 3. python package',
    long_description=open('README.txt').read(),
    url='https://github.com/ROBOTIS-GIT/DynamixelSDK',
    author=authors,
    author_email=author_emails,
    install_requires=['pyserial']
)
