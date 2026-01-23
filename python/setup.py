
from setuptools import setup, find_packages
import platform
import shutil
import os

# Copy control_table folders for distribution
def copy_control_tables():
    source = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'control_table')
    dest = os.path.join('src', 'dynamixel_easy_sdk', 'control_table')
    
    if os.path.exists(dest):
        shutil.rmtree(dest)
    
    if os.path.exists(source):
        shutil.copytree(source, dest)
        print(f"Copied control_tables from {source} to {dest}")
    else:
        print(f"Warning: Control table source not found at {source}")

# Execute copy before setup
copy_control_tables()

authors_info = [
    ('Leon Jung', 'rwjung@robotis.com'),
    ('Wonho Yun', 'ywh@robotis.com'),
    ('Hyungyu Kim', 'kimhg@robotis.com'),
]

authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)

setup(
    name='dynamixel_sdk',
    version='4.0.4',
    packages=['dynamixel_sdk', 'dynamixel_easy_sdk'],
    package_dir={'': 'src'},
    package_data={'dynamixel_easy_sdk': ['control_table/*.json', 'control_table/*.model']},
    license='Apache 2.0',
    description='Dynamixel SDK 4. python package',
    long_description=open('README.txt').read(),
    url='https://github.com/ROBOTIS-GIT/DynamixelSDK',
    author=authors,
    author_email=author_emails,
    install_requires=['pyserial']
)
