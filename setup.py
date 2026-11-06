from setuptools import setup

package_name = 'following_study'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaan',
    maintainer_email='kaanbolukbas0116@gmail.com',
    description='HRI çalışması — kişi takip simülasyonu',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follower_node = following_study.follower_node:main',
        ],
    },
)
