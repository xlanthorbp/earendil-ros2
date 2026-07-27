from setuptools import setup, find_packages

setup(
    name="rscp-protobuf",
    version="0.3.0",
    packages=find_packages(),
    install_requires=[
        "protobuf>=3.0.0",
    ],
    package_data={
        "rscp_protobuf": ["*.py"],
    },
    include_package_data=True,
)
