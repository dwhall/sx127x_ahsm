import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="sx127x_ahsm",
    version="0.1.0",
    author="Dean Hall",
    author_email="dwhall256@gmail.com",
    description="A driver for the Semtech SX127X radio data modem.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/dwhall/sx127x_ahsm",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3.4",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "License :: OSI Approved :: MIT License",

        # This project is deprected
        "Development Status :: 7 - Inactive",

        # This project is designed to run on a Raspberry Pi
        # with a SX127X LoRa radio attached via the SPI bus
        "Operating System :: POSIX :: Linux",
        "Topic :: System :: Hardware :: Hardware Drivers",
        "Topic :: Communications :: Ham Radio",
    ],
)
