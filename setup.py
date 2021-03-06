import setuptools

from source.conf import release, author, project

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name=project,
    version=release,
    author=author,
    author_email="kingfree@toyama.moe",
    description="Lebai Robot Python HTTP SDK",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/lebai-robotics/lebai-python-http",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.4',
)