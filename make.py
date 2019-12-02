#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import subprocess
import sys
from urllib.request import urlretrieve


def dl_progress(count, block_size, total_size):
    percent = min(int(count * block_size * 100 / total_size), 100)
    sys.stdout.write(f"\r-> {percent}%")
    sys.stdout.flush()


def download_file(maven_url, filename, dest_dir):
    """Download file from maven server.

    Keyword arguments:
    filename -- file to download
    dest_dir -- destination directory for file
    """
    dest_file = f"{dest_dir}/{filename}"
    if not os.path.exists(f"{dest_dir}/{filename}"):
        print(f"Downloading {filename}...")
        print(f"Querying {maven_url}/{filename}")
        urlretrieve(
            url=f"{maven_url}/{filename}", filename=dest_file, reporthook=dl_progress
        )
        print(" done.")

    folder_name = f"{dest_dir}/{os.path.splitext(filename)[0]}"
    if not os.path.exists(folder_name):
        print(f"Unzipping {dest_file}...", end="")
        sys.stdout.flush()
        subprocess.run(["unzip", "-q", "-d", folder_name, f"{dest_file}"])
        print(" done.")


def download_lib(maven_url, artifact_name, version, classifier):
    maven_url += f"/{artifact_name}/{version}"

    # Download libs
    filename = f"{artifact_name}-{version}-{classifier}.zip"
    zip_name = f"build/{filename}"
    download_file(maven_url, filename, "build")

    # Download headers
    filename = f"{artifact_name}-{version}-headers.zip"
    zip_name = f"build/{filename}"
    download_file(maven_url, filename, "build")


def main():
    parser = argparse.ArgumentParser(description="Builds and deploys FRC C++ programs")
    parser.add_argument(
        "target",
        choices=["build", "deploy", "clean", "test"],
        help="'build' compiles the robot program and downloads missing dependencies. 'deploy' compiles the program if it hasn't already and deploys it. 'clean' removes all build artifacts from the build folder. 'test' compiles the robot program and downloads missing dependencies, then runs the tests.",
    )
    args = parser.parse_args()

    if not os.path.exists("build/"):
        os.makedirs("build/")

    WPI_MAVEN_URL = "http://first.wpi.edu/FRC/roborio/maven/release"
    REV_MAVEN_URL = "http://www.revrobotics.com/content/sw/max/sdk/maven"
    WPI_URL = WPI_MAVEN_URL + "/edu/wpi/first"
    OPENCV_URL = WPI_MAVEN_URL + "/edu/wpi/first/thirdparty/frc2019"
    REV_URL = REV_MAVEN_URL + "/com/revrobotics/frc"
    REV_LOCAL_URL = f"file://{Path.home()}/releases/maven/release/com/revrobotics/frc"

    WPI_VERSION = "2019.4.1"

    if args.target == "build":
        classifier = "linuxathena"
    else:
        classifier = "linuxx86-64"
        REV_URL = REV_LOCAL_URL

    download_lib(WPI_URL + "/wpilibc", "wpilibc-cpp", WPI_VERSION, classifier)
    download_lib(WPI_URL + "/cameraserver", "cameraserver-cpp", WPI_VERSION, classifier)
    download_lib(WPI_URL + "/ntcore", "ntcore-cpp", WPI_VERSION, classifier)
    download_lib(WPI_URL + "/hal", "hal-cpp", WPI_VERSION, classifier)
    download_lib(WPI_URL + "/cscore", "cscore-cpp", WPI_VERSION, classifier)
    download_lib(WPI_URL + "/wpiutil", "wpiutil-cpp", WPI_VERSION, classifier)
    download_lib(OPENCV_URL + "/opencv", "opencv-cpp", "3.4.4-5", classifier)

    if args.target == "build":
        download_lib(WPI_URL + "/ni-libraries", "netcomm", "2019.12.1", classifier)
        download_lib(WPI_URL + "/ni-libraries", "chipobject", "2019.12.1", classifier)

    download_lib(REV_URL, "SparkMax-cpp", "1.4.1", classifier + "static")
    download_lib(REV_URL, "SparkMax-driver", "1.4.1", classifier + "static")

    # Generate pubsub messages
    if (
        not os.path.exists("build/generated")
        or os.path.getmtime("msgs") > os.path.getmtime("build/generated")
        or os.path.getmtime("python/generate_messages.py")
        > os.path.getmtime("build/generated")
    ):
        print("Generating PubSub messages...", end="")
        subprocess.run(
            [
                sys.executable,
                "python/generate_messages.py",
                "--input",
                "msgs",
                "--output",
                "build/generated",
            ]
        )
        subprocess.run(["touch", "build/generated"])
        print(" done.")

    nproc = subprocess.check_output("nproc", encoding="utf-8").strip()
    make_athena = ["make", "-f", "mk/Makefile-linuxathena"]
    make_x86_64 = ["make", "-f", "mk/Makefile-linuxx86-64"]

    if args.target == "build":
        subprocess.run(make_athena + ["build", f"-j{nproc}"])
    elif args.target == "deploy":
        subprocess.run(make_athena + ["deploy"])
    elif args.target == "clean":
        subprocess.run(make_athena + ["clean"])
        subprocess.run(make_x86_64 + ["clean"])
    elif args.target == "test":
        subprocess.run(make_x86_64 + ["build", f"-j{nproc}"], check=True)
        subprocess.run(["build/linuxx86-64/FRCUserProgram"])


if __name__ == "__main__":
    main()