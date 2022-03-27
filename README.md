# FRC 2022
Team 6940's 2022 FRC robot code.Code is written in Java and is based off WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions, the function of each package, and some of the variable naming conventions used. Additional information about each specific class can be found in that class' Java file.

### Setup Instructions

1. Clone this repo
1. Run `./gradlew` to download Gradle and needed FRC/Vendor libraries (make sure you're using Java 11 or greater)
1. Run `./gradlew` downloadAll to download FRC tools (ShuffleBoard, etc.)
1. Run `./gradlew` tasks to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension from the release page on [this repository](https://github.com/wpilibsuite/allwpilib/releases/latest)
2. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### IntelliJ
1. Run `./gradlew idea`
1. Open the `2022RapidReact.ipr` file with IntelliJ
1. When prompted, select import Gradle build

### Eclipse
1. Run `./gradlew eclipse`
1. Open Eclipse and go to File > Open Projects from File System...
1. Set the import source to the `2022RapidReact` folder then click finish

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details
* Run `./gradlew test` to run all of the JUnit tests

### Code Structure

![FRC code structure](https://user-images.githubusercontent.com/62934294/160263652-df9f4034-478b-48ed-a99c-72ce4bbe1fda.png)

## Code Highlights
- Path following with [PathPlanner](https://github.com/mjansen4857/pathplanner)(Credit [@mjansen4857](https://github.com/mjansen4857))

- Shooter algorithm. We have two modes for shooting. One depends on the [Ball Trajectory equation](), the other depends on the [interpolation table]().

## Package Functions

