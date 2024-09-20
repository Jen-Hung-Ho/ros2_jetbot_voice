## Jetbot Voice-Activated Copilot Tools Setup

1. **Configure Docker Engine**:
   Follow these [setup steps](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md) to configure your Docker engine.

2. **Set Up ROS Development Environment**:
   Set up your ROS development environment by following the instructions [here](https://docs.ros.org/en/humble/Installation.html).

3. **Clone the Repository**:
   Open your terminal and run the following command to clone the repository:
   ```bash
   git clone https://github.com/Jen-Hung-Ho/ros2_jetbot_voice
   ```

4. **Navigate to the Repository Directory**:
   Change to the directory of the cloned repository:
   ```bash
   cd ros2_jetbot_voice
   ```

5. **Build the Docker Image**:
   Ensure the `build.sh` script has execute permissions. If not, add execute permissions using:
   ```bash
   chmod +x build.sh
   ```

   Then, run the `build.sh` script to build the Docker image:
   ```bash
   ./build.sh
   ```
   
7. **Start Docker and Build the CNN Model**:
   Execute the following commands to run and build the CNN model `(this only needs to be done once)` The model data will be saved under `/data/models/ASR_classify_model` for the voice-activated tool to load and extract in the ROS2 node launch:
   ```bash
   . run.sh
   cd ..
   cd app
   python3 robot_command_text_classification.py  # build CNN model
   python3 robot_command_model_evaluation.py     # run unit test
   ```

8. **Start Docker:**
   Execute the following commands to run the docker
   ```bash
   . run.sh
   ```
    
9. **Attach to an Existing Running Docker Container**:
   To attach to an existing running Docker container, use the following commands:
   ```bash
   docker ps
   ```

   Identify the `CONTAINER ID` of the running container (e.g., `422fc05b7655`), then run:
   ```bash
   . start_ros2_shell.sh <CONTAINER_ID>
   ```

   For example:
   ```bash
   . start_ros2_shell.sh 422fc05b7655
