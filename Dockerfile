#ベースイメージosrf/ros:humble-desktop-full
FROM osrf/ros:humble-desktop-full

#環境変数とロケール設定
#非対話モードでapt-getなどの質問を禁止
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV WORKSPACE_DIR /ws

#必要なシステムパッケージをインストール
#gazebo-ros、シミュレータのコア機能
#gazebo-ros-pkgs、gazebo用のROSパッケージ
#nav2-bringup、ナビゲーション2の経路計画や障害物回避などのパッケージ
#turtlebot3-navigation2、TurtleBot3用のナビゲーション
#colcon-common-extensions、symlink-installを使うため
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-navigation2 \
    nano \
    git \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ワークスペースの作成とTurtleBot3ソースコードのクローン
WORKDIR $WORKSPACE_DIR
RUN mkdir -p src && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git src/turtlebot3 && \
    git clone -b humble https://github.COM/ROBOTIS-GIT/turtlebot3_msgs.git src/turtlebot3_msgs && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git src/turtlebot3_simulations

# -----------------------------------------------------
# 🌟 修正ポイント：rosdep を実行して依存関係を解決 🌟
# -----------------------------------------------------
RUN apt-get update && apt-get install -y python3-rosdep \
    && rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
    && rosdep init \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y --rosdistro humble

# ワークスペースのビルドを実行
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

RUN echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
#RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$WORKSPACE_DIR/src/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc
# Gazebo起動時にGAZEBO_MODEL_PATHを確実にするエイリアス（別名）を設定
#RUN echo "alias tblaunch='export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/ws/src/turtlebot3_simulations/turtlebot3_gazebo/models && ros2 launch'" >> ~/.bashrc

# 起動コマンドは bash
CMD ["bash"]