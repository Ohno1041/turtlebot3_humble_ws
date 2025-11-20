# ベースイメージ
FROM osrf/ros:humble-desktop-full

# 環境変数
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV TURTLEBOT_WS=/opt/turtlebot3_ws

# 動的な環境検出用のスクリプトを作成
RUN echo '#!/bin/bash' > /usr/local/bin/detect_environment.sh && \
    echo '# WSL2環境かどうかを検出' >> /usr/local/bin/detect_environment.sh && \
    echo 'if [ -d "/usr/lib/wsl" ]; then' >> /usr/local/bin/detect_environment.sh && \
    echo '    echo "WSL2_ENV=true"' >> /usr/local/bin/detect_environment.sh && \
    echo '    echo "LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH"' >> /usr/local/bin/detect_environment.sh && \
    echo 'else' >> /usr/local/bin/detect_environment.sh && \
    echo '    echo "WSL2_ENV=false"' >> /usr/local/bin/detect_environment.sh && \
    echo '    echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"' >> /usr/local/bin/detect_environment.sh && \
    echo 'fi' >> /usr/local/bin/detect_environment.sh && \
    chmod +x /usr/local/bin/detect_environment.sh

# 必要なシステムパッケージをインストール
# python3-pip を追加（Pythonライブラリを手動で入れるため）
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-dynamixel-sdk \
    x11-xserver-utils \
    nano \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# 【追加】必要なPythonライブラリをインストール
# transforms3d をインストール（tf_transformationsの依存関係）
RUN pip3 install --no-cache-dir transforms3d

# tf_transformations はROSパッケージとして後でインストールされる

# rosdepの初期化
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
    && rosdep init \
    && rosdep update

# ワークスペースの作成とTurtleBot3ソースコードのクローン
WORKDIR $TURTLEBOT_WS
RUN mkdir -p src && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git src/turtlebot3 && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git src/turtlebot3_msgs && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git src/turtlebot3_simulations

# 依存関係のインストール
# 【修正ポイント】
# gripper_controllers: アーム用なので無視
# hls-lfcd-lds-driver: ROS 2 Humbleの公式リポジトリに含まれていないためスキップ
# 他にも互換性のない依存関係をまとめて除外
RUN rosdep install --from-paths src --ignore-src -y --rosdistro humble \
    --skip-keys "gripper_controllers hls-lfcd-lds-driver ld06_driver" || true

# ワークスペースのビルドを実行
# シミュレーション用の重要なパッケージのみを優先的にビルド
# 並列度を制限してリソース使用量を抑制
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --continue-on-error --parallel-workers 2 --packages-select turtlebot3_gazebo turtlebot3_navigation2 turtlebot3_teleop turtlebot3_cartographer turtlebot3_description turtlebot3_msgs"

# 残りのパッケージをビルド（失敗しても問題ない）
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --continue-on-error --parallel-workers 1" || true

# .bashrc の設定（環境に応じて動的設定）
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source $TURTLEBOT_WS/install/setup.bash" >> /root/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc
RUN echo "# 動的環境設定の読み込み" >> /root/.bashrc
RUN echo 'eval "$(/usr/local/bin/detect_environment.sh)"' >> /root/.bashrc
RUN echo "# Gazebo GUI設定（プラットフォーム共通）" >> /root/.bashrc
RUN echo "export GAZEBO_GUI=false" >> /root/.bashrc
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /root/.bashrc
RUN echo "export QT_X11_NO_MITSHM=1" >> /root/.bashrc

# 開発用ワークスペースの初期化
RUN mkdir -p /workspaces && \
    echo "source /opt/ros/humble/setup.bash" > /tmp/setup_workspace.sh && \
    echo "source /opt/turtlebot3_ws/install/setup.bash" >> /tmp/setup_workspace.sh && \
    echo "export TURTLEBOT3_MODEL=burger" >> /tmp/setup_workspace.sh && \
    echo 'eval "$(/usr/local/bin/detect_environment.sh)"' >> /tmp/setup_workspace.sh && \
    echo "export GAZEBO_GUI=false" >> /tmp/setup_workspace.sh && \
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /tmp/setup_workspace.sh && \
    echo "export QT_X11_NO_MITSHM=1" >> /tmp/setup_workspace.sh && \
    chmod +x /tmp/setup_workspace.sh

# .bashrc の設定を更新
RUN echo "source /tmp/setup_workspace.sh" >> /root/.bashrc

# VS Codeが開くディレクトリの設定（Dev Containersがマウントするパス）
WORKDIR /workspaces

# 開発用ワークスペースの自動ビルド設定
# コンテナ起動時にワークスペースを自動ビルドし、環境をセットアップするスクリプトを作成
RUN echo '#!/bin/bash' > /usr/local/bin/setup_dev_workspace.sh && \
    echo 'cd /workspaces/turtlebot3_humble_ws' >> /usr/local/bin/setup_dev_workspace.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /usr/local/bin/setup_dev_workspace.sh && \
    echo 'source /opt/turtlebot3_ws/install/setup.bash' >> /usr/local/bin/setup_dev_workspace.sh && \
    echo 'if [ -f "src/my_continuous_nav/package.xml" ] && [ ! -f "install/setup.bash" ]; then' >> /usr/local/bin/setup_dev_workspace.sh && \
    echo '  echo "Building my_continuous_nav workspace..."' >> /usr/local/bin/setup_dev_workspace.sh && \
    echo '  colcon build --symlink-install' >> /usr/local/bin/setup_dev_workspace.sh && \
    echo '  echo "Workspace build complete!"' >> /usr/local/bin/setup_dev_workspace.sh && \
    echo 'fi' >> /usr/local/bin/setup_dev_workspace.sh && \
    echo 'if [ -f "install/setup.bash" ]; then' >> /usr/local/bin/setup_dev_workspace.sh && \
    echo '  source install/setup.bash' >> /usr/local/bin/setup_dev_workspace.sh && \
    echo 'fi' >> /usr/local/bin/setup_dev_workspace.sh && \
    chmod +x /usr/local/bin/setup_dev_workspace.sh

# .bashrc に自動セットアップを追加
RUN echo 'if [ -d "/workspaces/turtlebot3_humble_ws" ]; then' >> /root/.bashrc && \
    echo '  /usr/local/bin/setup_dev_workspace.sh' >> /root/.bashrc && \
    echo 'fi' >> /root/.bashrc

CMD ["bash"]