#!/bin/bash

# Gazebo起動前の環境チェックと設定スクリプト
echo "=== Gazebo環境チェック開始 ==="

# 環境変数の読み込み
source /opt/ros/humble/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
if [ -f "/workspaces/turtlebot3_humble_ws/install/setup.bash" ]; then
    source /workspaces/turtlebot3_humble_ws/install/setup.bash
fi

# 動的環境設定の読み込み
eval "$(/usr/local/bin/detect_environment.sh)"

echo "現在の環境設定:"
echo "  DISPLAY: $DISPLAY"
echo "  WSL2_ENV: ${WSL2_ENV:-false}"
echo "  TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"

# X11接続テスト
echo "=== X11接続テスト ==="
if command -v xset &> /dev/null; then
    if xset q &>/dev/null; then
        echo "✓ X11接続: 成功"
    else
        echo "✗ X11接続: 失敗"
        echo "  解決方法:"
        echo "  1. ネイティブUbuntu: 'xhost +local:docker' を実行"
        echo "  2. WSL2: Windows側でVcXsrvまたはWSLgを確認"
        exit 1
    fi
else
    echo "! X11ツールが利用できません（Headlessモード）"
fi

# OpenGL/Mesa テスト
echo "=== OpenGL/Mesa テスト ==="
if command -v glxinfo &> /dev/null; then
    GL_INFO=$(glxinfo 2>/dev/null | grep -i "opengl renderer" || echo "OpenGL renderer情報を取得できませんでした")
    echo "  $GL_INFO"
else
    echo "! glxinfo コマンドが利用できません"
fi

# Gazeboキャッシュのクリア
echo "=== Gazeboキャッシュのクリア ==="
rm -rf /tmp/clean_home/.gazebo/log/*
rm -rf /tmp/clean_home/.gazebo/models/.model_database_cache
echo "✓ Gazeboキャッシュをクリアしました"

# Gazeboの起動
echo "=== Gazebo起動 ==="
echo "以下のコマンドでGazeboを起動します:"
echo "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo ""

# 起動オプションの確認
if [ "$1" = "--start" ]; then
    echo "Gazeboを起動中..."
    exec ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
else
    echo "環境チェック完了。Gazeboを起動するには以下を実行してください:"
    echo "bash /usr/local/bin/check_gazebo_env.sh --start"
    echo ""
    echo "または直接:"
    echo "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
fi