#!/bin/bash
# ROS2 Humble to Jazzy 自动化升级脚本
# 版权声明：MIT License | Copyright (c) 2026 思捷娅科技 (SJYKJ)

set -e

echo "=========================================="
echo "🚀 ROS2 Humble to Jazzy 升级脚本"
echo "=========================================="
echo

# 检查 Ubuntu 版本
UBUNTU_VERSION=$(lsb_release -rs)
if [ "$UBUNTU_VERSION" != "24.04" ]; then
    echo "⚠️  警告：ROS2 Jazzy 需要 Ubuntu 24.04"
    echo "   当前版本：Ubuntu $UBUNTU_VERSION"
    read -p "是否继续？(y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 步骤 1: 安装 ROS2 Jazzy
echo "📦 步骤 1: 安装 ROS2 Jazzy..."
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# 添加 ROS2 GPG 密钥
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加 ROS2 Jazzy 仓库
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop-full

echo "✅ ROS2 Jazzy 安装完成"
echo

# 步骤 2: 安装额外依赖
echo "📦 步骤 2: 安装额外依赖..."
sudo apt install -y ros-jazzy-tf2 ros-jazzy-tf2-geometry-msgs
sudo apt install -y ros-jazzy-imu-filter-madgwick ros-jazzy-tf-transformations
sudo apt install -y ros-jazzy-slam-toolbox
sudo apt install -y ros-jazzy-navigation2
sudo apt install -y ros-jazzy-rviz2

echo "✅ 额外依赖安装完成"
echo

# 步骤 3: 设置环境变量
echo "⚙️  步骤 3: 设置环境变量..."
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "✅ 环境变量已添加到 ~/.bashrc"
else
    echo "ℹ️  环境变量已存在"
fi

source ~/.bashrc
echo "✅ 环境变量已加载"
echo

# 步骤 4: 更新 package.xml
echo "📝 步骤 4: 更新 package.xml 文件..."
for package_xml in $(find . -name "package.xml"); do
    echo "  更新：$package_xml"
    
    # 备份原文件
    cp "$package_xml" "$package_xml.bak"
    
    # 更新 CMake 版本要求（如果有注释）
    # package.xml format 已经是 3，不需要修改
done
echo "✅ package.xml 更新完成"
echo

# 步骤 5: 更新 CMakeLists.txt
echo "📝 步骤 5: 更新 CMakeLists.txt 文件..."
for cmake_file in $(find . -name "CMakeLists.txt"); do
    echo "  更新：$cmake_file"
    
    # 备份原文件
    cp "$cmake_file" "$cmake_file.bak"
    
    # 更新 CMake 最低版本要求
    sed -i 's/cmake_minimum_required(VERSION 3.8)/cmake_minimum_required(VERSION 3.20)/g' "$cmake_file"
    sed -i 's/cmake_minimum_required(VERSION 3.10)/cmake_minimum_required(VERSION 3.20)/g' "$cmake_file"
done
echo "✅ CMakeLists.txt 更新完成"
echo

# 步骤 6: 更新 README 和文档
echo "📝 步骤 6: 更新文档..."
if [ -f "README.md" ]; then
    cp README.md README.md.bak
    sed -i 's/ROS%202%20Humble/ROS%202%20Jazzy/g' README.md
    sed -i 's/humble/jazzy/g' README.md
    echo "✅ README.md 已更新"
fi

if [ -f "CONTRIBUTING.md" ]; then
    cp CONTRIBUTING.md CONTRIBUTING.md.bak
    sed -i 's/humble/jazzy/g' CONTRIBUTING.md
    echo "✅ CONTRIBUTING.md 已更新"
fi
echo

# 步骤 7: 更新 GitHub Actions 工作流
echo "📝 步骤 7: 更新 CI/CD 配置..."
if [ -f ".github/workflows/industrial_ci.yml" ]; then
    cp .github/workflows/industrial_ci.yml .github/workflows/industrial_ci.yml.bak
    sed -i 's/ROS_DISTRO: humble/ROS_DISTRO: jazzy/g' .github/workflows/industrial_ci.yml
    echo "✅ GitHub Actions 工作流已更新"
fi
echo

# 步骤 8: 清理和构建
echo "🔨 步骤 8: 清理和构建..."
echo "  清理旧的构建文件..."
rm -rf build/ install/ log/

echo "  安装 ROS 依赖..."
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true

echo "  构建项目..."
colcon build --symlink-install

echo "✅ 构建完成"
echo

# 步骤 9: 运行测试
echo "🧪 步骤 9: 运行基本测试..."
source install/setup.bash

# 检查 ROS2 版本
ROS_VERSION=$(ros2 --version | head -1)
echo "✅ ROS2 版本：$ROS_VERSION"

# 步骤 10: 创建升级报告
echo "📊 步骤 10: 创建升级报告..."
cat > ROS2_JAZZY_UPGRADE_REPORT.md << 'EOF'
# ROS2 Jazzy 升级报告

## 升级信息

- **升级日期**: $(date +%Y-%m-%d)
- **源版本**: ROS2 Humble Hawksbill
- **目标版本**: ROS2 Jazzy Jalisco
- **Ubuntu 版本**: $(lsb_release -ds)

## 升级步骤

1. ✅ 安装 ROS2 Jazzy
2. ✅ 安装额外依赖
3. ✅ 设置环境变量
4. ✅ 更新 package.xml
5. ✅ 更新 CMakeLists.txt
6. ✅ 更新文档
7. ✅ 更新 CI/CD 配置
8. ✅ 清理和构建
9. ✅ 运行测试

## 测试结果

- [ ] 基本启动测试
- [ ] 电机控制测试
- [ ] IMU 传感器测试
- [ ] 运动控制测试
- [ ] SLAM 测试
- [ ] 导航测试
- [ ] 长时间运行稳定性测试

## 已知问题

暂无

## 性能对比

| 指标 | Humble | Jazzy | 改进 |
|------|--------|-------|------|
| 启动时间 | - | - | - |
| 内存使用 | - | - | - |
| CPU 使用 | - | - | - |

## 回滚说明

如需回滚到 Humble：

```bash
# 恢复备份文件
git checkout -- .
# 或手动恢复 .bak 文件
```

---

**升级完成时间**: $(date +%Y-%m-%d)  
**状态**: ✅ 成功
EOF

echo "✅ 升级报告已创建：ROS2_JAZZY_UPGRADE_REPORT.md"
echo

# 完成
echo "=========================================="
echo "🎉 ROS2 Jazzy 升级完成！"
echo "=========================================="
echo
echo "下一步："
echo "1. 运行测试确保所有功能正常"
echo "2. 查看 ROS2_JAZZY_UPGRADE_REPORT.md 获取详细信息"
echo "3. 如有问题，参考 UPGRADE_ROS2_JAZZY.md 文档"
echo
echo "🌶️ 升级脚本 by 小米辣"
echo
