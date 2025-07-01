# ROS 2 Python Package Quick Reference Card

## 🔥 CRITICAL: The 4-Line File That Makes Everything Work

**File:** `src/your_package/setup.cfg`
```ini
[develop]
script_dir = src/your_package
[install]
install_scripts = $base/lib/your_package
```
**Without this file, launch files will fail to find your executables!**

---

## 📁 Required Directory Structure
```
src/your_package/
├── your_package/           # Python module (SAME NAME as package!)
│   ├── __init__.py        # Required
│   └── your_node.py       # Node with main() function
├── resource/
│   └── your_package       # Empty file (SAME NAME!)
├── package.xml            # ROS 2 manifest
├── setup.py               # Python setup
└── setup.cfg              # 🔥 CRITICAL installation config
```

---

## ⚡ Quick Setup Commands
```bash
# 1. Create structure
mkdir -p src/my_package/my_package src/my_package/resource
touch src/my_package/my_package/__init__.py
touch src/my_package/resource/my_package

# 2. Copy templates (customize names!)
# setup.cfg, setup.py, package.xml, node files

# 3. Build (NEVER use --symlink-install!)
rm -rf build/ install/ log/
source /opt/ros/jazzy/setup.bash
colcon build

# 4. Test
source install/setup.bash
ros2 run my_package my_node
```

---

## 🎯 setup.py Entry Points Template
```python
entry_points={
    'console_scripts': [
        'node1 = my_package.node1:main',
        'node2 = my_package.node2:main',
        # Add all your nodes here
    ],
},
```

---

## 🚨 Common Mistakes to Avoid
- ❌ Forgetting setup.cfg file
- ❌ Using `colcon build --symlink-install`
- ❌ Mismatched package names
- ❌ Missing main() functions in nodes
- ❌ Copying .py files via data_files

---

## ✅ Success Verification
```bash
# Check executable location
ls install/my_package/lib/my_package/
# Should show executables WITHOUT .py extensions

# Test commands
ros2 run my_package my_node
ros2 launch my_package my_launch.launch.py
```

---

## 🛠️ Troubleshooting
| Error | Solution |
|-------|----------|
| "executable not found" | Add setup.cfg file |
| "No executable found" | Fix entry_points in setup.py |
| "Package not found" | Source install/setup.bash |
| ".py extensions on executables" | Remove data_files, use entry_points |

---

## 📋 Pre-Flight Checklist
- [ ] setup.cfg exists with install_scripts directive
- [ ] Package names match in all files
- [ ] All nodes have main() functions
- [ ] entry_points list all nodes
- [ ] Built with `colcon build` (no flags)
- [ ] Sourced install/setup.bash

**Print this card and keep it handy! 🖨️**
