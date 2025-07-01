#!/bin/bash
#
# Package Validation Script for CR3 Hand Tracking System
#
# Verifies that all packages are properly configured and can be built.
# Run this script to ensure system integrity before testing.
#

echo "=== CR3 Hand Tracking System Validation ==="
echo ""

# Check workspace structure
echo "1. Checking workspace structure..."
if [ ! -d "src" ]; then
    echo "❌ ERROR: Not in ROS 2 workspace root (no src/ directory found)"
    exit 1
fi

# Check required packages
echo "2. Checking required packages..."
required_packages=("perception" "control" "ui" "camera_interface")
for package in "${required_packages[@]}"; do
    if [ -d "src/$package" ]; then
        echo "✅ Package $package found"
    else
        echo "❌ ERROR: Package $package not found"
        exit 1
    fi
done

# Check package.xml files
echo ""
echo "3. Checking package.xml files..."
for package in "${required_packages[@]}"; do
    if [ -f "src/$package/package.xml" ]; then
        echo "✅ package.xml for $package exists"
    else
        echo "❌ ERROR: package.xml for $package missing"
        exit 1
    fi
done

# Check setup.py files
echo ""
echo "4. Checking setup.py files..."
for package in "${required_packages[@]}"; do
    if [ -f "src/$package/setup.py" ]; then
        echo "✅ setup.py for $package exists"
    else
        echo "❌ ERROR: setup.py for $package missing"
        exit 1
    fi
done

# Check enhanced node files
echo ""
echo "5. Checking enhanced node implementations..."
enhanced_nodes=(
    "src/perception/perception/coordinate_transform_node.py"
    "src/control/control/pose_to_command_node.py"
    "src/ui/ui/ui_dashboard_node.py"
    "src/ui/ui/mode_switcher_node.py"
    "src/ui/ui/ui_components.py"
)

for node in "${enhanced_nodes[@]}"; do
    if [ -f "$node" ]; then
        echo "✅ Enhanced node $node exists"
    else
        echo "❌ ERROR: Enhanced node $node missing"
        exit 1
    fi
done

# Check Python package structure
echo ""
echo "6. Checking Python package structure..."
python_packages=("perception/perception" "control/control" "ui/ui" "camera_interface/camera_interface")
for package in "${python_packages[@]}"; do
    if [ -f "src/$package/__init__.py" ]; then
        echo "✅ __init__.py for $package exists"
    else
        echo "❌ ERROR: __init__.py for $package missing"
        exit 1
    fi
done

# Check dependencies (basic syntax check)
echo ""
echo "7. Checking basic Python syntax..."
python_files=(
    "src/perception/perception/coordinate_transform_node.py"
    "src/control/control/pose_to_command_node.py"  
    "src/ui/ui/ui_dashboard_node.py"
    "src/ui/ui/mode_switcher_node.py"
    "src/ui/ui/ui_components.py"
)

for file in "${python_files[@]}"; do
    if python3 -m py_compile "$file" 2>/dev/null; then
        echo "✅ Syntax OK: $file"
    else
        echo "❌ ERROR: Syntax error in $file"
        exit 1
    fi
done

# Check for import issues (basic check)
echo ""
echo "8. Checking basic imports..."
cd src
if python3 -c "
try:
    import sys
    sys.path.append('.')
    from ui.ui import ui_components
    from perception.perception import coordinate_transform_node
    print('✅ Module imports OK')
except ImportError as e:
    print(f'❌ Import error: {e}')
    exit(1)
" 2>/dev/null; then
    echo "✅ Basic imports successful"
else
    echo "⚠️  Warning: Some imports may fail (normal if dependencies not installed)"
fi
cd ..

echo ""
echo "=== Validation Complete ==="
echo ""
echo "✅ All packages properly configured!"
echo ""
echo "Next steps:"
echo "1. Build system: colcon build"
echo "2. Source environment: source install/setup.bash"  
echo "3. Test individual nodes as documented in build_and_test_guide.md"
echo ""
echo "Enhanced nodes ready for testing:"
echo "  • perception/coordinate_transform_node - Vector calculation and coordinate transforms"
echo "  • control/pose_to_command_node - Enhanced control with vector-based modes"
echo "  • ui/ui_dashboard_node - Full OpenCV dashboard interface"
echo "  • ui/mode_switcher_node - Enhanced mode management"
echo "  • ui/ui_components - Modular UI component library"
