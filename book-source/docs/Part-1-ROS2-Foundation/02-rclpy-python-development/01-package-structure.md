---
sidebar_position: 1
title: "Lesson 1: ROS 2 Python Package Structure"
description: "Learn to create and structure ROS 2 Python packages manually, understanding the purpose of each component."
---

# Lesson 1: ROS 2 Python Package Structure

When you look at a professional ROS 2 Python project, you'll notice a specific organization: directories, configuration files, and entry points. This lesson teaches you to build that structure by hand—understanding each piece as you create it.

## Understanding ROS 2 Packages

A **package** is the fundamental unit of code organization in ROS 2. It's not just a Python module. It's a container that tells ROS 2 how to find, build, and execute your code.

Think of it this way: If Python modules are individual books, a ROS 2 package is an organized library with a catalog (metadata), shelving (directory structure), and a checkout system (entry points that tell ROS what programs to run).

## The Package Structure

Here's what a minimal ROS 2 Python package looks like:

```
my_package/
├── my_package/
│   ├── __init__.py
│   └── my_node.py
├── package.xml
├── setup.py
└── setup.cfg
```

Each file serves a specific purpose in how ROS 2 builds and executes your code.

### package.xml: Package Metadata

This XML file tells ROS 2 about your package—what it's called, who maintains it, and what it depends on.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>My first ROS 2 Python package</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
</package>
```

**Key fields:**
- **name**: Package identifier (must match directory name)
- **version**: Semantic versioning for your package
- **description**: One-line summary of what this package does
- **maintainer**: Who keeps this working
- **buildtool_depend**: How to build the package (`ament_cmake_python` for Python packages)
- **depend**: What other packages this depends on (rclpy, message types, etc.)

The `<depend>` tags are critical—they tell colcon (the ROS 2 build system) what to find before building your code.

### setup.py: Python Build Configuration

This tells the Python build system how to package your code and which functions are executable nodes.

```python
from setuptools import setup, find_packages

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='My first ROS 2 Python package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'my_node=my_package.my_node:main',
        ],
    },
)
```

**Critical element: entry_points**

The `entry_points` section is how ROS 2 knows which Python functions to run as executable nodes.

```python
'console_scripts': [
    'my_node=my_package.my_node:main',
]
```

This means: "When someone runs `ros2 run my_package my_node`, execute the `main()` function from `my_package/my_node.py`."

### setup.cfg: Additional Build Configuration

A minimal setup.cfg file:

```ini
[develop]
script_dir=$base/lib/my_package

[install]
install_scripts=$base/lib/my_package
```

This tells the build system where to install executables. For most packages, this is boilerplate—you can copy it as-is.

### __init__.py: Python Package Marker

Python needs a file named `__init__.py` in each directory to recognize it as a package:

```python
# my_package/__init__.py
```

This can be empty. Its presence tells Python: "This directory is a package."

## Manual Package Creation Exercise

Let's create a real package by hand, understanding each step.

**Exercise 1.1a: Create Package Directories and Files**

Create this structure:

```bash
# Create the root package directory
mkdir -p my_package/my_package

# Create an empty Python node file
touch my_package/my_package/__init__.py
touch my_package/my_package/my_node.py
```

Your directory structure should now be:
```
my_package/
└── my_package/
    ├── __init__.py
    └── my_node.py
```

**Success criteria:**
- [ ] `my_package/` directory exists
- [ ] `my_package/my_package/` subdirectory exists
- [ ] Both `__init__.py` and `my_node.py` files exist

---

**Exercise 1.1b: Create package.xml**

In the root `my_package/` directory, create `package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>My first ROS 2 Python package</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
</package>
```

**Success criteria:**
- [ ] File is named `package.xml` (in root of my_package/)
- [ ] XML is valid (proper opening/closing tags)
- [ ] All required fields present: name, version, description, maintainer, license

---

**Exercise 1.1c: Create setup.py**

In the root `my_package/` directory, create `setup.py`:

```python
from setuptools import setup, find_packages

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='My first ROS 2 Python package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'my_node=my_package.my_node:main',
        ],
    },
)
```

**Success criteria:**
- [ ] File is named `setup.py` (in root of my_package/)
- [ ] Python syntax is valid
- [ ] `entry_points` section includes `my_node` entry
- [ ] Entry point references `my_package.my_node:main`

---

**Exercise 1.1d: Create setup.cfg**

In the root `my_package/` directory, create `setup.cfg`:

```ini
[develop]
script_dir=$base/lib/my_package

[install]
install_scripts=$base/lib/my_package
```

**Success criteria:**
- [ ] File is named `setup.cfg` (in root of my_package/)
- [ ] Two sections present: [develop] and [install]

---

**Exercise 1.1e: Create a Minimal Node**

In `my_package/my_package/my_node.py`, create a minimal ROS 2 node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('My node started!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Success criteria:**
- [ ] File is in correct location: `my_package/my_package/my_node.py`
- [ ] Imports are present: `rclpy`, `Node`
- [ ] Class `MyNode` extends `Node`
- [ ] `main()` function exists and calls `rclpy.init()`, `rclpy.spin()`, `rclpy.shutdown()`

---

**Exercise 1.1f: Verify Package Structure**

Use the `tree` command (or `ls -R` if tree isn't available) to verify your structure:

```bash
cd my_package
tree .
```

Expected output:
```
.
├── my_package
│   ├── __init__.py
│   └── my_node.py
├── package.xml
├── setup.py
└── setup.cfg
```

**Success criteria:**
- [ ] All files and directories present
- [ ] Directory structure matches the expected layout
- [ ] No extra files or incorrect nesting

---

**Exercise 1.1g: Build the Package**

Now that you've created the package structure manually, let's build it with colcon:

```bash
cd ~/ros2_ws  # or your ROS 2 workspace root
colcon build --packages-select my_package
```

Expected output (final lines):
```
Starting >>> my_package
Finished <<< my_package [0.50s]

Summary: 1 package finished [0.75s]
```

**Success criteria:**
- [ ] Build completes without errors
- [ ] "Finished" message appears for my_package
- [ ] Build summary shows 1 package built

If you see errors:
- **"Could not find package.xml"** → Verify package.xml is in the correct directory (root of my_package/)
- **"No module named setup"** → Verify setup.py is in the root directory
- **"Entry point 'main' not found"** → Verify my_node.py contains a `main()` function

---

## Understanding Dependencies

The `<depend>` tags in package.xml are crucial. They tell ROS 2 what packages your code needs.

**Common ROS 2 dependencies:**

```xml
<depend>rclpy</depend>           <!-- ROS 2 Python client library -->
<depend>std_msgs</depend>        <!-- Standard message types (Int32, String, etc.) -->
<depend>geometry_msgs</depend>   <!-- Geometry types (Point, Quaternion, etc.) -->
<depend>sensor_msgs</depend>     <!-- Sensor data types (LaserScan, Image, etc.) -->
```

When you use `from std_msgs.msg import String`, you're using classes defined in the `std_msgs` package. That's why it must be listed as a dependency.

**Exercise 1.1h: Add Dependencies**

Open `package.xml` and examine the dependencies. Answer these questions:

- [ ] What does rclpy do? (Hint: what do you import in my_node.py?)
- [ ] What would `std_msgs` be used for?
- [ ] If you wanted to use laser scanner data, what dependency might you need?

---

## How Colcon Uses Package Structure

When you run `colcon build`, it:

1. **Finds** all packages (looks for package.xml)
2. **Reads** metadata (name, version, dependencies)
3. **Orders** packages (builds dependencies first)
4. **Builds** each package using the build tool (ament_cmake_python)
5. **Installs** executables using setup.py entry_points
6. **Updates** the ROS 2 environment so `ros2 run` can find your nodes

The directory structure and configuration files you created manually are exactly what ROS 2 expects.

## Try With AI

**Setup:** Open your terminal and navigate to your ROS 2 workspace.

**Prompt 1: Understand Directory Purpose**

Ask your AI tool: "I have a ROS 2 Python package with these files. Explain the purpose of each file and why it needs to be in that exact location: package.xml, setup.py, setup.cfg, __init__.py."

**Prompt 2: Identify Missing Configuration**

Create a broken package.xml (remove one required field). Ask AI: "This package.xml is causing a build error. What's missing? (Here's the file...)"

**Prompt 3: Dependency Analysis**

Ask AI: "I want to use nav2_msgs in my node. What change do I need to make to my package.xml and why?"

**Expected Outcomes:**

Your AI should help clarify:
- Package.xml defines ROS 2 package metadata
- setup.py tells Python how to build your code
- setup.cfg handles installation paths
- __init__.py marks Python packages
- Dependencies must be declared for ROS 2 to find them

**Self-Reflection:**

Before moving to the next lesson, verify:
- [ ] You created a package structure by hand (not using ros2 pkg create)
- [ ] You understand what each file does
- [ ] You can explain why the structure matters for ROS 2
- [ ] colcon build succeeded on your package
